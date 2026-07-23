#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <atomic>
#include <mutex>
#include <condition_variable>

using namespace std;
using namespace Eigen;

// ─── DH Parameters (UR10) ────────────────────────────────────────────────────
const double d1_val = 0.1273;
const double a2_val = -0.612;
const double a3_val = -0.5723;
const double d4_val = 0.1639;
const double d5_val = 0.1157;
const double d6_val = 0.0922;

const double a_dh[6] = {0, a2_val, a3_val, 0, 0, 0};
const double d_dh[6] = {d1_val, 0, 0, d4_val, d5_val, d6_val};
const double alph[6] = {M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0};

// ─── Configurable offsets ─────────────────────────────────────────────────────
// XYZ offset is added on top of the position received from
// /circle_detector/socket_pose_tool0 to get the final IK target.
// RPY is now fixed (orientation from socket_pose_tool0 is not used/trusted —
// its quaternion is not mathematically valid, see circle_detector_node).
static constexpr double OFFSET_X     = 0;
static constexpr double OFFSET_Y     = 0;
static constexpr double OFFSET_Z     = 0;
static constexpr double FIXED_ROLL   = M_PI;
static constexpr double FIXED_PITCH  = 0.0;
static constexpr double FIXED_YAW    = (3.0 * M_PI) / 2.0;

// Phase 1 (approach) / Phase 3 (retreat) back off this much along X from
// the captured socket X, before/after Phase 2 (plug in) goes to full X.
static constexpr double APPROACH_OFFSET_X = -0.25;

static constexpr int ARM_WAIT_S = 4;

// ─── IK helpers ──────────────────────────────────────────────────────────────
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Matrix4d ah(int n, const Matrix<double, 6, 8>& th, int c) {
    Matrix4d T_a = Matrix4d::Identity(); T_a(0, 3) = a_dh[n-1];
    Matrix4d T_d = Matrix4d::Identity(); T_d(2, 3) = d_dh[n-1];

    Matrix4d Rzt = Matrix4d::Zero();
    Rzt(0,0) = cos(th(n-1,c)); Rzt(0,1) = -sin(th(n-1,c));
    Rzt(1,0) = sin(th(n-1,c)); Rzt(1,1) =  cos(th(n-1,c));
    Rzt(2,2) = 1.0; Rzt(3,3) = 1.0;

    Matrix4d Rxa = Matrix4d::Zero();
    Rxa(0,0) = 1.0;
    Rxa(1,1) = cos(alph[n-1]); Rxa(1,2) = -sin(alph[n-1]);
    Rxa(2,1) = sin(alph[n-1]); Rxa(2,2) =  cos(alph[n-1]);
    Rxa(3,3) = 1.0;

    return T_d * Rzt * T_a * Rxa;
}

Matrix4d calculate_fk(const vector<double>& q) {
    Matrix4d T = Matrix4d::Identity();
    for (int i = 0; i < 6; ++i) {
        double ct = cos(q[i]), st = sin(q[i]);
        double ca = cos(alph[i]), sa = sin(alph[i]);
        Matrix4d A;
        A << ct, -st*ca,  st*sa, a_dh[i]*ct,
             st,  ct*ca, -ct*sa, a_dh[i]*st,
              0,     sa,     ca,     d_dh[i],
              0,      0,      0,           1;
        T = T * A;
    }
    return T;
}

Matrix4d get_t_matrix(double x, double y, double z, double r, double p, double yaw) {
    Matrix4d Rx, Ry, Rz, T;
    Rx << 1,      0,       0, 0,
          0, cos(r), -sin(r), 0,
          0, sin(r),  cos(r), 0,
          0,      0,       0, 1;
    Ry << cos(p), 0, sin(p), 0,
               0,  1,      0, 0,
         -sin(p),  0, cos(p), 0,
               0,  0,      0, 1;
    Rz << cos(yaw), -sin(yaw), 0, 0,
          sin(yaw),  cos(yaw), 0, 0,
                 0,         0, 1, 0,
                 0,         0, 0, 1;
    T = Matrix4d::Identity();
    T(0,3) = x; T(1,3) = y; T(2,3) = z;
    return T * Rz * Ry * Rx;
}

vector<vector<double>> inverse_kinematics(const Matrix4d& T_06) {
    Matrix<double, 6, 8> th = Matrix<double, 6, 8>::Zero();

    Vector4d P_05 = T_06 * Vector4d(0, 0, -d6_val, 1);
    double psi = atan2(P_05(1), P_05(0));
    double val_phi = d4_val / sqrt(P_05(0)*P_05(0) + P_05(1)*P_05(1));
    if (std::abs(val_phi) > 1) return {};
    double phi = acos(val_phi);

    for (int i = 0; i < 4; ++i) th(0, i) = M_PI/2 + psi + phi;
    for (int i = 4; i < 8; ++i) th(0, i) = M_PI/2 + psi - phi;

    int cl_5[] = {0, 4};
    for (int c : cl_5) {
        Matrix4d T_16 = ah(1, th, c).inverse() * T_06;
        double val = (T_16(2, 3) - d4_val) / d6_val;
        if (std::abs(val) > 1) val = sgn(val);
        th(4, c) = th(4, c+1) = -acos(val);
        th(4, c+2) = th(4, c+3) = acos(val);
    }

    int cl_6[] = {0, 2, 4, 6};
    for (int c : cl_6) {
        Matrix4d T_16 = (ah(1, th, c).inverse() * T_06).inverse();
        if (std::abs(sin(th(4, c))) < 1e-9) continue;
        th(5, c) = th(5, c+1) = atan2(
            -T_16(1,2) / sin(th(4,c)),
             T_16(0,2) / sin(th(4,c)));
    }

    for (int c : cl_6) {
        Matrix4d T_14 = (ah(1,th,c).inverse() * T_06) * (ah(5,th,c) * ah(6,th,c)).inverse();
        Vector4d P_13 = T_14 * Vector4d(0, -d4_val, 0, 1) - Vector4d(0,0,0,1);
        double costh3 = (P_13(0)*P_13(0) + P_13(1)*P_13(1) - a2_val*a2_val - a3_val*a3_val)
                        / (2 * a2_val * a3_val);
        if (std::abs(costh3) > 1) costh3 = sgn(costh3);
        th(2, c)   =  acos(costh3);
        th(2, c+1) = -acos(costh3);
    }

    for (int c = 0; c < 8; ++c) {
        Matrix4d T_14 = (ah(1,th,c).inverse() * T_06) * ah(6,th,c).inverse() * ah(5,th,c).inverse();
        Vector4d P_13 = T_14 * Vector4d(0, -d4_val, 0, 1) - Vector4d(0,0,0,1);
        double val_asin = a3_val * sin(th(2,c)) / sqrt(P_13(0)*P_13(0) + P_13(1)*P_13(1));
        if (std::abs(val_asin) > 1) val_asin = sgn(val_asin);
        th(1, c) = -atan2(P_13(1), -P_13(0)) + asin(val_asin);
        Matrix4d T_34 = ah(3,th,c).inverse() * ah(2,th,c).inverse() * T_14;
        th(3, c) = atan2(T_34(1,0), T_34(0,0));
    }

    vector<vector<double>> solutions;
    for (int i = 0; i < 8; ++i) {
        vector<double> sol(6);
        for (int j = 0; j < 6; ++j) sol[j] = th(j, i);
        solutions.push_back(sol);
    }
    return solutions;
}

vector<double> select_best_solution(const vector<vector<double>>& solutions,
                                    double tx, double ty, double tz) {
    vector<vector<double>> valid;
    for (const auto& sol : solutions) {
        Matrix4d T = calculate_fk(sol);
        double err = sqrt(pow(tx-T(0,3),2) + pow(ty-T(1,3),2) + pow(tz-T(2,3),2));
        if (err < 0.005) valid.push_back(sol);
    }
    if (valid.empty()) return {};

    vector<double> best;
    double min_err = 1e9;
    for (const auto& sol : valid) {
        double sum_deg = (sol[1] + sol[2] + sol[3]) * 180.0 / M_PI;
        double a = sum_deg - (-270.0) + 180.0;
        double err = std::abs(fmod(fmod(a, 360.0) + 360.0, 360.0) - 180.0);
        if (err < min_err) { min_err = err; best = sol; }
    }
    return best;
}

// ─── Node ────────────────────────────────────────────────────────────────────
class ArmTeleopNode : public rclcpp::Node {
public:
    ArmTeleopNode()
    : Node("arm_ik_teleop_node_cpp"),
      pose_captured_(false),
      waiting_for_pose_(false)
    {
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        joint_names_ = {
            "ur10_shoulder_pan_joint", "ur10_shoulder_lift_joint",
            "ur10_elbow_joint",        "ur10_wrist_1_joint",
            "ur10_wrist_2_joint",      "ur10_wrist_3_joint"
        };

        // Subscribe directly to the tool0-remapped socket pose. No TF lookup
        // needed anymore — we only use its xyz.
        socket_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/circle_detector/socket_pose_tool0", 10,
            std::bind(&ArmTeleopNode::socket_callback, this, std::placeholders::_1));

        input_thread_ = std::thread(&ArmTeleopNode::input_loop, this);

        RCLCPP_INFO(this->get_logger(), "Sẵn sàng. Gõ 'ok' để bắt đầu.");
    }

    ~ArmTeleopNode() {
        if (input_thread_.joinable()) input_thread_.join();
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    socket_sub_;
    vector<string> joint_names_;
    std::thread    input_thread_;

    std::mutex              pose_mutex_;
    std::condition_variable pose_cv_;
    // Captured values (xyz from socket_pose_tool0 + fixed offset; rpy fixed)
    double captured_x_,   captured_y_,   captured_z_;
    double captured_roll_, captured_pitch_, captured_yaw_;
    bool   pose_captured_;
    bool   waiting_for_pose_;

    // ── Callback: capture xyz from socket_pose_tool0, rpy stays fixed ────────
    void socket_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!waiting_for_pose_ || pose_captured_) return;

        // xyz straight from the topic + fixed offset. No TF transform.
        captured_x_ = msg->pose.position.x + OFFSET_X;
        captured_y_ = msg->pose.position.y + OFFSET_Y;
        captured_z_ = msg->pose.position.z + OFFSET_Z;

        // rpy stays fixed — orientation from socket_pose_tool0 is not used.
        captured_roll_  = FIXED_ROLL;
        captured_pitch_ = FIXED_PITCH;
        captured_yaw_   = FIXED_YAW;

        RCLCPP_INFO(this->get_logger(),
            "Socket xyz from socket_pose_tool0 (+ offset):\n"
            "  X=%.4f  Y=%.4f  Z=%.4f\n"
            "  Roll=%.4f  Pitch=%.4f  Yaw=%.4f (fixed)",
            captured_x_, captured_y_, captured_z_,
            captured_roll_, captured_pitch_, captured_yaw_);

        pose_captured_ = true;
        pose_cv_.notify_one();
    }

    // ── Send IK command (6-DOF) ───────────────────────────────────────────────
    bool move_arm(double x, double y, double z,
                  double roll, double pitch, double yaw) {
        RCLCPP_INFO(this->get_logger(),
            "[IK] X=%.4f  Y=%.4f  Z=%.4f  Roll=%.4f  Pitch=%.4f  Yaw=%.4f",
            x, y, z, roll, pitch, yaw);

        Matrix4d T_06 = get_t_matrix(x, y, z, roll, pitch, yaw);
        auto solutions = inverse_kinematics(T_06);
        if (solutions.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Ngoài vùng làm việc!");
            return false;
        }

        auto joints = select_best_solution(solutions, x, y, z);
        if (joints.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Không tìm được nghiệm hợp lệ!");
            return false;
        }

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions = joints;
        pt.time_from_start.sec = 3;
        traj.points.push_back(pt);
        traj_pub_->publish(traj);

        RCLCPP_INFO(this->get_logger(), "Đã gửi lệnh tay, chờ %ds...", ARM_WAIT_S);
        std::this_thread::sleep_for(std::chrono::seconds(ARM_WAIT_S));
        return true;
    }

    // ── Input loop: wait for "ok" from terminal ───────────────────────────────
    void input_loop() {
        while (rclcpp::ok()) {
            cout << "\nGõ 'ok' để bắt đầu: ";
            string line;
            if (!getline(cin, line)) break;
            if (line != "ok") continue;

            // Step 0: capture xyz from socket_pose_tool0 (once)
            RCLCPP_INFO(this->get_logger(), "Chờ lấy pose từ topic...");
            {
                std::unique_lock<std::mutex> lock(pose_mutex_);
                pose_captured_    = false;
                waiting_for_pose_ = true;
                bool got = pose_cv_.wait_for(lock, std::chrono::seconds(5),
                                             [this]{ return pose_captured_; });
                waiting_for_pose_ = false;
                if (!got) {
                    RCLCPP_ERROR(this->get_logger(),
                        "Timeout! Không nhận được dữ liệu từ topic trong 5s.");
                    continue;
                }
            }

            const double x     = captured_x_;
            const double y     = captured_y_;
            const double z     = captured_z_;
            const double roll  = captured_roll_;
            const double pitch = captured_pitch_;
            const double yaw   = captured_yaw_;

            const double approach_x = x + APPROACH_OFFSET_X;

            // Phase 1: approach — go to (x + APPROACH_OFFSET_X, y, z)
            RCLCPP_INFO(this->get_logger(),
                "=== Giai đoạn 1: Tiến gần X=%.4f Y=%.4f Z=%.4f ===",
                approach_x, y, z);
            if (!move_arm(approach_x, y, z, roll, pitch, yaw)) continue;

            // Phase 2: plug in — go to full (x, y, z)
            RCLCPP_INFO(this->get_logger(),
                "=== Giai đoạn 2: Cắm X=%.4f Y=%.4f Z=%.4f ===",
                x, y, z);
            if (!move_arm(x, y, z, roll, pitch, yaw)) continue;

            // Phase 3: retreat — back to phase 1 position
            RCLCPP_INFO(this->get_logger(),
                "=== Giai đoạn 3: Lùi về X=%.4f Y=%.4f Z=%.4f ===",
                approach_x, y, z);
            move_arm(approach_x, y, z, roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(), "=== Hoàn thành ===");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmTeleopNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
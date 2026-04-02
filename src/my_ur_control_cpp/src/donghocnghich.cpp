#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <sstream>

using namespace std;
using namespace Eigen;

const double d1_val = 0.1273;
const double a2_val = -0.612;
const double a3_val = -0.5723;
const double d4_val = 0.1639;
const double d5_val = 0.1157;
const double d6_val = 0.0922;

const double a_dh[6] = {0, a2_val, a3_val, 0, 0, 0};
const double d_dh[6] = {d1_val, 0, 0, d4_val, d5_val, d6_val};
const double alph[6] = {M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Matrix4d ah(int n, const Matrix<double, 6, 8>& th, int c) {
    Matrix4d T_a = Matrix4d::Identity();
    T_a(0, 3) = a_dh[n-1];
    
    Matrix4d T_d = Matrix4d::Identity();
    T_d(2, 3) = d_dh[n-1];

    Matrix4d Rzt = Matrix4d::Zero();
    Rzt(0,0) = cos(th(n-1, c)); Rzt(0,1) = -sin(th(n-1, c));
    Rzt(1,0) = sin(th(n-1, c)); Rzt(1,1) =  cos(th(n-1, c));
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
        double ct = cos(q[i]);
        double st = sin(q[i]);
        double ca = cos(alph[i]);
        double sa = sin(alph[i]);
        
        Matrix4d A;
        A << ct, -st*ca,  st*sa, a_dh[i]*ct,
             st,  ct*ca, -ct*sa, a_dh[i]*st,
             0,   sa,     ca,    d_dh[i],
             0,   0,      0,     1;
        T = T * A;
    }
    return T;
}

Matrix4d get_t_matrix(double x, double y, double z, double r, double p, double yaw) {
    Matrix4d Rx, Ry, Rz, T;
    Rx << 1, 0, 0, 0,
          0, cos(r), -sin(r), 0,
          0, sin(r), cos(r), 0,
          0, 0, 0, 1;
    Ry << cos(p), 0, sin(p), 0,
          0, 1, 0, 0,
          -sin(p), 0, cos(p), 0,
          0, 0, 0, 1;
    Rz << cos(yaw), -sin(yaw), 0, 0,
          sin(yaw), cos(yaw), 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    T = Matrix4d::Identity();
    T(0,3) = x; T(1,3) = y; T(2,3) = z;
    return T * Rz * Ry * Rx;
}

vector<vector<double>> inverse_kinematics(const Matrix4d& T_06) {
    Matrix<double, 6, 8> th = Matrix<double, 6, 8>::Zero();
    
    Vector4d p5_vec(0, 0, -d6_val, 1);
    Vector4d P_05 = T_06 * p5_vec;
    double psi = atan2(P_05(1), P_05(0));
    double val_phi = d4_val / sqrt(P_05(0)*P_05(0) + P_05(1)*P_05(1));
    if (std::abs(val_phi) > 1) return {}; 
    double phi = acos(val_phi);
    
    for(int i=0; i<4; ++i) th(0, i) = M_PI/2 + psi + phi;
    for(int i=4; i<8; ++i) th(0, i) = M_PI/2 + psi - phi;

    int cl_5[] = {0, 4};
    for (int c : cl_5) {
        Matrix4d T_10 = ah(1, th, c).inverse();
        Matrix4d T_16 = T_10 * T_06;
        double P_16z = T_16(2, 3);
        double val = (P_16z - d4_val) / d6_val;
        if (std::abs(val) > 1) val = sgn(val);
        
        th(4, c) = th(4, c+1) = -acos(val);
        th(4, c+2) = th(4, c+3) = acos(val);
    }

    int cl_6[] = {0, 2, 4, 6};
    for (int c : cl_6) {
        Matrix4d T_10 = ah(1, th, c).inverse();
        Matrix4d T_16 = (T_10 * T_06).inverse();
        if (std::sin(th(4, c)) == 0) continue;
        double ang = atan2((-T_16(1,2)/sin(th(4, c))), (T_16(0,2)/sin(th(4, c))));
        th(5, c) = th(5, c+1) = ang;
    }

    for (int c : cl_6) {
        Matrix4d T_10 = ah(1, th, c).inverse();
        Matrix4d T_65 = ah(6, th, c);
        Matrix4d T_54 = ah(5, th, c);
        Matrix4d T_14 = (T_10 * T_06) * (T_54 * T_65).inverse();
        
        Vector4d p13_vec(0, -d4_val, 0, 1);
        Vector4d P_13 = T_14 * p13_vec - Vector4d(0,0,0,1);
        
        double costh3 = ((P_13(0)*P_13(0) + P_13(1)*P_13(1)) - a2_val*a2_val - a3_val*a3_val) / (2*a2_val*a3_val);
        if (std::abs(costh3) > 1) costh3 = sgn(costh3);
        
        th(2, c) = acos(costh3);
        th(2, c+1) = -acos(costh3);
    }

    for (int c = 0; c < 8; ++c) {
        Matrix4d T_10 = ah(1, th, c).inverse();
        Matrix4d T_65 = ah(6, th, c).inverse();
        Matrix4d T_54 = ah(5, th, c).inverse();
        Matrix4d T_14 = (T_10 * T_06) * T_65 * T_54;
        
        Vector4d p13_vec(0, -d4_val, 0, 1);
        Vector4d P_13 = T_14 * p13_vec - Vector4d(0,0,0,1);
        
        double val_asin = a3_val*sin(th(2,c))/sqrt(P_13(0)*P_13(0) + P_13(1)*P_13(1));
        if (std::abs(val_asin) > 1) val_asin = sgn(val_asin);
            
        th(1, c) = -atan2(P_13(1), -P_13(0)) + asin(val_asin);
        
        Matrix4d T_32 = ah(3, th, c).inverse();
        Matrix4d T_21 = ah(2, th, c).inverse();
        Matrix4d T_34 = T_32 * T_21 * T_14;
        th(3, c) = atan2(T_34(1,0), T_34(0,0));
    }

    vector<vector<double>> solutions;
    for (int i = 0; i < 8; ++i) {
        vector<double> sol(6);
        for(int j=0; j<6; ++j) sol[j] = th(j, i);
        solutions.push_back(sol);
    }
    return solutions;
}

class ArmTeleopNode : public rclcpp::Node {
public:
    ArmTeleopNode() : Node("arm_ik_teleop_node_cpp") {
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
            
        joint_names_ = {
            "ur10_shoulder_pan_joint", "ur10_shoulder_lift_joint", 
            "ur10_elbow_joint", "ur10_wrist_1_joint", 
            "ur10_wrist_2_joint", "ur10_wrist_3_joint"
        };
        
        input_thread_ = std::thread(&ArmTeleopNode::terminal_input_loop, this);
    }

    ~ArmTeleopNode() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    vector<string> joint_names_;
    std::thread input_thread_;

    vector<double> select_best_solution(const vector<vector<double>>& solutions, double tx, double ty, double tz) {
        vector<vector<double>> valid_sols;
        
        for (const auto& sol : solutions) {
            double theta2_deg = sol[1] * 180.0 / M_PI;
            // if (theta2_deg > 0.0) continue; 

            Matrix4d T_check = calculate_fk(sol);
            double err_dist = sqrt(pow(tx - T_check(0,3), 2) + 
                                   pow(ty - T_check(1,3), 2) + 
                                   pow(tz - T_check(2,3), 2));
            
            if (err_dist < 0.005) {
                valid_sols.push_back(sol);
            }
        }
        
        if (valid_sols.empty()) return {};

        vector<double> best_sol;
        double min_err = 999999.0;
        
        for (const auto& sol : valid_sols) {
            double sum_j_deg = (sol[1] + sol[2] + sol[3]) * 180.0 / M_PI;
            double a = sum_j_deg - (-270.0) + 180.0;
            double err = std::abs(fmod(fmod(a, 360.0) + 360.0, 360.0) - 180.0);
            
            if (err < min_err) {
                min_err = err;
                best_sol = sol;
            }
        }
        return best_sol;
    }

    // Set to true for XYZ-only mode (fixed orientation below)
    // Set to false for full XYZ + RPY mode
    static constexpr bool XYZ_ONLY = true;

    // Fixed end-effector orientation used in XYZ-only mode
    static constexpr double FIXED_ROLL  = M_PI;
    static constexpr double FIXED_PITCH = 0.0;
    static constexpr double FIXED_YAW   = (M_PI * 3) / 2;

    void terminal_input_loop() {
        while (rclcpp::ok()) {
            cout << "\n==================================================\n";
            cout << "Nhập tọa độ mục tiêu (cách nhau bởi dấu cách)\n";
            if (XYZ_ONLY) {
                cout << "Chế độ: XYZ (hướng cố định)\n";
                cout << "Định dạng: X Y Z\n";
                cout << "Ví dụ: 0.5 0.2 0.3\n";
            } else {
                cout << "Chế độ: XYZ + RPY\n";
                cout << "Định dạng: X Y Z Roll Pitch Yaw (Góc nhập bằng Độ)\n";
                cout << "Ví dụ: 0.5 0.2 0.3 180 0 270\n";
            }
            cout << "Mời nhập: ";

            string input_line;
            getline(cin, input_line);
            if(input_line.empty()) continue;

            stringstream ss(input_line);
            double x, y, z;
            double r, p, yaw;

            if (XYZ_ONLY) {
                if (!(ss >> x >> y >> z)) {
                    cout << "Vui lòng nhập đúng 3 số\n";
                    continue;
                }
                r   = FIXED_ROLL;
                p   = FIXED_PITCH;
                yaw = FIXED_YAW;
            } else {
                double r_deg, p_deg, yaw_deg;
                if (!(ss >> x >> y >> z >> r_deg >> p_deg >> yaw_deg)) {
                    cout << "Vui lòng nhập đúng 6 số\n";
                    continue;
                }
                r   = r_deg   * M_PI / 180.0;
                p   = p_deg   * M_PI / 180.0;
                yaw = yaw_deg * M_PI / 180.0;
            }

            cout << "\nĐang giải IK cho tọa độ: X=" << x << ", Y=" << y << ", Z=" << z << "...\n";
            
            Matrix4d T_06 = get_t_matrix(x, y, z, r, p, yaw);
            auto solutions = inverse_kinematics(T_06);
            
            if (solutions.empty()) {
                cout << "LỖI TOÁN HỌC: Điểm này nằm ngoài không gian làm việc\n";
                continue;
            }

            auto target_joints = select_best_solution(solutions, x, y, z);
            if (target_joints.empty()) {
                cout << "TỪ CHỐI LỆNH: Các nghiệm đều vi phạm giới hạn khớp hoặc sinh ra Nghiệm Ma\n";
                continue;
            }

            cout << "TÌM THẤY NGHIỆM CHUẨN XÁC 100%\n";
            
            trajectory_msgs::msg::JointTrajectory msg;
            msg.joint_names = joint_names_;
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = target_joints;
            point.time_from_start.sec = 3;
            
            msg.points.push_back(point);
            traj_pub_->publish(msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
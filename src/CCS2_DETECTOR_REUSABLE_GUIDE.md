# CCS2 Socket Detector — Reusable Module Guide

> Hướng dẫn đóng gói và tái sử dụng module phát hiện cổng sạc CCS2 (bằng classic CV + camera depth) vào workspace ROS2 mới.

---

## Mục lục

1. [Tổng quan module](#1-tổng-quan-module)
2. [Cấu trúc thư mục tối thiểu](#2-cấu-trúc-thư-mục-tối-thiểu)
3. [Cách copy sang workspace mới](#3-cách-copy-sang-workspace-mới)
4. [Cấu hình & tùy chỉnh](#4-cấu-hình--tùy-chỉnh)
5. [Kiểm tra & chạy thử](#5-kiểm-tra--chạy-thử)
6. [Troubleshooting thường gặp](#6-troubleshooting-thường-gặp)
7. [Tích hợp với các module khác](#7-tích-hợp-với-các-module-khác)
8. [Nâng cấp lên AI/ML sau này](#8-nâng-cấp-lên-aiml-sau-này)

---

## 1. Tổng quan module

Module CCS2 detector gồm **3 nodes chính**, tất cả đều là classic CV (không cần GPU, không cần training):

| Node | File | Chức năng | Input | Output |
|------|------|-----------|-------|--------|
| `ccs2_detector_node` | `src/ccs2_detector_node.cpp` | Phát hiện socket từ RGB+D, tính centroid, bbox, depth, góc nghiêng | `/base_camera/.../image_raw` + `/base_camera/.../depth/image_raw` | `~/socket_detection` (SocketDetection msg) |
| `depth_heatmap` | `src/depth_heatmap.cpp` | Visualize depth dạng heatmap | `/base_camera/.../depth/image_raw` | `/base_camera/depth_heatmap` |
| `solvepnp` | `src/solvepnp.cpp` | Tính pose 6-DOF từ template matching + PnP | `/base_camera/.../image_raw` | `/object_pose` (PoseStamped) |

### Dependencies (ROS2 packages)

```
rclcpp, std_msgs, geometry_msgs, sensor_msgs, cv_bridge, OpenCV, tf2, tf2_geometry_msgs, message_filters
```

### Dependencies (system)

```
OpenCV (core, imgproc, calib3d, features2d)
Eigen3
```

---

## 2. Cấu trúc thư mục tối thiểu

Để tái sử dụng, bạn chỉ cần **2 packages** sau:

```
your_new_ws/src/
├── ccs2_socket_detector/          # Message definitions (bắt buộc)
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/
│       └── SocketDetection.msg
│
└── my_ur_control_cpp/             # Các node C++ (bắt buộc)
    ├── CMakeLists.txt
    ├── package.xml
    ├── include/
    │   └── ccs2_socket_detector/
    │       └── ccs2_detector_node.hpp
    └── src/
        ├── ccs2_detector_node.cpp   # Node chính
        ├── depth_heatmap.cpp        # (tùy chọn)
        └── solvepnp.cpp             # (tùy chọn)
```

> **Không cần**: `plugin.cpp`, `god_view_2.cpp`, `god_view.py`, `donghocnghich.cpp`, `swerve_drive*.cpp` — đó là module điều khiển xe/tay, không liên quan detect.

---

## 3. Cách copy sang workspace mới

### Bước 1: Copy 2 package

```bash
# Từ workspace cũ
cp -r ~/new_design_ws/src/ccs2_socket_detector ~/your_new_ws/src/
cp -r ~/new_design_ws/src/my_ur_control_cpp ~/your_new_ws/src/

# Hoặc chỉ copy các file cần thiết (nếu muốn gọn)
mkdir -p ~/your_new_ws/src/my_ur_control_cpp/src
mkdir -p ~/your_new_ws/src/my_ur_control_cpp/include/ccs2_socket_detector
cp ~/new_design_ws/src/my_ur_control_cpp/src/ccs2_detector_node.cpp ~/your_new_ws/src/my_ur_control_cpp/src/
cp ~/new_design_ws/src/my_ur_control_cpp/src/depth_heatmap.cpp ~/your_new_ws/src/my_ur_control_cpp/src/
cp ~/new_design_ws/src/my_ur_control_cpp/src/solvepnp.cpp ~/your_new_ws/src/my_ur_control_cpp/src/
cp ~/new_design_ws/src/my_ur_control_cpp/include/ccs2_socket_detector/ccs2_detector_node.hpp ~/your_new_ws/src/my_ur_control_cpp/include/ccs2_socket_detector/
```

### Bước 2: Copy file ảnh template cho solvepnp

```bash
# File template matching (cần để ở thư mục chạy lệnh ros2 run)
cp ~/new_design_ws/src/congsac2.png ~/your_new_ws/
# Hoặc copy vào thư mục config của package
mkdir -p ~/your_new_ws/src/my_ur_control_cpp/config
cp ~/new_design_ws/src/congsac2.png ~/your_new_ws/src/my_ur_control_cpp/config/
# Sau đó sửa đường dẫn trong solvepnp.cpp
```

### Bước 3: Sửa CMakeLists.txt và package.xml

**package.xml** — giữ nguyên dependencies:

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>OpenCV</depend>
<depend>message_filters</depend>
<depend>ccs2_socket_detector</depend>
<depend>tf2</depend>
<depend>tf2_geometry_msgs</depend>
```

**CMakeLists.txt** — chỉ giữ lại các node cần thiết:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_ur_control_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(ccs2_socket_detector REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# CCS2 Socket Detector Node
add_executable(ccs2_detector_node src/ccs2_detector_node.cpp)
target_include_directories(ccs2_detector_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(ccs2_detector_node
  rclcpp sensor_msgs std_msgs cv_bridge OpenCV message_filters ccs2_socket_detector)

# Depth Heatmap Node (tùy chọn)
add_executable(depth_heatmap src/depth_heatmap.cpp)
ament_target_dependencies(depth_heatmap rclcpp sensor_msgs cv_bridge OpenCV)

# SolvePnP Node (tùy chọn)
add_executable(solvepnp src/solvepnp.cpp)
ament_target_dependencies(solvepnp rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV tf2 tf2_geometry_msgs)

install(TARGETS
  ccs2_detector_node
  depth_heatmap
  solvepnp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### Bước 4: Build

```bash
cd ~/your_new_ws
colcon build --packages-select ccs2_socket_detector my_ur_control_cpp
source install/setup.bash
```

---

## 4. Cấu hình & tùy chỉnh

### 4.1. Camera topics

Mặc định node subscribe:
- `/base_camera/base_camera_sensor/image_raw`
- `/base_camera/base_camera_sensor/depth/image_raw`

**Nếu camera của bạn khác**, sửa trong [`ccs2_detector_node.cpp`](my_ur_control_cpp/src/ccs2_detector_node.cpp) tại dòng 107-108:

```cpp
rgb_sub_.subscribe(this, "/your_camera/image_raw");
depth_sub_.subscribe(this, "/your_camera/depth/image_raw");
```

Hoặc dùng remapping trong launch file:

```python
Node(
    package='my_ur_control_cpp',
    executable='ccs2_detector_node',
    remappings=[
        ('/base_camera/base_camera_sensor/image_raw', '/your_camera/image_raw'),
        ('/base_camera/base_camera_sensor/depth/image_raw', '/your_camera/depth/image_raw'),
    ]
)
```

### 4.2. Parameters (tune theo camera & môi trường)

Có thể set qua launch file hoặc CLI:

```bash
# Chọn mode
ros2 param set /ccs2_detector_node use_color_filter true   # HSV (sim)
ros2 param set /ccs2_detector_node use_color_filter false  # Gamma (real)

# Gamma pipeline
ros2 param set /ccs2_detector_node gamma 0.001
ros2 param set /ccs2_detector_node median_ksize 9
ros2 param set /ccs2_detector_node morph_radius 12
ros2 param set /ccs2_detector_node use_clahe true
ros2 param set /ccs2_detector_node clahe_clip 2.0

# HSV filter (nếu dùng mode màu)
ros2 param set /ccs2_detector_node hsv_h_low 8
ros2 param set /ccs2_detector_node hsv_h_high 29
ros2 param set /ccs2_detector_node hsv_s_low 150
ros2 param set /ccs2_detector_node hsv_v_low 15

# Kích thước socket (mm) — đo từ socket thật
ros2 param set /ccs2_detector_node socket_h_mm 135.0
ros2 param set /ccs2_detector_node socket_w_mm 90.0

# Adaptive morph
ros2 param set /ccs2_detector_node adaptive_morph true
ros2 param set /ccs2_detector_node ref_depth_m 0.5
ros2 param set /ccs2_detector_node min_scale 0.20

# Aspect filter
ros2 param set /ccs2_detector_node use_aspect_filter true
ros2 param set /ccs2_detector_node aspect_ratio_min 0.55
ros2 param set /ccs2_detector_node aspect_ratio_max 2.80
```

### 4.3. Camera matrix (solvepnp)

Nếu camera của bạn khác (FOV, resolution khác), sửa trong [`solvepnp.cpp`](my_ur_control_cpp/src/solvepnp.cpp) tại dòng 26-28:

```cpp
// Ví dụ: camera 640×480, FOV 70°
double fx = 640.0 / (2.0 * tan((70.0 * M_PI / 180.0) / 2.0));
K_ = (cv::Mat_<double>(3, 3) << fx, 0, 320.0, 0, fx, 240.0, 0, 0, 1);
```

### 4.4. Object points (solvepnp)

Nếu socket của bạn khác kích thước, sửa trong [`solvepnp.cpp`](my_ur_control_cpp/src/solvepnp.cpp) tại dòng 30-34:

```cpp
// Ví dụ: socket 60mm × 80mm
float hw = 0.030f, hh = 0.040f;
obj_pts_ = {
    cv::Point3f(-hw, hh, 0), cv::Point3f(hw, hh, 0),
    cv::Point3f(hw, -hh, 0), cv::Point3f(-hw, -hh, 0)
};
```

---

## 5. Kiểm tra & chạy thử

### 5.1. Chạy detector node

```bash
# Terminal 1: Chạy detector
ros2 run my_ur_control_cpp ccs2_detector_node

# Terminal 2: Xem kết quả detect
ros2 topic echo /ccs2_detector_node/socket_detection

# Terminal 3: Xem debug image
ros2 run rqt_image_view rqt_image_view /ccs2_detector_node/debug_image
```

### 5.2. Chạy depth heatmap

```bash
ros2 run my_ur_control_cpp depth_heatmap
ros2 run rqt_image_view rqt_image_view /base_camera/depth_heatmap
```

### 5.3. Chạy solvepnp

```bash
# Lưu ý: cần file congsac2.png ở thư mục hiện tại
cp ~/your_new_ws/src/my_ur_control_cpp/config/congsac2.png .
ros2 run my_ur_control_cpp solvepnp
ros2 topic echo /object_pose
```

### 5.4. Launch file mẫu

Tạo file `launch/ccs2_detection.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_ur_control_cpp',
            executable='ccs2_detector_node',
            name='ccs2_detector_node',
            output='screen',
            parameters=[{
                'use_color_filter': False,
                'gamma': 0.001,
                'use_clahe': True,
                'adaptive_morph': True,
                'ref_depth_m': 0.5,
            }],
            remappings=[
                ('/base_camera/base_camera_sensor/image_raw', '/camera/image_raw'),
                ('/base_camera/base_camera_sensor/depth/image_raw', '/camera/depth/image_raw'),
            ]
        ),
        Node(
            package='my_ur_control_cpp',
            executable='depth_heatmap',
            name='depth_heatmap',
            output='screen',
        ),
    ])
```

---

## 6. Troubleshooting thường gặp

### 6.1. "No socket detected" liên tục

**Nguyên nhân thường gặp:**
1. **Camera topics sai** → kiểm tra bằng `ros2 topic list | grep image`
2. **Gamma quá cao/thấp** → thử `gamma=0.5` rồi giảm dần xuống `0.001`
3. **Thiếu sáng** → bật CLAHE: `use_clahe=true`
4. **Socket quá xa** → giảm `min_area` xuống `100`, tăng `ref_depth_m` lên `1.0`
5. **Aspect ratio sai** → nới lỏng `aspect_ratio_min=0.3`, `aspect_ratio_max=3.5`

**Cách debug:**
```bash
# Xem raw mask
ros2 run rqt_image_view rqt_image_view /ccs2_detector_node/raw_mask
# Xem mask sau xử lý
ros2 run rqt_image_view rqt_image_view /ccs2_detector_node/mask_image
# Xem debug image (có overlay)
ros2 run rqt_image_view rqt_image_view /ccs2_detector_node/debug_image
```

### 6.2. Depth toàn 0 hoặc NaN

**Nguyên nhân:**
1. Camera depth chưa được cấu hình đúng
2. Encoding sai (16UC1 vs 32FC1)

**Sửa:** Trong `syncCallback`, node tự động xử lý cả 32FC1 và 16UC1. Nếu depth vẫn 0, kiểm tra:
```bash
ros2 topic echo /camera/depth/image_raw --once | grep encoding
```

### 6.3. solvepnp không tìm thấy template

**Nguyên nhân:**
1. File `congsac2.png` không đúng thư mục
2. Template không khớp với socket thật

**Giải pháp:**
- Copy file vào thư mục hiện tại trước khi chạy
- Hoặc sửa đường dẫn tuyệt đối trong code
- Chụp ảnh template mới từ camera thật

### 6.4. Lỗi build

```bash
# Thiếu dependencies
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
# Hoặc tự build từ source nếu dùng ROS2 khác distribution
```

---

## 7. Tích hợp với các module khác

### 7.1. Kết nối với navigation (Nav2)

```
ccs2_detector_node → /socket_detection (depth_m, centroid)
    → Dùng depth_m để biết khoảng cách đến socket
    → Dùng centroid để căn chỉnh xe (PID điều khiển /cmd_vel)
```

### 7.2. Kết nối với arm controller (UR10)

```
solvepnp → /object_pose (PoseStamped)
    → Dùng position (x, y, z) làm target cho IK
    → Dùng orientation (quaternion) để căn góc cắm
```

### 7.3. Kết nối với docking station

```
god_view.py → Nav2 + PID
    → Dùng /socket_detection.depth_m để biết khi nào đủ gần
    → Dùng /socket_detection.centroid_x để căn trái/phải
```

---

## 8. Nâng cấp lên AI/ML sau này

Nếu sau này muốn thay thế classic CV bằng deep learning, cấu trúc topic vẫn giữ nguyên:

| Module cũ | Module mới | Topic giữ nguyên |
|-----------|-----------|------------------|
| `ccs2_detector_node` (classic) | YOLOv8 + segmentation | `~/socket_detection` |
| `solvepnp` (template) | Keypoint regression network | `/object_pose` |

**Các node downstream (plugin, god_view) không cần sửa** — chỉ cần đổi node upstream.

---

## File checklist tối thiểu cần copy

| File | Bắt buộc? | Mô tả |
|------|:---------:|-------|
| `ccs2_socket_detector/msg/SocketDetection.msg` | ✅ | Message definition |
| `ccs2_socket_detector/CMakeLists.txt` | ✅ | Build message |
| `ccs2_socket_detector/package.xml` | ✅ | Package metadata |
| `my_ur_control_cpp/src/ccs2_detector_node.cpp` | ✅ | Node chính |
| `my_ur_control_cpp/include/ccs2_socket_detector/ccs2_detector_node.hpp` | ✅ | Header |
| `my_ur_control_cpp/src/depth_heatmap.cpp` | ❌ Tùy chọn | Visualize depth |
| `my_ur_control_cpp/src/solvepnp.cpp` | ❌ Tùy chọn | Pose 6-DOF |
| `congsac2.png` | ❌ (nếu dùng solvepnp) | Template image |
| `my_ur_control_cpp/CMakeLists.txt` | ✅ | Build config |
| `my_ur_control_cpp/package.xml` | ✅ | Package metadata |

---

> **Tóm lại:** Chỉ cần copy 2 package `ccs2_socket_detector` + `my_ur_control_cpp` (tối thiểu 3 file .cpp), sửa topic camera cho phù hợp, build là chạy được. Toàn bộ classic CV, không cần GPU, không cần dataset, không cần training.

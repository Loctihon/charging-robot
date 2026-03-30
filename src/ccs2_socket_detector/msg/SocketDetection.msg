std_msgs/Header header

# Detection status
bool detected

# Centroid in image coordinates (pixels)
float32 centroid_x
float32 centroid_y

# Bounding box in image coordinates (pixels)
float32 bbox_x
float32 bbox_y
float32 bbox_width
float32 bbox_height

# Tilt angles in degrees
# XY: orientation in image plane (via ellipse fitting)
# YZ: vertical tilt relative to camera (uses depth + socket height H)
# XZ: horizontal tilt relative to camera (uses depth + socket width W)
float32 angle_xy
float32 angle_yz
float32 angle_xz

# Average depth to socket center (meters)
float32 depth_m
# circle_detector_node (solvepnp.cpp)

Detects the pin holes of a CCS2 charging socket in a binary mask image,
clusters them into upper/lower groups, selects 6 keypoints, and runs
`cv::solvePnP` to estimate the 6-DOF pose of the socket relative to the camera.

---

## Pipeline overview

```
/ccs2_detector_node/raw_mask  (sensor_msgs/Image, mono8 or bgr8)
        │
        ▼
  [1] Threshold + Invert
        │   dark holes  → white blobs
        │   socket body → black (invisible to findContours)
        │   background  → white (huge area, filtered out)
        ▼
  [2] findContours  (RETR_LIST)
        │
        ▼
  [3] Circularity filter
        │   area   ∈ [min_area, max_area]
        │   circ   = 4π·A / P²  ≥  min_circularity
        ▼
  [4] Upper / Lower clustering  (largest Y-gap)
        │
        ▼
  [5] Keypoint selection  (6 of 9 circles)
        │
        ▼
  [6] cv::solvePnP  (ITERATIVE)
        │
        ├──► /circle_detector/debug_image   (sensor_msgs/Image)
        ├──► /circle_detector/circle_center (geometry_msgs/PointStamped, one per circle)
        └──► /circle_detector/socket_pose   (geometry_msgs/PoseStamped)
```

---

## Step-by-step explanation

### Step 1 — Threshold + Invert

The input is a binary mask published by `ccs2_detector_node` on `~/raw_mask`.
White (255) = socket body, black (0) = dark pin holes + background.

```
input pixel > 127  →  255  (socket body)
input pixel ≤ 127  →    0  (holes, background)
```

After `bitwise_not`:
- socket body → 0 (invisible)
- pin holes   → 255 (small white blobs)
- background  → 255 (one huge blob, filtered by `max_area`)

### Step 2 — findContours

`cv::RETR_LIST` is used — no hierarchy needed since all blobs are independent
after the inversion step.

### Step 3 — Circularity filter

Each contour is accepted only if:

| Condition | Default |
|-----------|---------|
| `contourArea(c)` ≥ `min_area` | 5 px² |
| `contourArea(c)` ≤ `max_area` | 5000 px² |
| `4π·area / perimeter²` ≥ `min_circularity` | 0.55 |

The background blob has area ~ (image width × image height) ≫ 5000 px²
and is automatically discarded by `max_area`.

The centroid of each accepted contour is stored as `(cx, cy, radius)`
where `radius = √(area / π)`.

### Step 4 — Upper / Lower clustering

All accepted circles are sorted by **image Y** (ascending = top of image).
The algorithm finds the **largest gap** between consecutive Y values and
splits there:

```
circles sorted by Y:  ●●●●●●● · · · ●●
                      ───────────^────
                      upper (7)      lower (2)
                                 └── largest gap here
```

- **Upper group** = AC pins of the CCS2 socket (7 circles)
- **Lower group** = DC power pins (2 circles)

Both groups are then sorted by **image X** (ascending = left to right).

### Step 5 — Keypoint selection

Out of 9 detected circles, only 6 are used for `solvePnP` because only
6 have known 3D coordinates in the socket frame.

The 6 3D object points are (in metres, socket frame):

| Label | 3D X | 3D Y | 3D Z | Physical meaning |
|-------|------|------|------|-----------------|
| TL    | −0.080 | 0.027 | 0.0801 | top-left outer AC pin |
| TR    | +0.080 | 0.027 | 0.0801 | top-right outer AC pin |
| ML    | −0.016 | 0.042 | 0.0790 | mid-left inner AC pin |
| MR    | +0.016 | 0.042 | 0.0790 | mid-right inner AC pin |
| BL    | −0.080 | 0.042 | 0.0776 | bottom-left DC pin |
| BR    | +0.080 | 0.042 | 0.0776 | bottom-right DC pin |

**Selection logic:**

```
Upper group (sorted by image Y ascending):

  top 2 circles (smallest image Y)
  ────────────────────────────────
  Why: TL/TR are at 3D Y = 0.027 (physically higher on the socket face).
       A head-on camera sees higher 3D Y as smaller image Y (higher in frame).
  How: sort those 2 by image X → left = TL, right = TR

  remaining 5 circles:
  ────────────────────
  pick the 2 CLOSEST to the horizontal centre of the upper group
  Why: ML/MR are at 3D X = ±0.016 m, which is much closer to the socket
       centre than the other AC pins. In the image they appear near the
       horizontal centre of the upper cluster.
  How: compute centre_x = mean(upper cx), sort remaining by |cx − centre_x|,
       take the 2 with smallest distance → sort by X → left = ML, right = MR

Lower group (sorted by image X):
  leftmost  → BL
  rightmost → BR
```

**Debug overlay colours:**

| Colour | Meaning |
|--------|---------|
| Red ring + label | Selected TL / TR |
| Yellow ring + label | Selected ML / MR |
| Blue ring + label | Selected BL / BR |
| Orange dot | Reprojected 3D point (after PnP) — should overlap the ring |
| RGB axes | Coordinate frame of the socket (red=X, green=Y, blue=Z) |

If the **orange dots do not overlap the coloured rings**, the correspondence
is wrong — the most common causes are:

1. The socket is tilted so the "top 2 by image Y" rule picks wrong pins.
2. The 3D object points do not match the actual physical pin positions.

### Step 6 — solvePnP

```
cv::solvePnP(object_points, image_points, K, D=zeros, rvec, tvec,
             useExtrinsicGuess=false, SOLVEPNP_ITERATIVE)
```

Output:
- `tvec` — position of socket origin in camera frame (metres)
- `rvec` — rotation as Rodrigues vector, converted to quaternion for publishing

---

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/ccs2_detector_node/raw_mask` | `sensor_msgs/Image` | **sub** | Binary mask from detector |
| `/circle_detector/debug_image` | `sensor_msgs/Image` | **pub** | Annotated debug frame |
| `/circle_detector/circle_center` | `geometry_msgs/PointStamped` | **pub** | One message per detected circle (`x=cx, y=cy, z=radius`) |
| `/circle_detector/socket_pose` | `geometry_msgs/PoseStamped` | **pub** | 6-DOF pose of socket in camera frame |

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_area` | 5.0 | Minimum contour area in px² |
| `max_area` | 5000.0 | Maximum contour area in px² |
| `min_circularity` | 0.55 | Minimum circularity (0=line, 1=perfect circle) |
| `fx` | 1513.7423 | Camera focal length X (px) |
| `fy` | 1513.7423 | Camera focal length Y (px) |
| `cx` | 640.5 | Principal point X (px) |
| `cy` | 360.5 | Principal point Y (px) |

Tune at runtime without recompiling:
```bash
ros2 param set /circle_detector_node min_circularity 0.4
ros2 param set /circle_detector_node min_area 10.0
```

---

## Build & run

```bash
# Build
cd ~/charging-robot
colcon build --packages-select my_ur_control_cpp
source install/setup.bash

# Run
ros2 run my_ur_control_cpp solvepnp

# View debug image
ros2 run rqt_image_view rqt_image_view /circle_detector/debug_image

# Check pose
ros2 topic echo /circle_detector/socket_pose
```

---

## Modifying the 3D object points

The 6 object points are hardcoded in the constructor of `CircleDetectorNode`
inside `solvepnp.cpp`:

```cpp
object_points_ = {
  {-0.08f,  0.027f, 0.0801f},  // TL
  { 0.08f,  0.027f, 0.0801f},  // TR
  {-0.016f, 0.042f, 0.079f},   // ML
  { 0.016f, 0.042f, 0.079f},   // MR
  {-0.08f,  0.042f, 0.0776f},  // BL
  { 0.08f,  0.042f, 0.0776f},  // BR
};
```

These must be measured from the physical socket in its own coordinate frame
and must remain in the same order as the keypoint selection logic produces
(TL, TR, ML, MR, BL, BR).

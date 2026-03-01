# FAST-LIO2 ROS 2 — Backpack Scanner Modifications

All changes made to the [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
fork for the Lidar Backpack V2 project. The upstream repo provides the ROS 1 to ROS 2
migration; changes listed here are backpack-specific customizations on top of that.

---

## 1. Ouster Ring Field Fix (Critical — Drift Fix)

**File:** `src/preprocess.h`

Changed the Ouster point type's ring field from `uint8_t` to `uint16_t` to match what
the Ouster ROS 2 driver actually publishes.

```cpp
// Before (line ~98):
uint8_t ring;
// ...
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    // ...
    (std::uint8_t, ring, ring)
)

// After:
uint16_t ring;
// ...
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    // ...
    (std::uint16_t, ring, ring)
)
```

**Why:** The upstream Ouster ROS driver publishes `ring` as a 16-bit field. When FAST-LIO
read it as 8-bit, point ring assignments were corrupted (values > 255 wrapped around).
This caused incorrect scan line ordering, which degraded the ICP matching and produced
severe localization drift. This was the single biggest fix for scan quality.

**Impact:** Without this fix, scans would accumulate drift and eventually spiral into
divergence. With the fix, the Ouster produces stable, accurate scans.

---

## 2. Gravity Alignment at IMU Initialization

**File:** `src/IMU_Processing.hpp`, function `IMU_init()`

Added automatic gravity alignment so FAST-LIO works correctly regardless of sensor
mounting orientation.

```cpp
// NEW: Compute rotation that aligns measured gravity with Z-up
Eigen::Quaterniond gravity_align =
    Eigen::Quaterniond::FromTwoVectors(mean_acc, Eigen::Vector3d::UnitZ());
mean_acc = gravity_align * mean_acc;
mean_gyr = gravity_align * mean_gyr;

state_ikfom init_state = kf_state.get_x();
init_state.rot = gravity_align;  // Initialize rotation to gravity-aligned frame
init_state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2);
```

**Why:** The backpack scanner mounts sensors at various angles:
- Ouster OS0-32: mounted vertically (90-degree rotation around Y axis)
- Livox MID-360: mounted at 20-30 degree incline on backpack frame

Without gravity alignment, the initial rotation is identity, which assumes the sensor
is perfectly level. The EKF would then fight against the actual gravity direction,
causing drift.

With this fix, FAST-LIO reads the IMU accelerometer during the ~1 second initialization
window, determines which way is "down", and sets the initial rotation accordingly. This
works for any mounting angle.

**Impact:** Eliminates the need to manually compute `extrinsic_R` for the mounting angle.
The sensor can be mounted in any orientation and FAST-LIO will auto-align to gravity.

---

## 3. MID-360 Lidar Type Support

**File:** `src/preprocess.h`

Added `MID360` to the lidar type enum:

```cpp
enum LID_TYPE {
    AVIA = 1,
    VELO16,
    OUST64,
    MID360    // NEW
};
```

Added Livox point type definitions for MID-360 compatibility:

```cpp
namespace livox_ros {
    struct LivoxPointXyzrtl {
        float x, y, z, reflectivity;
        uint8_t tag, line;
    };
    struct LivoxPointXyzitl {
        float x, y, z, intensity;
        uint8_t tag, line;
    };
}
```

Both point types registered with PCL for automatic conversions.

**File:** `src/preprocess.cpp`

Added `mid360_handler()` function and case in the lidar type switch:

```cpp
case MID360:
    mid360_handler(msg);
    break;
```

**Why:** The upstream FAST_LIO_ROS2 fork only handled Avia-style Livox custom messages.
The MID-360 uses a different point layout with `reflectivity` instead of `intensity` and
a different field order. The new handler processes MID-360 point clouds correctly.

---

## 4. First Lidar Message Guard

**File:** `src/laserMapping.cpp`

Added a check to prevent false "lidar loop back" detection on the first message:

```cpp
bool is_first_lidar = true;  // NEW global

void standard_pcl_cbk(...) {
    double cur_time = get_time_sec(msg->header.stamp);
    if (!is_first_lidar && cur_time < last_timestamp_lidar) {  // NEW guard
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar) {
        is_first_lidar = false;
    }
    // ...
}
```

Same pattern applied to `livox_pcl_cbk()`.

**Why:** On startup, `last_timestamp_lidar` is 0. If the first lidar message has a
timestamp that's somehow less than a previous session's residual value, or if there's
a timing edge case, FAST-LIO would clear its buffer and log a confusing "loop back"
warning. The guard ensures the first message is always accepted.

---

## 5. RViz Visualization Tuning

**File:** `rviz/fastlio.rviz`

Adjusted point cloud visualization for better clarity on smaller screens:

| Setting | Before | After |
|---------|--------|-------|
| Point size (pixels) | 3 | 2 |
| Point size (meters) | 0.05 | 0.01 |
| Render style | Flat Squares | Points |
| View distance | 217 | 50 |
| View pitch | 1.57 | 0.79 |

**Why:** Smaller point sizes and closer default view provide better detail for indoor
and forest environments where the backpack is used.

---

## 6. Dense Publish Setting

**File:** `config/mid360.yaml`

Changed `dense_publish_en: false` to `dense_publish_en: true`.

**Why:** Dense publishing includes all points (not just feature points) in the registered
cloud output. This produces better-looking visualizations in RViz and is needed for the
scan health monitor's point count detection.

---

## 7. Backpack Configuration Files (New)

### `config/ouster32_backpack.yaml`

Tuned for the Ouster OS0-32 on the rotating backpack mount:

```yaml
preprocess:
    lidar_type: 3        # Ouster
    scan_line: 32        # OS0-32
    timestamp_unit: 3    # nanoseconds
    blind: 0.5           # OS0 min range ~0.3m

mapping:
    extrinsic_est_en: false          # Fixed mount, known transform
    extrinsic_T: [0.0, 0.0, 0.0]    # Co-located with IMU
    extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Identity (gravity alignment handles orientation)

publish:
    dense_publish_en: true           # Full cloud for health monitor
    scan_bodyframe_pub_en: true      # Body frame for debugging

pcd_save:
    pcd_save_en: true    # Save PCD on graceful exit
    interval: -1         # Single file (not periodic)
```

Key choice: `extrinsic_est_en: false` with identity extrinsic. The Ouster driver is
configured with `point_cloud_frame: os_sensor`, which aligns the point cloud with the
IMU frame. Combined with gravity alignment, this means the identity extrinsic is correct.

### `config/mid360_backpack.yaml`

Tuned for the Livox MID-360 on the backpack frame:

```yaml
preprocess:
    lidar_type: 1        # Livox
    scan_line: 4         # MID-360 has 4 scan lines
    blind: 0.5

mapping:
    extrinsic_est_en: true                          # Online calibration enabled
    extrinsic_T: [-0.011, -0.02329, 0.04412]       # Measured offset
    extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]      # Identity (gravity-aligned)
```

Key choice: `extrinsic_est_en: true` for the Livox. The MID-360's IMU-to-lidar offset
was measured physically but online estimation refines it during operation.

---

## Summary of Changes

| # | Change | File(s) | Impact |
|---|--------|---------|--------|
| 1 | Ring field uint8 to uint16 | preprocess.h | **Critical** — fixes Ouster drift |
| 2 | Gravity alignment | IMU_Processing.hpp | Handles arbitrary mount angles |
| 3 | MID-360 support | preprocess.h, preprocess.cpp | Enables dual-lidar setup |
| 4 | First lidar guard | laserMapping.cpp | Prevents false sync warnings |
| 5 | RViz visualization | fastlio.rviz | Better point cloud display |
| 6 | Dense publish | mid360.yaml | Full cloud output |
| 7 | Backpack configs | config/*_backpack.yaml | Hardware-specific tuning |

---

## Files Modified (vs upstream)

```
Modified:
  src/preprocess.h         # Ring field fix + MID-360 types
  src/preprocess.cpp       # MID-360 handler
  src/IMU_Processing.hpp   # Gravity alignment
  src/laserMapping.cpp     # First lidar guard
  config/mid360.yaml       # Dense publish
  rviz/fastlio.rviz        # Visualization tuning

Added (untracked):
  config/ouster32_backpack.yaml
  config/mid360_backpack.yaml
  config/ouster32.yaml
  CHANGES_BACKPACK.md      # This file
```

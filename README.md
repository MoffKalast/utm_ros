# UTM ROS

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


A set of tools for turning GNSS LLA data into local [UTM](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system) metric values and back. It converts NavSatFix (and optionally Imu) data into local UTM-based Odometry, Pose, and TF topics.

Expect usable (e.g. < 1m) accuracy at most ~30km from the local origin.

### Install

```bash
sudo apt install python3-utm
```
That can fail on older systems, as a fallback you can use:
```bash
pip install utm
```

### Launch

For a typical MavROS config:
```bash
roslaunch utm_ros mavros.launch
```

For a ublox receiver driver with heading support:
```bash
roslaunch utm_ros ublox_heading.launch
```

For a nmea_navsat_driver receiver:
```bash
roslaunch utm_ros nmea_navsat.launch
```

### Subscribed Topics

* `/fix` (`sensor_msgs/NavSatFix`)

  * Incoming GNSS data used to calculate UTM position.
  * Mandatory for node operation.

* `/imu/data` (`sensor_msgs/Imu`) *(optional)*

  * Quaternion pose used for Pose, Odometry, and TF messages.
  * Only used if `~use_imu` is set to `True`.

#### Published Topics

* `/gnss/local_origin_fix` (`sensor_msgs/NavSatFix`)

  * Latched  initial GNSS fix used as the local UTM origin, for displaying map tiles relative to the origin.

* `/gnss/pose` (`geometry_msgs/PoseWithCovarianceStamped`)

  * Pose in the local UTM frame.

* `/gnss/odom` (`nav_msgs/Odometry`) *(optional)*

  * Odometry message in the local UTM frame.
  * Only published if `~publish_odom` is `True`.

#### Services

* `/gnss/lla_to_utm_local` (`gnss_nav/LLAToUTM`)

  * Converts a latitude/longitude input into local UTM coordinates (relative to origin).
  * Returns `x`, `y` (meters) and success flag.

* `/gnss/utm_local_to_lla` (`gnss_nav/UTMToLLA`)

  * Converts local UTM coordinates (relative to origin) back to latitude/longitude.
  * Returns `latitude`, `longitude` and success flag.


### Parameters

* `~base_frame` (default: `"base_link"`)

  * The name of the base frame of the robot.
  * Used as the `child_frame_id` if publishing TF transforms.

* `~local_frame` (default: `"local"`)

  * The name of the local UTM frame based on the first received GNSS fix, published relative to world. This is what should be used as your visualization fixed_frame.
  * Acts as the parent frame for local pose and odometry data.

* `~world_frame` (default: `"world"`)

  * The top-level frame, representing a fixed world reference at 0,0 LLA.
  * Used for fixed positions since they remain invariant across restarts and reboots while the local frame changes, for e.g. waypoints.

* `~publish_tf` (default: `True`)

  * Whether to publish the transform between the `local_frame` and the `base_frame`. The full Tf graph would be `world_frame -> local_frame -> base_frame`.

* `~publish_odom` (default: `True`)

  * Whether to publish odometry messages (`/gnss/odom`).

* `~use_imu` (default: `False`)

  * Whether to subscribe to IMU data (`/imu/data`) and use its orientation in pose and odometry. If false, an identity quaternion is used.

* `~ignore_imu_covariance` (default: `True`)

  * If `True`, ignores the IMU's covariance and uses its orientation regardless of uncertainty.
  * If `False`, only uses orientation if the covariance is below a threshold.

* `~ignore_fix_status` (default: `False`)

  * If `True`, bypasses the GNSS fix status check (`NavSatFix.status.status`).
  * Useful if the GPS driver provides a nonsensical fix status, such as with MavROS.

# ROS 2 pointcloud <-> laserscan converters

This is a ROS 2 package that provides components to convert `sensor_msgs/msg/PointCloud2` messages to `sensor_msgs/msg/LaserScan` messages and back.
It is essentially a port of the original ROS 1 package.

## pointcloud\_to\_laserscan::PointCloudToLaserScanNode

This ROS 2 component projects `sensor_msgs/msg/PointCloud2` messages into `sensor_msgs/msg/LaserScan` messages.

### Published Topics

* `scan` (`sensor_msgs/msg/LaserScan`) - The output laser scan.

### Subscribed Topics

* `cloud_in` (`sensor_msgs/msg/PointCloud2`) - The input point cloud. By default, input is only processed when there is at least one subscriber to `scan` (lazy subscription).

### Parameters

* `min_height` (double, default: 2.2e-308) - The minimum height to sample in the point cloud in meters.
* `max_height` (double, default: 1.8e+308) - The maximum height to sample in the point cloud in meters.
* `angle_min` (double, default: -π) - The minimum scan angle in radians.
* `angle_max` (double, default: π) - The maximum scan angle in radians.
* `angle_increment` (double, default: π/180) - Resolution of laser scan in radians per ray.
* `queue_size` (int, default: detected number of cores) - Input point cloud queue size.
* `input_qos_reliability` (str, default: `best_effort`) - Input subscription reliability QoS. Supported values: `best_effort`, `reliable`.
* `input_qos_durability` (str, default: `volatile`) - Input subscription durability QoS. Supported values: `volatile`, `transient_local`.
* `always_subscribe` (bool, default: false) - Keep the `cloud_in` subscription active even when there are no `scan` subscribers.
* `auto_resubscribe_on_stall` (bool, default: false) - If enabled (and `always_subscribe` is true), automatically recreate the `cloud_in` subscription when a stall is detected.
* `exit_on_cloud_stall` (bool, default: true) - If enabled, exit the process when cloud input stalls while publishers are still active (useful with launch respawn policies).
* `diagnostics_timeout_sec` (double, default: 2.0) - Stall detection timeout used by periodic diagnostics.
* `resubscribe_cooldown_sec` (double, default: 15.0) - Minimum interval between automatic re-subscribe attempts.
* `scan_time` (double, default: 1.0/30.0) - The scan rate in seconds. Only used to populate the scan_time field of the output laser scan message.
* `range_min` (double, default: 0.0) - The minimum ranges to return in meters.
* `range_max` (double, default: 1.8e+308) - The maximum ranges to return in meters.
* `target_frame` (str, default: none) - If provided, transform the pointcloud into this frame before converting to a laser scan. Otherwise, laser scan will be generated in the same frame as the input point cloud.
* `transform_tolerance` (double, default: 0.01) - Time tolerance for transform lookups. Only used if a `target_frame` is provided.
* `use_inf` (boolean, default: true) - If disabled, report infinite range (no obstacle) as range_max + 1. Otherwise report infinite range as +inf.
* `inf_epsilon` (double, default: 1.0) - Value added to `range_max` when `use_inf` is false.

## pointcloud\_to\_laserscan::LaserScanToPointCloudNode

This ROS 2 component re-publishes `sensor_msgs/msg/LaserScan` messages as `sensor_msgs/msg/PointCloud2` messages.

### Published Topics

* `cloud` (`sensor_msgs/msg/PointCloud2`) - The output point cloud.

### Subscribed Topics

* `scan_in` (`sensor_msgs/msg/LaserScan`) - The input laser scan. Input is processed only when there is at least one subscriber to `cloud` (lazy subscription).

### Parameters

* `queue_size` (int, default: detected number of cores) - Input laser scan queue size.
* `target_frame` (str, default: none) - If provided, transform the laser scan into this frame before converting to a pointcloud. Otherwise, pointcloud will be generated in the same frame as the input laser scan.
* `transform_tolerance` (double, default: 0.01) - Time tolerance for transform lookups. Only used if a `target_frame` is provided.

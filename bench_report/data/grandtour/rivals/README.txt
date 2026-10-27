Rivals on the GrandTour development missions (chapter 33). Each runs its own published parameters; only the sensor
settings change: Hesai XT-32 + STIM320, extrinsics from the export (LiDAR -> IMU), per-point time field `time`.

BIEVR-LIO   github.com/leggedrobotics/BIEVR-LIO (= ethz-asl/BIEVR-LIO) 21121698f273d6fbfffca57546b940edb1de2ff0,
            BSD-3-Clause, built on ROS 2 Jazzy with the system Ceres 2.2. config/params.yaml as shipped, with
            trajectory_path set and the dashboard off. Run: scripts/p1_grandtour_rival.sh bievr ...
            COMFORT test leaderboard: 0.022 m (patripfr, 2026-09-22).
FAST-LIO2   github.com/hku-mars/FAST_LIO branch ROS2 cc3c9e8b1469b33b8072764d6db2348ed6dc597f (+ ikd-Tree submodule
            e2e3f4e9), GPL-2.0. Two build-only changes: C++17 for ROS 2 Jazzy (fastlio_jazzy_cxx17.patch), and a
            message-only livox_ros_driver2 package (livox_ros_driver2_msgs_only/, CustomMsg and CustomPoint as in
            Livox-SDK/livox_ros_driver2) instead of the driver and Livox-SDK2; the bags are PointCloud2.
            COMFORT test leaderboard: 0.034 m (parkhaneum, 2026-09-17).

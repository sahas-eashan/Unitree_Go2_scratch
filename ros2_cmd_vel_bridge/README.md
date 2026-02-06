# ros2_cmd_vel_bridge

C++ ROS 2 node that bridges `/cmd_vel` into Unitree Sport API requests.

## What It Does
- Subscribes to `/cmd_vel` (`geometry_msgs/Twist`)
- Publishes `unitree_api::msg::Request` to `/api/sport/request`
- Enforces safety limits and a watchdog timeout

## Run (Robot)
```bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/go2_bringup_ws/install/setup.bash
ros2 run cmd_vel_unitree_bridge cmd_vel_unitree_bridge_node
```

## Notes
- Tune limits via params: `max_vx`, `max_vy`, `max_wz`, `timeout_s`.
- Ensure CycloneDDS is configured between laptop and robot.

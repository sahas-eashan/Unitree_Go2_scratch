# Go2 Scratch / TurboWarp Teleop (ROS 2 Foxy)

This project lets you control a Unitree Go2 via TurboWarp (Scratch-like blocks) by sending velocity commands over WebSocket to a ROS 2 `/cmd_vel` publisher.

## Components
- `ros2_ws_bridge/scratch_cmdvel_ws.py`: WebSocket server (ws://0.0.0.0:8765) that publishes `/cmd_vel`
- `turbowarp_project/go2_teleop.sb3`: TurboWarp project that sends JSON messages like `{"vx":0.2,"vy":0,"wz":0.7}`

## Prerequisites
- Ubuntu 20.04 + ROS 2 Foxy
- Python 3.8
- `rclpy` available in your environment
- Network configured to reach robot (if `/cmd_vel` must reach robot over DDS)

## Run (Laptop)
### 1) Start the WebSocket -> /cmd_vel bridge
```bash
cd ros2_ws_bridge
pip3 install --user -r requirements.txt
source ~/unitree_ros2/setup.sh
python3 scratch_cmdvel_ws.py
```

### 2) Run TurboWarp project

Open TurboWarp editor and load `turbowarp_project/go2_teleop.sb3`.

Connection URL:

* If TurboWarp runs on the SAME laptop as the server: `ws://127.0.0.1:8765` (recommended)
* If connecting from another device on LAN: `ws://<LAPTOP_IP>:8765` (may require browser permissions for insecure WebSockets)

## Verify

```bash
source ~/unitree_ros2/setup.sh
ros2 topic hz /cmd_vel
```

## Robot motion

To actually move the robot, ensure your `/cmd_vel -> Unitree Sport API` bridge is running on the robot.

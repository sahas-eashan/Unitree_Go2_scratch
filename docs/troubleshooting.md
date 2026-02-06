# Troubleshooting

## WebSocket Connection Fails
- Confirm the bridge is running and prints `WebSocket listening on ws://0.0.0.0:8765`.
- If connecting from another device, use the laptop's LAN IP, not `127.0.0.1`.
- Check firewall rules and ensure port `8765` is open.
- Some browsers block insecure WebSockets from `https://` pages. Allow insecure content or use a local file.

## `/cmd_vel` Not Publishing
- Re-source your ROS 2 environment: `source ~/unitree_ros2/setup.sh`.
- Verify the topic exists with `ros2 topic list`.
- Check for Python dependency errors in the bridge logs.

## Robot Not Moving
- Make sure your `/cmd_vel` -> Unitree Sport API bridge is running on the robot.
- Confirm the robot is in a mode that accepts velocity commands and is not E-stopped.
- Verify DDS networking between laptop and robot (CycloneDDS config, peers, interface).

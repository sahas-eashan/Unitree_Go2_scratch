# Setup

1. Install ROS 2 Foxy and ensure `rclpy` works.
2. Install Python deps for the bridge:
```bash
cd ros2_ws_bridge
pip3 install --user -r requirements.txt
```
3. Source your ROS 2 environment:
```bash
source ~/unitree_ros2/setup.sh
```
4. Run the WebSocket bridge:
```bash
python3 scratch_cmdvel_ws.py
```
5. Open TurboWarp and load `turbowarp_project/go2_teleop.sb3`.
6. Set the WebSocket URL:
- Same laptop: `ws://127.0.0.1:8765`
- Another device on LAN: `ws://<LAPTOP_IP>:8765`
7. Verify messages arrive:
```bash
ros2 topic hz /cmd_vel
```

## Network Notes
- If TurboWarp runs on another device, make sure port `8765` is allowed through your firewall.
- Some browsers block insecure WebSockets from `https://` pages. Use a local file or allow insecure content.

# ros2_ws_bridge

WebSocket -> ROS 2 `/cmd_vel` bridge for TurboWarp/Scratch.

## What It Does
- Listens on `ws://0.0.0.0:8765`
- Accepts JSON like `{"vx":0.2,"vy":0,"wz":0.7}`
- Publishes `geometry_msgs/Twist` on `/cmd_vel`
- Watchdog stops the robot if commands stop arriving

## Run
```bash
pip3 install --user -r requirements.txt
source ~/unitree_ros2/setup.sh
python3 scratch_cmdvel_ws.py
```

## Notes
- Safety limits and watchdog timing are defined at the top of `scratch_cmdvel_ws.py`.
- Use `ws://127.0.0.1:8765` if TurboWarp is on the same laptop.

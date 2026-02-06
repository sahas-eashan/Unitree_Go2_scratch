#!/usr/bin/env python3
"""
TurboWarp / Scratch WebSocket -> ROS 2 /cmd_vel bridge (Foxy)

- Listens on ws://0.0.0.0:8765
- Expects JSON messages like: {"vx":0.2,"vy":0,"wz":0.7}
- Publishes geometry_msgs/Twist on /cmd_vel
- Watchdog: if commands stop for WATCHDOG_SEC, publishes zero Twist
- Logs connections + received messages (so your terminal is not "silent")
"""

import asyncio
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import websockets

HOST = "0.0.0.0"
PORT = 8765

# Safety limits (tune as needed)
MAX_VX = 0.6
MAX_VY = 0.3
MAX_WZ = 1.2

# Stop robot if no commands received recently
WATCHDOG_SEC = 0.5

# Publish loop rate (your Scratch/TurboWarp loop is ~20 Hz)
WATCHDOG_TICK_SEC = 0.05  # 20 Hz


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ScratchCmdVelBridge(Node):
    def __init__(self):
        super().__init__("scratch_cmdvel_ws_bridge")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.last_cmd_time = time.time()

        # Timer to enforce watchdog stop
        self.create_timer(WATCHDOG_TICK_SEC, self._watchdog_tick)

        self.get_logger().info("ScratchCmdVelBridge node started. Publishing /cmd_vel")

    def publish_cmd(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x = clamp(vx, -MAX_VX, MAX_VX)
        msg.linear.y = clamp(vy, -MAX_VY, MAX_VY)
        msg.angular.z = clamp(wz, -MAX_WZ, MAX_WZ)
        self.pub.publish(msg)
        self.last_cmd_time = time.time()

    def stop(self):
        self.pub.publish(Twist())

    def _watchdog_tick(self):
        # If no new cmd recently, stop
        if (time.time() - self.last_cmd_time) > WATCHDOG_SEC:
            self.stop()


async def ws_handler(websocket, _path, node: ScratchCmdVelBridge):
    # Log connection
    print(f"WS CONNECT from {websocket.remote_address}", flush=True)
    node.get_logger().info(f"TurboWarp connected: {websocket.remote_address}")

    try:
        async for message in websocket:
            # Log received raw message (useful for debugging)
            print(f"WS MSG: {message}", flush=True)

            try:
                data = json.loads(message)

                # Accept several possible key spellings (vx/vy/wz are preferred)
                vx = float(data.get("vx", 0.0))
                vy = float(data.get("vy", 0.0))
                wz = float(data.get("wz", 0.0))

                node.publish_cmd(vx, vy, wz)

            except Exception as e:
                node.get_logger().warn(f"Bad msg (ignored): {message} | err={e}")

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        node.get_logger().info("TurboWarp disconnected; STOP")
        print("WS DISCONNECT -> STOP", flush=True)
        node.stop()


async def main_async(node: ScratchCmdVelBridge):
    server = await websockets.serve(
        lambda ws, p: ws_handler(ws, p, node),
        HOST,
        PORT,
        ping_interval=20,
        ping_timeout=20,
    )

    node.get_logger().info(f"WebSocket listening on ws://{HOST}:{PORT}")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.01)
    finally:
        server.close()
        await server.wait_closed()


def main():
    rclpy.init()
    node = ScratchCmdVelBridge()
    try:
        asyncio.run(main_async(node))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# ws_ros2_bridge.py

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
import json
import asyncio
import threading
import websockets

PORT = 8080


class WSROS2Bridge(Node):
    def __init__(self):
        super().__init__('ws_ros2_bridge')

        # ---------------- ROS2 Publishers ----------------
        self.joint_pub   = self.create_publisher(JointState,         '/fk_joint_states', 10)
        self.pose_pub    = self.create_publisher(Float64MultiArray,   '/ik_target_pose',  10)
        self.mission_pub = self.create_publisher(String,              '/mission_cmd',     10)

        # ---------------- ROS2 Subscribers ----------------
        self.status_sub      = self.create_subscription(String, '/rover_status', self.rover_status_cb, 10)
        self.node_status_sub = self.create_subscription(String, '/node_status',  self.node_status_cb,  10)

        # ---------------- Internal ----------------
        self.ws_clients   = set()
        self.rover_status = {
            "rover_state": "IDLE",
            "active_mission": "",
            "supervisor_message": "",
            "node_statuses": []
        }

        # asyncio event loop running in a dedicated thread
        self.loop = asyncio.new_event_loop()
        self.ws_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.ws_thread.start()

    # ── asyncio loop thread ──────────────────────────────────────────────────

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._ws_server())

    async def _ws_server(self):
        async with websockets.serve(self._handler, "0.0.0.0", PORT):
            self.get_logger().info(f"WebSocket server running on ws://0.0.0.0:{PORT}")
            await asyncio.Future()  # run forever

    # ── WebSocket handler ────────────────────────────────────────────────────

    async def _handler(self, websocket):
        self.ws_clients.add(websocket)
        self.get_logger().info(f"Client connected: {websocket.remote_address}")
        try:
            async for message in websocket:
                await self._handle_message(message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)
            self.get_logger().info(f"Client disconnected: {websocket.remote_address}")

    async def _handle_message(self, message):
        try:
            msg      = json.loads(message)
            msg_type = msg.get("type")

            if msg_type == "joint_cmd":
                mode = msg.get("mode")
                data = msg.get("data", [])

                if mode == "FK" and len(data) == 6:
                    joint_msg          = JointState()
                    joint_msg.name     = [f'joint{i+1}' for i in range(6)]
                    joint_msg.position = [float(x) for x in data]
                    self.joint_pub.publish(joint_msg)

                elif mode == "IK":
                    pose_msg      = Float64MultiArray()
                    pose_msg.data = [float(x) for x in data]
                    self.pose_pub.publish(pose_msg)

            elif msg_type == "mission_cmd":
                cmd     = msg.get("command", "")
                mission = msg.get("mission", "")
                out     = String()
                out.data = json.dumps({"command": cmd, "mission": mission})
                self.mission_pub.publish(out)

        except Exception as e:
            self.get_logger().error(f"Failed to handle WS message: {e}")

    # ── broadcast helper (thread-safe) ───────────────────────────────────────

    def broadcast(self, payload: str):
        """Called from ROS callbacks (main thread) — schedules send on the WS loop."""
        if not self.ws_clients:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self.loop)

    async def _broadcast(self, payload: str):
        dead = set()
        for client in self.ws_clients:
            try:
                await client.send(payload)
            except Exception:
                dead.add(client)
        self.ws_clients -= dead

    # ── ROS2 callbacks ───────────────────────────────────────────────────────

    def rover_status_cb(self, msg):
        try:
            self.rover_status = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse rover_status: {e}")

        self.broadcast(json.dumps({
            "type": "rover_status",
            "data": json.dumps(self.rover_status)
        }))

    def node_status_cb(self, msg):
        self.broadcast(json.dumps({
            "type": "node_status",
            "data": msg.data
        }))


# ── entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WSROS2Bridge()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
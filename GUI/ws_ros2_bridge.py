#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import json
import threading
import websocket

class WSROS2Bridge(Node):
    def __init__(self, ws_url="ws://localhost:8080"):
        super().__init__('ws_ros2_bridge')

        # ===== Publishers =====
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.ik_pub = self.create_publisher(Pose, '/ik_target', 10)

        # ===== Dummy joint state feedback =====
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)

        # ===== WebSocket setup =====
        self.ws = websocket.WebSocketApp(
            ws_url,
            on_message=self.on_message,
            on_open=lambda ws: print("[WS] Connected to ArmControlView"),
            on_close=lambda ws: print("[WS] WebSocket closed")
        )
        threading.Thread(target=self.ws.run_forever, daemon=True).start()

        # Joint values cache
        self.joint_values = [0.0]*6

    def on_message(self, ws, message):
        msg = json.loads(message)
        if msg.get("type") == "joint_cmd":
            self.publish_joint_command(msg)
        # You can add more types if needed

    def publish_joint_command(self, msg):
        mode = msg.get("mode", "FK")
        data = msg.get("data", {})
        
        # ===== JointState for FK =====
        if mode == "FK":
            joint_msg = JointState()
            joint_msg.name = [f'joint{i}' for i in range(1,7)]
            joint_msg.position = [data.get(f'joint{i}', 0) * 3.1416/180 for i in range(1,7)]
            self.joint_pub.publish(joint_msg)
            self.joint_values = joint_msg.position
            self.get_logger().info(f"Published FK joint command: {joint_msg.position}")
        
        # ===== Pose for IK =====
        elif mode == "IK":
            # Expecting data.x, data.y, data.z
            pose = Pose()
            pose.position.x = data.get("x", 0.0)
            pose.position.y = data.get("y", 0.0)
            pose.position.z = data.get("z", 0.0)
            self.ik_pub.publish(pose)
            self.get_logger().info(f"Published IK target pose: {pose.position}")

    def publish_joint_state(self):
        # Dummy feedback; in real robot, subscribe to actual joint states
        state_msg = JointState()
        state_msg.name = [f'joint{i}' for i in range(1,7)]
        state_msg.position = self.joint_values
        self.state_pub.publish(state_msg)

def main():
    rclpy.init()
    node = WSROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()

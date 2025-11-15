#!/usr/bin/env python3
"""
Click-to-Move Navigation Node
Listens to /clicked_point from Foxglove and sends movement commands to Alvik
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import paho.mqtt.client as mqtt
import json
import math


class ClickToMoveNode(Node):
    def __init__(self):
        super().__init__('click_to_move')

        # Parameters
        self.declare_parameter('mqtt_host', '192.168.1.152')
        self.declare_parameter('mqtt_port', 1883)

        mqtt_host = self.get_parameter('mqtt_host').value
        mqtt_port = self.get_parameter('mqtt_port').value

        # Subscribe to clicked point from Foxglove
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        # Connect to MQTT
        self.mqtt_client = mqtt.Client(client_id="alvik_click_to_move")
        self.mqtt_client.connect(mqtt_host, mqtt_port, 60)
        self.mqtt_client.loop_start()

        # Robot state (simple odometry tracking)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.get_logger().info(f'Click-to-Move node started, connected to MQTT at {mqtt_host}:{mqtt_port}')
        self.get_logger().info('Click points in Foxglove 3D view to move the robot!')

    def clicked_point_callback(self, msg: PointStamped):
        """Handle clicked point from Foxglove"""
        target_x = msg.point.x
        target_y = msg.point.y

        self.get_logger().info(f'Clicked point: ({target_x:.2f}, {target_y:.2f})')

        # Calculate distance and angle to target
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y

        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)

        # Calculate rotation needed
        angle_diff = target_angle - self.robot_theta

        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        angle_deg = math.degrees(angle_diff)
        distance_cm = distance * 100  # meters to cm

        self.get_logger().info(f'Move plan: rotate {angle_deg:.1f}°, then move {distance_cm:.1f}cm')

        # Send commands to Alvik via MQTT
        if abs(angle_deg) > 5:  # Only rotate if significant
            rotate_cmd = {
                'type': 'rotate',
                'angle': angle_deg
            }
            self.mqtt_client.publish('alvik/command', json.dumps(rotate_cmd))
            self.get_logger().info(f'Sent rotate command: {angle_deg:.1f}°')

        if distance_cm > 2:  # Only move if significant
            move_cmd = {
                'type': 'move',
                'distance': distance_cm
            }
            self.mqtt_client.publish('alvik/command', json.dumps(move_cmd))
            self.get_logger().info(f'Sent move command: {distance_cm:.1f}cm')

        # Update estimated position (dead reckoning)
        self.robot_theta = target_angle
        self.robot_x = target_x
        self.robot_y = target_y

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ClickToMoveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

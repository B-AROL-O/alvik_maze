#!/usr/bin/env python3
"""
Odometry Publisher Node
Subscribes to MQTT odometry data from Alvik and publishes as ROS2 Odometry messages
Also publishes odom->base_link TF transform required for SLAM
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import paho.mqtt.client as mqtt
import json
import math


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Parameters
        self.declare_parameter('mqtt_host', '192.168.1.152')
        self.declare_parameter('mqtt_port', 1883)

        mqtt_host = self.get_parameter('mqtt_host').value
        mqtt_port = self.get_parameter('mqtt_port').value

        # ROS2 publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Connect to MQTT
        self.mqtt_client = mqtt.Client(client_id="alvik_odom_publisher")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect(mqtt_host, mqtt_port, 60)
        self.mqtt_client.loop_start()

        # Timer for periodic TF publishing (even without odom updates)
        self.create_timer(0.1, self.publish_tf)

        self.get_logger().info(f'Odometry publisher started, connected to MQTT at {mqtt_host}:{mqtt_port}')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when MQTT connection is established"""
        if rc == 0:
            self.get_logger().info('MQTT connected successfully')
            client.subscribe('alvik/odom')
        else:
            self.get_logger().error(f'MQTT connection failed with code {rc}')

    def on_mqtt_message(self, client, userdata, msg):
        """Receive odometry update from Alvik"""
        try:
            if msg.topic == 'alvik/odom':
                data = json.loads(msg.payload.decode())
                self.x = data.get('x', 0.0)
                self.y = data.get('y', 0.0)
                self.theta = data.get('theta', 0.0)

                # Publish ROS2 odometry message
                self.publish_odom()

        except Exception as e:
            self.get_logger().error(f'Error parsing odometry: {e}')

    def publish_odom(self):
        """Publish odometry message"""
        current_time = self.get_clock().now()

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_euler(0, 0, self.theta)

        # Set velocity (not available from Alvik, set to zero)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # Publish
        self.odom_pub.publish(odom)
        self.last_time = current_time

    def publish_tf(self):
        """Publish odom->base_link transform"""
        current_time = self.get_clock().now()

        # Create TF message
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Set rotation
        t.transform.rotation = quaternion_from_euler(0, 0, self.theta)

        # Broadcast TF
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

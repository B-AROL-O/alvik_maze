#!/usr/bin/env python3
"""
ToF Receiver Node - Receives Time-of-Flight sensor data from Alvik via MQTT
and publishes as ROS2 LaserScan messages
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import paho.mqtt.client as mqtt
import math
import json

class AlvikToFReceiver(Node):
    def __init__(self):
        super().__init__('alvik_tof_receiver')
        
        # Declare parameters
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('scan_topic', 'alvik/scan')
        
        # Get parameters
        mqtt_host = self.get_parameter('mqtt_host').value
        mqtt_port = self.get_parameter('mqtt_port').value
        scan_topic = self.get_parameter('scan_topic').value
        
        # ROS2 publishers
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Storage for current scan
        self.current_scan = {}
        self.last_scan_time = self.get_clock().now()
        
        # MQTT client setup
        self.mqtt_client = mqtt.Client(client_id='ros2_alvik_receiver')
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        try:
            self.mqtt_client.connect(mqtt_host, mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f'Connected to MQTT broker at {mqtt_host}:{mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT: {e}')
        
        # Publish TF periodically
        self.timer = self.create_timer(0.1, self.publish_tf)
        
        self.get_logger().info('Alvik ToF Receiver started')
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when MQTT connection is established"""
        if rc == 0:
            self.get_logger().info('MQTT connected successfully')
            client.subscribe('alvik/scan')
            client.subscribe('alvik/scan_complete')
        else:
            self.get_logger().error(f'MQTT connection failed with code {rc}')
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when MQTT disconnects"""
        self.get_logger().warn(f'MQTT disconnected with code {rc}')
    
    def on_mqtt_message(self, client, userdata, msg):
        """Receive individual ToF reading from Alvik"""
        try:
            topic = msg.topic
            
            if topic == 'alvik/scan':
                # Parse "angle,distance" or "angle,(dist1,dist2,...)" message
                data = msg.payload.decode().strip()

                # Split only on first comma to separate angle from distance(s)
                if ',' not in data:
                    return

                angle_str, distance_part = data.split(',', 1)
                angle = float(angle_str)

                # Handle multiple distances: "(7.4, 9.4, 10.2)" or single "500"
                distance_part = distance_part.strip()
                if distance_part.startswith('(') and distance_part.endswith(')'):
                    # Multiple distances - take the average
                    distance_str = distance_part[1:-1]  # Remove parentheses
                    distances = [float(d.strip()) for d in distance_str.split(',')]
                    distance = sum(distances) / len(distances) / 100.0  # Average in meters (cm to m)
                else:
                    # Single distance
                    distance = float(distance_part) / 100.0  # cm to meters

                # Store in current scan
                self.current_scan[angle] = distance
                
            elif topic == 'alvik/scan_complete':
                # Full 360° scan received, publish it
                if len(self.current_scan) > 0:
                    self.publish_scan()
                    self.current_scan = {}  # Reset for next scan
                    
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')
    
    def publish_scan(self):
        """Convert ToF readings to ROS2 LaserScan message"""
        if not self.current_scan:
            return
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'alvik_laser'
        
        # Get sorted angles
        angles = sorted(self.current_scan.keys())
        
        if len(angles) < 2:
            return
        
        # Calculate angle increment (average)
        angle_diffs = [angles[i+1] - angles[i] for i in range(len(angles)-1)]
        avg_increment = sum(angle_diffs) / len(angle_diffs) if angle_diffs else 5.0
        
        scan.angle_min = math.radians(angles[0])
        scan.angle_max = math.radians(angles[-1])
        scan.angle_increment = math.radians(avg_increment)
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = 0.02  # 2cm minimum
        scan.range_max = 2.0   # 2m maximum for ToF
        
        # Build ranges array in order
        scan.ranges = []
        for angle in angles:
            distance = self.current_scan[angle]
            # Filter invalid readings
            if distance < scan.range_min or distance > scan.range_max:
                scan.ranges.append(float('inf'))  # Invalid reading
            else:
                scan.ranges.append(distance)
        
        self.scan_pub.publish(scan)
        self.get_logger().info(f'Published scan with {len(scan.ranges)} points')
        self.last_scan_time = self.get_clock().now()
    
    def publish_tf(self):
        """Publish transform from base_link to laser frame"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'alvik_laser'
        
        # Laser is at the front-center of the robot
        t.transform.translation.x = 0.08  # 8cm forward
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05  # 5cm up
        
        # No rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = AlvikToFReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

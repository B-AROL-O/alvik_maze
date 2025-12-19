#!/usr/bin/env python3
"""
ToF Receiver Node - Receives Time-of-Flight sensor data from Alvik via MQTT
and publishes as ROS2 LaserScan messages
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
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
        # Use sensor QoS with BEST_EFFORT reliability for SLAM compatibility
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.scan_pub = self.create_publisher(LaserScan, 'scan', scan_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Storage for current scan
        self.current_scan = {}
        self.last_scan_time = self.get_clock().now()
        self.scan_publish_rate = 5.0  # Hz - publish scans continuously for SLAM

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
        self.tf_timer = self.create_timer(0.1, self.publish_tf)

        # Publish scans periodically for continuous SLAM
        self.scan_timer = self.create_timer(1.0 / self.scan_publish_rate, self.publish_scan_timer_callback)

        self.get_logger().info('Alvik ToF Receiver started')
        self.get_logger().info(f'Publishing scans at {self.scan_publish_rate} Hz for continuous SLAM')
    
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
        """Receive ToF/Lidar readings from robot"""
        try:
            topic = msg.topic

            if topic == 'alvik/scan':
                data = msg.payload.decode().strip()

                # Handle batched format: "angle1,dist1;angle2,dist2;..."
                if ';' in data:
                    for point in data.split(';'):
                        if ',' not in point:
                            continue
                        angle_str, dist_str = point.split(',', 1)
                        try:
                            angle = float(angle_str)
                            distance = float(dist_str) / 100.0  # cm to meters
                            self.current_scan[angle] = distance
                        except ValueError:
                            continue
                else:
                    # Single point format: "angle,distance" or "angle,(dist1,dist2,...)"
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
                # Full 360° scan received - only publish if we have enough data
                # This prevents partial scans from being published due to timing
                if len(self.current_scan) > 50:
                    self.publish_scan()
                    self.current_scan = {}  # Reset for next 360° scan
                # If not enough data, keep accumulating (don't clear)

        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')

    def publish_scan_timer_callback(self):
        """Publish scan data periodically for continuous SLAM (ToF only)"""
        # Skip entirely - only publish on scan_complete for clean lidar scans
        # This prevents noisy partial scans from interfering
        return
    
    def publish_scan(self):
        """Convert ToF/Lidar readings to ROS2 LaserScan message"""
        if not self.current_scan:
            return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'alvik_laser'

        # Check if this is a full 360° scan (lidar) or partial scan (ToF)
        angles = list(self.current_scan.keys())
        if len(angles) < 2:
            return

        # Normalize angles to 0-360 range for analysis
        normalized_angles = [(a % 360) for a in angles]
        angle_span = max(normalized_angles) - min(normalized_angles)

        # Detect 360° lidar: points spread across most of circle
        # LD06 may have gaps from filtered readings, so lower threshold
        is_full_360 = len(angles) > 50 and angle_span > 270

        if is_full_360:
            # Full 360° lidar scan - create fixed-size scan
            scan.angle_min = 0.0
            scan.angle_max = 2 * math.pi - math.radians(1)  # 0 to 359°
            scan.angle_increment = math.radians(1)  # 1° resolution

            scan.range_min = 0.02   # 2cm minimum
            scan.range_max = 5.0    # 5m maximum - filter outliers

            # Build 360-element ranges array
            # LD06 with cable facing back and triangle forward:
            # - LD06 0° is at cable (= robot's back)
            # - Need 180° rotation + left-right flip
            scan.ranges = []
            for deg in range(360):
                # Combined transform: rotate 180° and flip
                ld06_angle = (180 - deg) % 360

                # Find closest angle in our data
                distance = None
                for angle, dist in self.current_scan.items():
                    angle_normalized = angle % 360
                    diff = abs(angle_normalized - ld06_angle)
                    if diff < 1.5 or diff > 358.5:  # Handle wrap-around
                        distance = dist
                        break

                if distance is not None and scan.range_min <= distance <= scan.range_max:
                    scan.ranges.append(float(distance))
                else:
                    scan.ranges.append(float('inf'))
        else:
            # Partial scan (ToF sensor) - use original logic
            normalized_scan = {}
            for angle, distance in self.current_scan.items():
                normalized_angle = angle
                if normalized_angle > 180:
                    normalized_angle -= 360
                normalized_scan[normalized_angle] = distance

            sorted_angles = sorted(normalized_scan.keys())

            scan.angle_min = math.radians(sorted_angles[0])
            scan.angle_max = math.radians(sorted_angles[-1])

            angle_diffs = [sorted_angles[i+1] - sorted_angles[i] for i in range(len(sorted_angles)-1)]
            avg_increment = sum(angle_diffs) / len(angle_diffs) if angle_diffs else 5.0
            scan.angle_increment = math.radians(avg_increment)

            scan.range_min = 0.02
            scan.range_max = 2.0

            scan.ranges = []
            for angle in sorted_angles:
                distance = normalized_scan[angle]
                if distance < scan.range_min or distance > scan.range_max:
                    scan.ranges.append(float('inf'))
                else:
                    scan.ranges.append(float(distance))

        scan.time_increment = 0.0
        scan.scan_time = 0.0

        self.scan_pub.publish(scan)
        self.get_logger().info(
            f'Published {"360° lidar" if is_full_360 else "ToF"} scan: {len(scan.ranges)} points'
        )
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

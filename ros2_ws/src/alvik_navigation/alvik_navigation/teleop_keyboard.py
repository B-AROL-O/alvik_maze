#!/usr/bin/env python3
"""
Keyboard Teleop Node for Alvik Robot
Control the robot using keyboard:
  w/↑ - Move forward
  s/↓ - Move backward
  a/← - Rotate left
  d/→ - Rotate right
  space - Stop
  q - Quit
"""
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import sys
import termios
import tty
import select


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        # Parameters
        self.declare_parameter('mqtt_host', '192.168.1.152')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('linear_step', 10.0)  # cm per keypress
        self.declare_parameter('angular_step', 15.0)  # degrees per keypress

        mqtt_host = self.get_parameter('mqtt_host').value
        mqtt_port = self.get_parameter('mqtt_port').value
        self.linear_step = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value

        # Connect to MQTT
        self.mqtt_client = mqtt.Client(client_id="alvik_teleop")
        self.mqtt_client.connect(mqtt_host, mqtt_port, 60)
        self.mqtt_client.loop_start()

        # Terminal settings for raw input
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('Keyboard Teleop Node Started!')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Control the Alvik Robot:')
        self.get_logger().info('  w/↑ : Move forward  ({} cm)'.format(self.linear_step))
        self.get_logger().info('  s/↓ : Move backward ({} cm)'.format(self.linear_step))
        self.get_logger().info('  a/← : Rotate left   ({} deg)'.format(self.angular_step))
        self.get_logger().info('  d/→ : Rotate right  ({} deg)'.format(self.angular_step))
        self.get_logger().info('  space : Stop')
        self.get_logger().info('  q : Quit')
        self.get_logger().info('=' * 50)

    def get_key(self):
        """Get a single keypress from stdin"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_move_command(self, distance):
        """Send move command via MQTT"""
        cmd = {
            'type': 'move',
            'distance': distance
        }
        self.mqtt_client.publish('alvik/command', json.dumps(cmd))
        self.get_logger().info('Move: {} cm'.format(distance))

    def send_rotate_command(self, angle):
        """Send rotate command via MQTT"""
        cmd = {
            'type': 'rotate',
            'angle': angle
        }
        self.mqtt_client.publish('alvik/command', json.dumps(cmd))
        self.get_logger().info('Rotate: {} deg'.format(angle))

    def send_stop_command(self):
        """Send stop command via MQTT"""
        cmd = {'type': 'stop'}
        self.mqtt_client.publish('alvik/command', json.dumps(cmd))
        self.get_logger().info('Stop')

    def run(self):
        """Main teleop loop"""
        try:
            while rclpy.ok():
                key = self.get_key()

                if key == 'w' or key == '\x1b[A':  # w or up arrow
                    self.send_move_command(self.linear_step)
                elif key == 's' or key == '\x1b[B':  # s or down arrow
                    self.send_move_command(-self.linear_step)
                elif key == 'a' or key == '\x1b[D':  # a or left arrow
                    self.send_rotate_command(-self.angular_step)
                elif key == 'd' or key == '\x1b[C':  # d or right arrow
                    self.send_rotate_command(self.angular_step)
                elif key == ' ':  # space
                    self.send_stop_command()
                elif key == 'q':  # quit
                    self.get_logger().info('Quitting...')
                    break
                elif key == '\x03':  # Ctrl+C
                    break

        except Exception as e:
            self.get_logger().error('Error: {}'.format(e))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

    def destroy_node(self):
        """Cleanup on shutdown"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

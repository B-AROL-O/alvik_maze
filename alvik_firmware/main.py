"""
Alvik Main Firmware - ROS2 Integration
Connects to MQTT broker and provides scanning and navigation capabilities
Uses asyncio for non-blocking MQTT communication
"""
from arduino_alvik import ArduinoAlvik
from my_secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT
import network
import time
import uasyncio as asyncio
from umqtt.robust import MQTTClient
import json
import math

# Initialize Alvik
alvik = ArduinoAlvik()
alvik.begin()

# Robot state
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

# Global MQTT client
mqtt_client = None
scanning = False

def connect_wifi():
    """Connect to WiFi network"""
    print("Connecting to WiFi...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)

        timeout = 50
        while not wlan.isconnected() and timeout > 0:
            print(".", end="")
            time.sleep(1)
            timeout -= 1

        if wlan.isconnected():
            print("\nWiFi connected!")
            print("IP: {}".format(wlan.ifconfig()[0]))
            return True
        else:
            print("\nWiFi connection failed!")
            return False

    return True

def connect_mqtt():
    """Connect to MQTT broker"""
    print("Connecting to MQTT broker at {}...".format(MQTT_BROKER))

    client = MQTTClient(
        client_id="alvik_robot",
        server=MQTT_BROKER,
        port=MQTT_PORT,
        keepalive=120  # Increased to 120s for better tolerance during blocking operations
    )

    try:
        client.connect()
        print("MQTT connected!")

        # Subscribe to command topic
        client.set_callback(on_mqtt_message)
        client.subscribe(b"alvik/command")

        # Send status
        client.publish(b"alvik/status", b"Alvik connected and ready")

        return client
    except Exception as e:
        print("MQTT connection failed: {}".format(e))
        return None

def on_mqtt_message(topic, msg):
    """Handle incoming MQTT commands"""
    global scanning
    try:
        command = json.loads(msg.decode())
        cmd_type = command.get('type')

        print("Received command: {}".format(cmd_type))

        if cmd_type == 'scan':
            # Schedule async scan task
            if not scanning:
                asyncio.create_task(perform_scan())
            else:
                print("Scan already in progress")

        elif cmd_type == 'move':
            # Move forward/backward
            distance = command.get('distance', 0)
            move_robot(distance)

        elif cmd_type == 'rotate':
            # Rotate in place
            angle = command.get('angle', 0)
            rotate_robot(angle)

        elif cmd_type == 'stop':
            # Emergency stop
            alvik.stop()
            publish_status("Stopped")

    except Exception as e:
        print("Error handling command: {}".format(e))

async def mqtt_keepalive():
    """Background task to keep MQTT connection alive"""
    global mqtt_client
    while True:
        try:
            if mqtt_client:
                mqtt_client.check_msg()
            await asyncio.sleep(0.1)  # Check every 100ms
        except Exception as e:
            print("MQTT keepalive error: {}".format(e))
            await asyncio.sleep(1)

async def continuous_tof_publisher():
    """Background task to continuously publish ToF sensor data"""
    global mqtt_client, robot_theta, scanning

    print("Starting continuous ToF publisher...")

    # Parameters for multi-zone ToF sensor (VL53L5CX)
    FOV_HORIZONTAL = 45.0
    FOV_PER_COLUMN = FOV_HORIZONTAL / 8.0  # 5.625° per column

    while True:
        try:
            # Skip publishing if we're doing a full 360° scan
            if scanning:
                await asyncio.sleep(0.5)
                continue

            # Get current robot angle in degrees
            current_angle = math.degrees(robot_theta) % 360

            # Read ToF sensor
            dist = alvik.get_distance()

            if isinstance(dist, tuple) and len(dist) == 64:
                # Multi-zone data: 8x8 grid
                # Publish all 64 zones with angle offsets
                for zone_idx, zone_dist in enumerate(dist):
                    if zone_dist > 0:  # Valid reading
                        row = zone_idx // 8  # 0-7 (top to bottom)
                        col = zone_idx % 8   # 0-7 (left to right)

                        # Calculate horizontal angle offset for this zone
                        col_angle_offset = (col - 3.5) * FOV_PER_COLUMN

                        # Total angle = robot angle + zone offset
                        zone_angle = (current_angle + col_angle_offset) % 360

                        # Publish individual zone reading
                        message = "{:.2f},{}".format(zone_angle, zone_dist)
                        mqtt_client.publish(b"alvik/scan", message.encode())

            elif isinstance(dist, tuple):
                # Multiple zones but not 64 - average them
                valid_dists = [d for d in dist if d > 0]
                if valid_dists:
                    avg_distance = sum(valid_dists) / len(valid_dists)
                    message = "{},{}".format(current_angle, avg_distance)
                    mqtt_client.publish(b"alvik/scan", message.encode())
            else:
                # Single distance reading
                if dist > 0:
                    message = "{},{}".format(current_angle, dist)
                    mqtt_client.publish(b"alvik/scan", message.encode())

            # Signal scan complete to trigger ROS2 publish
            mqtt_client.publish(b"alvik/scan_complete", b"1")

            # Publish at ~5 Hz (reasonable rate for continuous scanning)
            await asyncio.sleep(0.2)

        except Exception as e:
            print("ToF publisher error: {}".format(e))
            await asyncio.sleep(1)

async def perform_scan():
    """Perform 360° scan with encoder-controlled rotation"""
    global mqtt_client, scanning

    scanning = True
    print("Starting encoder-controlled 360° scan...")
    publish_status("Scanning...")

    # Scan parameters
    num_readings = 360   # One reading per degree
    readings_sent = 0
    rotation_speed = 30  # Motor speed for continuous rotation

    # Robot wheel parameters for encoder calculation
    WHEEL_DIAMETER_CM = 3.9  # Alvik wheel diameter
    WHEEL_BASE_CM = 14.3     # Distance between wheel centers
    DEGREES_PER_ROTATION = 360.0
    ROTATION_CALIBRATION = 0.73  # Calibration factor (actual 360° / measured rotation)

    # Calculate target encoder degrees for 360° robot rotation
    # For rotation: arc_length = (360° / 360°) * pi * wheel_base = pi * wheel_base
    arc_length_cm = 3.14159 * WHEEL_BASE_CM
    wheel_rotations = arc_length_cm / (3.14159 * WHEEL_DIAMETER_CM)
    target_encoder_degrees = wheel_rotations * DEGREES_PER_ROTATION * ROTATION_CALIBRATION

    print("Target encoder degrees: {:.1f}".format(target_encoder_degrees))

    try:
        # CRITICAL: Reset motor control mode
        # Ensure motors are in velocity control mode and ready to respond
        # Multiple stop commands help reset any stuck state
        alvik.set_wheels_speed(0, 0)
        await asyncio.sleep(0.3)
        alvik.set_wheels_speed(0, 0)  # Double-stop to force reset
        await asyncio.sleep(0.3)
        alvik.set_wheels_speed(0, 0)  # Triple-stop for reliability
        await asyncio.sleep(0.5)  # Extended settle time

        # Get starting encoder positions
        left_start, right_start = alvik.get_wheels_position()
        print("Starting encoders: L={:.1f}, R={:.1f}".format(left_start, right_start))

        # Start continuous rotation (clockwise)
        alvik.set_wheels_speed(-rotation_speed, rotation_speed)
        print("Started continuous rotation at speed {}".format(rotation_speed))
        await asyncio.sleep(0.1)  # Give motors time to start

        scan_start = time.time()
        rotation_complete = False
        final_angle = 0.0

        for i in range(num_readings):
            # Get REAL-TIME angle from encoders (not iteration number!)
            try:
                left_pos, right_pos = alvik.get_wheels_position()
                left_delta = abs(left_pos - left_start)
                right_delta = abs(right_pos - right_start)
                avg_encoder_delta = (left_delta + right_delta) / 2.0

                # Convert encoder degrees to robot rotation angle
                # wheel_rotations * 360° but accounting for wheel base
                actual_rotation = (avg_encoder_delta / target_encoder_degrees) * 360.0
                angle = actual_rotation % 360  # Current real angle
                final_angle = actual_rotation  # Track total rotation (may exceed 360)

                # Check if we've completed 360°
                if not rotation_complete and actual_rotation >= 360.0:
                    alvik.set_wheels_speed(0, 0)
                    rotation_complete = True
                    elapsed = time.time() - scan_start
                    print("Rotation complete at {:.2f}s (actual: {:.1f}°)".format(elapsed, actual_rotation))
                    # Break loop - we've completed the scan
                    break

            except Exception as e:
                print("Encoder read error: {}".format(e))
                # Fallback to iteration-based angle if encoder fails
                angle = i

            # Get ToF distance (allow sensor time to measure)
            await asyncio.sleep(0.03)  # 30ms delay for ToF to take fresh reading
            dist = alvik.get_distance()

            try:
                # VL53L5CX returns 8x8=64 zone array with ~45° FOV
                # Publish all zones individually with angle offsets
                if isinstance(dist, tuple) and len(dist) == 64:
                    # Multi-zone data: 8x8 grid
                    # FOV: 45° total, so each column is ~5.625° wide
                    # Columns 0-7 span from -22.5° to +22.5° (left to right)
                    FOV_HORIZONTAL = 45.0
                    FOV_PER_COLUMN = FOV_HORIZONTAL / 8.0  # 5.625°

                    for zone_idx, zone_dist in enumerate(dist):
                        if zone_dist > 0:  # Valid reading
                            # Calculate zone position in 8x8 grid
                            row = zone_idx // 8  # 0-7 (top to bottom)
                            col = zone_idx % 8   # 0-7 (left to right)

                            # Calculate horizontal angle offset for this zone
                            # Column 0 = -22.5°, Column 4 = 0°, Column 7 = +22.5°
                            col_angle_offset = (col - 3.5) * FOV_PER_COLUMN

                            # Total angle = robot angle + zone offset
                            zone_angle = (angle + col_angle_offset) % 360

                            # Publish individual zone reading
                            message = "{:.2f},{}".format(zone_angle, zone_dist)
                            mqtt_client.publish(b"alvik/scan", message.encode())
                            readings_sent += 1

                elif isinstance(dist, tuple):
                    # Fallback: average if not exactly 64 zones
                    valid_dists = [d for d in dist if d > 0]
                    if valid_dists:
                        avg_distance = sum(valid_dists) / len(valid_dists)
                        message = "{},{}".format(angle, avg_distance)
                        mqtt_client.publish(b"alvik/scan", message.encode())
                        readings_sent += 1
                else:
                    # Single value
                    if dist > 0:
                        message = "{},{}".format(angle, dist)
                        mqtt_client.publish(b"alvik/scan", message.encode())
                        readings_sent += 1
            except Exception as e:
                if i % 60 == 0:
                    print("Publish error at {}°: {}".format(angle, e))

            # Update odometry during scan (robot is rotating in place)
            # Rotation in place only changes theta, not x/y position
            robot_theta = math.radians(angle)

            # Publish odometry every 10 readings (every ~10 degrees)
            if i % 10 == 0:
                publish_odometry()

            # Print progress every 60 degrees
            if i % 60 == 0:
                try:
                    curr_left, curr_right = alvik.get_wheels_position()
                    print("Progress: {:.1f}° ({}/360) | Encoders: L={:.1f}, R={:.1f} | Delta: {:.1f}".format(
                        angle, i, curr_left, curr_right, avg_encoder_delta))
                except:
                    print("Progress: {:.1f}° ({}/360)".format(angle, i))

            # Yield to event loop
            await asyncio.sleep(0.01)

        # Ensure rotation stopped
        alvik.set_wheels_speed(0, 0)
        total_time = time.time() - scan_start
        print("Scan completed in {:.2f}s".format(total_time))

        # Correction: rotate back if we overshot 360°
        overshoot = final_angle - 360.0
        if abs(overshoot) > 0.5:  # Only correct if overshoot > 0.5°
            print("Correcting overshoot: {:.1f}°".format(overshoot))
            await asyncio.sleep(0.2)  # Let motors settle

            # Calculate encoder distance needed to correct overshoot
            # Use velocity control (set_wheels_speed) to stay in velocity mode
            overshoot_encoder_delta = (overshoot / 360.0) * target_encoder_degrees
            correction_start_left, correction_start_right = alvik.get_wheels_position()

            # Rotate backwards at slow speed (opposite of scan direction)
            correction_speed = 15  # Slow speed for precision
            if overshoot > 0:
                # Overshot forward, rotate backwards (reverse scan direction)
                alvik.set_wheels_speed(correction_speed, -correction_speed)
            else:
                # Undershot, rotate forwards (same as scan direction)
                alvik.set_wheels_speed(-correction_speed, correction_speed)

            # Monitor encoders until we've rotated back by overshoot amount
            correction_complete = False
            max_correction_time = 3.0  # Safety timeout
            correction_start_time = time.time()

            while not correction_complete and (time.time() - correction_start_time) < max_correction_time:
                curr_left, curr_right = alvik.get_wheels_position()
                left_moved = abs(curr_left - correction_start_left)
                right_moved = abs(curr_right - correction_start_right)
                avg_moved = (left_moved + right_moved) / 2.0

                if avg_moved >= abs(overshoot_encoder_delta):
                    alvik.set_wheels_speed(0, 0)
                    correction_complete = True
                    print("Position corrected to 360°")

                await asyncio.sleep(0.01)

            if not correction_complete:
                print("Correction timeout - partial correction applied")
                alvik.set_wheels_speed(0, 0)

    except Exception as e:
        print("Scan error: {}".format(e))
        alvik.set_wheels_speed(0, 0)

    finally:
        # Stop any motion - use set_wheels_speed to keep encoders active
        alvik.set_wheels_speed(0, 0)

        # Publish scan complete signal
        try:
            mqtt_client.publish(b"alvik/scan_complete", b"1")
            print("Scan complete! Sent {}/{} readings".format(readings_sent, num_readings))
        except Exception as e:
            print("Error publishing scan complete: {}".format(e))

        publish_status("Scan complete")
        scanning = False

def move_robot(distance_cm):
    """Move robot forward/backward by specified distance in cm"""
    global robot_x, robot_y, robot_theta

    print("Moving {}cm...".format(distance_cm))

    # Move the robot
    alvik.move(distance_cm)

    # Update odometry (simple dead reckoning)
    distance_m = distance_cm / 100.0
    robot_x += distance_m * math.cos(robot_theta)
    robot_y += distance_m * math.sin(robot_theta)

    publish_odometry()
    publish_status("Moved {}cm".format(distance_cm))

def rotate_robot(angle_deg):
    """Rotate robot by specified angle in degrees"""
    global robot_theta

    print("Rotating {}°...".format(angle_deg))

    # Rotate the robot
    alvik.rotate(angle_deg)

    # Update odometry
    robot_theta += math.radians(angle_deg)

    # Normalize angle to [-pi, pi]
    while robot_theta > math.pi:
        robot_theta -= 2 * math.pi
    while robot_theta < -math.pi:
        robot_theta += 2 * math.pi

    publish_odometry()
    publish_status("Rotated {}°".format(angle_deg))

def publish_odometry():
    """Publish current robot position"""
    global robot_x, robot_y, robot_theta, mqtt_client

    odom_data = {
        "x": round(robot_x, 3),
        "y": round(robot_y, 3),
        "theta": round(robot_theta, 3)
    }

    mqtt_client.publish(b"alvik/odom", json.dumps(odom_data).encode())

def publish_status(status):
    """Publish status message"""
    global mqtt_client
    mqtt_client.publish(b"alvik/status", status.encode())

async def continuous_odom_publisher():
    """Background task to continuously publish odometry"""
    global mqtt_client

    while True:
        try:
            publish_odometry()
            # Publish odometry at 10 Hz
            await asyncio.sleep(0.1)
        except Exception as e:
            print("Odometry publisher error: {}".format(e))
            await asyncio.sleep(1)

async def main_loop():
    """Main program loop using asyncio"""
    global mqtt_client

    print("Alvik ROS2 Integration Starting...")

    # Connect to WiFi
    if not connect_wifi():
        print("Failed to connect to WiFi. Please check credentials.")
        return

    # Connect to MQTT
    mqtt_client = connect_mqtt()
    if not mqtt_client:
        print("Failed to connect to MQTT. Please check broker address.")
        return

    print("\nAlvik ready! Waiting for commands...")
    print("Robot at position: ({}, {}), angle: {}°".format(robot_x, robot_y, math.degrees(robot_theta)))

    # Publish initial odometry
    publish_odometry()

    # Start background tasks
    asyncio.create_task(mqtt_keepalive())
    asyncio.create_task(continuous_tof_publisher())
    asyncio.create_task(continuous_odom_publisher())

    print("Background tasks started: MQTT keepalive, ToF publisher, Odometry publisher")

    # Main loop - just keep the event loop alive
    try:
        while True:
            await asyncio.sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down...")
        alvik.stop()
        mqtt_client.disconnect()

# Start the program
if __name__ == "__main__":
    try:
        asyncio.run(main_loop())
    except KeyboardInterrupt:
        print("\nProgram terminated")

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped, Point
from tf2_ros import TransformBroadcaster
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64
from geometry_msgs.msg import InertiaStamped, AccelStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
import socket
import struct
import math
import json

# TelemetryPacket matching C++ definition
# uint64_t timestamp_us
# 9x double (pos, vel, acc)
# 7x double (quat, ang_vel)
# 1x double (mass)
# 3x double (thrust)
# 3x double (aero_force)
# 3x double (inertia)
# 3x double (wind)
# 4x double (tvc)
# Total size: 8 + 34*8 = 280 bytes
PACKET_FORMAT = '<Q34d'
PACKET_SIZE = 280

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        
        self.odom_pub = self.create_publisher(Odometry, 'rocket/odometry', 1000)
        self.accel_pub = self.create_publisher(AccelStamped, 'rocket/acceleration', 1000)
        self.gps_pub = self.create_publisher(NavSatFix, 'rocket/gps', 1000)
        
        self.tvc_cmd_pitch_pub = self.create_publisher(Float64, 'rocket/tvc/cmd_pitch', 1000)
        self.tvc_cmd_yaw_pub = self.create_publisher(Float64, 'rocket/tvc/cmd_yaw', 1000)
        self.tvc_error_pitch_pub = self.create_publisher(Float64, 'rocket/tvc/error_pitch', 1000)
        self.tvc_error_yaw_pub = self.create_publisher(Float64, 'rocket/tvc/error_yaw', 1000)
        
        self.mass_pub = self.create_publisher(Float64, 'rocket/mass', 1000)
        self.inertia_pub = self.create_publisher(InertiaStamped, 'rocket/inertia', 1000)
        self.thrust_pub = self.create_publisher(WrenchStamped, 'rocket/thrust', 1000)
        self.aero_pub = self.create_publisher(WrenchStamped, 'rocket/aero_forces', 1000)
        
        self.body_marker_pub = self.create_publisher(Marker, 'rocket/visuals/body', 1000)
        self.thrust_marker_pub = self.create_publisher(Marker, 'rocket/visuals/thrust', 1000)
        self.aero_marker_pub = self.create_publisher(Marker, 'rocket/visuals/aero', 1000)
        self.wind_marker_pub = self.create_publisher(Marker, 'rocket/visuals/wind', 1000)
        
        self.cg_marker_pub = self.create_publisher(Marker, 'rocket/visuals/cg', 1000)
        self.cop_marker_pub = self.create_publisher(Marker, 'rocket/visuals/cop', 1000)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.rocket_length = 2.0
        self.rocket_radius = 0.1
        self.cop_z = -1.2
        self.engine_pos_z = -2.0
        
        # Default Location (London)
        self.start_lat = 51.5074
        self.start_lon = -0.1278
        self.start_alt = 100.0

        try:
            with open('config.json', 'r') as f:
                cfg = json.load(f)
                if 'visuals' in cfg:
                    self.rocket_length = cfg['visuals'].get('rocket_length_m', 2.0)
                    self.rocket_radius = cfg['visuals'].get('rocket_radius_m', 0.1)
                if 'rocket' in cfg:
                    if 'aerodynamics' in cfg['rocket']:
                        self.cop_z = cfg['rocket']['aerodynamics'].get('center_of_pressure_z_m', -1.2)
                    if 'engine' in cfg['rocket']:
                        self.engine_pos_z = cfg['rocket']['engine'].get('engine_position_z_m', -2.0)
                if 'location' in cfg:
                    self.start_lat = cfg['location'].get('latitude', 51.5074)
                    self.start_lon = cfg['location'].get('longitude', -0.1278)
                    self.start_alt = cfg['location'].get('altitude_m', 100.0)
        except Exception as e:
            self.get_logger().warn(f"Could not load config.json, using default visuals: {e}")

        self.packets_received = 0
        self.base_time_ns = None

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 9876))
        self.sock.setblocking(False)

        # Polling timer (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Telemetry bridge started. Listening on UDP 9876...")

    def timer_callback(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(2048)
                if len(data) == PACKET_SIZE:
                    self.process_packet(data)
                else:
                    self.get_logger().warn(f"Received packet of size {len(data)}, expected {PACKET_SIZE}")
        except BlockingIOError:
            pass

    def process_packet(self, data):
        unpacked = struct.unpack(PACKET_FORMAT, data)
        timestamp_us = unpacked[0]
        
        # Unpack translations
        pos_x, pos_y, pos_z = unpacked[1:4]
        vel_x, vel_y, vel_z = unpacked[4:7]
        acc_x, acc_y, acc_z = unpacked[7:10]
        
        # Unpack rotations
        quat_w, quat_x, quat_y, quat_z = unpacked[10:14]
        ang_vel_x, ang_vel_y, ang_vel_z = unpacked[14:17]

        # Unpack diagnostics
        mass_kg = unpacked[17]
        cg_z = unpacked[18]
        thrust_x, thrust_y, thrust_z = unpacked[19:22]
        aero_x, aero_y, aero_z = unpacked[22:25]
        inertia_x, inertia_y, inertia_z = unpacked[25:28]
        wind_x, wind_y, wind_z = unpacked[28:31]
        
        # Unpack TVC diagnostics
        tvc_cmd_pitch, tvc_cmd_yaw, tvc_err_pitch, tvc_err_yaw = unpacked[31:35]

        # Sync simulation time with real-world time to avoid 1970 epoch issues in Foxglove
        now_ns = self.get_clock().now().nanoseconds
        if self.base_time_ns is None:
            self.base_time_ns = now_ns - (timestamp_us * 1000)
            self.get_logger().info("Synchronized simulation time with system epoch.")

        target_time_ns = self.base_time_ns + (timestamp_us * 1000)
        
        import rclpy.time
        sim_time = rclpy.time.Time(nanoseconds=target_time_ns).to_msg()
        
        # Publish Odometry (Pose + Twist)
        odom = Odometry()
        odom.header.stamp = sim_time
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'rocket'
        
        odom.pose.pose.position.x = pos_x
        odom.pose.pose.position.y = pos_y
        odom.pose.pose.position.z = pos_z
        odom.pose.pose.orientation.w = quat_w
        odom.pose.pose.orientation.x = quat_x
        odom.pose.pose.orientation.y = quat_y
        odom.pose.pose.orientation.z = quat_z
        
        # Assuming velocities in inertial frame, but typically child_frame_id twist is in child_frame.
        # Since Foxglove generally handles it well when frame_id is world and child is rocket:
        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = vel_y
        odom.twist.twist.linear.z = vel_z
        odom.twist.twist.angular.x = ang_vel_x
        odom.twist.twist.angular.y = ang_vel_y
        odom.twist.twist.angular.z = ang_vel_z
        self.odom_pub.publish(odom)

        # Publish Acceleration
        accel = AccelStamped()
        accel.header.stamp = sim_time
        accel.header.frame_id = 'world'
        accel.accel.linear.x = acc_x
        accel.accel.linear.y = acc_y
        accel.accel.linear.z = acc_z
        self.accel_pub.publish(accel)

        # Publish GPS (NavSatFix)
        gps = NavSatFix()
        gps.header.stamp = sim_time
        gps.header.frame_id = 'world'
        
        # Flat Earth approximation
        # 1 degree of latitude is ~111,320 meters
        # 1 degree of longitude is ~111,320 * cos(latitude) meters
        lat_conversion = 1.0 / 111320.0
        lon_conversion = 1.0 / (111320.0 * math.cos(math.radians(self.start_lat)))
        
        gps.latitude = self.start_lat + (pos_y * lat_conversion)  # Y is North
        gps.longitude = self.start_lon + (pos_x * lon_conversion) # X is East
        gps.altitude = self.start_alt + pos_z                     # Z is Up
        self.gps_pub.publish(gps)
        
        # Publish TVC Diagnostics
        msg = Float64()
        msg.data = tvc_cmd_pitch
        self.tvc_cmd_pitch_pub.publish(msg)
        msg.data = tvc_cmd_yaw
        self.tvc_cmd_yaw_pub.publish(msg)
        msg.data = tvc_err_pitch
        self.tvc_error_pitch_pub.publish(msg)
        msg.data = tvc_err_yaw
        self.tvc_error_yaw_pub.publish(msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = sim_time
        t.header.frame_id = 'world'
        t.child_frame_id = 'rocket'
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = pos_z
        t.transform.rotation.w = quat_w
        t.transform.rotation.x = quat_x
        t.transform.rotation.y = quat_y
        t.transform.rotation.z = quat_z
        self.tf_broadcaster.sendTransform(t)

        # Publish Mass
        mass_msg = Float64()
        mass_msg.data = mass_kg
        self.mass_pub.publish(mass_msg)

        # Publish Inertia
        inertia_msg = InertiaStamped()
        inertia_msg.header.stamp = sim_time
        inertia_msg.header.frame_id = 'rocket'
        inertia_msg.inertia.m = mass_kg
        inertia_msg.inertia.ixx = inertia_x
        inertia_msg.inertia.iyy = inertia_y
        inertia_msg.inertia.izz = inertia_z
        self.inertia_pub.publish(inertia_msg)

        # Publish Thrust
        thrust_msg = WrenchStamped()
        thrust_msg.header.stamp = sim_time
        thrust_msg.header.frame_id = 'rocket' # Wrench is relative to the rocket body
        thrust_msg.wrench.force.x = thrust_x
        thrust_msg.wrench.force.y = thrust_y
        thrust_msg.wrench.force.z = thrust_z
        self.thrust_pub.publish(thrust_msg)

        # Publish Aero Forces
        aero_msg = WrenchStamped()
        aero_msg.header.stamp = sim_time
        aero_msg.header.frame_id = 'rocket'
        aero_msg.wrench.force.x = aero_x
        aero_msg.wrench.force.y = aero_y
        aero_msg.wrench.force.z = aero_z
        self.aero_pub.publish(aero_msg)

        # Publish Visual Markers (independently)

        # 1. Rocket Cylinder Marker
        rocket_marker = Marker()
        rocket_marker.header.stamp = sim_time
        rocket_marker.header.frame_id = 'rocket'
        rocket_marker.ns = 'rocket_body'
        rocket_marker.id = 0
        rocket_marker.type = Marker.CYLINDER
        rocket_marker.action = Marker.ADD
        rocket_marker.pose.position.x = 0.0
        rocket_marker.pose.position.y = 0.0
        rocket_marker.pose.position.z = -self.rocket_length / 2.0
        rocket_marker.pose.orientation.w = 1.0
        rocket_marker.scale.x = self.rocket_radius * 2
        rocket_marker.scale.y = self.rocket_radius * 2
        rocket_marker.scale.z = self.rocket_length
        rocket_marker.color.r = 0.7
        rocket_marker.color.g = 0.7
        rocket_marker.color.b = 0.7
        rocket_marker.color.a = 0.5 # Semi-transparent
        self.body_marker_pub.publish(rocket_marker)

        # 2. Thrust Arrow Marker
        thrust_len = math.sqrt(thrust_x**2 + thrust_y**2 + thrust_z**2)
        if thrust_len > 0.1:
            thrust_marker = Marker()
            thrust_marker.header.stamp = sim_time
            thrust_marker.header.frame_id = 'rocket'
            thrust_marker.ns = 'forces'
            thrust_marker.id = 1
            thrust_marker.type = Marker.ARROW
            thrust_marker.action = Marker.ADD
            
            # Scale for forces
            scale_factor = 0.002 
            
            # Arrow start: engine position
            p_start = Point()
            p_start.x = 0.0
            p_start.y = 0.0
            p_start.z = self.engine_pos_z
            
            # Arrow end: inverted vector (pointing down like exhaust)
            p_end = Point()
            p_end.x = p_start.x - thrust_x * scale_factor
            p_end.y = p_start.y - thrust_y * scale_factor
            p_end.z = p_start.z - thrust_z * scale_factor
            
            # Render only if arrow has meaningful length
            dist = math.sqrt((p_end.x - p_start.x)**2 + (p_end.y - p_start.y)**2 + (p_end.z - p_start.z)**2)
            if dist > 0.15:
                thrust_marker.points = [p_start, p_end]
                thrust_marker.scale.x = 0.05 # shaft diameter
                thrust_marker.scale.y = 0.15 # head diameter
                thrust_marker.scale.z = 0.15 # head length
                # Pure red
                thrust_marker.color.r = 1.0
                thrust_marker.color.g = 0.0
                thrust_marker.color.b = 0.0
                thrust_marker.color.a = 0.9
                self.thrust_marker_pub.publish(thrust_marker)

        # 3. Aero Arrow Marker
        aero_len = math.sqrt(aero_x**2 + aero_y**2 + aero_z**2)
        if aero_len > 0.1:
            aero_marker = Marker()
            aero_marker.header.stamp = sim_time
            aero_marker.header.frame_id = 'rocket'
            aero_marker.ns = 'forces'
            aero_marker.id = 2
            aero_marker.type = Marker.ARROW
            aero_marker.action = Marker.ADD
            
            scale_factor = 0.005 # Aero usually weaker than thrust, boost scale slightly
            
            # Arrow start: Center of Pressure (CoP)
            p_start = Point()
            p_start.x = 0.0
            p_start.y = 0.0
            p_start.z = self.cop_z
            
            # Arrow end: inverted vector (drag pulls down, so inverted points up)
            p_end = Point()
            p_end.x = p_start.x - aero_x * scale_factor
            p_end.y = p_start.y - aero_y * scale_factor
            p_end.z = p_start.z - aero_z * scale_factor
            
            dist = math.sqrt((p_end.x - p_start.x)**2 + (p_end.y - p_start.y)**2 + (p_end.z - p_start.z)**2)
            if dist > 0.15:
                aero_marker.points = [p_start, p_end]
                aero_marker.scale.x = 0.05
                aero_marker.scale.y = 0.15
                aero_marker.scale.z = 0.15
                # Pure blue
                aero_marker.color.r = 0.0
                aero_marker.color.g = 0.0
                aero_marker.color.b = 1.0
                aero_marker.color.a = 0.9
                self.aero_marker_pub.publish(aero_marker)

        # 3.5 Wind Arrow Marker
        wind_len = math.sqrt(wind_x**2 + wind_y**2 + wind_z**2)
        if wind_len > 0.1:
            wind_marker = Marker()
            wind_marker.header.stamp = sim_time
            wind_marker.header.frame_id = 'world' # Wind is in inertial frame
            wind_marker.ns = 'environment'
            wind_marker.id = 5
            wind_marker.type = Marker.ARROW
            wind_marker.action = Marker.ADD
            
            # Start far away to point AT the rocket, or start AT the rocket pointing away?
            # Let's start AT the rocket and point in the direction of the wind
            p_start = Point()
            p_start.x = pos_x
            p_start.y = pos_y
            p_start.z = pos_z
            
            p_end = Point()
            p_end.x = pos_x + wind_x
            p_end.y = pos_y + wind_y
            p_end.z = pos_z + wind_z
            
            wind_marker.points = [p_start, p_end]
            wind_marker.scale.x = 0.05
            wind_marker.scale.y = 0.15
            wind_marker.scale.z = 0.15
            # Cyan for wind
            wind_marker.color.r = 0.0
            wind_marker.color.g = 1.0
            wind_marker.color.b = 1.0
            wind_marker.color.a = 0.6
            self.wind_marker_pub.publish(wind_marker)

        # 4. Center of Gravity (CG) Marker
        cg_marker = Marker()
        cg_marker.header.stamp = sim_time
        cg_marker.header.frame_id = 'rocket'
        cg_marker.ns = 'points'
        cg_marker.id = 3
        cg_marker.type = Marker.SPHERE
        cg_marker.action = Marker.ADD
        cg_marker.pose.position.x = 0.0
        cg_marker.pose.position.y = 0.0
        cg_marker.pose.position.z = cg_z
        cg_marker.pose.orientation.w = 1.0
        cg_marker.scale.x = 0.2
        cg_marker.scale.y = 0.2
        cg_marker.scale.z = 0.2
        cg_marker.color.r = 0.0
        cg_marker.color.g = 1.0
        cg_marker.color.b = 0.0
        cg_marker.color.a = 1.0 # Green
        self.cg_marker_pub.publish(cg_marker)

        # 5. Center of Pressure (CoP) Marker
        cop_marker = Marker()
        cop_marker.header.stamp = sim_time
        cop_marker.header.frame_id = 'rocket'
        cop_marker.ns = 'points'
        cop_marker.id = 4
        cop_marker.type = Marker.SPHERE
        cop_marker.action = Marker.ADD
        cop_marker.pose.position.x = 0.0
        cop_marker.pose.position.y = 0.0
        cop_marker.pose.position.z = self.cop_z
        cop_marker.pose.orientation.w = 1.0
        cop_marker.scale.x = 0.2
        cop_marker.scale.y = 0.2
        cop_marker.scale.z = 0.2
        cop_marker.color.r = 1.0
        cop_marker.color.g = 1.0
        cop_marker.color.b = 0.0
        cop_marker.color.a = 1.0 # Yellow
        self.cop_marker_pub.publish(cop_marker)

        self.packets_received += 1
        if self.packets_received % 100 == 1:
            self.get_logger().info(f"Forwarded packet {self.packets_received}, z={pos_z:.2f}m")


def main(args=None):
    rclpy.init(args=args)
    bridge = TelemetryBridge()
    try:
        rclpy.spin(bridge)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        bridge.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

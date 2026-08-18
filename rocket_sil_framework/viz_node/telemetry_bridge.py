#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped, Point
from tf2_ros import TransformBroadcaster
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64
from geometry_msgs.msg import InertiaStamped, AccelStamped, PointStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from visualization_msgs.msg import Marker, MarkerArray
import socket
import struct
import math
import json
import threading
import queue
import os

# TelemetryPacket matching C++ definition
# TelemetryPacket matching C++ definition
PACKET_HEADER_FORMAT = '<Q43dI'
PACKET_HEADER_SIZE = 356
ENGINE_STRUCT_FORMAT = '<3d'
ENGINE_STRUCT_SIZE = 24

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        
        # Clock publisher for simulated time
        self.clock_pub = self.create_publisher(Clock, '/clock', 100)
        
        self.odom_pub = self.create_publisher(Odometry, 'rocket/odometry', 1000)
        self.accel_pub = self.create_publisher(AccelStamped, 'rocket/acceleration', 1000)
        self.gps_pub = self.create_publisher(NavSatFix, 'rocket/gps', 1000)
        
        # Noisy Sensor Publishers
        self.noisy_imu_pub = self.create_publisher(Imu, 'rocket/sensors/imu', 1000)
        self.noisy_gps_pub = self.create_publisher(NavSatFix, 'rocket/sensors/gps', 1000)
        
        # Rocket TVC
        self.tvc_cmd_pitch_pub = self.create_publisher(PointStamped, '/rocket/tvc/cmd_pitch', 1000)
        self.tvc_cmd_yaw_pub = self.create_publisher(PointStamped, '/rocket/tvc/cmd_yaw', 1000)
        self.tvc_err_pitch_pub = self.create_publisher(PointStamped, '/rocket/tvc/err_pitch', 1000)
        self.tvc_err_yaw_pub = self.create_publisher(PointStamped, '/rocket/tvc/err_yaw', 1000)
        self.mass_pub = self.create_publisher(PointStamped, 'rocket/mass', 1000)
        
        # Velocity explicitly for Foxglove charts
        self.vel_pub = self.create_publisher(PointStamped, 'rocket/velocity', 1000)
        self.vel_mag_pub = self.create_publisher(PointStamped, 'rocket/velocity_mag', 1000)
        
        self.inertia_pub = self.create_publisher(InertiaStamped, 'rocket/inertia', 1000)
        self.thrust_pub = self.create_publisher(WrenchStamped, 'rocket/thrust', 1000)
        self.aero_pub = self.create_publisher(WrenchStamped, 'rocket/aero_forces', 1000)
        
        self.body_marker_pub = self.create_publisher(Marker, 'rocket/visuals/body', 1000)
        self.thrust_marker_pub = self.create_publisher(MarkerArray, 'rocket/visuals/thrust_array', 1000)
        self.aero_marker_pub = self.create_publisher(Marker, 'rocket/visuals/aero', 1000)
        self.wind_marker_pub = self.create_publisher(Marker, 'rocket/visuals/wind', 1000)
        
        self.cg_marker_pub = self.create_publisher(Marker, 'rocket/visuals/cg', 1000)
        self.cop_marker_pub = self.create_publisher(Marker, 'rocket/visuals/cop', 1000)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.rocket_length = 2.0
        self.rocket_radius = 0.1
        self.cop_z = -1.2
        self.engine_positions = []
        
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
                    if 'actuators' in cfg['rocket']:
                        for act in cfg['rocket']['actuators']:
                            self.engine_positions.append(act['position_m'][2]) # Z pos
                if 'location' in cfg:
                    self.start_lat = cfg['location'].get('latitude', 51.5074)
                    self.start_lon = cfg['location'].get('longitude', -0.1278)
                    self.start_alt = cfg['location'].get('altitude_m', 100.0)
        except Exception as e:
            self.get_logger().warn(f"Could not load config.json, using default visuals: {e}")

        # Resolve absolute path to the generated OBJ file once
        # Use host path if provided by orchestrator, otherwise fallback to container path
        host_path = os.environ.get('HOST_WORKSPACE_PATH')
        if host_path:
            workspace_dir = host_path
        else:
            workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        self.mesh_path = os.path.join(workspace_dir, "rocket_sil_framework", "meshes", "rocket_z_up.obj")

        self.packets_received = 0
        self.base_time_ns = None

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Increase receive buffer to 10MB to avoid dropping bursts
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 10 * 1024 * 1024)
        self.sock.bind(('0.0.0.0', 9876))
        # Blocking socket for the dedicated thread
        self.sock.setblocking(True)

        self.udp_queue = queue.Queue()

        # Start dedicated UDP receiver thread
        self.recv_thread = threading.Thread(target=self.udp_receive_thread, daemon=True)
        self.recv_thread.start()

        # Polling timer to process queue and publish to ROS2
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Telemetry bridge started. Listening on UDP 9876...")

    def udp_receive_thread(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                if len(data) >= PACKET_HEADER_SIZE:
                    self.udp_queue.put(data)
                else:
                    self.get_logger().warn(f"Received packet of size {len(data)}, expected >= {PACKET_HEADER_SIZE}")
            except Exception as e:
                self.get_logger().error(f"UDP receive error: {e}")

    def timer_callback(self):
        if getattr(self, 'eof_received', False):
            return

        # Drain the queue and publish
        # Limit to 50 packets per timer tick to prevent overwhelming the TF Broadcaster's depth=100 QoS queue
        processed = 0
        while not self.udp_queue.empty() and processed < 50:
            try:
                data = self.udp_queue.get_nowait()
                self.process_packet(data)
                processed += 1
            except queue.Empty:
                break

    def process_packet(self, data):
        header_data = data[:PACKET_HEADER_SIZE]
        unpacked = struct.unpack(PACKET_HEADER_FORMAT, header_data)
        timestamp_us = unpacked[0]
        
        if timestamp_us == 0xFFFFFFFFFFFFFFFF:
            self.get_logger().info("Received End of Simulation marker. Shutting down bridge gracefully.")
            self.eof_received = True
            return
            
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
        
        # Unpack Noisy Sensors
        imu_gyro_x, imu_gyro_y, imu_gyro_z = unpacked[35:38]
        imu_acc_x, imu_acc_y, imu_acc_z = unpacked[38:41]
        gps_lat, gps_lon, gps_alt = unpacked[41:44]
        
        num_engines = unpacked[44]
        
        # Extract engine dynamic payload
        engine_thrusts = []
        if num_engines > 0 and len(data) >= PACKET_HEADER_SIZE + num_engines * ENGINE_STRUCT_SIZE:
            for i in range(num_engines):
                offset = PACKET_HEADER_SIZE + i * ENGINE_STRUCT_SIZE
                eng_data = struct.unpack(ENGINE_STRUCT_FORMAT, data[offset:offset+ENGINE_STRUCT_SIZE])
                engine_thrusts.append(eng_data)

        # Sync simulation time with real-world time to avoid 1970 epoch issues in Foxglove
        now_ns = self.get_clock().now().nanoseconds
        if self.base_time_ns is None:
            self.base_time_ns = now_ns - (timestamp_us * 1000)
            self.get_logger().info("Synchronized simulation time with system epoch.")

        target_time_ns = self.base_time_ns + (timestamp_us * 1000)
        
        import rclpy.time
        sim_time = rclpy.time.Time(nanoseconds=target_time_ns).to_msg()
        
        # Publish Clock
        clock_msg = Clock()
        clock_msg.clock = sim_time
        self.clock_pub.publish(clock_msg)
        
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
        
        # Publish Noisy IMU
        noisy_imu = Imu()
        noisy_imu.header.stamp = sim_time
        noisy_imu.header.frame_id = 'rocket'
        noisy_imu.angular_velocity.x = imu_gyro_x
        noisy_imu.angular_velocity.y = imu_gyro_y
        noisy_imu.angular_velocity.z = imu_gyro_z
        noisy_imu.linear_acceleration.x = imu_acc_x
        noisy_imu.linear_acceleration.y = imu_acc_y
        noisy_imu.linear_acceleration.z = imu_acc_z
        # Orientację z IMU (idealną w tym projekcie) przepinamy
        noisy_imu.orientation.w = quat_w
        noisy_imu.orientation.x = quat_x
        noisy_imu.orientation.y = quat_y
        noisy_imu.orientation.z = quat_z
        self.noisy_imu_pub.publish(noisy_imu)
        
        # Publish Noisy GPS
        noisy_gps = NavSatFix()
        noisy_gps.header.stamp = sim_time
        noisy_gps.header.frame_id = 'world'
        noisy_gps.latitude = gps_lat
        noisy_gps.longitude = gps_lon
        noisy_gps.altitude = gps_alt
        self.noisy_gps_pub.publish(noisy_gps)
        
        # Publish TVC
        cmd_pitch_msg = PointStamped()
        cmd_pitch_msg.header.stamp = sim_time
        cmd_pitch_msg.point.x = tvc_cmd_pitch
        self.tvc_cmd_pitch_pub.publish(cmd_pitch_msg)

        cmd_yaw_msg = PointStamped()
        cmd_yaw_msg.header.stamp = sim_time
        cmd_yaw_msg.point.x = tvc_cmd_yaw
        self.tvc_cmd_yaw_pub.publish(cmd_yaw_msg)

        err_pitch_msg = PointStamped()
        err_pitch_msg.header.stamp = sim_time
        err_pitch_msg.point.x = tvc_err_pitch
        self.tvc_err_pitch_pub.publish(err_pitch_msg)

        err_yaw_msg = PointStamped()
        err_yaw_msg.header.stamp = sim_time
        err_yaw_msg.point.x = tvc_err_yaw
        self.tvc_err_yaw_pub.publish(err_yaw_msg)

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
        mass_msg = PointStamped()
        mass_msg.header.stamp = sim_time
        mass_msg.header.frame_id = 'rocket'
        mass_msg.point.x = mass_kg
        self.mass_pub.publish(mass_msg)

        # Publish Velocity for plotting
        vel_msg = PointStamped()
        vel_msg.header.stamp = sim_time
        vel_msg.header.frame_id = 'world'
        vel_msg.point.x = vel_x
        vel_msg.point.y = vel_y
        vel_msg.point.z = vel_z
        self.vel_pub.publish(vel_msg)

        vel_mag_msg = PointStamped()
        vel_mag_msg.header.stamp = sim_time
        vel_mag_msg.header.frame_id = 'world'
        vel_mag_msg.point.x = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        self.vel_mag_pub.publish(vel_mag_msg)

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
        rocket_marker.type = Marker.MESH_RESOURCE
        rocket_marker.action = Marker.ADD
        
        rocket_marker.mesh_resource = f"file://{self.mesh_path}"
        
        rocket_marker.pose.position.x = 0.0
        rocket_marker.pose.position.y = 0.0
        rocket_marker.pose.position.z = 0.0
        rocket_marker.pose.orientation.x = 0.0
        rocket_marker.pose.orientation.y = 0.0
        rocket_marker.pose.orientation.z = 0.0
        rocket_marker.pose.orientation.w = 1.0
        rocket_marker.scale.x = 1.0 # MESH_RESOURCE scale is a multiplier of original size
        rocket_marker.scale.y = 1.0
        rocket_marker.scale.z = 1.0
        rocket_marker.color.a = 1.0
        rocket_marker.color.r = 0.8
        rocket_marker.color.g = 0.8
        rocket_marker.color.b = 0.8
        self.body_marker_pub.publish(rocket_marker)

        # 2. Thrust Arrow MarkerArray
        thrust_array = MarkerArray()
        scale_factor = 0.002
        
        for i, eng_t in enumerate(engine_thrusts):
            tx, ty, tz = eng_t
            thrust_len = math.sqrt(tx**2 + ty**2 + tz**2)
            if thrust_len > 0.1:
                thrust_marker = Marker()
                thrust_marker.header.stamp = sim_time
                thrust_marker.header.frame_id = 'rocket'
                thrust_marker.ns = 'forces'
                thrust_marker.id = 100 + i  # Unique ID per engine
                thrust_marker.type = Marker.ARROW
                thrust_marker.action = Marker.ADD
                
                # Z position from configuration (fallback to 0)
                z_pos = self.engine_positions[i] if i < len(self.engine_positions) else -2.0
                
                p_start = Point()
                p_start.x = 0.0
                p_start.y = 0.0
                p_start.z = z_pos
                
                p_end = Point()
                p_end.x = p_start.x - tx * scale_factor
                p_end.y = p_start.y - ty * scale_factor
                p_end.z = p_start.z - tz * scale_factor
                
                dist = math.sqrt((p_end.x - p_start.x)**2 + (p_end.y - p_start.y)**2 + (p_end.z - p_start.z)**2)
                if dist > 0.15:
                    thrust_marker.points = [p_start, p_end]
                    thrust_marker.scale.x = 0.05
                    thrust_marker.scale.y = 0.15
                    thrust_marker.scale.z = 0.15
                    thrust_marker.color.r = 1.0
                    thrust_marker.color.g = 0.0
                    thrust_marker.color.b = 0.0
                    thrust_marker.color.a = 0.9
                    thrust_array.markers.append(thrust_marker)
        
        if thrust_array.markers:
            self.thrust_marker_pub.publish(thrust_array)

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

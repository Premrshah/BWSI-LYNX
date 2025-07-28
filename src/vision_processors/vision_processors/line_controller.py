#!/usr/bin/env python3

import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
from line_interfaces.msg import Line
from scipy.spatial.transform import Rotation as R
import tf_transformations as tft 

#############
# CONSTANTS #
#############
_RATE = 10               # Hz
_MAX_SPEED = 1.5        # m/s
_MAX_CLIMB_RATE = 1.0   # m/s
_MAX_ROTATION_RATE = 5.0# rad/s
IMAGE_HEIGHT = 960
IMAGE_WIDTH = 1280
CENTER = np.array([IMAGE_WIDTH//2, IMAGE_HEIGHT//2])

# Control parameters
FORWARD_SPEED = 0.8      # m/s
KP_LATERAL = 0.002       # lateral P gain
KD_LATERAL = 0.002       # lateral D gain
KP_YAW = 2             # yaw P gain
LOOKAHEAD_DISTANCE = 200 # pixels

class CoordTransforms:
    def __init__(self):
        # static: camera(dc) → body-down(bd)
        self.R_dc2bd = np.array([
            [ 0.0, -1.0,  0.0],  # bd.x = -dc.y
            [ 1.0,  0.0,  0.0],  # bd.y =  dc.x
            [ 0.0,  0.0,  1.0]   # bd.z =  dc.z
        ])

    def dc_to_bd(self, v_dc):
        return self.R_dc2bd @ v_dc

class LineController(Node):
    def __init__(self) -> None:
        super().__init__('line_controller')
        self.coord_transforms = CoordTransforms()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # publishers
        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        # subscribers
        self.sub_pos = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_cb, qos)
        self.sub_status = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos)
        self.sub_att = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_cb, qos)
        self.sub_line = self.create_subscription(Line, '/line_detector/line', self.line_cb, 1)

        # state
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.attitude: R | None = None

        self.offboard_counter = 0
        self.takeoff_height = -3.0
        self.line_detected = False
        self.prev_lateral_error = 0.0

        # commanded motions in camera frame
        self.vx_dc = 0.0
        self.vy_dc = 0.0
        self.wz_dc = 0.0

        # timer
        self.timer = self.create_timer(1.0/_RATE, self.timer_cb)

    def pos_cb(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    def status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def att_cb(self, msg: VehicleAttitude):
        q = msg.q
        self.attitude = R.from_quat([q[1], q[2], q[3], q[0]])

    def arm(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode')

    def send_cmd(self, cmd, **params):
        m = VehicleCommand()
        m.command = cmd
        for i, k in enumerate(['param1','param2','param3','param4','param5','param6','param7'], start=1):
            setattr(m, k, float(params.get(k, 0.0)))
        m.target_system = m.source_system = 1
        m.target_component = m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.pub_cmd.publish(m)

    def publish_offboard_heartbeat(self):
        m = OffboardControlMode()
        m.velocity = True
        m.body_rate = True
        m.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.pub_offboard.publish(m)

    def timer_cb(self):
        self.publish_offboard_heartbeat()
        if self.offboard_counter == 10:
            self.engage_offboard(); self.arm()
        if self.offboard_counter >= 100:
            if not self.line_detected:
                self.vx_dc = self.vy_dc = self.wz_dc = 0.0
            vx_bd, vy_bd, _ = self.coord_transforms.dc_to_bd(np.array([self.vx_dc, self.vy_dc, 0.0]))
            # body to world using attitude quaternion
            if self.attitude is not None:
                v_bd = np.array([vx_bd, vy_bd, 0.0])
                v_ned = self.attitude.apply(v_bd)
                vx_w, vy_w = v_ned[0], v_ned[1]
            else:
                vx_w, vy_w = 0.0, 0.0
            wz = np.clip(self.wz_dc, -_MAX_ROTATION_RATE, _MAX_ROTATION_RATE)

            msg = TrajectorySetpoint()
            msg.position = [math.nan, math.nan, self.takeoff_height]
            msg.velocity = [float(np.clip(vx_w, -_MAX_SPEED, _MAX_SPEED)),
                             float(np.clip(vy_w, -_MAX_SPEED, _MAX_SPEED)), 0.0]
            msg.yaw = float('nan')
            msg.yawspeed = wz
            msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
            self.pub_traj.publish(msg)
        self.offboard_counter += 1

    def line_cb(self, param: Line):
        self.line_detected = True
        x, y, vx, vy = param.x, param.y, param.vx, param.vy
        p = np.array([x, y])
        d = np.array([vx, vy])
        n = np.linalg.norm(d)
        if n < 1e-6:
            self.get_logger().warn('Line dir too small'); return
        d /= n
        to_center = CENTER - p
        proj = np.dot(to_center, d)
        closest = p + proj*d
        target = closest + LOOKAHEAD_DISTANCE*d
        perp = np.array([-d[1], d[0]])
        lat_err = np.dot(CENTER - closest, perp)
        lat_rate = (lat_err - self.prev_lateral_error)/(1.0/_RATE)
        self.prev_lateral_error = lat_err
        tvec = target - CENTER
        tdist = np.linalg.norm(tvec)
        if tdist < 1e-6:
            self.get_logger().warn('Target too close'); return
        tunit = tvec/tdist
        self.vx_dc = FORWARD_SPEED*tunit[0] - (KP_LATERAL*lat_err + KD_LATERAL*lat_rate)*perp[0]
        self.vy_dc = FORWARD_SPEED*tunit[1] - (KP_LATERAL*lat_err + KD_LATERAL*lat_rate)*perp[1]
        if self.attitude is not None:
            # Transform line direction from camera → world frame
            v_cam = np.array([d[0], d[1], 0.0])
            R_bd2lned = self.attitude.as_matrix()
            R_dc2bd = self.coord_transforms.R_dc2bd
            R_dc2lned = R_bd2lned @ R_dc2bd
            v_ned = R_dc2lned @ v_cam

            # Desired yaw (line direction in world)
            desired_yaw = math.atan2(v_ned[1], v_ned[0])

            # Current yaw (extract from quaternion)
            q = self.attitude.as_quat()  # [x, y, z, w]
            roll, pitch, current_yaw = tft.euler_from_quaternion([q[0], q[1], q[2], q[3]])

            # Shortest signed angle difference
            yaw_error = math.atan2(math.sin(desired_yaw - current_yaw), math.cos(desired_yaw - current_yaw))

            # Set yaw rate
            self.wz_dc = KP_YAW * yaw_error
        else:
            self.wz_dc = 0.0
        self.get_logger().info(f"L:{lat_err:.1f}px T:{tdist:.1f}px Yerr:{math.degrees(self.wz_dc/KP_YAW):+.1f}°")

def main(args=None):
    rclpy.init(args=args)
    node = LineController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
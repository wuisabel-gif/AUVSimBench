import random

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu, Range
from visualization_msgs.msg import Marker
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .dynamics import AUVDynamics, AUVParams


class AUVSimNode(Node):
    def __init__(self):
        super().__init__("auv_sim_bench")

        self.declare_parameter("sim_rate_hz", 200.0)
        self.declare_parameter("imu_rate_hz", 100.0)
        self.declare_parameter("dvl_rate_hz", 8.0)
        self.declare_parameter("depth_rate_hz", 5.0)

        # generic defaults; remap or load a profile (e.g. config/barracuda.yaml)
        self.declare_parameter("topic_prefix", "")
        self.declare_parameter("frame_prefix", "")
        self.declare_parameter("wrench_topic", "cmd_wrench")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("dvl_odom_topic", "dvl/odometry")
        self.declare_parameter("dvl_alt_topic", "dvl/altitude")
        self.declare_parameter("ping_range_topic", "ping/range")
        self.declare_parameter("pressure_topic", "pressure")
        self.declare_parameter("truth_topic", "ground_truth")
        self.declare_parameter("estimate_topic", "")  # "" disables the estimate marker

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("ping_frame", "ping_link")

        self.declare_parameter("wrench_timeout_sec", 0.5)
        self.declare_parameter("floor_z", -10.0)
        self.declare_parameter("surface_z", 0.0)
        self.declare_parameter("atm_pressure", 101325.0)

        self.declare_parameter("mass", 15.0)
        self.declare_parameter("volume", 0.0)
        self.declare_parameter("water_density", 1025.0)
        self.declare_parameter("current", [0.0, 0.0, 0.0])
        self.declare_parameter("inertia", [0.5, 0.8, 0.8])
        self.declare_parameter("cob_offset", [0.0, 0.0, 0.02])
        self.declare_parameter("linear_drag", [20.0, 40.0, 40.0])
        self.declare_parameter("quadratic_drag", [30.0, 80.0, 80.0])
        self.declare_parameter("angular_drag", [2.0, 4.0, 4.0])
        self.declare_parameter("angular_quadratic_drag", [1.0, 2.0, 2.0])
        self.declare_parameter("initial_position", [0.0, 0.0, 0.0])
        self.declare_parameter("initial_orientation_wxyz", [1.0, 0.0, 0.0, 0.0])
        self.declare_parameter("initial_linear_velocity", [0.0, 0.0, 0.0])
        self.declare_parameter("initial_angular_velocity", [0.0, 0.0, 0.0])

        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("body_size", [0.6, 0.35, 0.25])

        self.declare_parameter("dvl_noise_stddev", 0.0)
        self.declare_parameter("dvl_dropout_start_sec", -1.0)
        self.declare_parameter("dvl_dropout_duration_sec", 0.0)
        self.declare_parameter("dvl_emit_nan", False)
        self.declare_parameter("seed", 0)

        def param_value(name):
            return self.get_parameter(name).value

        def vec_param(name, size=3):
            value = [float(x) for x in param_value(name)]
            if len(value) != size:
                raise ValueError(f"{name} must contain {size} values")
            return value

        def normalize_prefix(prefix):
            prefix = str(prefix).strip().strip("/")
            return f"/{prefix}" if prefix else ""

        topic_prefix = normalize_prefix(param_value("topic_prefix"))
        frame_prefix = str(param_value("frame_prefix")).strip().strip("/")

        def expand_name(value):
            return str(value).format(
                topic_prefix=topic_prefix,
                frame_prefix=frame_prefix,
            )

        self.odom_frame = expand_name(param_value("odom_frame"))
        self.base_frame = expand_name(param_value("base_frame"))
        self.imu_frame = expand_name(param_value("imu_frame"))
        self.dvl_frame = expand_name(param_value("dvl_frame"))
        self.ping_frame = expand_name(param_value("ping_frame"))
        self.wrench_timeout = float(param_value("wrench_timeout_sec"))
        self.floor_z = float(param_value("floor_z"))
        self.surface_z = float(param_value("surface_z"))
        self.atm = float(param_value("atm_pressure"))
        self.publish_tf = bool(param_value("publish_tf"))
        self.publish_markers = bool(param_value("publish_markers"))
        self.body_size = [float(x) for x in param_value("body_size")]
        self.dvl_noise = float(param_value("dvl_noise_stddev"))
        self.dvl_drop_start = float(param_value("dvl_dropout_start_sec"))
        self.dvl_drop_dur = float(param_value("dvl_dropout_duration_sec"))
        self.dvl_nan = bool(param_value("dvl_emit_nan"))
        self.rng = random.Random(int(param_value("seed")))

        params = AUVParams()
        params.mass = float(param_value("mass"))
        params.water_density = float(param_value("water_density"))
        vol = float(param_value("volume"))
        params.volume = vol if vol > 0.0 else params.mass / params.water_density
        params.current_world = np.array(vec_param("current"))
        params.inertia = np.array(vec_param("inertia"))
        params.cob_offset = np.array(vec_param("cob_offset"))
        params.lin_drag = np.array(vec_param("linear_drag"))
        params.quad_drag = np.array(vec_param("quadratic_drag"))
        params.ang_drag = np.array(vec_param("angular_drag"))
        params.ang_quad_drag = np.array(vec_param("angular_quadratic_drag"))
        self.sim = AUVDynamics(params)
        self.sim.s.p = np.array(vec_param("initial_position"))
        self.sim.s.q = np.array(vec_param("initial_orientation_wxyz", 4))
        self.sim.s.v = np.array(vec_param("initial_linear_velocity"))
        self.sim.s.w = np.array(vec_param("initial_angular_velocity"))

        self.force = np.zeros(3)
        self.torque = np.zeros(3)
        self.last_wrench_t = self.now()
        self.estimate = None

        self.create_subscription(Wrench, expand_name(param_value("wrench_topic")), self.on_wrench, 10)
        self.imu_pub = self.create_publisher(Imu, expand_name(param_value("imu_topic")), 10)
        self.dvl_pub = self.create_publisher(Odometry, expand_name(param_value("dvl_odom_topic")), 10)
        self.alt_pub = self.create_publisher(Range, expand_name(param_value("dvl_alt_topic")), 10)
        self.ping_pub = self.create_publisher(Range, expand_name(param_value("ping_range_topic")), 10)
        self.press_pub = self.create_publisher(FluidPressure, expand_name(param_value("pressure_topic")), 10)
        self.truth_pub = self.create_publisher(PoseStamped, expand_name(param_value("truth_topic")), 10)

        if self.publish_tf:
            self.tf_bc = TransformBroadcaster(self)
            self.static_bc = StaticTransformBroadcaster(self)
            self._publish_static_frames()
        if self.publish_markers:
            self.marker_pub = self.create_publisher(Marker, "auv_sim/markers", 10)
        est = expand_name(param_value("estimate_topic"))
        if est:
            self.create_subscription(PoseStamped, est, self.on_estimate, 10)

        self.sim_dt = 1.0 / float(param_value("sim_rate_hz"))
        self.acc = {"imu": 0.0, "dvl": 0.0, "depth": 0.0}
        self.rates = {
            "imu": 1.0 / float(param_value("imu_rate_hz")),
            "dvl": 1.0 / float(param_value("dvl_rate_hz")),
            "depth": 1.0 / float(param_value("depth_rate_hz")),
        }
        self.elapsed = 0.0
        self.create_timer(self.sim_dt, self.step)
        self.get_logger().info(
            f"AUV sim bench @ {param_value('sim_rate_hz')} Hz, neutral V={params.volume:.4f} m^3; "
            f"rviz fixed frame = {self.odom_frame}"
        )

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_wrench(self, msg):
        self.force = np.array([msg.force.x, msg.force.y, msg.force.z])
        self.torque = np.array([msg.torque.x, msg.torque.y, msg.torque.z])
        self.last_wrench_t = self.now()

    def on_estimate(self, msg):
        self.estimate = msg

    def step(self):
        if self.now() - self.last_wrench_t > self.wrench_timeout:
            self.force = np.zeros(3)
            self.torque = np.zeros(3)

        s = self.sim.step(self.sim_dt, self.force, self.torque)
        self.elapsed += self.sim_dt
        for k in self.acc:
            self.acc[k] += self.sim_dt
        stamp = self.get_clock().now().to_msg()

        if self.acc["imu"] >= self.rates["imu"]:
            self.acc["imu"] = 0.0
            self.publish_imu(stamp, s)
            self.publish_truth(stamp, s)
        if self.acc["dvl"] >= self.rates["dvl"]:
            self.acc["dvl"] = 0.0
            self.publish_dvl(stamp, s)
        if self.acc["depth"] >= self.rates["depth"]:
            self.acc["depth"] = 0.0
            self.publish_depth(stamp, s)

    def publish_imu(self, stamp, s):
        f = self.sim.specific_force
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = s.q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = s.w
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = f
        self.imu_pub.publish(msg)

    def publish_dvl(self, stamp, s):
        if self.dvl_drop_start >= 0.0 and (
            self.dvl_drop_start <= self.elapsed < self.dvl_drop_start + self.dvl_drop_dur
        ):
            return
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.dvl_frame
        if self.dvl_nan:
            vx = vy = vz = float("nan")
        else:
            def noise():
                return self.rng.gauss(0.0, self.dvl_noise) if self.dvl_noise > 0 else 0.0

            vx, vy, vz = s.v[0] + noise(), s.v[1] + noise(), s.v[2] + noise()
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = vz
        self.dvl_pub.publish(msg)

    def publish_depth(self, stamp, s):
        altitude = float(s.p[2] - self.floor_z)
        for pub, frame in ((self.alt_pub, self.dvl_frame), (self.ping_pub, self.ping_frame)):
            r = Range()
            r.header.stamp = stamp
            r.header.frame_id = frame
            r.radiation_type = Range.ULTRASOUND
            r.min_range = 0.1
            r.max_range = 100.0
            r.range = altitude
            pub.publish(r)

        depth = max(0.0, self.surface_z - float(s.p[2]))
        pmsg = FluidPressure()
        pmsg.header.stamp = stamp
        pmsg.header.frame_id = self.imu_frame
        pmsg.fluid_pressure = self.atm + self.sim.p.water_density * self.sim.p.gravity * depth
        self.press_pub.publish(pmsg)

    def publish_truth(self, stamp, s):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = s.p
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = s.q
        self.truth_pub.publish(msg)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = s.p
            tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = s.q
            self.tf_bc.sendTransform(tf)

        if self.publish_markers:
            self._publish_body_marker(stamp)
            if self.estimate is not None:
                self._publish_estimate_marker(stamp)

    def _publish_static_frames(self):
        tfs = []
        for child in (self.imu_frame, self.dvl_frame, self.ping_frame):
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.base_frame
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            tfs.append(tf)
        self.static_bc.sendTransform(tfs)

    def _publish_body_marker(self, stamp):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.base_frame
        m.ns = "auv"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = self.body_size
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.5, 1.0, 0.85
        self.marker_pub.publish(m)

    def _publish_estimate_marker(self, stamp):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.odom_frame
        m.ns = "estimate"
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = self.estimate.pose
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.3, 0.9
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = AUVSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

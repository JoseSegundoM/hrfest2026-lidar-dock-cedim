"""Nodo de acoplamiento guiado por LiDAR para el iRobot Create 3.

Interfaces usadas, todas dentro de la lista de permitidas de las bases:

  suscribe   /scan              unica fuente de percepcion del entorno
             /odom              propagacion del estimado entre barridos
             /dock_status       confirmacion de acoplamiento
             /hazard_detection  contactos, para no acumular penalizaciones
             /tf, /tf_static    montaje del sensor sobre el robot
  publica    /cmd_vel           comando de velocidad

No se usa la accion /dock, ni los topicos de infrarrojos, ni los
comportamientos prellenados del Create 3, ni ninguna fuente de ground truth.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener

from .control import DockController, State, axis_errors
from .estimator import DockEstimator
from .geometry import transform_points
from .perception import DockDetector

# El Create 3 publica varias clases de riesgo en el mismo topico. Solo se
# atienden las de contacto fisico. OBJECT_PROXIMITY se ignora a proposito:
# procede de los sensores infrarrojos, que las bases prohiben.
HAZARD_BUMP = 1
HAZARD_STALL = 3
HAZARD_WHEEL_DROP = 4
CONTACT_HAZARDS = (HAZARD_BUMP, HAZARD_STALL, HAZARD_WHEEL_DROP)

DEFAULTS = {
    'beam_spacing_at_1m': 0.008727,
    'max_walls': 4,
    'min_wall_points': 25,
    'wall_inlier_tol': 0.02,
    'ransac_iterations': 120,
    'max_wall_range': 3.0,
    'min_protrusion': 0.035,
    'max_protrusion': 0.13,
    'cluster_gap': 0.035,
    'min_cluster_points': 2,
    'front_face_tol': 0.015,
    'max_cluster_span': 0.14,
    'separation_tol': 0.04,
    'min_quality': 0.35,
    'nominal_wall_points': 120,
    'drift_xy': 0.01,
    'drift_theta': 0.01,
    'sigma_wall_normal': 0.004,
    'sigma_lateral_floor': 0.006,
    'sigma_theta_at_1m': 0.012,
    'min_quality_scale': 0.2,
    'gate_after_updates': 5,
    'gate_threshold': 16.0,
    'min_updates': 6,
    'confident_sigma_xy': 0.05,
    'confident_sigma_theta': 0.06,
    'search_omega': 0.9,
    'standoff_distance': 0.75,
    'standoff_distance_tol': 0.08,
    'standoff_lateral_tol': 0.03,
    'standoff_heading_tol': 0.06,
    'turn_in_place_angle': 0.7,
    'rho_settled': 0.06,
    'k_rho': 0.7,
    'k_alpha': 1.5,
    'k_beta': -0.5,
    'k_beta_final': 1.2,
    'v_max_approach': 0.30,
    'w_max': 1.2,
    'dock_stop_distance': 0.266,
    'k_v_enter': 0.45,
    'v_min_enter': 0.05,
    'v_max_enter': 0.18,
    'k_heading_enter': 1.0,
    'k_lateral_enter': 2.0,
    'w_enter_max': 0.6,
    'abort_below_remaining': 0.15,
    'abort_lateral': 0.02,
    'settle_seconds': 3.0,
    'settle_speed': 0.04,
    'w_settle': 0.15,
    'recover_seconds': 1.0,
    'recover_speed': 0.08,
    'control_rate': 20.0,
    'scan_timeout': 0.5,
    'run_timeout': 175.0,
}


def yaw_from_quaternion(q):
    """Extrae el angulo de guinada de un cuaternion ROS."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DockingNode(Node):
    """Integra percepcion, estimacion y control en un solo lazo."""

    def __init__(self):
        super().__init__('cedim_docking_node')

        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}

        self.detector = DockDetector(self.p, np.random.default_rng(12345))
        self.estimator = DockEstimator(self.p)
        self.controller = DockController(self.p)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.laser_offset = None        # (x, y, yaw) de laser_link en base_link

        self.odom_pose = None
        self.last_scan_time = None
        self.last_estimator_time = None
        self.started_at = None
        self.finished = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom,
                                 qos_profile_sensor_data)
        self._subscribe_create3_topics()

        period = 1.0 / float(self.p['control_rate'])
        self.create_timer(period, self.on_control)

        self.get_logger().info(
            'cedim_dock listo. Buscando la firma del marcador en /scan.')

    def _subscribe_create3_topics(self):
        """Suscribe los topicos propios del Create 3, si sus tipos existen.

        Se aislan para que el nodo siga siendo utilizable en un banco de
        pruebas donde irobot_create_msgs no este instalado.
        """
        try:
            from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
        except ImportError:
            self.get_logger().warn(
                'irobot_create_msgs no disponible: sin confirmacion de '
                'acoplamiento ni deteccion de contactos.')
            return
        self.create_subscription(DockStatus, 'dock_status', self.on_dock_status,
                                 qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, 'hazard_detection',
                                 self.on_hazard, qos_profile_sensor_data)

    # -- entradas ----------------------------------------------------------

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.odom_pose = np.array([
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        ])

    def on_dock_status(self, msg):
        if msg.is_docked and not self.finished:
            self.controller.notify_docked()
            self._finish('acoplado: /dock_status confirma is_docked')

    def on_hazard(self, msg):
        for detection in msg.detections:
            if detection.type in CONTACT_HAZARDS:
                self.controller.notify_hazard(self._now())
                self.get_logger().warn('contacto detectado: retrocediendo')
                return

    def on_scan(self, msg):
        """Percepcion: cada barrido aporta una medida al estimador."""
        now = self._now()
        self.last_scan_time = now
        if self.started_at is None:
            self.started_at = now
        if self.odom_pose is None or self.finished:
            return

        if self.laser_offset is None:
            self.laser_offset = self._lookup_laser_offset(msg.header.frame_id)
            if self.laser_offset is None:
                return

        points = self._scan_to_base(msg)
        if len(points) < self.p['min_wall_points']:
            return

        dt = 0.0 if self.last_estimator_time is None else now - self.last_estimator_time
        self.last_estimator_time = now
        self.estimator.predict(dt)

        detection = self.detector.detect(points)
        if detection is not None:
            self.estimator.update(detection, self.odom_pose, now)

    # -- salida ------------------------------------------------------------

    def on_control(self):
        """Lazo de control, desacoplado de la llegada de barridos."""
        if self.finished:
            return
        now = self._now()

        # Vigilancia: sin LiDAR fresco el robot no se mueve.
        if self.last_scan_time is None or (now - self.last_scan_time) > self.p['scan_timeout']:
            self._publish(0.0, 0.0)
            return

        if self.started_at is not None and (now - self.started_at) > self.p['run_timeout']:
            self._finish('limite de tiempo alcanzado sin acoplar')
            return

        dock_in_base = (self.estimator.dock_in_base(self.odom_pose)
                        if self.odom_pose is not None else None)
        v, w = self.controller.step(
            dock_in_base, self.estimator.confident(),
            1.0 / float(self.p['control_rate']), now)
        self._publish(v, w)

        if self.controller.state is State.DOCKED:
            self._finish('acoplado')
        elif dock_in_base is not None:
            self._log_progress(dock_in_base)

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _finish(self, reason):
        self.finished = True
        self._publish(0.0, 0.0)
        elapsed = 0.0 if self.started_at is None else self._now() - self.started_at
        self.get_logger().info(f'{reason} (t = {elapsed:.1f} s)')

    # -- utilidades --------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup_laser_offset(self, laser_frame):
        """Monta el sensor sobre el robot leyendo TF, sin cotas escritas a mano."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', laser_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        offset = (t.x, t.y, yaw_from_quaternion(tf.transform.rotation))
        self.get_logger().info(
            f'LiDAR en base_link: x={offset[0]:.4f} y={offset[1]:.4f} '
            f'yaw={math.degrees(offset[2]):.2f} deg')
        return offset

    def _scan_to_base(self, msg):
        """Pasa el barrido a puntos cartesianos en ``base_link``."""
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = (np.isfinite(ranges)
                 & (ranges > msg.range_min)
                 & (ranges < msg.range_max))
        ranges, angles = ranges[valid], angles[valid]
        local = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
        return transform_points(local, *self.laser_offset)

    def _log_progress(self, dock_in_base):
        distance, lateral, heading = axis_errors(dock_in_base)
        self.get_logger().info(
            f'{self.controller.state.value}  d={distance:.3f} m  '
            f'lat={lateral * 1000.0:+.0f} mm  '
            f'rumbo={math.degrees(heading):+.1f} deg  '
            f'n={self.estimator.n_updates}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

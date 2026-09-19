"""Filtro de Kalman sobre la pose del dock como landmark estatico en ``odom``.

Esta es la decision de diseno que sostiene la precision final. El dock no se
trata como una deteccion instantanea sino como un punto fijo del mundo: el
estado es su pose en el marco ``odom``, donde por definicion no se mueve.
Cada barrido aporta una medida y el filtro acumula evidencia.

Importa porque los ultimos centimetros son ciegos. En la pose de acoplamiento
el robot queda a ~0.27 m de la pared y el LiDAR va 5 cm por detras de su
centro, de modo que las cajas caen contra el rango minimo del sensor y la
firma se degrada justo cuando mas precision hace falta. Con el dock anclado en
``odom``, el tramo final se ejecuta contra un estimado ya convergido y
propagado por odometria, en lugar de contra una medida que se esta perdiendo.
"""

import math

import numpy as np

from .geometry import compose, relative, wrap_angle


class DockEstimator:
    """Estima la pose del dock en ``odom`` fusionando detecciones sucesivas."""

    def __init__(self, params):
        self.p = params
        self.state = None                    # (x, y, theta) del dock en odom
        self.cov = None
        self.n_updates = 0
        self.last_update_stamp = None

    @property
    def initialised(self):
        return self.state is not None

    def predict(self, dt):
        """El dock es estatico: solo crece la incertidumbre por deriva de odometria."""
        if self.state is None or dt <= 0.0:
            return
        drift = np.array([
            self.p['drift_xy'] ** 2,
            self.p['drift_xy'] ** 2,
            self.p['drift_theta'] ** 2,
        ])
        self.cov = self.cov + np.diag(drift) * dt

    def update(self, detection, odom_pose, stamp):
        """Incorpora una deteccion medida desde ``odom_pose``.

        Devuelve ``True`` si la medida se acepto.
        """
        z = compose(odom_pose, detection.pose)
        r_cov = self._measurement_cov(detection, z[2])

        if self.state is None:
            self.state = z
            self.cov = r_cov.copy()
            self.n_updates = 1
            self.last_update_stamp = stamp
            return True

        innovation = z - self.state
        innovation[2] = wrap_angle(innovation[2])

        s = self.cov + r_cov
        try:
            s_inv = np.linalg.inv(s)
        except np.linalg.LinAlgError:
            return False

        # Puerta de Mahalanobis: una vez convergido el filtro, un falso
        # positivo lejano cuesta mucho mas que perder una medida buena.
        if self.n_updates >= self.p['gate_after_updates']:
            mahalanobis = float(innovation @ s_inv @ innovation)
            if mahalanobis > self.p['gate_threshold']:
                return False

        gain = self.cov @ s_inv
        self.state = self.state + gain @ innovation
        self.state[2] = wrap_angle(self.state[2])
        self.cov = (np.eye(3) - gain) @ self.cov
        self.cov = 0.5 * (self.cov + self.cov.T)     # fuerza la simetria
        self.n_updates += 1
        self.last_update_stamp = stamp
        return True

    def dock_in_base(self, odom_pose):
        """Pose del dock en ``base_link`` segun el estimado actual."""
        if self.state is None:
            return None
        return relative(odom_pose, self.state)

    def confident(self):
        """El estimado ya es utilizable para la maniobra de entrada."""
        if self.state is None or self.n_updates < self.p['min_updates']:
            return False
        sigma_xy = math.sqrt(max(self.cov[0, 0] + self.cov[1, 1], 0.0))
        sigma_theta = math.sqrt(max(self.cov[2, 2], 0.0))
        return (sigma_xy < self.p['confident_sigma_xy']
                and sigma_theta < self.p['confident_sigma_theta'])

    # -- interno -----------------------------------------------------------

    def _measurement_cov(self, detection, dock_theta_odom):
        """Covarianza de la medida, construida en el marco propio del dock.

        El error no es isotropo. La distancia a la pared sale del ajuste de
        cientos de puntos y es muy precisa; el centrado lateral depende del
        numero de ecos que caen sobre las cajas, que baja con la distancia.
        """
        rng = max(detection.wall_distance, 0.15)
        spacing = self.p['beam_spacing_at_1m'] * rng

        sigma_normal = self.p['sigma_wall_normal']
        sigma_lateral = math.sqrt(
            (spacing / math.sqrt(max(detection.n_points, 1))) ** 2
            + self.p['sigma_lateral_floor'] ** 2
        )
        sigma_theta = self.p['sigma_theta_at_1m'] * max(rng, 1.0)

        scale = 1.0 / max(detection.quality, self.p['min_quality_scale'])
        local = np.diag([sigma_normal ** 2, sigma_lateral ** 2, sigma_theta ** 2]) * scale

        # El eje x del dock es su normal saliente: rota el bloque 2x2 a odom.
        c, s = math.cos(dock_theta_odom), math.sin(dock_theta_odom)
        rot = np.array([[c, -s], [s, c]])
        cov = np.zeros((3, 3))
        cov[:2, :2] = rot @ local[:2, :2] @ rot.T
        cov[2, 2] = local[2, 2]
        return cov

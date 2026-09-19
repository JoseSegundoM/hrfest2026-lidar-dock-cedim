"""Deteccion del dock a partir de la firma geometrica del marcador.

El dock no es visible para el LiDAR. Lo que si se ve son las dos cajas
montadas en la pared que hay detras: dos salientes de ~8 cm de ancho que
sobresalen ~8 cm sobre el plano de la pared, separadas por un hueco de
9.5 cm. El eje del dock es la perpendicular a la pared que pasa por el
centro de ese hueco.

El detector no busca bordes ni el hueco directamente. Ajusta primero el
plano de la pared con todos sus puntos --cientos de muestras, lo que da una
normal muy estable-- y despues busca los salientes en el marco de esa pared.
Estimar la orientacion a partir de la pared y no de las cajas es lo que
mantiene el error angular muy por debajo del margen de +-6 grados.
"""

import math
from dataclasses import dataclass

import numpy as np

from .geometry import BOX_WIDTH, CENTER_SEP, ransac_line


@dataclass
class Detection:
    """Una observacion del dock expresada en el marco ``base_link``."""

    pose: np.ndarray        # (x, y, theta) del dock; theta mira hacia la sala
    wall_distance: float    # distancia perpendicular del robot a la pared
    quality: float          # 0..1, para ponderar la actualizacion del filtro
    n_points: int           # puntos que sostienen los dos salientes


class DockDetector:
    """Detector de la firma del marcador en un barrido cartesiano."""

    def __init__(self, params, rng=None):
        self.p = params
        self.rng = rng if rng is not None else np.random.default_rng(0)

    def detect(self, points):
        """Devuelve la mejor ``Detection`` del barrido, o ``None``.

        ``points`` son los ecos validos del barrido en ``base_link`` (N, 2).
        """
        best = None
        remaining = points
        for _ in range(self.p['max_walls']):
            if len(remaining) < self.p['min_wall_points']:
                break
            fit = ransac_line(
                remaining,
                self.p['wall_inlier_tol'],
                self.p['ransac_iterations'],
                self.rng,
            )
            if fit is None:
                break
            normal, offset, inliers = fit

            if offset <= self.p['max_wall_range']:
                candidate = self._marker_on_wall(remaining, normal, offset, inliers)
                if candidate is not None and (best is None or candidate.quality > best.quality):
                    best = candidate

            # Quita esta pared y vuelve a intentar con el resto del barrido.
            remaining = remaining[~inliers]

        return best

    # -- interno -----------------------------------------------------------

    def _marker_on_wall(self, points, normal, offset, inliers):
        """Busca el par de salientes sobre una pared ya ajustada."""
        tangent = np.array([-normal[1], normal[0]])

        depth = offset - points @ normal    # >0 => delante de la pared
        along = points @ tangent

        n_inliers = int(inliers.sum())
        if n_inliers < self.p['min_wall_points']:
            return None

        # Los salientes solo son creibles dentro de la extension de la pared.
        wall_along = along[inliers]
        lo, hi = wall_along.min() - 0.10, wall_along.max() + 0.10

        mask = (
            (depth > self.p['min_protrusion'])
            & (depth < self.p['max_protrusion'])
            & (along > lo)
            & (along < hi)
        )
        if int(mask.sum()) < 4:
            return None

        clusters = self._cluster(along[mask], depth[mask])
        if len(clusters) < 2:
            return None

        pair = self._best_pair(clusters)
        if pair is None:
            return None
        left, right, sep_error = pair

        s_mid = 0.5 * (left['centre'] + right['centre'])
        dock_xy = offset * normal + s_mid * tangent
        dock_theta = math.atan2(-normal[1], -normal[0])

        n_points = left['n'] + right['n']
        quality = self._quality(sep_error, n_points, n_inliers, offset)
        if quality < self.p['min_quality']:
            return None

        return Detection(
            pose=np.array([dock_xy[0], dock_xy[1], dock_theta]),
            wall_distance=float(offset),
            quality=quality,
            n_points=n_points,
        )

    def _cluster(self, along, depth):
        """Agrupa los ecos salientes y localiza el centro de cada caja.

        El centro se calcula **solo con los ecos de la cara frontal**, y esa
        restriccion no es cosmetica. Vista de frente, una caja devuelve una
        cara; vista en oblicuo devuelve tambien su cara lateral, cuyos ecos
        caen todos sobre el borde exterior y estan a menor profundidad. Si se
        promedian junto con los de la cara frontal, el centro estimado se
        desplaza hacia fuera, y el desplazamiento crece con el angulo de
        vision. Medido en el banco sintetico ese sesgo llega a 2 cm: mas del
        doble del margen lateral de 8 mm que dan las bases. Quedarse con la
        cara frontal lo elimina de raiz.
        """
        order = np.argsort(along)
        s, d = along[order], depth[order]
        splits = np.flatnonzero(np.diff(s) > self.p['cluster_gap']) + 1

        clusters = []
        for chunk_s, chunk_d in zip(np.split(s, splits), np.split(d, splits)):
            if len(chunk_s) < self.p['min_cluster_points']:
                continue
            if float(chunk_s[-1] - chunk_s[0]) > self.p['max_cluster_span']:
                continue

            # Los ecos mas salientes del grupo definen la cara frontal. El
            # umbral es relativo al propio grupo, asi que un pequeno error en
            # el ajuste de la pared no lo descoloca.
            reference = (float(np.percentile(chunk_d, 90)) if len(chunk_d) >= 4
                         else float(chunk_d.max()))
            face = chunk_d >= reference - self.p['front_face_tol']
            if int(face.sum()) < self.p['min_cluster_points']:
                continue

            face_s = chunk_s[face]
            span = float(face_s[-1] - face_s[0]) if len(face_s) > 1 else 0.0
            if span > self.p['max_cluster_span']:
                continue

            clusters.append({
                'centre': float(face_s.mean()),
                'span': span,
                'depth': float(chunk_d[face].mean()),
                'n': int(face.sum()),
            })
        return clusters

    def _best_pair(self, clusters):
        """Elige el par de salientes cuya separacion se acerca mas a la cota."""
        best = None
        best_error = self.p['separation_tol']
        for i in range(len(clusters)):
            for j in range(i + 1, len(clusters)):
                a, b = clusters[i], clusters[j]
                error = abs(abs(b['centre'] - a['centre']) - CENTER_SEP)
                if error < best_error:
                    best_error = error
                    best = (a, b, error) if a['centre'] < b['centre'] else (b, a, error)
        return best

    def _quality(self, sep_error, n_points, n_inliers, wall_range):
        """Puntua la deteccion: cuanto encaja con la cota y cuanto la sostiene."""
        # Encaje con la separacion nominal entre centros de caja.
        fit = 1.0 - sep_error / self.p['separation_tol']
        # Soporte: mas ecos sobre las cajas => centro mejor determinado.
        expected = 2.0 * BOX_WIDTH / max(self.p['beam_spacing_at_1m'] * wall_range, 1e-3)
        support = min(1.0, n_points / max(expected * 0.5, 2.0))
        # Estabilidad de la normal de la pared.
        wall = min(1.0, n_inliers / float(self.p['nominal_wall_points']))
        return float(max(0.0, fit) * (0.5 + 0.3 * support + 0.2 * wall))

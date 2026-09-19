"""Primitivas geometricas y cotas del marcador.

Las unicas constantes de escenario que aparecen aqui son las **medidas
relativas del marcador** que las bases publican explicitamente. No hay
ninguna coordenada del mundo, ni la pose del dock, ni la pose inicial del
robot, ni la posicion de las paredes.
"""

import math

import numpy as np

# --- Cotas del marcador publicadas en las bases (metros) ---------------------
BOX_WIDTH = 0.08      # ancho de la cara de cada caja, a lo largo de la pared
BOX_DEPTH = 0.08      # cuanto sobresale cada caja respecto de la pared
GAP = 0.095           # hueco libre entre caja y caja
CENTER_SEP = 0.175    # separacion entre centros de caja


def wrap_angle(a):
    """Normaliza un angulo al intervalo (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def polar_to_xy(ranges, angles):
    """Convierte un barrido polar en puntos cartesianos (N, 2)."""
    return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))


def transform_points(points, x, y, theta):
    """Aplica la transformacion rigida (x, y, theta) a un conjunto de puntos."""
    c, s = math.cos(theta), math.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + np.array([x, y])


def compose(pose_a, pose_b):
    """Compone dos poses 2D: devuelve pose_a (+) pose_b."""
    xa, ya, ta = pose_a
    xb, yb, tb = pose_b
    c, s = math.cos(ta), math.sin(ta)
    return np.array([
        xa + c * xb - s * yb,
        ya + s * xb + c * yb,
        wrap_angle(ta + tb),
    ])


def invert(pose):
    """Invierte una pose 2D."""
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    return np.array([-c * x - s * y, s * x - c * y, wrap_angle(-t)])


def relative(pose_from, pose_to):
    """Expresa pose_to en el marco de pose_from."""
    return compose(invert(pose_from), pose_to)


def fit_line_tls(points):
    """Ajusta una recta por minimos cuadrados totales.

    Devuelve ``(normal, offset)`` con ``normal`` unitaria y el signo elegido
    de forma que ``offset = normal . p >= 0`` para los puntos de la recta.
    Es decir: la normal apunta desde el origen del sensor hacia la pared y
    ``offset`` es la distancia perpendicular del sensor a la pared.
    """
    centroid = points.mean(axis=0)
    centred = points - centroid
    # El vector singular menor de la matriz de covarianza es la normal.
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    normal = vh[-1]
    offset = float(normal @ centroid)
    if offset < 0.0:
        normal, offset = -normal, -offset
    return normal / np.linalg.norm(normal), offset


def ransac_line(points, threshold, iterations, rng):
    """Busca la recta dominante en ``points`` por RANSAC.

    Devuelve ``(normal, offset, mascara_de_inliers)`` o ``None`` si no hay
    puntos suficientes.
    """
    n = len(points)
    if n < 8:
        return None

    best_mask = None
    best_count = 0
    for _ in range(iterations):
        i, j = rng.choice(n, size=2, replace=False)
        p, q = points[i], points[j]
        direction = q - p
        norm = np.linalg.norm(direction)
        if norm < 0.05:          # muestras demasiado juntas: recta mal definida
            continue
        normal = np.array([-direction[1], direction[0]]) / norm
        offset = float(normal @ p)
        mask = np.abs(points @ normal - offset) < threshold
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask

    if best_mask is None or best_count < 8:
        return None

    normal, offset = fit_line_tls(points[best_mask])
    mask = np.abs(points @ normal - offset) < threshold
    if int(mask.sum()) < 8:
        return None
    return normal, offset, mask

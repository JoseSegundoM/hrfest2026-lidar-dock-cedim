"""Maquina de estados y reguladores de la maniobra de acoplamiento.

La maniobra esta partida en dos tramos con objetivos distintos, y esa
separacion es deliberada. Un unico regulador polar que llevase al robot desde
cualquier pose hasta el dock acopla el error lateral con el angular: llega
girando, y el ultimo grado de giro se traduce en milimetros de desvio justo
donde el margen es de 8 mm.

En su lugar el robot primero alcanza un punto de espera **sobre el eje del
dock**, mirandolo de frente, donde las tolerancias son holgadas. Desde ahi la
entrada es un seguimiento de recta: el error lateral se corrige a lo largo de
todo el tramo y llega a cero antes del contacto, en vez de resolverse en el
ultimo instante.
"""

import math
from enum import Enum

import numpy as np

from .geometry import wrap_angle


class State(Enum):
    SEARCH = 'SEARCH'
    APPROACH = 'APPROACH'
    ENTER = 'ENTER'
    RECOVER = 'RECOVER'
    DOCKED = 'DOCKED'


def axis_errors(dock_in_base):
    """Errores del robot respecto del eje del dock, en ``base_link``.

    Devuelve ``(distancia_sobre_el_eje, error_lateral, error_de_rumbo)``:

    - distancia: cuanto falta hasta el plano de la pared siguiendo el eje;
    - lateral: desvio del robot respecto del eje, positivo si esta a la
      izquierda del sentido de avance;
    - rumbo: giro que le falta al robot para quedar alineado con el eje.
    """
    dx, dy, dtheta = dock_in_base
    c, s = math.cos(dtheta), math.sin(dtheta)
    distance = -(dx * c + dy * s)
    lateral = dy * c - dx * s
    heading = wrap_angle(dtheta + math.pi)
    return distance, lateral, heading


class DockController:
    """Genera ``(v, w)`` a partir del estimado del dock y del estado interno."""

    def __init__(self, params):
        self.p = params
        self.state = State.SEARCH
        self.search_direction = 1.0
        self.recover_until = None
        self.settle_elapsed = 0.0
        self.attempts = 0

    def reset_to_approach(self):
        self.state = State.APPROACH
        self.settle_elapsed = 0.0

    # -- entradas externas -------------------------------------------------

    def notify_hazard(self, now):
        """Un contacto obliga a retroceder: una colision cuesta 10 puntos."""
        if self.state in (State.DOCKED, State.RECOVER):
            return
        self.state = State.RECOVER
        self.recover_until = now + self.p['recover_seconds']
        self.attempts += 1

    def notify_docked(self):
        self.state = State.DOCKED

    # -- ciclo de control --------------------------------------------------

    def step(self, dock_in_base, confident, dt, now):
        """Un ciclo de control. Devuelve ``(v, w)`` en m/s y rad/s."""
        if self.state is State.DOCKED:
            return 0.0, 0.0

        if self.state is State.RECOVER:
            if now < self.recover_until:
                return -self.p['recover_speed'], 0.0
            self.state = State.APPROACH if confident else State.SEARCH
            return 0.0, 0.0

        if self.state is State.SEARCH:
            if confident:
                self.state = State.APPROACH
            else:
                return 0.0, self.search_direction * self.p['search_omega']

        if dock_in_base is None:
            return 0.0, self.search_direction * self.p['search_omega']

        distance, lateral, heading = axis_errors(dock_in_base)

        if self.state is State.APPROACH:
            return self._approach(dock_in_base, distance, lateral, heading, dt)

        return self._enter(distance, lateral, heading, dt)

    # -- tramos ------------------------------------------------------------

    def _approach(self, dock_in_base, distance, lateral, heading, dt):
        """Regulacion polar hasta el punto de espera sobre el eje."""
        standoff = self.p['standoff_distance']

        # Si ya se cumplen las tolerancias del punto de espera, entra.
        if (abs(distance - standoff) < self.p['standoff_distance_tol']
                and abs(lateral) < self.p['standoff_lateral_tol']
                and abs(heading) < self.p['standoff_heading_tol']):
            self.state = State.ENTER
            self.settle_elapsed = 0.0
            return self._enter(distance, lateral, heading, dt)

        dx, dy, dtheta = dock_in_base
        goal_x = dx + standoff * math.cos(dtheta)
        goal_y = dy + standoff * math.sin(dtheta)
        goal_theta = wrap_angle(dtheta + math.pi)

        rho = math.hypot(goal_x, goal_y)
        alpha = wrap_angle(math.atan2(goal_y, goal_x))
        beta = wrap_angle(goal_theta - alpha)

        # Con el objetivo muy desalineado conviene girar en el sitio: avanzar
        # describiendo un arco largo cuesta mas tiempo del que ahorra.
        if abs(alpha) > self.p['turn_in_place_angle'] and rho > self.p['rho_settled']:
            return 0.0, _clamp(self.p['k_alpha'] * alpha,
                               -self.p['w_max'], self.p['w_max'])

        if rho < self.p['rho_settled']:
            # Sobre el punto de espera: solo queda cuadrar el rumbo.
            return 0.0, _clamp(self.p['k_beta_final'] * wrap_angle(goal_theta),
                               -self.p['w_max'], self.p['w_max'])

        v = self.p['k_rho'] * rho * math.cos(alpha)
        w = self.p['k_alpha'] * alpha + self.p['k_beta'] * beta
        return (_clamp(v, -self.p['v_max_approach'], self.p['v_max_approach']),
                _clamp(w, -self.p['w_max'], self.p['w_max']))

    def _enter(self, distance, lateral, heading, dt):
        """Entrada recta sobre el eje, con el lateral corriendose durante todo el tramo."""
        target = self.p['dock_stop_distance']
        remaining = distance - target

        # El error lateral debe estar resuelto antes del contacto. Si a mitad
        # de tramo sigue fuera de margen, no hay recorrido para corregirlo:
        # sale y repite la aproximacion en vez de acoplar mal.
        if (remaining < self.p['abort_below_remaining']
                and abs(lateral) > self.p['abort_lateral']):
            self.state = State.APPROACH
            self.attempts += 1
            return 0.0, 0.0

        if remaining <= 0.0:
            # En la pose de acoplamiento pero sin confirmacion del dock:
            # empuja muy despacio contra la rampa durante un margen acotado.
            self.settle_elapsed += dt
            if self.settle_elapsed > self.p['settle_seconds']:
                return 0.0, 0.0
            return self.p['settle_speed'], _clamp(
                self.p['k_heading_enter'] * heading,
                -self.p['w_settle'], self.p['w_settle'])

        v = _clamp(self.p['k_v_enter'] * remaining,
                   self.p['v_min_enter'], self.p['v_max_enter'])
        w = self.p['k_heading_enter'] * heading - self.p['k_lateral_enter'] * lateral
        return v, _clamp(w, -self.p['w_enter_max'], self.p['w_enter_max'])


def _clamp(value, low, high):
    return float(np.clip(value, low, high))

"""
Después de un frenado de emergencia: el ego retoma la marcha y vuelve a seguir al líder.

No depende de CARLA. Solo se usa en los ensayos con `reanudar=True`.

Funcionamiento:
  1. Mientras el AEB frena (o el ego está detenido por él) no se hace nada.
  2. Cuando el camino queda libre (el peatón ya cruzó) durante `espera_libre_s`, se suelta la retención del AEB.
  3. A partir de ahí un conductor virtual sigue al líder con una separación que crece con la velocidad:
         separación deseada = d_min + t_h * v_ego
         v_objetivo = v_líder + k_gap * (separación - separación deseada)   (entre 0 y v_crucero)
     La velocidad objetivo sube con rampa suave (a_arranque) y baja con a_frenado; si el ego va más rápido que el
     objetivo, pide freno. El AEB sigue activo como protección.
  4. El ensayo termina `duracion_s` después de retomar la marcha.
"""

import math


class ReanudarSeguimiento:

    def __init__(self, dt, v_crucero, d_max, espera_libre_s=1.0, d_min=5.0, t_h=1.5, k_gap=0.35,
                 a_arranque=1.5, a_frenado=3.0, duracion_s=8.0):
        self.dt, self.v_crucero, self.d_max = dt, v_crucero, d_max
        self.espera_libre_s, self.d_min, self.t_h, self.k_gap = espera_libre_s, d_min, t_h, k_gap
        self.a_arranque, self.a_frenado, self.duracion_s = a_arranque, a_frenado, duracion_s
        self.reanudado = False
        self.t_reanuda = None
        self.v_ref = None
        self._libre_s = 0.0

    def actualizar(self, t, frenado, camino_libre):
        """frenado: el AEB ya intervino. camino_libre: no hay peatón delante. True en el ciclo en que reanuda."""
        if self.reanudado or not frenado:
            return False
        self._libre_s = self._libre_s + self.dt if camino_libre else 0.0
        if self._libre_s >= self.espera_libre_s:
            self.reanudado, self.t_reanuda = True, t
            return True
        return False

    def comando(self, v_ego, gap, v_lider):
        """gap, v_lider: del líder delante (None o NaN si no hay). Devuelve (velocidad objetivo [m/s], freno [%])."""
        if self.v_ref is None:
            self.v_ref = max(v_ego, 0.0)
        if gap is None or v_lider is None or math.isnan(gap) or math.isnan(v_lider):
            v_des = self.v_crucero
        else:
            v_des = v_lider + self.k_gap * (gap - (self.d_min + self.t_h * v_ego))
        v_des = min(max(v_des, 0.0), self.v_crucero)
        if v_des > self.v_ref:
            self.v_ref = min(v_des, self.v_ref + self.a_arranque * self.dt)
        else:
            self.v_ref = max(v_des, self.v_ref - self.a_frenado * self.dt)
        exceso = v_ego - self.v_ref
        a_frenar = 0.0 if exceso <= 0.3 else min(self.a_frenado, 1.2 * exceso)
        return self.v_ref, 100.0 * a_frenar / self.d_max

    def terminado(self, t):
        return self.reanudado and t - self.t_reanuda >= self.duracion_s

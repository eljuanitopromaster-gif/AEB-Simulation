"""Modelo hidráulico del freno (presión con rampa + retardo puro).

Origen: dinámica de frenado identificada en MATLAB a partir de datos experimentales
(aceleración, presión y tiempo) del vehículo "Fantástico".

Se usa con:  python run_scenario.py <ensayo> --actuador hidraulico
En esa corrida el AEB usa D_max = A_MAX (4.6 m/s²) en lugar del D_max del perfil.
"""
from collections import deque

import numpy as np


class ModeloFreno:
    K = 0.0994       # m/s² por bar
    R_SUBE = 42.0    # bar/s
    R_BAJA = 55.0    # bar/s
    RETARDO = 0.10   # s
    A_MAX = 4.6      # m/s², máximo medido en las pruebas

    def __init__(self, Ts, tabla_a, tabla_brake):
        self.Ts = Ts
        self.P = 0.0
        # retardo = round(RETARDO/Ts) pasos (con Ts = 0.05 -> 2 pasos = 0.10 s)
        self.buf = deque([0.0] * int(round(self.RETARDO / Ts)))
        orden = np.argsort(tabla_a)
        self.tabla_a = np.asarray(tabla_a, dtype=float)[orden]
        self.tabla_brake = np.asarray(tabla_brake, dtype=float)[orden]

    def paso(self, a_des):
        """a_des [m/s², positiva al frenar] -> (brake de CARLA por tabla [0-1], a_real [m/s²])."""
        a_des = min(max(a_des, 0.0), self.A_MAX)
        P_cmd = a_des / self.K
        dP = np.clip(P_cmd - self.P, -self.R_BAJA * self.Ts, self.R_SUBE * self.Ts)
        self.P += dP
        self.buf.append(self.P)
        a_real = self.K * self.buf.popleft()
        brake = float(np.interp(a_real, self.tabla_a, self.tabla_brake))
        return brake, a_real

    @classmethod
    def parametros(cls):
        return {"modelo": "hidraulico", "origen_datos": "Fantastico", "K_mps2_por_bar": cls.K,
                "subida_bar_s": cls.R_SUBE, "bajada_bar_s": cls.R_BAJA,
                "retardo_s": cls.RETARDO, "a_max_mps2": cls.A_MAX}

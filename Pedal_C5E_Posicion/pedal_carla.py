"""Adaptador SOLO para simulador CARLA; no interfaz con Nanotec físico."""
from __future__ import annotations
from dataclasses import asdict, dataclass
import json,socket
from pedal_model import PedalPosicion, Lectura

@dataclass
class RespuestaPedal:
    request_pct:float
    brake_fraction:float  # índice geométrico estimado; requiere calibrar contra freno real
    state:Lectura

class PedalCARLA:
    def __init__(self,telemetry=False,udp_host='127.0.0.1',udp_port=5055,mm_6900=20.):
        self.model=PedalPosicion.con_mm_por_6900(mm_6900)
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM) if telemetry else None
        self.addr=(udp_host,udp_port)
    def step(self,aeb_percent:float,dt_s:float):
        # 0-100 % del RANGO DE POSICIÓN VIRTUAL, NO frenado del vehículo validado.
        s=self.model.step(aeb_percent,dt_s)
        if self.sock:
            try:self.sock.sendto(json.dumps(asdict(s)).encode(),self.addr)
            except OSError:pass
        return RespuestaPedal(s.request_pct,s.brake_fraction,s)
    def close(self):
        if self.sock:self.sock.close();self.sock=None

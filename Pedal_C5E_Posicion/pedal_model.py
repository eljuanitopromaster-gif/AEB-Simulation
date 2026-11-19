"""Modelo DIGITAL C5-E Profile Position, CiA 402 (6060h = 1).

NO hardware, NO control de frenado real. Referencias de Bloom Drive en unidades
brutas (ticks de la grabación), NO demostradas equivalentes a unidades internas C5-E.
Motor, movimiento, fuerza y freno equivalente son ESTIMACIONES sin validación.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from math import radians, degrees, sin, cos, tan, atan2, sqrt, pi, isfinite
import bisect
import json
from pathlib import Path


def clamp(x, a, b):
    return max(a, min(b, x))


def interpolar_tabla(x, xs, ys):
    """Interpolación lineal con límites; no elimina saltos ni puntos próximos."""
    x=float(x)
    if x <= xs[0]: return float(ys[0])
    if x >= xs[-1]: return float(ys[-1])
    j=bisect.bisect_right(xs,x)
    a=(x-xs[j-1])/(xs[j]-xs[j-1])
    return float(ys[j-1]+a*(ys[j]-ys[j-1]))


@dataclass
class CalibracionBloom:
    dec_breakpoints: list[float]
    brake_map: list[float]
    bp_brake: list[float]
    table_ticks: list[float]

    @classmethod
    def cargar(cls, ruta=None):
        p=Path(ruta) if ruta else Path(__file__).with_name('calibracion_bloomdrive.json')
        d=json.loads(p.read_text(encoding='utf-8'))
        return cls(**{k:d[k] for k in ('dec_breakpoints','brake_map','bp_brake','table_ticks')})

    def desaceleracion_a_demanda_pct(self, magnitude_m_s2):
        return interpolar_tabla(max(0,float(magnitude_m_s2)), self.dec_breakpoints,self.brake_map)

    def demanda_pct_a_ticks(self, demanda_pct):
        return interpolar_tabla(clamp(float(demanda_pct),0,100),self.bp_brake,self.table_ticks)


@dataclass
class Geometria:
    largo_pedal_m: float=0.21                 # Medición facilitada por usuario
    punto_empuje_reposo_m: float=0.11       # Medición facilitada por usuario
    varilla_visible_reposo_m: float=0.07    # Medición facilitada por usuario
    angulo_reposo_deg: float=58.0           # HIPÓTESIS; no medido
    holgura_inicial_m: float=0.0            # En contacto en reposo
    carrera_mecanica_catalogo_m: float=0.0381 # Ficha: carrera MÍNIMA total, no restante
    carrera_disponible_desde_reposo_m: float=0.020 # HIPÓTESIS de trabajo
    tope_pedal_deg: float=12.0              # HIPÓTESIS
    juego_freno_deg: float=0.7              # HIPÓTESIS

    @property
    def altura_contacto_m(self):
        return self.punto_empuje_reposo_m*sin(radians(self.angulo_reposo_deg))
    @property
    def x_contacto_reposo_m(self):
        return self.punto_empuje_reposo_m*cos(radians(self.angulo_reposo_deg))
    @property
    def giro_reachable_rad(self):
        x=self.x_contacto_reposo_m-max(0,self.carrera_disponible_desde_reposo_m-self.holgura_inicial_m)
        return clamp(atan2(self.altura_contacto_m,max(0.000001,x))-radians(self.angulo_reposo_deg),0,radians(self.tope_pedal_deg))
    def angulo_desde_x_rad(self,x_m):
        x=max(0,x_m-self.holgura_inicial_m)
        return clamp(atan2(self.altura_contacto_m,max(1e-6,self.x_contacto_reposo_m-x))-radians(self.angulo_reposo_deg),0,radians(self.tope_pedal_deg))


@dataclass
class ControladorC5E:
    modo_cia402: int=1                      # Profile Position, NO Profile Torque
    # SON parámetros de simulación en unidades físicas de varilla. NO valores
    # directamente transmitibles a 6081h/6083h/6084h sin factor configurado.
    profile_velocity_6081_mm_s: float=30.0 # Referencia de catálogo a 24 V, no medida del montaje
    profile_acceleration_6083_mm_s2: float=120.0 # HIPÓTESIS
    profile_deceleration_6084_mm_s2: float=180.0 # HIPÓTESIS
    position_loop_time_constant_s: float=0.05   # HIPÓTESIS de seguimiento
    limite_comando_bloom_ticks: float=6900.0    # Límite OBSERVADO en SLP; verificar aplicación
    ticks_por_mm_bloom: float=345.0             # HIPÓTESIS: 6900 unidades ↔ 20 mm
    fuerza_maxima_simulada_n: float=938.9       # Empuje de ficha, NO garantizado en operación

    def __post_init__(self):
        if self.modo_cia402 != 1: raise ValueError('Este simulador es solo Profile Position (6060h=1).')
        for n in ('profile_velocity_6081_mm_s','profile_acceleration_6083_mm_s2','profile_deceleration_6084_mm_s2','position_loop_time_constant_s','limite_comando_bloom_ticks','ticks_por_mm_bloom'):
            if getattr(self,n)<=0:raise ValueError(f'{n} debe ser positivo')


@dataclass
class PedalFisico:
    muelle_pivote_nm_rad: float=0.4         # HIPÓTESIS
    rigidez_freno_nm_rad: float=65.0       # HIPÓTESIS
    no_lineal_freno_nm_rad2: float=110.0   # HIPÓTESIS
    corriente_nominal_motor_a: float=2.0  # Ficha 2 A/fase
    eficiencia_husillo: float=0.70        # HIPÓTESIS
    kt_nm_a: float=0.19                  # HIPÓTESIS; no se proporciona en ficha
    paso_husillo_m_vuelta: float=0.002   # Ficha 2 mm/rev

    @property
    def fuerza_por_amp_n(self):
        return 2*pi*self.eficiencia_husillo*self.kt_nm_a/self.paso_husillo_m_vuelta


@dataclass
class Lectura:
    time_s: float
    request_pct: float
    target_position_607A_ticks: float
    profile_position_6062_ticks: float
    actual_position_6064_ticks: float
    position_error_ticks: float
    position_pct: float
    target_rod_mm: float
    rod_displacement_mm: float
    exposed_rod_mm: float
    rod_velocity_mm_s: float
    pedal_angle_deg: float
    pedal_tip_travel_mm: float
    contact_force_n: float
    motor_current_a: float
    motor_speed_rpm: float
    brake_fraction: float       # SOLO estimación geométrica, NO presión hidráulica
    contact: bool
    at_forward_limit: bool
    saturation_warning: bool
    limite_carrera_mm: float
    control_mode: int


@dataclass
class PedalPosicion:
    geometry: Geometria=field(default_factory=Geometria)
    driver: ControladorC5E=field(default_factory=ControladorC5E)
    pedal: PedalFisico=field(default_factory=PedalFisico)
    time_s: float=0.0
    rod_x_m: float=0.0
    rod_v_m_s: float=0.0
    profile_x_m: float=0.0
    profile_v_m_s: float=0.0
    request_pct: float=0.0
    target_ticks: float=0.0
    saturated_target: bool=False

    def __post_init__(self):
        if self.geometry.carrera_disponible_desde_reposo_m<=0:
            raise ValueError('Carrera disponible debe ser positiva')
        if self.driver.limite_comando_bloom_ticks/self.driver.ticks_por_mm_bloom/1000 > self.geometry.carrera_disponible_desde_reposo_m+1e-9:
            raise ValueError('La escala ticks/mm supondría recorrido mayor al permitido; ajusta --mm-6900 y carrera.')

    @classmethod
    def con_mm_por_6900(cls, mm_6900=20.0, carrera_mm=None):
        if carrera_mm is None: carrera_mm=mm_6900
        return cls(geometry=Geometria(carrera_disponible_desde_reposo_m=carrera_mm/1000),
                   driver=ControladorC5E(ticks_por_mm_bloom=6900/mm_6900))

    def reset(self):
        self.time_s=self.rod_x_m=self.rod_v_m_s=self.profile_x_m=self.profile_v_m_s=0.0
        self.request_pct=self.target_ticks=0.0
        self.saturated_target=False

    def step(self, request_pct:float,dt_s:float=.02) -> Lectura:
        """Entrada 0–100% DE RECORRIDO DEL COMANDO (NO % presión real).

        Esta entrada normalizada se usa para experimentar sin saturar artificialmente
        50/75/100. Para la conversión REAL de Bloom por mapa, usa
        step_demanda_bloom_pct o step_desaceleracion, o reproduce el CSV directo.
        """
        if not isfinite(float(request_pct)):raise ValueError('Solicitud inválida')
        r=clamp(float(request_pct),0,100)
        return self.step_target_ticks(self.driver.limite_comando_bloom_ticks*r/100,dt_s,request_pct=r)

    def step_demanda_bloom_pct(self,demanda_frenado_pct,dt_s=.02,calibracion=None):
        c=calibracion or CalibracionBloom.cargar()
        ticks=c.demanda_pct_a_ticks(demanda_frenado_pct)
        return self.step_target_ticks(ticks,dt_s)

    def step_desaceleracion(self,desaceleracion_m_s2,dt_s=.02,calibracion=None):
        c=calibracion or CalibracionBloom.cargar()
        demanda=c.desaceleracion_a_demanda_pct(desaceleracion_m_s2)
        return self.step_target_ticks(c.demanda_pct_a_ticks(demanda),dt_s)

    def step_target_ticks(self,ticks:float,dt_s:float=.02,request_pct=None)->Lectura:
        """Acepta referencia en unidades Bloom, NO demuestra conversión a 607Ah real."""
        if not isfinite(float(ticks)) or not isfinite(float(dt_s)) or not 0<dt_s<=.2:
            raise ValueError('Ticks finitos y 0<dt<=0.2 s son requeridos')
        c,g,p=self.driver,self.geometry,self.pedal
        self.saturated_target=(ticks>c.limite_comando_bloom_ticks or ticks<0)
        self.target_ticks=clamp(float(ticks),0,c.limite_comando_bloom_ticks)
        self.request_pct=clamp(100*self.target_ticks/c.limite_comando_bloom_ticks,0,100) if request_pct is None else clamp(request_pct,0,100)
        target_m=min(g.carrera_disponible_desde_reposo_m,self.target_ticks/c.ticks_por_mm_bloom/1000)
        dt_left=float(dt_s)
        # Internos <=5 ms para dar una respuesta estable a señales de 100 Hz.
        while dt_left>1e-12:
            h=min(dt_left,.005)
            dt_left-=h
            # Generador de perfil triangular/trapezoidal acotado por 6081/3/4 (HIPÓTESIS de ajuste).
            ve=c.profile_velocity_6081_mm_s/1000
            ac=c.profile_acceleration_6083_mm_s2/1000
            de=c.profile_deceleration_6084_mm_s2/1000
            err=target_m-self.profile_x_m
            if abs(err)<1e-7 and abs(self.profile_v_m_s)<2e-4:
                self.profile_x_m=target_m
                self.profile_v_m_s=0.0
            else:
                # Distancia para detenerse antes del destino (perfil simétrico en ambos sentidos).
                v=self.profile_v_m_s
                braking_dist=v*v/(2*de)
                if abs(err)<=braking_dist+abs(v)*h:
                    wanted_v=0.0
                else:
                    wanted_v=(1 if err>0 else -1)*ve
                max_change=(de if v*wanted_v<=0 or abs(wanted_v)<abs(v) else ac)*h
                new_v=v+clamp(wanted_v-v,-max_change,max_change)
                new_x=self.profile_x_m+new_v*h
                if (target_m-self.profile_x_m)*(target_m-new_x)<=0:
                    new_x=target_m
                    new_v=0.0
                self.profile_x_m=clamp(new_x,0,g.carrera_disponible_desde_reposo_m)
                self.profile_v_m_s=new_v
            # Modelo virtual de seguimiento: perfil + constante de tiempo y aceleración finita.
            v_goal=clamp((self.profile_x_m-self.rod_x_m)/c.position_loop_time_constant_s,-ve,ve)
            acc=(ac if abs(v_goal)>abs(self.rod_v_m_s) and v_goal*self.rod_v_m_s>=0 else de)
            self.rod_v_m_s+=clamp(v_goal-self.rod_v_m_s,-acc*h,acc*h)
            old=self.rod_x_m
            new=clamp(old+self.rod_v_m_s*h,0,g.carrera_disponible_desde_reposo_m)
            if (self.profile_x_m-old)*(self.profile_x_m-new)<0 and abs(self.profile_x_m-target_m)<1e-7:
                new=self.profile_x_m;self.rod_v_m_s=0.0
            self.rod_x_m=new
            if new in (0,g.carrera_disponible_desde_reposo_m) and abs(new-old)<1e-12:
                self.rod_v_m_s=0.0
            self.time_s+=h
        theta=g.angulo_desde_x_rad(self.rod_x_m)
        g0=radians(g.juego_freno_deg)
        engagement=max(0,theta-g0)
        # Estimación casi-estática de resistencia del pedal: no reproduce fuerza experimental.
        tau=p.muelle_pivote_nm_rad*theta+p.rigidez_freno_nm_rad*engagement+p.no_lineal_freno_nm_rad2*engagement**2
        force=clamp(tau/max(g.altura_contacto_m,1e-6),0,c.fuerza_maxima_simulada_n)
        # Corriente equivalente inferida del modelo de husillo: NO lecturas C5-E.
        cur=clamp((force+7.0)/p.fuerza_por_amp_n,0,p.corriente_nominal_motor_a) if self.rod_x_m>1e-6 else 0.0
        reached_theta=max(1e-9,g.giro_reachable_rad-g0)
        brake=clamp(engagement/reached_theta,0,1)
        pos_pct=clamp(100*self.rod_x_m*1000*c.ticks_por_mm_bloom/c.limite_comando_bloom_ticks,0,100)
        return Lectura(time_s=self.time_s,request_pct=self.request_pct,
            target_position_607A_ticks=self.target_ticks,
            profile_position_6062_ticks=self.profile_x_m*1000*c.ticks_por_mm_bloom,
            actual_position_6064_ticks=self.rod_x_m*1000*c.ticks_por_mm_bloom,
            position_error_ticks=self.target_ticks-self.rod_x_m*1000*c.ticks_por_mm_bloom,
            position_pct=pos_pct,target_rod_mm=target_m*1000,
            rod_displacement_mm=self.rod_x_m*1000,
            exposed_rod_mm=g.varilla_visible_reposo_m*1000+self.rod_x_m*1000,
            rod_velocity_mm_s=self.rod_v_m_s*1000,
            pedal_angle_deg=degrees(theta),
            pedal_tip_travel_mm=g.largo_pedal_m*1000*(sin(radians(g.angulo_reposo_deg)+theta)-sin(radians(g.angulo_reposo_deg))),
            contact_force_n=force,motor_current_a=cur,
            motor_speed_rpm=self.rod_v_m_s/p.paso_husillo_m_vuelta*60,
            brake_fraction=brake,contact=self.rod_x_m>1e-6,
            at_forward_limit=self.rod_x_m>=g.carrera_disponible_desde_reposo_m-1e-6,
            saturation_warning=self.saturated_target,
            limite_carrera_mm=g.carrera_disponible_desde_reposo_m*1000,control_mode=1)

# Alias para compatibilidad con ejemplos anteriores; ya NO es un modelo de torque.
PedalAEB=PedalPosicion

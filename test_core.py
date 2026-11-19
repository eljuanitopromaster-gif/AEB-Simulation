"""
Pruebas del núcleo AEB. No necesitan CARLA ni ROS.

Uso:  python test_core.py
"""

import math
import random
import sys

import numpy as np

import config as C
from aeb_core import (AebPipeline, CrossingPredictor, Detections, EgoState, LeadState, LeadTracker, RadarTarget,
                      RequiredDecelAEB, StagedTTCAEB)
from aeb_core.logic import ALERTA, DETENIDO, FRENADO_PARCIAL, FRENADO_TOTAL, FRENANDO, INACTIVO, required_deceleration

ok = True
PERFIL = C.VEHICLES[C.VEHICLE]
D_MAX = PERFIL["d_max"]
AEB_PERFIL = dict(C.AEB_BTN, btn_on=PERFIL["btn_on"])


def check(name, cond, detail=""):
    global ok
    ok &= bool(cond)
    print(f"[{'OK' if cond else 'FALLA'}] {name:66s} {detail}")


def dr(d, v_c, v_e, a_e=0.0, a_l=0.0, tau=0.0, d0=0.0):
    return required_deceleration(d, v_c, v_e, a_e, a_l, tau, d0, 10.5)


def wall_scan(distance, closing):
    """Un objeto al frente: unas pocas detecciones a la altura del radar."""
    n = 5
    azi = np.linspace(-0.02, 0.02, n)
    return Detections(np.full(n, float(distance)), azi, np.zeros(n), np.full(n, -float(closing)))


def ground_scan(pitch_deg, mount_z=C.RADAR["mount_z"]):
    """Rebotes del piso plano, de 5 a 100 m, vistos por un radar con ese cabeceo."""
    depth = np.arange(5.0, 100.0, 1.0)
    alt = np.arcsin(-mount_z / depth) - math.radians(pitch_deg)
    return Detections(depth, np.zeros(depth.size), alt, np.full(depth.size, -22.0))


def closed_loop(pipeline, v0, d0, seed=None, radar_lost_at=None, t_max=15.0):
    """Lazo cerrado de masa puntual contra un objeto detenido: ego a 10 Hz y radar a 20 Hz con fluctuación."""
    rng = random.Random(seed)
    v, d, a, t = v0, d0, 0.0, 0.0
    tick, next_ego, next_radar = 0.05, 0.0, 0.0
    log = []
    while t < t_max and d > 0.0 and v > 0.05:
        if t >= next_ego:
            pipeline.on_ego(t, EgoState(v, a))
            next_ego += 0.10
        if t >= next_radar and (radar_lost_at is None or t < radar_lost_at):
            pipeline.on_radar(t, wall_scan(d, v))
            next_radar += 0.05 + (rng.uniform(-0.01, 0.01) if seed is not None else 0.0)
        out = pipeline.tick(t)
        log.append((t, d, v, out))
        a = -out.demand_pct / 100.0 * D_MAX
        d -= v * tick
        v = max(v + a * tick, 0.0)
        t += tick
    return d, v, log


# Desaceleración requerida: casos analíticos (τ = 0, d0 = 0)
x = dr(50, 20, 20)
check("Objeto estático: v²/(2d) = 400/100 = 4.0", abs(x - 4.0) < 1e-9, f"{x:.3f}")
x = dr(25, 10, 20)
check("Líder a velocidad constante: Δv²/(2d) = 100/50 = 2.0", abs(x - 2.0) < 1e-9, f"{x:.3f}")
x = dr(25, 0, 20, a_l=-4)
check("Líder que frena (igual velocidad): v²/(2(d+v²/2D_l)) = 2.667", abs(x - 400 / 150) < 1e-9, f"{x:.3f}")
x = dr(25, 0, 20, a_l=0)
check("Líder a igual velocidad sin frenar: 0", x == 0.0, f"{x:.3f}")
x = dr(10, 5, 20, a_l=-2)     # igualan velocidad en 2d/Δv = 4 s < 7.5 s que tarda el líder en detenerse
check("Líder que frena sin detenerse: D_l + Δv²/(2d) = 3.25", abs(x - 3.25) < 1e-9, f"{x:.3f}")
x = dr(40, 5, 20, a_l=-2)     # 2d/Δv = 16 s > 7.5 s: el líder se detiene antes
check("Líder que se detiene antes: v²/(2(d+v_l²/2D_l)) = 2.078", abs(x - 400 / (2 * (40 + 225 / 4))) < 1e-9,
      f"{x:.3f}")
check("Distancia agotada: D_max", dr(0.0, 10, 10) == 10.5)
xs = [dr(d / 10, 5, 20, a_l=-6) for d in range(600, 50, -1)]
check("Monótona: a menor distancia, mayor D_req", all(b >= a - 1e-9 for a, b in zip(xs, xs[1:])))
check("Continua entre casos (salto máximo < 0.2 m/s² por cada 0.1 m)",
      max(abs(b - a) for a, b in zip(xs, xs[1:])) < 0.2, f"{max(abs(b - a) for a, b in zip(xs, xs[1:])):.3f}")
check("La predicción por retardo aumenta la exigencia", dr(50, 20, 20, tau=0.3) > dr(50, 20, 20))

# Lógica continua
c = RequiredDecelAEB(AEB_PERFIL, d_max=D_MAX)
v, d, dt, d_cmd, cmds, states = 20.0, 80.0, 0.05, 0.0, [], []
while v > 0.0 and d > 0:
    st, demand, _ = c.step(True, d, v, v, -d_cmd, 0.0)
    d_cmd = c.d_cmd
    states.append(st)
    cmds.append(d_cmd)
    d -= v * dt
    v = max(v - d_cmd * dt, 0.0)
check("Objeto estático a 72 km/h (actuador ideal): se detiene sin chocar", d > 0, f"distancia final {d:.2f} m")
check("Pasa por ALERTA antes de frenar", ALERTA in states and states.index(ALERTA) < states.index(FRENANDO))
active = [x for x, s in zip(cmds, states) if s == FRENANDO]
check("Mando continuo (≥ 3 valores distintos) dentro de (0, D_max]",
      len(set(round(a, 2) for a in active)) >= 3 and all(0.0 < a <= D_MAX + 1e-9 for a in active),
      f"{min(active):.2f}–{max(active):.2f} m/s², {len(set(round(a, 2) for a in active))} valores")
c2 = RequiredDecelAEB(AEB_PERFIL, d_max=D_MAX)
st, demand, _ = c2.step(True, 30.0, -2.0, 20.0, 0.0, 0.0)
check("Objetivo que se aleja: no frena", st == INACTIVO and demand == 0.0)
st, demand, _ = c2.step(True, 5.0, 10.0, 2.0, 0.0, 0.0)
check("Bajo 10 km/h no interviene (R152 / FMVSS 127)", st == INACTIVO)

# Lógica escalonada por TTC
s = StagedTTCAEB(C.AEB_TTC)
seq = [s.step(True, 15.0, 10.0, 10.0)[0], s.step(True, 5.0, 8.0, 8.0)[0],
       s.step(True, 4.0, 6.0, 6.0)[0], s.step(True, 3.0, 0.2, 0.2)[0]]
check("Escalonada: parcial → total → enclavada → detenido",
      seq == [FRENADO_PARCIAL, FRENADO_TOTAL, FRENADO_TOTAL, DETENIDO])

# Cabeceo del radar y filtro de altura
PITCH = -0.35   # cabeceo medido en las pruebas de manejo (-0.35°)


def ground_target(mount_pitch_deg):
    p = RadarTarget(C.PERCEPTION, C.RADAR["mount_z"], C.RADAR["min_range"], math.radians(mount_pitch_deg))
    p.update(0.0, ground_scan(PITCH), 22.0)
    return p


raw = ground_target(0.0)
fixed = ground_target(PITCH)
d_false = required_deceleration(raw.distance, raw.closing_speed, 22.0, 0.0, 0.0,
                                C.AEB_BTN["tau"], C.AEB_BTN["d0"], D_MAX) if raw.valid else 0.0
check("Sin corregir el cabeceo, el piso parece un objeto y el BTN pasa el umbral",
      raw.valid and d_false / D_MAX >= PERFIL["btn_on"],
      f"a {raw.distance:.0f} m, BTN {d_false / D_MAX:.2f}" if raw.valid else "")
check("Corrigiendo el cabeceo, el piso no genera objetivo", not fixed.valid)
sm = RadarTarget(C.PERCEPTION, C.RADAR["mount_z"], C.RADAR["min_range"], math.radians(PITCH))
sm.update(0.0, ground_scan(PITCH - 1.0), 22.0, ego_pitch=math.radians(-1.0))
check("La inclinación durante una frenada (-1°) también se corrige", not sm.valid)

# Cadena completa
d, v, log = closed_loop(AebPipeline.from_config(C, vehicle=C.VEHICLE), 20.0, 90.0, seed=1)
check("Radar 20 Hz con fluctuación + ego 10 Hz: se detiene sin chocar", d > 0 and v <= 0.05,
      f"distancia final {d:.2f} m")
dem = [o.demand_pct for _, _, _, o in log if o.state == FRENANDO]
check("La demanda es continua y queda en 0–100 %", len(set(round(x, 1) for x in dem)) >= 3
      and all(0.0 <= x <= 100.0 for x in dem), f"{min(dem):.0f}–{max(dem):.0f} %")

_, _, log = closed_loop(AebPipeline.from_config(C, vehicle=C.VEHICLE), 20.0, 90.0, radar_lost_at=1.0, t_max=3.0)
after = [o for t, _, _, o in log if t > 1.0 + C.INPUTS["target_timeout"] + 0.05 * (C.PERCEPTION["hold_frames"] + 1)]
check("Radar perdido: se marca la falla y se pierde el objetivo",
      after and all(o.fault == "radar" and not o.target_valid and o.demand_pct == 0.0 for o in after))

pl = AebPipeline.from_config(C, vehicle=C.VEHICLE)
pl.on_ego(0.0, EgoState(20.0, 0.0))
pl.on_radar(0.0, wall_scan(8.0, 20.0))
pl.tick(0.0)
pl.on_radar(0.05, wall_scan(7.0, 20.0))
braking = pl.tick(0.05)
pl.on_radar(1.0, wall_scan(6.0, 20.0))
stale = pl.tick(1.0)
check("Estado propio vencido en plena frenada: demanda 0 y falla",
      braking.state == FRENANDO and stale.fault == "ego" and stale.demand_pct == 0.0,
      f"antes {braking.demand_pct:.0f} %")

pl = AebPipeline.from_config(C, vehicle=C.VEHICLE)
pl.on_ego(0.0, EgoState(float("nan"), 0.0))
check("Estado propio con NaN: demanda 0 y falla", pl.tick(0.0).fault == "ego")

pl = AebPipeline.from_config(C, vehicle=C.VEHICLE)
pl.on_ego(0.0, EgoState(0.1, 0.0))
pl.logic.state = DETENIDO
pl.on_radar(0.0, wall_scan(2.0, 0.1))
held = pl.tick(0.0)
pl.on_ego(0.05, EgoState(0.1, 0.0, driver_override=True))
released = pl.tick(0.05)
check("Retención en parada al 100 %; el conductor la libera",
      held.demand_pct == 100.0 and released.demand_pct == 0.0 and released.state == INACTIVO)

per = RadarTarget(C.PERCEPTION, C.RADAR["mount_z"], C.RADAR["min_range"])
per.update(1.0, wall_scan(30.0, 5.0), 20.0)
per.update(1.0, wall_scan(29.0, 5.0), 20.0)
check("Una medición repetida se ignora (dt = 0)", per.valid and abs(per.distance - 30.0) < 0.1, f"{per.distance:.2f} m")

# Entrada directa del líder
from actuator import BrakeActuator


def lead_loop(v0, gap, lead_decel, accel_known=True, seed=0, t_brake=1.0, t_max=15.0):
    """CCRb de masa puntual con el actuador modelado: líder y ego llegan a 10 Hz con ruido, el AEB corre a 20 Hz."""
    rng = random.Random(seed)
    pl = AebPipeline.from_config(C, vehicle=C.VEHICLE, source="lider")
    act = BrakeActuator(C.ACTUATOR, 0.05)
    v, vl, d, a, t, k = v0, v0, gap, 0.0, 0.0, 0
    outs = []
    while t < t_max and d > 0.0 and v > 0.05:
        al = -lead_decel if t >= t_brake and vl > 0 else 0.0
        if k % 2 == 0:
            a_meas = al + rng.gauss(0, 0.5) if accel_known else float("nan")
            pl.on_lead(t, LeadState(d + rng.gauss(0, 0.05), vl + rng.gauss(0, 0.1), a_meas))
            pl.on_ego(t, EgoState(v + rng.gauss(0, 0.1), a + rng.gauss(0, 0.3)))
        out = pl.tick(t)
        outs.append(out)
        a = -act.step(out.demand_pct) / 100.0 * D_MAX
        vl = max(vl + al * 0.05, 0.0)
        d -= (v - vl) * 0.05
        v = max(v + a * 0.05, 0.0)
        t += 0.05
        k += 1
    return d, v, outs


for known, label in ((True, "medida"), (False, "estimada de su velocidad")):
    res = [lead_loop(22.0, 25.0, 6.0, accel_known=known, seed=s) for s in range(5)]
    check(f"Líder frena a 6 m/s², 79 km/h, 25 m, a_líder {label}: 5 de 5 sin chocar",
          all(d > 0 for d, _, _ in res), "distancia final " + ", ".join(f"{d:.1f}" for d, _, _ in res) + " m")

pl = AebPipeline.from_config(C, vehicle=C.VEHICLE, source="lider")
pl.on_ego(0.0, EgoState(20.0, float("nan")))
pl.on_lead(0.0, None)
none_out = pl.tick(0.0)
check("Sin líder: no frena y no es falla; sin aceleración propia tampoco",
      none_out.fault == "" and none_out.demand_pct == 0.0 and not none_out.target_valid)
pl.on_lead(0.05, LeadState(30.0, 15.0, -4.0))
pl.tick(0.05)
pl.on_ego(0.40, EgoState(20.0, 0.0))
stale_lead = pl.tick(0.40)
check("Datos del líder vencidos: falla «lider»", stale_lead.fault == "lider")

lt = LeadTracker(C.PERCEPTION)
lt.update(0.0, LeadState(40.0, 20.0), 20.0)
lt.update(0.1, LeadState(40.0, 18.0), 20.0)          # el líder pierde 2 m/s en 0.1 s
a_before = lt.target_accel
lt.update(0.2, LeadState(12.0, 25.0), 20.0)          # otro vehículo se mete a 12 m
check("Un líder nuevo (salto de distancia) reinicia la estimación de su aceleración",
      a_before < -3.0 and lt.target_accel == 0.0 and lt.distance == 12.0, f"antes {a_before:.1f} m/s²")

# Freno virtual
from actuator import VirtualBrake


def brake_loop(plant_gain, t_end=3.0, lock_at=None):
    """El AEB pide 100 % con el actuador modelado. Planta de primer orden (0.1 s) con ganancia plant_gain."""
    vb = VirtualBrake(6.8, [(0.0, 0.0), (100.0, 10.0)], 0.05, C.VIRTUAL_BRAKE)
    act = BrakeActuator(C.ACTUATOR, 0.05)
    decel, hist = 0.0, []
    for _ in range(int(t_end / 0.05)):
        u = vb.to_carla_brake(act.step(100.0), decel)
        gain = plant_gain * (2.0 if lock_at is not None and len(hist) >= lock_at else 1.0)
        decel += (gain * 10.0 * u - decel) * 0.05 / 0.1
        hist.append(decel)
    return hist


for gain in (0.7, 1.15):
    h = brake_loop(gain)
    settled = next((i * 0.05 for i in range(len(h)) if all(abs(x - 6.8) < 0.3 for x in h[i:])), None)
    check(f"Freno virtual: el auto entrega {gain:.0%} de su curva y aun así se asienta en 6.8 ± 0.3 m/s² antes de 1 s",
          settled is not None and settled <= 1.0 and max(h) < 6.8 * 1.2,
          f"desde {settled} s, máximo {max(h):.2f} m/s²")

h = brake_loop(1.0, lock_at=30)
over = sum(x > 7.1 for x in h)
check("Freno virtual: si el auto de golpe frena el doble (bloqueo), el limitador lo baja de 7.1 m/s² en 0.2 s y lo asienta en D_max",
      abs(sum(h[-10:]) / 10 - 6.8) < 0.1 and over <= 4,
      f"final {sum(h[-10:]) / 10:.2f} m/s², {over * 0.05:.2f} s sobre 7.1 m/s², pico {max(h):.1f}")

# Demanda = BTN × 100 mientras frena, 100 % detenido
c3 = RequiredDecelAEB(AEB_PERFIL, d_max=D_MAX)
pares, st = [], INACTIVO
v3, d3 = 15.0, 40.0
while v3 > 0.0 and d3 > 0:
    st, demand, _ = c3.step(True, d3, v3, v3, -c3.d_cmd, 0.0)
    pares.append((st, demand, c3.btn))
    d3 -= v3 * 0.05
    v3 = max(v3 - c3.d_cmd * 0.05, 0.0)
st, demand, _ = c3.step(True, d3, 0.0, 0.0, 0.0, 0.0)
check("Demanda = BTN × 100 mientras frena y 100 % detenido",
      all(abs(dm - 100.0 * b) < 1e-9 for s, dm, b in pares if s == FRENANDO) and st == DETENIDO and demand == 100.0
      and any(s == FRENANDO for s, _, _ in pares),
      f"{sum(s == FRENANDO for s, _, _ in pares)} ciclos frenando")

# Predicción de cruce
def scan_objects(objs, v_ego=0.0, mount_z=C.RADAR["mount_z"], height=1.0):
    """Objetos pequeños en el marco del radar: 4 detecciones cada uno, a `height` m del piso.
    objs: (x, y) o (x, y, vx, vy) con la velocidad del objeto respecto del piso [m/s].
    Doppler = velocidad relativa proyectada en la línea de vista (negativo si se acerca)."""
    depth, azi, alt, vel = [], [], [], []
    for obj in objs:
        x, y = obj[:2]
        vxg, vyg = obj[2:] if len(obj) == 4 else (0.0, 0.0)
        if x < 1.0:
            continue
        for dx, dy in ((0.0, -0.15), (0.0, 0.15), (0.2, -0.1), (0.2, 0.1)):
            d = math.hypot(x + dx, y + dy)
            az, el = math.atan2(y + dy, x + dx), math.asin((height - mount_z) / d)
            depth.append(d)
            azi.append(az)
            alt.append(el)
            vel.append(((vxg - v_ego) * math.cos(az) + vyg * math.sin(az)) * math.cos(el))
    return Detections(np.array(depth, float), np.array(azi), np.array(alt), np.array(vel))


def crossing_run(x0, y0, vy, v_ego, steps=20, dt=0.05, vx=0.0):
    """Objeto que se mueve respecto del piso a (vx, vy) [m/s]; devuelve el predictor y si marcó amenaza."""
    cp = CrossingPredictor(C.PREDICCION_CRUCE, C.PERCEPTION["lane_half_width"], C.RADAR["mount_z"],
                           C.RADAR["min_range"], C.PERCEPTION["min_height"], C.PERCEPTION["max_height"])
    hist = []
    for k in range(steps):
        cp.update(k * dt, scan_objects([(x0 - (v_ego - vx) * k * dt, y0 + vy * k * dt, vx, vy)], v_ego), v_ego)
        hist.append(cp.amenaza is not None)
    return cp, hist


cp, hist = crossing_run(30.0, 4.6, -1.39, 8.33)
check("Cruce: peatón que camina hacia el carril → amenaza antes de entrar",
      any(hist) and cp.amenaza is not None and abs(cp.amenaza.vy_ground + 1.39) < 0.3 and abs(cp.amenaza.y) > 1.6,
      f"desde el ciclo {hist.index(True) if any(hist) else '-'}, vy {cp.amenaza.vy_ground if cp.amenaza else math.nan:+.2f} m/s")
cp, hist = crossing_run(30.0, -4.6, 1.39, 8.33)
check("Cruce: también desde la izquierda", any(hist))
cp, hist = crossing_run(30.0, 4.0, 0.0, 8.33, vx=1.25)
check("Banqueta: peatón que camina en paralelo (mismo sentido) → no es amenaza", not any(hist))
cp, hist = crossing_run(30.0, 4.0, 0.0, 8.33, vx=-1.25)
check("Banqueta: peatón que camina en paralelo (sentido contrario) → no es amenaza", not any(hist))
cp, hist = crossing_run(30.0, 3.0, 1.39, 8.33)
check("Peatón que se aleja del carril → no es amenaza", not any(hist))
cp, hist = crossing_run(40.0, 2.5, -1.39, 4.0)
check("Cruza y sale del carril mucho antes de que llegue el ego → no es amenaza", not any(hist))
cp, hist = crossing_run(30.0, 4.6, -1.39, 8.33, steps=4)
check("Recién visto (menos de edad_min ciclos) → aún no predice", not any(hist))
# objeto quieto (auto estacionado) cuya posición aparente "camina" hacia el carril: su Doppler propio es 0
cp, hist = crossing_run(30.0, 3.5, 0.0, 8.33)
cp2 = CrossingPredictor(C.PREDICCION_CRUCE, C.PERCEPTION["lane_half_width"], C.RADAR["mount_z"],
                        C.RADAR["min_range"], C.PERCEPTION["min_height"], C.PERCEPTION["max_height"])
deriva = []
for k in range(20):
    x_rel = 30.0 - 8.33 * k * 0.05
    cp2.update(k * 0.05, scan_objects([(x_rel, 3.5 - 0.8 * k * 0.05)], 8.33), 8.33)   # deriva aparente 0.8 m/s
    deriva.append(cp2.amenaza is not None)
check("Auto estacionado (Doppler propio 0) que parece acercarse → no es amenaza", not any(hist) and not any(deriva))
# con giro del ego: un peatón quieto parece moverse de lado; la compensación lo deja quieto
cp = CrossingPredictor(C.PREDICCION_CRUCE, C.PERCEPTION["lane_half_width"], C.RADAR["mount_z"],
                       C.RADAR["min_range"], C.PERCEPTION["min_height"], C.PERCEPTION["max_height"])
w, giro_hist = 0.06, []
for k in range(20):
    x_rel = 30.0 - 8.33 * k * 0.05
    cp.update(k * 0.05, scan_objects([(x_rel, 3.5 - w * x_rel * k * 0.05, 0.0, -0.3)], 8.33), 8.33, yaw_rate=w)
    giro_hist.append(cp.amenaza is not None)
check("Ego girando: peatón casi quieto al costado no se toma como cruce", not any(giro_hist))


def ped_loop(prediccion, v0=8.33, x0=30.0, y0=4.6, vy=-1.39, t_cruza=0.0, t_max=8.0):
    """Lazo cerrado (actuador ideal): el peatón cruza y se queda en medio del carril.
    Devuelve (distancia mínima en el carril, distancia al empezar a frenar, desaceleración máxima)."""
    pl = AebPipeline.from_config(C, logic="btn", prediccion_cruce=prediccion)
    v, s, t, a, t_brake, x_brake, a_max = v0, 0.0, 0.0, 0.0, None, None, 0.0
    gap_min = math.inf
    while t < t_max and v > 0.05:
        y = y0 if t < t_cruza else max(0.0, y0 + vy * (t - t_cruza))
        x = x0 - s
        if abs(y) < 1.0:
            gap_min = min(gap_min, x)
            if x <= 0.0:
                break
        pl.on_ego(t, EgoState(v, -a))
        moving = vy < 0 and y > 0.0 and t >= t_cruza
        pl.on_radar(t, scan_objects([(x, y, 0.0, vy if moving else 0.0)], v))
        out = pl.tick(t)
        if out.state == FRENANDO and t_brake is None:
            t_brake, x_brake = t, x
        a = out.d_cmd if out.state in (FRENANDO, DETENIDO) else 0.0
        if v > 2.0:
            a_max = max(a_max, a)      # en marcha (sin el último metro ni la retención al detenerse)
        s += v * 0.05
        v = max(v - a * 0.05, 0.0)
        t += 0.05
    return gap_min, x_brake, a_max


g_sin, xb_sin, a_sin = ped_loop(False, x0=24.0)
g_con, xb_con, a_con = ped_loop(True, x0=24.0)
check("Niño que cruza tarde a 30 km/h: sin predicción frena tarde y a fondo",
      xb_sin is not None and g_sin > 0, f"freno a {xb_sin:.1f} m, {a_sin:.1f} m/s², queda a {g_sin:.2f} m")
check("Con predicción: frena antes, más suave y sin tocarlo",
      xb_con is not None and xb_con > xb_sin + 3 and a_con < 0.7 * a_sin and g_con > 0,
      f"freno a {xb_con:.1f} m, {a_con:.1f} m/s², queda a {g_con:.2f} m")

check("El núcleo no carga CARLA ni ROS", not {"carla", "rclpy"} & set(sys.modules))

raise SystemExit(0 if ok else 1)

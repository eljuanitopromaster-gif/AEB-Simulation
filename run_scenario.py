"""Corre un ensayo de ensayos/ en CARLA y guarda log.csv, meta.json y video.

Uso (con CarlaUE4.exe abierto):
    python run_scenario.py PROPIO_FRENA_18_3453                # radar
    python run_scenario.py ENCAP_CCRb_6_12 --entrada lider     # datos del líder
    python run_scenario.py ENCAP_CCRb_6_12 --sin-aeb

Freno por defecto: pedal Nanotec + C5-E en Profile Position (6060h=1) de Pedal_C5E_Posicion/, con su ventana animada
(visualizador_pedal.py). --pedal torque usa el modelo anterior, Pedal_Nanotec_Torque (Profile Torque).
Para el freno anterior al pedal: --actuador modelo.

Columnas de log.csv: *_gt = verdad del simulador (solo para evaluar), radar_* = radar simulado,
*_est = estimado por la percepción.
"""

import argparse
import csv
import json
import math
import os
import socket
import subprocess
import sys
import time
from dataclasses import asdict
from datetime import datetime

import carla
import cv2
import numpy as np

import config as C
import escenario as E
import sim_utils as U
from actuator import BrakeActuator, VirtualBrake
from modelo_freno import ModeloFreno
from aeb_core import BRAKING_STATES, AebPipeline, LeadState
from analyze import results_root
from overlay import draw_hud, draw_schematic
from seguimiento import ReanudarSeguimiento
from traffic import RoadFrame, in_path_threat, lateral_reference, make_scripted, maneuver_events, peaton_cerca
from agents.navigation.controller import VehiclePIDController

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
# Modelos del pedal (--pedal). Los dos son el banco virtual Nanotec LGA561 + C5-E; cambia el modo del C5-E.
PEDALES = {
    "torque": {"origen": "Pedal_Nanotec_Torque", "modo": "Profile Torque", "kwargs": {"carrera_mm": 20.0},
               "sufijo": "-torque", "lado": False},
    "posicion": {"origen": "Pedal_C5E_Posicion", "modo": "Profile Position (6060h=1)", "kwargs": {"mm_6900": 20.0},
                 "sufijo": "", "lado": True},
}
PEDAL_POR_DEFECTO = "posicion"


def pedal_dir(pedal):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), PEDALES[pedal]["origen"])


PEDALES = {k: v for k, v in PEDALES.items() if os.path.isdir(pedal_dir(k))}   # solo los modelos presentes


LIVE_WINDOW = "AEB El Fantastico"
LIVE_SIZE = (960, 540)          # video en vivo a la izquierda; la ventana del pedal ocupa el resto a la derecha
VARIANTS = ("sinaeb", "ttc", "btn")


def attach_sensors(world, ego, record_video, radar_pitch_deg):
    bpl = world.get_blueprint_library()
    ext, off = ego.bounding_box.extent, ego.bounding_box.location

    radar_bp = bpl.find("sensor.other.radar")
    for key in ("range", "horizontal_fov", "vertical_fov", "points_per_second", "sensor_tick"):
        radar_bp.set_attribute(key, str(C.RADAR[key]))
    radar_tf = carla.Transform(carla.Location(x=off.x + ext.x + C.RADAR["mount_x_offset"], z=C.RADAR["mount_z"]),
                               carla.Rotation(pitch=radar_pitch_deg))
    radar = world.spawn_actor(radar_bp, radar_tf, attach_to=ego)

    collision = world.spawn_actor(bpl.find("sensor.other.collision"), carla.Transform(), attach_to=ego)

    camera = None
    if record_video:
        cam_bp = bpl.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(C.VIDEO["width"]))
        cam_bp.set_attribute("image_size_y", str(C.VIDEO["height"]))
        cam_bp.set_attribute("fov", str(C.VIDEO["fov"]))
        cam_tf = carla.Transform(carla.Location(x=-7.0, z=2.8), carla.Rotation(pitch=-10))
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=ego)
    return radar, collision, camera


def resolve_blueprint(bpl, spec, index):
    """Modelo de CARLA del actor: el que diga el ensayo, o el primero de config.TRAFFIC (el líder).

    Nunca el mismo modelo que el ego: CARLA comparte la configuración de llantas entre autos del mismo
    modelo y, al crear otro, la fricción del ego vuelve a la de fábrica (3.5) y se pierden el perfil y el clima.
    """
    ego_bp = C.VEHICLES[C.VEHICLE]["ego_blueprint"]
    pool = [b for b in C.TRAFFIC["blueprints"] if b != ego_bp]
    bp_id = spec.blueprint or pool[index % len(pool)]
    if bp_id == ego_bp:
        print(f"  Aviso: '{spec.name}' pedía {ego_bp}, el mismo modelo que el ego; se usa {C.TARGET_BLUEPRINT}")
        bp_id = C.TARGET_BLUEPRINT
    return bpl.find(bp_id if bpl.filter(bp_id) else C.TARGET_BLUEPRINT)


def carril_de_trafico(carla_map, frame, s, lat):
    """Sentido del carril de circulación en (s, lat) del marco del ego: "adelante", "contrario" o None si ahí no hay
    un carril de circulación (banqueta, camellón, fuera de la calle)."""
    tf = frame.transform_at(s, lat)
    wp = carla_map.get_waypoint(tf.location, project_to_road=False, lane_type=carla.LaneType.Driving)
    if wp is None or wp.is_junction or wp.transform.location.distance(tf.location) > 0.6 * wp.lane_width:
        return None
    dyaw = abs((wp.transform.rotation.yaw - frame.wp0.transform.rotation.yaw + 180.0) % 360.0 - 180.0)
    return "adelante" if dyaw < 90.0 else "contrario"


def spawn_scenario_actors(world, frame, ego, sc):
    bpl = world.get_blueprint_library()
    carla_map = world.get_map()
    scripted = []
    for i, spec in enumerate(sc.actors):
        bp = resolve_blueprint(bpl, spec, i)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", spec.name)
        # posición aproximada; la exacta se fija en la aproximación previa
        half = {"peaton": 0.3, "objeto": 0.7}.get(spec.kind, 2.5)
        s = (ego.bounding_box.extent.x + spec.gap + half) if spec.gap >= 0 else (spec.gap - ego.bounding_box.extent.x - half)
        if spec.lateral_m is not None:
            spec.lane = spec.lateral_m / frame.lane_width
        if spec.ambiente:
            sentido = carril_de_trafico(carla_map, frame, s, spec.lane * frame.lane_width)
            if sentido is None:
                print(f"  Tráfico: '{spec.name}' (carril {spec.lane:+.0f}) no tiene un carril de circulación ahí; se omite")
                continue
            spec.sentido = sentido
        tf = frame.transform_at(s, spec.lane * frame.lane_width)
        tf.location.z += {"peaton": 1.0, "objeto": 0.1}.get(spec.kind, 0.5)
        tf.rotation.roll = spec.roll
        if spec.ambiente and spec.sentido == "contrario":
            tf.rotation.yaw += 180.0
        actor = world.try_spawn_actor(bp, tf)
        if actor is None and spec.ambiente:
            print(f"  Tráfico: '{spec.name}' no cabía en su lugar; se omite")
            continue
        if actor is None and spec.kind == "vehiculo" and spec.fijo and spec.speed <= 0.0:
            # auto estacionado: prueba un poco más adelante o atrás; si no cabe, se omite (es ambiente)
            for ds in (0.5, -0.5, 1.0, -1.0):
                tf = frame.transform_at(s + ds, spec.lane * frame.lane_width)
                tf.location.z += 0.5
                actor = world.try_spawn_actor(bp, tf)
                if actor is not None:
                    break
            if actor is None:
                print(f"  Aviso: '{spec.name}' no cabía en su lugar; se omite")
                continue
        if actor is None and spec.kind == "peaton":
            # en una ciudad puede haber postes, árboles o mobiliario en la acera: prueba lugares cercanos
            for ds, dl in ((0.0, -0.5), (0.0, 0.5), (1.5, 0.0), (-1.5, 0.0), (0.0, -1.0), (0.0, 1.0)):
                tf = frame.transform_at(s + ds, spec.lane * frame.lane_width + dl)
                tf.location.z += 1.0
                actor = world.try_spawn_actor(bp, tf)
                if actor is not None:
                    print(f"  Aviso: '{spec.name}' no cabía en su lugar; se movió {ds:+.1f} m adelante y {dl:+.1f} m al lado")
                    break
        if actor is None:
            raise RuntimeError(f"No se pudo crear '{spec.name}' ({bp.id}) en s={s:.1f} m, lateral {spec.lane:+.2f}")
        scripted.append(make_scripted(spec, actor, frame))
    return scripted


def rel_speed(v1, v2):
    return math.sqrt((v1.x - v2.x) ** 2 + (v1.y - v2.y) ** 2 + (v1.z - v2.z) ** 2)


# hidraulico: ModeloFreno (datos del Fantástico); en esa corrida D_max = ModeloFreno.A_MAX
# pedal: banco virtual Nanotec LGA561 + C5-E (se importa solo si se usa), modelo según PEDALES / --pedal.
#   posicion: la demanda [%] es el % del rango de posición (0..6900 ticks Bloom); mm_6900 = mm de esos 6900 ticks
#   (SUPUESTO, igual que en el banco: 20 mm). torque: carrera de 20 mm.
ACTUATORS = {"modelo": C.ACTUATOR, "ideal": C.ACTUATOR, "hidraulico": ModeloFreno.parametros(), "pedal": {}}


def parametros_pedal(pedal):
    p = PEDALES[pedal]
    return {"origen": p["origen"], "modo": p["modo"], **p["kwargs"]}


def crear_pedal(pedal):
    """PedalCARLA del banco virtual; manda telemetría UDP a visualizador_pedal.py si está abierto."""
    carpeta = pedal_dir(pedal)
    if carpeta not in sys.path:
        sys.path.append(carpeta)          # al final: no tapa módulos del proyecto
    from pedal_carla import PedalCARLA
    return PedalCARLA(telemetry=True, **PEDALES[pedal]["kwargs"])


PEDAL_UDP_PORT = 5055


def _puerto_pedal_ocupado():
    """True si ya hay una ventana del pedal escuchando la telemetría."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", PEDAL_UDP_PORT))
        return False
    except OSError:
        return True
    finally:
        s.close()


def _ventanas_pedal():
    """{pid: carpeta del modelo} de las ventanas del pedal abiertas (procesos visualizador_pedal.py)."""
    salida = subprocess.run(
        ["powershell", "-NoProfile", "-Command",
         "Get-CimInstance Win32_Process -Filter \"Name like 'python%'\" | "
         "ForEach-Object { \"$($_.ProcessId)|$($_.CommandLine)\" }"],
        capture_output=True, text=True).stdout
    ventanas = {}
    for linea in salida.splitlines():
        pid, _, cmd = linea.partition("|")
        if "visualizador_pedal.py" in cmd:
            ventanas[int(pid)] = next((p["origen"] for p in PEDALES.values() if p["origen"] in cmd), "")
    return ventanas


def abrir_ventana_pedal(pedal, espera_s=10.0):
    """Abre visualizador_pedal.py del modelo del pedal (si no está abierto) y espera a que escuche.

    Si hay abierta una ventana del OTRO modelo, la cierra (se guarda sola al cerrarse): sus datos no coinciden.
    Es un proceso aparte: sigue abierto al terminar el ensayo, para repetir la prueba con ▶ REPETIR.
    """
    origen = PEDALES[pedal]["origen"]
    otras = [pid for pid, o in _ventanas_pedal().items() if o != origen]
    for pid in otras:
        subprocess.run(["taskkill", "/PID", str(pid)], capture_output=True)   # cierre normal, sin /F
    if otras:
        limite = time.perf_counter() + 5.0
        while _puerto_pedal_ocupado() and time.perf_counter() < limite:
            time.sleep(0.2)
        print(f"  Ventana del pedal: se cerró la de otro modelo; se abre la de {origen}")
    if _puerto_pedal_ocupado():
        print("  Ventana del pedal: ya estaba abierta, recibirá esta corrida")
        return
    carpeta = pedal_dir(pedal)
    lado = ["--lado", str(LIVE_SIZE[0])] if PEDALES[pedal]["lado"] else []
    subprocess.Popen([sys.executable, os.path.join(carpeta, "visualizador_pedal.py"), *lado], cwd=carpeta,
                     stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    limite = time.perf_counter() + espera_s
    while time.perf_counter() < limite:
        if _puerto_pedal_ocupado():
            print("  Ventana del pedal abierta")
            return
        time.sleep(0.2)
    print("  Aviso: la ventana del pedal no respondió; la corrida sigue sin ella")


def veredicto(evaluar, impact):
    """Criterio del ensayo: evaluar(velocidad de impacto en km/h, o None si no hubo contacto)."""
    return evaluar(impact["vrel_mps"] * 3.6 if impact else None)


def run(world, sc, evaluar, variant="btn", actuator_mode="modelo", record_video=True, live=False,
        vehicle=C.VEHICLE, source="radar", out_dir=None, ventana_pedal=True, prediccion_cruce=None, sufijo="",
        clima="despejado", pedal=PEDAL_POR_DEFECTO, carpeta="ensayos"):
    use_aeb = variant != "sinaeb"
    logic = "ttc" if variant == "ttc" else "btn"   # modo sombra
    nombre_actuador = actuator_mode + (PEDALES[pedal]["sufijo"] if actuator_mode == "pedal" else "")
    run_name = (f"{sc.run_id}_{variant}" + ("" if actuator_mode == "modelo" else f"_actuador-{nombre_actuador}")
                + ("_sin-prediccion" if prediccion_cruce is False else "") + sufijo
                + ("" if clima == "despejado" else f"_clima-{clima}"))
    out_dir = out_dir or os.path.join(results_root(vehicle, source), carpeta, run_name)
    os.makedirs(out_dir, exist_ok=True)
    print(f"\n=== {run_name} (perfil {vehicle}, entrada {source}) ===")

    carla_brake_map = U.carla_brake_map(vehicle)
    hidraulico = actuator_mode == "hidraulico"
    con_pedal = actuator_mode == "pedal"
    d_max = ModeloFreno.A_MAX if hidraulico else C.VEHICLES[vehicle]["d_max"]
    # Con el hidráulico o el pedal la dinámica ya está en su modelo: el freno virtual solo sigue
    # la posición entregada (sin su filtro de referencia, para no contar dos veces el retraso).
    virtual_brake_params = (dict(C.VIRTUAL_BRAKE, ref_tau=C.FIXED_DT) if hidraulico or con_pedal
                            else C.VIRTUAL_BRAKE)
    to_destroy = []
    writer_video = None
    modelo_pedal = None
    pedal_rows = []
    if con_pedal and ventana_pedal:
        abrir_ventana_pedal(pedal)
    try:
        carla_map = world.get_map()
        if sc.paso_peatonal_s is not None:
            wp0 = U.waypoint_en_paso_peatonal(world, sc.paso_peatonal_s)
        elif sc.inicio is not None:
            wp0 = carla_map.get_waypoint(carla.Location(x=sc.inicio[0], y=sc.inicio[1], z=0.5))
            print(f"  Mapa {carla_map.name.split('/')[-1]}: inicio en x = {sc.inicio[0]:g}, y = {sc.inicio[1]:g} "
                  f"(calle {wp0.road_id}, carril {wp0.lane_id})")
        else:
            wp0 = U.start_waypoint(world)
        frame = RoadFrame(wp0)
        ego = U.spawn_ego(world, wp0, vehicle)
        to_destroy.append(ego)
        scripted = spawn_scenario_actors(world, frame, ego, sc)
        # física y clima del ego DESPUÉS de crear los demás actores (que no la restablezcan) y comprobados
        U.apply_physics(ego, U.ego_setup(vehicle)[1])
        clima_info = U.aplicar_clima(world, ego, clima)
        print(f"  Clima: {clima_info['texto']} (agarre de las llantas del ego x{clima_info['agarre']:g}, "
              f"fricción {clima_info['tire_friction_despues'][0]:.2f})")
        to_destroy += [sa.actor for sa in scripted]
        by_id = {sa.actor.id: sa for sa in scripted}

        radar, collision, camera = attach_sensors(world, ego, record_video, C.VEHICLES[vehicle]["radar_pitch_deg"])
        to_destroy += [radar, collision, camera]
        radar_q = U.SensorQueue(radar)
        cam_q = U.SensorQueue(camera) if camera else None
        collisions = []
        collision.listen(lambda e: collisions.append(e))

        if live:
            # ventana del video más chica y arriba a la izquierda, para que se vea junto a la del pedal
            cv2.namedWindow(LIVE_WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(LIVE_WINDOW, *LIVE_SIZE)
            cv2.moveWindow(LIVE_WINDOW, 0, 0)
        if record_video:
            writer_video = cv2.VideoWriter(os.path.join(out_dir, "video.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                                           1.0 / C.FIXED_DT, (C.VIDEO["width"], C.VIDEO["height"]))

        # asentamiento (1 s) y condiciones iniciales
        for sa in scripted:
            if sa.spec.kind == "vehiculo":
                sa.actor.apply_control(carla.VehicleControl(hand_brake=True))
        for _ in range(int(1.0 / C.FIXED_DT)):
            world.tick()
        for sa in scripted:
            sa.start()

        driver = VehiclePIDController(ego, args_lateral=C.LATERAL_PID, args_longitudinal=C.LONGITUDINAL_PID,
                                      max_throttle=0.75, max_brake=0.0, max_steering=0.8)
        ego_half = ego.bounding_box.extent.x

        # en la aproximación previa los actores mantienen su distancia relativa
        def hold_all():
            ego_s = frame.s_of(ego.get_location())
            v_ego = U.forward_speed(ego)
            for sa in scripted:
                sa.hold(ego_s, ego_half, (v_ego - (0.0 if sa.stopped else sa.v_along)) * C.FIXED_DT)

        gear = U.run_up(world, carla_map, ego, driver, sc.ego_speed, on_tick=hold_all,
                        queues=[q for q in (radar_q, cam_q) if q])

        gap_at_start = in_path_threat(ego, scripted)[0]
        if sc.paso_peatonal_s is not None or sc.inicio is not None:
            print(f"  Aproximación previa: {frame.s_of(ego.get_location()):.1f} m recorridos")
        for sa in scripted:
            if sa.spec.kind == "peaton" and sa.spec.activar_a_m is not None:
                falta = frame.s_of(sa.actor.get_location()) - frame.s_of(ego.get_location()) - ego.bounding_box.extent.x
                if falta <= sa.spec.activar_a_m + 5.0:
                    print(f"  Aviso: al empezar, '{sa.spec.name}' está a {falta:.1f} m; casi no queda distancia antes "
                          f"del disparo ({sa.spec.activar_a_m:g} m). Aleja al peatón en el ensayo.")
        seg = None
        if sc.reanudar:
            seg = ReanudarSeguimiento(C.FIXED_DT, sc.reanudar_hasta_kmh / 3.6, C.VEHICLES[vehicle]["d_max"],
                                      duracion_s=sc.seguimiento_s)
        pipeline = AebPipeline.from_config(C, logic=logic, vehicle=vehicle, source=source, d_max=d_max,
                                           prediccion_cruce=prediccion_cruce)
        perception = pipeline.perception
        crossing = pipeline.crossing
        print(f"  Predicción de cruce: {'activa' if crossing else 'apagada'}")
        t_primera_pred = None
        if hidraulico:
            # La tabla de CARLA es (brake %, desaceleración); ModeloFreno pide (desaceleración, brake 0-1).
            freno_h = ModeloFreno(C.FIXED_DT, [a for _, a in carla_brake_map], [p / 100.0 for p, _ in carla_brake_map])
        elif con_pedal:
            modelo_pedal = crear_pedal(pedal)
            print(f"  Pedal Nanotec + C5-E {PEDALES[pedal]['modo']} ({PEDALES[pedal]['origen']}, virtual): "
                  f"telemetría en 127.0.0.1:{PEDAL_UDP_PORT}")
        else:
            actuator = BrakeActuator(ACTUATORS[actuator_mode], C.FIXED_DT, ideal=actuator_mode == "ideal")
        brake = VirtualBrake(d_max, carla_brake_map, C.FIXED_DT, virtual_brake_params)

        rows = []
        impact = None
        hay_lider = any(sa.spec.name == "lider" for sa in scripted)
        n_seen = 0
        prev_vel = {}
        t, t_end = 0.0, None
        aeb_braked = False
        verdict = None             # (cumple, texto)
        a_ritmo_real = live or (con_pedal and ventana_pedal)
        last_shown = time.perf_counter()
        while t < sc.duration:
            frame_id = world.tick()
            t += C.FIXED_DT

            # Observación
            ego_state = U.ego_state(ego)
            v_ego, a_ego = ego_state.speed, ego_state.accel
            pipeline.on_ego(t, ego_state)
            scan = U.radar_detections(radar_q.get(frame_id))
            if source == "radar":
                pipeline.on_radar(t, scan)
            else:
                d_lead, v_lead, a_lead, lead_name = in_path_threat(ego, scripted)
                pipeline.on_lead(t, LeadState(d_lead, v_lead, a_lead) if lead_name else None)

            # Retomar la marcha tras el frenado (solo ensayos con reanudar=True)
            follow_speed, follow_demand = None, 0.0
            if seg:
                if seg.actualizar(t, aeb_braked, not peaton_cerca(ego, scripted)):
                    pipeline.logic.reset()          # suelta la retención en parada del AEB
                    aeb_braked = False
                    print(f"  t = {t:.1f} s: camino libre; el ego retoma la marcha y sigue al líder")
                if seg.reanudado:
                    lead_gap, lead_v, _, lead_name = in_path_threat(ego, [sa for sa in scripted if sa.spec.kind == "vehiculo"])
                    follow_speed, follow_demand = seg.comando(v_ego, lead_gap if lead_name else None,
                                                              lead_v if lead_name else None)

            # Decisión
            out = pipeline.tick(t)
            state, demand = out.state, out.demand_pct

            # Actuación
            demand_cmd = max(demand if use_aeb else 0.0, follow_demand)
            if hidraulico:
                _, a_real = freno_h.paso(demand_cmd / 100.0 * d_max)
                position = a_real / d_max * 100.0     # freno entregado [% de D_max]
            elif con_pedal:
                lectura = modelo_pedal.step(demand_cmd, C.FIXED_DT).state
                position = lectura.brake_fraction * 100.0   # freno equivalente por ángulo del pedal [%]
                pedal_rows.append(dict(asdict(lectura), session_time_s=t))
            else:
                position = actuator.step(demand_cmd)
            aeb_active = use_aeb and state in BRAKING_STATES
            aeb_braked |= aeb_active
            resumed = bool(seg and seg.reanudado)
            lat_ref = lateral_reference(0, sc.ego_maneuvers, t, frame.lane_width)
            ego_s = frame.s_of(ego.get_location())
            target_kmh = (follow_speed if follow_speed is not None else sc.ego_speed) * 3.6
            ctrl = driver.run_step(target_kmh, frame.target(ego_s + max(5.0, 0.5 * v_ego), lat_ref))
            if aeb_active if resumed else aeb_braked:
                ctrl.throttle = 0.0   # el AEB anula al conductor virtual una vez que interviene
            elif gear and not resumed:
                ctrl.manual_gear_shift, ctrl.gear = True, gear   # crucero en marcha fija
            ctrl.brake = brake.to_carla_brake(position, -a_ego)   # lazo cerrado con la desaceleración medida
            ego.apply_control(ctrl)

            # Actores del escenario
            for sa in scripted:
                sa.ver_ego(t, ego)
                sa.step(t, C.FIXED_DT)

            # Verdad del simulador (evaluación)
            gap, v_tgt, a_tgt, threat = in_path_threat(ego, scripted)
            v_rel = v_ego - v_tgt
            hits = [(by_id.get(ev.other_actor.id), ev.other_actor) for ev in collisions[n_seen:]]
            if gap <= 0.0 and threat:       # respaldo geométrico (p. ej., objetos sin colisión física)
                hits.append((next(sa for sa in scripted if sa.spec.name == threat), None))
            for sa, other in hits:
                actor = sa.actor if sa else other
                if impact is None:
                    ego_v = prev_vel.get(ego.id, ego.get_velocity())
                    oth_v = prev_vel.get(actor.id, actor.get_velocity())
                    impact = {"vrel_mps": rel_speed(ego_v, oth_v), "with": sa.spec.name if sa else other.type_id}
                if sa:
                    sa.release()
            n_seen = len(collisions)
            prev_vel = {a.id: a.get_velocity() for a in [ego] + [sa.actor for sa in scripted]}
            collided = impact is not None

            row = {
                "t": round(t, 3), "frame": frame_id,
                "v_ego": v_ego, "a_ego": a_ego, "pitch_ego_deg": math.degrees(ego_state.pitch), "lat_ref_ego": lat_ref,
                "threat_gt": threat, "v_target_gt": v_tgt, "a_target_gt": a_tgt, "gap_gt": gap, "v_rel_gt": v_rel,
                "ttc_gt": gap / v_rel if v_rel > 0.1 else math.inf,
                "radar_valid": int(out.target_valid), "radar_points": perception.n_points,
                "radar_distance": out.distance, "radar_closing_speed": out.closing_speed,
                "v_target_est": out.target_speed, "a_target_est": out.target_accel,
                "ttc_radar": out.ttc, "d_req": out.d_req, "btn": out.btn, "d_cmd": out.d_cmd,
                "aeb_state": state, "demand_pct": demand if use_aeb else 0.0,
                "shadow_demand_pct": demand, "actuator_pct": position, "brake_target_decel": brake.target, "brake_limited": int(brake.limited),
                "brake_cmd": ctrl.brake, "throttle_cmd": ctrl.throttle,
                "collision": int(collided), "reanudado": int(resumed),
            }
            am = crossing.amenaza if crossing else None
            if am is not None and t_primera_pred is None:
                t_primera_pred = t
                print(f"  t = {t:.2f} s: predicción de cruce: objeto a {am.x_near:.1f} m, {am.y:+.1f} m al lado, "
                      f"{am.vy_ground:+.1f} m/s hacia el carril (entra en {crossing.t_entra:.1f} s, "
                      f"el ego llega en {crossing.t_llega:.1f} s)")
            row.update({
                "cruce_objetivo": int(out.cruce), "cruce_amenaza": int(am is not None),
                "cruce_x": am.x_near if am else math.nan, "cruce_y": am.y if am else math.nan,
                "cruce_vy": am.vy_ground if am else math.nan,
                "cruce_t_entra": crossing.t_entra if crossing else math.nan,
                "cruce_t_llega": crossing.t_llega if crossing else math.nan,
                "cruce_n_objetos": len(crossing.tracks) if crossing else 0,
            })
            if con_pedal:
                row.update({
                    "pedal_angle_deg": lectura.pedal_angle_deg, "pedal_rod_mm": lectura.rod_displacement_mm,
                    "pedal_force_n": lectura.contact_force_n, "pedal_current_a": lectura.motor_current_a,
                    # solo en Profile Position (Pedal_C5E_Posicion); en Profile Torque quedan vacías
                    "pedal_target_mm": getattr(lectura, "target_rod_mm", math.nan),
                    "pedal_position_pct": getattr(lectura, "position_pct", math.nan),
                    "pedal_error_ticks": getattr(lectura, "position_error_ticks", math.nan),
                    "pedal_rod_speed_mm_s": lectura.rod_velocity_mm_s,
                    "pedal_tope": int(lectura.at_forward_limit),
                    # solo en Profile Torque (Pedal_Nanotec_Torque) [‰ del par nominal]
                    "pedal_torque_target_permil": getattr(lectura, "target_torque_6071_permille", math.nan),
                    "pedal_torque_actual_permil": getattr(lectura, "torque_actual_6077_permille", math.nan),
                })
            rows.append(row)

            U.follow_with_spectator(world, ego)
            if cam_q:
                image = cam_q.get(frame_id)
                img = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)[:, :, :3].copy()
                inset = draw_schematic(frame, ego, scripted, radar, perception, threat, crossing)
                draw_hud(img, inset, sc, variant, row, len(scripted), collided, verdict, clima_info)
                writer_video.write(img)
                if live:
                    cv2.imshow(LIVE_WINDOW, img)
                    cv2.waitKey(1)
            if a_ritmo_real:
                # a velocidad real (video en vivo o ventana del pedal)
                time.sleep(max(0.0, C.FIXED_DT - (time.perf_counter() - last_shown)))
                last_shown = time.perf_counter()

            # Fin del ensayo
            if t_end is None:
                stopped = v_ego < 0.1 and t > 3.0
                # "más lento que el líder o sin objetivo delante" solo vale si hay líder; con un peatón que
                # todavía no llega al carril el ensayo terminaría con el ego en marcha
                matched = hay_lider and aeb_braked and (math.isnan(v_rel) or v_rel <= 0.05)
                fin = (collided or seg.terminado(t)) if seg else (collided or stopped or matched)
                if fin:
                    t_end = t + 1.5
                    verdict = veredicto(evaluar, impact)
            elif t >= t_end:
                break
            if t_end is None and t >= sc.duration - 1.5:
                verdict = veredicto(evaluar, impact)   # termina por tiempo: se muestra el resultado

        with open(os.path.join(out_dir, "log.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

        meta = {
            "run": run_name, "scenario": asdict(sc), "variant": variant, "aeb_enabled": use_aeb, "aeb_logic": logic,
            "vehicle": vehicle, "target_source": source, "radar_pitch_deg": C.VEHICLES[vehicle]["radar_pitch_deg"],
            "actuator": actuator_mode, "town": C.TOWN, "dt": C.FIXED_DT,
            "ego_blueprint": ego.type_id, "ego_physics_profile": U.ego_setup(vehicle)[1],
            "carla_brake_map": carla_brake_map,
            "actors": {sa.spec.name: sa.actor.type_id for sa in scripted},
            "events": maneuver_events(sc),
            "radar": C.RADAR, "perception": C.PERCEPTION,
            "prediccion_cruce": C.PREDICCION_CRUCE if crossing else None,
            "prediccion_cruce_primera_t": t_primera_pred,
            "aeb": pipeline.logic.p, "d_max": d_max,
            "actuator_params": parametros_pedal(pedal) if con_pedal else ACTUATORS[actuator_mode],
            "virtual_brake": virtual_brake_params,
            "ego_physics": U.physics_summary(ego),
            "clima": clima_info,
            "gap_at_start_m": gap_at_start,
            "reanudo_t": seg.t_reanuda if seg else None,
            "peaton_inicio_s": {sa.spec.name: sa.t_inicio for sa in scripted if sa.spec.kind == "peaton"},
            "collision": impact is not None,
            "impact_vrel_mps": impact["vrel_mps"] if impact else None,
            "collision_with": impact["with"] if impact else None,
        }
        friccion_final = meta["ego_physics"]["wheels_tire_friction"][0]
        if abs(friccion_final - clima_info["tire_friction_despues"][0]) > 0.01:
            print(f"  AVISO: la fricción de las llantas del ego terminó en {friccion_final:.2f} y no en "
                  f"{clima_info['tire_friction_despues'][0]:.2f}; esta corrida no respeta el perfil ni el clima")
        verdict = veredicto(evaluar, impact)
        meta["ensayo_cumple"], meta["ensayo_resultado"] = verdict
        with open(os.path.join(out_dir, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)

        braked = any(r["aeb_state"] in BRAKING_STATES for r in rows)
        outcome = (f"COLISIÓN con {impact['with']} a {impact['vrel_mps'] * 3.6:.1f} km/h" if impact
                   else "sin colisión")
        gaps = [r["gap_gt"] for r in rows if not math.isnan(r["gap_gt"])]
        print(f"  {outcome} | AEB {'intervino' if braked else 'no intervino'}"
              f"{'' if use_aeb else ' (sombra)'} | gap mínimo {min(gaps) if gaps else math.nan:.2f} m | {len(rows)} ciclos")
        print(f"  {sc.norma}\n  Criterio: {sc.criterio}\n  >>> {verdict[1]}")
        if pedal_rows:
            from reportes import exportar_resultados
            pedal_files = exportar_resultados(pedal_rows, os.path.join(out_dir, "pedal"), prefijo="carla")
            used = [r for r in pedal_rows if r["request_pct"] > 0]
            print(f"  Pedal: máx. pedido {max(r['request_pct'] for r in pedal_rows):.0f} % | "
                  f"máx. entregado {100 * max(r['brake_fraction'] for r in pedal_rows):.1f} % | "
                  f"varilla máx. {max(r['rod_displacement_mm'] for r in pedal_rows):.1f} mm | "
                  f"ciclos en el tope de carrera {sum(r['at_forward_limit'] for r in used)}")
            print(f"  Gráficas del pedal: {pedal_files['carpeta']}")
        return out_dir
    finally:
        if modelo_pedal:
            modelo_pedal.close()
        if live:
            cv2.destroyAllWindows()
        if writer_video:
            writer_video.release()
        U.destroy(to_destroy)
        world.tick()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ensayo", choices=E.disponibles(), help="archivo de ensayos/ (sin .py)")
    ap.add_argument("--entrada", choices=["radar", "lider"], default="radar",
                    help="radar: detecciones crudas; lider: distancia y velocidad del líder ya conocidas")
    ap.add_argument("--logica", choices=["btn", "ttc"], default=C.AEB_LOGIC, help="lógica AEB de una corrida")
    ap.add_argument("--sin-aeb", action="store_true", help="AEB en modo sombra (calcula pero no frena)")
    ap.add_argument("--actuador", choices=sorted(ACTUATORS), default="pedal",
                    help="pedal (por defecto): banco virtual Nanotec + C5-E (modelo según --pedal) y su ventana animada; "
                         "modelo: retardo 0.1 s y rampa 0.3 s; ideal: sin retardo ni rampa; "
                         "hidraulico: ModeloFreno (datos del Fantástico), D_max = 4.6 m/s²")
    ap.add_argument("--pedal", choices=sorted(PEDALES), default=PEDAL_POR_DEFECTO,
                    help="modelo del pedal: posicion (por defecto) = Pedal_C5E_Posicion, Profile Position; "
                         "torque = Pedal_Nanotec_Torque, Profile Torque (modelo anterior)")
    ap.add_argument("--sin-trafico", action="store_true",
                    help="quita el tráfico de ambiente del ensayo (para medir: con tráfico, CARLA puede dar un frenón "
                         "irreal al detenerse)")
    ap.add_argument("--sin-prediccion", action="store_true",
                    help="apaga la predicción de cruce (solo reacciona a lo que ya está en el carril)")
    ap.add_argument("--clima", choices=sorted(C.CLIMAS), default="despejado",
                    help="clima de CARLA; además escala el agarre de las llantas del ego (config.CLIMAS)")
    ap.add_argument("--sin-ventana-pedal", action="store_true",
                    help="con --actuador pedal, no abre la ventana animada del pedal")
    ap.add_argument("--sin-video", action="store_true")
    ap.add_argument("--ver", action="store_true", help="muestra el video y la telemetría en vivo, en una ventana")
    ap.add_argument("--abrir", action="store_true", help="al terminar genera las gráficas y abre el video y las gráficas")
    ap.add_argument("--descargas", action="store_true",
                    help="guarda la corrida en Descargas, en una carpeta con la fecha y hora (AAAA-MM-DD_HH-MM-SS)")
    ap.add_argument("--graficas", action="store_true", help="al terminar genera las gráficas, sin abrirlas")
    ap.add_argument("--carpeta", default="ensayos", metavar="NOMBRE",
                    help="carpeta dentro de results/<perfil>/ donde se guarda la corrida (por defecto: ensayos)")
    args = ap.parse_args()

    # Ensayo -> escenario
    ensayo = E.cargar(args.ensayo)
    esc = ensayo.escenario()
    if args.sin_trafico:
        esc.trafico = []
    if esc.mapa:
        C.TOWN = esc.mapa
    print(E.describir(esc))
    sc = E.a_simulacion(esc)

    out_dir = None
    if args.descargas:
        out_dir = os.path.join(os.path.expanduser("~"), "Downloads", datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    # Simulación en CARLA
    client, world = U.connect()
    with U.SyncMode(world):
        out_dir = run(world, sc, ensayo.evaluar, variant="sinaeb" if args.sin_aeb else args.logica,
                      actuator_mode=args.actuador, record_video=args.ver or not args.sin_video, live=args.ver,
                      source=args.entrada, out_dir=out_dir, ventana_pedal=not args.sin_ventana_pedal,
                      prediccion_cruce=False if args.sin_prediccion else None,
                      sufijo="_sin-trafico" if args.sin_trafico else "", clima=args.clima, pedal=args.pedal,
                      carpeta=args.carpeta)

    if args.abrir or args.graficas:
        import analyze
        try:
            analyze.plot_one(out_dir)
        except OSError:
            # Windows no deja sobrescribir graficas.png si el visor de imágenes lo tiene abierto
            print("  Aviso: no se pudo guardar graficas.png (¿está abierta en el visor de imágenes?). "
                  "Ciérrala y corre: python -c \"import analyze; analyze.plot_one(r'" + out_dir + "')\"")
    if args.abrir:
        for name in ("graficas.png", "video.mp4"):
            if os.path.exists(os.path.join(out_dir, name)):
                os.startfile(os.path.join(out_dir, name))

    # ensayo.bat muestra solo las líneas que empiezan con >>>
    with open(os.path.join(out_dir, "meta.json"), encoding="utf-8") as f:
        print(f">>> Resultado: {json.load(f)['ensayo_resultado']}")
    print(f">>> Archivos en: {out_dir}")


if __name__ == "__main__":
    main()

"""
Busca en el mapa cargado un paso peatonal sobre un tramo recto, para ubicar ahí el ensayo del peatón.

Devuelve el waypoint de partida del ego: un carril exterior (junto a la acera) tal que, `distancia_m` metros
adelante, hay un paso peatonal, y que antes y después del paso no hay cruces, curvas ni bifurcaciones.
"""

import carla


def poligonos(puntos):
    """get_crosswalks() devuelve todos los polígonos juntos; cada uno repite su primer punto para cerrarse."""
    polis, actual = [], []
    for p in puntos:
        if len(actual) >= 3 and abs(p.x - actual[0].x) < 1e-3 and abs(p.y - actual[0].y) < 1e-3:
            polis.append(actual)
            actual = []
        else:
            actual.append(p)
    if len(actual) >= 3:
        polis.append(actual)
    return polis


def dentro(x, y, poli):
    """Punto en polígono (2D)."""
    c, n = False, len(poli)
    for i in range(n):
        a, b = poli[i], poli[(i + 1) % n]
        if (a.y > y) != (b.y > y) and x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x:
            c = not c
    return c


def _giro(yaw, yaw0):
    return abs((yaw - yaw0 + 180.0) % 360.0 - 180.0)


def recto_adelante(wp, largo, paso=5.0, tol_yaw=4.0):
    """True si de wp hacia adelante hay `largo` m sin cruces, bifurcaciones ni curvas."""
    yaw0, w, d = wp.transform.rotation.yaw, wp, 0.0
    while d < largo - 1e-6:
        dd = min(paso, largo - d)
        sig = w.next(dd)
        if len(sig) != 1:
            return False
        w, d = sig[0], d + dd
        if w.is_junction or _giro(w.transform.rotation.yaw, yaw0) > tol_yaw:
            return False
    return True


def retroceder(wp, largo, paso=5.0):
    """Waypoint `largo` m atrás de wp por un camino sin cruces ni bifurcaciones; None si no hay."""
    w, d = wp, 0.0
    while d < largo - 1e-6:
        dd = min(paso, largo - d)
        ant = w.previous(dd)
        if len(ant) != 1:
            return None
        w, d = ant[0], d + dd
        if w.is_junction:
            return None
    return w


def _carriles_vecinos(wp):
    """wp y los carriles de conducción contiguos (a ambos lados) en el mismo punto."""
    out = [wp]
    for lado in ("get_left_lane", "get_right_lane"):
        w = wp
        for _ in range(4):
            w = getattr(w, lado)()
            if w is None or w.lane_type != carla.LaneType.Driving:
                break
            out.append(w)
    return out


def _es_exterior(wp):
    """Carril de la derecha del sentido de marcha: a su derecha ya no hay otro carril de conducción."""
    der = wp.get_right_lane()
    return der is None or der.lane_type != carla.LaneType.Driving


def buscar(mapa, distancia_m, despues_m=70.0, minimo_despues_m=40.0):
    """(waypoint_de_partida, descripción). Lanza RuntimeError si el mapa no tiene un paso peatonal adecuado."""
    polis = poligonos(mapa.get_crosswalks())
    for despues in (despues_m, minimo_despues_m):
        for poli in polis:
            cx = sum(p.x for p in poli) / len(poli)
            cy = sum(p.y for p in poli) / len(poli)
            cz = sum(p.z for p in poli) / len(poli)
            wp = mapa.get_waypoint(carla.Location(cx, cy, cz), project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
            if wp is None:
                continue
            for cand in _carriles_vecinos(wp):
                loc = cand.transform.location
                if cand.is_junction or not dentro(loc.x, loc.y, poli) or not _es_exterior(cand):
                    continue
                wp0 = retroceder(cand, distancia_m)
                if wp0 is not None and recto_adelante(wp0, distancia_m + despues):
                    return wp0, (f"paso peatonal en x={cx:.0f}, y={cy:.0f} (calle {cand.road_id}, carril {cand.lane_id}); "
                                 f"tramo recto de {distancia_m + despues:.0f} m")
    raise RuntimeError(
        "No encontré en el mapa un paso peatonal sobre un tramo recto de "
        f"{distancia_m + minimo_despues_m:.0f} m. Prueba con otro mapa (por ejemplo Town03, Town05 o Town10HD_Opt) "
        "o pon paso_peatonal=False en el ensayo.")

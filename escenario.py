"""Descripción de un ensayo y su traducción al formato del simulador.

Frenada: cuándo y con qué desaceleración frena el líder.
Vehiculo: el líder (separación inicial, velocidad y frenada).
Peaton: peatón que espera y cruza el carril del ego.
AutoEstacionado, PeatonAcera: ambiente de una calle de ciudad (autos estacionados y peatones en la acera).
AutoCirculando: tráfico de ambiente por otros carriles (mismo sentido o de frente, según el mapa).
Escenario: el ensayo completo.
"""

import importlib.util
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

ENSAYOS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ensayos")
KMH = 1 / 3.6
EGO_MEDIO_LARGO_M = 2.4          # mitad del largo del ego (Tesla Model 3 de CARLA), solo para ubicar el paso peatonal


# Clases que usa cada ensayo

@dataclass
class Frenada:
    a_los_s: float                  # instante en que el líder empieza a frenar, desde el inicio [s]
    desaceleracion: float           # [m/s²], positiva
    hasta_kmh: float = 0.0          # velocidad final (0 = hasta detenerse)


@dataclass
class Vehiculo:
    separacion_m: float             # defensa a defensa al inicio del ensayo [m]
    velocidad_kmh: float
    frenada: Optional[Frenada] = None
    carril: float = 0.0             # 0 = mismo carril que el ego (autos centrados)


@dataclass
class Peaton:
    """Peatón que espera quieto en el borde y cruza el carril del ego."""
    separacion_m: float             # del paragolpes del ego al peatón, al inicio del ensayo [m]
    lateral_m: float = 3.3          # del eje del carril del ego al lugar donde espera [m]
    velocidad_kmh: float = 5.0      # velocidad al cruzar
    hacia: str = "izquierda"        # "izquierda": espera a la derecha; "derecha": espera a la izquierda
    activar_a_m: Optional[float] = None   # empieza a cruzar cuando el ego está a esta distancia [m]; None: desde t = 0
    modelo: str = "walker.pedestrian.0001"
    se_detiene_en_m: Optional[float] = None  # se queda parado al llegar a este lateral (0 = eje del carril del ego) [m];
                                             # None: cruza sin detenerse


@dataclass
class AutoCirculando:
    """Tráfico de ambiente: auto que circula a velocidad constante por OTRO carril (nunca el del ego).
    El sentido lo da el mapa: si ese carril va en sentido contrario, el auto viene de frente.
    Si en ese lugar no hay un carril de circulación, el auto se omite (se avisa en la corrida)."""
    separacion_m: float             # a lo largo de la calle, desde la trompa del ego al empezar: + adelante, - atrás [m]
    carril: int                     # carriles respecto del ego: -1 = el de la izquierda, +1 = el de la derecha
    velocidad_kmh: float
    modelo: str = ""                # "" = uno de config.TRAFFIC["blueprints"]


@dataclass
class AutoEstacionado:
    """Auto quieto con freno de mano (escena con inicio fijo en el mapa)."""
    separacion_m: float             # de la trompa del ego, en su punto de partida, a la cola del auto [m]
    lateral_m: float = 4.0          # del eje del carril del ego al centro del auto [m], + derecha
    modelo: str = ""                # "" = uno de config.TRAFFIC["blueprints"]


@dataclass
class PeatonAcera:
    """Peatón que camina por la acera, paralelo a la calle, desde que empieza la corrida."""
    separacion_m: float             # de la trompa del ego, en su punto de partida, al peatón [m]
    lateral_m: float = 7.5          # del eje del carril del ego al peatón [m], + derecha
    velocidad_kmh: float = 4.5
    sentido: str = "adelante"       # "adelante" (mismo sentido que el ego) o "atras"
    modelo: str = "walker.pedestrian.0002"


@dataclass
class Escenario:
    nombre: str
    ego_kmh: float
    lider: Optional[Vehiculo] = None    # None: no hay líder
    duracion_s: float = 15.0        # límite de la corrida [s]
    referencia: str = ""            # documento, sección y página
    descripcion: str = ""
    criterio: str = ""
    peatones: List[Peaton] = field(default_factory=list)
    mapa: str = ""                  # "" = el de config.TOWN
    paso_peatonal: bool = False     # busca un paso peatonal recto del mapa y pone ahí al peatón
    aproximacion_m: float = 40.0    # distancia que se estima recorre el ego antes de empezar el ensayo
    reanudar: bool = False          # tras el frenado del AEB, el ego vuelve a seguir al líder
    reanudar_hasta_kmh: float = 30.0    # velocidad máxima al retomar la marcha
    seguimiento_s: float = 12.0         # el ensayo termina esta cantidad de segundos después de retomar
    inicio: Optional[Tuple[float, float]] = None    # (x, y) del mapa donde arranca el ego. Si se da, todos los
                                                    # actores quedan fijos en el mapa y sus separaciones se miden
                                                    # desde la trompa del ego en ese punto (quieto)
    estacionados: List[AutoEstacionado] = field(default_factory=list)
    peatones_acera: List[PeatonAcera] = field(default_factory=list)
    trafico: List[AutoCirculando] = field(default_factory=list)   # autos circulando por otros carriles (ambiente)


# Formato interno (run_scenario.py y traffic.py)

@dataclass
class Maneuver:
    kind: str                       # "brake"
    t: float                        # [s] instante de inicio
    decel: float = 0.0              # [m/s²]
    v_final: float = 0.0            # [m/s]
    to_lane: int = 0                # (no se usa: cambios de carril)
    duration: float = 0.0           # (no se usa)
    direction: str = ""             # (no se usa: peatones)
    speed: float = 0.0              # (no se usa)


@dataclass
class ActorSpec:
    name: str
    lane: float                     # carril relativo al ego (lateral = lane × ancho de carril)
    gap: float                      # separación inicial defensa a defensa [m]
    speed: float                    # [m/s]
    maneuvers: List[Maneuver] = field(default_factory=list)
    blueprint: Optional[str] = None # None: config.TARGET_BLUEPRINT / TRAFFIC["blueprints"]
    kind: str = "vehiculo"
    roll: float = 0.0
    lateral_m: Optional[float] = None   # si se da, reemplaza a lane: lateral en metros (+ derecha)
    activar_a_m: Optional[float] = None # peatón: empieza a caminar cuando el ego está a esta distancia [m]
    fijo: bool = False                  # no acompaña al ego en la aproximación previa (queda en su lugar del mapa)
    sentido: str = "adelante"           # peatón de acera: "adelante" o "atras"
    detener_lat_m: Optional[float] = None   # peatón que cruza: se queda parado al llegar a este lateral [m]
    ambiente: bool = False              # tráfico de ambiente: si no cabe o no hay carril, se omite


@dataclass
class Scenario:
    name: str
    ego_speed: float                # [m/s]
    actors: List[ActorSpec]
    ego_maneuvers: List[Maneuver] = field(default_factory=list)
    expect_brake: bool = True
    duration: float = 15.0
    description: str = ""
    norma: str = ""
    criterio: str = ""
    reanudar: bool = False          # tras el frenado del AEB, el ego vuelve a seguir al líder
    reanudar_hasta_kmh: float = 30.0
    seguimiento_s: float = 12.0
    paso_peatonal_s: Optional[float] = None   # distancia del ego al paso peatonal que se busca en el mapa [m]
    inicio: Optional[Tuple[float, float]] = None  # (x, y) del mapa donde arranca el ego

    @property
    def ego_kmh(self):
        return round(self.ego_speed * 3.6)

    @property
    def run_id(self):
        return f"{self.name}_{self.ego_kmh}kmh"


def a_simulacion(esc):
    """Traduce el Escenario del ensayo al formato interno del simulador."""
    actores, paso_s = [], None
    lid = esc.lider
    if lid:
        maniobras = []
        if lid.frenada:
            maniobras.append(Maneuver("brake", lid.frenada.a_los_s, decel=lid.frenada.desaceleracion,
                                      v_final=lid.frenada.hasta_kmh * KMH))
        actores.append(ActorSpec("lider", lid.carril, lid.separacion_m, lid.velocidad_kmh * KMH, maniobras))
    escena_fija = esc.inicio is not None
    for i, p in enumerate(esc.peatones):
        fijo = esc.paso_peatonal or escena_fija
        gap = p.separacion_m + (esc.aproximacion_m if esc.paso_peatonal else 0.0)
        lado = 1.0 if p.hacia == "izquierda" else -1.0      # espera a la derecha si cruza hacia la izquierda
        actores.append(ActorSpec("peaton" if len(esc.peatones) == 1 else f"peaton{i + 1}", 0.0, gap, 0.0,
                                 [Maneuver("walk", 0.0, direction=p.hacia, speed=p.velocidad_kmh * KMH)],
                                 blueprint=p.modelo, kind="peaton", lateral_m=lado * p.lateral_m,
                                 activar_a_m=p.activar_a_m, fijo=fijo, detener_lat_m=p.se_detiene_en_m))
        if esc.paso_peatonal and paso_s is None:
            paso_s = EGO_MEDIO_LARGO_M + gap + 0.3
    for i, a in enumerate(esc.estacionados):
        actores.append(ActorSpec(f"estacionado{i + 1}", 0.0, a.separacion_m, 0.0, blueprint=a.modelo or None,
                                 lateral_m=a.lateral_m, fijo=escena_fija))
    for i, p in enumerate(esc.peatones_acera):
        actores.append(ActorSpec(f"acera{i + 1}", 0.0, p.separacion_m, p.velocidad_kmh * KMH, blueprint=p.modelo,
                                 kind="peaton", lateral_m=p.lateral_m, fijo=escena_fija, sentido=p.sentido))
    for i, a in enumerate(esc.trafico):
        if a.carril == 0:
            raise ValueError(f"trafico[{i}]: el tráfico de ambiente no puede ir en el carril del ego (carril 0)")
        # sentido "adelante" provisional: al crearlo, run_scenario lo cambia a "contrario" si el mapa lo indica
        actores.append(ActorSpec(f"trafico{i + 1}", float(a.carril), a.separacion_m, a.velocidad_kmh * KMH,
                                 blueprint=a.modelo or None, fijo=escena_fija, ambiente=True))
    return Scenario(esc.nombre, esc.ego_kmh * KMH, actores, duration=esc.duracion_s,
                    description=esc.descripcion, norma=esc.referencia, criterio=esc.criterio,
                    reanudar=esc.reanudar, paso_peatonal_s=paso_s, inicio=esc.inicio,
                    reanudar_hasta_kmh=esc.reanudar_hasta_kmh, seguimiento_s=esc.seguimiento_s)


# Carga de ensayos

def disponibles():
    return sorted(os.path.splitext(f)[0] for f in os.listdir(ENSAYOS_DIR) if f.endswith(".py"))


def cargar(nombre):
    """Carga ensayos/<nombre>.py. Debe definir escenario() y evaluar(impacto_kmh)."""
    path = os.path.join(ENSAYOS_DIR, nombre + ".py")
    spec = importlib.util.spec_from_file_location(f"ensayo_{nombre}", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for fn in ("escenario", "evaluar"):
        if not callable(getattr(mod, fn, None)):
            raise ValueError(f"{path}: falta la función {fn}()")
    return mod


def describir(esc):
    """Lo que se va a simular, en texto, a partir del Escenario ya armado."""
    lid, v = esc.lider, esc.ego_kmh
    out = [f"Ensayo {esc.nombre}, cargado de ensayos\\{esc.nombre}.py",
           f"  Referencia: {esc.referencia}",
           f"  Ensayo:     {esc.descripcion}",
           f"  Ego:        {v:g} km/h"]
    if lid:
        out.append(f"  Líder:      {lid.velocidad_kmh:g} km/h, {lid.separacion_m:g} m delante (defensa a defensa)")
    if lid and lid.velocidad_kmh < v:
        out.append(f"              TTC al inicio = {lid.separacion_m / ((v - lid.velocidad_kmh) * KMH):.1f} s")
    if lid and lid.frenada:
        f = lid.frenada
        hasta = "detenerse" if f.hasta_kmh <= 0 else f"{f.hasta_kmh:g} km/h"
        out.append(f"              frena a {f.desaceleracion:g} m/s² desde t = {f.a_los_s:g} s, hasta {hasta}")
    for p in esc.peatones:
        cuando = "desde t = 0" if p.activar_a_m is None else f"cuando el ego está a {p.activar_a_m:g} m"
        para = "" if p.se_detiene_en_m is None else f"; se queda parado a {p.se_detiene_en_m:g} m del eje"
        out.append(f"  Peatón:     espera quieto a {p.lateral_m:g} m del eje y cruza hacia la {p.hacia} "
                   f"a {p.velocidad_kmh:g} km/h ({cuando}{para})")
    if esc.estacionados or esc.peatones_acera:
        out.append(f"  Ambiente:   {len(esc.estacionados)} autos estacionados, "
                   f"{len(esc.peatones_acera)} peatones caminando por la acera")
    if esc.trafico:
        out.append(f"  Tráfico:    {len(esc.trafico)} autos circulando por otros carriles (ambiente, no cambian la prueba)")
    if esc.mapa:
        out.append(f"  Mapa:       {esc.mapa}" + (" (paso peatonal recto, buscado en el mapa)" if esc.paso_peatonal else "")
                   + (f", inicio en x = {esc.inicio[0]:g}, y = {esc.inicio[1]:g}" if esc.inicio else ""))
    if esc.reanudar:
        out.append("  Después del frenado: el ego retoma la marcha y sigue al líder")
    out.append(f"  Criterio:   {esc.criterio}")
    return "\n".join(out)

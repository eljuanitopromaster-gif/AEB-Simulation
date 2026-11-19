# ENSAYO MI_ESCENARIO_PEATON_30KMH: un niño sale entre autos estacionados y se atraviesa frente al ego
# Calle de ciudad (Town05, calle recta de 200 m): el ego va por el carril derecho a 30 km/h, hay autos
# estacionados en el arcén y peatones caminando por la acera. Un niño que esperaba oculto entre dos autos
# estacionados cruza cuando el ego se acerca y se queda parado en medio del carril. El AEB lo detecta con el
# radar y debe detener el auto sin atropellarlo.
# Ensayo propio, inspirado en el CPNC de Euro NCAP (niño que sale entre autos estacionados, 5 km/h).

from escenario import AutoEstacionado, Escenario, Peaton, PeatonAcera

# Ego
VELOCIDAD_EGO_KMH = 30                 # velocidad urbana
DURACION_MAXIMA_S = 30                 # límite de la corrida (termina 1.5 s después de detenerse)

# Lugar: Town05, carril derecho de una calle recta de 200 m, junto al arcén y la acera
MAPA = "Town05"
INICIO_XY = (210.1, 98.1)              # donde arranca el ego (quieto); de ahí acelera hasta 30 km/h

# Autos estacionados en el arcén (todas las distancias: desde la trompa del ego en su punto de partida)
ESTACIONADOS_DESDE_M = 60.0
ESTACIONADOS_CADA_M = 7.5              # de un auto al siguiente (quedan ~2 m libres entre autos)
ESTACIONADOS_LATERAL_M = 4.0           # del eje del carril del ego al centro de los autos (a la derecha)
MODELOS_ESTACIONADOS = [
    "vehicle.toyota.prius", "vehicle.audi.a2", "vehicle.citroen.c3", "vehicle.dodge.charger_2020",
    "vehicle.mini.cooper_s_2021", "vehicle.nissan.patrol_2021", "vehicle.audi.etron", "vehicle.lincoln.mkz_2020",
    "vehicle.toyota.prius", "vehicle.citroen.c3", "vehicle.audi.a2",
]

# Niño que cruza: espera oculto entre el 6.º y el 7.º auto estacionado
HUECO_DEL_NINO = 6                     # entre el auto HUECO_DEL_NINO y el siguiente
NINO_LATERAL_M = 4.6                   # donde espera, a la derecha del eje del carril del ego
NINO_VELOCIDAD_KMH = 5.0               # Euro NCAP CPNC: 5 km/h
NINO_CRUZA_CUANDO_EGO_ESTA_A_S = 4.2   # empieza a cruzar cuando al ego le faltan estos segundos para llegar a él
                                      # (a 30 km/h: 35 m). En tiempo, para que siga siendo evitable si cambias la velocidad
NINO_SE_DETIENE_EN_M = 0.0             # se queda parado en el eje del carril del ego (0 = en medio)
NINO_MODELO = "walker.pedestrian.0011" # niño (1.1 m)

# Peatones caminando por la acera (ambiente): (distancia [m], lateral [m], velocidad [km/h], sentido, modelo)
PEATONES_ACERA = [
    (35.0, 7.2, 4.5, "adelante", "walker.pedestrian.0002"),
    (65.0, 8.4, 4.0, "atras", "walker.pedestrian.0003"),
    (85.0, 7.0, 5.0, "adelante", "walker.pedestrian.0004"),
    (110.0, 8.2, 4.5, "atras", "walker.pedestrian.0005"),
    (125.0, 7.4, 3.5, "atras", "walker.pedestrian.0006"),
    (145.0, 8.0, 4.5, "adelante", "walker.pedestrian.0007"),
]


# Escenario
def escenario():
    estacionados = [AutoEstacionado(separacion_m=ESTACIONADOS_DESDE_M + i * ESTACIONADOS_CADA_M,
                                    lateral_m=ESTACIONADOS_LATERAL_M, modelo=m)
                    for i, m in enumerate(MODELOS_ESTACIONADOS)]
    # centro del hueco entre dos autos estacionados, menos medio peatón
    centro_hueco = ESTACIONADOS_DESDE_M + (HUECO_DEL_NINO - 1) * ESTACIONADOS_CADA_M + 2.5 + ESTACIONADOS_CADA_M / 2
    nino = Peaton(separacion_m=centro_hueco - 0.3, lateral_m=NINO_LATERAL_M, velocidad_kmh=NINO_VELOCIDAD_KMH,
                  hacia="izquierda", activar_a_m=VELOCIDAD_EGO_KMH / 3.6 * NINO_CRUZA_CUANDO_EGO_ESTA_A_S,
                  se_detiene_en_m=NINO_SE_DETIENE_EN_M, modelo=NINO_MODELO)
    acera = [PeatonAcera(separacion_m=d, lateral_m=lat, velocidad_kmh=v, sentido=s, modelo=m)
             for d, lat, v, s, m in PEATONES_ACERA]
    return Escenario("MI_ESCENARIO_PEATON_30KMH", ego_kmh=VELOCIDAD_EGO_KMH, duracion_s=DURACION_MAXIMA_S,
                     referencia="Ensayo propio, inspirado en Euro NCAP CPNC (niño entre autos estacionados)",
                     descripcion=("Calle de ciudad con autos estacionados y peatones en la acera. Un niño sale entre "
                                  "dos autos estacionados a 5 km/h y se queda parado en el carril; el AEB frena"),
                     criterio="El ego se detiene sin tocar al peatón",
                     peatones=[nino], mapa=MAPA, inicio=INICIO_XY,
                     estacionados=estacionados, peatones_acera=acera)


# Criterio
def evaluar(impacto_kmh):
    """impacto_kmh: velocidad relativa del choque, o None si no hubo contacto."""
    if impacto_kmh is None:
        return True, "CUMPLE: se detuvo sin tocar al peatón"
    return False, f"NO CUMPLE: atropello a {impacto_kmh:.1f} km/h"

# ENSAYO MI_ESCENARIO_40KMH_PEATON: en una calle de ciudad (Town05), el lider pasa y, cuando el ego
# va a avanzar, un peaton que esperaba quieto en la acera cruza. Debe actuar el AEB; cuando el camino queda
# libre, el ego retoma la marcha y vuelve a seguir al lider.
# Ensayo propio. Mismos parametros del lider que MI_ESCENARIO_40KMH (excepto que el lider no se detiene del todo).

from escenario import Escenario, Frenada, Peaton, Vehiculo

# Lider
VELOCIDAD_EGO_KMH = 40
VELOCIDAD_LIDER_KMH = 40
SEPARACION_INICIAL_M = 40.0            # defensa a defensa
DESACELERACION_LIDER_MS2 = 6.0
LIDER_FRENA_A_LOS_S = 2.0              # 2 s a velocidad constante
LIDER_VELOCIDAD_FINAL_KMH = 25         # frena hasta esta velocidad y sigue; 0 = hasta detenerse
DURACION_MAXIMA_S = 40                 # limite de la corrida (termina antes: ver SEGUIMIENTO_S)

# Mapa: ciudad. Antes era Town03 con un paso peatonal buscado en el mapa, pero en esta PC CARLA se cierra al
# cargar Town03, y Town05 no tiene pasos peatonales sobre tramos rectos (12/11/2026). Ahora usa la misma calle
# recta de 200 m de Town05 que MI_ESCENARIO_PEATON_30KMH, y el peaton cruza a media calle (sin paso pintado).
MAPA = "Town05"
INICIO_XY = (210.1, 98.1)              # donde arranca el ego (quieto), carril derecho; de ahi acelera a 40 km/h

# Peaton: espera QUIETO en la acera y cruza solo cuando el ego esta a PEATON_CRUZA_CUANDO_EGO_ESTA_A_M
PEATON_ESPERA_LATERAL_M = 3.3          # distancia del eje del carril del ego a la acera, a la derecha
PEATON_CRUZA_CUANDO_EGO_ESTA_A_M = 48.0
PEATON_VELOCIDAD_KMH = 3.5             # camina lento: queda mas tiempo en el carril
PEATON_SEPARACION_M = 82.0             # desde la trompa del ego en INICIO_XY: ~60 m al empezar (acelera ~22 m antes)

# Despues del frenado
REANUDAR_SEGUIMIENTO = True            # cuando el peaton ya cruzo, el ego retoma la marcha y sigue al lider
VELOCIDAD_MAXIMA_AL_REANUDAR_KMH = 30
SEGUIMIENTO_S = 12                     # el ensayo termina esta cantidad de segundos despues de retomar


# Escenario
def escenario():
    lider = Vehiculo(separacion_m=SEPARACION_INICIAL_M,
                     velocidad_kmh=VELOCIDAD_LIDER_KMH,
                     frenada=Frenada(a_los_s=LIDER_FRENA_A_LOS_S,
                                     desaceleracion=DESACELERACION_LIDER_MS2,
                                     hasta_kmh=LIDER_VELOCIDAD_FINAL_KMH))
    peaton = Peaton(separacion_m=PEATON_SEPARACION_M, lateral_m=PEATON_ESPERA_LATERAL_M,
                    velocidad_kmh=PEATON_VELOCIDAD_KMH, hacia="izquierda",
                    activar_a_m=PEATON_CRUZA_CUANDO_EGO_ESTA_A_M)
    return Escenario("MI_ESCENARIO_40KMH_PEATON", ego_kmh=VELOCIDAD_EGO_KMH, lider=lider,
                     duracion_s=DURACION_MAXIMA_S,
                     referencia="Ensayo propio",
                     descripcion=("Calle de ciudad. Ambos a 40 km/h; a los 2 s el lider frena "
                                  "a 6 m/s2 hasta 25 km/h. Un peaton quieto en la acera cruza a 3.5 km/h al acercarse "
                                  "el ego; el AEB frena. Cuando el peaton ya cruzo, el ego retoma y sigue al lider"),
                     criterio="Sin contacto con el peaton ni con el lider",
                     peatones=[peaton], mapa=MAPA, inicio=INICIO_XY,
                     reanudar=REANUDAR_SEGUIMIENTO, reanudar_hasta_kmh=VELOCIDAD_MAXIMA_AL_REANUDAR_KMH,
                     seguimiento_s=SEGUIMIENTO_S)


# Criterio
def evaluar(impacto_kmh):
    """impacto_kmh: velocidad relativa del choque, o None si no hubo contacto."""
    if impacto_kmh is None:
        return True, "CUMPLE: sin contacto"
    return False, f"NO CUMPLE: contacto a {impacto_kmh:.1f} km/h"

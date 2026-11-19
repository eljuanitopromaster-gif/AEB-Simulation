# ENSAYO MI_ESCENARIO_40KMH: el lider frena hasta detenerse
# Ensayo propio.

from escenario import Escenario, Frenada, Vehiculo

# Parametros
VELOCIDAD_EGO_KMH = 40
VELOCIDAD_LIDER_KMH = 40
SEPARACION_INICIAL_M = 40.0            # defensa a defensa
DESACELERACION_LIDER_MS2 = 6.0
LIDER_FRENA_A_LOS_S = 2.0              # 2 s a velocidad constante
DURACION_MAXIMA_S = 15                 # limite de la corrida


# Escenario
def escenario():
    lider = Vehiculo(separacion_m=SEPARACION_INICIAL_M,
                     velocidad_kmh=VELOCIDAD_LIDER_KMH,
                     frenada=Frenada(a_los_s=LIDER_FRENA_A_LOS_S,
                                     desaceleracion=DESACELERACION_LIDER_MS2))
    return Escenario("MI_ESCENARIO_40KMH", ego_kmh=VELOCIDAD_EGO_KMH, lider=lider,
                     duracion_s=DURACION_MAXIMA_S,
                     referencia="Ensayo propio",
                     descripcion="Ambos a 40 km/h; a los 2 s, el lider frena a 6 m/s2 hasta detenerse",
                     criterio="Sin contacto con el lider")


# Criterio
def evaluar(impacto_kmh):
    """impacto_kmh: velocidad relativa del choque, o None si no hubo contacto."""
    if impacto_kmh is None:
        return True, "CUMPLE: sin contacto"
    return False, f"NO CUMPLE: contacto a {impacto_kmh:.1f} km/h"

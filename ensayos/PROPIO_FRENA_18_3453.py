# ENSAYO PROPIO_FRENA_18_3453: el líder frena hasta detenerse, 34.53 m delante
# Ensayo definido por el equipo; no corresponde a ninguna norma.

from escenario import AutoCirculando, Escenario, Frenada, Vehiculo

# Parámetros
VELOCIDAD_EGO_KMH = 18
VELOCIDAD_LIDER_KMH = 18
SEPARACION_INICIAL_M = 34.53           # defensa a defensa
DESACELERACION_LIDER_MS2 = 3.87
LIDER_FRENA_A_LOS_S = 1.0              # 1 s a velocidad constante
DURACION_MAXIMA_S = 15                 # límite de la corrida

# Tráfico de ambiente (solo para que la escena se vea natural; no cambia la prueba):
# (separación a lo largo de la calle [m] (+ adelante, - atrás), carril (-1 izquierda, +1 derecha), velocidad [km/h]).
# Nunca en el carril del ego. Si un carril no existe en el mapa se omite; si va en sentido contrario, viene de frente.
# En cada carril el auto de adelante va igual o más rápido que el de atrás, para que no se alcancen.
TRAFICO = [
    (25.0, -1, 28), (-15.0, -1, 26),
    (12.0, +1, 22), (-20.0, +1, 16),
    (55.0, -2, 32), (5.0, -2, 30),
    (40.0, +2, 20), (-10.0, +2, 18),
]


# Escenario
def escenario():
    lider = Vehiculo(separacion_m=SEPARACION_INICIAL_M,
                     velocidad_kmh=VELOCIDAD_LIDER_KMH,
                     frenada=Frenada(a_los_s=LIDER_FRENA_A_LOS_S,
                                     desaceleracion=DESACELERACION_LIDER_MS2))
    return Escenario("PROPIO_FRENA_18_3453", ego_kmh=VELOCIDAD_EGO_KMH, lider=lider,
                     duracion_s=DURACION_MAXIMA_S,
                     referencia="Ensayo propio | definido por el equipo (5/11/2026)",
                     descripcion="Ambos a 18 km/h; al segundo, el líder frena a 3.87 m/s² hasta detenerse",
                     criterio="Sin contacto con el líder",
                     trafico=[AutoCirculando(separacion_m=s, carril=c, velocidad_kmh=v) for s, c, v in TRAFICO])


# Criterio
def evaluar(impacto_kmh):
    """impacto_kmh: velocidad relativa del choque, o None si no hubo contacto."""
    if impacto_kmh is None:
        return True, "CUMPLE: sin contacto"
    return False, f"NO CUMPLE: contacto a {impacto_kmh:.1f} km/h"


# Supuestos:
#   - El conductor del ego no frena ni esquiva: solo actúa el AEB.
#   - El líder alcanza 3.87 m/s² al instante, sin rampa de frenado.
#   - Ego: Tesla Model 3 de CARLA con la física del Porsche (2,605 kg).

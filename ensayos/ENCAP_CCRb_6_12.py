# ENCAP_CCRb_6_12: el líder frena a 6 m/s², 12 m delante
# Euro NCAP, "Test Protocol - AEB Car-to-Car systems" v4.3 (dic. 2023), CCRb, §8.2.2.3, p. 18

from escenario import AutoCirculando, Escenario, Frenada, Vehiculo

# Parámetros
VELOCIDAD_EGO_KMH = 50          # §8.2.2.3, p. 18: "fixed speed of 50 km/h for both VUT and GVT"
VELOCIDAD_LIDER_KMH = 50        # §8.2.2.3, p. 18: igual que el ego
SEPARACION_INICIAL_M = 12       # §8.2.2.3, p. 18: "12 and 40 m headway"
                                #   tolerancia ± 0.5 m (§8.4.2, p. 25)
DESACELERACION_LIDER_MS2 = 6.0  # §8.2.2.3, p. 18: "-2 and -6 m/s²", alcanzada en 1 s como máximo
LIDER_FRENA_A_LOS_S = 1.0       # §8.2.2.3, p. 18: T0 = 1 s antes de que el líder empiece a frenar
DURACION_MAXIMA_S = 15          # límite de la corrida (no es parte de la norma)

# Tráfico de ambiente (NO es parte del protocolo Euro NCAP, que se corre en pista sin otros autos; solo para que la
# escena se vea natural): (separación a lo largo de la calle [m] (+ adelante, - atrás), carril (-1 izquierda,
# +1 derecha), velocidad [km/h]). Nunca en el carril del ego. Si un carril no existe en el mapa se omite; si va en
# sentido contrario, viene de frente. En cada carril el auto de adelante va igual o más rápido que el de atrás.
TRAFICO = [
    (30.0, -1, 62), (-20.0, -1, 58),
    (15.0, +1, 48), (-35.0, +1, 45),
    (60.0, -2, 66), (0.0, -2, 60),
    (45.0, +2, 52), (-25.0, +2, 50),
]
# Autos centrados (traslape 100 %), §8.2.2.3, p. 18.
# Tolerancias (§8.4.2, p. 25): ego +1.0 km/h; desvío lateral 0 ± 0.05 m.
# Fin del ensayo (§8.4.3, p. 25): el ego se detiene, queda más lento que el líder, o hay contacto.


# Escenario
def escenario():
    lider = Vehiculo(separacion_m=SEPARACION_INICIAL_M,
                     velocidad_kmh=VELOCIDAD_LIDER_KMH,
                     frenada=Frenada(a_los_s=LIDER_FRENA_A_LOS_S,
                                     desaceleracion=DESACELERACION_LIDER_MS2))
    return Escenario("ENCAP_CCRb_6_12", ego_kmh=VELOCIDAD_EGO_KMH, lider=lider,
                     duracion_s=DURACION_MAXIMA_S,
                     referencia="Euro NCAP | AEB Car-to-Car v4.3 (2023), §8.2.2.3, p. 18",
                     descripcion="El líder va a 50 km/h y frena a 6 m/s² con 12 m de separación",
                     criterio="Euro NCAP puntúa la reducción de velocidad; ideal: sin contacto",
                     trafico=[AutoCirculando(separacion_m=s, carril=c, velocidad_kmh=v) for s, c, v in TRAFICO])


# Criterio
def evaluar(impacto_kmh):
    """Euro NCAP no aprueba ni reprueba un ensayo: puntúa la reducción de velocidad.
    impacto_kmh: velocidad relativa del choque, o None si no hubo contacto."""
    if impacto_kmh is None:
        return True, "SIN CONTACTO (evitación completa)"
    return False, f"CONTACTO a {impacto_kmh:.1f} km/h (reducción {VELOCIDAD_EGO_KMH - impacto_kmh:.1f} km/h)"


# Diferencias con el ensayo real:
#   - Objetivo: auto de CARLA en lugar del Global Vehicle Target (GVT, ISO 19206-3).
#   - El líder alcanza los 6 m/s² al instante; el protocolo le da hasta 1 s (§8.2.2.3, p. 18).
#   - Datos a 20 Hz; el protocolo pide al menos 100 Hz (§4.1.1, p. 8).
#   - Velocidad inicial del ego: 49-50 km/h; el protocolo admite +1.0 km/h (§8.4.2, p. 25).

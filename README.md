# AEB "El Fantástico" en CARLA, con el pedal Nanotec + C5-E y cambios de clima

Simulación de un sistema de frenado automático de emergencia (AEB) para un Porsche Taycan 4S ("El Fantástico") en el simulador **CARLA 0.9.16**. El AEB detecta el obstáculo con un radar simulado, calcula cuánto hay que frenar (BTN) y ese freno pasa por un **modelo del actuador real del pedal** (motor lineal Nanotec LGA561 + controlador C5-E en *Profile Position*, carpeta `Pedal_C5E_Posicion/`) antes de llegar a CARLA. Cada ensayo se puede correr con **cuatro climas** (despejado, lluvia, niebla y tormenta de noche) y con o sin tráfico en los carriles de los lados.

> El modelo del pedal es solo de simulación: no se conecta al hardware ni sirve para frenar un vehículo real.

## Qué se necesita

| | |
|---|---|
| Sistema | Windows 10/11 con tarjeta de video dedicada |
| Memoria | ~5 GB de RAM libres al abrir CARLA (cierra Chrome y programas pesados) |
| CARLA | **0.9.16** para Windows ([descarga](https://github.com/carla-simulator/carla/releases/tag/0.9.16)), descomprimido en cualquier carpeta |
| Python | **3.12** (el paquete de CARLA 0.9.16 es para 3.12), con el lanzador `py` |

## Instalación (una sola vez)

1. Clona este repositorio:
   ```bat
   git clone https://github.com/eljuanitopromaster-gif/AEB-Simulation.git
   cd AEB-Simulation
   ```
2. Indica dónde está CARLA (la carpeta que tiene `CarlaUE4.exe`) y **abre una consola nueva**:
   ```bat
   setx CARLA_ROOT "C:\CARLA_0.9.16"
   ```
3. Instala todo (crea `venv\`, instala las librerías y la API de CARLA, y corre las pruebas del AEB):
   ```bat
   instalar.bat
   ```

## Correr un ensayo

```bat
ensayo.bat ENCAP_CCRb_6_12
ensayo.bat ENCAP_CCRb_6_12 --clima tormenta_noche
ensayo.bat PROPIO_FRENA_18_3453 --clima lluvia --sin-trafico
ensayo.bat MI_ESCENARIO_PEATON_30KMH --clima niebla
```

`ensayo.bat` abre CARLA si no está abierto, corre el ensayo mostrando el **video de CARLA en vivo** y la **ventana de Python del pedal**, y al terminar abre las gráficas y el video. Sin argumentos, lista los ensayos.

Lo mismo a mano, con CARLA ya abierto:
```bat
venv\Scripts\python.exe run_scenario.py ENCAP_CCRb_6_12 --clima lluvia --ver --abrir
```

### Ensayos

| Ensayo | Qué pasa | Mapa |
|---|---|---|
| `PROPIO_FRENA_18_3453` | Ego y líder a 18 km/h, 34.53 m; el líder frena a 3.87 m/s² hasta detenerse (prueba física del equipo) | Town04 |
| `ENCAP_CCRb_6_12` | Basado en Euro NCAP CCRb: ambos a 50 km/h, 12 m; el líder frena a 6 m/s² | Town04 |
| `MI_ESCENARIO_PEATON_30KMH` | Ego a 30 km/h; un niño sale entre autos estacionados y se queda en el carril | Town05 |
| `MI_ESCENARIO_40KMH` | Ambos a 40 km/h; el líder frena a 6 m/s² hasta detenerse | Town04 |
| `MI_ESCENARIO_40KMH_PEATON` | Ciudad a 40 km/h; el líder frena y un peatón cruza; después el ego retoma la marcha | Town05 |

PROPIO y ENCAP traen 8 autos de tráfico en los carriles de los lados (se crean los que caben en el mapa); `--sin-trafico` los quita.

### Opciones principales de `run_scenario.py`

| Opción | Qué hace |
|---|---|
| `--clima despejado\|lluvia\|niebla\|tormenta_noche` | Clima de CARLA y agarre de las llantas del ego (ver abajo) |
| `--sin-trafico` | Quita los autos de los carriles de los lados |
| `--sin-prediccion` | Apaga la predicción de cruce de peatones |
| `--carpeta NOMBRE` | Guarda la corrida en `results\porsche\NOMBRE\` (por defecto `ensayos`) |
| `--ver` / `--abrir` | Video en vivo / abrir gráficas y video al terminar |
| `--sin-ventana-pedal` | No abre la ventana del pedal |

`venv\Scripts\python.exe run_scenario.py --help` muestra todas.

## Climas

En CARLA el clima solo cambia la imagen; para que se note en el frenado, cada clima también **baja el agarre de las llantas del ego** (`config.py`, `CLIMAS`). El AEB no sabe qué clima hay: siempre supone 6.8 m/s² de frenado máximo.

| Clima | Imagen | Agarre | Frenado máximo medido a 50 km/h |
|---|---|---|---|
| `despejado` | Mediodía despejado | 100 % | 8.7 m/s² |
| `niebla` | Niebla densa, piso húmedo | 85 % | 7.9 m/s² |
| `lluvia` | Lluvia, piso mojado | 70 % | 6.7 m/s² |
| `tormenta_noche` | Tormenta de noche, piso encharcado | 50 % | 4.8 m/s² |

## Resultados

Cada corrida queda en `results\porsche\<carpeta>\<corrida>\`:

| Archivo | Contenido |
|---|---|
| `video.mp4` | La simulación en CARLA, con el estado del AEB y la vista del radar |
| `video_pedal.mp4` | La ventana del pedal durante la corrida (lo genera `video_pedal.py`) |
| `graficas.png` | Velocidad del ego, BTN y freno pedido |
| `log.csv` | Una fila por ciclo de 0.05 s: radar, AEB, pedal, freno enviado a CARLA |
| `meta.json` | Configuración del ensayo, clima, pedal y resultado |
| `pedal\carla_<fecha>\` | Gráficas y datos del pedal (varilla, giro, fuerza, corriente) |

Para generar los `video_pedal.mp4` que falten (la ventana se pone encima mientras graba; no la tapes):
```bat
venv\Scripts\python.exe video_pedal.py
```

### Reproducir las 12 simulaciones de clima

Con el mismo AEB y el pedal C5-E, todas en `results\porsche\con_trafico\`:

```bat
for %c in (despejado lluvia niebla tormenta_noche) do venv\Scripts\python.exe run_scenario.py PROPIO_FRENA_18_3453 --clima %c --carpeta con_trafico --ver --abrir
for %c in (despejado lluvia niebla tormenta_noche) do venv\Scripts\python.exe run_scenario.py ENCAP_CCRb_6_12 --clima %c --carpeta con_trafico --ver --abrir
rem cierra y vuelve a abrir CARLA antes de cambiar a Town05
for %c in (despejado lluvia niebla tormenta_noche) do venv\Scripts\python.exe run_scenario.py MI_ESCENARIO_PEATON_30KMH --clima %c --carpeta con_trafico --ver --abrir
venv\Scripts\python.exe video_pedal.py
```

Resultado obtenido (distancia final al obstáculo; ninguna con contacto):

| Ensayo | Tráfico | Despejado | Niebla | Lluvia | Tormenta de noche |
|---|---|---|---|---|---|
| PROPIO_FRENA_18_3453 (18 km/h) | 6 autos | 1.83 m | 1.77 m | 1.31 m | 1.66 m |
| ENCAP_CCRb_6_12 (50 km/h) | 6 autos | 1.49 m | 1.28 m | 1.34 m | 0.55 m |
| MI_ESCENARIO_PEATON_30KMH (30 km/h) | sin tráfico | 1.58 m | 1.73 m | 1.77 m | 1.64 m |

La física de CARLA varía un poco entre corridas: las distancias pueden cambiar unos centímetros.

## Problemas comunes

- **CARLA se cierra al abrir o al cargar un mapa:** casi siempre es memoria. Cierra programas pesados. CARLA también suele cerrarse al **cambiar de mapa** (Town04 ↔ Town05): ciérralo y vuelve a abrirlo antes de correr un ensayo de otro mapa.
- **`No encuentro los controladores de CARLA`:** falta `CARLA_ROOT` o no apunta a la carpeta de CARLA 0.9.16.
- **`graficas.png` no se guarda:** ciérrala en el visor de fotos y vuelve a correr.
- **No aparece la ventana del pedal:** otra ventana del pedal ocupa el puerto UDP 5055; ciérrala.

## Estructura

| Carpeta / archivo | Qué es |
|---|---|
| `run_scenario.py` | Corre un ensayo en CARLA y guarda los resultados |
| `ensayos\` | Definición de cada ensayo (velocidades, distancias, tráfico, peatones) |
| `aeb_core\` | AEB sin CARLA: percepción del radar, predicción de cruce, lógica BTN |
| `actuator.py` | `VirtualBrake`: traduce el freno logrado por el pedal al freno de CARLA (0–1) |
| `Pedal_C5E_Posicion\` | Modelo del pedal Nanotec + C5-E y su ventana (`visualizador_pedal.py`); ver su README |
| `config.py` | Parámetros: radar, AEB, Porsche, climas, tráfico |
| `sim_utils.py`, `traffic.py`, `escenario.py`, `overlay.py` | Utilidades de CARLA, actores, escenarios y el texto del video |
| `analyze.py`, `video_pedal.py` | Gráficas y video del pedal a partir de los resultados |
| `diagramas\` | Diagrama de la cadena de frenado, gráficas y la sección del artículo IEEE |
| `test_core.py` | Pruebas del AEB (`venv\Scripts\python.exe -m unittest test_core`) |

La calibración del pedal (`Pedal_C5E_Posicion\calibracion_bloomdrive.json`) y el registro `datos\LEON_Run10249_Evento04.csv` provienen del equipo Bloom Drive.

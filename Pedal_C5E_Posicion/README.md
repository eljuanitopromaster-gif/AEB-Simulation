# PEDAL NANOTEC + C5-E — PROFILE POSITION / Bloom Drive

Este proyecto **sustituye el modelo Profile Torque por un modelo digital Profile Position (CiA402 6060h=1)**. No controla hardware ni dispone de comunicación con el C5-E real. La ventana con dibujo de pedal y gráficas de la versión anterior se conserva.

## Ejecutar en VS Code (Windows / Python 3.10+)

Abre esta carpeta `Pedal_Nanotec_Profile_Position` en VS Code. Instala dependencias opcionales para PNG y PDF:

```powershell
py -m pip install -r requirements.txt
```

En **terminal 1**:

```powershell
py visualizador_pedal.py
```

En **terminal 2**, elige **uno**:

```powershell
py demo_bloomdrive.py        # reproduce las 1154 muestras del evento LEÓN 04 a velocidad real
py demo_telemetria.py        # pruebas manuales automáticas 0 / 25 / 50 / 75 / 100 %
```

También hay modo manual SIN segunda terminal:

```powershell
py visualizador_pedal.py --manual
```

Para modo claro: `py visualizador_pedal.py --claro`. Para cambiar la escala provisional:

```powershell
py demo_bloomdrive.py --mm-6900 20
```

**Importante**: `--mm-6900 20` significa *SUPOSICIÓN* de que 6900 ticks Bloom equivalen a 20 mm. NO lo indica Bloom Drive ni el fabricante. El catálogo señala carrera total de **al menos 38.1 mm**, pero no cuánto puede avanzar la varilla desde la posición instalada. Los 70 mm visibles son longitud inicial, no carrera. No utilizar este banco para accionar un freno real.

## Qué cambió respecto al modelo anterior

- **6060h = 1** Profile Position, no torque (`6060h=4` corresponde al simulador anterior). Se representan *virtualmente* las variables 607Ah (target), 6062h (perfil), 6064h (posición) y el perfil relacionado con 6081h/6083h/6084h. Las unidades reales del C5-E dependen de su configuración y NO coinciden necesariamente con los ticks Bloom Drive.
- Entrada de LEÓN: se utiliza **`brake_cmd_final`** (orden final), NO `position_mode_cmd` si la orden final fue inhibida.
- CSV original incluido `datos/LEON_Run10249_Evento04.csv`; `calibracion_bloomdrive.json` contiene las tablas auténticas de `GENERALWORKSPACE_final.mat`.
- La telemetría Bloom `veh_brake` es **valor bruto, NO posición real**. `Brake_encoder_pos` de la grabación permanece en cero.
- El modelo simula posiciones con perfil trapezoidal limitado a velocidad 30 mm/s de catálogo y aceleración/deceleración/retardo SUPUESTOS. No reproduce una respuesta experimental medida.
- Visualizador muestra la solicitud de posición (normalizada), la posición estimada, ángulo y fuerza aproximados. `brake_fraction` es **estimación de giro de pedal**, NO medición de presión ni calibración del frenado del coche.

## Dos entradas diferentes

- `modelo.step(75, 0.02)` significa **75 % de rango normalizado de referencia de posición (0..6900 unidades)**, útil para probar 0/25/50/75/100 SIN saturación aparente.
- `modelo.step_demanda_bloom_pct(30,0.02)` toma **30 % de demanda de freno Bloom**, pasa por `bp_brake → table_ticks` y luego limita a 6900 unidades. Por la tabla, demandas superiores a aproximadamente 30.34 % pueden mandar LA MISMA referencia.
- `modelo.step_desaceleracion(3.0,0.02)` convierte desaceleración positiva en magnitud m/s² por `dec_breakpoints → brake_map → table_ticks`.
- `modelo.step_target_ticks(4231.914, 0.01)` reproduce directamente la orden bruta final de LEÓN.

**No confundas** porcentaje de posición con porcentaje real de presión de freno. Es necesario medir ambas variables para mapear CARLA con rigor.

## Archivos

- `pedal_model.py`: modelo virtual de perfil de posición, calibración y mecanismo
- `demo_bloomdrive.py`: reproduccion del evento LEÓN Run 10249, 195.30-206.83 s
- `demo_telemetria.py`: secuencia automática
- `visualizador_pedal.py`: interfaz con pedal grande y gráficas en vivo
- `reportes.py`: exportación CSV, PNG y PDF con títulos en ejes
- `calibracion_bloomdrive.json`: mapas de calibración de origen
- `pedal_carla.py`: interfaz SOLO simulada, no conectada a hardware
- `tests/test_position.py`: pruebas automatizadas

## Validación y pendientes

- Confirmar qué significa `pc_autopilot=1`.
- Verificar qué representa exactamente el límite 6900 en el controlador de Bloom Drive y las unidades verdaderas de los ticks.
- Conseguir encoder real funcional/posición vs tiempo, límites de velocidad, aceleración y retracción del conjunto.
- Medir carrera efectiva desde reposo y presión/freno real vs posición; no inferir 100% freno de 100% recorrido.
- Validar ajuste del lazo real y fuerzas. Este modelo es **solo de análisis y simulación digital**.

El modelo anterior de torque queda intacto en un ZIP independiente incluido como respaldo.

Para ver una comparativa inmediata sin ejecutar ninguna ventana, corre:

```powershell
py comparacion_leon.py
```

La imagen `vista_previa_LEON.png` es ejemplo de la RESPUESTA DEL MODELO, **no es una prueba de que el motor real tenga esa misma respuesta**. La imagen `vista_previa_simulador.png` muestra el tema oscuro original.

Pruebas del código:

```powershell
py -m unittest discover -s tests -v
```

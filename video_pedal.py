"""Video de la ventana de Python del pedal para corridas ya hechas.

Abre la misma ventana del visualizador del pedal que usó la corrida (Pedal_C5E_Posicion o, en corridas
viejas, Pedal_Nanotec_Torque), la avanza ciclo a ciclo con pedal/carla_<fecha>/datos_completos.csv y guarda
cada cuadro en <corrida>/video_pedal.mp4, a la misma velocidad que la simulación (20 cuadros por segundo).

Uso:
  venv\\Scripts\\python.exe video_pedal.py                       # todas las corridas con datos del pedal
  venv\\Scripts\\python.exe video_pedal.py CARPETA [CARPETA ...]  # solo esas corridas
  --rehacer   vuelve a generar los videos que ya existen
La ventana se pone encima de las demás mientras graba: no la tapes.
"""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parent
ENSAYOS = RAIZ / "results" / "porsche" / "ensayos"
FPS = 20
MODELOS = ("Pedal_C5E_Posicion", "Pedal_Nanotec_Torque")


def modelo_de(corrida):
    """Carpeta del modelo del pedal con que se hizo la corrida (según su meta.json)."""
    try:
        meta = json.loads((corrida / "meta.json").read_text(encoding="utf-8"))
        origen = (meta.get("actuator_params") or {}).get("origen")
    except (OSError, ValueError):
        origen = None
    return origen if origen in MODELOS else MODELOS[0]


def grabar(corrida, modelo):
    """Se ejecuta en un proceso aparte por corrida: los dos modelos tienen módulos con los mismos nombres."""
    import ctypes
    import types

    import cv2
    import numpy as np
    from PIL import ImageGrab

    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)    # coordenadas reales para capturar la ventana
    except (AttributeError, OSError):
        pass
    sys.path.insert(0, str(RAIZ / modelo))
    import visualizador_base

    class SocketMudo:
        """Sustituye al socket UDP: la grabación no escucha CARLA ni choca con una ventana abierta."""

        def bind(self, *_):
            pass

        def setblocking(self, *_):
            pass

        def recvfrom(self, *_):
            raise BlockingIOError

        def close(self):
            pass

    visualizador_base.socket = types.SimpleNamespace(socket=lambda *a, **k: SocketMudo(), AF_INET=0, SOCK_DGRAM=0)
    from visualizador_compacto import cargar_grabacion
    from visualizador_pedal import MonitorEstiloOriginal

    class Grabador(MonitorEstiloOriginal):
        def update(self):
            pass    # el ciclo lo avanza este bucle, no el temporizador de tkinter

        def close(self):
            self.root.destroy()    # sin el guardado automático de la ventana normal

    filas, archivo = cargar_grabacion(corrida / "pedal")
    m = Grabador(udp=True)
    pendientes = [m.root]
    while pendientes:    # «EN VIVO» no aplica a una corrida ya grabada
        w = pendientes.pop()
        pendientes.extend(w.winfo_children())
        if w.winfo_class() == "Label":
            texto = str(w.cget("text"))
            if texto.startswith("●  EN VIVO"):
                w.config(text="●  GRABACIÓN DE LA CORRIDA EN CARLA")
            elif "EN VIVO" in texto:
                w.config(text=texto.replace(" EN VIVO", ""))
    m.root.geometry("1130x1000+0+0")
    m.root.attributes("-topmost", True)
    m.root.update()
    t0 = float(filas[0]["session_time_s"])
    salida = corrida / "video_pedal.mp4"
    video = cuadro = None
    try:
        for fila in filas:
            m.history.append(fila)
            m.latest = fila
            m._mostrar_metricas()
            m.status.set(f"GRABACIÓN | {corrida.name} | t = {float(fila['session_time_s']) - t0:5.2f} s")
            m.draw_mechanism()
            m.draw_chart()
            m.root.update_idletasks()
            m.root.update()
            x, y = m.root.winfo_rootx(), m.root.winfo_rooty()
            w, h = m.root.winfo_width() // 2 * 2, m.root.winfo_height() // 2 * 2
            cuadro = cv2.cvtColor(np.array(ImageGrab.grab(bbox=(x, y, x + w, y + h))), cv2.COLOR_RGB2BGR)
            if video is None:
                video = cv2.VideoWriter(str(salida), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (w, h))
            video.write(cuadro)
        for _ in range(FPS):    # 1 s quieto al final
            video.write(cuadro)
    finally:
        if video is not None:
            video.release()
        m.close()
    print(f"listo: {corrida.name} ({len(filas)} cuadros, {modelo}, de {archivo.parent.name})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("corridas", nargs="*", help="carpetas de results/porsche/ensayos (por defecto, todas)")
    ap.add_argument("--rehacer", action="store_true", help="vuelve a generar videos que ya existen")
    ap.add_argument("--modelo", choices=MODELOS, help=argparse.SUPPRESS)    # uso interno: proceso por corrida
    args = ap.parse_args()
    corridas = [Path(c) if os.path.isabs(c) else ENSAYOS / c for c in args.corridas] or sorted(
        p for p in ENSAYOS.iterdir() if (p / "pedal").is_dir())
    if args.modelo:
        grabar(corridas[0], args.modelo)
        return
    for corrida in corridas:
        if (corrida / "video_pedal.mp4").exists() and not args.rehacer:
            print(f"ya existe: {corrida.name}")
            continue
        r = subprocess.run([sys.executable, __file__, str(corrida), "--modelo", modelo_de(corrida)])
        if r.returncode:
            print(f"ERROR en {corrida.name} (código {r.returncode})")


if __name__ == "__main__":
    main()

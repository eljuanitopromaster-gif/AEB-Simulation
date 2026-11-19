"""Diagrama: cómo frena el Porsche en CARLA con el modelo del pedal (cadena por ciclo de 0.05 s)."""
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
os.makedirs(OUT, exist_ok=True)

INK, MUTED, LINE = "#1F2937", "#6B7280", "#9CA3AF"
C_CARLA, C_AEB, C_PEDAL, C_CONV, C_OUT = "#DBEAFE", "#FEF3C7", "#D1FAE5", "#EDE9FE", "#F3F4F6"
E_CARLA, E_AEB, E_PEDAL, E_CONV, E_OUT = "#2563EB", "#D97706", "#059669", "#7C3AED", "#6B7280"

fig, ax = plt.subplots(figsize=(17, 10.2))
ax.set_xlim(-6, 170)
ax.set_ylim(0, 102)
ax.axis("off")
plt.rcParams["font.family"] = "DejaVu Sans"


def caja(x, y, w, h, titulo, texto, fc, ec, n=None, lw=1.8, ls="-", tsize=12.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.6",
                                fc=fc, ec=ec, lw=lw, ls=ls))
    if n is not None:
        ax.add_patch(plt.Circle((x + 2.4, y + h - 2.4), 2.0, fc=ec, ec="none"))
        ax.text(x + 2.4, y + h - 2.45, str(n), color="white", ha="center", va="center", fontsize=10.5, weight="bold")
    ax.text(x + w / 2 + (1.6 if n is not None else 0), y + h - 2.6, titulo, ha="center", va="center",
            fontsize=tsize, weight="bold", color=INK)
    ax.text(x + w / 2, y + h / 2 - 2.2, texto, ha="center", va="center", fontsize=9.6, color=INK, linespacing=1.35)


def flecha(p0, p1, texto=None, color=INK, ls="-", rad=0.0, tx=0, ty=1.6, lw=2.0, tsize=9.6):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=17, color=color, lw=lw, ls=ls,
                                 connectionstyle=f"arc3,rad={rad}", shrinkA=2, shrinkB=2))
    if texto:
        mx, my = (p0[0] + p1[0]) / 2 + tx, (p0[1] + p1[1]) / 2 + ty
        ax.text(mx, my, texto, ha="center", va="bottom", fontsize=tsize, color=color, weight="bold",
                bbox=dict(fc="white", ec="none", pad=0.6))


# Título
ax.text(85, 99, "Cómo frena el Porsche en CARLA con el modelo del pedal Nanotec + C5-E",
        ha="center", va="center", fontsize=18, weight="bold", color=INK)
ax.text(85, 95.2, "Se repite en cada ciclo de 0.05 s  ·  run_scenario.py  ·  freno por defecto: --actuador pedal",
        ha="center", va="center", fontsize=11, color=MUTED)

# Fila 1: decisión
caja(4, 70, 30, 19, "Radar (CARLA)", "Detecciones: distancia,\nángulo y velocidad Doppler\nde lo que hay adelante",
     C_CARLA, E_CARLA, 1)
caja(44, 70, 34, 19, "Percepción", "Objeto en el carril del ego\n+ predicción de cruce\n(aeb_core/perception.py, crossing.py)",
     C_AEB, E_AEB, 2)
caja(88, 70, 34, 19, "Lógica AEB (BTN)", "Desaceleración necesaria ÷ 6.8 m/s²\n= BTN; frena si BTN ≥ 0.5\n(aeb_core/logic.py)",
     C_AEB, E_AEB, 3)
caja(132, 70, 34, 19, "% de freno PEDIDO", "demand_cmd: 0–100 %\n«cuánto freno hace falta»",
     "white", E_AEB, None, lw=2.2)
flecha((34.8, 79.5), (43.2, 79.5))
flecha((78.8, 79.5), (87.2, 79.5))
flecha((122.8, 79.5), (131.2, 79.5))

# Fila 2: el pedal (lo nuevo)
ax.add_patch(FancyBboxPatch((52, 33), 114, 31, boxstyle="round,pad=0.4,rounding_size=2.2",
                            fc="#ECFDF5", ec=E_PEDAL, lw=2.6))
ax.text(54, 61.4, "4   MODELO DEL PEDAL  (lo que se conectó)", fontsize=13.5, weight="bold", color=E_PEDAL, va="center")
ax.text(54, 57.4, "Pedal_C5E_Posicion/pedal_carla.py  ·  PedalCARLA.step(% pedido, 0.05 s)",
        fontsize=9.6, color=E_PEDAL, va="center", ha="left")
sub = [(56, "Controlador C5-E", "Profile Position (6060h=1)\n% pedido → posición\nobjetivo de la varilla"),
       (84, "Motor Nanotec", "Mueve la varilla con\nvelocidad máx. 30 mm/s\n(perfil de aceleración)"),
       (112, "Varilla → pedal", "Empuja el pedal a 11 cm\ndel pivote; carrera\nhasta 20 mm"),
       (140, "Giro del pedal", "Ángulo contra la\nresistencia del freno\n→ brake_fraction (0–1)")]
for x, t, s in sub:
    caja(x, 36, 23.5, 17.5, t, s, "white", E_PEDAL, None, lw=1.4, tsize=11)
for a, b in ((79.8, 83.2), (107.8, 111.2), (135.8, 139.2)):
    flecha((a, 44.5), (b, 44.5), color=E_PEDAL)
flecha((149, 69.2), (149, 54.3), "entra", color=E_AEB, tx=4.5, ty=3)

# Antes (reemplazado)
caja(4, 36, 40, 17.5, "Antes: actuador genérico", "BrakeActuator (actuator.py)\nretraso fijo 0.1 s + rampa 0.3 s\nno sabía nada del motor ni del pedal",
     "#F9FAFB", LINE, None, lw=1.6, ls="--", tsize=11.5)
flecha((44.8, 44.5), (51.2, 44.5), "se reemplazó por", color=LINE, ls="--", tx=0, ty=1.8, tsize=8.8)

# Fila 3: de vuelta a CARLA
caja(118, 7, 48, 19, "% de freno LOGRADO", "brake_fraction × 100\ncon el retraso y el límite\nreales del mecanismo", "white", E_PEDAL, None,
     lw=2.2)
flecha((152, 35.2), (152, 26.8), "sale", color=E_PEDAL, tx=4, ty=-1)
caja(62, 7, 46, 19, "VirtualBrake", "% logrado × 6.8 m/s² = desaceleración objetivo\ntabla carla_brake_map → freno 0–1\n+ corrección PI con la desaceleración medida",
     C_CONV, E_CONV, 5)
flecha((117.2, 16.5), (108.8, 16.5))
caja(4, 7, 48, 19, "CARLA frena el auto", "ctrl.brake (0–1) → ego.apply_control()\nla física de CARLA frena las ruedas\ndel Porsche", C_CARLA, E_CARLA, 6)
flecha((61.2, 16.5), (52.8, 16.5), "ctrl.brake")

# Realimentación
ax.plot([3.4, -2.5, -2.5], [16.5, 16.5, 79.5], color=E_CARLA, lw=2.0, ls=(0, (4, 3)))
flecha((-2.5, 79.5), (3.4, 79.5), color=E_CARLA)
ax.text(-4.3, 48, "el auto se mueve → el radar mide la nueva distancia", rotation=90, ha="center", va="center",
        fontsize=9.4, color=E_CARLA, weight="bold")

# Salidas laterales
caja(4, 55.5, 40, 11.5, "", "", C_OUT, E_OUT, None, lw=1.2)
ax.text(24, 65.0, "Lo que ves y lo que queda guardado", ha="center", fontsize=10.5, weight="bold", color=INK)
ax.text(24, 59.6, "UDP 5055 → ventana de Python del pedal\nlog.csv · graficas.png · video.mp4\npedal\\…\\graficas_completas.png",
        ha="center", va="center", fontsize=8.8, color=INK, linespacing=1.3)
flecha((56, 52.2), (44.8, 59.5), color=E_OUT, ls=(0, (3, 3)), rad=0.15, lw=1.6)

# Nota
ax.text(85, 1.6, "CARLA sigue haciendo la física del frenado; el pedal decide cuánto freno llega y qué tan rápido.  "
                 "Modelo anterior (Profile Torque): Pedal_Nanotec_Torque — mismo mecanismo, máximo 92.6 %.",
        ha="center", va="center", fontsize=9.6, color=MUTED, style="italic")

for ext in ("png", "pdf"):
    fig.savefig(os.path.join(OUT, f"diagrama_freno_pedal.{ext}"), dpi=170, bbox_inches="tight", facecolor="white")
print(os.path.join(OUT, "diagrama_freno_pedal.png"))

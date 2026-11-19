r"""Figuras de la sección IEEE «Integración del modelo del pedal con CARLA».
Uso: venv\Scripts\python.exe diagramas\seccion_ieee\generar_figuras_ieee.py"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

AQUI = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(AQUI, "figuras_pedal")
RAIZ = os.path.abspath(os.path.join(AQUI, "..", ".."))
ENS = os.path.join(RAIZ, "results", "porsche", "ensayos")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"], "font.size": 7.5,
                     "axes.grid": True, "grid.alpha": 0.3, "lines.linewidth": 1.2, "legend.fontsize": 6.5,
                     "axes.labelsize": 7.5, "xtick.labelsize": 7, "ytick.labelsize": 7, "mathtext.fontset": "stix"})
NAR, VER, MOR, AZU, ROJ = "#D97706", "#059669", "#7C3AED", "#2563EB", "#DC2626"

# 1) Diagrama de la cadena de frenado (columna IEEE)
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle
fig, ax = plt.subplots(figsize=(3.5, 3.2))
ax.set_xlim(0, 100); ax.set_ylim(0, 91); ax.axis("off")
W, H = 28, 17
COL = (6, 38, 70)            # x izquierda de cada columna
FIL = (64, 36, 6)            # y inferior de cada fila


def caja(c, f, titulo, l1, l2="", fc="white", ec="#374151"):
    x, y = COL[c], FIL[f]
    ax.add_patch(FancyBboxPatch((x, y), W, H, boxstyle="round,pad=0.3,rounding_size=1.5", fc=fc, ec=ec, lw=0.9))
    ax.text(x + W / 2, y + H - 4, titulo, ha="center", va="center", fontsize=7, weight="bold")
    ax.text(x + W / 2, y + H / 2 - 2.2, l1, ha="center", va="center", fontsize=6.4)
    if l2:
        ax.text(x + W / 2, y + 2.9, l2, ha="center", va="center", fontsize=6.4)


def flecha(p0, p1, txt=None, dx=0, dy=1.2, ha="center"):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=8, lw=0.9, color="#111827", shrinkA=0, shrinkB=0))
    if txt:
        ax.text((p0[0] + p1[0]) / 2 + dx, (p0[1] + p1[1]) / 2 + dy, txt, ha=ha, va="bottom", fontsize=6.6)


# grupo del actuador
ax.add_patch(Rectangle((3.5, 33), 97 - 3.5, 25.5, fc="#ECFDF5", ec=VER, lw=1.0, ls=(0, (4, 2))))
ax.text(5, 56.2, "Modelo del actuador (Pedal_C5E_Posicion, Python)", fontsize=6.4, color=VER, style="italic", va="center")
caja(0, 0, "Radar (CARLA)", r"$d,\ \dot d$, ángulo", "20 Hz")
caja(1, 0, "Percepción", "objetivo en el carril", "+ predicción de cruce")
caja(2, 0, "Lógica AEB", r"BTN $=a_{req}/a_{\max}$", r"$u_b=100\,$BTN")
caja(2, 1, "C5-E (posición)", r"$z_{ref}=0.2\,u_b$ mm", "perfil trapezoidal")
caja(1, 1, "Motor + husillo", r"$|\dot z|\leq 30$ mm/s", r"$0\leq z\leq 20$ mm")
caja(0, 1, "Varilla y pedal", r"$z\rightarrow\theta_p$", r"$\theta_p\rightarrow p_{log}$")
caja(0, 2, "VirtualBrake", r"$b_{obj}=p_{log}\,a_{\max}/100$", r"$b_C=f_C^{-1}(b_{obj})+$PI", fc="#F5F3FF")
caja(2, 2, "Vehículo en CARLA", "apply_control", r"freno $b_C\in[0,1]$", fc="#EFF6FF")
y1, y2, y3 = FIL[0] + H / 2, FIL[1] + H / 2, FIL[2] + H / 2
flecha((COL[0] + W, y1), (COL[1], y1)); flecha((COL[1] + W, y1), (COL[2], y1))
flecha((COL[2] + W / 2, FIL[0]), (COL[2] + W / 2, FIL[1] + H), r"$u_b$", dx=1.5, dy=-2.3, ha="left")
flecha((COL[2], y2), (COL[1] + W, y2)); flecha((COL[1], y2), (COL[0] + W, y2))
flecha((COL[0] + W / 2, FIL[1]), (COL[0] + W / 2, FIL[2] + H), r"$p_{log}$", dx=1.5, dy=-2.3, ha="left")
flecha((COL[0] + W, y3), (COL[2], y3), r"$b_C$", dy=0.8)
# realimentación: vehículo -> radar por el borde derecho y superior
ax.plot([COL[2] + W, 99, 99, COL[0] + W / 2], [y3, y3, 85.5, 85.5], color="#2563EB", lw=0.9, ls=(0, (3, 2)))
flecha((COL[0] + W / 2, 85.5), (COL[0] + W / 2, FIL[0] + H))
ax.text(50, 86.3, "movimiento del ego: nueva medición del radar en el ciclo siguiente (0.05 s)", ha="center", va="bottom",
        fontsize=6.2, color="#2563EB")
fig.subplots_adjust(0, 0, 1, 1)
fig.savefig(os.path.join(OUT, "fig_cadena.pdf")); fig.savefig(os.path.join(OUT, "fig_cadena.png"), dpi=300)
plt.close(fig)

# 2) ENCAP sin tráfico: pedido, logrado y CARLA; varilla; respuesta del vehículo
d = pd.read_csv(os.path.join(ENS, "ENCAP_CCRb_6_12_50kmh_btn_actuador-pedal_sin-trafico", "log.csv"))
d = d[(d.t >= 0.9) & (d.t <= 4.8)]
fig, ax = plt.subplots(3, 1, figsize=(3.5, 4.4), sharex=True)
ax[0].step(d.t, d.demand_pct, where="post", color=NAR, label="Pedido $u_b$")
ax[0].plot(d.t, d.actuator_pct, color=VER, lw=1.6, label="Logrado $p_{log}$")
ax[0].plot(d.t, d.brake_cmd * 100, color=MOR, ls="--", label=r"CARLA $100\,b_C$")
ax[0].set_ylabel("Freno [%]"); ax[0].set_ylim(-3, 105)
ax[0].legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=3, frameon=False, handlelength=1.6, columnspacing=0.8)
ax[1].plot(d.t, d.pedal_target_mm, color=NAR, label="Referencia $z_{ref}$")
ax[1].plot(d.t, d.pedal_rod_mm, color=VER, lw=1.6, label="Posición $z$")
ax[1].set_ylabel("Varilla [mm]"); ax[1].set_ylim(-1, 21)
ax[1].legend(loc="lower right", framealpha=0.9)
ax[2].plot(d.t, d.v_ego * 3.6, color=AZU, label="$v_e$ [km/h]")
ax[2].set_ylabel("$v_e$ [km/h]")
c = ax[2].twinx(); c.plot(d.t, -d.a_ego, color=ROJ, lw=0.9, label="Desaceleración [m/s$^2$]")
c.axhline(6.8, color=ROJ, ls=":", lw=0.8); c.set_ylim(-1, 11); c.set_ylabel("[m/s$^2$]"); c.grid(False)
h1, l1 = ax[2].get_legend_handles_labels(); h2, l2 = c.get_legend_handles_labels()
ax[2].legend(h1 + h2, l1 + l2, loc="lower center", bbox_to_anchor=(0.36, 0.02), framealpha=0.9)
ax[2].set_xlabel("Tiempo [s]")
for a, s in zip(ax, "abc"):
    a.text(0.01, 0.93, f"({s})", transform=a.transAxes, fontsize=7.5, va="top")
fig.tight_layout(pad=0.3, h_pad=0.4)
fig.savefig(os.path.join(OUT, "fig_encap_cadena.pdf")); fig.savefig(os.path.join(OUT, "fig_encap_cadena.png"), dpi=300)

# 3) Niño que cruza tarde: con y sin predicción
fig, ax = plt.subplots(2, 1, figsize=(3.5, 2.9), sharex=True)
for modo, color, nombre in (("con", VER, "Con predicción"), ("sin", ROJ, "Sin predicción")):
    r = pd.read_csv(os.path.join(ENS, f"MI_ESCENARIO_PEATON_30KMH_variante-tarde-3.2s_{modo}-prediccion", "log.csv"))
    r = r[(r.t >= 8.8) & (r.t <= 12.5)]
    ax[0].plot(r.t, r.v_ego * 3.6, color=color, lw=1.5, label=nombre)
    ax[1].step(r.t, r.demand_pct, where="post", color=color, lw=0.8, ls="--")
    ax[1].plot(r.t, r.actuator_pct, color=color, lw=1.5)
    if r.collision.max():
        k = r[r.collision > 0].iloc[0]
        ax[0].plot(k.t, k.v_ego * 3.6, "X", color=color, ms=7)
        ax[0].annotate("atropello", xy=(k.t, k.v_ego * 3.6), xytext=(k.t + 0.25, k.v_ego * 3.6 + 3), fontsize=6.5, color=color)
ax[0].set_ylabel("$v_e$ [km/h]"); ax[0].legend(loc="lower left", framealpha=0.9)
ax[1].set_ylabel("Freno [%]"); ax[1].set_xlabel("Tiempo [s]")
ax[1].plot([], [], color="k", lw=0.8, ls="--", label="pedido $u_b$"); ax[1].plot([], [], color="k", lw=1.5, label="logrado $p_{log}$")
ax[1].legend(loc="upper left", framealpha=0.9)
for a, s in zip(ax, "ab"):
    a.text(0.99, 0.93, f"({s})", transform=a.transAxes, fontsize=7.5, va="top", ha="right")
fig.tight_layout(pad=0.3, h_pad=0.4)
fig.savefig(os.path.join(OUT, "fig_nino_tarde.pdf")); fig.savefig(os.path.join(OUT, "fig_nino_tarde.png"), dpi=300)
print("listo:", OUT)

# 4) Fotos de CARLA (fotograma del video sincronizado con log.csv), sin la franja inferior
import cv2
for corrida, t, nombre in (("ENCAP_CCRb_6_12_50kmh_btn_actuador-pedal_sin-trafico", 2.0, "foto_encap"),
                           ("MI_ESCENARIO_PEATON_30KMH_variante-tarde-3.2s_con-prediccion", 10.2, "foto_nino_con"),
                           ("MI_ESCENARIO_PEATON_30KMH_variante-tarde-3.2s_sin-prediccion", 11.0, "foto_nino_sin")):
    log = pd.read_csv(os.path.join(ENS, corrida, "log.csv"))
    k = int((log.t - t).abs().idxmin())          # un fotograma por ciclo de 0.05 s
    cap = cv2.VideoCapture(os.path.join(ENS, corrida, "video.mp4"))
    cap.set(cv2.CAP_PROP_POS_FRAMES, k)
    ok, img = cap.read()
    img = cv2.resize(img[:632], (960, 474), interpolation=cv2.INTER_AREA)
    cv2.imwrite(os.path.join(OUT, nombre + ".jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
print("fotos listas")

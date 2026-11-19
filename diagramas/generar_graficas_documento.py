"""Gráficas para el documento: pedido vs logrado vs CARLA (ENCAP) y niño que cruza tarde con/sin predicción."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

AQUI = os.path.dirname(os.path.abspath(__file__))
ENS = os.path.join(AQUI, "..", "results", "porsche", "ensayos")
plt.rcParams.update({"font.size": 11, "axes.grid": True, "grid.alpha": 0.3})

# 1) ENCAP sin tráfico: la cadena pedido -> pedal -> CARLA
d = pd.read_csv(os.path.join(ENS, "ENCAP_CCRb_6_12_50kmh_btn_actuador-pedal_sin-trafico", "log.csv"))
d = d[(d.t >= 0.8) & (d.t <= 5.0)]
fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
a1.step(d.t, d.demand_pct, where="post", color="#D97706", lw=2, label="% de freno PEDIDO por el AEB")
a1.plot(d.t, d.actuator_pct, color="#059669", lw=2.4, label="% de freno LOGRADO por el pedal")
a1.plot(d.t, d.brake_cmd * 100, color="#7C3AED", lw=1.8, ls="--", label="Freno enviado a CARLA (ctrl.brake × 100)")
a1.set_ylabel("%")
a1.set_ylim(-3, 105)
a1.legend(loc="lower right", fontsize=9.5)
a1.set_title("ENCAP_CCRb_6_12 (sin tráfico): lo que pide el AEB, lo que logra el pedal y lo que recibe CARLA", fontsize=12, weight="bold")
t0 = d[d.demand_pct > 0].t.iloc[0]
a1.annotate("el pedal tarda 0.2 s\nen empezar a frenar", xy=(t0 + 0.2, 2), xytext=(t0 + 0.55, 30),
            arrowprops=dict(arrowstyle="->"), fontsize=9.5)
a2.plot(d.t, d.v_ego * 3.6, color="#2563EB", lw=2.2, label="Velocidad del ego [km/h]")
a2.set_ylabel("km/h")
b = a2.twinx()
b.plot(d.t, -d.a_ego, color="#DC2626", lw=1.4, alpha=0.8, label="Desaceleración medida [m/s²]")
b.axhline(6.8, color="#DC2626", ls=":", lw=1)
b.text(4.95, 6.95, "6.8 m/s² (máx. Porsche real)", ha="right", fontsize=9, color="#DC2626")
b.set_ylabel("m/s²")
b.set_ylim(-1, 11)
b.grid(False)
h1, l1 = a2.get_legend_handles_labels(); h2, l2 = b.get_legend_handles_labels()
a2.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=9.5)
a2.set_xlabel("Tiempo [s]")
fig.tight_layout()
fig.savefig(os.path.join(AQUI, "encap_pedido_logrado_carla.png"), dpi=150)

# 2) Niño que cruza tarde: con y sin predicción de cruce
fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
for modo, color in (("con", "#059669"), ("sin", "#DC2626")):
    r = pd.read_csv(os.path.join(ENS, f"MI_ESCENARIO_PEATON_30KMH_variante-tarde-3.2s_{modo}-prediccion", "log.csv"))
    r = r[r.t >= 8.5]
    a1.plot(r.t, r.v_ego * 3.6, color=color, lw=2.2, label=f"{modo} predicción de cruce")
    a2.plot(r.t, r.actuator_pct, color=color, lw=2.2, label=f"% logrado por el pedal, {modo} predicción")
    if r.collision.max():
        tc = r[r.collision > 0].t.iloc[0]
        vc = r[r.collision > 0].v_ego.iloc[0] * 3.6
        a1.plot(tc, vc, "X", color=color, ms=12)
        a1.annotate("atropello (21.7 km/h relativo)", xy=(tc, vc), xytext=(tc + 0.35, vc + 4), fontsize=9.5, color=color)
    else:
        ts = r[(r.t > 9.3) & (r.v_ego < 0.1)].t.iloc[0]
        a1.annotate("se detiene a 1.6 m del niño", xy=(ts, 0), xytext=(ts + 0.3, 8), fontsize=9.5, color=color,
                    arrowprops=dict(arrowstyle="->", color=color))
a1.set_ylabel("Velocidad del ego [km/h]")
a1.legend(loc="lower left")
a1.set_title("Niño que cruza tarde a 30 km/h: con predicción el AEB frena 0.85 s antes", fontsize=12, weight="bold")
a2.set_ylabel("Freno logrado [%]")
a2.set_xlabel("Tiempo [s]")
a2.legend(loc="upper left", fontsize=9.5)
fig.tight_layout()
fig.savefig(os.path.join(AQUI, "nino_tarde_con_sin_prediccion.png"), dpi=150)
print("ok")

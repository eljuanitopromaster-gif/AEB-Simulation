"""Compara los 4 climas en PROPIO_FRENA_18_3453, ENCAP_CCRb_6_12 y MI_ESCENARIO_PEATON_30KMH.
Uso: venv\\Scripts\\python.exe diagramas\\generar_comparacion_climas.py [posicion|torque]   (por defecto posicion)"""
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

AQUI = os.path.dirname(os.path.abspath(__file__))
ENS = os.path.join(AQUI, "..", "results", "porsche", "ensayos")
PEDAL = sys.argv[1] if len(sys.argv) > 1 else "posicion"
ACT = {"torque": "actuador-pedal-torque", "posicion": "actuador-pedal"}[PEDAL]
NOMBRE_PEDAL = {"torque": "Pedal_Nanotec_Torque", "posicion": "Pedal_C5E_Posicion"}[PEDAL]
ESCENARIOS = [
    ("PROPIO_FRENA_18_3453 (18 km/h, líder frena)", f"PROPIO_FRENA_18_3453_18kmh_btn_{ACT}_sin-trafico", "líder"),
    ("ENCAP_CCRb_6_12 (50 km/h, 12 m)", f"ENCAP_CCRb_6_12_50kmh_btn_{ACT}_sin-trafico", "líder"),
    ("MI_ESCENARIO_PEATON_30KMH (niño cruza)", f"MI_ESCENARIO_PEATON_30KMH_30kmh_btn_{ACT}", "niño"),
]
CLIMAS = [("", "Despejado (agarre 100 %)", "#2563EB"), ("_clima-niebla", "Niebla (85 %)", "#6B7280"),
          ("_clima-lluvia", "Lluvia (70 %)", "#059669"), ("_clima-tormenta_noche", "Tormenta de noche (50 %)", "#DC2626")]
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3})

fig, ax = plt.subplots(3, 3, figsize=(15, 10.5), sharex="col")
for j, (titulo, base, objetivo) in enumerate(ESCENARIOS):
    for sufijo, nombre, color in CLIMAS:
        r = os.path.join(ENS, base + sufijo)
        d = pd.read_csv(os.path.join(r, "log.csv"))
        res = json.load(open(os.path.join(r, "meta.json"), encoding="utf-8"))
        t0 = d[d.demand_pct > 0].t.iloc[0]
        d = d[(d.t >= t0 - 0.5) & (d.t <= t0 + 4.0)]
        x = d.t - t0
        ax[0, j].plot(x, d.v_ego * 3.6, color=color, lw=2, label=nombre)
        ax[1, j].plot(x, -d.a_ego, color=color, lw=1.4)
        g = d.gap_gt.where(d.gap_gt < 30)
        ax[2, j].plot(x, g, color=color, lw=2,
                      label=f"{nombre}: {d.gap_gt.min():.2f} m" + (" (choque)" if res["collision"] else ""))
    ax[0, j].set_title(titulo, fontsize=11, weight="bold")
    ax[1, j].axhline(6.8, color="k", ls=":", lw=1)
    ax[1, j].text(3.95, 7.0, "6.8 m/s² (lo que supone el AEB)", ha="right", fontsize=8)
    ax[1, j].set_ylim(-1, 10)
    ax[2, j].axhline(0, color="k", lw=0.8)
    ax[2, j].legend(fontsize=8, loc="upper right", title="distancia final", title_fontsize=8)
    ax[2, j].set_xlabel("Tiempo desde que el AEB pide freno [s]")
ax[0, 0].set_ylabel("Velocidad del ego [km/h]")
ax[1, 0].set_ylabel("Desaceleración [m/s²]")
ax[2, 0].set_ylabel("Distancia al obstáculo [m]")
ax[0, 0].legend(fontsize=8.5, loc="lower left")
fig.suptitle(f"Mismo AEB y mismo pedal ({NOMBRE_PEDAL}) con cuatro climas: el agarre baja, el AEB no lo sabe", fontsize=13, weight="bold")
fig.tight_layout()
salida = os.path.join(AQUI, "comparacion_climas.png" if PEDAL == "posicion" else f"comparacion_climas_{PEDAL}.png")
fig.savefig(salida, dpi=150)
print(salida)

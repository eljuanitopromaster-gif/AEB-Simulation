"""Compara las corridas de un ensayo: actuador del repo, hidráulico (ModeloFreno) y, si existe, sin AEB.

Uso:  python comparar_actuadores.py MI_ESCENARIO_40KMH
Busca en results/porsche/ensayos/ las carpetas <ensayo>..._btn, ..._btn_actuador-hidraulico y ..._sinaeb.
Guarda comparacion_actuadores.png en la carpeta de la corrida hidráulica.
"""
import csv
import json
import math
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = os.path.join("results", "porsche", "ensayos")
SUFIJOS = {"actuador del repo": "_btn", "hidraulico": "_btn_actuador-hidraulico", "sin AEB": "_sinaeb"}


def leer(carpeta):
    with open(os.path.join(BASE, carpeta, "log.csv"), encoding="utf-8") as f:
        filas = list(csv.DictReader(f))
    with open(os.path.join(BASE, carpeta, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)
    cols = ("t", "v_ego", "a_ego", "gap_gt", "demand_pct", "actuator_pct", "brake_target_decel", "collision")
    return {k: [float(r[k]) for r in filas] for k in cols}, meta


def primero(D, col, umbral):
    return next((t for t, x in zip(D["t"], D[col]) if x > umbral), math.nan)


def en_marcha(D, col):
    """Valores de col con el ego a más de 2 m/s (descarta los picos de CARLA al detenerse)."""
    return [x for x, v in zip(D[col], D["v_ego"]) if v > 2.0]


def dato_en(D, col, t):
    return next((x for tt, x in zip(D["t"], D[col]) if tt >= t), math.nan)


def main():
    nombre = sys.argv[1] if len(sys.argv) > 1 else "MI_ESCENARIO_40KMH"
    carpetas = [d for d in os.listdir(BASE) if d.startswith(nombre + "_")]
    sel = {}
    for etiqueta, sufijo in SUFIJOS.items():
        # nombre exacto: <ensayo>_<vel>kmh<sufijo> (evita confundir MI_ESCENARIO_40KMH con MI_ESCENARIO_40KMH_PEATON)
        patron = re.compile(re.escape(nombre) + r"_\d+kmh" + re.escape(sufijo) + "$")
        ok = [d for d in carpetas if patron.match(d)]
        if ok:
            sel[etiqueta] = max(ok, key=lambda d: os.path.getmtime(os.path.join(BASE, d)))
    if "actuador del repo" not in sel or "hidraulico" not in sel:
        sys.exit(f"Faltan corridas de '{nombre}'. Encontré: {carpetas}\n"
                 "Corre el ensayo sin y con --actuador hidraulico.")

    datos = {k: leer(d) for k, d in sel.items()}
    for k, d in sel.items():
        print(f"  {k:18} -> {d}")

    filas = [
        ("D_max usado [m/s2]", lambda D, M: M["d_max"]),
        ("choque (0/1)", lambda D, M: max(D["collision"])),
        ("vel. impacto [km/h]", lambda D, M: (M["impact_vrel_mps"] or 0.0) * 3.6),
        ("distancia minima [m]", lambda D, M: min((g for g in D["gap_gt"] if math.isfinite(g)), default=math.nan)),
        ("t AEB pide freno [s]", lambda D, M: primero(D, "demand_pct", 0.0)),
        ("dist. al pedir freno [m]", lambda D, M: dato_en(D, "gap_gt", primero(D, "demand_pct", 0.0))),
        ("decel. pedida max [m/s2]", lambda D, M: max(en_marcha(D, "brake_target_decel"), default=0.0)),
        ("decel. medida max [m/s2]", lambda D, M: max((-a for a in en_marcha(D, "a_ego")), default=0.0)),
    ]
    print()
    print(f"{'':24}" + "".join(f"{k:>20}" for k in datos))
    for nom, f in filas:
        print(f"{nom:24}" + "".join(f"{f(D, M):20.2f}" for D, M in datos.values()))
    print()
    for k, (D, M) in datos.items():
        print(f"  {k:18} >>> {M['ensayo_resultado']}")

    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for k, (D, M) in datos.items():
        if k != "sin AEB":
            ax[0].plot(D["t"], D["brake_target_decel"], label=f"{k} (D_max {M['d_max']})")
        ax[1].plot(D["t"], [-a for a in D["a_ego"]], label=k)
        ax[2].plot(D["t"], D["gap_gt"], label=k)
    ax[0].set_ylabel("decel. entregada por\nel actuador [m/s²]")
    ax[1].set_ylabel("decel. medida\nen CARLA [m/s²]")
    ax[2].set_ylabel("distancia al líder [m]")
    ax[2].set_xlabel("t [s]")
    ax[2].set_ylim(0, 60)
    for a in ax:
        a.grid(True)
        a.legend()
    plt.tight_layout()
    salida = os.path.join(BASE, sel["hidraulico"], "comparacion_actuadores.png")
    plt.savefig(salida, dpi=120)
    print(f"\nGráfica guardada: {salida}")


if __name__ == "__main__":
    main()

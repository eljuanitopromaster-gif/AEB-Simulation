"""Comparación visual: mando medido Bloom vs respuesta de posición *SIMULADA*.
No confundir con validación de posición experimental: no hay encoder variable.
"""
import csv
from pathlib import Path
from pedal_model import PedalPosicion

def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root=Path(__file__).parent
    with (root/'datos'/'LEON_Run10249_Evento04.csv').open(newline='',encoding='utf-8-sig') as f:
        data=list(csv.DictReader(f))
    p=PedalPosicion.con_mm_por_6900(20.0)
    ts=[];cmd=[];sim_ticks=[];xs=[];x_refs=[];auto=[];veh=[]
    prev=0.0
    for row in data:
        t=float(row['time_relative_s'])
        h=t-prev if ts else .01;prev=t
        h=max(h,.0001)
        c=float(row['brake_cmd_final'])
        r=p.step_target_ticks(c,h)
        ts.append(t);cmd.append(c);sim_ticks.append(r.actual_position_6064_ticks)
        xs.append(r.rod_displacement_mm);x_refs.append(r.target_rod_mm)
        auto.append(float(row['pc_autopilot']));veh.append(float(row['veh_brake']))
    fig, ax=plt.subplots(4,1,figsize=(13,11),sharex=True,layout='constrained')
    ax[0].step(ts,auto,where='post',label='pc_autopilot (bruto)')
    ax[0].set_ylabel('Estado (0/1)');ax[0].set_ylim(-.15,1.2);ax[0].legend()
    ax[1].plot(ts,cmd,label='Comando final Bloom Drive (registrado)',lw=1.7)
    ax[1].plot(ts,sim_ticks,label='Posición virtual del modelo (NO medida)',lw=1.7,linestyle='--')
    ax[1].set_ylabel('Unidades de comando Bloom');ax[1].legend(loc='best')
    ax[2].plot(ts,x_refs,label='Referencia convertida a mm (hipótesis)',lw=1.7)
    ax[2].plot(ts,xs,label='Desplazamiento virtual Nanotec (estimado)',lw=1.7,linestyle='--')
    ax[2].set_ylabel('Varilla virtual (mm)');ax[2].legend(loc='best')
    ax[3].step(ts,veh,where='post',label='veh_brake Bloom (valor bruto, NO mm)',lw=1.6)
    ax[3].set_ylabel('veh_brake (bruto)');ax[3].set_xlabel('Tiempo desde comienzo del CSV (s)');ax[3].legend()
    for a in ax:a.grid(alpha=.22)
    fig.suptitle('Bloom Drive LEÓN Run 10249 · Evento 04 | C5-E PROFILE POSITION DIGITAL\n'
                 'Línea discontinua = salida SIMULADA; no existe posición real medida',fontweight='bold')
    dest=root/'vista_previa_LEON.png'
    fig.savefig(dest,dpi=145)
    plt.close(fig)
    print(dest)

if __name__=='__main__':main()

"""Reportes del modelo digital en posición; guarda datos, curvas y advertencias."""
from __future__ import annotations
import csv, math
from datetime import datetime
from pathlib import Path

SIGNALS = [
    ('Seguimiento de posición normalizada','%',[
        ('request_pct','Objetivo de posición','#60A5FA',1),
        ('position_pct','Posición estimada','#34D399',1)],(0,100)),
    ('Referencia y posición del C5-E (virtual)','ticks Bloom',[
        ('target_position_607A_ticks','Objetivo 607Ah (virtual)','#60A5FA',1),
        ('profile_position_6062_ticks','Perfil 6062h (virtual)','#FBBF24',1),
        ('actual_position_6064_ticks','Posición 6064h (virtual)','#34D399',1)],(0,7000)),
    ('Avance de la varilla','mm',[
        ('target_rod_mm','Referencia','#60A5FA',1),
        ('rod_displacement_mm','Posición estimada','#34D399',1),
        ('exposed_rod_mm','Longitud visible','#FBBF24',1)],None),
    ('Giro del pedal','grados',[
        ('pedal_angle_deg','Ángulo desde reposo','#34D399',1)],None),
    ('Fuerza de contacto modelada','N',[
        ('contact_force_n','Fuerza estimada','#FB7185',1)],None),
    ('Corriente equivalente modelada','A',[
        ('motor_current_a','Corriente estimada','#C4B5FD',1)],None),
    ('Velocidad de la varilla','mm/s',[
        ('rod_velocity_mm_s','Velocidad estimada','#FBBF24',1)],None),
]


def value(r,key,mult=1):
    try:
        x=float(r.get(key,float('nan')))
        return x*mult if math.isfinite(x) else float('nan')
    except (ValueError,TypeError):return float('nan')


def timeline(rows):
    key='session_time_s' if all('session_time_s' in r for r in rows) else 'time_s'
    return [float(r.get(key,0)) for r in rows]


def exportar_resultados(registros,base_dir='resultados',prefijo='prueba',generar_png=True):
    rows=list(registros)
    if not rows:raise ValueError('Sin registros')
    base=Path(base_dir);base.mkdir(parents=True,exist_ok=True)
    folder=base/(prefijo+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True)
    fields=sorted(set().union(*(r.keys() for r in rows)))
    p=folder/'datos_completos.csv'
    with p.open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    paths={'carpeta':str(folder.resolve()),'csv':str(p.resolve())}
    t=timeline(rows)
    with (folder/'resumen.txt').open('w',encoding='utf-8') as f:
        f.write('C5-E PROFILE POSITION / 6060h=1 / MODELO DIGITAL, SIN HARDWARE\n')
        f.write('IMPORTANTE: ticks Bloom no validados como unidades nativas C5-E.\n')
        f.write('Ticks/mm, aceleración y respuesta del actuador SON HIPÓTESIS.\n')
        f.write('NO hay encoder real: 6064h es una SALIDA SIMULADA, NO medición Porsche.\n')
        f.write('Freno equivalente es estimación geométrica; NO porcentaje de presión real.\n')
        f.write('BLOOM: pc_autopilot=1 como supuesto (no confirmado).\n\n')
        f.write(f'Muestras: {len(rows)}\nDuración: {t[-1]-t[0]:.3f} s\n')
        for k,label in [('target_position_607A_ticks','Referencia máxima ticks'),('actual_position_6064_ticks','Posición virtual máxima ticks'),('rod_displacement_mm','Recorrido virtual máximo mm'),('position_error_ticks','Error máximo absoluto ticks')]:
            vals=[abs(value(r,k)) if k=='position_error_ticks' else value(r,k) for r in rows]
            vals=[v for v in vals if math.isfinite(v)]
            if vals:f.write(f'{label}: {max(vals):.3f}\n')
    paths['resumen']=str((folder/'resumen.txt').resolve())
    if generar_png:
        try:
            import matplotlib
            matplotlib.use('Agg',force=True)
            import matplotlib.pyplot as plt
        except ImportError:
            paths['aviso']='matplotlib no instalado; solo CSV y resumen. Instala: py -m pip install matplotlib'
            return paths
        fig,axs=plt.subplots(4,2,figsize=(15,15),layout='constrained')
        for ax, (title,unit,series,lim) in zip(axs.flat,SIGNALS):
            for key,label,color,mul in series:
                ax.plot(t,[value(r,key,mul) for r in rows],label=label,color=color,lw=1.5)
            ax.set_title(title,weight='bold');ax.set_xlabel('Tiempo (s)');ax.set_ylabel(unit)
            if lim:ax.set_ylim(*lim)
            ax.grid(alpha=.25);ax.legend(fontsize=7)
        for ax in list(axs.flat)[len(SIGNALS):]:ax.set_axis_off()
        fig.suptitle('Nanotec LGA561 + C5-E · PROFILE POSITION (MODELO NO VALIDADO)',fontsize=14,fontweight='bold')
        pp=folder/'graficas_completas.png';pd=folder/'graficas_completas.pdf'
        fig.savefig(pp,dpi=145);fig.savefig(pd);plt.close(fig)
        paths['png']=str(pp.resolve());paths['pdf']=str(pd.resolve())
        for i,(title,unit,series,lim) in enumerate(SIGNALS,1):
            fig,ax=plt.subplots(figsize=(9,4))
            for key,label,color,mul in series:
                ax.plot(t,[value(r,key,mul) for r in rows],color=color,label=label)
            ax.set_title(title);ax.set_xlabel('Tiempo (s)');ax.set_ylabel(unit)
            if lim:ax.set_ylim(*lim)
            ax.grid(alpha=.25);ax.legend()
            fig.tight_layout();fig.savefig(folder/f'{i:02d}_grafica.png',dpi=150);plt.close(fig)
    return paths

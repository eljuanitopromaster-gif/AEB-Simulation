"""Demo local por UDP, modo posición 6060h=1, sin hardware ni CARLA."""
from __future__ import annotations
import argparse,json,socket,time
from dataclasses import asdict
from pathlib import Path
from pedal_model import PedalPosicion
from reportes import exportar_resultados
DESTINO=('127.0.0.1',5055)
PERIODO=14.0

def solicitud(t):
    t=t%PERIODO
    if t<1:return 0
    if t<3.5:return 25
    if t<6:return 50
    if t<8.5:return 75
    if t<11:return 100
    return 0

def ejecutar(dt=.02,una_vez=False,mm_6900=20.,rapido=False):
    p=PedalPosicion.con_mm_por_6900(mm_6900)
    print('\nC5-E PROFILE POSITION · 6060h=1 · banco DIGITAL sin hardware')
    print(f'Escala ticks/mm asumida: 6900 ticks = {mm_6900:.1f} mm (NO confirmada por Bloom Drive)')
    print('Control 0-100% = % de rango de posición normalizada, NO presión hidráulica')
    print('Abre py visualizador_pedal.py en otra terminal. Ctrl+C termina.\n')
    records=[];sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    ciclo=0
    try:
        while True:
            p.reset();tstart=time.perf_counter()
            for i in range(round(PERIODO/dt)):
                if not rapido:
                    pause=tstart+i*dt-time.perf_counter()
                    if pause>0:time.sleep(pause)
                pct=solicitud(i*dt)
                r=asdict(p.step(pct,dt))
                r['session_time_s']=ciclo*PERIODO+r['time_s'];r['cycle_id']=ciclo+1
                records.append(r)
                sock.sendto(json.dumps(r).encode('utf-8'),DESTINO)
                if i%max(1,round(1/dt))==0:
                    print(f't={ciclo*PERIODO+i*dt:6.2f} s | ref={pct:3.0f}% | '
                          f'posición virtual={r["position_pct"]:5.1f}% | varilla={r["rod_displacement_mm"]:5.2f} mm | '
                          f'error={r["position_error_ticks"]:+7.1f} ticks')
            ciclo+=1
            if una_vez:break
    except KeyboardInterrupt:print('\nSimulación detenida')
    finally:
        sock.close()
        if records:
            r=exportar_resultados(records,Path(__file__).parent/'resultados',prefijo='demo_posicion')
            print('Datos guardados en:',r['carpeta'])

if __name__=='__main__':
    ap=argparse.ArgumentParser()
    ap.add_argument('--dt',type=float,default=.02)
    ap.add_argument('--mm-6900',type=float,default=20.)
    ap.add_argument('--una-vez',action='store_true')
    ap.add_argument('--rapido',action='store_true',help='Simular sin ritmo real, útil para pruebas (no para animación)')
    x=ap.parse_args()
    ejecutar(x.dt,x.una_vez,x.mm_6900,x.rapido)

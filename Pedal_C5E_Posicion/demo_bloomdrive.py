"""Reproducción OFFLINE del comando final LEÓN: jamás controla hardware."""
from __future__ import annotations
import argparse,csv,json,socket,time
from dataclasses import asdict
from pathlib import Path
from pedal_model import PedalPosicion
from reportes import exportar_resultados
DESTINO=('127.0.0.1',5055)
BASE=Path(__file__).resolve().parent

def replay(csv_path,mm_6900=20.,rapido=False,only_autopilot=False):
    with open(csv_path,newline='',encoding='utf-8-sig') as f:
        rows=list(csv.DictReader(f))
    if not rows:raise ValueError('CSV vacío')
    required={'time_relative_s','brake_cmd_final','position_mode_cmd','pc_autopilot','veh_brake'}
    if not required.issubset(rows[0]):raise ValueError('Faltan columnas: '+str(required-set(rows[0])))
    model=PedalPosicion.con_mm_por_6900(mm_6900)
    print('LEÓN 10249 · EVENTO 04 | PROFILE POSITION (6060h=1)')
    print('Se usa brake_cmd_final, NO position_mode_cmd, como orden que entra al modelo.')
    print('Bloom brake_pos/veh_brake son señales brutas, NO desplazamiento medido.')
    print(f'6900 ticks -> {mm_6900:.1f} mm HIPOTÉTICOS; no hay conversión física validada.\n')
    records=[];start=time.perf_counter();prev=0.0
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        for i,row in enumerate(rows):
            t=float(row['time_relative_s'])
            dt=(t-prev) if i else (.01 if len(rows)==1 else float(rows[1]['time_relative_s'])-t)
            dt=max(.0001,dt)
            prev=t
            if not rapido:
                wait=start+t-time.perf_counter()
                if wait>0:time.sleep(wait)
            auto=int(round(float(row['pc_autopilot'])))
            cmd=float(row['brake_cmd_final'])
            if only_autopilot and auto!=1:cmd=0.0
            state=asdict(model.step_target_ticks(cmd,dt))
            state.update({
                'session_time_s':t,
                'bloom_time_s':float(row.get('time_s',t)),
                'bloom_autopilot_raw':auto,
                'bloom_final_command_ticks':float(row['brake_cmd_final']),
                'bloom_position_mode_cmd_ticks':float(row['position_mode_cmd']),
                'bloom_veh_brake_raw':float(row['veh_brake']),
                'bloom_brake_encoder_pos_raw':float(row.get('brake_encoder_pos',0)),
                'bloom_acc_stp':float(row.get('acc_stp',0)),
                'bloom_acc_fbk':float(row.get('acc_fbk',0)),
            })
            records.append(state)
            sock.sendto(json.dumps(state).encode('utf-8'),DESTINO)
            if i%100==0:
                print(f't={t:5.2f}s auto={auto} comando={cmd:7.1f} ticks  '
                      f'posición virtual={state["actual_position_6064_ticks"]:7.1f} ticks / '
                      f'{state["rod_displacement_mm"]:5.2f} mm')
    out=exportar_resultados(records,BASE/'resultados',prefijo='LEON_evento04_posicion')
    print('\nPRUEBA OFFLINE CONCLUIDA. Archivos:',out['carpeta'])
    return records,out

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--csv',default=str(BASE/'datos'/'LEON_Run10249_Evento04.csv'))
    parser.add_argument('--mm-6900',type=float,default=20.)
    parser.add_argument('--rapido',action='store_true')
    parser.add_argument('--solo-autopiloto',action='store_true',help='Anula referencias cuando pc_autopilot=0 (asumiendo 1=activo)')
    args=parser.parse_args()
    replay(args.csv,args.mm_6900,args.rapido,args.solo_autopiloto)

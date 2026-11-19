"""Visualizador EN VIVO del pedal y SEIS curvas que avanzan con el tiempo.
No altera las ecuaciones de pedal_model.py. Por defecto ESCUCHA UDP local.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import socket
import time
import tkinter as tk
from collections import deque
from dataclasses import asdict
from pathlib import Path
from tkinter import filedialog, messagebox

from reportes import SIGNALS, exportar_resultados, value
from visualizador_base import Monitor, BG, PANEL, TEXT, MUTED, CYAN, GREEN, GRID, ORANGE

OUTPUT=Path(__file__).resolve().parent/'resultados'
RESULTADOS_CARLA=Path(__file__).resolve().parent.parent/'results'
WINDOW_S=14.
VELOCIDADES=(1.0,.5,.25)


def _convertir(texto):
    """Celda del CSV de reportes.py -> bool, número o texto."""
    if texto in ('True','False'):
        return texto=='True'
    try:
        return float(texto)
    except (TypeError,ValueError):
        return texto


def cargar_grabacion(ruta):
    """Lee datos_completos.csv (o lo busca dentro de una carpeta, p. ej. la corrida de CARLA)."""
    ruta=Path(ruta)
    if ruta.is_dir():
        candidatos=sorted(ruta.rglob('datos_completos.csv'),key=lambda p:p.stat().st_mtime)
        if not candidatos:
            raise FileNotFoundError(f'No hay datos_completos.csv dentro de {ruta}')
        ruta=candidatos[-1]
    with ruta.open(encoding='utf-8-sig',newline='') as fh:
        filas=[{k:_convertir(v) for k,v in fila.items()} for fila in csv.DictReader(fh)]
    if not filas:
        raise ValueError(f'{ruta.name} no tiene muestras')
    for fila in filas:
        if not isinstance(fila.get('session_time_s'),float):
            fila['session_time_s']=float(fila.get('time_s',0.))
    filas.sort(key=lambda f:f['session_time_s'])
    return filas,ruta


def ultima_grabacion():
    """La grabación más reciente: corridas de CARLA (results/**/pedal) o banco automático (resultados/)."""
    candidatos=[]
    for base in (RESULTADOS_CARLA,OUTPUT):
        if base.exists():
            candidatos.extend(base.rglob('datos_completos.csv'))
    return max(candidatos,key=lambda p:p.stat().st_mtime) if candidatos else None


class MonitorGraficas(Monitor):
    def __init__(self, udp=True, reproducir=None):
        self.registros=[]
        self._ultima_marca=-1.
        self._ultimo_t_local=None
        self._desfase_t=0.
        self._guardado_hasta=0
        self._nuevos=0
        self._inicio_prueba=0          # índice en registros donde empieza la última corrida
        # Reproducción: muestras ya grabadas que se vuelven a dibujar a su ritmo original
        self.grabacion=None            # (filas, nombre)
        self._rep=None                 # estado de la reproducción en curso
        self._vel_idx=0
        self._mostrando_rep=False
        self._origen='vivo'            # qué se mostró por último: 'vivo' o 'grabacion'
        self._ultimo_paquete=None      # reloj del último dato en vivo (para traer la ventana al frente)
        self._encima=False
        super().__init__(udp=udp)
        self.history=deque(maxlen=1800)
        if reproducir:
            self.root.after(300,lambda:self.abrir_grabacion(reproducir))

    # ---------- Ventana junto al video de CARLA ----------
    def acoplar_derecha(self,x0):
        """Ocupa la franja derecha de la pantalla, desde x0 (el video de CARLA queda a la izquierda)."""
        sw,sh=self.root.winfo_screenwidth(),self.root.winfo_screenheight()
        w=max(480,sw-int(x0))
        h=max(560,sh-80)                 # deja libre la barra de tareas
        self.root.minsize(480,560)
        self.root.geometry(f'{w}x{h}+{sw-w}+0')

    def _al_frente(self):
        """Empieza una corrida: la ventana pasa al frente y se queda visible mientras lleguen datos."""
        try:
            self.root.deiconify()
            self.root.lift()
            self.root.attributes('-topmost',True)
            self._encima=True
        except tk.TclError:
            pass

    def _vigilar_en_vivo(self,llego):
        ahora=time.perf_counter()
        if llego:
            if self._ultimo_paquete is None or ahora-self._ultimo_paquete>1.5:
                self._al_frente()
            self._ultimo_paquete=ahora
        elif self._encima and self._ultimo_paquete is not None and ahora-self._ultimo_paquete>2.0:
            self.root.attributes('-topmost',False)   # terminó la corrida: vuelve a ser una ventana normal
            self._encima=False

    # ---------- Reproducción ----------
    def botones_reproduccion(self,parent,bg,fg,acento):
        """Fila de botones: Repetir, Pausa, Velocidad y Abrir grabación."""
        estilo=dict(bg=bg,fg=fg,relief='flat',font=('Segoe UI',9,'bold'),padx=9,pady=7)
        tk.Button(parent,text='▶ REPETIR',command=self.repetir,bg=acento,fg='white',
                  activebackground=acento,relief='flat',font=('Segoe UI',10,'bold'),
                  padx=13,pady=7).pack(side='left',padx=(10,3))
        self._btn_pausa=tk.Button(parent,text='⏸ Pausa',command=self.pausar,**estilo)
        self._btn_pausa.pack(side='left',padx=3)
        self._btn_vel=tk.Button(parent,text='Velocidad 1x',command=self.cambiar_velocidad,**estilo)
        self._btn_vel.pack(side='left',padx=3)
        tk.Button(parent,text='📂 Abrir grabación',command=self.elegir_grabacion,**estilo).pack(side='left',padx=3)

    def repetir(self):
        """Vuelve a reproducir lo último recibido; si no hay nada, la última grabación guardada."""
        ultima=self.registros[self._inicio_prueba:]
        if self.grabacion and (self._origen=='grabacion' or not ultima):
            filas,nombre=self.grabacion
        elif ultima:
            filas,nombre=[dict(r) for r in ultima],'última prueba recibida'
        else:
            ruta=ultima_grabacion()
            if ruta is None:
                messagebox.showinfo('Sin datos','Todavía no hay ninguna prueba para repetir.')
                return
            self.abrir_grabacion(ruta)
            return
        self._iniciar_reproduccion(filas,nombre)

    def elegir_grabacion(self):
        inicial=RESULTADOS_CARLA/'porsche'/'ensayos'
        ruta=filedialog.askopenfilename(
            title='Elige datos_completos.csv del pedal',
            initialdir=str(inicial if inicial.exists() else OUTPUT),
            filetypes=[('Datos del pedal','datos_completos.csv'),('CSV','*.csv')])
        if ruta:
            self.abrir_grabacion(ruta)

    def abrir_grabacion(self,ruta):
        try:
            filas,archivo=cargar_grabacion(ruta)
        except Exception as exc:
            messagebox.showerror('No se pudo abrir la grabación',str(exc))
            return
        partes=archivo.parts
        # nombre legible: la corrida de CARLA (…/ensayos/<corrida>/pedal/…) o la carpeta del CSV
        nombre=partes[partes.index('pedal')-1] if 'pedal' in partes else archivo.parent.name
        self.grabacion=(filas,nombre)
        self._origen='grabacion'
        self._iniciar_reproduccion(filas,nombre)

    def _iniciar_reproduccion(self,filas,nombre):
        self.history.clear()
        self.latest=None
        self._rep={'filas':filas,'i':0,'nombre':nombre,'t0':float(filas[0]['session_time_s']),
                   'cursor':float(filas[0]['session_time_s']),'pausa':False,
                   'reloj':time.perf_counter(),'fin':float(filas[-1]['session_time_s'])}
        self._mostrando_rep=True
        self._btn_pausa.config(text='⏸ Pausa')

    def pausar(self):
        if not self._rep:
            return
        self._rep['pausa']=not self._rep['pausa']
        self._rep['reloj']=time.perf_counter()
        self._btn_pausa.config(text='▶ Seguir' if self._rep['pausa'] else '⏸ Pausa')

    def cambiar_velocidad(self):
        self._vel_idx=(self._vel_idx+1)%len(VELOCIDADES)
        self._btn_vel.config(text=f'Velocidad {VELOCIDADES[self._vel_idx]:g}x')

    def _avanzar_reproduccion(self):
        rep=self._rep
        ahora=time.perf_counter()
        if not rep['pausa']:
            rep['cursor']+=(ahora-rep['reloj'])*VELOCIDADES[self._vel_idx]
        rep['reloj']=ahora
        filas=rep['filas']
        while rep['i']<len(filas) and float(filas[rep['i']]['session_time_s'])<=rep['cursor']:
            fila=filas[rep['i']]
            self.history.append(fila)
            self.latest=fila
            rep['i']+=1
        transcurrido=min(rep['cursor'],rep['fin'])-rep['t0']
        estado='EN PAUSA' if rep['pausa'] else 'REPRODUCIENDO'
        if rep['i']>=len(filas):
            estado='REPETICIÓN TERMINADA (pulsa ▶ REPETIR para verla otra vez)'
            self._rep=None
        self.status.set(f'{estado} | {rep["nombre"]} | {transcurrido:.2f} / {rep["fin"]-rep["t0"]:.2f} s | '
                        f'velocidad {VELOCIDADES[self._vel_idx]:g}x | {len(filas)} muestras')

    def _mostrar_metricas(self):
        for key,(var,unit) in self.metrics.items():
            v=value(self.latest,key,100 if key=='brake_fraction' else 1)
            var.set(f'{v:.0f} {unit}' if key=='target_position_607A_ticks' else f'{v:.1f} {unit}')

    def build(self):
        self.root.title('Pedal Nanotec + C5-E | Animación y gráficas EN VIVO')
        self.root.geometry('1250x965')
        self.root.minsize(1000,850)

        header=tk.Frame(self.root,bg=BG)
        header.pack(fill='x',padx=18,pady=(12,4))
        tk.Label(header,text='BANCO VIRTUAL DE FRENO  ·  NANOTEC + C5-E',
                 bg=BG,fg=TEXT,font=('Segoe UI',17,'bold')).pack(anchor='w')
        tk.Label(header,text='PROFILE POSITION  |  Pedal 21 cm  |  Empuje a 11 cm del pivote  |  70 mm visibles en reposo  |  Modelo por calibrar',
                 bg=BG,fg=MUTED,font=('Segoe UI',9)).pack(anchor='w',pady=(1,0))

        controls=tk.Frame(self.root,bg=PANEL)
        controls.pack(fill='x',padx=18,pady=(4,7))
        if not self.udp:
            tk.Label(controls,text='POSICIÓN SOLICITADA:',fg=TEXT,bg=PANEL,
                     font=('Segoe UI',9,'bold')).pack(side='left',padx=(12,5))
            tk.Scale(controls,from_=0,to=100,orient='horizontal',variable=self.request,
                     resolution=1,length=210,showvalue=True,bg=PANEL,fg=TEXT,
                     activebackground=CYAN,troughcolor=GRID,highlightthickness=0).pack(side='left')
            for pct in (0,25,50,75,100):
                tk.Button(controls,text=f'{pct}%',bg=GRID,fg=TEXT,relief='flat',
                          command=lambda p=pct: self.request.set(p)).pack(side='left',padx=3)
            tk.Button(controls,text='Resetear pedal',bg='#72502e',fg=TEXT,
                      relief='flat',command=self.reset).pack(side='left',padx=6)
        else:
            tk.Label(controls,text='● EN VIVO  ·  UDP 127.0.0.1:5055',
                     fg=GREEN,bg=PANEL,font=('Segoe UI',10,'bold')).pack(side='left',padx=15,pady=13)
        self.botones_reproduccion(controls,GRID,TEXT,'#2563EB')
        tk.Button(controls,text='GUARDAR GRÁFICAS + CSV',command=self.guardar,
                  bg='#15826c',fg='white',activebackground=GREEN,relief='flat',
                  font=('Segoe UI',10,'bold'),padx=15,pady=8).pack(side='right',padx=10,pady=5)
        tk.Button(controls,text='Limpiar curvas',command=self.limpiar_curvas,
                  bg=GRID,fg=TEXT,relief='flat',padx=10,pady=7).pack(side='right',padx=2)

        self.canvas=tk.Canvas(self.root,height=295,bg=PANEL,highlightthickness=0)
        self.canvas.pack(fill='x',padx=18,pady=(0,6))
        metrics=tk.Frame(self.root,bg=BG)
        metrics.pack(fill='x',padx=14)
        self.metrics={}
        names=[('AEB pedido','request_pct','%'),('Posición virtual','position_pct','%'),
               ('Corriente motor','motor_current_a','A'),('Varilla visible','exposed_rod_mm','mm'),
               ('Avance varilla','rod_displacement_mm','mm'),('Ángulo pedal','pedal_angle_deg','°'),
               ('Contacto','contact_force_n','N'),('Freno equivalente','brake_fraction','%')]
        for i,(label,key,unit) in enumerate(names):
            tile=tk.Frame(metrics,bg=PANEL)
            tile.grid(row=i//4,column=i%4,sticky='nsew',padx=4,pady=2)
            metrics.columnconfigure(i%4,weight=1)
            tk.Label(tile,text=label,bg=PANEL,fg=MUTED,font=('Segoe UI',9)).pack(pady=(5,0))
            var=tk.StringVar(value='—')
            tk.Label(tile,textvariable=var,bg=PANEL,fg=GREEN if key=='brake_fraction' else TEXT,
                     font=('Segoe UI',13,'bold')).pack(pady=(0,4))
            self.metrics[key]=(var,unit)

        tk.Label(self.root,text='GRÁFICAS EN TIEMPO REAL  ·  Ventana deslizante de los últimos 14 segundos',
                 fg=TEXT,bg=BG,font=('Segoe UI',11,'bold')).pack(anchor='w',padx=20,pady=(8,3))
        charts=tk.Frame(self.root,bg=BG)
        charts.pack(fill='both',expand=True,padx=14,pady=(0,4))
        self.charts=[]
        for k,(title,unit,series,limits) in enumerate(SIGNALS):
            tile=tk.Frame(charts,bg=PANEL)
            tile.grid(row=k//3,column=k%3,sticky='nsew',padx=4,pady=4)
            chart=tk.Canvas(tile,height=133,bg=PANEL,highlightthickness=0)
            chart.pack(fill='both',expand=True)
            self.charts.append(chart)
        for col in range(3):charts.columnconfigure(col,weight=1)
        for row in range(2):charts.rowconfigure(row,weight=1)
        tk.Label(self.root,textvariable=self.status,bg=BG,fg=MUTED,
                 anchor='w',font=('Segoe UI',9)).pack(fill='x',padx=22,pady=(0,8))

    def reset(self):
        self.model.reset()
        self.request.set(0.)
        self.limpiar_curvas()

    def limpiar_curvas(self):
        self.history.clear()
        self.registros.clear()
        self._ultima_marca=-1.
        self._ultimo_t_local=None
        self._desfase_t=0.
        self._guardado_hasta=0
        self._inicio_prueba=0
        self._rep=None
        self._mostrando_rep=False
        self.latest=None

    def _registrar(self, row):
        if not isinstance(row,dict):return
        record=row.copy()
        self._origen='vivo'
        if self._mostrando_rep:
            # Llegan datos en vivo: tienen prioridad sobre la repetición.
            self._rep=None
            self._mostrando_rep=False
            self.history.clear()
        if 'session_time_s' not in record:
            local=float(record.get('time_s',0))
            if self._ultimo_t_local is not None and local < self._ultimo_t_local-0.2:
                self._desfase_t+=self._ultimo_t_local
                self._inicio_prueba=len(self.registros)   # empezó otra corrida (CARLA reinicia el reloj)
            elif self._ultimo_t_local is None:
                self._inicio_prueba=len(self.registros)
            self._ultimo_t_local=local
            record['session_time_s']=local+self._desfase_t
        t=float(record['session_time_s'])
        # Evita duplicar puntos al refrescar la ventana o al recibir el mismo paquete.
        if t <= self._ultima_marca+1e-8:
            return
        self._ultima_marca=t
        self.registros.append(record)
        self.history.append(record)
        self.latest=record
        self._nuevos+=1

    def update(self):
        if self.udp:
            llego=False
            try:
                while True:
                    packet,_=self.sock.recvfrom(16000)
                    self._registrar(json.loads(packet.decode('utf-8')))
                    llego=True
            except BlockingIOError:
                pass
            except (ValueError,UnicodeError,OSError):
                pass
            self._vigilar_en_vivo(llego)
        elif not self._mostrando_rep:
            self._registrar(asdict(self.model.step(self.request.get(),.02)))

        if self._mostrando_rep:
            if self._rep:
                self._avanzar_reproduccion()
            if self.latest:
                self._mostrar_metricas()
        elif self.latest:
            self._mostrar_metricas()
            s=self.latest
            self.status.set(
                f'{"TELEMETRÍA UDP" if self.udp else "BANCO MANUAL"} | '
                f't={s.get("session_time_s",0):.2f} s | '
                f'{"CONTACTO" if s.get("contact") else "SIN CONTACTO"} | '
                f'{"TOPE: límite provisional" if s.get("saturation_warning") else ("TOPE" if s.get("at_forward_limit") else "SIN TOPE")} | '
                f'{len(self.registros)} muestras | ⚠ Modelo mecánico y eléctrico por calibrar')
        self.draw_mechanism()
        self.draw_chart()
        self.root.after(40,self.update)

    def draw_mechanism(self):
        # Reutiliza la animación física EXACTA del visualizador anterior, a escala uniforme.
        super().draw_mechanism()
        self.canvas.scale('all',0,0,.62,.62)
        self.canvas.move('all',18,0)

    def draw_chart(self):
        if not self.history: return
        last_time=float(self.history[-1].get('session_time_s',0.))
        start=max(0,last_time-WINDOW_S)
        data=[p for p in self.history if float(p.get('session_time_s',0))>=start]
        end=max(WINDOW_S,last_time)
        left=end-WINDOW_S
        for canvas,(title,unit,series,limits) in zip(self.charts,SIGNALS):
            canvas.delete('all')
            w=max(300,canvas.winfo_width());h=max(120,canvas.winfo_height())
            px0,px1=52,w-12;py0,py1=40,h-27
            allvals=[]
            for key,_,__,factor in series:
                allvals.extend(value(row,key,factor) for row in data)
            allvals=[v for v in allvals if math.isfinite(v)]
            if limits:
                vmin,vmax=limits
            else:
                vmin=min(allvals+[0]);vmax=max(allvals+[0]);pad=max(.1,(vmax-vmin)*.12)
                vmin-=pad;vmax+=pad
            rng=max(1e-12,vmax-vmin)
            canvas.create_text(10,12,text=title,fill=TEXT,anchor='w',font=('Segoe UI',10,'bold'))
            for key,label,color,mult in series:
                pass
            for i,(key,label,color,mult) in enumerate(series):
                xx=10+i*(w-10)/len(series)
                canvas.create_line(xx,28,xx+12,28,fill=color,width=3)
                canvas.create_text(xx+16,28,text=label,fill=MUTED,anchor='w',font=('Segoe UI',7))
            for u in range(4):
                y=py0+(py1-py0)*u/3
                canvas.create_line(px0,y,px1,y,fill=GRID,dash=(2,4))
                v=vmax-u*rng/3
                canvas.create_text(px0-6,y,text=f'{v:.1f}',fill=MUTED,anchor='e',font=('Segoe UI',7))
            for i in range(5):
                x=px0+(px1-px0)*i/4
                canvas.create_text(x,py1+13,text=f'{left+WINDOW_S*i/4:.1f}s',fill=MUTED,font=('Segoe UI',7))
            for key,label,color,mult in series:
                pts=[]
                # Limitar puntos por columna para que Tkinter siga fluido con telemetría densa.
                stride=max(1,len(data)//max(1,int(w)))
                for d in data[::stride]:
                    t=float(d['session_time_s'])
                    yval=value(d,key,mult)
                    if math.isfinite(yval):
                        xx=px0+(t-left)/WINDOW_S*(px1-px0)
                        yy=py1-(yval-vmin)/rng*(py1-py0)
                        pts.extend((xx,yy))
                if len(pts)>=4:
                    canvas.create_line(*pts,fill=color,width=2,smooth=False)

    def guardar(self):
        if not self.registros:
            messagebox.showinfo('Sin datos','Primero ejecuta una prueba para recibir mediciones.')
            return
        try:
            paths=exportar_resultados(self.registros,OUTPUT,prefijo='visualizador')
            self._guardado_hasta=len(self.registros)
        except Exception as exc:
            messagebox.showerror('No se pudo exportar',str(exc))
            return
        msg='Archivos guardados en:\n'+paths['carpeta']+'\n\nCSV + resumen'
        msg+=' + gráficas PNG y PDF' if 'png' in paths else '\n'+paths.get('aviso','')
        messagebox.showinfo('Prueba exportada',msg)

    def close(self):
        if self.registros and len(self.registros)>self._guardado_hasta:
            try:
                paths=exportar_resultados(self.registros,OUTPUT,prefijo='cierre_automatico')
                print('Datos guardados al cerrar:',paths['carpeta'])
            except Exception as exc:
                print('No se pudo guardar al cerrar:',exc)
        super().close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description='Pedal en tiempo real con gráficas deslizantes y descarga de resultados')
    p.add_argument('--manual',action='store_true',help='Comandos manuales sin emisor')
    p.add_argument('--reproducir',metavar='RUTA',
                   help='Repite una grabación: datos_completos.csv o la carpeta de una corrida de CARLA')
    args=p.parse_args()
    MonitorGraficas(udp=not args.manual,reproducir=args.reproducir).run()

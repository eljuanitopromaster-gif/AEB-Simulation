"""Panel original: pedal grande, 5 indicadores, curva grande, y
curvas adicionales abajo. Escucha UDP por defecto; --manual simula localmente.
Utiliza la física de Profile Position (6060h=1).
"""
from __future__ import annotations

import argparse
import math
import tkinter as tk
from tkinter import messagebox

from reportes import SIGNALS, value
from visualizador_compacto import MonitorGraficas, WINDOW_S

BG = '#0D1728'
WHITE = '#17263D'
DARK = '#EFF6FF'
MUTED = '#A8BAD1'
BLUE = '#60A5FA'
GREEN = '#34D399'
ORANGE = '#FBBF24'
GRID = '#30425C'
PANEL_EDGE = '#344963'

COLORS = {
    'request_pct': BLUE,
    'position_pct': GREEN,
    'target_position_607A_ticks': BLUE,
    'profile_position_6062_ticks': ORANGE,
    'actual_position_6064_ticks': GREEN,
    'target_rod_mm': BLUE,
    'rod_displacement_mm': GREEN,
    'exposed_rod_mm': ORANGE,
    'pedal_angle_deg': GREEN,
    'contact_force_n': '#FB7185',
    'motor_current_a': '#C4B5FD',
    'rod_velocity_mm_s': ORANGE,
}


def modo_claro():
    """Restaurar paleta clara mediante --claro, sin afectar las ecuaciones."""
    global BG,WHITE,DARK,MUTED,BLUE,GREEN,ORANGE,GRID,PANEL_EDGE,COLORS
    BG='#F4F7FC';WHITE='#FFFFFF';DARK='#152C4D';MUTED='#8294B0'
    BLUE='#1563B9';GREEN='#16947C';ORANGE='#D59A2B'
    GRID='#DFE6EF';PANEL_EDGE='#E0E7F1'
    for key in ('request_pct','target_position_607A_ticks','target_rod_mm'):
        COLORS[key]=BLUE
    for key in ('position_pct','actual_position_6064_ticks','rod_displacement_mm','pedal_angle_deg'):
        COLORS[key]=GREEN
    for key in ('profile_position_6062_ticks','exposed_rod_mm','rod_velocity_mm_s'):
        COLORS[key]=ORANGE


class MonitorEstiloOriginal(MonitorGraficas):
    def build(self):
        r = self.root
        r.configure(bg=BG)
        r.title('Pedal AEB | Nanotec LGA561 + C5-E · PROFILE POSITION')
        r.geometry('1130x940')
        r.minsize(890,740)
        title = tk.Frame(r, bg=BG)
        title.pack(fill='x', padx=28, pady=(17,7))
        tk.Label(title, text='PEDAL DE FRENO  •  BANCO DIGITAL',
                 bg=BG,fg=DARK,font=('Segoe UI',22,'bold')).pack(anchor='w')
        tk.Label(title,
                 text='21 cm  |  Empuje a 11 cm del pivote  |  Nanotec + C5-E en PROFILE POSITION (6060h=1)  |  Modelo estimado',
                 bg=BG,fg=MUTED,font=('Segoe UI',10)).pack(anchor='w',pady=(7,0))

        controls=tk.Frame(r,bg=WHITE,highlightbackground=PANEL_EDGE,highlightthickness=1)
        controls.pack(fill='x',padx=28,pady=(3,9))
        if self.udp:
            tk.Label(controls,text='●  EN VIVO  |  UDP 127.0.0.1:5055',
                     fg=GREEN,bg=WHITE,font=('Segoe UI',10,'bold')).pack(side='left',padx=13,pady=12)
        else:
            tk.Label(controls,text='Referencia de posición (%):',fg=DARK,bg=WHITE,
                     font=('Segoe UI',10,'bold')).pack(side='left',padx=(10,5))
            tk.Scale(controls,from_=0,to=100,orient='horizontal',length=185,
                     resolution=1,variable=self.request,bg=WHITE,fg=DARK,
                     highlightthickness=0,troughcolor=GRID,activebackground=BLUE).pack(side='left')
            for pct in (0,25,50,75,100):
                tk.Button(controls,text=f'{pct}%',bg=BG,fg=DARK,relief='flat',
                          command=lambda p=pct:self.request.set(p),padx=5).pack(side='left',padx=2)
            tk.Button(controls,text='Reiniciar',bg=BG,fg=DARK,relief='flat',
                      command=self.reset).pack(side='left',padx=8)
        self.botones_reproduccion(controls,BG,DARK,BLUE)

        tk.Button(controls,text='GUARDAR GRÁFICAS + CSV',bg=GREEN,fg='white',
                  font=('Segoe UI',9,'bold'),relief='flat',activebackground='#0B7965',
                  command=self.guardar,padx=14,pady=9).pack(side='right',padx=10,pady=5)
        tk.Button(controls,text='Limpiar curvas',bg=BG,fg=DARK,relief='flat',
                  command=self.limpiar_curvas,padx=8,pady=8).pack(side='right')

        # Desplazamiento vertical para conservar la gráfica grande de la captura.
        outer = tk.Frame(r,bg=BG)
        outer.pack(fill='both',expand=True,padx=(28,15))
        self.scroller=tk.Canvas(outer,bg=BG,highlightthickness=0)
        self.scroller.pack(side='left',fill='both',expand=True)
        sb=tk.Scrollbar(outer,orient='vertical',command=self.scroller.yview)
        sb.pack(side='right',fill='y')
        self.scroller.configure(yscrollcommand=sb.set)
        self.inner=tk.Frame(self.scroller,bg=BG)
        self.win=self.scroller.create_window((0,0),window=self.inner,anchor='nw')
        self.inner.bind('<Configure>',lambda e:self.scroller.configure(scrollregion=self.scroller.bbox('all')))
        self.scroller.bind('<Configure>',lambda e:self.scroller.itemconfigure(self.win,width=e.width))
        self.scroller.bind_all('<MouseWheel>',self._wheel)

        mechanism=tk.Frame(self.inner,bg=WHITE,highlightthickness=1,highlightbackground=PANEL_EDGE)
        mechanism.pack(fill='x',pady=(0,10))
        tk.Label(mechanism,text='Mecanismo esquemático  •  Vista lateral',
                 bg=WHITE,fg=DARK,font=('Segoe UI',14,'bold')).pack(anchor='w',padx=25,pady=(16,0))
        self.canvas=tk.Canvas(mechanism,bg=WHITE,height=383,highlightthickness=0)
        self.canvas.pack(fill='x',padx=(8,8))
        tk.Label(mechanism,
                 text='Modelo SOLO digital. Ticks/mm, respuesta dinámica y posición real del actuador pendientes de calibración.',
                 bg=WHITE,fg=MUTED,font=('Segoe UI',9),anchor='w').pack(fill='x',padx=25,pady=(0,13))

        metrics=tk.Frame(self.inner,bg=BG)
        metrics.pack(fill='x',pady=(0,10))
        self.metrics={}
        names=[('Posición pedida','request_pct','%'),
               ('Posición virtual','position_pct','%'),
               ('Varilla','rod_displacement_mm','mm'),
               ('Ángulo pedal','pedal_angle_deg','°'),
               ('Fuerza contacto','contact_force_n','N')]
        for i,(label,key,unit) in enumerate(names):
            card=tk.Frame(metrics,bg=WHITE,highlightbackground=PANEL_EDGE,highlightthickness=1)
            card.grid(row=0,column=i,sticky='nsew',padx=(0 if i==0 else 4,0 if i==4 else 4))
            metrics.columnconfigure(i,weight=1,uniform='stats')
            tk.Label(card,text=label,bg=WHITE,fg=MUTED,font=('Segoe UI',10)).pack(pady=(14,7))
            var=tk.StringVar(value='0.0 '+unit)
            tk.Label(card,textvariable=var,bg=WHITE,
                     fg=GREEN if key=='position_pct' else DARK,
                     font=('Segoe UI',17,'bold')).pack(pady=(0,17))
            self.metrics[key]=(var,unit)

        main=tk.Frame(self.inner,bg=WHITE,highlightthickness=1,highlightbackground=PANEL_EDGE)
        main.pack(fill='x',pady=(0,12))
        tk.Label(main,text='SEGUIMIENTO DE POSICIÓN EN VIVO',bg=WHITE,fg=DARK,
                 font=('Segoe UI',13,'bold')).pack(anchor='w',padx=25,pady=(14,0))
        self.charts=[]
        main_chart=tk.Canvas(main,bg=WHITE,height=215,highlightthickness=0)
        main_chart.pack(fill='x',padx=(18,15),pady=(0,11))
        self.charts.append(main_chart)

        tk.Label(self.inner,text='MÁS VARIABLES  •  GRÁFICAS EN TIEMPO REAL (VIRTUALES)',bg=BG,fg=DARK,
                 font=('Segoe UI',12,'bold')).pack(anchor='w',pady=(4,6))
        graph_grid=tk.Frame(self.inner,bg=BG)
        graph_grid.pack(fill='x',pady=(0,12))
        for i, sig in enumerate(SIGNALS[1:]):
            frame=tk.Frame(graph_grid,bg=WHITE,highlightthickness=1,highlightbackground=PANEL_EDGE)
            frame.grid(row=i//2,column=i%2,sticky='nsew',padx=(0,9) if i%2==0 else (0,0),pady=(0,9))
            chart=tk.Canvas(frame,bg=WHITE,height=192,highlightthickness=0)
            chart.pack(fill='x',padx=8,pady=4)
            self.charts.append(chart)
        graph_grid.columnconfigure(0,weight=1,uniform='graph')
        graph_grid.columnconfigure(1,weight=1,uniform='graph')

        tk.Label(r,textvariable=self.status,bg=BG,fg=MUTED,
                 font=('Segoe UI',9),anchor='w').pack(fill='x',padx=29,pady=(5,7))

    def _wheel(self,event):
        if self.scroller.winfo_exists():
            self.scroller.yview_scroll(-int(event.delta/120), 'units')

    def draw_mechanism(self):
        c=self.canvas
        c.delete('all')
        w=max(800,c.winfo_width())
        g=self.model.geometry
        latest=self.latest or {}
        angle=math.radians(g.angulo_reposo_deg+float(latest.get('pedal_angle_deg',0)))
        initial=math.radians(g.angulo_reposo_deg)
        x0=min(w*.225,260)
        y0=58
        length=min(310,w*.29)
        fact=length/g.largo_pedal_m
        end_x=x0+length*math.cos(angle)
        end_y=y0+length*math.sin(angle)
        c.create_line(x0,y0,x0+length*math.cos(initial),y0+length*math.sin(initial),
                      fill='#C8D8EB',width=8,dash=(5,7))
        c.create_line(x0,y0,end_x,end_y,fill=BLUE,width=11,capstyle='round')
        # Pedal de goma, orientado perpendicularmente al brazo.
        tx,ty=math.cos(angle),math.sin(angle)
        nx,ny=-ty,tx
        pad=[]
        for a,b in ((-34,-20),(34,-20),(34,20),(-34,20)):
            pad.extend((end_x+nx*a+tx*b,end_y+ny*a+ty*b))
        c.create_polygon(*pad,fill='#35445F',outline='#253148',width=2)
        for k in range(-2,3):
            px=end_x+nx*k*10
            py=end_y+ny*k*10
            c.create_line(px-tx*9,py-ty*9,px+tx*9,py+ty*9,fill='#556783',width=1)
        c.create_oval(x0-11,y0-11,x0+11,y0+11,fill='#475B7A',outline='#CCD6E6',width=3)
        c.create_text(x0-4,y0-28,text='Pivote',fill=DARK,font=('Segoe UI',11),anchor='s')

        cy=y0+fact*g.altura_contacto_m
        cx=x0+fact*g.altura_contacto_m/math.tan(angle)
        extension=float(latest.get('rod_displacement_mm',0))
        tip0=x0+fact*(g.x_contacto_reposo_m+g.holgura_inicial_m)
        tip_x=tip0-fact*extension/1000
        motor_face=min(w-170,max(w*.76,cx+220))
        motor_face=max(motor_face,tip_x+25)
        motor_end=min(w-20,motor_face+138)
        # motor = fijo; la punta se desplaza hacia la izquierda al extender.
        c.create_rectangle(motor_face,cy-48,motor_end,cy+48,
                           fill='#222D42',outline='#6A7C94',width=3)
        c.create_rectangle(motor_face+14,cy-34,motor_end-13,cy+34,
                           fill='#334158',outline='#647994')
        c.create_text((motor_face+motor_end)/2,cy,text='NANOTEC',
                      fill='#FFFFFF',font=('Segoe UI',12,'bold'))
        c.create_rectangle(motor_face-9,cy-14,motor_face+5,cy+14,fill='#BAC5D5',outline='#7D90AB')
        c.create_line(tip_x,cy,motor_face,cy,fill='#828B9B',width=12)
        for p in range(9):
            xx=tip_x+12+p*(motor_face-tip_x-20)/9
            if xx<motor_face-10:
                c.create_line(xx,cy-5,xx+7,cy+5,fill='#CCD4DF',width=2)
        engaged=bool(latest.get('contact'))
        c.create_oval(tip_x-10,cy-10,tip_x+10,cy+10,
                      fill=GREEN if engaged else ORANGE,outline=WHITE,width=2)
        # Contacto con el pedal a la altura del eje del actuador.
        c.create_oval(cx-8,cy-8,cx+8,cy+8,fill=ORANGE,outline=WHITE,width=2)
        if abs(cx-tip_x)>2:
            c.create_line(tip_x,cy+16,cx,cy+16,dash=(3,4),fill=ORANGE,width=2)
        c.create_text(cx-18,cy+26,text='11 cm',anchor='e',fill=ORANGE,font=('Segoe UI',10,'bold'))
        c.create_text((tip_x+motor_face)/2,cy-29,
                      text=f'Varilla {extension:.1f} / {float(latest.get("limite_carrera_mm", 1000*g.carrera_disponible_desde_reposo_m)):.1f} mm',
                      fill=DARK,font=('Segoe UI',11,'bold'))
        c.create_text((tip_x+motor_face)/2,cy+30,
                      text=f'Visible: {float(latest.get("exposed_rod_mm",70)):.1f} mm en este modelo',
                      fill=MUTED,font=('Segoe UI',9))
        brake=value(latest,'position_pct',1)
        c.create_text(w/2,357,
                      text=f'Pedal: {float(latest.get("pedal_angle_deg",0)):.2f}°   |   Posición simulada: {brake:.1f}%',
                      fill=GREEN,font=('Segoe UI',13,'bold'))
        c.create_line(24,376,w-24,376,fill=GRID,width=1)

    def draw_chart(self):
        if not self.charts:
            return
        samples=list(self.history)
        t_end=float(samples[-1].get('session_time_s',0)) if samples else WINDOW_S
        t_left=max(0.,t_end-WINDOW_S)
        t_right=t_left+WINDOW_S
        for n,(c,(title,unit,series,limits)) in enumerate(zip(self.charts,SIGNALS)):
            c.delete('all')
            w=max(380,c.winfo_width())
            h=max(165,c.winfo_height())
            main=(n==0)
            top=43 if main else 50
            bottom=h-48
            left=92
            right=w-22
            if main:
                keys=series
                legend=[('request_pct','Posición solicitada'),('position_pct','Posición simulada')]
            else:
                c.create_text(15,17,text=title,anchor='w',fill=DARK,font=('Segoe UI',11,'bold'))
                legend=[(key,label) for key,label,_,_ in series]
            # leyenda con intervalos proporcionados
            for j,(key,label) in enumerate(legend):
                x=left+j*(right-left)/max(2,len(legend))
                c.create_line(x,top-18,x+16,top-18,fill=COLORS.get(key,BLUE),width=3)
                c.create_text(x+21,top-18,text=label,anchor='w',fill=MUTED,font=('Segoe UI',8))
            data=[row for row in samples if float(row.get('session_time_s',row.get('time_s',0)))>=t_left]
            ys=[]
            for key,_,_,mult in series:
                ys.extend(value(d,key,mult) for d in data)
            ys=[f for f in ys if math.isfinite(f)]
            if limits:
                vmin,vmax=limits
            else:
                vmin=min([0.]+ys)
                vmax=max([0.]+ys)
                pad=max(.05,.12*(vmax-vmin))
                vmin-=pad;vmax+=pad
            rng=max(1.e-9,vmax-vmin)
            for j in range(5):
                yy=top+(bottom-top)*j/4
                c.create_line(left,yy,right,yy,fill=GRID,width=1)
                val=vmax-rng*j/4
                c.create_text(left-7,yy,text=f'{val:.1f}' if not main else f'{val:.0f}',
                              fill=MUTED,anchor='e',font=('Segoe UI',8))
            for j in range(5):
                xx=left+(right-left)*j/4
                tt=t_left+(t_right-t_left)*j/4
                c.create_text(xx,bottom+12,text=f'{tt:.1f}s',fill=MUTED,font=('Segoe UI',8))
            c.create_text((left+right)/2,h-9,text='Tiempo (s)',fill=MUTED,font=('Segoe UI',9,'bold'))
            nombres_y = {'%':'Posición (%)','ticks Bloom':'Posición (ticks)',
                         'mm':'Longitud (mm)','grados':'Ángulo (°)','N':'Fuerza (N)',
                         'A':'Corriente (A)','mm/s':'Velocidad (mm/s)'}
            c.create_text(13,(top+bottom)/2,text=nombres_y.get(unit,unit),angle=90,
                          fill=MUTED,font=('Segoe UI',9,'bold'))
            max_points=max(300,int(w*1.2))
            step=max(1,len(data)//max_points)
            for key,_,_,mult in series:
                pts=[]
                for rec in data[::step]:
                    t=float(rec.get('session_time_s',rec.get('time_s',0)))
                    y=value(rec,key,mult)
                    if not math.isfinite(y):
                        continue
                    xx=left+(t-t_left)/(t_right-t_left)*(right-left)
                    yy=bottom-(y-vmin)/rng*(bottom-top)
                    pts.extend((xx,yy))
                if len(pts)>=4:
                    c.create_line(*pts,fill=COLORS.get(key,BLUE),width=2.7 if main else 2,smooth=False)

    def close(self):
        self.scroller.unbind_all('<MouseWheel>')
        super().close()

if __name__=='__main__':
    pa=argparse.ArgumentParser(description='Pedal estilo original con gráficas en vivo y exportación')
    pa.add_argument('--manual',action='store_true',help='Usar controles manuales en la misma ventana')
    pa.add_argument('--claro',action='store_true',help='Cambiar al tema claro')
    pa.add_argument('--reproducir',metavar='RUTA',
                    help='Repite una grabación: datos_completos.csv o la carpeta de una corrida de CARLA')
    pa.add_argument('--lado',type=int,metavar='X',
                    help='Coloca la ventana en la franja derecha de la pantalla, desde X px (junto al video de CARLA)')
    args=pa.parse_args()
    if args.claro: modo_claro()
    monitor=MonitorEstiloOriginal(udp=not args.manual,reproducir=args.reproducir)
    if args.lado is not None:
        monitor.acoplar_derecha(args.lado)
    monitor.run()

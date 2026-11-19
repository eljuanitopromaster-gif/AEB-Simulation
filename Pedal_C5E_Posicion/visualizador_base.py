"""Ventana para telemetría en vivo del pedal en modo posición, sin CARLA.
Por defecto recibe demo_telemetria.py o el escenario CARLA por UDP local.
Con --manual también permite simular localmente mediante botones.
Usa Tkinter, incluido normalmente en Python de Windows.
"""
import argparse, json, math, socket, time, tkinter as tk
from collections import deque
from dataclasses import asdict
from tkinter import ttk
from pedal_model import PedalPosicion

BG='#0d1728'; PANEL='#17263d'; TEXT='#e9f0fb'; MUTED='#a8bad1'
CYAN='#52d5ed'; GREEN='#3fdfa1'; ORANGE='#ffb25c'; GRID='#30425c'

class Monitor:
    def __init__(self, udp=False):
        self.udp=udp
        self.model=PedalPosicion()
        self.sock=None
        if udp:
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.sock.bind(('127.0.0.1',5055))
            self.sock.setblocking(False)
        self.root=tk.Tk()
        self.root.title('Banco virtual · Pedal 21 cm + Nanotec LGA561 + C5-E en posición')
        self.root.geometry('1090x810')
        self.root.minsize(900,690)
        self.root.configure(bg=BG)
        self.request=tk.DoubleVar(value=0.)
        self.status=tk.StringVar(value='Simulación independiente: sin CARLA' if not udp else 'Esperando datos del banco automático o CARLA en 127.0.0.1:5055')
        self.latest=None
        self.history=deque(maxlen=280)
        self.last_tick=time.perf_counter()
        self.build()
        self.root.protocol('WM_DELETE_WINDOW',self.close)
        self.root.after(20,self.update)

    def build(self):
        tk.Label(self.root,text='PEDAL DE FRENO  |  NANOTEC + C5-E',bg=BG,fg=TEXT,font=('Segoe UI',19,'bold')).pack(anchor='w',padx=24,pady=(18,1))
        tk.Label(self.root,text='Modo Profile Position (CiA402 = 1) • 21 cm pedal • 11 cm desde pivote • varilla visible 70 mm en reposo',
                 bg=BG,fg=MUTED,font=('Segoe UI',10)).pack(anchor='w',padx=24,pady=(0,10))
        frame=tk.Frame(self.root,bg=PANEL)
        frame.pack(fill='x',padx=24,pady=(0,9))
        if not self.udp:
            tk.Label(frame,text='SOLICITUD AEB (%):',bg=PANEL,fg=TEXT,font=('Segoe UI',10,'bold')).pack(side='left',padx=12)
            tk.Scale(frame,from_=0,to=100,resolution=1,orient='horizontal',showvalue=True,variable=self.request,
                     bg=PANEL,fg=TEXT,activebackground=CYAN,troughcolor=GRID,highlightthickness=0,length=260).pack(side='left')
            for v in (0,25,50,75,100):
                tk.Button(frame,text=f'{v}%',bg=GRID,fg=TEXT,activebackground=CYAN,
                          font=('Segoe UI',10,'bold'),relief='flat',padx=8,command=lambda val=v:self.request.set(val)).pack(side='left',padx=3,pady=11)
            tk.Button(frame,text='Reiniciar',bg='#72502e',fg=TEXT,font=('Segoe UI',10,'bold'),
                      relief='flat',command=self.reset).pack(side='right',padx=12)
        else:
            tk.Label(frame,text='RECIBIENDO EN VIVO · Banco automático o CARLA (UDP local 5055)',bg=PANEL,fg=TEXT,font=('Segoe UI',10,'bold')).pack(padx=20,pady=16)
        self.canvas=tk.Canvas(self.root,height=420,bg=PANEL,highlightthickness=0)
        self.canvas.pack(fill='x',padx=24,pady=(3,10))
        metrics=tk.Frame(self.root,bg=BG)
        metrics.pack(fill='x',padx=24)
        self.metrics={}
        rows=[('Solicitud AEB','request_pct','%'),('Torque 6074','torque_demand_6074_permille','‰'),('Corriente equivalente','motor_current_a','A'),
              ('Varilla visible','exposed_rod_mm','mm'),('Avance desde reposo','rod_displacement_mm','mm'),('Pedal gira','pedal_angle_deg','°'),
              ('Fuerza de contacto','contact_force_n','N'),('Freno equivalente','brake_fraction','%')]
        for i,(label,key,unit) in enumerate(rows):
            tile=tk.Frame(metrics,bg=PANEL)
            tile.grid(row=i//4,column=i%4,sticky='nsew',padx=4,pady=4)
            metrics.columnconfigure(i%4,weight=1)
            tk.Label(tile,text=label,fg=MUTED,bg=PANEL,font=('Segoe UI',9)).pack(pady=(7,0))
            vv=tk.StringVar(value='—')
            tk.Label(tile,textvariable=vv,fg=GREEN if key=='brake_fraction' else TEXT,bg=PANEL,font=('Segoe UI',14,'bold')).pack(pady=(0,7))
            self.metrics[key]=(vv,unit)
        self.chart=tk.Canvas(self.root,height=145,bg=PANEL,highlightthickness=0)
        self.chart.pack(fill='both',expand=True,padx=24,pady=(8,4))
        tk.Label(self.root,textvariable=self.status,bg=BG,fg=MUTED,anchor='w',font=('Segoe UI',9)).pack(fill='x',padx=26,pady=(2,12))

    def reset(self):
        self.model.reset()
        self.history.clear()
        self.request.set(0)

    def update(self):
        if not self.udp:
            self.latest=asdict(self.model.step(self.request.get(),.02))
        else:
            try:
                while True:
                    data,_=self.sock.recvfrom(16000)
                    self.latest=json.loads(data.decode('utf-8'))
            except BlockingIOError:
                pass
            except (OSError,UnicodeError,ValueError):
                pass
        if self.latest:
            self.history.append(self.latest.copy())
            for key,(v,unit) in self.metrics.items():
                x=float(self.latest.get(key,0))
                if key=='brake_fraction':x*=100
                if key=='torque_demand_6074_permille':
                    v.set(f'{x:.0f} {unit}')
                else:v.set(f'{x:.1f} {unit}')
            st=self.latest
            tag=('CONTACTO' if st.get('contact',False) else 'SIN CONTACTO')
            limit=' | TOPE DE CARRERA' if st.get('at_forward_limit',False) else ''
            self.status.set(f'{"TELEMETRÍA / UDP" if self.udp else "BANCO MANUAL"} | t={st.get("time_s",0):.2f} s | {tag}{limit} | Modelo NO calibrado: ángulo, rigidez, torque, eficiencia y recorrido útil supuestos')
        self.draw_mechanism()
        self.draw_chart()
        self.root.after(20,self.update)

    def draw_mechanism(self):
        C=self.canvas;C.delete('all')
        w=C.winfo_width()
        if w<100:w=980
        zoom=max(.9,min(1.4,(w-120)/960))
        scale=1780*zoom
        gx=self.model.geometry
        # Geometría físicamente coherente (eje horizontal y0, barra girando)
        x0,y0=95*zoom,62
        theta=math.radians(float((self.latest or {}).get('pedal_angle_deg',0)))
        alpha=math.radians(gx.angulo_reposo_deg)+theta
        pa=(x0,y0)
        xend=x0+scale*gx.largo_pedal_m*math.cos(alpha)
        yend=y0+scale*gx.largo_pedal_m*math.sin(alpha)
        cy=y0+scale*gx.altura_contacto_m
        cx=x0+scale*(gx.altura_contacto_m/math.tan(alpha))
        tip0=x0+scale*(gx.x_contacto_reposo_m+gx.holgura_inicial_m)
        displacement=float((self.latest or {}).get('rod_displacement_mm',0))/1000
        tip_x=tip0-scale*displacement
        face_x=tip0+scale*gx.varilla_visible_reposo_m
        C.create_text(20,20,text='VISTA LATERAL · Contacto por empuje unilateral',font=('Segoe UI',12,'bold'),fill=TEXT,anchor='w')
        C.create_line(10,396,w-10,396,fill=GRID,width=2)
        # posición de descanso trazada
        rx=x0+scale*gx.largo_pedal_m*math.cos(math.radians(gx.angulo_reposo_deg))
        ry=y0+scale*gx.largo_pedal_m*math.sin(math.radians(gx.angulo_reposo_deg))
        C.create_line(x0,y0,rx,ry,fill=GRID,width=8,dash=(5,6))
        C.create_line(x0,y0,xend,yend,fill=CYAN,width=11,capstyle='round')
        # pedal pad, perpendicular
        nx,ny=-math.sin(alpha),math.cos(alpha)
        tx,ty=math.cos(alpha),math.sin(alpha)
        pad=[]
        for a,b in [(-27,-23),(27,-23),(27,19),(-27,19)]:
            pad.extend([xend+nx*a+tx*b,yend+ny*a+ty*b])
        C.create_polygon(*pad,fill='#7789a1',outline='#d2dbea',width=2)
        C.create_oval(x0-10,y0-10,x0+10,y0+10,fill=ORANGE,outline=TEXT,width=2)
        C.create_text(x0-12,y0-19,text='Pivote',fill=TEXT,font=('Segoe UI',9),anchor='e')
        # Contacto exacto se desliza a lo largo de la barra
        C.create_line(x0,y0,cx,cy,fill=ORANGE,width=3,dash=(5,3))
        C.create_oval(cx-7,cy-7,cx+7,cy+7,fill=ORANGE,outline=TEXT)
        C.create_text(cx-3,cy+18,text='contacto (11 cm inicial)',fill=ORANGE,font=('Segoe UI',9),anchor='e')
        # motor a la derecha, cuerpo sin desplazamiento
        C.create_rectangle(face_x,cy-36,face_x+140*zoom,cy+36,fill='#0a101b',outline='#657b95',width=3)
        C.create_rectangle(face_x+12,cy-27,face_x+115*zoom,cy+27,fill='#242f43',outline='#7c8ba2')
        C.create_text(face_x+63*zoom,cy,text='NANOTEC',fill=TEXT,font=('Segoe UI',10,'bold'))
        C.create_rectangle(face_x-7,cy-11,face_x+7,cy+11,fill='#93a5bb',outline='white')
        C.create_line(tip_x,cy,face_x,cy,fill='#c5d2e3',width=9)
        for k in range(8):
            xpos=tip_x+13+k*max(3,(face_x-tip_x-26)/8)
            if xpos<face_x-10:
                C.create_line(xpos,cy-4,xpos+6,cy+4,fill='#64758c',width=2)
        C.create_oval(tip_x-8,cy-8,tip_x+8,cy+8,fill=ORANGE,outline='white')
        C.create_line(tip_x,cy,cx,cy,fill=GREEN if (self.latest or {}).get('contact') else ORANGE,dash=(3,3),width=2)
        C.create_text(face_x-10,cy-51,text=f'Expuesta {float((self.latest or {}).get("exposed_rod_mm",70)):.1f} mm',fill=TEXT,font=('Segoe UI',10,'bold'),anchor='e')
        C.create_text(x0+340*zoom,357,text=f'Pedal {math.degrees(theta):.2f}° | Contacto F={float((self.latest or {}).get("contact_force_n",0)):.1f} N',
                      fill=GREEN,font=('Segoe UI',11,'bold'),anchor='w')
        C.create_text(22,375,text='El ángulo de reposo (58°), holgura y posición del motor son ilustrativos hasta medir montaje real.',fill=MUTED,font=('Segoe UI',9),anchor='w')

    def draw_chart(self):
        C=self.chart;C.delete('all'); w=max(750,C.winfo_width());h=max(140,C.winfo_height())
        x0,x1=48,w-24;y0,y1=22,h-25
        for pct in (0,25,50,75,100):
            y=y1-(y1-y0)*pct/100
            C.create_line(x0,y,x1,y,fill=GRID,dash=(2,5))
            C.create_text(x0-6,y,text=str(pct),anchor='e',fill=MUTED,font=('Segoe UI',8))
        data=list(self.history)
        for key,col in [('request_pct',CYAN),('brake_fraction',GREEN)]:
            pts=[]
            for i,d in enumerate(data):
                v=100*float(d.get(key,0)) if key=='brake_fraction' else float(d.get(key,0))
                pts.extend([x1-(len(data)-i-1)/279*(x1-x0),y1-(y1-y0)*clamp(v,0,100)/100])
            if len(pts)>=4:C.create_line(*pts,fill=col,width=2.5)
        C.create_text(x0+5,h-8,text='AEB solicitado',fill=CYAN,anchor='w',font=('Segoe UI',9,'bold'))
        C.create_text(x0+140,h-8,text='Freno equivalente',fill=GREEN,anchor='w',font=('Segoe UI',9,'bold'))

    def close(self):
        if self.sock:self.sock.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def clamp(x,a,b):return max(a,min(b,x))

if __name__=='__main__':
    pa=argparse.ArgumentParser()
    pa.add_argument('--manual',action='store_true',help='Simular dentro de esta ventana con botones')
    pa.add_argument('--udp',action='store_true',help='Escuchar datos externos (comportamiento predeterminado)')
    args=pa.parse_args()
    Monitor(udp=not args.manual).run()

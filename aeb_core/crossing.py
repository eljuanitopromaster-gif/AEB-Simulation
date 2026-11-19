"""Predicción de cruce con el radar: objetos al costado del carril que se mueven hacia él.

La percepción del carril (perception.RadarTarget) solo ve lo que ya está dentro del corredor.
Este módulo vigila una franja más ancha, a ambos lados:

1. Agrupa las detecciones (altura válida, dentro de la franja) en objetos.
2. Sigue cada objeto entre ciclos (asociación al más cercano + filtro α-β en x e y).
3. Velocidad lateral respecto del piso = velocidad lateral relativa + giro del ego × distancia.
4. Predice: si el objeto camina hacia el carril y entrará al corredor antes de que llegue el ego
   (y no habrá terminado de cruzar), se marca como amenaza.

Quien camina por la banqueta en paralelo (velocidad lateral ≈ 0) o se aleja del carril no se marca.
No adivina intenciones: alguien parado en la orilla no se marca hasta que empieza a caminar hacia el carril.
"""

import math

import numpy as np


class CrossingTrack:
    """Objeto seguido. Marco del radar: x adelante, y a la derecha [m]; velocidades relativas al ego [m/s]."""

    def __init__(self, track_id, x, y, x_near, closing, own, ego_speed):
        self.id = track_id
        self.x, self.y, self.x_near = x, y, x_near
        self.closing = closing                   # velocidad de cierre por Doppler (filtrada) [m/s]
        self.own = own                           # Doppler propio respecto del piso (filtrado) [m/s]
        self.vx, self.vy = -ego_speed, 0.0       # al nacer se supone quieto respecto del piso
        self.age = 1
        self.missed = 0
        self.vy_ground = 0.0                     # velocidad lateral respecto del piso [m/s]

    def predict(self, dt):
        return self.x + self.vx * dt, self.y + self.vy * dt


class CrossingPredictor:
    def __init__(self, params, lane_half_width, mount_z, min_range, min_height, max_height, mount_pitch=0.0):
        self.p = params
        self.lane_half_width = lane_half_width
        self.mount_z = mount_z
        self.min_range = min_range
        self.min_height, self.max_height = min_height, max_height
        self.mount_pitch = mount_pitch
        self.alpha = params["ab_alpha"]
        self.beta = self.alpha ** 2 / (2 - self.alpha)
        self.reset()

    def reset(self):
        self.tracks = []
        self._stamp = None
        self._next_id = 1
        self._hold = 0
        self.amenaza = None                      # CrossingTrack que se predice que entrará al carril
        self.t_entra = self.t_llega = math.nan   # de la amenaza (diagnóstico) [s]

    # ---------- entrada ----------
    def update(self, stamp, detections, ego_speed, ego_pitch=0.0, yaw_rate=0.0):
        """yaw_rate [rad/s], positivo girando a la derecha (mismo sentido que y)."""
        dt = stamp - self._stamp if self._stamp is not None else 0.0
        if self._stamp is not None and dt <= 0.0:
            return self
        self._stamp = stamp
        clusters = self._clusters(detections, ego_pitch, ego_speed)
        self._associate(clusters, dt, ego_speed)
        for tr in self.tracks:
            tr.vy_ground = tr.vy + yaw_rate * tr.x
        self._predict()
        return self

    # ---------- 1. agrupamiento ----------
    def _clusters(self, det, ego_pitch, ego_speed):
        """Lista de (x mediana, y del borde interior, x más cercana, cierre Doppler, Doppler propio)."""
        p = self.p
        depth, azi, alt = det.depth, det.azimuth, det.altitude
        x = depth * np.cos(alt) * np.cos(azi)
        y = depth * np.cos(alt) * np.sin(azi)
        h = self.mount_z + depth * np.sin(alt + self.mount_pitch + ego_pitch)
        m = ((depth > self.min_range) & (h > self.min_height) & (h < self.max_height)
             & (x > 0.5) & (x < p["alcance"]) & (np.abs(y) < p["ancho_vigilancia"]))
        # Doppler propio del objeto (respecto del piso, en la línea de vista): 0 si está quieto.
        # Solo se vigilan los puntos que se mueven: autos estacionados, postes y paredes quedan fuera
        # (su posición aparente "camina" con la perspectiva, pero su Doppler propio es 0).
        own = det.velocity + ego_speed * np.cos(azi) * np.cos(alt)
        m &= np.abs(own) >= p["doppler_min"]
        x, y, vel, own = x[m], y[m], det.velocity[m], own[m]
        n = len(x)
        if n == 0:
            return []
        if n > p["max_puntos"]:                  # los más cercanos bastan
            keep = np.argsort(x)[:p["max_puntos"]]
            x, y, vel, own, n = x[keep], y[keep], vel[keep], own[keep], p["max_puntos"]
        # componentes conexas: puntos a menos de eps_cluster se unen
        near = np.hypot(x[:, None] - x[None, :], y[:, None] - y[None, :]) < p["eps_cluster"]
        label = -np.ones(n, dtype=int)
        clusters = []
        for i in range(n):
            if label[i] >= 0:
                continue
            label[i] = len(clusters)
            stack, members = [i], []
            while stack:
                j = stack.pop()
                members.append(j)
                for k in np.nonzero(near[j] & (label < 0))[0]:
                    label[k] = label[i]
                    stack.append(k)
            if len(members) >= p["min_puntos"]:
                xs, ys = x[members], y[members]
                # Borde del lado del carril (no el centro): en un auto estacionado el radar ve cada vez más
                # de su costado al acercarse y el centro "camina" hacia el carril; el borde se queda quieto.
                y_in = math.copysign(float(np.percentile(np.abs(ys), p["percentil_borde"])), float(np.median(ys)))
                clusters.append((float(np.median(xs)), y_in, float(xs.min()),
                                 float(-np.median(vel[members])), float(np.median(own[members]))))
        return clusters

    # ---------- 2. seguimiento ----------
    def _associate(self, clusters, dt, ego_speed):
        p = self.p
        free = set(range(len(clusters)))
        if dt > 0.0 and self.tracks:
            preds = [tr.predict(dt) for tr in self.tracks]
            pairs = sorted((math.hypot(px - cl[0], py - cl[1]), ti, ci)
                           for ti, (px, py) in enumerate(preds)
                           for ci, cl in enumerate(clusters))
            used = set()
            for dist, ti, ci in pairs:
                if dist > p["gate"]:
                    break
                if ti in used or ci not in free:
                    continue
                used.add(ti)
                free.discard(ci)
                tr, (px, py), (cx, cy, cn, cc, co) = self.tracks[ti], preds[ti], clusters[ci]
                rx, ry = cx - px, cy - py
                tr.x, tr.y = px + self.alpha * rx, py + self.alpha * ry
                tr.vx += self.beta / dt * rx
                tr.vy += self.beta / dt * ry
                tr.x_near = cn
                tr.closing = p["ema_doppler"] * cc + (1 - p["ema_doppler"]) * tr.closing
                tr.own = p["ema_doppler"] * co + (1 - p["ema_doppler"]) * tr.own
                tr.age += 1
                tr.missed = 0
            for ti, tr in enumerate(self.tracks):
                if ti not in used:
                    tr.missed += 1
                    tr.x, tr.y = preds[ti]
                    tr.x_near += tr.vx * dt
        self.tracks = [tr for tr in self.tracks if tr.missed <= p["max_perdidos"]]
        for ci in free:
            cx, cy, cn, cc, co = clusters[ci]
            self.tracks.append(CrossingTrack(self._next_id, cx, cy, cn, cc, co, ego_speed))
            self._next_id += 1

    # ---------- 3. predicción ----------
    def _conflicto(self, tr):
        """(t_entra, t_llega) si se predice que estará en el carril cuando llegue el ego; si no, None."""
        p, hw = self.p, self.lane_half_width
        ay, vy = abs(tr.y), tr.vy_ground
        closing = tr.closing
        if (tr.age < p["edad_min"] or tr.missed > 0 or ay < hw or closing < 0.5
                or tr.y * vy >= 0.0 or abs(vy) < p["v_lat_min"] or abs(vy) > p["v_lat_max"]):
            return None
        t_llega = max(tr.x_near, 0.0) / closing
        t_entra = (ay - hw) / abs(vy)
        t_sale = (ay + hw) / abs(vy)
        if t_entra <= t_llega + p["margen_s"] and t_sale >= t_llega - p["margen_s"]:
            return t_entra, t_llega
        return None

    def _predict(self):
        best = None
        for tr in self.tracks:
            c = self._conflicto(tr)
            if c and (best is None or tr.x_near < best[0].x_near):
                best = (tr, *c)
        if best:
            self.amenaza, self.t_entra, self.t_llega = best
            self._hold = self.p["retencion"]
        elif self.amenaza is not None and self._hold > 0 and self.amenaza in self.tracks:
            self._hold -= 1                      # breve retención: evita que parpadee
        else:
            self.amenaza = None
            self.t_entra = self.t_llega = math.nan

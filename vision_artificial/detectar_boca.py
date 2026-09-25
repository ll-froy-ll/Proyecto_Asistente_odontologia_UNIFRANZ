"""
Deteccion de la cavidad bucal (BCN3D Moveo - asistente odontologico)
--------------------------------------------------------------------
- Detecta la boca con MediaPipe Face Mesh dentro de una zona fija.
- Suaviza el recuadro con media movil.
- Entrega la posicion NORMALIZADA (u, v) de la boca dentro de la zona
  (0..1), que es lo que el modulo de control usara para interpolar
  la pose del brazo.
- Registra eventos en consola y en archivo (carpeta logs/).

Se puede correr solo (python detectar_boca.py) o importar la clase
DetectorBoca desde control_motores.
"""

import cv2
import mediapipe as mp
import numpy as np
import logging
import time
from collections import deque
from datetime import datetime
from pathlib import Path

# Landmarks del contorno de la boca en MediaPipe Face Mesh
MOUTH_POINTS = [
    # Contorno exterior
    61, 185, 40, 39, 37, 0, 267, 269, 270, 409,
    291, 375, 321, 405, 314, 17, 84, 181, 91, 146,
    # Contorno interior 
    78, 191, 80, 81, 82, 13, 312, 311, 310, 415,
    308, 324, 318, 402, 317, 14, 87, 178, 88, 95,
]


def configurar_logging():
    """Logging a consola y a archivo logs/vision_AAAAMMDD.log"""
    carpeta = Path("logs")
    carpeta.mkdir(exist_ok=True)
    archivo = carpeta / f"vision_{datetime.now():%Y%m%d}.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s  %(levelname)s  %(message)s",
        handlers=[
            logging.FileHandler(archivo, encoding="utf-8"),
            logging.StreamHandler(),
        ],
    )
    return logging.getLogger("vision")


class Suavizador:
    """Media movil de las esquinas del recuadro de la boca."""

    def __init__(self, ventana=10):
        self.hist = {k: deque(maxlen=ventana) for k in ("x1", "y1", "x2", "y2")}

    def actualizar(self, x1, y1, x2, y2):
        vals = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
        for k, v in vals.items():
            self.hist[k].append(v)
        return tuple(int(np.mean(self.hist[k])) for k in ("x1", "y1", "x2", "y2"))


class DetectorBoca:
    """Detecta la cavidad bucal y entrega su posicion (u, v) dentro
    de una zona de deteccion fija."""

    def __init__(self, zona=(0.25, 0.30, 0.75, 0.85), margen=0.35,
                 ventana=10, log=None):
        # zona = (zx1, zy1, zx2, zy2) en fracciones del frame (0..1)
        self.zona = zona
        self.margen = margen
        self.suavizador = Suavizador(ventana)
        self.log = log or logging.getLogger("vision")

        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6,
        )
        self._estado_previo = False     # para loguear solo los cambios
        self._t_ultimo_log = 0.0        # throttle del log de posicion

    def procesar(self, frame):
        """Procesa un frame: dibuja sobre el y devuelve un dict
        { 'detectada': bool, 'u': float|None, 'v': float|None,
          'centro': (cx, cy)|None }"""
        h, w = frame.shape[:2]
        zx1 = int(w * self.zona[0])
        zy1 = int(h * self.zona[1])
        zx2 = int(w * self.zona[2])
        zy2 = int(h * self.zona[3])

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resultado = self.face_mesh.process(rgb)

        salida = {"detectada": False, "u": None, "v": None, "centro": None}

        if resultado.multi_face_landmarks:
            lm = resultado.multi_face_landmarks[0].landmark
            xs = [int(lm[i].x * w) for i in MOUTH_POINTS]
            ys = [int(lm[i].y * h) for i in MOUTH_POINTS]
            cx_raw = int(np.mean(xs))
            cy_raw = int(np.mean(ys))

            # Solo procesar si el centro de la boca esta dentro de la zona
            if zx1 < cx_raw < zx2 and zy1 < cy_raw < zy2:
                x1r, y1r = min(xs), min(ys)
                x2r, y2r = max(xs), max(ys)
                bw, bh = x2r - x1r, y2r - y1r

                x1m = max(0, int(x1r - bw * self.margen))
                y1m = max(0, int(y1r - bh * self.margen))
                x2m = min(w, int(x2r + bw * self.margen))
                y2m = min(h, int(y2r + bh * self.margen))

                x1, y1, x2, y2 = self.suavizador.actualizar(x1m, y1m, x2m, y2m)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                # (u, v) normalizado dentro de la zona, recortado a [0, 1]
                u = (cx - zx1) / max(1, (zx2 - zx1))
                v = (cy - zy1) / max(1, (zy2 - zy1))
                u = min(1.0, max(0.0, u))
                v = min(1.0, max(0.0, v))

                salida = {"detectada": True, "u": u, "v": v, "centro": (cx, cy)}

                # Dibujo del recuadro y los landmarks
                cv2.rectangle(frame, (x1, y1), (x2, y2), (200, 160, 0), 2)
                etiqueta = "Cavidad Bucal"
                (tw, th), _ = cv2.getTextSize(etiqueta, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
                cv2.rectangle(frame, (x1, y1 - th - 10), (x1 + tw + 10, y1), (200, 160, 0), -1)
                cv2.putText(frame, etiqueta, (x1 + 5, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
                for px, py in zip(xs, ys):
                    cv2.circle(frame, (px, py), 2, (0, 220, 255), -1)
                cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

        # Zona de deteccion (verde si detecta, gris si espera)
        color = (0, 220, 80) if salida["detectada"] else (200, 200, 200)
        cv2.rectangle(frame, (zx1, zy1), (zx2, zy2), color, 2)
        largo, grosor = 25, 3
        for (px, py, dx, dy) in [
            (zx1, zy1, 1, 1), (zx2, zy1, -1, 1),
            (zx1, zy2, 1, -1), (zx2, zy2, -1, -1),
        ]:
            cv2.line(frame, (px, py), (px + dx * largo, py), color, grosor)
            cv2.line(frame, (px, py), (px, py + dy * largo), color, grosor)
        cv2.putText(frame, "Zona de deteccion", (zx1 + 8, zy1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

        # Estado + valores u,v
        if salida["detectada"]:
            cv2.putText(frame, "Boca detectada", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 80), 2)
            cv2.putText(frame, f"u={salida['u']:.2f}  v={salida['v']:.2f}", (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 80), 2)
        else:
            cv2.putText(frame, "Buscando rostro...", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 80, 200), 2)

        # Log: solo cambios de estado + posicion cada 2 s
        if salida["detectada"] != self._estado_previo:
            if salida["detectada"]:
                self.log.info("Boca DETECTADA (u=%.2f, v=%.2f)", salida["u"], salida["v"])
            else:
                self.log.info("Deteccion PERDIDA")
            self._estado_previo = salida["detectada"]

        if salida["detectada"]:
            ahora = time.time()
            if ahora - self._t_ultimo_log > 2.0:
                self.log.info("Posicion boca: u=%.2f v=%.2f centro=%s",
                              salida["u"], salida["v"], salida["centro"])
                self._t_ultimo_log = ahora

        return salida

    def cerrar(self):
        self.face_mesh.close()


def main():
    log = configurar_logging()
    log.info("Iniciando deteccion de cavidad bucal")

    detector = DetectorBoca(log=log)

    # En Windows, CAP_DSHOW abre la camara mas rapido y estable
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    t_prev = time.time()
    fps = 0.0
    log.info("Presiona Q para salir.")

    while True:
        ok, frame = cap.read()
        if not ok:
            log.error("No se pudo leer la camara.")
            break

        frame = cv2.flip(frame, 1)
        salida = detector.procesar(frame)

        # FPS suavizado
        ahora = time.time()
        dt = ahora - t_prev
        t_prev = ahora
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        cv2.putText(frame, f"FPS: {fps:.0f}", (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

        cv2.imshow("Deteccion Cavidad Bucal", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    detector.cerrar()
    log.info("Deteccion terminada.")


if __name__ == "__main__":
    main()

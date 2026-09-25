"""
SEGUIMIENTO EN VIVO (Fase 3 + 4) - BCN3D Moveo
----------------------------------------------
Une la vision con el brazo:
  camara -> DetectorBoca -> (u,v) -> interpolacion bilineal de las
  4 esquinas (config/calibracion.json) -> PuenteESP32 -> brazo.

Comportamiento:
  - Al iniciar va de HOME a la zona de trabajo (CENTRO).
  - Sigue la boca con ZONA MUERTA (no tiembla) e histeresis.
  - Si pierde la cara, el brazo SE QUEDA QUIETO (hold).
  - Al salir (Q) vuelve a HOME.
  - El brazo se maneja en un hilo aparte para que la camara no se congele.

Fase 4 (datos de tesis):
  - Al cerrar muestra un RESUMEN de la sesion y lo guarda en
    logs/sesiones.csv (una fila por sesion).

Correr desde la RAIZ del proyecto (venv activo, monitor cerrado,
brazo en HOME):
    python seguir_boca.py
    python seguir_boca.py COM7
"""

import sys
import csv
import json
import time
import logging
import threading
from datetime import datetime
from pathlib import Path

import cv2

from vision_artificial.detectar_boca import DetectorBoca
from control_motores.puente_serial import PuenteESP32, listar_puertos

# ---- Parametros ajustables ----
DEADBAND = 0.06        # solo se mueve si la boca se corrio mas de 6% de la zona
FRAMES_CONFIRM = 3     # frames seguidos fuera de la zona muerta antes de mover
ARCHIVO_CALIB = Path("config/calibracion.json")
ARCHIVO_CSV = Path("logs/sesiones.csv")


def configurar_logging():
    carpeta = Path("logs")
    carpeta.mkdir(exist_ok=True)
    archivo = carpeta / f"sistema_{datetime.now():%Y%m%d}.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s  %(levelname)s  %(message)s",
        handlers=[
            logging.FileHandler(archivo, encoding="utf-8"),
            logging.StreamHandler(),
        ],
    )
    return logging.getLogger("sistema")


def interpolar(u, v, poses):
    """Interpolacion bilineal de las 4 esquinas -> pose de 5 pasos."""
    TL, TR, BL, BR = poses["TL"], poses["TR"], poses["BL"], poses["BR"]
    salida = []
    for i in range(5):
        arriba = TL[i] * (1 - u) + TR[i] * u
        abajo  = BL[i] * (1 - u) + BR[i] * u
        salida.append(round(arriba * (1 - v) + abajo * v))
    return salida


def guardar_resumen(log, t_inicio, m):
    """Imprime el resumen de la sesion y lo agrega a logs/sesiones.csv"""
    dur = max(0.001, time.time() - t_inicio)
    fps_prom = m["frames"] / dur
    pct_det = 100.0 * (dur - m["ciego"]) / dur
    mm, ss = divmod(int(dur), 60)

    print("\n========= RESUMEN DE SESION =========")
    print(f"  Duracion:               {mm} min {ss} s")
    print(f"  Correcciones del brazo: {m['correcciones']}")
    print(f"  Perdidas de deteccion:  {m['perdidas']}")
    print(f"  Tiempo sin detectar:    {m['ciego']:.1f} s  ({100 - pct_det:.0f}% del tiempo)")
    print(f"  Tiempo detectada:       {pct_det:.0f}%")
    print(f"  FPS promedio:           {fps_prom:.0f}")
    print(f"  Guardado en {ARCHIVO_CSV}")
    print("=====================================\n")

    log.info("RESUMEN sesion: dur=%.0fs correcciones=%d perdidas=%d ciego=%.1fs det=%.0f%% fps=%.0f",
             dur, m["correcciones"], m["perdidas"], m["ciego"], pct_det, fps_prom)

    ARCHIVO_CSV.parent.mkdir(exist_ok=True)
    nuevo = not ARCHIVO_CSV.exists()
    with ARCHIVO_CSV.open("a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if nuevo:
            w.writerow(["fecha_hora", "duracion_s", "correcciones",
                        "perdidas_deteccion", "tiempo_ciego_s",
                        "pct_detectada", "fps_promedio"])
        w.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    round(dur, 1), m["correcciones"], m["perdidas"],
                    round(m["ciego"], 1), round(pct_det), round(fps_prom)])


class SeguidorBrazo:
    """Hilo que manda al brazo la ULTIMA pose objetivo (sin acumular)."""

    def __init__(self, esp, log):
        self.esp = esp
        self.log = log
        self._objetivo = None
        self._enviado = None
        self._lock = threading.Lock()
        self._corriendo = False
        self._hilo = None

    def set_objetivo(self, pose):
        with self._lock:
            self._objetivo = list(pose)

    def _loop(self):
        while self._corriendo:
            with self._lock:
                obj = self._objetivo
            if obj is not None and obj != self._enviado:
                if self.esp.mover(*obj):
                    self._enviado = obj
            else:
                time.sleep(0.02)

    def iniciar(self):
        self._corriendo = True
        self._hilo = threading.Thread(target=self._loop, daemon=True)
        self._hilo.start()

    def detener(self):
        self._corriendo = False
        if self._hilo:
            self._hilo.join(timeout=5)


def main():
    log = configurar_logging()
    puerto = sys.argv[1] if len(sys.argv) > 1 else "COM11"

    if not ARCHIVO_CALIB.exists():
        print(f"No encuentro {ARCHIVO_CALIB}. Calibra primero (Fase 2).")
        return
    datos = json.loads(ARCHIVO_CALIB.read_text(encoding="utf-8"))
    zona = datos["zona"]
    poses = datos["poses"]
    centro = poses["CENTRO"]
    log.info("Calibracion cargada. CENTRO=%s", centro)

    print("Puertos disponibles:")
    for p in listar_puertos():
        print("  ", p)

    esp = PuenteESP32(puerto, log=log)
    if not esp.conectar():
        print("No se pudo conectar (cerra el monitor de ESP-IDF).")
        return

    # Metricas de sesion (definidas antes del try para el resumen final)
    m = {"frames": 0, "correcciones": 0, "perdidas": 0, "ciego": 0.0}
    t_inicio = time.time()

    detector = None
    seg = None
    cap = None
    try:
        print("Yendo a la zona de trabajo...")
        esp.mover(*centro)
        log.info("Brazo en zona de trabajo (CENTRO).")

        detector = DetectorBoca(
            zona=(zona["zx1"], zona["zy1"], zona["zx2"], zona["zy2"]),
            log=log,
        )
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        seg = SeguidorBrazo(esp, log)
        seg.iniciar()

        uv_comandado = [0.5, 0.5]
        confirm = 0

        # arranca la medicion del seguimiento
        t_inicio = time.time()
        t_prev = t_inicio
        detectada_previo = False
        fps = 0.0

        print("Siguiendo. Presiona Q en la ventana para salir.")

        while True:
            ok, frame = cap.read()
            if not ok:
                log.error("No se pudo leer la camara.")
                break

            ahora = time.time()
            dt = ahora - t_prev
            t_prev = ahora
            m["frames"] += 1
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            frame = cv2.flip(frame, 1)
            salida = detector.procesar(frame)

            if salida["detectada"]:
                u, v = salida["u"], salida["v"]
                fuera = (abs(u - uv_comandado[0]) > DEADBAND or
                         abs(v - uv_comandado[1]) > DEADBAND)
                if fuera:
                    confirm += 1
                    if confirm >= FRAMES_CONFIRM:
                        objetivo = interpolar(u, v, poses)
                        seg.set_objetivo(objetivo)
                        uv_comandado = [u, v]
                        confirm = 0
                        m["correcciones"] += 1
                        estado, color = "SIGUIENDO (mov)", (0, 200, 80)
                    else:
                        estado, color = "SIGUIENDO", (0, 200, 80)
                else:
                    confirm = 0
                    estado, color = "EN ZONA", (0, 200, 80)
                detectada_previo = True
            else:
                m["ciego"] += dt
                if detectada_previo:
                    m["perdidas"] += 1
                detectada_previo = False
                confirm = 0
                estado, color = "SIN DETECCION - QUIETO", (0, 140, 255)

            cv2.putText(frame, estado, (20, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(frame, f"FPS:{fps:.0f}  corr:{m['correcciones']}", (20, 185),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)

            cv2.imshow("Asistente Odontologico - Seguimiento", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        if seg is not None:
            seg.detener()
        print("Volviendo a HOME...")
        try:
            esp.mover(0, 0, 0, 0, 0)
        except Exception:
            pass
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        if detector is not None:
            detector.cerrar()
        esp.cerrar()
        try:
            guardar_resumen(log, t_inicio, m)
        except Exception as e:
            print("No pude guardar el resumen:", e)
        print("Listo.")


if __name__ == "__main__":
    main()

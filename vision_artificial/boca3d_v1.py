"""
BOCA EN 3D  v1
--------------
Entrega la posicion de la boca en coordenadas del BRAZO (mm), combinando:
  - MediaPipe Face Mesh: donde esta la boca en la imagen
  - El IRIS como regla: su diametro real es casi constante (~11.7 mm), asi que
    su tamano en pixeles da la distancia de la cara a la camara
  - El ArUco: donde esta la camara respecto del brazo

Necesita:
    config/camara.json       (Fase 2a)
    config/cinematica.json   (marcador)
    vision_artificial/detectar_boca.py   (de ahi salen los puntos de la boca)

Uso (desde la RAIZ del proyecto, con Iriun transmitiendo):
    python vision_artificial/boca3d_v1.py
    python vision_artificial/boca3d_v1.py 2      -> forzar un indice de camara

En la ventana:
    g        imprime y guarda una muestra en logs/boca3d.csv
    q o ESC  salir

NOTA: la imagen NO se espeja. Si se espejara, las coordenadas saldrian
invertidas en un eje.
"""

import sys
import csv
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import mediapipe as mp

sys.path.insert(0, str(Path(__file__).resolve().parent))
from camara_util_v1 import abrir_camara                               # noqa: E402
from autocalibrar_v1 import (cargar_configs, pose_marcador_en_mundo,   # noqa: E402
                             esquinas_objeto, crear_detector, pose_camara)
from detectar_boca import MOUTH_POINTS                                 # noqa: E402

# ================== CONSTANTES AJUSTABLES ==================
IRIS_MM = 11.7              # diametro real del iris humano (casi constante)
OFFSET_BOCA_MM = 0.0        # cuanto mas lejos esta la boca que los ojos (se ajusta al validar)
VENTANA_SUAVIZADO = 8       # cuadros promediados de la posicion de la boca
MUESTRAS_CAMARA = 15        # cuadros promediados de la pose de la camara
SALTO_AVISO_MM = 50.0       # aviso si la boca salta mas que esto entre cuadros
ARCHIVO_CSV = Path("logs/boca3d.csv")

# Landmarks del iris en Face Mesh con refine_landmarks=True
IRIS_IZQ = [469, 470, 471, 472]
IRIS_DER = [474, 475, 476, 477]
# ===========================================================


def centro_y_diametro(lm, indices, w, h):
    """Centro (px) y diametro (px) de un iris a partir de sus 4 puntos."""
    pts = np.array([[lm[i].x * w, lm[i].y * h] for i in indices])
    centro = pts.mean(axis=0)
    d1 = np.linalg.norm(pts[0] - pts[2])
    d2 = np.linalg.norm(pts[1] - pts[3])
    return centro, (d1 + d2) / 2.0


def pixel_a_rayo(pixel, K, dist):
    """Pixel -> direccion normalizada en el sistema de la camara (z = 1)."""
    p = np.array([[[float(pixel[0]), float(pixel[1])]]], dtype=np.float64)
    n = cv2.undistortPoints(p, K, dist).reshape(2)
    return np.array([n[0], n[1], 1.0])


def main():
    forzado = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else None
    K, dist, cam_cfg, mk, _ = cargar_configs()
    fx = K[0, 0]
    R_wm, t_wm = pose_marcador_en_mundo(mk)
    obj = esquinas_objeto(float(mk["lado_mm"]))
    detectar_aruco = crear_detector(mk.get("diccionario", "DICT_4X4_50"))
    id_marcador = int(mk.get("id", 0))

    cap, indice, nombre = abrir_camara(forzado)
    if cap is None:
        return
    print(__doc__)

    face = mp.solutions.face_mesh.FaceMesh(
        max_num_faces=1, refine_landmarks=True,
        min_detection_confidence=0.6, min_tracking_confidence=0.6)

    centros_cam, rots = [], []
    hist_boca = []
    ultimo = None
    t_prev, fps = time.time(), 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Se corto la imagen.")
            break
        h, w = frame.shape[:2]
        vista = frame.copy()

        # ---- 1) pose de la camara con el ArUco ----
        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        esq, ids = detectar_aruco(gris)
        if ids is not None:
            for c, i in zip(esq, ids.flatten()):
                if int(i) != id_marcador:
                    continue
                r = pose_camara(c.reshape(4, 2).astype(np.float32), obj, K, dist, R_wm, t_wm)
                if r is not None:
                    centros_cam.append(r[0])
                    rots.append(r[1])
                    if len(centros_cam) > MUESTRAS_CAMARA:
                        centros_cam.pop(0), rots.pop(0)
        hay_camara = bool(centros_cam)

        # ---- 2) cara: boca e iris ----
        res = face.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        boca_mundo = None
        info = []
        if res.multi_face_landmarks and hay_camara:
            lm = res.multi_face_landmarks[0].landmark
            xs = [lm[i].x * w for i in MOUTH_POINTS]
            ys = [lm[i].y * h for i in MOUTH_POINTS]
            boca_px = (float(np.mean(xs)), float(np.mean(ys)))

            c_izq, d_izq = centro_y_diametro(lm, IRIS_IZQ, w, h)
            c_der, d_der = centro_y_diametro(lm, IRIS_DER, w, h)
            d_iris = (d_izq + d_der) / 2.0

            if d_iris > 1.0:
                z_ojos = fx * IRIS_MM / d_iris          # distancia camara-cara (mm)
                z_boca = z_ojos + OFFSET_BOCA_MM
                rayo = pixel_a_rayo(boca_px, K, dist)
                p_cam = rayo * z_boca                    # punto en el sistema de la camara
                centro_cam = np.mean(centros_cam, axis=0)
                R_cw = rots[-1]
                p_mundo = centro_cam + R_cw.T @ p_cam

                hist_boca.append(p_mundo)
                if len(hist_boca) > VENTANA_SUAVIZADO:
                    hist_boca.pop(0)
                suave = np.mean(hist_boca, axis=0)
                salto = np.linalg.norm(p_mundo - ultimo) if ultimo is not None else 0.0
                ultimo = p_mundo
                boca_mundo = suave

                info = [
                    f"BOCA  x={suave[0]:7.1f}  y={suave[1]:7.1f}  z={suave[2]:7.1f} mm",
                    f"distancia camara-cara: {z_ojos:.0f} mm   iris {d_iris:.1f} px",
                ]
                if salto > SALTO_AVISO_MM:
                    info.append(f"! salto de {salto:.0f} mm entre cuadros")

                for px, py in zip(xs, ys):
                    cv2.circle(vista, (int(px), int(py)), 2, (0, 220, 255), -1)
                cv2.circle(vista, (int(boca_px[0]), int(boca_px[1])), 5, (0, 255, 0), -1)
                for c in (c_izq, c_der):
                    cv2.circle(vista, (int(c[0]), int(c[1])), int(d_iris / 2), (255, 0, 255), 2)
        elif not hay_camara:
            info = ["no se ve el marcador ArUco"]
        else:
            info = ["no se ve la cara"]
            hist_boca.clear()
            ultimo = None

        ahora = time.time()
        dt = ahora - t_prev
        t_prev = ahora
        if dt > 0:
            fps = 0.9 * fps + 0.1 / dt
        color = (0, 255, 0) if boca_mundo is not None else (0, 140, 255)
        for k, t in enumerate(info):
            cv2.putText(vista, t, (10, 30 + 28 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(vista, f"FPS {fps:.0f}   g=guardar muestra   q=salir",
                    (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        cv2.imshow("Boca en 3D", vista)

        t = cv2.waitKey(1) & 0xFF
        if t in (27, ord("q")):
            break
        if t == ord("g"):
            if boca_mundo is None:
                print("  Todavia no tengo la boca y la camara a la vez.")
            else:
                print(f"  BOCA  x={boca_mundo[0]:.1f}  y={boca_mundo[1]:.1f}  z={boca_mundo[2]:.1f} mm")
                ARCHIVO_CSV.parent.mkdir(exist_ok=True)
                nuevo = not ARCHIVO_CSV.exists()
                with ARCHIVO_CSV.open("a", newline="", encoding="utf-8") as f:
                    wcsv = csv.writer(f)
                    if nuevo:
                        wcsv.writerow(["fecha_hora", "x_mm", "y_mm", "z_mm"])
                    wcsv.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                                   round(float(boca_mundo[0]), 1),
                                   round(float(boca_mundo[1]), 1),
                                   round(float(boca_mundo[2]), 1)])
                print(f"  Guardado en {ARCHIVO_CSV}")

    cap.release()
    cv2.destroyAllWindows()
    face.close()


if __name__ == "__main__":
    main()

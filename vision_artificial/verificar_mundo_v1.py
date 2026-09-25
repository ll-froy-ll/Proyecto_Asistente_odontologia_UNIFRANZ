"""
VERIFICAR MUNDO  v1
-------------------
Comprueba toda la cadena camara -> brazo: haces clic en un punto de la imagen
y te dice sus coordenadas en el mundo del brazo (mm).

Como funciona: el ArUco da la posicion de la camara en cada cuadro; el clic
define un rayo que sale del lente y se corta con un plano horizontal de altura
conocida (por defecto z = 0, la superficie de la tabla).

Necesita:
    config/camara.json       (Fase 2a)
    config/cinematica.json   (marcador)

Uso (desde la RAIZ del proyecto, con Iriun transmitiendo):
    python vision_artificial/verificar_mundo_v1.py
    python vision_artificial/verificar_mundo_v1.py -40      -> plano a z = -40 mm
    python vision_artificial/verificar_mundo_v1.py -40 2    -> ademas fuerza la camara 2

En la ventana:
    clic izquierdo   calcula las coordenadas de ese punto
    + / -            sube o baja el plano 10 mm
    m                marca el centro del marcador (prueba rapida)
    q o ESC          salir
"""

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from camara_util_v1 import abrir_camara                              # noqa: E402
from autocalibrar_v1 import (cargar_configs, pose_marcador_en_mundo,  # noqa: E402
                             esquinas_objeto, crear_detector, pose_camara)

# ================== CONSTANTES AJUSTABLES ==================
Z_PLANO_MM = 0.0          # altura del plano donde se proyecta el clic
PASO_PLANO_MM = 10.0      # cuanto sube/baja con + y -
MUESTRAS = 15             # cuadros promediados de la pose de la camara
MAX_MARCAS = 6            # clics que se muestran a la vez
# ===========================================================

marcas = []               # [(pixel, punto_mundo)]
clic_nuevo = [None]


def al_hacer_clic(evento, x, y, flags, param):
    if evento == cv2.EVENT_LBUTTONDOWN:
        clic_nuevo[0] = (x, y)


def rayo_a_plano(pixel, K, dist, centro, R_cw, z_plano):
    """Convierte un pixel en un punto del plano horizontal z = z_plano."""
    p = np.array([[[float(pixel[0]), float(pixel[1])]]], dtype=np.float64)
    n = cv2.undistortPoints(p, K, dist).reshape(2)          # coordenadas normalizadas
    d_cam = np.array([n[0], n[1], 1.0])
    d_mundo = R_cw.T @ d_cam
    if abs(d_mundo[2]) < 1e-9:
        return None
    t = (z_plano - centro[2]) / d_mundo[2]
    if t <= 0:
        return None
    return centro + t * d_mundo


def main():
    args = [a for a in sys.argv[1:]]
    z_plano = Z_PLANO_MM
    forzado = None
    if args:
        try:
            z_plano = float(args[0])
        except ValueError:
            pass
    if len(args) > 1 and args[1].lstrip("-").isdigit():
        forzado = int(args[1])

    K, dist, cam_cfg, mk, cam_medida = cargar_configs()
    R_wm, t_wm = pose_marcador_en_mundo(mk)
    obj = esquinas_objeto(float(mk["lado_mm"]))
    detectar = crear_detector(mk.get("diccionario", "DICT_4X4_50"))
    id_esperado = int(mk.get("id", 0))

    cap, indice, nombre = abrir_camara(forzado)
    if cap is None:
        return
    print(__doc__)

    cv2.namedWindow("Verificar mundo")
    cv2.setMouseCallback("Verificar mundo", al_hacer_clic)

    centros, rots = [], []
    pose = None
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Se corto la imagen.")
            break
        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        esq, ids = detectar(gris)
        vista = frame.copy()
        centro_marcador_px = None

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vista, esq, ids)
            for c, i in zip(esq, ids.flatten()):
                if int(i) != id_esperado:
                    continue
                pts = c.reshape(4, 2).astype(np.float32)
                centro_marcador_px = tuple(np.mean(pts, axis=0).astype(int))
                r = pose_camara(pts, obj, K, dist, R_wm, t_wm)
                if r is None:
                    continue
                centro, R_cw, err = r
                centros.append(centro)
                rots.append(R_cw)
                if len(centros) > MUESTRAS:
                    centros.pop(0), rots.pop(0)
                pose = (np.mean(centros, axis=0), rots[-1], err)

        if pose is not None and clic_nuevo[0] is not None:
            centro, R_cw, _ = pose
            punto = rayo_a_plano(clic_nuevo[0], K, dist, centro, R_cw, z_plano)
            if punto is not None:
                marcas.append((clic_nuevo[0], punto))
                if len(marcas) > MAX_MARCAS:
                    marcas.pop(0)
                print(f"  pixel {clic_nuevo[0]}  ->  x={punto[0]:7.1f}  y={punto[1]:7.1f}"
                      f"  z={punto[2]:7.1f} mm   (plano z={z_plano:.0f})")
            else:
                print("  Ese punto no corta el plano (la camara mira casi paralelo).")
        clic_nuevo[0] = None

        for px, pt in marcas:
            cv2.drawMarker(vista, px, (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(vista, f"({pt[0]:.0f}, {pt[1]:.0f})", (px[0] + 10, px[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        if pose is not None:
            c, _, err = pose
            cv2.putText(vista, f"camara ({c[0]:.0f}, {c[1]:.0f}, {c[2]:.0f}) mm   repro {err:.2f} px",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(vista, "no se ve el marcador", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(vista, f"plano z = {z_plano:.0f} mm   (+/- para cambiar)", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(vista, "clic = medir   m = centro del marcador   q = salir",
                    (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        cv2.imshow("Verificar mundo", vista)

        t = cv2.waitKey(1) & 0xFF
        if t in (27, ord("q")):
            break
        if t in (ord("+"), ord("=")):
            z_plano += PASO_PLANO_MM
        if t in (ord("-"), ord("_")):
            z_plano -= PASO_PLANO_MM
        if t == ord("m") and centro_marcador_px is not None:
            clic_nuevo[0] = (int(centro_marcador_px[0]), int(centro_marcador_px[1]))
            print(f"  (prueba) centro del marcador: deberia dar x={mk['x_mm']:.0f} "
                  f"y={mk['y_mm']:.0f} con el plano en z={mk['z_mm']:.0f}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

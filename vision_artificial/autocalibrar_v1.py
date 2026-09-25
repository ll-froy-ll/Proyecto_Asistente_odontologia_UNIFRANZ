"""
AUTOCALIBRAR CAMARA (ArUco)  v1
-------------------------------
Calcula DONDE ESTA LA CAMARA respecto del brazo usando el marcador pegado en la
tabla, y lo compara con las medidas que cargaste a mano en config/cinematica.json.

Necesita:
    config/camara.json       (calibracion del lente, Fase 2a)
    config/cinematica.json   (posicion del marcador y de la camara medida a mano)

Uso (desde la RAIZ del proyecto, con Iriun transmitiendo):
    python vision_artificial/autocalibrar_v1.py
    python vision_artificial/autocalibrar_v1.py 0     -> forzar un indice

En la ventana:
    g        guardar el resultado en config/camara_pose.json
    q o ESC  salir sin guardar

El sistema de coordenadas es el del brazo: origen en el eje de la base,
+x flecha de la tabla, +y izquierda mirando la flecha, +z arriba.
"""

import sys
import json
import math
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from camara_util_v1 import abrir_camara     # noqa: E402

# ================== CONSTANTES AJUSTABLES ==================
ARCHIVO_CAMARA = Path("config/camara.json")
ARCHIVO_CINEMATICA = Path("config/cinematica.json")
ARCHIVO_SALIDA = Path("config/camara_pose.json")

MUESTRAS = 30                # cuadros que se promedian
DIF_AVISO_MM = 60.0          # diferencia con la medida a mano que hace saltar un aviso
DIF_AVISO_DEG = 10.0
ERROR_REPRO_AVISO_PX = 2.0
# ===========================================================


def rotz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])


def rotx(deg):
    a = math.radians(deg)
    return np.array([[1.0, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def cargar_configs():
    cam = json.loads(ARCHIVO_CAMARA.read_text(encoding="utf-8"))
    K = np.array([[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1.0]])
    dist = np.array(cam["distorsion"], dtype=float)
    cin = json.loads(ARCHIVO_CINEMATICA.read_text(encoding="utf-8"))
    return K, dist, cam, cin["marcador"], cin.get("camara", {})


def pose_marcador_en_mundo(mk):
    """R y t que llevan del sistema del marcador al del brazo."""
    R = rotz(mk.get("giro_deg") or 0.0) @ rotx(mk.get("inclinacion_deg") or 0.0)
    t = np.array([mk["x_mm"], mk["y_mm"], mk["z_mm"]], dtype=float)
    return R, t


def esquinas_objeto(lado):
    h = lado / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def crear_detector(nombre_dic):
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, nombre_dic))
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)[:2]
    par = cv2.aruco.DetectorParameters_create()
    return lambda img: cv2.aruco.detectMarkers(img, dic, parameters=par)[:2]


def pose_camara(esquinas_img, obj, K, dist, R_wm, t_wm):
    """Devuelve (centro de la camara en mundo, R mundo->camara, error de reproyeccion px)."""
    ok, rvec, tvec = cv2.solvePnP(obj, esquinas_img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R_cm, _ = cv2.Rodrigues(rvec)                 # marcador -> camara
    R_cw = R_cm @ R_wm.T                          # mundo -> camara
    centro = t_wm - R_cw.T @ tvec.ravel()         # camara en coordenadas del brazo
    proy, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = float(np.mean(np.linalg.norm(proy.reshape(-1, 2) - esquinas_img.reshape(-1, 2), axis=1)))
    return centro, R_cw, err


def angulos_camara(R_cw):
    """Inclinacion (negativa mirando abajo) y giro visto desde arriba, en grados."""
    d = R_cw.T @ np.array([0, 0, 1.0])            # hacia donde mira el eje optico, en mundo
    inclinacion = math.degrees(math.asin(np.clip(d[2], -1, 1)))
    horiz = math.hypot(d[0], d[1])
    giro = math.degrees(math.atan2(d[1], d[0])) if horiz > 1e-6 else 0.0
    return inclinacion, giro, d


def main():
    for f in (ARCHIVO_CAMARA, ARCHIVO_CINEMATICA):
        if not f.exists():
            print(f"Falta {f}. Corre antes la calibracion de la Fase 2a.")
            return
    K, dist, cam_cfg, mk, cam_medida = cargar_configs()
    R_wm, t_wm = pose_marcador_en_mundo(mk)
    obj = esquinas_objeto(float(mk["lado_mm"]))
    detectar = crear_detector(mk.get("diccionario", "DICT_4X4_50"))
    id_esperado = int(mk.get("id", 0))

    forzado = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else None
    cap, indice, nombre = abrir_camara(forzado)
    if cap is None:
        return
    print(__doc__)
    print(f"  Marcador ID {id_esperado}, lado {mk['lado_mm']} mm, centro "
          f"({mk['x_mm']}, {mk['y_mm']}, {mk['z_mm']}) mm, giro {mk['giro_deg']} grados\n")

    centros, ds, errores = [], [], []
    ultimo = None
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Se corto la imagen.")
            break
        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        esq, ids = detectar(gris)
        vista = frame.copy()
        visto = False
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vista, esq, ids)
            for c, i in zip(esq, ids.flatten()):
                if int(i) != id_esperado:
                    continue
                r = pose_camara(c.reshape(4, 2).astype(np.float32), obj, K, dist, R_wm, t_wm)
                if r is None:
                    continue
                centro, R_cw, err = r
                inc, giro, d = angulos_camara(R_cw)
                centros.append(centro)
                ds.append(d)
                errores.append(err)
                if len(centros) > MUESTRAS:
                    centros.pop(0), ds.pop(0), errores.pop(0)
                visto = True
                ultimo = (np.mean(centros, axis=0), np.mean(ds, axis=0), float(np.mean(errores)),
                          np.std(centros, axis=0))

        if ultimo is not None:
            c_med, d_med, err_med, disp = ultimo
            inc = math.degrees(math.asin(np.clip(d_med[2] / np.linalg.norm(d_med), -1, 1)))
            giro = math.degrees(math.atan2(d_med[1], d_med[0]))
            txt = [
                f"camara: x={c_med[0]:7.1f}  y={c_med[1]:7.1f}  z={c_med[2]:7.1f} mm",
                f"inclinacion {inc:6.1f}   giro {giro:6.1f} grados",
                f"estabilidad +-{np.max(disp):.1f} mm   reproyeccion {err_med:.2f} px",
            ]
            for k, t in enumerate(txt):
                cv2.putText(vista, t, (10, 30 + 28 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0) if visto else (0, 165, 255), 2)
        else:
            cv2.putText(vista, "no se ve el marcador", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(vista, "g=guardar  q=salir", (10, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        cv2.imshow("Autocalibracion ArUco", vista)

        t = cv2.waitKey(1) & 0xFF
        if t in (27, ord("q")):
            break
        if t == ord("g"):
            if ultimo is None:
                print("  Todavia no vi el marcador.")
                continue
            c_med, d_med, err_med, disp = ultimo
            inc = math.degrees(math.asin(np.clip(d_med[2] / np.linalg.norm(d_med), -1, 1)))
            giro = math.degrees(math.atan2(d_med[1], d_med[0]))
            print("\n===== POSICION CALCULADA DE LA CAMARA =====")
            print(f"  x = {c_med[0]:.1f} mm   y = {c_med[1]:.1f} mm   z = {c_med[2]:.1f} mm")
            print(f"  inclinacion = {inc:.1f} grados   giro = {giro:.1f} grados")
            print(f"  dispersion entre cuadros: +-{np.max(disp):.1f} mm")
            print(f"  error de reproyeccion: {err_med:.2f} px")
            if cam_medida.get("x_mm") is not None:
                m = np.array([cam_medida["x_mm"], cam_medida["y_mm"], cam_medida["z_mm"]])
                dif = c_med - m
                print("\n  Comparacion con lo que mediste a mano:")
                print(f"    medido:   x={m[0]:.0f}  y={m[1]:.0f}  z={m[2]:.0f} mm")
                print(f"    calculado x={c_med[0]:.0f}  y={c_med[1]:.0f}  z={c_med[2]:.0f} mm")
                print(f"    diferencia: {np.round(dif, 0)} mm  (|d| = {np.linalg.norm(dif):.0f} mm)")
                if np.max(np.abs(dif)) > DIF_AVISO_MM:
                    print("    ! Diferencia grande: revisa el giro del marcador o su posicion.")
                di = inc - (cam_medida.get("inclinacion_deg") or 0.0)
                if abs(di) > DIF_AVISO_DEG:
                    print(f"    ! La inclinacion difiere {di:.0f} grados de la medida.")
            if err_med > ERROR_REPRO_AVISO_PX:
                print("  ! Reproyeccion alta: el marcador puede no estar plano o mal medido.")
            ARCHIVO_SALIDA.parent.mkdir(parents=True, exist_ok=True)
            ARCHIVO_SALIDA.write_text(json.dumps({
                "version": 1,
                "x_mm": float(c_med[0]), "y_mm": float(c_med[1]), "z_mm": float(c_med[2]),
                "eje_optico": [float(v) for v in (d_med / np.linalg.norm(d_med))],
                "inclinacion_deg": float(inc), "giro_deg": float(giro),
                "dispersion_mm": float(np.max(disp)),
                "reproyeccion_px": float(err_med),
                "muestras": len(centros),
            }, indent=2), encoding="utf-8")
            print(f"\n  Guardado en {ARCHIVO_SALIDA.resolve()}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

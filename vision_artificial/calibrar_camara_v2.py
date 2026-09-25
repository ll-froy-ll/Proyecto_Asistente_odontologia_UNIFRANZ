"""
CALIBRAR CAMARA  v2
-------------------
Mide la optica del celular (distancia focal, centro optico y distorsion) con el
tablero de ajedrez impreso. El resultado se guarda en config/camara.json y lo
usa la autocalibracion con ArUco y la estimacion 3D de la boca.

Elige la camara por NOMBRE (Iriun), asi no importa que cambie el indice.

Uso (desde la RAIZ del proyecto, con Iriun transmitiendo):
    python vision_artificial/calibrar_camara_v2.py
    python vision_artificial/calibrar_camara_v2.py 0     -> forzar un indice

En la ventana:
    c o ESPACIO  capturar la vista actual (solo si detecta el tablero)
    d            descartar la ultima captura
    k            calibrar con lo capturado y guardar
    q o ESC      salir sin guardar

IMPORTANTE: la resolucion tiene que ser la misma que despues use el sistema.
Si cambias la resolucion en Iriun, hay que recalibrar.
"""

import sys
import json
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from camara_util_v1 import abrir_camara     # noqa: E402

# ================== CONSTANTES AJUSTABLES ==================
ARCHIVO_SALIDA = Path("config/camara.json")

PATRON = (9, 6)                 # esquinas INTERNAS del tablero (10x7 cuadros)
LADO_CUADRO_MM = 25.0
CAPTURAS_MINIMAS = 12
CAPTURAS_RECOMENDADAS = 18
SEPARACION_MINIMA_PX = 60       # las capturas deben diferir entre si
RMS_ACEPTABLE_PX = 1.0          # error de reproyeccion admisible
# ===========================================================

CRITERIO = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

POSES = [
    "1. Tablero de frente, llenando el centro de la imagen",
    "2. Tablero en cada esquina de la imagen (4 capturas)",
    "3. Inclinado a izquierda y derecha, unos 30 grados",
    "4. Inclinado arriba y abajo, unos 30 grados",
    "5. Cerca (llena la imagen) y lejos (se ve chico)",
]


def puntos_objeto():
    p = np.zeros((PATRON[0] * PATRON[1], 3), np.float32)
    p[:, :2] = np.mgrid[0:PATRON[0], 0:PATRON[1]].T.reshape(-1, 2)
    return p * LADO_CUADRO_MM


def buscar_tablero(gris):
    ok, esq = cv2.findChessboardCorners(
        gris, PATRON,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
    if not ok:
        return None
    return cv2.cornerSubPix(gris, esq, (11, 11), (-1, -1), CRITERIO)


def suficientemente_distinta(esq, capturas):
    if not capturas:
        return True
    c = esq.reshape(-1, 2).mean(axis=0)
    return all(np.linalg.norm(c - x.reshape(-1, 2).mean(axis=0)) > SEPARACION_MINIMA_PX
               for x in capturas)


def calibrar_y_guardar(capturas, tamano):
    obj = [puntos_objeto() for _ in capturas]
    rms, K, dist, _, _ = cv2.calibrateCamera(obj, capturas, tamano, None, None)
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    print("\n===== RESULTADO =====")
    print(f"  Capturas usadas: {len(capturas)}")
    print(f"  Error de reproyeccion (RMS): {rms:.3f} px")
    print(f"  Distancia focal: fx={fx:.1f}  fy={fy:.1f} px")
    print(f"  Centro optico:   cx={cx:.1f}  cy={cy:.1f} px  (imagen {tamano[0]}x{tamano[1]})")
    print(f"  Distorsion: {np.round(dist.ravel(), 4)}")
    fov = 2 * np.degrees(np.arctan(tamano[0] / (2 * fx)))
    print(f"  Campo de vision horizontal: {fov:.1f} grados")

    avisos = []
    if rms > RMS_ACEPTABLE_PX:
        avisos.append(f"RMS alto ({rms:.2f} px): repite con el tablero mas plano y mas capturas")
    if abs(fx - fy) / max(fx, fy) > 0.05:
        avisos.append("fx y fy difieren mas del 5 %: revisa que la resolucion no este estirada")
    if abs(cx - tamano[0] / 2) > tamano[0] * 0.15 or abs(cy - tamano[1] / 2) > tamano[1] * 0.15:
        avisos.append("el centro optico esta lejos del centro de la imagen: faltan poses variadas")
    for a in avisos:
        print("  ! " + a)

    ARCHIVO_SALIDA.parent.mkdir(parents=True, exist_ok=True)
    ARCHIVO_SALIDA.write_text(json.dumps({
        "version": 1,
        "ancho": tamano[0], "alto": tamano[1],
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "distorsion": dist.ravel().tolist(),
        "rms_px": float(rms),
        "capturas": len(capturas),
        "patron": list(PATRON),
        "lado_cuadro_mm": LADO_CUADRO_MM,
    }, indent=2), encoding="utf-8")
    print(f"\n  Guardado en {ARCHIVO_SALIDA.resolve()}")
    return not avisos


def main():
    forzado = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else None
    cap, indice, nombre = abrir_camara(forzado)
    if cap is None:
        print("Revisa que Iriun este conectado y transmitiendo.")
        return

    print(__doc__)
    print("Poses a cubrir:")
    for p in POSES:
        print("   " + p)
    print()

    capturas = []
    tamano = None
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Se corto la imagen.")
            break
        tamano = (frame.shape[1], frame.shape[0])
        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        esq = buscar_tablero(gris)
        vista = frame.copy()
        if esq is not None:
            cv2.drawChessboardCorners(vista, PATRON, esq, True)
        color = (0, 255, 0) if esq is not None else (0, 0, 255)
        cv2.putText(vista, f"[{indice}] {nombre}", (10, tamano[1] - 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        cv2.putText(vista, f"capturas: {len(capturas)}/{CAPTURAS_RECOMENDADAS}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.putText(vista, "TABLERO OK" if esq is not None else "no se ve el tablero", (10, 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(vista, "c=capturar  d=descartar  k=calibrar  q=salir", (10, tamano[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        cv2.imshow("Calibrar camara", vista)

        t = cv2.waitKey(1) & 0xFF
        if t in (27, ord("q")):
            print("Salida sin guardar.")
            break
        if t in (ord("c"), 32):
            if esq is None:
                print("  No se ve el tablero completo.")
            elif not suficientemente_distinta(esq, capturas):
                print("  Muy parecida a otra captura: mueve o inclina el tablero.")
            else:
                capturas.append(esq)
                print(f"  Captura {len(capturas)} guardada.")
        if t == ord("d") and capturas:
            capturas.pop()
            print(f"  Descartada. Quedan {len(capturas)}.")
        if t == ord("k"):
            if len(capturas) < CAPTURAS_MINIMAS:
                print(f"  Hacen falta al menos {CAPTURAS_MINIMAS} capturas.")
            else:
                calibrar_y_guardar(capturas, tamano)
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

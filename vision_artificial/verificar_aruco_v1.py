"""
VERIFICAR ARUCO  v1
-------------------
Comprueba que la camara detecte el marcador pegado en la tabla, cuantas veces
por segundo lo ve y de que tamano lo ve en pixeles.

Uso (desde la raiz del proyecto):
    python vision_artificial/verificar_aruco_v1.py
    python vision_artificial/verificar_aruco_v1.py 0      -> forzar otra camara

En la ventana: ESC o q para salir, g para guardar una foto.
Deja correr 30 segundos y anota el porcentaje de deteccion y el lado en pixeles.
"""

import sys
import cv2
import numpy as np

# ================== CONSTANTES AJUSTABLES ==================
INDICE_CAMARA = 1          # 1 = Iriun (la del celular)
ID_ESPERADO = 0            # el marcador pegado en la tabla
ANCHO = 1280
ALTO = 720
BACKEND = cv2.CAP_DSHOW
LADO_MINIMO_PX = 40        # por debajo de esto la posicion sale imprecisa
VENTANA_TASA = 100         # cuadros para calcular el porcentaje de deteccion
ARCHIVO_FOTO = "prueba_aruco.jpg"
# ===========================================================


def crear_detector():
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "ArucoDetector"):          # OpenCV 4.7 o mas nuevo
        det = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)[:2]
    par = cv2.aruco.DetectorParameters_create()      # OpenCV antiguo
    return lambda img: cv2.aruco.detectMarkers(img, dic, parameters=par)[:2]


def lado_px(esquinas):
    p = esquinas.reshape(4, 2)
    return float(np.mean([np.linalg.norm(p[i] - p[(i + 1) % 4]) for i in range(4)]))


def main():
    indice = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else INDICE_CAMARA
    detectar = crear_detector()
    cap = cv2.VideoCapture(indice, BACKEND)
    if not cap.isOpened():
        print(f"No se pudo abrir la camara {indice}.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO)
    print("ESC o q para salir | g para guardar foto")

    historial = []
    lados = []
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Se corto la imagen.")
            break
        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        esquinas, ids = detectar(gris)
        visto = False
        lado = 0.0
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, esquinas, ids)
            for c, i in zip(esquinas, ids.flatten()):
                if int(i) == ID_ESPERADO:
                    visto = True
                    lado = lado_px(c)
                    lados.append(lado)
        historial.append(visto)
        if len(historial) > VENTANA_TASA:
            historial.pop(0)
        if len(lados) > VENTANA_TASA:
            lados.pop(0)

        tasa = 100.0 * sum(historial) / len(historial)
        prom = np.mean(lados) if lados else 0.0
        estab = np.std(lados) if len(lados) > 5 else 0.0
        color = (0, 255, 0) if visto else (0, 0, 255)
        txt1 = f"ID {ID_ESPERADO}: {'VISTO' if visto else 'NO'} | deteccion {tasa:.0f}%"
        txt2 = f"lado {lado:.0f} px (prom {prom:.0f}, variacion {estab:.1f})"
        cv2.putText(frame, txt1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(frame, txt2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if 0 < prom < LADO_MINIMO_PX:
            cv2.putText(frame, "MUY CHICO: acerca o agranda el marcador", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        cv2.imshow("Verificar ArUco", frame)

        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord("q")):
            break
        if k == ord("g"):
            cv2.imwrite(ARCHIVO_FOTO, frame)
            print(f"  Guardada {ARCHIVO_FOTO}")

    cap.release()
    cv2.destroyAllWindows()
    if historial:
        print(f"\nDeteccion final: {100.0 * sum(historial) / len(historial):.0f} %")
        if lados:
            print(f"Lado promedio: {np.mean(lados):.1f} px | variacion {np.std(lados):.2f} px")


if __name__ == "__main__":
    main()

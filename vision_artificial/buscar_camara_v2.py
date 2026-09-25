"""
BUSCAR CAMARA  v2
-----------------
Muestra el NOMBRE de cada camara junto a su indice, para saber cual es la
del celular (Iriun) y cual la webcam de la laptop.

Uso (desde la raiz del proyecto):
    python vision_artificial/buscar_camara_v2.py        -> lista nombre + indice
    python vision_artificial/buscar_camara_v2.py 1      -> abre la camara 1

Para ver los nombres hace falta pygrabber (solo Windows):
    pip install pygrabber
Sin pygrabber igual funciona, pero solo lista indices.

En la ventana: ESC o q para salir, g para guardar una foto de prueba.
"""

import sys
import cv2

# ================== CONSTANTES AJUSTABLES ==================
MAX_INDICE = 5           # cuantos indices probar (0 .. MAX_INDICE-1)
ANCHO = 1280
ALTO = 720
BACKEND = cv2.CAP_DSHOW  # en Windows
ARCHIVO_FOTO = "prueba_camara.jpg"
UMBRAL_NEGRO = 8         # brillo medio por debajo de esto = imagen en negro
# ===========================================================


def nombres():
    """Nombres de las camaras en el mismo orden que los indices de OpenCV."""
    try:
        from pygrabber.dshow_graph import FilterGraph
        return FilterGraph().get_input_devices()
    except Exception:
        return None


def listar():
    lista = nombres()
    if lista is None:
        print("(sin pygrabber: no puedo mostrar nombres. pip install pygrabber)\n")
    print(f"Probando indices 0 a {MAX_INDICE - 1}...\n")
    for i in range(MAX_INDICE):
        nombre = lista[i] if lista and i < len(lista) else "?"
        cap = cv2.VideoCapture(i, BACKEND)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO)
        ok, frame = cap.read()
        if ok:
            h, w = frame.shape[:2]
            brillo = frame.mean()
            estado = "SIN SENAL (imagen negra)" if brillo < UMBRAL_NEGRO else "con imagen"
            print(f"  Camara {i}: {nombre}")
            print(f"             {w}x{h} | brillo medio {brillo:.1f} | {estado}")
        cap.release()
    print(f"\nPara ver una: python {sys.argv[0]} <indice>")


def ver(indice):
    lista = nombres()
    nombre = lista[indice] if lista and indice < len(lista) else f"cam {indice}"
    cap = cv2.VideoCapture(indice, BACKEND)
    if not cap.isOpened():
        print(f"No se pudo abrir la camara {indice}.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO)
    print(f"Abriendo [{indice}] {nombre}")
    print("ESC o q para salir | g para guardar una foto")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Se corto la imagen.")
            break
        h, w = frame.shape[:2]
        cv2.putText(frame, f"[{indice}] {nombre}  {w}x{h}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Camara", frame)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord("q")):
            break
        if k == ord("g"):
            cv2.imwrite(ARCHIVO_FOTO, frame)
            print(f"  Guardada {ARCHIVO_FOTO}")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        ver(int(sys.argv[1]))
    else:
        listar()

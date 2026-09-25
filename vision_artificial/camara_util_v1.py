"""
UTILIDAD DE CAMARA  v1
----------------------
Abre siempre la camara correcta buscandola por NOMBRE, no por indice.
El indice cambia cuando se conectan o quitan otras camaras (OBS, webcam, etc.),
asi que todos los programas del proyecto usan esto.

    from camara_util_v1 import abrir_camara
    cap, indice, nombre = abrir_camara()        # busca la preferida
    cap, indice, nombre = abrir_camara(2)       # fuerza un indice
"""

import cv2

# ================== CONSTANTES AJUSTABLES ==================
NOMBRE_PREFERIDO = "iriun"      # parte del nombre, sin importar mayusculas
ANCHO, ALTO = 1280, 720
BACKEND = cv2.CAP_DSHOW         # en Windows
MAX_INDICE = 6
UMBRAL_NEGRO = 8                # brillo medio por debajo de esto = sin senal
# ===========================================================


def nombres_camaras():
    """Nombres en el mismo orden que los indices de OpenCV (None si no se puede)."""
    try:
        from pygrabber.dshow_graph import FilterGraph
        return FilterGraph().get_input_devices()
    except Exception:
        return None


def _abrir(indice):
    cap = cv2.VideoCapture(indice, BACKEND)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO)
    ok, frame = cap.read()
    if not ok:
        cap.release()
        return None
    return cap, frame


def abrir_camara(indice=None, nombre_preferido=NOMBRE_PREFERIDO):
    """Devuelve (cap, indice, nombre) o (None, None, None)."""
    lista = nombres_camaras()

    if indice is not None:
        r = _abrir(indice)
        if r is None:
            print(f"  No se pudo abrir la camara {indice}.")
            return None, None, None
        cap, _ = r
        nombre = lista[indice] if lista and indice < len(lista) else f"camara {indice}"
        print(f"  Usando [{indice}] {nombre} (forzada)")
        return cap, indice, nombre

    # 1) por nombre, si pygrabber esta disponible
    if lista:
        for i, n in enumerate(lista):
            if nombre_preferido.lower() in n.lower():
                r = _abrir(i)
                if r is not None:
                    cap, frame = r
                    if frame.mean() < UMBRAL_NEGRO:
                        print(f"  ! [{i}] {n} esta en negro: revisa que el celular transmita.")
                    print(f"  Usando [{i}] {n}")
                    return cap, i, n
        print(f"  No aparece ninguna camara con '{nombre_preferido}' en el nombre.")
        print("  Camaras detectadas: " + ", ".join(f"[{i}] {n}" for i, n in enumerate(lista)))
        return None, None, None

    # 2) sin pygrabber: primera camara con imagen que no sea la 0 (suele ser la del portatil)
    print("  (sin pygrabber no puedo leer nombres: pip install pygrabber)")
    for i in list(range(1, MAX_INDICE)) + [0]:
        r = _abrir(i)
        if r is not None:
            cap, frame = r
            if frame.mean() >= UMBRAL_NEGRO:
                print(f"  Usando camara {i} (elegida a ciegas: verifica la imagen)")
                return cap, i, f"camara {i}"
            cap.release()
    print("  No se encontro ninguna camara con imagen.")
    return None, None, None

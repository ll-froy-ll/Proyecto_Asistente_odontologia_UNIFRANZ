import sys
import json
from pathlib import Path

from puente_serial import PuenteESP32, listar_puertos, configurar_logging

# Limites por direccion (min, max). Deben COINCIDIR con el firmware.
# Art5 es asimetrico: -5000 a +6000
LIM_MIN = {1: -2100, 2: -200, 3: -36000, 4: -4500, 5: -5000}
LIM_MAX = {1:  2100, 2:  200, 3:  36000, 4:  4500, 5:  6000}

# Zona de deteccion (debe coincidir con detectar_boca.py)
ZONA = {"zx1": 0.25, "zy1": 0.30, "zx2": 0.75, "zy2": 0.85}

ESQUINAS = ["TL", "TR", "BL", "BR", "CENTRO"]
ARCHIVO = Path("config/calibracion.json")


def clamp(art, val):
    return max(LIM_MIN[art], min(LIM_MAX[art], val))


def ayuda():
    print("""
--- CALIBRACION DE LA GRILLA ---
  <art> <pasos>          mueve esa articulacion (relativo). ej:  5 100  | 1 -50
  abs <art> <valor>      pone esa articulacion en absoluto. ej:  abs 5 6000
  fijar a1,a2,a3,a4,a5   pone la pose completa de una vez (restaurar poses)
  home                   vuelve todo a 0 (HOME)
  pose                   muestra la pose actual y lo guardado
  g <ESQUINA>            guarda la pose actual: g TL | g TR | g BL | g BR | g CENTRO
  escribir               guarda todo en config/calibracion.json
  ?                      esta ayuda
  q                      salir
  (art = 1..5,  ESQUINAS = TL TR BL BR CENTRO)
""")


def main():
    log = configurar_logging()
    puerto = sys.argv[1] if len(sys.argv) > 1 else "COM11"

    print("Puertos disponibles:")
    for p in listar_puertos():
        print("  ", p)

    esp = PuenteESP32(puerto, log=log)
    if not esp.conectar():
        print("No se pudo conectar (cerra el monitor de ESP-IDF).")
        return

    pose = [0, 0, 0, 0, 0]   # HOME
    guardadas = {}

    print("\nBrazo en HOME. Mové la lámpara a cada posicion y guardala.")
    esp.mover(*pose)
    ayuda()

    while True:
        try:
            txt = input("calib> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not txt:
            continue
        low = txt.lower()

        if low in ("q", "salir", "exit"):
            break
        if low in ("?", "h", "ayuda"):
            ayuda(); continue
        if low == "pose":
            print("  pose actual:", pose)
            if guardadas:
                for k, v in guardadas.items():
                    print(f"    {k}: {v}")
            faltan = [e for e in ESQUINAS[:4] if e not in guardadas]
            if faltan:
                print("  faltan esquinas:", faltan)
            continue
        if low == "home":
            pose = [0, 0, 0, 0, 0]
            esp.mover(*pose)
            print("  en HOME")
            continue
        if low == "escribir":
            faltan = [e for e in ESQUINAS[:4] if e not in guardadas]
            if faltan:
                print("  Faltan esquinas:", faltan, "- guardalas antes de escribir.")
                continue
            datos = {"zona": ZONA, "poses": {}}
            for e in ["TL", "TR", "BL", "BR"]:
                datos["poses"][e] = guardadas[e]
            if "CENTRO" in guardadas:
                datos["poses"]["CENTRO"] = guardadas["CENTRO"]
            else:
                datos["poses"]["CENTRO"] = [
                    round(sum(guardadas[e][i] for e in ["TL", "TR", "BL", "BR"]) / 4)
                    for i in range(5)
                ]
            ARCHIVO.parent.mkdir(parents=True, exist_ok=True)
            ARCHIVO.write_text(json.dumps(datos, indent=2), encoding="utf-8")
            print(f"  Guardado en {ARCHIVO.resolve()}")
            log.info("Calibracion escrita: %s", datos["poses"])
            continue

        partes = txt.split()

        if partes[0].lower() == "fijar":
            resto = txt[len(partes[0]):].replace(",", " ").split()
            if len(resto) != 5:
                print("  fijar necesita 5 numeros"); continue
            try:
                vals = [int(x) for x in resto]
            except ValueError:
                print("  numeros invalidos"); continue
            pose = [clamp(i + 1, vals[i]) for i in range(5)]
            esp.mover(*pose)
            print("  pose:", pose)
            continue

        if partes[0].lower() == "g" and len(partes) == 2:
            nombre = partes[1].upper()
            if nombre not in ESQUINAS:
                print("  Esquina invalida. Usa TL TR BL BR CENTRO")
                continue
            guardadas[nombre] = pose.copy()
            print(f"  guardada {nombre} = {pose}")
            continue

        if partes[0].lower() == "abs" and len(partes) == 3:
            try:
                a, val = int(partes[1]), int(partes[2])
            except ValueError:
                print("  numeros invalidos"); continue
            if not (1 <= a <= 5):
                print("  art 1..5"); continue
            pose[a - 1] = clamp(a, val)
            esp.mover(*pose)
            print("  pose:", pose)
            continue

        # nudge relativo: <art> <pasos>
        if len(partes) == 2:
            try:
                a, d = int(partes[0]), int(partes[1])
            except ValueError:
                print("  No entendi. Escribe ? para ayuda."); continue
            if not (1 <= a <= 5):
                print("  art 1..5"); continue
            pose[a - 1] = clamp(a, pose[a - 1] + d)
            esp.mover(*pose)
            print("  pose:", pose)
            continue

        print("  No entendi. Escribe ? para ayuda.")

    esp.cerrar()
    print("Listo.")


if __name__ == "__main__":
    main()
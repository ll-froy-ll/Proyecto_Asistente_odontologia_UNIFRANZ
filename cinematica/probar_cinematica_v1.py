"""
PROBAR CINEMATICA  v1
---------------------
Verifica el modelo cinematico contra el brazo real.

Uso (desde la RAIZ del proyecto, brazo en HOME, monitor ESP-IDF cerrado):
    python cinematica/probar_cinematica_v1.py COM11

Comandos
  fk p1 p2 p3 p4 p5    calcula donde quedaria la linterna con esos pasos (no mueve)
  ir p1 p2 p3 p4 p5    mueve el brazo a esos pasos y muestra la prediccion
  apunta x y z [d]     calcula la pose para iluminar el punto (x, y, z) mm y pregunta si mover
  home                 vuelve a 0,0,0,0,0
  info                 resumen del modelo
  q                    vuelve a HOME y sale

Los pasos van en el orden del firmware: Art1 Art2 Art3 Art4 Art5
  (Art1 pitch, Art2 roll, Art3 codo, Art4 hombro, Art5 base)
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "control_motores"))

from cinematica_v1 import ModeloBrazo, ORDEN, ARCHIVO_CONFIG     # noqa: E402
from puente_serial import PuenteESP32, configurar_logging 

# ================== CONSTANTES AJUSTABLES ==================
PUERTO_DEF = "COM11"
PEDIR_CONFIRMACION = True      # preguntar antes de mover
# ===========================================================


def mostrar(modelo, q, titulo="Prediccion"):
    f = modelo.directa(q)
    fmt = lambda v: f"({v[0]:7.1f}, {v[1]:7.1f}, {v[2]:7.1f})"
    print(f"\n  {titulo}")
    print("  angulos (grados): " + "  ".join(f"{n}={a:+.1f}" for n, a in zip(ORDEN, q)))
    print(f"  pasos Art1..Art5: {modelo.a_pasos(q)}")
    print(f"  eje codo   (mm): {fmt(f['codo'])}")
    print(f"  eje Art1   (mm): {fmt(f['art1'])}")
    print(f"  LENTE      (mm): {fmt(f['lente'])}")
    print(f"  direccion haz  : {fmt(f['haz'])}")
    mal = modelo.problemas(q)
    if mal:
        print("  ! ATENCION: " + ", ".join(mal))


def leer_pasos(partes):
    if len(partes) != 6:
        print("  Uso: fk|ir p1 p2 p3 p4 p5")
        return None
    try:
        return tuple(int(x) for x in partes[1:])
    except ValueError:
        print("  Los pasos deben ser enteros.")
        return None


def confirmar(msg="  Mover el brazo? (s/n): "):
    if not PEDIR_CONFIRMACION:
        return True
    return input(msg).strip().lower() == "s"


def main():
    log = configurar_logging()
    modelo = ModeloBrazo.cargar(ARCHIVO_CONFIG)
    puerto = sys.argv[1] if len(sys.argv) > 1 else PUERTO_DEF

    esp = PuenteESP32(puerto, log=log)
    if not esp.conectar():
        print("No se pudo conectar con el brazo: solo calculos (fk, apunta, info).")
        esp = None
    else:
        esp.mover(0, 0, 0, 0, 0)

    q_actual = np.zeros(5)
    print(__doc__)

    def mover(pasos):
        nonlocal q_actual
        if esp is None:
            print("  (sin brazo conectado: no se mueve)")
            return False
        ok = esp.mover(*pasos)
        if ok:
            q_actual = modelo.a_grados(pasos)
        else:
            print("  ! El ESP32 no confirmo el movimiento.")
        return ok

    try:
        while True:
            txt = input("cinematica> ").strip().lower()
            if not txt:
                continue
            p = txt.split()
            cmd = p[0]
            if cmd == "q":
                break
            elif cmd == "info":
                print(f"\n  Tramos A/B/C: {modelo.A:.0f} / {modelo.B:.0f} / {modelo.C:.0f} mm")
                print(f"  Lente en HOME respecto del eje Art1: {np.round(modelo.lente0, 1)} mm")
                print(f"  Haz en HOME: {np.round(modelo.haz0, 2)} | distancia de trabajo {modelo.distancia:.0f} mm")
                for n in ORDEN:
                    lo, hi = modelo.lim_deg[n]
                    print(f"  {n:13s} {modelo.ppg[n]:+9.3f} pasos/grado   rango {lo:+7.1f} a {hi:+7.1f} grados")
                mostrar(modelo, np.zeros(5), "HOME")
            elif cmd == "home":
                mover((0, 0, 0, 0, 0))
            elif cmd in ("fk", "ir"):
                pasos = leer_pasos(p)
                if pasos is None:
                    continue
                q = modelo.a_grados(pasos)
                mostrar(modelo, q)
                if cmd == "ir" and confirmar():
                    if mover(pasos):
                        print("  Listo. Mide el centro del lente y compara con LENTE.")
            elif cmd == "apunta":
                if len(p) not in (4, 5):
                    print("  Uso: apunta x y z [distancia]")
                    continue
                try:
                    boca = [float(v) for v in p[1:4]]
                    dist = float(p[4]) if len(p) == 5 else None
                except ValueError:
                    print("  Numeros invalidos.")
                    continue
                print("  Buscando pose...")
                r = modelo.apuntar(boca, distancia=dist, q_actual=q_actual)
                if r is None:
                    print("  Sin solucion segura para ese punto. Proba otro mas alcanzable.")
                    continue
                mostrar(modelo, r["q"], "Pose para iluminar el punto")
                print(f"  haz desviado {r['desvio_haz_deg']:.0f} grados de la vertical preferida")
                if confirmar():
                    if mover(r["pasos"]):
                        print("  Listo. El centro de la mancha deberia caer sobre el punto.")
            else:
                print("  Comandos: fk | ir | apunta | home | info | q")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        if esp is not None:
            print("Volviendo a HOME...")
            try:
                esp.mover(0, 0, 0, 0, 0)
            except Exception:
                pass
            esp.cerrar()


if __name__ == "__main__":
    main()

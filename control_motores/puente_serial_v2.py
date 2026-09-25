"""
PUENTE SERIAL  v2
-----------------
Conexion con el ESP32 en modo autonomo (firmware v3.5).

Mejoras respecto de v1:
  - Timeout proporcional al movimiento: se calcula cuanto deberia tardar cada
    articulacion segun sus pasos, su velocidad y su rampa. Antes eran 20 s fijos
    para todo, asi que un movimiento largo daba falso timeout y uno corto dejaba
    la app colgada 20 s.
  - Reintento automatico: si no llega el OK, reenvia la pose. Como son posiciones
    ABSOLUTAS, reenviar es seguro (no se acumula movimiento).
  - Reconexion automatica si se desenchufa el USB o se cae el puerto.
  - Descarta lineas de ruido en vez de tomarlas por respuesta.
  - posiciones(): consulta las posiciones reales al ESP32 (si el firmware lo
    soporta); devuelve None si no.

API compatible con la v1: PuenteESP32(puerto).conectar() / .mover(...) / .cerrar()

Uso suelto:
    python control_motores/puente_serial_v2.py COM11
"""

import sys
import time
import logging
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

# ================== CONSTANTES AJUSTABLES ==================
PUERTO_DEF = "COM11"
BAUD = 115200
ESPERA_BOOT_S = 2.5          # el ESP32 se resetea al abrir el puerto
INTENTOS_MOVER = 2           # envios de la misma pose antes de darse por vencido
INTENTOS_CONECTAR = 2
TIMEOUT_MINIMO_S = 2.0
TIMEOUT_MARGEN = 1.8         # factor sobre el tiempo calculado
TIMEOUT_EXTRA_S = 1.5        # margen fijo (pausas del firmware, troceado)
RECONECTAR = True            # reabrir el puerto solo si se cae

# Copia de los valores del firmware v3.5 (solo para estimar cuanto tarda)
#                    ---   Art1   Art2   Art3   Art4   Art5
VEL_US    = [0,  500,  4000,   150,   500,   500]
RAMPA     = [0,  300,     0,  2000,   800,   300]
VEL_INI_US= [0, 2000,     0,  1000,  2000,  2000]
LIM_MIN   = [0, -2100, -200, -36000, -4500, -5000]
LIM_MAX   = [0,  2100,  200,  36000,  4500,  6000]
# ===========================================================


def configurar_logging():
    carpeta = Path("logs")
    carpeta.mkdir(exist_ok=True)
    archivo = carpeta / f"control_{datetime.now():%Y%m%d}.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s  %(levelname)s  %(message)s",
        handlers=[logging.FileHandler(archivo, encoding="utf-8"), logging.StreamHandler()],
    )
    return logging.getLogger("control")


def listar_puertos():
    return [f"{p.device} ({p.description})" for p in list_ports.comports()]


def recortar(indice, valor):
    return max(LIM_MIN[indice], min(LIM_MAX[indice], int(valor)))


def duracion_estimada_s(pos_actual, objetivo):
    """Cuanto tarda el movimiento mas largo, en segundos."""
    peor = 0.0
    for i in range(1, 6):
        pasos = abs(recortar(i, objetivo[i - 1]) - pos_actual[i - 1])
        if pasos == 0:
            continue
        # tramo a velocidad de crucero
        rampa = min(RAMPA[i], pasos // 2)
        crucero = pasos - 2 * rampa
        t = crucero * 2 * VEL_US[i] / 1e6
        if rampa > 0:
            medio = (VEL_INI_US[i] + VEL_US[i]) / 2.0     # promedio durante la rampa
            t += 2 * rampa * 2 * medio / 1e6
        peor = max(peor, t)
    return peor


class PuenteESP32:
    """Maneja la conexion serial con el ESP32 en modo autonomo."""

    def __init__(self, puerto=PUERTO_DEF, baud=BAUD, log=None):
        self.puerto = puerto
        self.baud = baud
        self.ser = None
        self.log = log or logging.getLogger("control")
        self.pos = [0, 0, 0, 0, 0]        # posicion que creemos que tiene el brazo
        self.reconexiones = 0

    # ----------------------------- conexion -----------------------------
    def _entrar_autonomo(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"4\n")
        t0 = time.time()
        while time.time() - t0 < 4.0:
            linea = self.ser.readline().decode(errors="ignore").strip()
            if linea and "AUTONOMO" in linea.upper():
                self.ser.reset_input_buffer()
                return True
        return False

    def conectar(self):
        for intento in range(1, INTENTOS_CONECTAR + 1):
            try:
                self.log.info("Abriendo %s a %d baudios (intento %d)...",
                              self.puerto, self.baud, intento)
                self.ser = serial.Serial(self.puerto, self.baud, timeout=1)
                time.sleep(ESPERA_BOOT_S)
                if self._entrar_autonomo():
                    self.log.info("ESP32 en MODO AUTONOMO.")
                    self.pos = [0, 0, 0, 0, 0]
                    return True
                # por si quedo en modo manual o autonomo de una sesion anterior
                self.log.warning("Reintentando entrar a modo autonomo...")
                self.ser.write(b"menu\n")
                time.sleep(0.5)
                if self._entrar_autonomo():
                    self.log.info("ESP32 en MODO AUTONOMO (en el reintento).")
                    self.pos = [0, 0, 0, 0, 0]
                    return True
                self.ser.close()
                self.ser = None
            except serial.SerialException as e:
                self.log.error("Error de puerto: %s", e)
                self.ser = None
                time.sleep(1.0)
        self.log.error("No pude entrar a modo autonomo.")
        return False

    def conectado(self):
        return self.ser is not None and self.ser.is_open

    def _reconectar(self):
        if not RECONECTAR:
            return False
        self.log.warning("Intentando reconectar...")
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        pos_previa = list(self.pos)
        if self.conectar():
            self.reconexiones += 1
            # el ESP32 se reinicio: cree estar en HOME
            self.pos = [0, 0, 0, 0, 0]
            self.log.warning("Reconectado. OJO: el ESP32 volvio a HOME "
                             "(antes creia estar en %s).", pos_previa)
            return True
        return False

    # ----------------------------- movimiento -----------------------------
    def _esperar_respuesta(self, timeout, esperadas=("OK", "ERR")):
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                linea = self.ser.readline().decode(errors="ignore").strip()
            except serial.SerialException as e:
                self.log.error("Se corto el puerto: %s", e)
                return None
            if not linea:
                continue
            if linea in esperadas or linea.startswith("P,"):
                return linea
            self.log.debug("(ruido) %s", linea)
        return None

    def mover(self, a1, a2, a3, a4, a5, timeout=None):
        """Envia las 5 posiciones absolutas y espera el OK del ESP32."""
        objetivo = [int(a1), int(a2), int(a3), int(a4), int(a5)]
        if not self.conectado() and not self._reconectar():
            self.log.error("No hay conexion.")
            return False

        if timeout is None:
            est = duracion_estimada_s(self.pos, objetivo)
            timeout = max(TIMEOUT_MINIMO_S, est * TIMEOUT_MARGEN + TIMEOUT_EXTRA_S)

        cmd = ",".join(str(v) for v in objetivo) + "\n"
        for intento in range(1, INTENTOS_MOVER + 1):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd.encode())
            except serial.SerialException as e:
                self.log.error("Fallo al escribir: %s", e)
                if not self._reconectar():
                    return False
                continue
            self.log.info("-> %s (timeout %.1f s%s)", cmd.strip(), timeout,
                          "" if intento == 1 else f", intento {intento}")
            r = self._esperar_respuesta(timeout)
            if r == "OK":
                self.log.info("<- OK")
                self.pos = [recortar(i + 1, objetivo[i]) for i in range(5)]
                return True
            if r == "ERR":
                self.log.error("<- ERR (formato rechazado por el ESP32)")
                return False
            self.log.warning("Sin respuesta (timeout de %.1f s).", timeout)
            if not self.conectado() and not self._reconectar():
                return False
        self.log.error("El movimiento no se confirmo tras %d intentos.", INTENTOS_MOVER)
        return False

    def posiciones(self):
        """Posiciones reales segun el ESP32, o None si el firmware no lo soporta."""
        if not self.conectado():
            return None
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"p\n")
        except serial.SerialException:
            return None
        r = self._esperar_respuesta(1.5)
        if r and r.startswith("P,"):
            try:
                vals = [int(v) for v in r[2:].split(",")]
                if len(vals) == 5:
                    return vals
            except ValueError:
                pass
        return None

    def verificar_posiciones(self):
        """Compara lo que cree la app con lo que dice el ESP32. True si coinciden."""
        reales = self.posiciones()
        if reales is None:
            return None
        if reales != self.pos:
            self.log.error("DESFASE: la app cree %s y el ESP32 dice %s", self.pos, reales)
            self.pos = reales
            return False
        return True

    def cerrar(self):
        if self.ser is not None:
            try:
                self.ser.write(b"menu\n")
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.log.info("Conexion cerrada. Reconexiones en la sesion: %d", self.reconexiones)


def main():
    log = configurar_logging()
    puerto = sys.argv[1] if len(sys.argv) > 1 else PUERTO_DEF

    print("Puertos disponibles:")
    for p in listar_puertos():
        print("  ", p)
    print()

    esp = PuenteESP32(puerto, log=log)
    if not esp.conectar():
        print("No se pudo conectar. Revisa el puerto y que el monitor de ESP-IDF este cerrado.")
        return

    print("\nConectado. Escribe 5 numeros (pasos absolutos) separados por espacio o coma.")
    print("  p   consulta las posiciones al ESP32")
    print("  q   salir\n")

    while True:
        try:
            txt = input("pose> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if txt.lower() in ("q", "salir", "exit"):
            break
        if txt.lower() == "p":
            r = esp.posiciones()
            print("  ", r if r is not None else "el firmware no responde posiciones")
            continue

        partes = txt.replace(",", " ").split()
        if len(partes) != 5:
            print("  Necesito exactamente 5 numeros.")
            continue
        try:
            vals = [int(p) for p in partes]
        except ValueError:
            print("  Solo numeros enteros.")
            continue

        t0 = time.time()
        ok = esp.mover(*vals)
        print(f"  -> {'OK' if ok else 'sin respuesta / rechazado'} en {time.time() - t0:.1f} s")

    esp.cerrar()
    print("Listo.")


if __name__ == "__main__":
    main()

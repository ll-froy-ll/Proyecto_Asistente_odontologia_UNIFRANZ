import sys
import time
import logging
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports


def configurar_logging():
    carpeta = Path("logs")
    carpeta.mkdir(exist_ok=True)
    archivo = carpeta / f"control_{datetime.now():%Y%m%d}.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s  %(levelname)s  %(message)s",
        handlers=[
            logging.FileHandler(archivo, encoding="utf-8"),
            logging.StreamHandler(),
        ],
    )
    return logging.getLogger("control")


def listar_puertos():
    return [f"{p.device} ({p.description})" for p in list_ports.comports()]


class PuenteESP32:
    """Maneja la conexion serial con el ESP32 en modo autonomo."""

    def __init__(self, puerto="COM11", baud=115200, log=None):
        self.puerto = puerto
        self.baud = baud
        self.ser = None
        self.log = log or logging.getLogger("control")

    def _entrar_autonomo(self):
        """Manda '4' y espera el marcador del firmware."""
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
        self.log.info("Abriendo %s a %d baudios...", self.puerto, self.baud)
        self.ser = serial.Serial(self.puerto, self.baud, timeout=1)
        time.sleep(2.5)  # el ESP32 se resetea al abrir el puerto: esperar boot

        if self._entrar_autonomo():
            self.log.info("ESP32 en MODO AUTONOMO.")
            return True

        # Recuperacion: por si quedo en otro modo
        self.log.warning("Reintentando entrar a modo autonomo...")
        self.ser.write(b"menu\n")
        time.sleep(0.5)
        if self._entrar_autonomo():
            self.log.info("ESP32 en MODO AUTONOMO (en el reintento).")
            return True

        self.log.error("No pude entrar a modo autonomo.")
        return False

    def mover(self, a1, a2, a3, a4, a5, timeout=20.0):
        """Envia las 5 posiciones absolutas y espera el OK del ESP32."""
        if self.ser is None:
            self.log.error("No hay conexion.")
            return False

        cmd = f"{int(a1)},{int(a2)},{int(a3)},{int(a4)},{int(a5)}\n"
        self.ser.reset_input_buffer()
        self.ser.write(cmd.encode())
        self.log.info("-> %s", cmd.strip())

        t0 = time.time()
        while time.time() - t0 < timeout:
            linea = self.ser.readline().decode(errors="ignore").strip()
            if linea == "OK":
                self.log.info("<- OK")
                return True
            if linea == "ERR":
                self.log.error("<- ERR (formato rechazado por el ESP32)")
                return False
        self.log.error("Sin respuesta (timeout).")
        return False

    def cerrar(self):
        if self.ser is not None:
            try:
                self.ser.write(b"menu\n")
            except Exception:
                pass
            self.ser.close()
            self.ser = None
            self.log.info("Conexion cerrada.")


def main():
    log = configurar_logging()
    puerto = sys.argv[1] if len(sys.argv) > 1 else "COM11"

    print("Puertos disponibles:")
    for p in listar_puertos():
        print("  ", p)
    print()

    esp = PuenteESP32(puerto, log=log)
    if not esp.conectar():
        print("No se pudo conectar. Revisa el puerto y que el monitor de ESP-IDF este cerrado.")
        return

    print("\nConectado. Escribe 5 numeros (pasos absolutos) separados por espacio o coma.")
    print("Ejemplos:  0 0 0 0 1000   |   0,0,0,0,5000   |   q para salir\n")

    while True:
        try:
            txt = input("pose> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if txt.lower() in ("q", "salir", "exit"):
            break

        partes = txt.replace(",", " ").split()
        if len(partes) != 5:
            print("  Necesito exactamente 5 numeros.")
            continue
        try:
            vals = [int(p) for p in partes]
        except ValueError:
            print("  Solo numeros enteros.")
            continue

        ok = esp.mover(*vals)
        print("  -> movimiento OK" if ok else "  -> sin respuesta / rechazado")

    esp.cerrar()
    print("Listo.")


if __name__ == "__main__":
    main()

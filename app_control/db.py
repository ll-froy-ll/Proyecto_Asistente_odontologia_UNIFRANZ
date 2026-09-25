import sqlite3
from pathlib import Path

RUTA = Path("config/asistente.db")

DEFAULTS = {
    "com": "COM11",
    "camara": "0",
    "deadband": "0.06",
    "frames": "3",
    "tema": "claro",
}


def _conn():
    RUTA.parent.mkdir(exist_ok=True)
    c = sqlite3.connect(str(RUTA))
    c.execute("CREATE TABLE IF NOT EXISTS ajustes (clave TEXT PRIMARY KEY, valor TEXT)")
    return c


def cargar():
    c = _conn()
    filas = dict(c.execute("SELECT clave, valor FROM ajustes").fetchall())
    c.close()
    datos = dict(DEFAULTS)
    datos.update(filas)
    return datos


def guardar(ajustes):
    c = _conn()
    for k, v in ajustes.items():
        c.execute(
            "INSERT INTO ajustes(clave, valor) VALUES(?, ?) "
            "ON CONFLICT(clave) DO UPDATE SET valor=excluded.valor",
            (k, str(v)),
        )
    c.commit()
    c.close()


def set_valor(clave, valor):
    guardar({clave: valor})

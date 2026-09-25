"""
CINEMATICA DEL BRAZO  v1
------------------------
Modelo del BCN3D Moveo adaptado con linterna, a partir de config/cinematica.json.

  - grados <-> pasos (orden del firmware: Art1, Art2, Art3, Art4, Art5)
  - cinematica directa: dado un juego de angulos, donde queda la linterna
  - cinematica inversa: dada la boca, que angulos ponen la luz sobre ella

Convenciones (heredadas de la Fase 0)
  Origen: punto de la tabla bajo el eje de la base. +x flecha, +y izquierda, +z arriba.
  Angulos en grados, 0 = HOME (brazo vertical).
  base, muneca_roll : + = antihorario visto desde arriba
  hombro, codo, muneca_pitch : + = inclina hacia el lado indicado en plano_brazo

Vector de articulaciones q (en este orden):
  q = [base, hombro, codo, muneca_roll, muneca_pitch]
"""

import json
import math
from pathlib import Path

import numpy as np

# ================== CONSTANTES AJUSTABLES ==================
ARCHIVO_CONFIG = Path("config/cinematica.json")

Z_MIN_MM = 40.0              # ningun punto del brazo por debajo de esta altura
MARGEN_LIMITE_DEG = 2.0      # no acercarse al tope de cada articulacion
CAMARA_RADIO_MM = 150.0      # zona prohibida alrededor de la camara
HAZ_PREFERIDO = (0.0, 0.0, -1.0)   # la luz idealmente cae desde arriba

TOL_POS_MM = 1.0             # tolerancia de posicion del lente
TOL_DIR_DEG = 0.5            # tolerancia de direccion del haz
MAX_ITER = 150
AMORTIGUAMIENTO = 0.3        # Levenberg-Marquardt
PESO_DIRECCION_MM = 300.0    # 1 rad de error de direccion "vale" 300 mm
ABANDONAR_ITER = 40          # si a esta iteracion el error sigue grande, se abandona la semilla
ABANDONAR_ERROR_MM = 40.0
SOLUCIONES_POR_HAZ = 2       # soluciones a comparar por direccion antes de decidir

# Busqueda de direcciones de iluminacion (angulo del haz respecto de la vertical)
BUSQUEDA_CONO_DEG = (0, 10, 20, 30, 40, 50, 60)
BUSQUEDA_AZIMUT_PASO_DEG = 30
# ===========================================================

ORDEN = ("base", "hombro", "codo", "muneca_roll", "muneca_pitch")
ART_DE = {"base": 5, "hombro": 4, "codo": 3, "muneca_roll": 2, "muneca_pitch": 1}
DIRECCIONES = {
    "frente": (1.0, 0.0, 0.0), "atras": (-1.0, 0.0, 0.0),
    "izquierda": (0.0, 1.0, 0.0), "derecha": (0.0, -1.0, 0.0),
}
Z = np.array([0.0, 0.0, 1.0])


def rot(eje, ang_deg):
    """Matriz de rotacion (Rodrigues) alrededor de un eje unitario."""
    k = np.asarray(eje, dtype=float)
    k = k / np.linalg.norm(k)
    a = math.radians(ang_deg)
    kx, ky, kz = k
    K = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def unitario(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def angulo_entre(u, v):
    c = float(np.clip(np.dot(unitario(u), unitario(v)), -1.0, 1.0))
    return math.degrees(math.acos(c))


class ModeloBrazo:
    def __init__(self, cfg):
        self.cfg = cfg
        t = cfg["tramos_mm"]
        self.A = float(t["A_tabla_art4"])
        self.B = float(t["B_art4_art3"])
        self.C = float(t["C_art3_art1"])

        # plano de inclinacion: eje de giro de hombro, codo y pitch
        lado = unitario(DIRECCIONES[cfg["plano_brazo"]["art4_positivo_inclina_hacia"]])
        self.eje_pitch = unitario(np.cross(Z, lado))   # girar +ang inclina Z hacia 'lado'

        # linterna en HOME (relativa al eje de la Art1)
        li = cfg["linterna"]
        h = unitario(DIRECCIONES[li["direccion_haz_home"]])
        inc = math.radians(li.get("inclinacion_haz_deg") or 0.0)
        costado = unitario(np.cross(Z, h))
        self.haz0 = unitario(h * math.cos(inc) + Z * math.sin(inc))
        self.lente0 = (li["adelante_mm"] * h + li["arriba_mm"] * Z
                       + (li.get("costado_mm") or 0.0) * costado)
        if li.get("lado_largo_en_home") == "vertical":
            self.largo0 = unitario(np.cross(self.haz0, costado))
        else:
            self.largo0 = costado
        self.distancia = float(li["distancia_trabajo_mm"])

        # articulaciones: pasos por grado y limites en grados
        self.ppg = {}
        self.lim_pasos = {}
        self.lim_deg = {}
        for num, a in cfg["articulaciones"].items():
            n = a["nombre"]
            self.ppg[n] = float(a["pasos_por_grado"])
            self.lim_pasos[n] = (int(a["min_pasos"]), int(a["max_pasos"]))
            lims = sorted([a["min_pasos"] / self.ppg[n], a["max_pasos"] / self.ppg[n]])
            self.lim_deg[n] = (lims[0] + MARGEN_LIMITE_DEG, lims[1] - MARGEN_LIMITE_DEG)
        self.q_min = np.array([self.lim_deg[n][0] for n in ORDEN])
        self.q_max = np.array([self.lim_deg[n][1] for n in ORDEN])

        cam = cfg.get("camara") or {}
        self.camara = (np.array([cam["x_mm"], cam["y_mm"], cam["z_mm"]], dtype=float)
                       if cam.get("x_mm") is not None else None)

    @classmethod
    def cargar(cls, archivo=ARCHIVO_CONFIG):
        return cls(json.loads(Path(archivo).read_text(encoding="utf-8")))

    # ------------------------- pasos <-> grados -------------------------
    def a_pasos(self, q):
        """q en grados (ORDEN) -> tupla de pasos en orden del firmware Art1..Art5."""
        por_art = {}
        for n, ang in zip(ORDEN, q):
            p = int(round(ang * self.ppg[n]))
            lo, hi = self.lim_pasos[n]
            por_art[ART_DE[n]] = max(lo, min(hi, p))
        return tuple(por_art[i] for i in (1, 2, 3, 4, 5))

    def a_grados(self, pasos):
        """Pasos en orden Art1..Art5 -> q en grados (ORDEN)."""
        por_art = dict(zip((1, 2, 3, 4, 5), pasos))
        return np.array([por_art[ART_DE[n]] / self.ppg[n] for n in ORDEN])

    # ------------------------- cinematica directa -------------------------
    def directa(self, q):
        base, hombro, codo, roll, pitch = q
        R = rot(Z, base)
        p_hombro = R @ np.array([0.0, 0.0, self.A])
        R = R @ rot(self.eje_pitch, hombro)
        p_codo = p_hombro + R @ np.array([0.0, 0.0, self.B])
        R = R @ rot(self.eje_pitch, codo)
        p_art1 = p_codo + R @ np.array([0.0, 0.0, self.C])
        R = R @ rot(Z, roll) @ rot(self.eje_pitch, pitch)
        return {
            "hombro": p_hombro,
            "codo": p_codo,
            "art1": p_art1,
            "lente": p_art1 + R @ self.lente0,
            "haz": R @ self.haz0,
            "lado_largo": R @ self.largo0,
        }

    # ------------------------- chequeos -------------------------
    def dentro_de_limites(self, q):
        return bool(np.all(q >= self.q_min - 1e-6) and np.all(q <= self.q_max + 1e-6))

    def puntos_cuerpo(self, f):
        pts = [f["codo"], f["art1"], f["lente"]]
        pts += [(f["hombro"] + f["codo"]) / 2, (f["codo"] + f["art1"]) / 2,
                (f["art1"] + f["lente"]) / 2]
        return pts

    def problemas(self, q):
        """Lista de motivos por los que una pose no es segura (vacia = OK)."""
        f = self.directa(q)
        mal = []
        if not self.dentro_de_limites(q):
            mal.append("fuera de limites")
        if any(p[2] < Z_MIN_MM for p in self.puntos_cuerpo(f)):
            mal.append("muy cerca de la tabla")
        if self.camara is not None and any(
                np.linalg.norm(p - self.camara) < CAMARA_RADIO_MM for p in self.puntos_cuerpo(f)):
            mal.append("muy cerca de la camara")
        return mal

    # ------------------------- cinematica inversa -------------------------
    def _residuo(self, q, lente_obj, haz_obj):
        f = self.directa(q)
        return np.concatenate([f["lente"] - lente_obj,
                               (f["haz"] - haz_obj) * PESO_DIRECCION_MM])

    def inversa_lente(self, lente_obj, haz_obj, q0):
        """Angulos que ponen el lente en lente_obj con el haz en haz_obj."""
        lente_obj = np.asarray(lente_obj, dtype=float)
        haz_obj = unitario(haz_obj)
        q = np.clip(np.asarray(q0, dtype=float), self.q_min, self.q_max)
        for it in range(MAX_ITER):
            r = self._residuo(q, lente_obj, haz_obj)
            f = self.directa(q)
            err_pos = np.linalg.norm(f["lente"] - lente_obj)
            err_dir = angulo_entre(f["haz"], haz_obj)
            if err_pos < TOL_POS_MM and err_dir < TOL_DIR_DEG:
                return q, err_pos, err_dir
            if it == ABANDONAR_ITER and err_pos > ABANDONAR_ERROR_MM:
                break
            J = np.zeros((6, 5))
            d = 1e-3
            for i in range(5):
                qd = q.copy()
                qd[i] += d
                J[:, i] = (self._residuo(qd, lente_obj, haz_obj) - r) / d
            H = J.T @ J + (AMORTIGUAMIENTO ** 2) * np.eye(5)
            q = np.clip(q + np.linalg.solve(H, -J.T @ r), self.q_min, self.q_max)
        f = self.directa(q)
        return q, np.linalg.norm(f["lente"] - lente_obj), angulo_entre(f["haz"], haz_obj)

    def semillas(self, objetivo):
        """Puntos de partida razonables para la inversa."""
        az = math.degrees(math.atan2(objetivo[1], objetivo[0]))
        # el brazo se inclina hacia 'lado' cuando base = 0: corregir el azimut
        lado = np.cross(self.eje_pitch, Z)
        az_lado = math.degrees(math.atan2(lado[1], lado[0]))
        base = ((az - az_lado + 180) % 360) - 180
        s = []
        for b in (base, base + 180, base - 180, 0.0):
            for h, c in ((30, 30), (15, 50), (-30, -30)):
                s.append(np.array([b, h, c, 0.0, 0.0]))
        return [np.clip(x, self.q_min, self.q_max) for x in s]

    def apuntar(self, boca, distancia=None, haz_preferido=HAZ_PREFERIDO, q_actual=None):
        """
        Busca la pose que ilumina 'boca' desde 'distancia' mm.
        Prueba direcciones de haz ordenadas por cercania a haz_preferido y
        devuelve la primera segura. Resultado: dict o None.
        """
        boca = np.asarray(boca, dtype=float)
        dist = self.distancia if distancia is None else float(distancia)
        pref = unitario(haz_preferido)

        candidatos = []
        # base ortonormal alrededor de la direccion preferida
        aux = np.array([1.0, 0, 0]) if abs(pref[0]) < 0.9 else np.array([0, 1.0, 0])
        e1 = unitario(np.cross(pref, aux))
        e2 = np.cross(pref, e1)
        for cono in BUSQUEDA_CONO_DEG:
            azs = [0] if cono == 0 else range(0, 360, BUSQUEDA_AZIMUT_PASO_DEG)
            for az in azs:
                c, a = math.radians(cono), math.radians(az)
                haz = unitario(pref * math.cos(c) + math.sin(c) * (e1 * math.cos(a) + e2 * math.sin(a)))
                candidatos.append((cono, haz))

        semillas = self.semillas(boca)
        if q_actual is not None:
            semillas.insert(0, np.asarray(q_actual, dtype=float))

        for cono, haz in candidatos:
            lente_obj = boca - dist * haz
            if lente_obj[2] < Z_MIN_MM:
                continue
            mejor = None
            encontradas = 0
            for s in semillas:
                q, ep, ed = self.inversa_lente(lente_obj, haz, s)
                if ep < TOL_POS_MM * 2 and ed < TOL_DIR_DEG * 2 and not self.problemas(q):
                    esfuerzo = float(np.sum(np.abs(q - (q_actual if q_actual is not None else 0))))
                    if mejor is None or esfuerzo < mejor[0]:
                        mejor = (esfuerzo, q, ep, ed)
                    encontradas += 1
                    if encontradas >= SOLUCIONES_POR_HAZ:
                        break
            if mejor is not None:
                _, q, ep, ed = mejor
                return {"q": q, "pasos": self.a_pasos(q), "haz": haz,
                        "desvio_haz_deg": cono, "error_pos_mm": ep, "error_dir_deg": ed,
                        "directa": self.directa(q)}
        return None

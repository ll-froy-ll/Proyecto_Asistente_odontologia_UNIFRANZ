"""
MEDICION DEL BRAZO (Fase 0)  v7
--------------------------------
Guarda en config/cinematica.json lo que necesita la cinematica inversa.

  d        tramos del brazo (A, B, C)
  l        linterna: direccion del haz, posicion del lente, mancha y distancia
  p        prueba: hacia que lado se inclina el brazo con pasos positivos (Art4)
  m <art>  pasos por grado de una articulacion (1..5)
  c        posicion y orientacion de la camara
  a        marcador ArUco de la tabla (autocalibracion)
  v        ver datos + lo que falta
  g        guardar
  q        volver a HOME y salir

ARTICULACIONES (numeracion del proyecto)
  Art5  base          gira todo el brazo (la mas baja)
  Art4  hombro        adelante/atras, 2 x NEMA23
  Art3  codo          adelante/atras
  Art2  muneca_roll   gira el antebrazo
  Art1  muneca_pitch  inclina la linterna (la de arriba)

TRAMOS
  A  tabla de madera -> eje Art4
  B  eje Art4        -> eje Art3
  C  eje Art3        -> eje Art1

LINTERNA (medida desde el CENTRO del eje de la Art1, brazo en HOME)
  adelante   distancia horizontal hasta la cara del lente, en la direccion del haz
  arriba     altura del centro del lente sobre el eje (negativa si esta mas abajo)
  costado    corrimiento lateral del lente (+ izquierda mirando la flecha), 0 si centrado

SIGNOS (el script pregunta la direccion en cada movimiento)
  Art5, Art2                 + = ANTIHORARIO visto desde ARRIBA
  Art4, Art3, Art1           + = se inclina hacia el FRENTE (la flecha +x de la tabla)

Uso (raiz del proyecto Python, brazo en HOME, monitor ESP-IDF cerrado):
    python control_motores/medir_brazo_v7.py COM11
Si el brazo no esta conectado, ofrece seguir solo para cargar datos (d, l, c, a, v, g).
"""

import sys
import json
from pathlib import Path

from puente_serial import PuenteESP32, listar_puertos, configurar_logging

# ================== CONSTANTES AJUSTABLES ==================
PUERTO_DEF = "COM11"
ARCHIVO = Path("config/cinematica.json")

# Limites (deben coincidir con el firmware)
LIM_MIN = {1: -2100, 2: -200, 3: -36000, 4: -4500, 5: -5000}
LIM_MAX = {1:  2100, 2:  200, 3:  36000, 4:  4500, 5:  6000}

FRACCION_PRUEBA = 0.7        # se mide al 70 % del limite
FRACCION_IDENT = 0.2         # movimiento corto para identificar el eje
MICROPASOS_DEF = 3200        # DM542 en 1/16 -> 200 x 16
UMBRAL_DIF_PCT = 3.0         # aviso si los sentidos + y - difieren mas
UMBRAL_JUEGO_DEG = 1.0       # aviso si no vuelve a la lectura inicial
UMBRAL_GIRO_MIN_DEG = 5.0    # giros menores dan medidas poco precisas
UMBRAL_DIENTES_PCT = 3.0     # aviso si medido vs teorico difiere mas

# Referencia del Moveo original (URDF comunitario) para los tramos B y C
REF_MOVEO_MM = {"B_art4_art3": 221.1, "C_art3_art1": 223.0}
REF_TOLERANCIA_MM = 15.0

# Articulacion por defecto segun el numero (se acepta con Enter)
NOMBRE_POR_ART = {5: "base", 4: "hombro", 3: "codo", 2: "muneca_roll", 1: "muneca_pitch"}
ROTACIONES = ("base", "muneca_roll")

COMO_MEDIR = {
    "base": "Cinta milimetrada en la parte lisa del cilindro que gira (metodo mm) o transportador (g).",
    "muneca_roll": "Cinta milimetrada en la pieza que gira encima de la union (mm) o transportador con ranura (g).",
    "hombro": "Celular PARADO, espalda pegada a la cara lateral plana del tramo Art4->Art3. Metodo g.",
    "codo": "Celular PARADO, espalda pegada a la cara lateral del tramo Art3->Art1. Metodo g.",
    "muneca_pitch": "Celular apoyado sobre el cuerpo de la linterna. Metodo g.",
}

TRAMOS = {
    "A_tabla_art4": "A  tabla -> eje Art4 (mm)",
    "B_art4_art3": "B  eje Art4 -> eje Art3 (mm)",
    "C_art3_art1": "C  eje Art3 -> eje Art1 (mm)",
}
DIST_PRUEBA_MANCHA_MM = 300     # distancia a la pared para medir la mancha
FRACCION_PRUEBA_PLANO = 0.2     # movimiento de la prueba de lado (Art4)

# Valores por defecto del marcador ArUco (medidos el 21/09/2026). Enter los acepta.
MARCADOR_DEF = {"id": 0, "diccionario": "DICT_4X4_50", "lado_mm": 46.0,
                "x_mm": 150.0, "y_mm": -252.0, "z_mm": 14.0,
                "inclinacion_deg": 0.0, "giro_deg": 180.0}
# ============================================================


def pedir_float(msg, actual=None, permitir_vacio=False, positivo=False):
    extra = f" [{actual}]" if permitir_vacio else ""
    while True:
        txt = input(f"{msg}{extra}: ").strip().replace(",", ".")
        if not txt and permitir_vacio:
            return None
        try:
            v = float(txt)
            if positivo and v <= 0:
                print("  Debe ser mayor que 0.")
                continue
            return v
        except ValueError:
            print("  Numero invalido.")


def pedir_opcion(msg, opciones, defecto=None):
    d = f" [{defecto}]" if defecto else ""
    while True:
        r = input(f"{msg}{d}: ").strip().lower()
        if not r and defecto:
            return defecto
        if r in opciones:
            return r
        print(f"  Opciones: {opciones}")


def plantilla():
    return {
        "version": 7,
        "articulaciones": {},
        "tramos_mm": {k: None for k in TRAMOS},
        "desfases_x_mm": {"art4": 0.0, "art3": 0.0, "art1": 0.0},
        "linterna": {
            "direccion_haz_home": None,      # frente | atras | izquierda | derecha
            "inclinacion_haz_deg": None,     # 0 = horizontal, + hacia arriba
            "adelante_mm": None,
            "arriba_mm": None,
            "costado_mm": None,
            "mancha_largo_mm": None,         # medida con la pared a DIST_PRUEBA_MANCHA_MM
            "mancha_ancho_mm": None,
            "lado_largo_en_home": None,      # horizontal | vertical
            "distancia_trabajo_mm": None,
        },
        "plano_brazo": {
            "art4_positivo_inclina_hacia": None,   # izquierda | derecha | frente | atras
        },
        "camara": {
            "montaje": None,
            "x_mm": None, "y_mm": None, "z_mm": None,
            "inclinacion_deg": None,
            "giro_deg": None,
        },
        "marcador": {k: None for k in MARCADOR_DEF},
    }


def cargar():
    base = plantilla()
    if ARCHIVO.exists():
        datos = json.loads(ARCHIVO.read_text(encoding="utf-8"))
        if datos.get("version") in (4, 5, 6, 7):
            for k in base:
                if k in datos and k != "version":
                    if isinstance(base[k], dict) and isinstance(datos[k], dict) and k != "articulaciones":
                        base[k].update({kk: vv for kk, vv in datos[k].items() if kk in base[k]})
                    else:
                        base[k] = datos[k]
            # de v4/v5: el tramo D y el reflector ya no aplican (se cambio por linterna)
            base["tramos_mm"] = {k: v for k, v in base["tramos_mm"].items() if k in TRAMOS}
            base["desfases_x_mm"] = {k: v for k, v in base["desfases_x_mm"].items()
                                     if k in ("art4", "art3", "art1")}
        else:
            print("  (cinematica.json de una version anterior: se conservan solo las articulaciones)")
            base["articulaciones"] = datos.get("articulaciones", {})
    return base


def guardar(datos):
    ARCHIVO.parent.mkdir(parents=True, exist_ok=True)
    ARCHIVO.write_text(json.dumps(datos, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"  Guardado en {ARCHIVO.resolve()}")


def mover_eje(esp, art, pasos):
    pose = [0, 0, 0, 0, 0]
    pose[art - 1] = max(LIM_MIN[art], min(LIM_MAX[art], int(pasos)))
    ok = esp.mover(*pose)
    if not ok:
        print("  ! El ESP32 no confirmo el movimiento.")
    return ok, pose[art - 1]


# ======================= pasos por grado =======================
def leer_giro(metodo, inicial, perimetro):
    if metodo == "g":
        lect = pedir_float("    Lectura del celular/transportador AHORA (grados)")
        return abs(lect - inicial)
    lect = pedir_float("    Lectura de la cinta frente al puntero AHORA (mm)")
    return abs(lect - inicial) * 360.0 / perimetro


def pedir_signo(nombre):
    if nombre in ROTACIONES:
        r = pedir_opcion("    Visto desde ARRIBA giro: a=antihorario, h=horario", ("a", "h"))
        return +1 if r == "a" else -1
    r = pedir_opcion("    Se inclino hacia: f=frente, t=atras", ("f", "t"))
    return +1 if r == "f" else -1


def verificar_dientes(ppg_medido):
    if input("  Verificar con conteo de dientes? (s/n): ").strip().lower() != "s":
        return None
    micro = pedir_float("    Micropasos por vuelta del driver", MICROPASOS_DEF, permitir_vacio=True) or MICROPASOS_DEF
    zm = pedir_float("    Dientes de la polea del MOTOR", positivo=True)
    ze = pedir_float("    Dientes de la polea/rueda del EJE", positivo=True)
    caja = pedir_float("    Reduccion de caja del motor (1 si no tiene)", 1, permitir_vacio=True) or 1
    teorico = micro * (ze / zm) * caja / 360.0
    dif = abs(abs(ppg_medido) - teorico) / teorico * 100
    print(f"    Teorico {teorico:.3f} | medido {abs(ppg_medido):.3f} | diferencia {dif:.1f} %")
    if dif > UMBRAL_DIENTES_PCT:
        print("    ! No coinciden. Recuenta dientes o repite la medida.")
    return {"micropasos": micro, "dientes_motor": zm, "dientes_eje": ze,
            "caja": caja, "teorico": round(teorico, 4), "diferencia_pct": round(dif, 1)}


def medir_articulacion(esp, datos, art):
    if art not in LIM_MIN:
        print("  Articulacion 1..5")
        return
    print(f"\n=== Art{art} === Observa cual se mueve (ida y vuelta corta):")
    mover_eje(esp, art, LIM_MAX[art] * FRACCION_IDENT)
    mover_eje(esp, art, 0)

    nombre = pedir_opcion("  Que articulacion es?", tuple(COMO_MEDIR), NOMBRE_POR_ART[art])
    for k, v in datos["articulaciones"].items():
        if v["nombre"] == nombre and k != str(art):
            print(f"  ! '{nombre}' ya estaba asignado a Art{k}. Revisa.")

    print("  COMO MEDIR: " + COMO_MEDIR[nombre])
    metodo = "g" if nombre not in ROTACIONES else pedir_opcion(
        "  Metodo: g=grados (transportador), mm=cinta milimetrada", ("g", "mm"), "mm")
    perimetro = None
    if metodo == "mm":
        print("  Perimetro = 3.1416 x diametro de la franja donde pegaste la cinta.")
        perimetro = pedir_float("  Perimetro (mm)", positivo=True)

    input("  Coloca el instrumento, NO lo muevas mas, y presiona Enter...")
    unidad = "grados" if metodo == "g" else "mm"
    inicial = pedir_float(f"  Lectura INICIAL en HOME ({unidad})")

    medidas = []
    for lim in (LIM_MAX[art], LIM_MIN[art]):
        n = int(lim * FRACCION_PRUEBA)
        if n == 0:
            continue
        print(f"\n  Moviendo a {n:+d} pasos... espera que pare y 3 s mas.")
        ok, n_real = mover_eje(esp, art, n)
        if not ok:
            mover_eje(esp, art, 0)
            return
        grados = leer_giro(metodo, inicial, perimetro)
        if grados < 0.5:
            print("  ! Casi no giro: medida descartada.")
        else:
            signo = pedir_signo(nombre)
            ppg = n_real / (signo * grados)
            medidas.append(ppg)
            print(f"  -> giro {signo * grados:+.2f} grados = {ppg:+.3f} pasos/grado")
            if grados < UMBRAL_GIRO_MIN_DEG:
                print(f"  ! Menos de {UMBRAL_GIRO_MIN_DEG} grados: poco preciso, usa el conteo de dientes.")
        mover_eje(esp, art, 0)

    vuelta = pedir_float(f"\n  De vuelta en HOME, lectura ({unidad})")
    juego = abs(vuelta - inicial) if metodo == "g" else abs(vuelta - inicial) * 360.0 / perimetro

    if not medidas:
        print("  Sin medidas validas.")
        return
    if len(medidas) == 2 and (medidas[0] > 0) != (medidas[1] > 0):
        print("  ! Los sentidos dieron signos opuestos: revisa la direccion que indicaste. No se guarda.")
        return

    ppg = sum(medidas) / len(medidas)
    dif = (max(medidas) - min(medidas)) / abs(ppg) * 100 if len(medidas) > 1 else 0.0
    rango = sorted([LIM_MIN[art] / ppg, LIM_MAX[art] / ppg])
    registro = {
        "nombre": nombre,
        "pasos_por_grado": round(ppg, 4),
        "metodo": metodo,
        "perimetro_mm": perimetro,
        "min_pasos": LIM_MIN[art],
        "max_pasos": LIM_MAX[art],
        "recorrido_deg": [round(rango[0], 1), round(rango[1], 1)],
        "juego_deg": round(juego, 2),
        "diferencia_sentidos_pct": round(dif, 1),
    }
    print(f"\n  RESULTADO Art{art} ({nombre}): {ppg:+.3f} pasos/grado")
    print(f"  Recorrido desde HOME: {rango[0]:.1f} a {rango[1]:.1f} grados")
    if dif > UMBRAL_DIF_PCT:
        print(f"  ! Sentidos + y - difieren {dif:.1f} %. Repite con m {art}.")
    if juego > UMBRAL_JUEGO_DEG:
        print(f"  ! Juego de {juego:.1f} grados al volver. Revisa correa, polea y prisioneros.")
    registro["dientes"] = verificar_dientes(ppg)
    datos["articulaciones"][str(art)] = registro


# ======================= tramos =======================
def resumen_tramos(datos):
    t = datos["tramos_mm"]
    print("\n  Tramo                                   valor")
    for k, txt in TRAMOS.items():
        v = t[k]
        linea = f"  {txt:<45} {v if v is not None else '-'}"
        ref = REF_MOVEO_MM.get(k)
        if ref and v is not None:
            linea += f"   (Moveo {ref:.0f} mm"
            linea += ", OK)" if abs(v - ref) <= REF_TOLERANCIA_MM else ", ! REVISA)"
        print(linea)
    if all(v is not None for v in t.values()):
        print(f"  Altura del eje Art1 en HOME: {sum(t.values()):.0f} mm")


def cargar_tramos(datos):
    print("\n  Medidas de eje a eje. Enter = dejar el valor actual.")
    for k, txt in TRAMOS.items():
        v = pedir_float(f"  {txt}", datos["tramos_mm"][k], permitir_vacio=True)
        if v is not None:
            datos["tramos_mm"][k] = v

    if input("\n  Cargar desfases hacia adelante/atras de los ejes? (s/n): ").strip().lower() == "s":
        print("  x = 80 - d  (80 = radio de la base, d = distancia desde la barra vertical)")
        for k in datos["desfases_x_mm"]:
            v = pedir_float(f"    desfase x de {k} (mm)", datos["desfases_x_mm"][k], permitir_vacio=True)
            if v is not None:
                datos["desfases_x_mm"][k] = v
    resumen_tramos(datos)


# ======================= linterna =======================
def cargar_linterna(datos):
    li = datos["linterna"]
    print("\n  LINTERNA. Brazo en HOME. Medidas desde el CENTRO del eje de la Art1 (el rodamiento).")
    print("  Enter = dejar el valor actual.")
    print("  Direccion: parado detras del brazo MIRANDO HACIA LA FLECHA +x.")
    li["direccion_haz_home"] = pedir_opcion(
        "  Hacia donde apunta el lente?", ("frente", "atras", "izquierda", "derecha"),
        li["direccion_haz_home"])
    campos = [
        ("inclinacion_haz_deg", "Inclinacion del haz (grados, 0 = horizontal, + hacia arriba)"),
        ("arriba_mm", "ARRIBA: del centro del rodamiento Art1 a la linea central de la linterna (mm)"),
        ("adelante_mm", "ADELANTE: de la vertical del rodamiento a la cara del lente (mm)"),
        ("costado_mm", "COSTADO: corrimiento lateral de la linterna (mm, 0 si esta centrada)"),
        ("mancha_largo_mm", f"Lado LARGO de la mancha a {DIST_PRUEBA_MANCHA_MM} mm de la pared (mm)"),
        ("mancha_ancho_mm", f"Lado CORTO de la mancha a {DIST_PRUEBA_MANCHA_MM} mm de la pared (mm)"),
    ]
    for k, txt in campos:
        v = pedir_float(f"  {txt}", li[k], permitir_vacio=True)
        if v is not None:
            li[k] = v
    li["lado_largo_en_home"] = pedir_opcion(
        "  En la pared, el lado LARGO de la mancha queda", ("horizontal", "vertical"),
        li["lado_largo_en_home"])
    v = pedir_float("  Distancia de trabajo linterna-boca (mm)", li["distancia_trabajo_mm"] or 300,
                    permitir_vacio=True)
    li["distancia_trabajo_mm"] = v if v is not None else (li["distancia_trabajo_mm"] or 300.0)
    print(f"  Linterna: lente hacia {li['direccion_haz_home']}, "
          f"arriba {li['arriba_mm']} mm, adelante {li['adelante_mm']} mm, costado {li['costado_mm']} mm")


def prueba_plano(esp, datos):
    """Mueve Art4 un poco en positivo para saber hacia que lado se inclina el brazo."""
    n = int(LIM_MAX[4] * FRACCION_PRUEBA_PLANO)
    print(f"\n  Parate DETRAS del brazo mirando hacia la flecha +x.")
    input(f"  Voy a mover Art4 a +{n} pasos y volver. Enter para empezar...")
    mover_eje(esp, 4, n)
    r = pedir_opcion("  Hacia donde se inclino el brazo?",
                     ("izquierda", "derecha", "frente", "atras"))
    mover_eje(esp, 4, 0)
    datos["plano_brazo"]["art4_positivo_inclina_hacia"] = r
    print(f"  Anotado: Art4 positivo inclina hacia {r}.")


# ======================= camara =======================
def cargar_camara(datos):
    cam = datos["camara"]
    cam["montaje"] = pedir_opcion("  Montaje", ("fija", "brazo"), cam["montaje"] or "fija")
    print("  Centro del LENTE. Origen: punto de la tabla bajo el eje de la base.")
    print("  +x = flecha de la tabla | +y = izquierda mirando hacia +x | +z = arriba")
    for k in ("x_mm", "y_mm", "z_mm"):
        v = pedir_float(f"    {k}", cam[k], permitir_vacio=True)
        if v is not None:
            cam[k] = v
    v = pedir_float("    Inclinacion (grados; negativo = mira hacia abajo)", cam["inclinacion_deg"], permitir_vacio=True)
    if v is not None:
        cam["inclinacion_deg"] = v
    v = pedir_float("    Giro visto desde arriba (0 = mira hacia +x; 180 = mira hacia el brazo)",
                    cam["giro_deg"], permitir_vacio=True)
    if v is not None:
        cam["giro_deg"] = v


# ======================= marcador ArUco =======================
def cargar_marcador(datos):
    mk = datos["marcador"]
    print("\n  Marcador ArUco de la tabla. Enter = aceptar el valor entre corchetes.")
    print("  Posicion del CENTRO del cuadrado negro, mismo origen que el brazo.")
    etiquetas = {
        "lado_mm": "Lado del cuadrado NEGRO (mm)",
        "x_mm": "x del centro (mm, + hacia la flecha)",
        "y_mm": "y del centro (mm, + a la izquierda mirando la flecha)",
        "z_mm": "z del centro sobre la tabla (mm)",
        "inclinacion_deg": "Inclinacion (grados, 0 = acostado mirando al techo)",
        "giro_deg": "Giro visto desde arriba respecto de la flecha (grados)",
    }
    mk["id"] = MARCADOR_DEF["id"]
    mk["diccionario"] = MARCADOR_DEF["diccionario"]
    for k, txt in etiquetas.items():
        actual = mk[k] if mk[k] is not None else MARCADOR_DEF[k]
        v = pedir_float(f"    {txt}", actual, permitir_vacio=True)
        mk[k] = v if v is not None else actual
    print(f"  Marcador: ID {mk['id']}, lado {mk['lado_mm']} mm, "
          f"centro ({mk['x_mm']}, {mk['y_mm']}, {mk['z_mm']}) mm, giro {mk['giro_deg']}")


def faltantes(datos):
    f = []
    f += [f"Art{a} ({NOMBRE_POR_ART[a]})" for a in (5, 4, 3, 2, 1) if str(a) not in datos["articulaciones"]]
    f += [f"tramo {k}" for k, v in datos["tramos_mm"].items() if v is None]
    f += [f"linterna.{k}" for k, v in datos["linterna"].items() if v is None]
    f += [f"plano_brazo.{k} (comando p)" for k, v in datos["plano_brazo"].items() if v is None]
    f += [f"camara.{k}" for k, v in datos["camara"].items() if v is None]
    f += [f"marcador.{k}" for k, v in datos["marcador"].items() if v is None]
    return f


def main():
    log = configurar_logging()
    puerto = sys.argv[1] if len(sys.argv) > 1 else PUERTO_DEF
    print("Puertos disponibles:")
    for p in listar_puertos():
        print("  ", p)

    esp = PuenteESP32(puerto, log=log)
    if not esp.conectar():
        print("No se pudo conectar con el brazo.")
        r = input("Seguir SOLO para cargar datos (d, l, c, a, v, g)? (s/n): ").strip().lower()
        if r != "s":
            return
        esp = None

    datos = cargar()
    if esp is not None:
        esp.mover(0, 0, 0, 0, 0)
    print(__doc__)

    try:
        while True:
            txt = input("medir> ").strip().lower()
            if not txt:
                continue
            p = txt.split()
            if p[0] == "q":
                break
            elif p[0] == "m" and len(p) == 2 and p[1].isdigit():
                if esp is None:
                    print("  Sin brazo conectado: 'm' no esta disponible.")
                else:
                    medir_articulacion(esp, datos, int(p[1]))
            elif p[0] == "a":
                cargar_marcador(datos)
            elif p[0] == "l":
                cargar_linterna(datos)
            elif p[0] == "p":
                if esp is None:
                    print("  Sin brazo conectado: 'p' no esta disponible.")
                else:
                    prueba_plano(esp, datos)
            elif p[0] == "d":
                cargar_tramos(datos)
            elif p[0] == "c":
                cargar_camara(datos)
            elif p[0] == "v":
                print(json.dumps(datos, indent=2, ensure_ascii=False))
                resumen_tramos(datos)
                f = faltantes(datos)
                print("  FALTA: " + ", ".join(f) if f else "  COMPLETO")
            elif p[0] == "g":
                guardar(datos)
            else:
                print("  Comandos: m <art> | d | l | p | c | a | v | g | q")
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
        try:
            if input("Guardar antes de salir? (s/n): ").strip().lower() == "s":
                guardar(datos)
        except (KeyboardInterrupt, EOFError):
            print("\nNo se guardo.")


if __name__ == "__main__":
    main()

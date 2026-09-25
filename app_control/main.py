import csv
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
from PySide6.QtCore import Qt, QThread, Signal, QSize, QPropertyAnimation, QEasingCurve
from PySide6.QtGui import QImage, QPixmap, QIcon, QFont
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QGridLayout, QStackedWidget, QComboBox, QLineEdit, QCheckBox,
    QFrame, QSpinBox, QDoubleSpinBox, QMessageBox, QSizePolicy, QTextEdit,
)

# --- modulos del proyecto (raiz) ---
from vision_artificial.detectar_boca import DetectorBoca
from control_motores.puente_serial import PuenteESP32, listar_puertos
from seguir_boca import SeguidorBrazo, interpolar, configurar_logging

# --- modulos del paquete ---
from . import db
from .estilos import qss, COLOR_ICONO

try:
    import qtawesome as qta
    _HAY_QTA = True
except Exception:
    _HAY_QTA = False


def icono(nombre, color):
    if _HAY_QTA:
        try:
            return qta.icon(nombre, color=color)
        except Exception:
            return QIcon()
    return QIcon()


# --- constantes ---
LIM_MIN = {1: -2100, 2: -200, 3: -36000, 4: -4500, 5: -5000}
LIM_MAX = {1:  2100, 2:  200, 3:  36000, 4:  4500, 5:  6000}
ZONA_DEF = {"zx1": 0.25, "zy1": 0.30, "zx2": 0.75, "zy2": 0.85}
ARCHIVO_CALIB = Path("config/calibracion.json")
ARCHIVO_CSV = Path("logs/sesiones.csv")

AYUDAS = {
    "com": "Puerto serial donde esta conectado el brazo (el ESP32). En Windows suele ser COM seguido de un numero, por ejemplo COM11.",
    "camara": "Indice de la camara. 0 es la camara principal de la laptop. Si tenes varias camaras conectadas, proba 1 o 2.",
    "deadband": "Zona muerta: cuanto se tiene que mover la boca para que el brazo corrija. Mas alto = el brazo se mueve menos seguido (no tiembla). Mas bajo = mas sensible.",
    "frames": "Cuadros seguidos que confirma antes de mover. Mas alto = mas estable pero reacciona un poco mas lento. Mas bajo = reacciona mas rapido.",
}


def clamp(art, val):
    return max(LIM_MIN[art], min(LIM_MAX[art], val))


# ===================== WORKERS (controladores) =====================
class ConectarWorker(QThread):
    listo = Signal(bool, str)

    def __init__(self, esp):
        super().__init__()
        self.esp = esp

    def run(self):
        try:
            ok = self.esp.conectar()
            self.listo.emit(ok, "" if ok else "No respondio en modo autonomo.")
        except Exception as e:
            self.listo.emit(False, str(e))


class MoverWorker(QThread):
    listo = Signal(bool)

    def __init__(self, esp, pose):
        super().__init__()
        self.esp = esp
        self.pose = list(pose)

    def run(self):
        try:
            self.listo.emit(self.esp.mover(*self.pose))
        except Exception:
            self.listo.emit(False)


class SecuenciaWorker(QThread):
    log = Signal(str)
    listo = Signal()

    def __init__(self, esp, cual):
        super().__init__()
        self.esp = esp
        self.cual = cual

    def run(self):
        try:
            if self.cual == 1:
                for i in range(5):
                    self.log.emit(f"Articulacion {i + 1}: recorriendo rango...")
                    p = [0, 0, 0, 0, 0]
                    p[i] = LIM_MAX[i + 1]; self.esp.mover(*p)
                    p[i] = LIM_MIN[i + 1]; self.esp.mover(*p)
                    p[i] = 0; self.esp.mover(*p)
                    self.log.emit(f"Articulacion {i + 1}: OK")
            else:
                self.log.emit("Todas a limite positivo...")
                self.esp.mover(*[LIM_MAX[i] for i in range(1, 6)])
                self.log.emit("Todas a limite negativo...")
                self.esp.mover(*[LIM_MIN[i] for i in range(1, 6)])
                self.log.emit("Regreso a HOME...")
                self.esp.mover(0, 0, 0, 0, 0)
                self.log.emit("Secuencia 2: OK")
        except Exception as e:
            self.log.emit(f"Error: {e}")
        self.listo.emit()


class CamaraWorker(QThread):
    frame_listo = Signal(QImage)
    telemetria = Signal(dict)

    def __init__(self, zona, poses, seguidor, deadband, frames_confirm, cam_index, log):
        super().__init__()
        self.zona = zona
        self.poses = poses
        self.seguidor = seguidor
        self.deadband = deadband
        self.frames_confirm = frames_confirm
        self.cam_index = cam_index
        self.log = log
        self.tracking = True
        self._corriendo = True
        self.m = {"frames": 0, "correcciones": 0, "perdidas": 0, "ciego": 0.0}
        self.t_inicio = time.time()

    def run(self):
        detector = DetectorBoca(zona=self.zona, log=self.log)
        cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(self.cam_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        uv_com = [0.5, 0.5]
        confirm = 0
        t_prev = time.time()
        fps = 0.0
        det_prev = False
        self.t_inicio = time.time()

        while self._corriendo:
            ok, frame = cap.read()
            if not ok:
                break
            ahora = time.time()
            dt = ahora - t_prev
            t_prev = ahora
            self.m["frames"] += 1
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            frame = cv2.flip(frame, 1)
            salida = detector.procesar(frame)

            if salida["detectada"]:
                u, v = salida["u"], salida["v"]
                if abs(u - uv_com[0]) > self.deadband or abs(v - uv_com[1]) > self.deadband:
                    confirm += 1
                    if confirm >= self.frames_confirm:
                        if self.tracking and self.seguidor is not None:
                            self.seguidor.set_objetivo(interpolar(u, v, self.poses))
                        uv_com = [u, v]
                        confirm = 0
                        self.m["correcciones"] += 1
                        estado = "SIGUIENDO (mov)"
                    else:
                        estado = "SIGUIENDO"
                else:
                    confirm = 0
                    estado = "EN ZONA"
                det_prev = True
            else:
                self.m["ciego"] += dt
                if det_prev:
                    self.m["perdidas"] += 1
                det_prev = False
                confirm = 0
                estado = "SIN DETECCION"

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
            self.frame_listo.emit(qimg)
            self.telemetria.emit({"estado": estado, "detectada": salida["detectada"],
                                  "fps": fps, "tracking": self.tracking, **self.m})

        cap.release()
        detector.cerrar()

    def detener(self):
        self._corriendo = False
        self.wait(3000)


# ===================== WIDGETS AUX =====================
class Indicador(QWidget):
    def __init__(self, texto):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        self.dot = QLabel()
        self.dot.setFixedSize(14, 14)
        self.lbl = QLabel(texto)
        lay.addWidget(self.dot)
        lay.addWidget(self.lbl)
        lay.addStretch()
        self.set_estado(False)

    def set_estado(self, ok, color=None):
        c = color or ("#22C55E" if ok else "#94A3B8")
        self.dot.setStyleSheet(f"background:{c}; border-radius:7px;")


def card(widget):
    f = QFrame()
    f.setObjectName("Card")
    lay = QVBoxLayout(f)
    lay.setContentsMargins(18, 18, 18, 18)
    lay.addWidget(widget)
    return f


# ===================== VENTANA PRINCIPAL =====================
class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Asistente Odontologico")
        self.log = configurar_logging()
        self.ajustes = db.cargar()
        self.tema = self.ajustes.get("tema", "claro")

        self.esp = None
        self.conectado = False
        self.home_confirmado = False
        self.camara = None
        self.seguidor = None
        self.tracking_activo = False
        self._cal_moviendo = False
        self.cal_pose = [0, 0, 0, 0, 0]
        self.cal_guardadas = {}
        self._workers = []
        self._nav_iconos = []

        raiz = QWidget()
        self.setCentralWidget(raiz)
        vraiz = QVBoxLayout(raiz)
        vraiz.setContentsMargins(0, 0, 0, 0)
        vraiz.setSpacing(0)
        vraiz.addWidget(self._topbar())

        medio = QWidget()
        hm = QHBoxLayout(medio)
        hm.setContentsMargins(0, 0, 0, 0)
        hm.setSpacing(0)
        hm.addWidget(self._sidebar())
        self.stack = QStackedWidget()
        hm.addWidget(self.stack, 1)
        vraiz.addWidget(medio, 1)

        self.stack.addWidget(self._pagina_inicio())        # 0
        self.stack.addWidget(self._pagina_calib_zona())    # 1
        self.stack.addWidget(self._pagina_calib_home())    # 2
        self.stack.addWidget(self._pagina_test())          # 3
        self.stack.addWidget(self._pagina_usar())          # 4
        self.stack.addWidget(self._pagina_ajustes())       # 5

        self._aplicar_tema()
        self._ir(0)
        self._refrescar_estado()
        # auto-conectar al iniciar
        self._conectar(silencioso=True)

    # ---------- TOP BAR ----------
    def _topbar(self):
        bar = QFrame()
        bar.setObjectName("TopBar")
        bar.setFixedHeight(56)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(12, 6, 16, 6)
        self.btn_burger = QPushButton()
        self.btn_burger.setObjectName("Icono")
        self.btn_burger.setIconSize(QSize(22, 22))
        self.btn_burger.clicked.connect(self._toggle_sidebar)
        lay.addWidget(self.btn_burger)
        titulo = QLabel("  Asistente Odontologico")
        titulo.setObjectName("SecTit")
        lay.addWidget(titulo)
        lay.addStretch()
        self.btn_tema = QPushButton()
        self.btn_tema.setObjectName("Icono")
        self.btn_tema.setIconSize(QSize(22, 22))
        self.btn_tema.clicked.connect(self._toggle_tema)
        lay.addWidget(self.btn_tema)
        return bar

    # ---------- SIDEBAR ----------
    def _sidebar(self):
        self.sidebar = QFrame()
        self.sidebar.setObjectName("Sidebar")
        self.sidebar.setMinimumWidth(0)
        self.sidebar.setMaximumWidth(240)
        lay = QVBoxLayout(self.sidebar)
        lay.setContentsMargins(14, 18, 14, 14)
        lay.setSpacing(5)

        logo = QLabel("Asistente")
        logo.setObjectName("Logo")
        sub = QLabel("Odontologico")
        sub.setObjectName("LogoSub")
        lay.addWidget(logo)
        lay.addWidget(sub)
        lay.addSpacing(18)

        self.nav_botones = []
        items = [
            ("Inicio", "fa5s.home", 0),
            ("Calibrar zona", "fa5s.bullseye", 1),
            ("Calibrar HOME", "fa5s.crosshairs", 2),
            ("Test", "fa5s.vial", 3),
            ("Usar Sistema", "fa5s.video", 4),
            ("Ajustes", "fa5s.cog", 5),
        ]
        for texto, ic, idx in items:
            b = QPushButton("  " + texto)
            b.setProperty("nav", True)
            b.setCheckable(True)
            b.setIconSize(QSize(18, 18))
            b.clicked.connect(lambda _=False, i=idx: self._ir(i))
            lay.addWidget(b)
            self.nav_botones.append(b)
            self._nav_iconos.append((b, ic))

        lay.addStretch()
        self.ind_lateral = Indicador("Desconectado")
        lay.addWidget(self.ind_lateral)
        return self.sidebar

    def _toggle_sidebar(self):
        objetivo = 0 if self.sidebar.maximumWidth() > 10 else 240
        self.anim = QPropertyAnimation(self.sidebar, b"maximumWidth")
        self.anim.setDuration(220)
        self.anim.setEasingCurve(QEasingCurve.InOutCubic)
        self.anim.setStartValue(self.sidebar.maximumWidth())
        self.anim.setEndValue(objetivo)
        self.anim.start()

    def _ir(self, idx):
        if self.tracking_activo and idx != 4:
            return
        self.stack.setCurrentIndex(idx)
        for i, b in enumerate(self.nav_botones):
            b.setChecked(i == idx)
        self._refrescar_iconos()

    def _bloquear_nav(self, bloquear):
        for i, b in enumerate(self.nav_botones):
            b.setEnabled(not bloquear or i == 4)

    # ---------- TEMA / ICONOS ----------
    def _toggle_tema(self):
        self.tema = "oscuro" if self.tema == "claro" else "claro"
        db.set_valor("tema", self.tema)
        self._aplicar_tema()

    def _aplicar_tema(self):
        QApplication.instance().setStyleSheet(qss(self.tema))
        self._refrescar_iconos()

    def _refrescar_iconos(self):
        col = COLOR_ICONO[self.tema]
        for b, nom in self._nav_iconos:
            b.setIcon(icono(nom, "white" if b.isChecked() else col))
        self.btn_burger.setIcon(icono("fa5s.bars", col))
        self.btn_tema.setIcon(icono("fa5s.moon" if self.tema == "claro" else "fa5s.sun", col))

    # ---------- INICIO ----------
    def _pagina_inicio(self):
        pag = QWidget()
        lay = QVBoxLayout(pag)
        lay.setContentsMargins(34, 28, 34, 28)
        lay.setSpacing(16)

        tit = QLabel("Panel principal")
        tit.setObjectName("Titulo")
        lay.addWidget(tit)
        lay.addWidget(self._sub("Conecta el brazo, confirma el HOME e inicia el sistema."))

        conx = QWidget()
        cl = QHBoxLayout(conx)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.addWidget(QLabel("Puerto:"))
        self.combo_com = QComboBox()
        self._poblar_puertos()
        cl.addWidget(self.combo_com)
        b_ref = QPushButton("Actualizar")
        b_ref.clicked.connect(self._poblar_puertos)
        cl.addWidget(b_ref)
        self.btn_conectar = QPushButton("Conectar")
        self.btn_conectar.setObjectName("Primario")
        self.btn_conectar.clicked.connect(lambda: self._conectar(silencioso=False))
        cl.addWidget(self.btn_conectar)
        self.lbl_conx = QLabel("")
        self.lbl_conx.setObjectName("Sub")
        cl.addWidget(self.lbl_conx)
        cl.addStretch()
        lay.addWidget(card(conx))

        inds = QWidget()
        il = QHBoxLayout(inds)
        il.setContentsMargins(0, 0, 0, 0)
        self.ind_conx = Indicador("Conexion")
        self.ind_calib = Indicador("Calibracion")
        self.ind_home = Indicador("HOME confirmado")
        for w in (self.ind_conx, self.ind_calib, self.ind_home):
            il.addWidget(card(w), 1)
        lay.addWidget(inds)

        chw = QWidget()
        chl = QVBoxLayout(chw)
        chl.setContentsMargins(0, 0, 0, 0)
        t = QLabel("Antes de usar el sistema")
        t.setObjectName("SecTit")
        chl.addWidget(t)
        chl.addWidget(QLabel("Verifica que el brazo este en la marca de HOME (pestaña Calibrar HOME)."))
        self.chk_home = QCheckBox("Confirmo HOME correctamente posicionado")
        self.chk_home.toggled.connect(self._toggle_home)
        chl.addWidget(self.chk_home)
        lay.addWidget(card(chw))

        self.btn_iniciar = QPushButton("INICIAR SISTEMA")
        self.btn_iniciar.setObjectName("Iniciar")
        self.btn_iniciar.setIconSize(QSize(30, 30))
        self.btn_iniciar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.btn_iniciar.clicked.connect(self._iniciar_sistema)
        lay.addWidget(self.btn_iniciar, 1)
        return pag

    def _sub(self, t):
        l = QLabel(t)
        l.setObjectName("Sub")
        return l

    def _poblar_puertos(self):
        self.combo_com.clear()
        puertos = listar_puertos()
        if puertos:
            self.combo_com.addItems(puertos)
        defecto = self.ajustes.get("com", "COM11")
        for i in range(self.combo_com.count()):
            if defecto in self.combo_com.itemText(i):
                self.combo_com.setCurrentIndex(i)
                return
        if not puertos:
            self.combo_com.addItem(defecto)

    def _puerto_elegido(self):
        t = self.combo_com.currentText()
        return t.split(" ")[0] if t else self.ajustes.get("com", "COM11")

    def _conectar(self, silencioso=False):
        if self.conectado:
            return
        puerto = self._puerto_elegido()
        self.btn_conectar.setEnabled(False)
        self.btn_conectar.setText("Conectando...")
        self.lbl_conx.setText("Conectando...")
        self.esp = PuenteESP32(puerto, log=self.log)
        w = ConectarWorker(self.esp)
        w.listo.connect(lambda ok, msg, s=silencioso: self._tras_conectar(ok, msg, s))
        self._workers.append(w)
        w.start()

    def _tras_conectar(self, ok, msg, silencioso):
        self.conectado = ok
        self.btn_conectar.setEnabled(not ok)
        self.btn_conectar.setText("Conectado" if ok else "Conectar")
        self.lbl_conx.setText("Conectado" if ok else "Sin conexion")
        if not ok and not silencioso:
            QMessageBox.warning(
                self, "Conexion",
                "No pude abrir el puerto.\n\n"
                "Causa mas comun: el monitor de ESP-IDF esta abierto (cerralo con Ctrl+]).\n"
                "Tambien verifica que el ESP32 este enchufado y el puerto sea el correcto.\n\n"
                f"Detalle: {msg}")
        self._refrescar_estado()

    def _toggle_home(self, val):
        self.home_confirmado = val
        self._refrescar_estado()

    def _refrescar_estado(self):
        calib_ok = ARCHIVO_CALIB.exists()
        self.ind_conx.set_estado(self.conectado)
        self.ind_calib.set_estado(calib_ok)
        self.ind_home.set_estado(self.home_confirmado)
        self.ind_lateral.set_estado(self.conectado)
        self.ind_lateral.lbl.setText("Conectado" if self.conectado else "Desconectado")
        listo = self.conectado and self.home_confirmado and calib_ok
        self.btn_iniciar.setEnabled(listo)
        if not self.conectado:
            self.btn_iniciar.setText("INICIAR SISTEMA\n(conecta primero)")
        elif not calib_ok:
            self.btn_iniciar.setText("INICIAR SISTEMA\n(falta calibrar la zona)")
        elif not self.home_confirmado:
            self.btn_iniciar.setText("INICIAR SISTEMA\n(confirma HOME)")
        else:
            self.btn_iniciar.setText("INICIAR SISTEMA")

    # ---------- USAR SISTEMA ----------
    def _pagina_usar(self):
        pag = QWidget()
        lay = QHBoxLayout(pag)
        lay.setContentsMargins(22, 22, 22, 22)
        lay.setSpacing(18)

        self.cam_label = QLabel("Camara")
        self.cam_label.setObjectName("CamView")
        self.cam_label.setAlignment(Qt.AlignCenter)
        self.cam_label.setMinimumSize(640, 460)
        self.cam_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lay.addWidget(self.cam_label, 3)

        panel = QWidget()
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(0, 0, 0, 0)
        pl.setSpacing(12)
        t = QLabel("Telemetria")
        t.setObjectName("Titulo")
        pl.addWidget(t)

        self.tel_estado = QLabel("—")
        self.tel_estado.setObjectName("Tel")
        pl.addWidget(self._tel("Estado", self.tel_estado))

        fila = QWidget()
        fl = QHBoxLayout(fila)
        fl.setContentsMargins(0, 0, 0, 0)
        self.tel_fps = QLabel("0")
        self.tel_fps.setObjectName("Tel")
        self.tel_corr = QLabel("0")
        self.tel_corr.setObjectName("Tel")
        fl.addWidget(self._tel("FPS", self.tel_fps), 1)
        fl.addWidget(self._tel("Correcciones", self.tel_corr), 1)
        pl.addWidget(fila)

        self.tel_perd = QLabel("0")
        self.tel_perd.setObjectName("Tel")
        pl.addWidget(self._tel("Perdidas de deteccion", self.tel_perd))
        pl.addStretch()

        self.btn_estop = QPushButton("  PARO DE EMERGENCIA")
        self.btn_estop.setObjectName("Estop")
        self.btn_estop.setIconSize(QSize(22, 22))
        self.btn_estop.clicked.connect(self._toggle_estop)
        pl.addWidget(self.btn_estop)

        self.btn_detener = QPushButton("Detener y volver a HOME")
        self.btn_detener.setObjectName("Primario")
        self.btn_detener.clicked.connect(self._detener_sistema)
        pl.addWidget(self.btn_detener)

        panel.setFixedWidth(320)
        lay.addWidget(panel)
        return pag

    def _tel(self, titulo, valor):
        w = QWidget()
        l = QVBoxLayout(w)
        l.setContentsMargins(0, 0, 0, 0)
        l.setSpacing(2)
        lt = QLabel(titulo)
        lt.setObjectName("TelLbl")
        l.addWidget(lt)
        l.addWidget(valor)
        return card(w)

    def _iniciar_sistema(self):
        try:
            datos = json.loads(ARCHIVO_CALIB.read_text(encoding="utf-8"))
        except Exception:
            self._aviso("No pude leer la calibracion.")
            return
        self.poses = datos["poses"]
        z = datos.get("zona", ZONA_DEF)
        self.zona_t = (z["zx1"], z["zy1"], z["zx2"], z["zy2"])
        centro = self.poses["CENTRO"]

        self.tracking_activo = True
        self._bloquear_nav(True)
        self._ir(4)
        self.tel_estado.setText("Yendo a la zona...")
        self.cam_label.setText("Moviendo el brazo a la zona de trabajo...")
        self.btn_detener.setEnabled(False)
        self.btn_estop_actualizar(True)

        w = MoverWorker(self.esp, centro)
        w.listo.connect(self._tras_centro)
        self._workers.append(w)
        w.start()

    def _tras_centro(self, ok):
        self.seguidor = SeguidorBrazo(self.esp, self.log)
        self.seguidor.iniciar()
        self.camara = CamaraWorker(self.zona_t, self.poses, self.seguidor,
                                   float(self.ajustes["deadband"]), int(self.ajustes["frames"]),
                                   int(self.ajustes["camara"]), self.log)
        self.camara.frame_listo.connect(self._mostrar_frame)
        self.camara.telemetria.connect(self._telemetria)
        self.camara.start()
        self.btn_detener.setEnabled(True)

    def _mostrar_frame(self, qimg):
        pix = QPixmap.fromImage(qimg)
        self.cam_label.setPixmap(pix.scaled(self.cam_label.size(),
                                            Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def _telemetria(self, t):
        self.tel_estado.setText(t["estado"] if t["tracking"] else "PARO ACTIVO")
        color = "#0F766E" if (t["detectada"] and t["tracking"]) else "#DC2626"
        self.tel_estado.setStyleSheet(f"color:{color}; font-size:22px; font-weight:700;")
        self.tel_fps.setText(f"{t['fps']:.0f}")
        self.tel_corr.setText(str(t["correcciones"]))
        self.tel_perd.setText(str(t["perdidas"]))

    def btn_estop_actualizar(self, activo):
        col = COLOR_ICONO[self.tema]
        if activo:
            self.btn_estop.setText("  PARO DE EMERGENCIA")
            self.btn_estop.setIcon(icono("fa5s.hand-paper", "white"))
        else:
            self.btn_estop.setText("  REANUDAR")
            self.btn_estop.setIcon(icono("fa5s.play", "white"))

    def _toggle_estop(self):
        if self.camara is None:
            return
        self.camara.tracking = not self.camara.tracking
        self.btn_estop_actualizar(self.camara.tracking)

    def _detener_sistema(self):
        m = dict(self.camara.m) if self.camara else None
        t0 = self.camara.t_inicio if self.camara else time.time()
        if self.camara:
            self.camara.detener()
            self.camara = None
        if self.seguidor:
            self.seguidor.detener()
            self.seguidor = None
        self.btn_detener.setEnabled(False)
        self.cam_label.setText("Volviendo a HOME...")
        w = MoverWorker(self.esp, [0, 0, 0, 0, 0])
        w.listo.connect(lambda ok, m=m, t0=t0: self._tras_detener(m, t0))
        self._workers.append(w)
        w.start()

    def _tras_detener(self, m, t0):
        self.tracking_activo = False
        self._bloquear_nav(False)
        self.cam_label.setPixmap(QPixmap())
        self.cam_label.setText("Sistema detenido")
        if m:
            self._guardar_resumen(t0, m)
        self._ir(0)

    def _guardar_resumen(self, t0, m):
        dur = max(0.001, time.time() - t0)
        fps = m["frames"] / dur
        pct = 100.0 * (dur - m["ciego"]) / dur
        mm, ss = divmod(int(dur), 60)
        try:
            ARCHIVO_CSV.parent.mkdir(exist_ok=True)
            nuevo = not ARCHIVO_CSV.exists()
            with ARCHIVO_CSV.open("a", newline="", encoding="utf-8") as f:
                wr = csv.writer(f)
                if nuevo:
                    wr.writerow(["fecha_hora", "duracion_s", "correcciones",
                                 "perdidas_deteccion", "tiempo_ciego_s",
                                 "pct_detectada", "fps_promedio"])
                wr.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                             round(dur, 1), m["correcciones"], m["perdidas"],
                             round(m["ciego"], 1), round(pct), round(fps)])
        except Exception as e:
            self.log.error("CSV: %s", e)
        QMessageBox.information(
            self, "Resumen de sesion",
            f"Duracion: {mm} min {ss} s\n"
            f"Correcciones automaticas: {m['correcciones']}\n"
            f"Perdidas de deteccion: {m['perdidas']}\n"
            f"Tiempo detectada: {pct:.0f}%\n"
            f"FPS promedio: {fps:.0f}\n\nGuardado en {ARCHIVO_CSV}")

    # ---------- CALIBRAR ZONA ----------
    def _pagina_calib_zona(self):
        pag = QWidget()
        lay = QVBoxLayout(pag)
        lay.setContentsMargins(34, 28, 34, 28)
        lay.setSpacing(14)
        tit = QLabel("Calibracion de zona de trabajo")
        tit.setObjectName("Titulo")
        lay.addWidget(tit)
        lay.addWidget(self._sub("Movie la lampara, guarda las 4 esquinas y el centro, y guarda la calibracion."))

        pw = QWidget()
        ph = QHBoxLayout(pw)
        ph.setContentsMargins(0, 0, 0, 0)
        ph.addWidget(QLabel("Paso:"))
        self.cal_paso = QComboBox()
        self.cal_paso.addItems(["10", "50", "200", "1000", "3000"])
        self.cal_paso.setCurrentText("200")
        ph.addWidget(self.cal_paso)
        ph.addStretch()
        b_home = QPushButton("Ir a HOME")
        b_home.clicked.connect(self._cal_home)
        ph.addWidget(b_home)
        lay.addWidget(pw)

        self.cal_valores = []
        grid = QGridLayout()
        grid.setSpacing(10)
        for i, nom in enumerate(["Art1", "Art2", "Art3", "Art4", "Art5"]):
            grid.addWidget(QLabel(nom), i, 0)
            bm = QPushButton("−")
            bm.setObjectName("Jog")
            bm.clicked.connect(lambda _=False, a=i + 1: self._jog(a, -1))
            grid.addWidget(bm, i, 1)
            val = QLabel("0")
            val.setAlignment(Qt.AlignCenter)
            val.setStyleSheet("font-size:16px; font-weight:700; min-width:80px;")
            grid.addWidget(val, i, 2)
            self.cal_valores.append(val)
            bp = QPushButton("+")
            bp.setObjectName("Jog")
            bp.clicked.connect(lambda _=False, a=i + 1: self._jog(a, +1))
            grid.addWidget(bp, i, 3)
        gw = QWidget()
        gw.setLayout(grid)
        lay.addWidget(card(gw))

        ew = QWidget()
        el = QHBoxLayout(ew)
        el.setContentsMargins(0, 0, 0, 0)
        self.cal_btns_esq = {}
        for nombre in ["TL", "TR", "BL", "BR", "CENTRO"]:
            b = QPushButton(f"Guardar {nombre}")
            b.clicked.connect(lambda _=False, n=nombre: self._cal_guardar_esq(n))
            el.addWidget(b)
            self.cal_btns_esq[nombre] = b
        lay.addWidget(card(ew))

        self.cal_estado = QLabel("Esquinas guardadas: ninguna")
        self.cal_estado.setObjectName("Sub")
        lay.addWidget(self.cal_estado)
        b_guardar = QPushButton("Guardar calibracion")
        b_guardar.setObjectName("Primario")
        b_guardar.clicked.connect(self._cal_guardar_json)
        lay.addWidget(b_guardar)
        lay.addStretch()
        return pag

    def _jog(self, art, signo):
        if not self.conectado:
            self._aviso("Conecta el brazo primero (pestaña Inicio).")
            return
        if self._cal_moviendo:
            return
        paso = int(self.cal_paso.currentText()) * signo
        self.cal_pose[art - 1] = clamp(art, self.cal_pose[art - 1] + paso)
        self._cal_moviendo = True
        w = MoverWorker(self.esp, self.cal_pose)
        w.listo.connect(self._jog_listo)
        self._workers.append(w)
        w.start()

    def _jog_listo(self, ok):
        self._cal_moviendo = False
        for i in range(5):
            self.cal_valores[i].setText(str(self.cal_pose[i]))

    def _cal_home(self):
        if not self.conectado or self._cal_moviendo:
            return
        self.cal_pose = [0, 0, 0, 0, 0]
        self._cal_moviendo = True
        w = MoverWorker(self.esp, self.cal_pose)
        w.listo.connect(self._jog_listo)
        self._workers.append(w)
        w.start()

    def _cal_guardar_esq(self, nombre):
        self.cal_guardadas[nombre] = list(self.cal_pose)
        self.cal_btns_esq[nombre].setText(f"✓ {nombre}")
        self.cal_estado.setText("Esquinas guardadas: " + ", ".join(self.cal_guardadas.keys()))

    def _cal_guardar_json(self):
        faltan = [e for e in ["TL", "TR", "BL", "BR"] if e not in self.cal_guardadas]
        if faltan:
            self._aviso("Faltan esquinas: " + ", ".join(faltan))
            return
        datos = {"zona": ZONA_DEF, "poses": {}}
        for e in ["TL", "TR", "BL", "BR"]:
            datos["poses"][e] = self.cal_guardadas[e]
        datos["poses"]["CENTRO"] = self.cal_guardadas.get("CENTRO") or [
            round(sum(self.cal_guardadas[e][i] for e in ["TL", "TR", "BL", "BR"]) / 4)
            for i in range(5)]
        ARCHIVO_CALIB.parent.mkdir(exist_ok=True)
        ARCHIVO_CALIB.write_text(json.dumps(datos, indent=2), encoding="utf-8")
        self._aviso("Calibracion guardada en config/calibracion.json")
        self._refrescar_estado()

    # ---------- CALIBRAR HOME ----------
    def _pagina_calib_home(self):
        pag = QWidget()
        lay = QVBoxLayout(pag)
        lay.setContentsMargins(34, 28, 34, 28)
        lay.setSpacing(14)
        tit = QLabel("Calibracion de HOME")
        tit.setObjectName("Titulo")
        lay.addWidget(tit)
        lay.addWidget(self._sub("Mové el brazo para verificar el HOME. Escribí o usá las flechas y dale Ejecutar."))

        grid = QGridLayout()
        grid.setSpacing(10)
        self.home_spins = []
        for i, nom in enumerate(["Art1", "Art2", "Art3", "Art4", "Art5"]):
            grid.addWidget(QLabel(nom), i, 0)
            sp = QSpinBox()
            sp.setRange(LIM_MIN[i + 1], LIM_MAX[i + 1])
            sp.setSingleStep(50)
            sp.setValue(0)
            grid.addWidget(sp, i, 1)
            self.home_spins.append(sp)
        gw = QWidget()
        gw.setLayout(grid)
        lay.addWidget(card(gw))

        bw = QWidget()
        bl = QHBoxLayout(bw)
        bl.setContentsMargins(0, 0, 0, 0)
        b_exe = QPushButton("Ejecutar movimiento")
        b_exe.setObjectName("Primario")
        b_exe.clicked.connect(self._home_ejecutar)
        bl.addWidget(b_exe)
        b_cero = QPushButton("Poner todo en 0")
        b_cero.clicked.connect(lambda: [sp.setValue(0) for sp in self.home_spins])
        bl.addWidget(b_cero)
        b_re = QPushButton("Re-establecer HOME aqui")
        b_re.clicked.connect(self._reestablecer_home)
        bl.addWidget(b_re)
        bl.addStretch()
        lay.addWidget(bw)

        nota = QLabel(
            "Si el brazo NO vuelve a la marca, el HOME esta mal: llevalo fisicamente "
            "a la marca y usá 'Re-establecer HOME aqui' (reconecta y toma esa posicion como cero).")
        nota.setObjectName("Sub")
        nota.setWordWrap(True)
        lay.addWidget(nota)
        lay.addStretch()
        return pag

    def _home_ejecutar(self):
        if not self.conectado:
            self._aviso("Conecta el brazo primero (pestaña Inicio).")
            return
        if self._cal_moviendo:
            return
        pose = [sp.value() for sp in self.home_spins]
        self._cal_moviendo = True
        w = MoverWorker(self.esp, pose)
        w.listo.connect(lambda ok: setattr(self, "_cal_moviendo", False))
        self._workers.append(w)
        w.start()

    def _reestablecer_home(self):
        if not self.conectado:
            self._aviso("Conecta el brazo primero.")
            return
        r = QMessageBox.question(self, "Re-establecer HOME",
                                 "¿El brazo esta exactamente en la marca de HOME?")
        if r != QMessageBox.StandardButton.Yes:
            return
        try:
            self.esp.cerrar()
        except Exception:
            pass
        self.conectado = False
        self._refrescar_estado()
        self._conectar(silencioso=False)
        for sp in self.home_spins:
            sp.setValue(0)
        self._aviso("HOME re-establecido en la posicion actual.")

    # ---------- TEST ----------
    def _pagina_test(self):
        pag = QWidget()
        lay = QVBoxLayout(pag)
        lay.setContentsMargins(34, 28, 34, 28)
        lay.setSpacing(14)
        tit = QLabel("Test de movimiento")
        tit.setObjectName("Titulo")
        lay.addWidget(tit)
        lay.addWidget(self._sub("Asegurate de que el area del brazo este despejada antes de probar."))

        fila = QWidget()
        fl = QHBoxLayout(fila)
        fl.setContentsMargins(0, 0, 0, 0)
        fl.setSpacing(16)
        b1 = QPushButton("Secuencia 1\n(una articulacion a la vez)")
        b1.setObjectName("Sec")
        b1.clicked.connect(lambda: self._correr_secuencia(1))
        b2 = QPushButton("Secuencia 2\n(todas a la vez)")
        b2.setObjectName("Sec")
        b2.clicked.connect(lambda: self._correr_secuencia(2))
        fl.addWidget(b1)
        fl.addWidget(b2)
        lay.addWidget(fila)

        self.test_log = QTextEdit()
        self.test_log.setReadOnly(True)
        self.test_log.setMinimumHeight(220)
        lay.addWidget(self.test_log, 1)
        self.test_botones = [b1, b2]
        return pag

    def _correr_secuencia(self, cual):
        if not self.conectado:
            self._aviso("Conecta el brazo primero (pestaña Inicio).")
            return
        for b in self.test_botones:
            b.setEnabled(False)
        self._bloquear_nav(True)
        self.test_log.append(f"--- Secuencia {cual} iniciada ---")
        w = SecuenciaWorker(self.esp, cual)
        w.log.connect(self.test_log.append)
        w.listo.connect(self._secuencia_lista)
        self._workers.append(w)
        w.start()

    def _secuencia_lista(self):
        for b in self.test_botones:
            b.setEnabled(True)
        self._bloquear_nav(False)
        self.test_log.append("--- Terminada. Brazo en HOME ---\n")

    # ---------- AJUSTES ----------
    def _pagina_ajustes(self):
        pag = QWidget()
        lay = QVBoxLayout(pag)
        lay.setContentsMargins(34, 28, 34, 28)
        lay.setSpacing(14)
        tit = QLabel("Ajustes")
        tit.setObjectName("Titulo")
        lay.addWidget(tit)

        grid = QGridLayout()
        grid.setSpacing(14)

        self.aj_com = QLineEdit(self.ajustes["com"])
        self._fila_ajuste(grid, 0, "Puerto COM por defecto:", self.aj_com, "com")

        self.aj_dead = QDoubleSpinBox()
        self.aj_dead.setRange(0.02, 0.30)
        self.aj_dead.setSingleStep(0.01)
        self.aj_dead.setValue(float(self.ajustes["deadband"]))
        self._fila_ajuste(grid, 1, "Zona muerta:", self.aj_dead, "deadband")

        self.aj_frames = QSpinBox()
        self.aj_frames.setRange(1, 15)
        self.aj_frames.setValue(int(self.ajustes["frames"]))
        self._fila_ajuste(grid, 2, "Frames de confirmacion:", self.aj_frames, "frames")

        self.aj_cam = QSpinBox()
        self.aj_cam.setRange(0, 5)
        self.aj_cam.setValue(int(self.ajustes["camara"]))
        self._fila_ajuste(grid, 3, "Indice de camara:", self.aj_cam, "camara")

        gw = QWidget()
        gw.setLayout(grid)
        lay.addWidget(card(gw))

        b = QPushButton("Guardar ajustes")
        b.setObjectName("Primario")
        b.clicked.connect(self._guardar_ajustes)
        lay.addWidget(b)
        lay.addWidget(self._sub("Los ajustes se guardan en la base de datos y se recuerdan la proxima vez."))
        lay.addStretch()
        return pag

    def _fila_ajuste(self, grid, fila, etiqueta, widget, clave):
        grid.addWidget(QLabel(etiqueta), fila, 0)
        grid.addWidget(widget, fila, 1)
        b = QPushButton()
        b.setObjectName("Icono")
        b.setIconSize(QSize(18, 18))
        b.setIcon(icono("fa5s.question-circle", COLOR_ICONO[self.tema]))
        b.clicked.connect(lambda _=False, c=clave: self._mostrar_ayuda(c))
        grid.addWidget(b, fila, 2)

    def _mostrar_ayuda(self, clave):
        QMessageBox.information(self, "¿Que es esto?", AYUDAS.get(clave, ""))

    def _guardar_ajustes(self):
        self.ajustes["com"] = self.aj_com.text().strip() or "COM11"
        self.ajustes["deadband"] = round(self.aj_dead.value(), 3)
        self.ajustes["frames"] = self.aj_frames.value()
        self.ajustes["camara"] = self.aj_cam.value()
        db.guardar(self.ajustes)
        self._poblar_puertos()
        self._aviso("Ajustes guardados.")

    # ---------- COMUNES ----------
    def _aviso(self, texto):
        QMessageBox.information(self, "Asistente Odontologico", texto)

    def closeEvent(self, event):
        try:
            if self.camara:
                self.camara.detener()
            if self.seguidor:
                self.seguidor.detener()
            if self.esp and self.conectado:
                self.esp.mover(0, 0, 0, 0, 0)
                self.esp.cerrar()
        except Exception:
            pass
        event.accept()


def main():
    app = QApplication([])
    app.setFont(QFont("Segoe UI", 10))
    win = App()
    win.showMaximized()
    app.exec()
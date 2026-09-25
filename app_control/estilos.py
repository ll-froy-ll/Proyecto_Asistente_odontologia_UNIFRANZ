ACENTO = "#14B8A6"
ACENTO_OSC = "#0F766E"

_COMUN = """
QPushButton[nav="true"] {{ text-align: left; padding: 11px 14px; border: none; border-radius: 10px;
    background: transparent; color: {nav_txt}; font-size: 15px; }}
QPushButton[nav="true"]:hover {{ background: {nav_hover}; }}
QPushButton[nav="true"]:checked {{ background: {acento}; color: white; font-weight: 600; }}
QPushButton[nav="true"]:disabled {{ color: {disabled}; }}
#Card {{ background: {card}; border: 1px solid {borde}; border-radius: 16px; }}
#Titulo {{ font-size: 25px; font-weight: 700; color: {titulo}; }}
#Sub {{ color: {sub}; font-size: 13px; }}
#SecTit {{ font-size: 16px; font-weight: 600; color: {titulo}; }}
#Logo {{ font-size: 19px; font-weight: 700; color: {acento}; }}
#LogoSub {{ color: {sub}; font-size: 11px; }}
QPushButton#Primario {{ background: {acento}; color: white; border: none; border-radius: 12px;
    padding: 11px 18px; font-size: 15px; font-weight: 600; }}
QPushButton#Primario:hover {{ background: {acento_osc}; }}
QPushButton#Primario:disabled {{ background: {disabled_bg}; color: {disabled}; }}
QPushButton#Iniciar {{ background: {acento}; color: white; border: none; border-radius: 26px;
    font-size: 28px; font-weight: 700; padding: 44px; }}
QPushButton#Iniciar:hover {{ background: {acento_osc}; }}
QPushButton#Iniciar:disabled {{ background: {disabled_bg}; color: {disabled}; }}
QPushButton#Estop {{ background: #DC2626; color: white; border: none; border-radius: 16px;
    font-size: 19px; font-weight: 700; padding: 20px; }}
QPushButton#Estop:hover {{ background: #B91C1C; }}
QPushButton#Sec {{ background: {card}; border: 2px solid {acento}; color: {acento}; border-radius: 14px;
    padding: 22px; font-size: 16px; font-weight: 600; }}
QPushButton#Sec:hover {{ background: {nav_hover}; }}
QPushButton#Icono {{ background: transparent; border: none; padding: 6px; border-radius: 8px; }}
QPushButton#Icono:hover {{ background: {nav_hover}; }}
QPushButton#Jog {{ background: {jog}; border: none; border-radius: 10px; font-size: 16px;
    font-weight: 700; padding: 8px; min-width: 42px; color: {titulo}; }}
QPushButton#Jog:hover {{ background: {acento}; color: white; }}
QPushButton {{ background: {boton}; border: none; border-radius: 8px; padding: 8px 14px; color: {titulo}; }}
QPushButton:hover {{ background: {boton_hover}; }}
QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {{ background: {input_bg}; border: 1px solid {borde};
    border-radius: 8px; padding: 8px; min-height: 20px; color: {titulo}; }}
QComboBox QAbstractItemView {{ background: {card}; color: {titulo}; selection-background-color: {acento}; }}
QTextEdit {{ background: {input_bg}; border: 1px solid {borde}; border-radius: 10px; color: {titulo}; }}
#CamView {{ background: #0B1220; border-radius: 16px; color: #64748B; }}
#Tel {{ font-size: 22px; font-weight: 700; color: {acento}; }}
#TelLbl {{ color: {sub}; font-size: 12px; }}
QCheckBox {{ font-size: 15px; spacing: 10px; color: {titulo}; }}
QCheckBox::indicator {{ width: 22px; height: 22px; }}
QLabel {{ color: {titulo}; }}
"""

CLARO = ("""
QWidget {{ background: #F1F5F9; color: #1E293B; font-family: 'Segoe UI','Arial'; font-size: 14px; }}
#Sidebar {{ background: #FFFFFF; border-right: 1px solid #E2E8F0; }}
#TopBar {{ background: #FFFFFF; border-bottom: 1px solid #E2E8F0; }}
""" + _COMUN).format(
    acento=ACENTO, acento_osc=ACENTO_OSC, card="#FFFFFF", borde="#E2E8F0",
    titulo="#0F172A", sub="#64748B", nav_txt="#475569", nav_hover="#F0FDFA",
    disabled="#94A3B8", disabled_bg="#CBD5E1", boton="#E2E8F0", boton_hover="#CBD5E1",
    jog="#E2E8F0", input_bg="#FFFFFF")

OSCURO = ("""
QWidget {{ background: #0F172A; color: #E2E8F0; font-family: 'Segoe UI','Arial'; font-size: 14px; }}
#Sidebar {{ background: #1E293B; border-right: 1px solid #334155; }}
#TopBar {{ background: #1E293B; border-bottom: 1px solid #334155; }}
""" + _COMUN).format(
    acento=ACENTO, acento_osc=ACENTO_OSC, card="#1E293B", borde="#334155",
    titulo="#F1F5F9", sub="#94A3B8", nav_txt="#CBD5E1", nav_hover="#0F3D3A",
    disabled="#475569", disabled_bg="#334155", boton="#334155", boton_hover="#475569",
    jog="#334155", input_bg="#0F172A")

COLOR_ICONO = {"claro": "#475569", "oscuro": "#CBD5E1"}


def qss(tema):
    return OSCURO if tema == "oscuro" else CLARO

"""
puente_serial.py  -  archivo puente (no contiene logica)
--------------------------------------------------------
Todo el codigo vive en puente_serial_v2.py. Este archivo existe para que los
programas que ya hacen 'from puente_serial import PuenteESP32' o
'from control_motores.puente_serial import PuenteESP32' sigan andando sin
tocarlos (la app, calibrar_grilla, medir_brazo, probar_cinematica...).

Para volver a la version anterior, cambia los dos '2' por '1' aqui abajo
(guarda tu puente viejo como puente_serial_v1.py).
"""

try:
    # importado como paquete:  from control_motores.puente_serial import ...
    from .puente_serial_v2 import (      # noqa: F401
        PuenteESP32,
        configurar_logging,
        listar_puertos,
        duracion_estimada_s,
        recortar,
        main,
    )
except ImportError:
    # importado suelto:  from puente_serial import ...
    from puente_serial_v2 import (       # noqa: F401
        PuenteESP32,
        configurar_logging,
        listar_puertos,
        duracion_estimada_s,
        recortar,
        main,
    )

if __name__ == "__main__":
    main()
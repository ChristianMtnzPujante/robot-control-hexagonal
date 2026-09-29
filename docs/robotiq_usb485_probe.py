#!/usr/bin/env python3
"""Sonda de la pinza Robotiq 2F (y del sensor FT 300) por un conversor
RS-485/USB, SIN pasar por el robot.

NO es un nodo ni forma parte de ningún paquete -- vive en docs/, como
`stub_node.py`, porque es una herramienta de laboratorio, no
infraestructura del sistema. Su razón de ser (18/09): la pinza no responde
a través del CR5, todo lo achacable a software está descartado, y queda
por decidir si el par RS-485 está cruzado. Contra un conversor USB los
hilos van a bornes de tornillo, así que probar las dos polaridades es
cuestión de un minuto -- y este script dice al instante cuál es la buena.

Por qué habla Modbus a mano en vez de usar pymodbus: son 30 líneas, evita
una dependencia más en el laboratorio, y deja el CRC y el formato de trama
a la vista, que es justo lo que hay que entender para depurar esto. Las
tramas de ejemplo de los manuales de Robotiq se usan como test (--autotest).

Uso:
    python3 robotiq_usb485_probe.py --port /dev/ttyUSB0
    python3 robotiq_usb485_probe.py --port /dev/ttyUSB0 --ft   # el sensor
    python3 robotiq_usb485_probe.py --autotest                 # sin hardware

    Acciones que MUEVEN los dedos (requieren --si-se-mueve para recordarlo):
    --activar          recorrido completo de calibración
    --apertura 0.0     0.0 abrir del todo .. 1.0 cerrar del todo

La pinza necesita sus 24V aparte: el conversor no los suministra.
"""

from __future__ import annotations

import argparse
import sys
import time

# Parámetros de fábrica (manuales Robotiq: 2F §4.7, FT 300 §4).
PINZA = {"nombre": "2F", "slave": 9, "baud": 115200, "estado": 2000, "accion": 1000}
SENSOR = {"nombre": "FT 300", "slave": 9, "baud": 19200, "estado": 180, "serie": 510}


def crc16(datos: bytes) -> bytes:
    """CRC de Modbus RTU (polinomio 0xA001), devuelto en el orden en que
    viaja: byte bajo primero."""
    crc = 0xFFFF
    for byte in datos:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def trama_leer(slave: int, addr: int, count: int) -> bytes:
    """FC03, leer registros de retención."""
    cuerpo = bytes([slave, 0x03, addr >> 8, addr & 0xFF, count >> 8, count & 0xFF])
    return cuerpo + crc16(cuerpo)


def trama_escribir(slave: int, addr: int, registros) -> bytes:
    """FC16, escribir varios registros."""
    cuerpo = bytes([slave, 0x10, addr >> 8, addr & 0xFF, 0x00, len(registros),
                    2 * len(registros)])
    for r in registros:
        cuerpo += bytes([(r >> 8) & 0xFF, r & 0xFF])
    return cuerpo + crc16(cuerpo)


def parsear_respuesta(datos: bytes, slave: int):
    """Devuelve (registros, error). error es None si todo fue bien."""
    if not datos:
        return None, "silencio: no llegó ni un byte"
    if len(datos) < 5:
        return None, "respuesta demasiado corta: %s" % datos.hex(" ")
    if datos[0] != slave:
        return None, "responde otro slave (%d, esperábamos %d)" % (datos[0], slave)
    if crc16(datos[:-2]) != datos[-2:]:
        return None, "CRC incorrecto -- hay comunicación pero llega corrupta: %s" % datos.hex(" ")
    if datos[1] & 0x80:
        # Excepción: el dispositivo ESTÁ AHÍ pero rechaza la petición. Es un
        # resultado EXCELENTE para depurar -- significa que el bus funciona.
        return None, ("excepción Modbus 0x%02X -- ¡pero el dispositivo RESPONDE! "
                      "El bus va bien; es la dirección/función lo que no le gusta"
                      % datos[2])
    n = datos[2]
    regs = [(datos[3 + 2 * i] << 8) | datos[4 + 2 * i] for i in range(n // 2)]
    return regs, None


def decodificar_pinza(regs):
    b0 = regs[0] >> 8
    gsta = (b0 >> 4) & 0x03
    gobj = (b0 >> 6) & 0x03
    return {
        "gACT": b0 & 1, "gGTO": (b0 >> 3) & 1, "gSTA": gsta, "gOBJ": gobj,
        "gFLT": regs[1] >> 8, "gPR": regs[1] & 0xFF,
        "gPO": regs[2] >> 8, "gCU": regs[2] & 0xFF,
        "activada": gsta == 3,
        "agarra_algo": gobj in (1, 2),
    }


FALLOS = {
    0x00: "sin fallo", 0x05: "falta completar la activación",
    0x07: "hay que activar antes de mandar nada",
    0x08: "temperatura máxima superada", 0x09: "MÁS DE 1s SIN COMUNICACIÓN",
    0x0A: "tensión por debajo del mínimo", 0x0B: "liberación automática en curso",
    0x0C: "fallo interno", 0x0D: "fallo de activación",
    0x0E: "sobrecorriente", 0x0F: "liberación automática completada",
}


def intercambio(puerto, trama: bytes, espera: float = 0.3) -> bytes:
    puerto.reset_input_buffer()
    puerto.write(trama)
    puerto.flush()
    time.sleep(espera)
    return puerto.read(puerto.in_waiting or 32)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="p. ej. /dev/ttyUSB0")
    p.add_argument("--baud", type=int, help="por defecto: 115200 (pinza) o 19200 (sensor)")
    p.add_argument("--slave", type=int, default=9)
    p.add_argument("--ft", action="store_true", help="sondear el sensor FT 300 en vez de la pinza")
    p.add_argument("--activar", action="store_true", help="MUEVE los dedos de tope a tope")
    p.add_argument("--apertura", type=float, help="0.0 abrir .. 1.0 cerrar -- MUEVE los dedos")
    p.add_argument("--si-se-mueve", action="store_true",
                   help="confirma que sabes que --activar/--apertura mueven la pinza")
    p.add_argument("--escuchar", type=float, metavar="SEGUNDOS",
                   help="modo ESCUCHA: no transmite nada, solo vuelca lo que pase "
                        "por el bus. Sirve para comprobar si OTRO maestro (p. ej. el "
                        "CR5) está transmitiendo de verdad.")
    p.add_argument("--autotest", action="store_true", help="prueba el código sin hardware")
    args = p.parse_args()

    if args.autotest:
        return autotest()
    if not args.port:
        p.error("hace falta --port (o --autotest)")

    perfil = SENSOR if args.ft else PINZA
    baud = args.baud or perfil["baud"]
    mueve = args.activar or args.apertura is not None
    if mueve and not args.si_se_mueve:
        print("Eso mueve los dedos. Añade --si-se-mueve cuando la pinza esté despejada.")
        return 2
    # Validar ANTES de abrir el puerto, mismo criterio que
    # Cr5RealRobotAdapter.set_joints: si el dato es inválido, no se toca el
    # hardware para nada.
    if args.apertura is not None and not 0.0 <= args.apertura <= 1.0:
        print("--apertura fuera de [0,1]: %s (0 = abrir, 1 = cerrar)" % args.apertura)
        return 2

    import serial  # se importa aquí para que --autotest no lo necesite

    print("Puerto %s a %d bps, 8N1, slave %d" % (args.port, baud, args.slave))
    try:
        puerto = serial.Serial(args.port, baud, bytesize=8, parity="N", stopbits=1,
                               timeout=0.5)
    except serial.SerialException as error:
        print("\nNo se pudo abrir %s: %s" % (args.port, error))
        print("Puertos serie disponibles ahora mismo:")
        import glob
        candidatos = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        print("  " + ("\n  ".join(candidatos) if candidatos else
                      "(ninguno: ¿está enchufado el conversor?)"))
        return 1
    with puerto:
        if args.escuchar:
            return escuchar(puerto, args.escuchar)
        if args.activar:
            print("Activando (reset + rACT, recorrido completo)...")
            intercambio(puerto, trama_escribir(args.slave, perfil["accion"], [0, 0, 0]))
            intercambio(puerto, trama_escribir(args.slave, perfil["accion"], [0x0100, 0, 0]))
        if args.apertura is not None:
            regs = [0x0900, round(args.apertura * 255), 0x8080]
            print("Mandando apertura %.2f -> %s" % (args.apertura, regs))
            intercambio(puerto, trama_escribir(args.slave, perfil["accion"], regs))

        addr = perfil["estado"]
        count = 6 if args.ft else 3
        crudo = intercambio(puerto, trama_leer(args.slave, addr, count))
        regs, error = parsear_respuesta(crudo, args.slave)
        print("Petición: %s" % trama_leer(args.slave, addr, count).hex(" "))
        print("Respuesta: %s" % (crudo.hex(" ") if crudo else "(nada)"))
        if error:
            print("\n>>> %s" % error)
            if "silencio" in error:
                print("    Si has probado las dos polaridades y sigue mudo, el problema")
                print("    no es el orden del par: mira alimentación y masa.")
                print("    RECUERDA: intercambia SOLO los dos hilos de datos.")
            return 1
        print("\n>>> RESPONDE: %s" % regs)
        if args.ft:
            fuerzas = [v - 65536 if v > 32767 else v for v in regs]
            print("    Fx,Fy,Fz (N) = %s" % [round(v / 100.0, 2) for v in fuerzas[:3]])
            print("    Mx,My,Mz (Nm) = %s" % [round(v / 1000.0, 3) for v in fuerzas[3:]])
        else:
            e = decodificar_pinza(regs)
            print("    gACT=%(gACT)d gGTO=%(gGTO)d gSTA=%(gSTA)d gOBJ=%(gOBJ)d" % e)
            print("    posición=%d/255  corriente≈%d mA" % (e["gPO"], e["gCU"] * 10))
            print("    fallo: 0x%02X (%s)" % (e["gFLT"], FALLOS.get(e["gFLT"], "desconocido")))
            print("    -> %s" % ("ACTIVADA" if e["activada"] else "sin activar"))
            if e["agarra_algo"]:
                print("    -> ha parado por contacto: AGARRA ALGO")
        return 0


def escuchar(puerto, segundos: float) -> int:
    """Modo pasivo: no manda NADA, solo lee. Contesta a la pregunta '¿hay
    alguien transmitiendo en este bus?', que es justo la que no se puede
    responder desde el propio maestro."""
    print("Escuchando %.0fs sin transmitir nada..." % segundos)
    fin = time.time() + segundos
    buf = b""
    trozos = 0
    while time.time() < fin:
        datos = puerto.read(256)
        if datos:
            trozos += 1
            buf += datos
            print("  [%6.2fs] %s" % (segundos - (fin - time.time()), datos.hex(" ")))
    print()
    if not buf:
        print(">>> SILENCIO ABSOLUTO: nadie transmite por este par.")
        print("    Si el CR5 estaba sondeando durante la escucha, su driver 485")
        print("    no está sacando nada -- el problema es del controlador, no del cable.")
        return 1
    print(">>> %d bytes en %d ráfagas. ALGUIEN TRANSMITE." % (len(buf), trozos))
    # ¿Se reconocen tramas Modbus bien formadas?
    validas = 0
    for i in range(len(buf) - 3):
        for n in range(4, min(16, len(buf) - i) + 1):
            trozo = buf[i:i + n]
            if len(trozo) >= 4 and crc16(trozo[:-2]) == trozo[-2:]:
                validas += 1
                print("    trama válida: %s  (slave %d, función 0x%02X)" %
                      (trozo.hex(" "), trozo[0], trozo[1]))
                break
    if validas:
        print("    -> Tramas con CRC correcto: la señal llega LIMPIA y con la")
        print("       polaridad correcta para este conversor.")
    else:
        print("    -> Bytes sí, pero ninguna trama con CRC válido: la señal llega")
        print("       CORRUPTA. Típico de polaridad invertida o velocidad distinta.")
        print("       Prueba a intercambiar los dos hilos en los bornes y repetir.")
    return 0


def autotest() -> int:
    """Valida el código contra las tramas de ejemplo de los manuales y
    contra un esclavo de mentira por un pty."""
    fallos = []

    # 1. CRC contra las tramas documentadas por Robotiq.
    casos = [
        ("FT300 leer Fx (manual FT §4)", bytes.fromhex("0903 00B4 0001"), "c5 64"),
        ("2F activación, borrar rACT (manual 2F §4.7.6)",
         bytes.fromhex("0910 03E8 0003 06 000000000000"), "73 30"),
    ]
    for nombre, cuerpo, esperado in casos:
        obtenido = crc16(cuerpo).hex(" ")
        ok = obtenido == esperado
        print("[%s] CRC %-46s esperado %s, obtenido %s" %
              ("ok" if ok else "FALLO", nombre, esperado, obtenido))
        if not ok:
            fallos.append(nombre)

    # 2. Ida y vuelta completa contra un esclavo falso por un pty.
    import os, threading
    maestro, esclavo = os.openpty()
    nombre_pty = os.ttyname(esclavo)

    def falso_esclavo():
        peticion = os.read(maestro, 256)
        # Estado inventado: activada (gSTA=3), sin objeto (gOBJ=3), gPO=128
        regs = [0xF900, 0x0000, 0x8010]
        cuerpo = bytes([9, 3, 6])
        for r in regs:
            cuerpo += bytes([r >> 8, r & 0xFF])
        os.write(maestro, cuerpo + crc16(cuerpo))
        return peticion

    hilo = threading.Thread(target=falso_esclavo, daemon=True)
    hilo.start()
    import serial
    with serial.Serial(nombre_pty, 115200, timeout=0.5) as puerto:
        crudo = intercambio(puerto, trama_leer(9, 2000, 3))
    hilo.join(timeout=2)
    regs, error = parsear_respuesta(crudo, 9)
    ok = error is None and regs == [0xF900, 0x0000, 0x8010]
    print("[%s] ida y vuelta por pty -> %s" % ("ok" if ok else "FALLO", regs or error))
    if not ok:
        fallos.append("pty")
    else:
        e = decodificar_pinza(regs)
        ok2 = e["activada"] and not e["agarra_algo"] and e["gPO"] == 128 and e["gFLT"] == 0
        print("[%s] decodificación -> activada=%s agarra=%s gPO=%d gFLT=0x%02X" %
              ("ok" if ok2 else "FALLO", e["activada"], e["agarra_algo"], e["gPO"], e["gFLT"]))
        if not ok2:
            fallos.append("decodificación")

    # 3. Una excepción Modbus debe reconocerse como "el dispositivo ESTÁ ahí".
    cuerpo = bytes([9, 0x83, 0x02])
    regs, error = parsear_respuesta(cuerpo + crc16(cuerpo), 9)
    ok = regs is None and "RESPONDE" in error
    print("[%s] excepción Modbus reconocida como dispositivo presente" % ("ok" if ok else "FALLO"))
    if not ok:
        fallos.append("excepción")

    # 4. Silencio y CRC corrupto.
    ok = parsear_respuesta(b"", 9)[1].startswith("silencio")
    print("[%s] silencio detectado" % ("ok" if ok else "FALLO"))
    if not ok: fallos.append("silencio")
    ok = "CRC incorrecto" in parsear_respuesta(bytes([9, 3, 6, 0, 0, 0, 0, 0, 0, 0xAA, 0xBB]), 9)[1]
    print("[%s] CRC corrupto detectado" % ("ok" if ok else "FALLO"))
    if not ok: fallos.append("crc corrupto")

    print("\n%s" % ("TODO OK" if not fallos else "FALLOS: %s" % fallos))
    return 0 if not fallos else 1


if __name__ == "__main__":
    sys.exit(main())

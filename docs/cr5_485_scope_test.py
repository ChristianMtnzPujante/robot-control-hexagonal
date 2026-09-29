#!/usr/bin/env python3
"""Hace transmitir al CR5 por el RS-485 de la brida, de forma repetitiva y
predecible, para observarlo con un osciloscopio.

NO es un nodo ni forma parte de ningún paquete -- vive en docs/, como
`stub_node.py` y `robotiq_usb485_probe.py`: herramienta de laboratorio.

Su razón de ser (21/09): la pinza Robotiq no recibe nada y no hemos podido
determinar si el problema es que el CR5 no transmite. Todo lo que sabemos
ha pasado por su propio driver 485, que desde el software es una caja
negra: `GetHoldRegs` devuelve `-1` tanto si no hay nadie como si la señal
sale mal o no sale. El osciloscopio mira la línea por fuera y lo resuelve.

Qué hace: abre un maestro Modbus RTU sobre la brida y pide el mismo
registro una y otra vez. Cada petición son unos pocos bytes por el par
485 -- eso es lo que hay que ver en la pantalla.

Uso:
    python3 cr5_485_scope_test.py                    # 115200, como la pinza
    python3 cr5_485_scope_test.py --baud 9600        # más lento, más fácil de ver
    python3 cr5_485_scope_test.py --intervalo 2      # una trama cada ~2,5 s, con hora
    python3 cr5_485_scope_test.py --dry-run          # sin robot: solo los cálculos
    python3 cr5_485_scope_test.py --via rtu          # ModbusRTUCreate en vez de la brida

Por qué maestro sale (29/09): por defecto `ModbusCreate("127.0.0.1",60000,
slave,1)`, el paso directo del controlador al 485 de la BRIDA -- la única
vía con la que la pinza ha contestado. `--via rtu` usa `ModbusRTUCreate`,
con el que la pinza no contestó nunca (probablemente sale por el 485 del
armario; sin confirmar). Ver robotiq_2f_adapter.py para las fuentes.

Dónde pinchar (brida del CR5, conector M8 de 8 polos):
    CH1 -> pin 1 (485A, hilo blanco)
    CH2 -> pin 2 (485B, hilo marrón)
    masa -> pin 8 (GND, hilo rojo)
    y en el osciloscopio, canal MATH = CH1 - CH2 (la señal real es la
    DIFERENCIA entre los dos hilos, no cada uno por su cuenta).
"""

from __future__ import annotations

import argparse
import socket
import sys
import time

_HOST = "192.168.5.1"
_PUERTO = 29999


def crc16(datos: bytes) -> bytes:
    """CRC de Modbus RTU, byte bajo primero. Duplicado a propósito de
    `robotiq_usb485_probe.py`: son dos herramientas sueltas de docs/ que
    deben poder copiarse a otra máquina por separado (mismo criterio que
    `_rpy_to_matrix` en urdf_kit/poe_adapter)."""
    crc = 0xFFFF
    for byte in datos:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def trama_leer(slave: int, addr: int, count: int) -> bytes:
    cuerpo = bytes([slave, 0x03, addr >> 8, addr & 0xFF, count >> 8, count & 0xFF])
    return cuerpo + crc16(cuerpo)


def bits_en_la_linea(byte: int) -> str:
    """Cómo viaja un byte por un UART: bit de arranque a 0, luego los 8
    bits EMPEZANDO POR EL MENOS SIGNIFICATIVO, luego el de parada a 1.
    En reposo la línea está a 1."""
    bits = [("inicio", 0)]
    for i in range(8):
        bits.append(("d%d" % i, (byte >> i) & 1))
    bits.append(("parada", 1))
    return " ".join("%d" % v for _, v in bits)


class Dash:
    def __init__(self, host: str):
        self._host = host
        self.s = None
        self.conectar()

    def conectar(self) -> None:
        for _ in range(15):
            try:
                s = socket.create_connection((self._host, _PUERTO), timeout=15)
            except OSError:
                time.sleep(3)
                continue
            s.settimeout(2)
            try:
                if "occupied" in s.recv(4096).decode(errors="replace"):
                    s.close()
                    time.sleep(4)
                    continue
            except Exception:
                pass
            s.settimeout(15)
            self.s = s
            return
        raise SystemExit("no se pudo abrir el dashboard del CR5")

    def cmd(self, c: str, mostrar: bool = False):
        nombre = c.split("(", 1)[0]
        try:
            self.s.sendall(c.encode())
        except Exception:
            self.reconectar()
            return None
        buf = ""
        fin = time.time() + 15
        while time.time() < fin:
            try:
                trozo = self.s.recv(4096).decode(errors="replace")
            except Exception:
                self.reconectar()
                return None
            if not trozo:
                self.reconectar()
                return None
            buf += trozo
            if nombre in buf and ";" in buf:
                if mostrar:
                    print("  %-36s -> %s" % (c, buf.strip()))
                    sys.stdout.flush()
                return buf.strip()
        return None

    def reconectar(self) -> None:
        # SetToolPower y compañía resetean la conexión con frecuencia
        # (ver ROADMAP, Bloque 0): reconectar y volver a pedir el control.
        try:
            self.s.close()
        except Exception:
            pass
        time.sleep(3)
        self.conectar()
        self.cmd("RequestControl()")


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--host", default=_HOST)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--slave", type=int, default=9)
    p.add_argument("--addr", type=int, default=2000, help="registro a pedir")
    p.add_argument("--count", type=int, default=3)
    p.add_argument("--duracion", type=float, default=120.0, help="segundos")
    p.add_argument("--intervalo", type=float, default=0.0,
                   help="segundos de pausa entre tramas (0 = seguidas, ~2/s); "
                        "con pausa se imprime cada envío con su hora")
    p.add_argument("--patron", action="store_true",
                   help="tramas de PATRÓN en vez de una lectura util: usa esclavo "
                        "0x55 y registro 0x5555 para que por la linea salgan bytes "
                        "0x55 (unos y ceros alternos = onda cuadrada limpia). "
                        "Nadie responde, y no hace falta: solo queremos ver la senal")
    p.add_argument("--via", choices=["brida", "rtu"], default="brida",
                   help='brida = ModbusCreate("127.0.0.1",60000,...) (llega a la '
                        'pinza); rtu = ModbusRTUCreate (no llega a la brida)')
    p.add_argument("--dry-run", action="store_true", help="no toca el robot")
    args = p.parse_args()

    if args.patron:
        # 0x55 = 0b01010101: el byte de prueba de toda la vida. Con bit de
        # arranque y de parada sale casi una onda cuadrada perfecta, y su
        # periodo mide los baudios directamente en pantalla.
        #
        # El `count` se queda en 3 y NO en 0x55 (=85) a propósito: un count
        # de 85 lo rechaza el propio controlador con -40003 en 1 ms, SIN
        # llegar a transmitir -- medido el 21/09. La forma de distinguirlo
        # es el tiempo de respuesta: un rechazo local tarda ~1 ms, una
        # transacción de verdad ~500 ms (el timeout Modbus del CR5). Si
        # este script va muy rápido, es que no está saliendo nada al cable.
        args.slave, args.addr = 0x55, 0x5555
    trama = trama_leer(args.slave, args.addr, args.count)
    bit_us = 1e6 / args.baud
    trama_us = len(trama) * 10 * bit_us

    print("=" * 66)
    print("LO QUE DEBERÍA APARECER EN PANTALLA")
    print("=" * 66)
    print("Trama enviada (%d bytes): %s" % (len(trama), trama.hex(" ").upper()))
    print("  slave=%d  función=03 (leer)  registro=%d  cantidad=%d  + CRC"
          % (args.slave, args.addr, args.count))
    print()
    print("Velocidad: %d bps  ->  1 bit = %.2f us  |  trama completa = %.0f us"
          % (args.baud, bit_us, trama_us))
    print()
    if args.patron:
        print("MODO PATRÓN: %d de los %d bytes son 0x55 (unos y ceros alternos)."
              % (sum(1 for b in trama if b == 0x55), len(trama)))
        print("  -> en pantalla, onda cuadrada a %.1f kHz durante esos tramos"
              % (args.baud / 2000.0))
        print("  -> midiendo su periodo (%.2f us) confirmas la velocidad real"
              % (2 * bit_us))
        print()
    print("Primer byte (0x%02X) tal y como viaja por la línea:" % trama[0])
    print("   reposo=1 | %s |" % bits_en_la_linea(trama[0]))
    print("   (el bit de ARRANQUE a 0 es el primer flanco de bajada: dispara ahí)")
    print()
    print("Ajustes sugeridos del osciloscopio:")
    print("  - MATH = CH1 - CH2   (la señal es la diferencia entre los dos hilos)")
    print("  - Base de tiempos: %.0f us/div deja ver la trama entera;"
          % max(trama_us / 10, 20))
    print("                     %.0f us/div deja ver los bits sueltos" % (bit_us * 2))
    print("  - Disparo: flanco de BAJADA en MATH, nivel ~0 V, modo NORMAL")
    print("  - Amplitud esperada de un driver sano: 2 a 5 V entre pico y pico")
    print()
    print("CÓMO INTERPRETARLO")
    print("  Nada en absoluto, línea plana .... el CR5 NO transmite -> avería del")
    print("                                     controlador, el cable queda absuelto")
    print("  Ráfagas con la forma de arriba ... el CR5 transmite bien; el fallo está")
    print("                                     en la pinza o en el tramo hasta ella")
    print("  Ráfagas pero invertidas .......... en reposo MATH debería estar ALTO;")
    print("                                     si está bajo y sube en los arranques,")
    print("                                     las líneas están CRUZADAS")
    print("  Amplitud mínima (< 0.5 V) ........ driver débil o línea cargada/cortada")
    print("=" * 66)

    if args.dry_run:
        print("\n(--dry-run: no se ha tocado el robot)")
        return 0

    d = Dash(args.host)
    print("\nPreparando el CR5...")
    d.cmd("RequestControl()", mostrar=True)
    d.cmd("RobotMode()", mostrar=True)
    # La alimentación va PRIMERO (24/09): SetToolPower resetea la conexión
    # del 29999 y quizá la placa del extremo, así que podría deshacer un
    # SetToolMode/SetTool485 dado antes. Configurar después no cuesta nada.
    d.cmd("SetToolPower(1)", mostrar=True)
    d.cmd("SetToolMode(1)", mostrar=True)         # el terminal a 485, no analógico
    d.cmd('SetTool485(%d,"N",1)' % args.baud, mostrar=True)
    if args.via == "brida":
        r = d.cmd('ModbusCreate("127.0.0.1",60000,%d,1)' % args.slave, mostrar=True)
    else:
        r = d.cmd('ModbusRTUCreate(%d,%d,"N",8,1)' % (args.slave, args.baud), mostrar=True)
    if not r or not r.startswith("0,"):
        print("No se pudo crear el maestro Modbus.")
        return 1
    idx = r.split("{", 1)[1].split("}", 1)[0]

    print("\n>>> TRANSMITIENDO durante %.0f s. Mira el osciloscopio. <<<" % args.duracion)
    t0 = time.time()
    n = 0
    try:
        while time.time() - t0 < args.duracion:
            t_envio = time.time()
            r = d.cmd("GetHoldRegs(%s,%d,%d)" % (idx, args.addr, args.count))
            n += 1
            if args.intervalo > 0:
                print("   #%d  %s  -> %s (%.2f s)" % (
                    n, time.strftime("%H:%M:%S"), r, time.time() - t_envio))
                sys.stdout.flush()
                time.sleep(args.intervalo)
                continue
            if n % 20 == 0:
                transcurrido = time.time() - t0
                ritmo = n / transcurrido if transcurrido else 0
                aviso = ""
                if ritmo > 20:
                    aviso = "  <-- DEMASIADO RÁPIDO: el controlador está " \
                            "rechazando la petición sin transmitir"
                print("   %d tramas (%.0f s, %.1f/s)%s" % (n, transcurrido, ritmo, aviso))
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n(interrumpido)")
    finally:
        print("\nTotal: %d tramas. Cerrando el maestro." % n)
        d.cmd("ModbusClose(%s)" % idx, mostrar=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

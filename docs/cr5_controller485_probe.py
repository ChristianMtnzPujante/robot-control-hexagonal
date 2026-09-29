#!/usr/bin/env python3
"""Sondea la pinza Robotiq 2F por el RS-485 del ARMARIO del controlador
(bornes 485A / 485B / RG de la interfaz de E/S, §6.1.9 del manual CR A), NO
por el de la brida.

NO es un nodo ni forma parte de ningún paquete -- vive en docs/, como
`cr5_485_scope_test.py` y `robotiq_usb485_probe.py`: herramienta de
laboratorio.

Su razón de ser (29/09): el manual TCP/IP V4.6.5 dice que
`ModbusRTUCreate` crea un maestro "sobre la interfaz RS485" sin decir cuál.
Lo que SÍ está verificado: a la BRIDA se llega con
`ModbusCreate("127.0.0.1",60000,9,1)` (la pinza contestó el 29/09), y con
`ModbusRTUCreate` la pinza de la brida no contestó nunca. Que
`ModbusRTUCreate` salga por el 485 del ARMARIO sigue siendo HIPÓTESIS: este
script sirve para comprobarlo (osciloscopio o dispositivo en el armario).

Por eso NO manda ningún comando de la brida: ni SetToolPower, ni
SetToolMode, ni SetTool485. Toda la configuración serie va en el propio
ModbusRTUCreate. La pinza necesita sus 24V de otra fuente (la brida ya no
la alimenta).

Solo LEE (GetHoldRegs sobre el registro de estado): no mueve los dedos.

Uso:
    python3 cr5_controller485_probe.py                  # 2F: slave 9, 115200 8N1
    python3 cr5_controller485_probe.py --duracion 300   # más tiempo para recablear
    python3 cr5_controller485_probe.py --dry-run        # sin robot: solo la trama

Cableado (armario):  485A -> 485A/+ de la pinza   485B -> 485B/- de la pinza
                     RG   -> GND de la pinza (masa común del 485)
Si no contesta, intercambiar 485A y 485B y repetir.
"""

from __future__ import annotations

import argparse
import re
import socket
import sys
import time

_HOST = "192.168.5.1"
_PUERTO = 29999

_GSTA = {0: "reset", 1: "activándose", 2: "?", 3: "activación completada"}
_GOBJ = {0: "en movimiento", 1: "objeto (abriendo)", 2: "objeto (cerrando)",
         3: "en posición, sin objeto"}


def crc16(datos: bytes) -> bytes:
    """CRC de Modbus RTU, byte bajo primero. Duplicado a propósito de las
    otras herramientas de docs/: cada una debe poder copiarse suelta."""
    crc = 0xFFFF
    for byte in datos:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def trama_leer(slave: int, addr: int, count: int) -> bytes:
    cuerpo = bytes([slave, 0x03, addr >> 8, addr & 0xFF, count >> 8, count & 0xFF])
    return cuerpo + crc16(cuerpo)


def decodificar(regs: list, invertir: bool) -> str:
    """Estado de la 2F a partir de los 3 registros de 2000. `invertir`
    cambia el orden de los bytes dentro de cada registro (la duda de
    endianness que dejó abierta la guía de la pinza)."""
    b = []
    for r in regs:
        alto, bajo = (r >> 8) & 0xFF, r & 0xFF
        b += [bajo, alto] if invertir else [alto, bajo]
    estado, flt, pr, po, cu = b[0], b[2], b[3], b[4], b[5]
    return ("gACT=%d gGTO=%d gSTA=%s gOBJ=%s | gFLT=0x%02X gPR=%d gPO=%d gCU≈%dmA"
            % (estado & 1, (estado >> 3) & 1, _GSTA[(estado >> 4) & 3],
               _GOBJ[(estado >> 6) & 3], flt, pr, po, cu * 10))


class Dash:
    """Cliente mínimo del 29999. Duplicado a propósito de
    cr5_485_scope_test.py (mismo criterio que crc16)."""

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
                # El 29999 admite UN cliente: si está ocupado lo dice por
                # el propio socket en vez de rechazar la conexión.
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
        try:
            self.s.close()
        except Exception:
            pass
        time.sleep(3)
        self.conectar()
        self.cmd("RequestControl()")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--host", default=_HOST)
    p.add_argument("--slave", type=int, default=9)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--paridad", default="N", choices=["N", "E", "O"],
                   help='OJO: si no se pasa, el CR5 usa "E"; la 2F es "N"')
    p.add_argument("--addr", type=int, default=2000, help="registro a leer")
    p.add_argument("--count", type=int, default=3)
    p.add_argument("--intervalo", type=float, default=1.0, help="segundos entre lecturas")
    p.add_argument("--duracion", type=float, default=120.0, help="segundos")
    p.add_argument("--dry-run", action="store_true", help="no toca el robot")
    args = p.parse_args()

    trama = trama_leer(args.slave, args.addr, args.count)
    print("Puerto: RS-485 del ARMARIO (ModbusRTUCreate), no la brida")
    print("Trama por lectura: %s  (slave %d, FC03, registro %d, %d regs)"
          % (trama.hex(" ").upper(), args.slave, args.addr, args.count))
    print("Formato serie: %d %s 8 1" % (args.baud, args.paridad))
    if args.dry_run:
        print("\n(--dry-run: no se ha tocado el robot)")
        return 0

    d = Dash(args.host)
    print("\nPreparando el CR5 (sin comandos de brida)...")
    d.cmd("RequestControl()", mostrar=True)
    d.cmd("RobotMode()", mostrar=True)
    r = d.cmd('ModbusRTUCreate(%d,%d,"%s",8,1)' % (args.slave, args.baud, args.paridad),
              mostrar=True)
    if not r or not r.startswith("0,"):
        print("No se pudo crear el maestro Modbus.")
        return 1
    idx = r.split("{", 1)[1].split("}", 1)[0]

    print("\n>>> LEYENDO durante %.0f s (Ctrl+C para parar) <<<" % args.duracion)
    t0 = time.time()
    n = buenas = 0
    try:
        while time.time() - t0 < args.duracion:
            t_envio = time.time()
            r = d.cmd("GetHoldRegs(%s,%d,%d)" % (idx, args.addr, args.count))
            n += 1
            dt = time.time() - t_envio
            m = re.match(r"\s*(-?\d+),\{([^}]*)\}", r or "")
            if m and m.group(1) == "0" and m.group(2):
                buenas += 1
                regs = [int(x) for x in m.group(2).split(",")]
                print("   #%d  %s  RESPUESTA %s (%.2f s)" % (
                    n, time.strftime("%H:%M:%S"), regs, dt))
                if args.addr == 2000 and len(regs) == 3:
                    print("        byte alto primero: %s" % decodificar(regs, False))
                    print("        byte bajo primero: %s" % decodificar(regs, True))
            else:
                # -1 en ~0,5 s = el CR5 transmitió y nadie contestó (timeout).
                # -1 en ~1 ms = rechazo local, no llegó a salir nada.
                print("   #%d  %s  -> %s (%.2f s)" % (n, time.strftime("%H:%M:%S"), r, dt))
            sys.stdout.flush()
            time.sleep(args.intervalo)
    except KeyboardInterrupt:
        print("\n(interrumpido)")
    finally:
        print("\nTotal: %d lecturas, %d con respuesta. Cerrando el maestro." % (n, buenas))
        d.cmd("ModbusClose(%s)" % idx, mostrar=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

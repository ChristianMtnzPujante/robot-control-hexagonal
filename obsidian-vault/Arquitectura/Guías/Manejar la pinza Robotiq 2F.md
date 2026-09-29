---
tags: [arquitectura, guia, cr5, pinza, robotiq]
---

# Manejar la pinza Robotiq 2F (y meterla en la simulación)

Guía práctica de la **Robotiq 2F Adaptive Gripper** montada en el CR5 del
laboratorio: cómo se le habla desde nuestro stack y cómo añadirla a la
simulación. Para el conector físico por el que va conectada (los 8 pines
de la brida) ver [[E-S del Extremo del CR5 (pinza)]].

> [!tip] Para USARLA, ve a [[Pinza Robotiq 2F - Uso práctico]]
> Esa nota es la chuleta verificada (29/09): conexión, órdenes, estado y
> fallos. Esta guía queda como contexto: diseño, simulación y la historia
> de por qué se habla así.

> [!warning] Todo lo de la parte 2 MUEVE los dedos
> La activación hace un recorrido COMPLETO de calibración nada más
> alimentarla. Nada de esta guía se prueba sin la pinza despejada y con
> alguien delante.

## 1. Cómo habla

| Qué | Valor |
| --- | --- |
| Protocolo | Modbus RTU sobre RS-485 |
| Puerto físico | Pines 1 (`485A`) y 2 (`485B`) de la brida; alimentación en el pin 5 (24V) |
| Serie | 115200 bps, 8 bits, **sin paridad**, 1 stop |
| Slave ID | **9** |
| Petición de acción (escritura) | `0x03E8` = **1000**, 3 registros |
| Estado (lectura, FC03) | `0x07D0` = **2000**, 3 registros |

La pinza usa 6 BYTES en cada dirección, y Modbus habla de registros de 16
bits: cada registro son **2 bytes, el primero en la parte alta**
(`reg = (byte_par << 8) + byte_impar`). De ahí salen los 3 registros.

## 2. Mandarla desde el CR5 (sin driver extra)

El controlador del CR5 hace de maestro Modbus, así que no hace falta ni un
USB-485 ni `pymodbus`: valen los comandos del puerto 29999 que ya habla
[[_cr5_protocol (protocolo TCP del CR5)]].

```
SetToolMode(1)                           -> pines 1-2 en modo 485
SetTool485(115200,"N",1)                 -> formato serie de la brida
ModbusCreate("127.0.0.1",60000,9,1)      -> 0,{idx}   (idx = índice del maestro)
...
ModbusClose(idx)
```

> [!important] `ModbusCreate` por el 60000, NO `ModbusRTUCreate` (29/09)
> El puerto 60000 del controlador es el paso directo al 485 de la brida.
> Con `ModbusRTUCreate` la pinza no contestó nunca (17/09-29/09); con esta
> vía respondió a la primera. Fuente: ejemplo oficial Dobot+ "Control End
> Gripper" (Robotiq EPick) y la doc V3 del protocolo; el manual V4.6.5 no
> lo dice. Ver [[2026-09-29]].

### Los tres registros de acción

Bytes: `[rACT | rGTO<<3 | rATR<<4] [0] [0] [rPR] [rSP] [rFR]`, donde `rPR`
es la posición pedida (0 = abierta, 255 = cerrada), `rSP` la velocidad y
`rFR` la fuerza (0-255 las tres). Empaquetados en U16:

| Orden | `SetHoldRegs(idx, 1000, 3, {...}, U16)` | Qué hace |
| --- | --- | --- |
| Reset | `{0, 0, 0}` | `rACT=0`, borra el estado |
| **Activar** | `{256, 0, 0}` | `rACT=1` → calibración de recorrido completo |
| Abrir | `{2304, 0, 32896}` | `rACT+rGTO`, `rPR=0`, `rSP=rFR=128` |
| Cerrar | `{2304, 255, 32896}` | igual con `rPR=255` |
| Medio | `{2304, 128, 32896}` | `rPR=128` |

(2304 = `0x0900` = `rACT=1, rGTO=1`; 32896 = `0x8080` = velocidad 128,
fuerza 128. Para otra velocidad/fuerza: `rSP*256 + rFR`.)

`SetHoldRegs` admite `count` entre 1 y 4, así que los 3 registros entran
en una sola llamada.

### Leer el estado

```
GetHoldRegs(idx, 2000, 3)   ->  0,{reg0,reg1,reg2}
```

Y se descompone así:

| Dato | De dónde | Significado |
| --- | --- | --- |
| `gACT` | `(reg0>>8) & 0x01` | activada |
| `gGTO` | `(reg0>>8>>3) & 0x01` | ejecutando una orden |
| `gSTA` | `(reg0>>8>>4) & 0x03` | 0 = reset, 1 = activándose, **3 = activación completada** |
| `gOBJ` | `(reg0>>8>>6) & 0x03` | 0 = en movimiento, 1 = **objeto detectado abriendo**, 2 = **objeto detectado cerrando**, 3 = llegó a la posición pedida |
| `gFLT` | `reg1>>8` | código de fallo (tabla abajo) |
| `gPR` | `reg1 & 0xFF` | eco de la posición pedida |
| `gPO` | `reg2>>8` | posición real, 0-255 |
| `gCU` | `reg2 & 0xFF` | corriente, ≈ `gCU * 10` mA |

`gOBJ` es lo interesante para agarrar: **1 o 2 significa "he pillado
algo"**, 3 significa "cerré del todo sin encontrar nada".

### Secuencia mínima

1. `SetToolPower(1)` si la pinza estaba sin alimentar (ver la otra guía).
2. `SetToolMode(1)`, `SetTool485(115200,"N",1)` y
   `ModbusCreate("127.0.0.1",60000,9,1)`.
3. `GetHoldRegs(idx,2000,3)` — responde en ~0,02 s. Un `-1` en ~0,5 s es
   que no contesta nadie (alimentación, cable).

   Verificado el 29/09: cierra/abre en 1,8 s a vel/fuerza 64, topes reales
   `gPO` 3 (abierta) y 228 (cerrada sin objeto). Con `gFLT=0x09` activo
   acepta igualmente una orden y el fallo se borra.
4. Reset `{0,0,0}` → activar `{256,0,0}` → **esperar a que `gSTA` valga 3**
   (tarda unos segundos y mueve los dedos de tope a tope).
5. Abrir/cerrar con `{2304, rPR, rSP*256+rFR}`, esperando a `gOBJ != 0`.
6. `ModbusClose(idx)` al terminar.

### Códigos de fallo (`gFLT`)

Verificados contra el manual oficial (§4.4, "Robot Input Registers &
Status"):

| `gFLT` | Qué pasa | LED |
| --- | --- | --- |
| `0x00` | sin fallo | azul |
| `0x05` | acción retrasada: falta completar la (re)activación | azul |
| `0x07` | hay que poner el bit de activación antes de mandar nada | azul |
| `0x08` | temperatura máxima superada, esperar a que enfríe | rojo fijo |
| `0x09` | **más de 1 s sin comunicación** | rojo fijo |
| `0x0A` | tensión por debajo del mínimo | rojo/azul parpadeando |
| `0x0B` | liberación automática en curso | rojo/azul parpadeando |
| `0x0C` | fallo interno (soporte Robotiq) | rojo/azul parpadeando |
| `0x0D` | fallo de activación: mirar si algo interfiere | rojo/azul parpadeando |
| `0x0E` | sobrecorriente | rojo/azul parpadeando |
| `0x0F` | liberación automática completada | rojo/azul parpadeando |

Los `0x0A`-`0x0F` exigen **reset** (flanco de subida de `rACT`) para
salir. El `0x09` es el que saldría si el maestro Modbus del CR5 deja de
refrescar: el manual recomienda mandar/leer del orden de 200 Hz.

> [!info] Endianness — CONFIRMADO el 29/09: byte alto primero
> (`12544 = 0x3100` = activada, gSTA=3). Lo que sigue es el porqué de la duda.
>
> Endianness
> El manual avisa de que, aunque Modbus RTU es big endian para las
> DIRECCIONES, "the data port is in the case of Robotiq products based on
> the Little Endian byte order". El driver ROS de referencia
> (`robotiq_modbus_rtu`) empaqueta sin embargo `(byte_par << 8) +
> byte_impar`, que es lo que asume esta guía. Si en la primera lectura
> real los campos salen cruzados (p. ej. `gPO` donde esperas `gFLT`),
> invierte el orden dentro de cada registro y listo.

## ¿2F-85 o 2F-140? Casi seguro que la 85

El TCP que el CR5 tiene calibrado en el frame 1 sobresale **152.85 mm**
(ver [[E-S del Extremo del CR5 (pinza)]]). Según las especificaciones
mecánicas del manual (§6.2), la altura máxima es **162.8 mm en la 2F-85**
y 232.8 mm en la 2F-140: el número encaja con la 85 y descarta la 140.

Otro dato de esa tabla que resuelve un pendiente del ROADMAP: la 2F-85
**pesa 925 g** (la 140, 1025 g). Es lo que hay que meter en
`PayLoad(peso, excentricidad)`, hoy a 0 en el controlador — y encaja con
la estimación de 1-2 kg que salió de los pares articulares al bajar el TCP
a 40 cm (ver [[2026-09-17]]).

| 2F-85 | Valor |
| --- | --- |
| Apertura | 85 mm |
| Peso | 925 g |
| Fuerza de agarre | 20-235 N |
| Velocidad de dedos | 20-150 mm/s |
| Resolución de posición | 0.4 mm |

## 2b. Hablarle SIN el robot: `docs/robotiq_usb485_probe.py`

Herramienta de laboratorio (vive en `docs/`, como `stub_node.py`: no es
infraestructura). Habla Modbus RTU a mano —30 líneas, sin `pymodbus`— por
un conversor RS-485/USB, saltándose el CR5 por completo.

```
python3 docs/robotiq_usb485_probe.py --port /dev/ttyUSB0      # estado de la pinza
python3 docs/robotiq_usb485_probe.py --port /dev/ttyUSB0 --ft # el sensor FT 300
python3 docs/robotiq_usb485_probe.py --autotest               # sin hardware
```

Para qué sirve, más allá de la depuración de hoy: en un conversor los
hilos van a **bornes de tornillo**, así que permite probar las dos
polaridades del par 485 en un minuto — que es la incógnita que quedó
abierta el 18/09.

Lo que distingue este script de los sondeos vía CR5 es que **ve la
respuesta cruda**, y eso desambigua el `-1` del controlador:

| Lo que ve | Qué significa |
| --- | --- |
| Silencio | no llega nada: polaridad, cableado o alimentación |
| CRC incorrecto | **hay comunicación pero llega corrupta** (velocidad, ruido) |
| Excepción Modbus | **el dispositivo RESPONDE**: el bus va bien, falla la dirección |
| Registros | todo correcto, y los decodifica |

Esa distinción es justo la que `GetHoldRegs` del CR5 no puede dar: colapsa
los cuatro casos en un `-1`.

`--activar` y `--apertura` MUEVEN los dedos y exigen además `--si-se-mueve`
para que no se escape por descuido. El rango se valida antes de abrir el
puerto (mismo criterio que `Cr5RealRobotAdapter.set_joints`: si el dato es
inválido, no se toca el hardware).

`--autotest` no necesita ni conversor ni pinza: valida el CRC contra las
tramas de ejemplo **de los manuales de Robotiq** (`09 03 00 B4 00 01 C5 64`
del FT 300 y `09 10 03 E8 00 03 06 ... 73 30` de la 2F) y hace un ida y
vuelta completo contra un esclavo de mentira por un pty. Necesita
`pyserial` (`pip install --user pyserial`, ya instalado el 18/09).

## 2c. Ver la señal por fuera: `docs/cr5_485_scope_test.py`

La segunda herramienta de laboratorio (21/09). Hace que el CR5 **transmita
de forma repetitiva** por el 485 de la brida, para observarlo con un
osciloscopio.

```
python3 docs/cr5_485_scope_test.py                 # 115200, como la pinza
python3 docs/cr5_485_scope_test.py --baud 9600     # 10x más lento, más fácil de ver
python3 docs/cr5_485_scope_test.py --dry-run       # solo los cálculos, sin tocar el robot
```

**Qué hace, paso a paso:** pide el control, fuerza el terminal a modo 485
(`SetToolMode(1)`), fija el formato (`SetTool485`), alimenta el extremo,
crea un maestro Modbus RTU y **repite la misma petición de lectura** hasta
que se acaba `--duracion`. Cada petición son 8 bytes por el par: eso es lo
que se ve en la pantalla. Al terminar cierra el maestro.

**Qué imprime antes de tocar nada** (y por eso `--dry-run` es útil por sí
solo): la trama exacta que va a salir con su CRC, la duración de un bit y
de la trama entera a esa velocidad, **el primer byte desglosado en bits tal
y como viaja** (arranque a 0, los 8 bits empezando por el menos
significativo, parada a 1), los ajustes sugeridos de base de tiempos y
disparo, y la tabla de interpretación.

A 115200 la trama es `09 03 07 D0 00 03 04 0E`, un bit dura 8,68 µs y la
trama 694 µs. Con `--baud 9600` pasan a 104 µs y 8 ms, cómodo en cualquier
osciloscopio; la pinza no responderá a esa velocidad, pero para esta medida
solo importa lo que SALE del robot.

**Dónde pinchar** (brida M8 de 8 polos, colores de la tabla 3.6 del manual
de hardware): CH1 al pin 1 (`485A`, blanco), CH2 al pin 2 (`485B`, marrón),
masa al pin 8 (`GND`, rojo), y **MATH = CH1 − CH2**: la señal RS-485 es la
diferencia entre los dos hilos, no cada uno por separado.

| Lo que se vea | Conclusión |
| --- | --- |
| Línea plana | **El CR5 no transmite** → avería del controlador; cable y pinza absueltos |
| Ráfagas con esa forma | El CR5 transmite bien → el fallo está en la pinza o en el tramo final |
| Ráfagas invertidas (MATH bajo en reposo) | **Líneas cruzadas** → confirmada la inversión de polaridad |
| Amplitud < 0,5 V | Driver débil, o línea cargada/cortada |

Es la única medida encontrada que **resuelve las dos incógnitas a la vez**
—si transmite y con qué polaridad—, porque en reposo la diferencia debe
estar ALTA y eso se lee directamente en pantalla.

## 3. Dónde encajaría en el stack (hexagonal)

*(Escrito antes del 18/09; hoy ya existen `GripperPort` y
`Robotiq2FGripperAdapter`, ver [[Pinza Robotiq 2F - Uso práctico]].)* Lo coherente con
[[Puertos y Adaptadores]] es un puerto propio, no ampliar
[[RobotConnectorPort]]: agarrar no es mover articulaciones, y hay robots
sin pinza.

```python
# shared_kernel/ports.py
class GripperPort(Protocol):
    def activate(self) -> None: ...
    def set_opening(self, fraction: float) -> None:   # 0.0 abierta, 1.0 cerrada
        ...
    def get_state(self) -> GripperState: ...          # posición + si agarra algo
    def close(self) -> None: ...
```

Con dos adaptadores, mismo patrón que robot real/simulado:

- `Robotiq2FCr5Adapter` (en `robot_node/adapters/`) — habla por el
  `Cr5CommandSocket` que ya existe, con los comandos de la parte 2.
  Reutiliza la conexión del 29999: recuerda que **ese puerto admite un
  solo cliente**.
- `CoppeliaSimGripperAdapter` (en `robot_node/adapters/`) — mueve el
  modelo de la simulación (parte 4).

Un `GripperState` con `opening` (0-1, de `gPO`) y `holding_object` (de
`gOBJ`) basta para empezar, y deja fuera del dominio los detalles Modbus.

## 4. Meterla en la simulación

### Opción rápida: el modelo que ya trae CoppeliaSim

CoppeliaSim incluye el 2F-85 (comprobado en esta máquina):

```
~/RoboticInvest/CoppeliaSim/models/components/grippers/ROBOTIQ 85.ttm
```

En [[Herramientas de CoppeliaSim]], `build_scene` ya deja el CR5 montado y
conoce el tip (`Link6_visual`), así que basta cargarlo y colgarlo de la
punta:

```python
handle = sim.loadModel(ROBOTIQ_TTM)
sim.setObjectParent(handle, tip_handle, True)   # True = mantener pose mundo
sim.setObjectPose(handle, tip_handle, [0,0,0, 0,0,0,1])  # pegado a la brida
```

Aviso honesto: **no he podido inspeccionar cómo se controla ese modelo**
(el `.ttm` está comprimido y no se deja leer sin abrir CoppeliaSim). Los
modelos de pinza de CoppeliaSim suelen traer un script hijo que escucha
una señal, pero puede que este mueva sus juntas directamente. Al abrirlo,
mirar su script; si no hay señal clara, el camino que funciona siempre es
coger sus juntas con `sim.getObjectsInTree(handle, sim.object_joint_type)`
y mandarles posición objetivo.

### Opción buena a medio plazo: que entre en el URDF

La opción de arriba es solo visual: para el resto del stack la pinza
**sigue sin existir**. Hoy eso ya se nota en tres sitios:

- La IK resuelve la BRIDA, no la punta de la pinza — por eso bajar el TCP
  a 40 cm necesitó corregir a mano el desfase de 16 cm (ver
  [[2026-09-17]]).
- La autocolisión de [[SelfCollisionAwarePlanningAdapter]] no ve los 16 cm
  de pinza: el eslabón más expuesto del brazo es justo el que no está
  modelado.
- `load` = 0 kg en el controlador, con la masa real colgando.

Como `build_scene` importa **cualquier** URDF (generalizado en el Bloque 9,
verificado con un Panda), lo natural es coger el URDF del 2F-85
(`robotiq_2f_85_gripper_description`, de ROS) y unirlo al
`cr5_robot.urdf` con un joint fijo de `Link6` a la base de la pinza. Con
eso, gratis y a la vez: FK hasta la punta real, `link_poses` con la pinza
dentro, autocolisión que la tiene en cuenta, y simulación fiel. El dedo
móvil sería un joint más, que el `CoppeliaSimGripperAdapter` mueve.

## Ver también

- [[E-S del Extremo del CR5 (pinza)]] — el conector de 8 pines y el estado del sondeo
- [[_cr5_protocol (protocolo TCP del CR5)]] — los comandos del 29999
- [[Herramientas de CoppeliaSim]] — `build_scene`, dónde engancharla
- [[Conectar un Robot Nuevo]] — el patrón de puerto + adaptadores
- Manual oficial (PDF, verificado): `~/Downloads/manuales_robotiq/2F-85_2F-140_Instruction_Manual_e-Series_PDF_20190206.pdf`
  — §4.2 mapa de registros, §4.3/§4.4 bits de cada byte, §4.7 parámetros
  Modbus RTU, §6.2 especificaciones mecánicas. Fuente:
  <https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_e-Series_PDF_20190206.pdf>

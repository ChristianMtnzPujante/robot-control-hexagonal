---
tags: [guia, cr5, pinza, robotiq, referencia]
---

# Pinza Robotiq 2F: uso práctico

Chuleta para usar la pinza del laboratorio: cómo conectarse, qué órdenes
acepta y cómo leer su estado. Todo lo de aquí está **verificado contra la
pinza real el [[2026-09-29]]**, salvo donde se indica. El porqué y la
historia están en [[Manejar la pinza Robotiq 2F]] y en
[[E-S del Extremo del CR5 (pinza)]].

> [!warning] Activar, cerrar y abrir MUEVEN los dedos
> Pinza despejada y alguien delante.

## Qué es y cómo se habla con ella

- **Robotiq 2F-85** en la brida del CR5 (conector M8 de 8 polos).
- **Modbus RTU** por el RS-485 de la brida: **esclavo 9, 115200 bps, 8N1**.
- El maestro Modbus es el **controlador del CR5**. Desde el PC solo se le
  mandan comandos de texto por **TCP a 192.168.5.1:29999**. No hace falta
  USB-485, Lua ni plugins.
- El 29999 admite **un solo cliente**: los scripts de `docs/` y
  `robot_node` no pueden estar conectados a la vez.

## 1. Conectarse

```
RequestControl()                      → aceptar órdenes por TCP
SetToolPower(1)                       → 24 V a la brida (pin 5); suele cortar el TCP: reconectar
SetToolMode(1)                        → pines 1-2 en modo 485 (no analógico)
SetTool485(115200,"N",1)              → formato serie del 485 de la brida
ModbusCreate("127.0.0.1",60000,9,1)   → 0,{idx}   ← abre el maestro; guardar idx
...
ModbusClose(idx)                      → al terminar
```

> [!important] `ModbusCreate("127.0.0.1",60000,9,1)`, NO `ModbusRTUCreate`
> El puerto 60000 es un servicio interno del propio controlador
> (`127.0.0.1` = el controlador, no el PC) que reenvía las tramas al 485
> de la **brida**. El último `1` = formato RTU.
> `ModbusRTUCreate` abre un maestro sobre otro RS-485, probablemente el del
> armario (**sin confirmar**). Con él la pinza no contestó nunca
> (17/09-29/09, siempre `-1` en ~0,5 s). La trama es la misma: cambia por
> qué cable sale.
> Fuente: ejemplo oficial Dobot+ "Control End Gripper" (una Robotiq EPick)
> y la doc V3 del protocolo ("60000 terminal transparent port"). El manual
> V4.6.5 no lo dice.

- Solo caben **5 maestros** abiertos. Si `ModbusCreate` falla, liberar con
  `ModbusClose(0)` … `ModbusClose(4)` (quedan abiertos si un script muere
  sin cerrar).
- `Use485()` / `UnLock485()` del ejemplo de Dobot+ son funciones **Lua**
  (dentro del controlador), no comandos del 29999: no se usan.

## 2. Órdenes: registro 1000, 3 registros

```
SetHoldRegs(idx,1000,3,{R0,R1,R2},U16)

R0 = byte0 << 8      byte0 = rACT(bit0) | rGTO(bit3) | rATR(bit4) | rARD(bit5)
R1 = rPR             posición pedida: 0 = abierta … 255 = cerrada
R2 = rSP*256 + rFR   velocidad y fuerza, 0-255 cada una
```

| Orden | `{R0,R1,R2}` | Qué hace |
| --- | --- | --- |
| Reset | `{0,0,0}` | rACT=0. Borra fallos. Hace falta antes de activar (flanco de subida) |
| **Activar** | `{256,0,0}` | rACT=1. Calibración: recorrido completo de tope a tope. **Solo si no está activada**: suelta lo que tenga agarrado |
| **Cerrar** | `{2304,255,32896}` | rACT+rGTO, posición 255, vel 128, fuerza 128 |
| **Abrir** | `{2304,0,32896}` | posición 0 |
| Ir a posición P | `{2304,P,V*256+F}` | medio: `{2304,128,32896}`; lento y suave (64/64): `{2304,255,16448}` |
| Parar | `{256,P,V*256+F}` | rGTO=0: se detiene donde esté (*sin probar*) |
| Liberación automática abriendo | `{12544,0,0}` | rACT+rATR+rARD (0x31). **Solo emergencias**; después exige reset + activar (*sin probar*) |
| Liberación automática cerrando | `{4352,0,0}` | rACT+rATR (0x11). Igual (*sin probar*) |

2304 = `0x0900` (rACT+rGTO) · 256 = `0x0100` (rACT) · 32896 = `0x8080`
(128/128) · 16448 = `0x4040` (64/64).

## 3. Estado: registro 2000, 3 registros

```
GetHoldRegs(idx,2000,3)   → 0,{reg0,reg1,reg2}     responde en ~0,02 s
```

Byte alto primero dentro de cada registro (confirmado).

| Dato | Cálculo | Significado |
| --- | --- | --- |
| gACT | `(reg0>>8) & 1` | activada |
| gGTO | `(reg0>>11) & 1` | ejecutando una orden |
| gSTA | `(reg0>>12) & 3` | 0 reset · 1 activándose · **3 activación completada** |
| gOBJ | `(reg0>>14) & 3` | 0 moviéndose · **1 objeto al abrir · 2 objeto al cerrar** · 3 llegó sin objeto |
| gFLT | `(reg1>>8) & 0x0F` | fallo (tabla abajo). Bits 4-7 = kFLT, controlador opcional de Robotiq (aquí 0) |
| gPR | `reg1 & 0xFF` | eco de la posición pedida: sirve de acuse de la orden |
| gPO | `reg2>>8` | posición real. **En esta pinza: 3 abierta, 228 cerrada sin objeto** |
| gCU | `reg2 & 0xFF` | corriente ≈ gCU × 10 mA (en vacío ≤ 110 mA) |

Lecturas reales: `{12544,0,768}` = activada, sin fallo, abierta (gPO 3).
`{256,2304,768}` = sin activar, gFLT 0x09.

## 4. Fallos (gFLT)

| gFLT | Qué pasa | Cómo se quita |
| --- | --- | --- |
| 0x00 | sin fallo (LED azul) | — |
| 0x05 / 0x07 | falta completar la activación / falta rACT | activar |
| 0x08 | temperatura | esperar |
| **0x09** | más de 1 s sin comunicación (LED rojo fijo). **Sale siempre que nadie le habla** | solo: acepta órdenes igual y se borra (verificado) |
| 0x0A-0x0F | graves: tensión baja, liberación automática, fallo interno, fallo de activación, sobrecorriente (LED rojo/azul parpadeando) | **reset + activar** |

## 5. Forma de trabajar

Criterios del SDK oficial de Robotiq (`robotiq/grippers`):

- Activar **solo si hace falta**: si gSTA=3 y no hay fallo grave, no
  reactivar.
- Cada movimiento: mandar la orden → esperar el acuse (gGTO=1 y gPR = lo
  pedido) → esperar el fin (gOBJ ≠ 0) → mirar gOBJ y gFLT.
- Fallos graves: reset + activar. Los menores se borran solos.
- Tiempos de referencia: a vel/fuerza 64 cierra o abre en **1,8 s**.

## 6. Desde nuestro código

- **ROS** (verificado 29/09; hace falta `-p cr5_host:=192.168.5.1`, el
  valor por defecto es otra IP):
  ```
  ros2 run robot_node robot_node --ros-args -p robot_target:=real -p cr5_host:=192.168.5.1 -p gripper_target:=robotiq_2f
  ros2 topic pub --once /gripper_activate std_msgs/msg/Empty {}
  ros2 topic pub --once /gripper_command std_msgs/msg/Float64 "data: 1.0"   # 0.0 abierta … 1.0 cerrada
  ```
- **Adaptador**: `src/robot_node/robot_node/adapters/robotiq_2f_adapter.py`.
  Usa la vía del 60000; `activate()` no reactiva si ya está activa;
  `opening` = gPO/255, así que cerrada del todo da ≈ 0,89.
- **Secuencia brazo + pinza** (verificada 29/09):
  `ros2 run commander lift_and_grip_demo --phase real --host 192.168.5.1`
  sube el brazo 5 cm y después abre y cierra la pinza (`--phase plan`
  para ver el cálculo sin mover nada).
- **Scripts de laboratorio** (`docs/`):
  - `cr5_485_scope_test.py`: lecturas repetidas (`--via brida|rtu`).
  - `cr5_controller485_probe.py`: el 485 del armario.
  - `robotiq_usb485_probe.py`: la pinza por un USB-485, sin robot.

## Pendiente

- Nada pendiente del camino ROS salvo `gripper_state`.
- Publicar `gripper_state` (no existe todavía).
- Decidir si `opening` se normaliza a los topes reales 3-228.
- Confirmar a qué 485 va `ModbusRTUCreate`.

## Fuentes

- Robotiq, [Controlling a Robotiq Gripper with a Dobot Robot](https://blog.robotiq.com/knowledge/robotiq-grippers-and-dobot-robots-compatibility)
  — kit `AGC-CRX-KIT-85`; plugin de Dobot preinstalado en DobotStudio Pro;
  [documentación del plugin](https://share.robotiq.com/public/74b3f) (solo
  se abre en navegador).
- [SDK oficial de Robotiq](https://github.com/robotiq/grippers).
- [Dobot+ "Control End Gripper"](https://toadyokai.github.io/dobotplus/en/examples/Basic/grip).
- Manual Robotiq 2F-85/2F-140 e-Series, §4: `~/Downloads/manuales_robotiq/`.

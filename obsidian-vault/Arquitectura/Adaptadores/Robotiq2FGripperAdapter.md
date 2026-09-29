---
tags: [arquitectura, adaptador, pinza, robotiq]
---

# Robotiq2FGripperAdapter

Implementa [[GripperPort]] para la **Robotiq 2F-85** montada en la brida
del CR5. Código:
`src/robot_node/robot_node/adapters/robotiq_2f_adapter.py`. Tests:
`src/robot_node/test/test_robotiq_2f_adapter.py` (19).

## Cómo habla

No abre ninguna conexión propia. Recibe en el constructor el
`Cr5CommandSocket` de [[Cr5RealRobotAdapter]] (propiedad
`command_socket`), porque el puerto 29999 admite un solo cliente. Por
ese socket le pide al controlador del CR5 que haga de maestro Modbus
con los comandos de [[_cr5_protocol (protocolo TCP del CR5)]]:

| Método | Comandos al controlador |
| --- | --- |
| (primer uso) | `SetToolMode(1)`, `SetTool485(115200,"N",1)`, **`ModbusCreate("127.0.0.1",60000,9,1)`** |
| `activate()` | `GetHoldRegs` y, solo si hace falta, `SetHoldRegs(idx,1000,3,{0,0,0})` → `{256,0,0}` |
| `set_opening(f)` | `SetHoldRegs(idx,1000,3,{2304, round(f·255), vel·256+fuerza},U16)` |
| `get_state()` | `GetHoldRegs(idx,2000,3)` → `GripperState` |
| `close()` | `ModbusClose(idx)` |

> [!important] `ModbusCreate` por el 60000, no `ModbusRTUCreate`
> El puerto 60000 del controlador reenvía las tramas al 485 de la
> brida. Con `ModbusRTUCreate` la pinza no contestó nunca. Detalle y
> fuentes en [[Pinza Robotiq 2F - Uso práctico]].

## Decisiones dentro del adaptador

- **No sondea por su cuenta**: solo habla cuando se le llama. El socket
  es secuencial y compartido con el brazo, así que un `GetHoldRegs`
  retrasaría un `MovJ`. El fallo 0x09 ("1 s sin comunicación") que eso
  provoca no impide aceptar órdenes (verificado 29/09).
- **`activate()` no reactiva** una pinza activada y sin fallo grave
  (`gFLT < 0x0A`), igual que el SDK oficial de Robotiq. Reactivar la
  abre y cierra entera y soltaría lo agarrado. Con un fallo grave hace
  reset + activación, que es lo que lo borra.
- **`fault_code` = bits 0-3 del byte 2** (`gFLT`). Los bits 4-7 son
  `kFLT`, del controlador opcional de Robotiq.
- **Byte alto primero** en cada registro (confirmado con la pinza real).
- **No llama a `SetToolPower`**, porque corta la conexión TCP.
- **Tras un error de la pinza, deja de intentarlo** (29/09). Si el
  controlador contesta con error a cualquier comando de la pinza (el `-1`
  de una pinza ausente o sin alimentar), el adaptador la marca como **no
  disponible** y las llamadas siguientes fallan al instante sin tocar el
  socket. Motivo: el socket es el del brazo; cada `-1` lo bloquea unos
  0,5 s y una ráfaga de fallos llegó a resetear la conexión (17/09).
  `close()` quita la marca para reintentar. Un fallo de la **conexión**
  (no de la pinza) no marca nada, porque afecta igual al brazo.

## Estado

- Verificado contra la pinza real el 29/09 dentro de
  `lift_and_grip_demo.py`: lectura, activación (no-op), abrir y cerrar,
  compartiendo socket con el brazo.
- `opening` = gPO/255, así que cerrada del todo da ≈ 0,89 (tope real
  228). Pendiente decidir si se normaliza.
- Sin probar todavía a través de los topics de [[RobotNode]].

## Ver también

- [[Integración de la Pinza]]
- [[Manejar la pinza Robotiq 2F]] · [[E-S del Extremo del CR5 (pinza)]]

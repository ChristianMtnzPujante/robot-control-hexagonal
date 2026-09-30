---
tags: [arquitectura, puerto, pinza]
---

# GripperPort

> `activate() -> None`
> `set_opening(fraction: float) -> None`
> `get_state() -> GripperState`
> `close() -> None`

`shared_kernel/ports.py`: una pinza. Abrir, cerrar y decir qué está
pasando. Es un puerto **aparte** de [[RobotConnectorPort]], porque
agarrar no es mover articulaciones y hay robots sin pinza.

- `activate()`: deja la pinza lista. **Si ya lo está, no hace nada.** Si
  no, en la Robotiq 2F hace una calibración que mueve los dedos de tope
  a tope y suelta lo que tuviera agarrado.
- `set_opening(fraction)`: 0.0 abierta, 1.0 cerrada. Vuelve en cuanto la
  orden se acepta, **no** cuando el movimiento termina; para eso,
  consultar `get_state()`.
- `get_state()`: devuelve un `GripperState` (`value_objects.py`) con
  `opening`, `activated`, `holding_object` y `fault_code`.
  `GripperState` no es un `Either` como `JointConfiguration`: es una
  lectura ya hecha, no un dato de entrada que validar.
- `close()`: libera lo que el adaptador tenga abierto (en la Robotiq,
  el maestro Modbus).

Los fallos se reportan como `RobotConnectorError`, la misma familia que
el puerto del robot. Hoy la única implementación habla por el socket
del CR5, cuyo `Cr5ProtocolError` ya es un `RobotConnectorError`. Si
algún día una pinza llega por un canal independiente, merecerá su propio
`GripperError`.

## Adaptadores

- [[Robotiq2FGripperAdapter]]: Robotiq 2F a través del controlador del
  CR5. **Verificado contra la pinza real (29/09).**
- [[CoppeliaSimGripperAdapter]]: la misma 2F-85 importada desde su URDF
  en CoppeliaSim, en modo cinemático. **Verificado en CoppeliaSim
  (30/09).**

## Quién lo usa

- [[RobotNode]], con `gripper_target: robotiq_2f`. Topics
  `gripper_activate` y `gripper_command`.
- `commander/lift_and_grip_demo.py` ([[Scripts de Demostración]]): fase
  real con la Robotiq y, desde el 30/09, fase `sim` con la simulada.
- `commander/cr5_gripper_sim_demo.py`: CR5 + 2F-85 solo en simulación.

## Ver también

- [[Integración de la Pinza]]: cómo entró todo esto en el repositorio.
- [[Puertos y Adaptadores]]

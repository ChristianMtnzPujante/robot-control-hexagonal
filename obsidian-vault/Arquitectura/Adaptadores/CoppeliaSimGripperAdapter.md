---
tags: [arquitectura, adaptador, pinza, robotiq, simulacion]
---

# CoppeliaSimGripperAdapter

Implementa [[GripperPort]] para una pinza **importada desde URDF en
CoppeliaSim**. Hoy es la Robotiq 2F-85, la misma que lleva el CR5 real
([[Robotiq2FGripperAdapter]]). Código:
`src/robot_node/robot_node/adapters/coppeliasim_gripper_adapter.py`.
Tests: `src/robot_node/test/test_coppeliasim_gripper_adapter.py` (7).
**Verificado en CoppeliaSim el 30/09** con `cr5_gripper_sim_demo`.

## De dónde sale la pinza

- **URDF y mallas:** `assets/robotiq_2f_85/`, copiados del paquete oficial
  `robotiq_description` (PickNik, BSD). Se usó ese paquete porque el
  `ROBOTIQ 85.ttm` de CoppeliaSim es un binario que no se puede
  inspeccionar sin tener el simulador en marcha. La procedencia está en su
  README.
- **Montaje:** lo hace `coppeliasim_scene_builder` con
  `mounts=[ROBOTIQ_2F_85_ON_CR5]`. La pinza cuelga de `joint6` (brida) sin
  acoplador; ver [[Herramientas de CoppeliaSim]].

## Cómo mueve los dedos

La 2F-85 tiene **un joint que se manda**
(`robotiq_85_left_knuckle_joint`, de 0 = abierta a 0,8 rad = cerrada) y
cinco `<mimic>` que lo copian con multiplicador ±1. En modo cinemático
CoppeliaSim no propaga los mimic, así que el adaptador escribe los seis
joints con `setJointPosition`. `gripper_joints_from_urdf()` lee el joint,
el límite y los multiplicadores **del propio URDF**, sin copiarlos a mano.

| Método | Qué hace |
| --- | --- |
| `activate()` | Nada: no hay calibración que simular |
| `set_opening(f)` | Anima de la apertura actual a `f` en 15 pasos y **vuelve al terminar** |
| `get_state()` | `opening` = ángulo / 0,8; `activated=True`, `holding_object=False`, `fault_code=0` |
| `close()` | Nada: la conexión es de quien construyó la escena |

## Diferencias con la pinza real, a propósito

- **`set_opening` bloquea hasta terminar.** La real vuelve en cuanto
  acepta la orden. Sin física no hay nada que siga moviendo los dedos
  después, y quien sondea `get_state()` (como `lift_and_grip_demo`) ve lo
  mismo que acabaría viendo con la real.
- **Sin contacto.** `holding_object` siempre es `False`. Agarrar objetos en
  simulación (física o "agarre falso" que pegue el objeto) es otro paso.

## Pendiente

- Cablearlo en [[RobotNode]] (`gripper_target` de simulación) para
  moverla por `/gripper_command`.
- El TCP de la pinza en la cinemática: PoE/GA siguen apuntando a la brida.

## Ver también

- [[Integración de la Pinza]]
- [[Decisiones de Diseño Clave]] (30/09: un URDF por pieza)
- [[CoppeliaSimRobotAdapter]]

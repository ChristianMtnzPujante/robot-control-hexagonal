---
tags: [arquitectura, adaptador]
---

# CoppeliaSimIkKinematicsAdapter

Implementa [[KinematicsPort]] delegando en el solver `simIK` del propio
CoppeliaSim — no matemática propia derivada a mano (eso es
[[PoeKinematicsAdapter]]/GA/DH). Código:
`src/controller_node/controller_node/adapters/coppeliasim_ik_adapter.py`.

Decisión consciente: rompe la promesa de `KinematicsPort` de ser
agnóstico de plataforma (SÍ depende de CoppeliaSim) a cambio de tener una
IK físicamente correcta ya, sin esperar a PoE/GA/DH.

## Entorno IK aislado — no mueve la escena real al calcular

`simIK.createEnvironment()` clona la cadena cinemática real
(`simIK.addElementFromScene`) pero sin tocar la escena visible al
resolver — el objetivo cartesiano se mueve siempre en el CLON, nunca en
la escena real. Así "calcular la trayectoria" no mueve el robot antes de
que empiece a enviarse de verdad por el pipeline normal (topics →
`robot_node`).

Solo restringe posición (`simIK.constraint_position`) — la orientación
del `Pose` objetivo se ignora.

## Convergencia

`simIK` resuelve por iteración LOCAL desde la configuración actual — para
saltos grandes puede no converger (límites de articulación, pasos
demasiado grandes). Si `handleGroup` no devuelve éxito, lanza
`RuntimeError` explícito en vez de una trayectoria hacia un sitio
equivocado — mismo criterio que [[PoeKinematicsAdapter]].

## Nombres de escena hardcodeados

`base_name="base_link_respondable"`, `tip_name="Link6_visual"` — objetos
de la escena `cr5_base.ttt`, puestos como default del constructor.
`controller_node` construye este adaptador sin argumentos
(`_build_adapter`), así que hoy no hay forma de cambiarlos por sesión sin
editar el archivo — pendiente de verdad, ver Bloque 9 en [[Estado del Roadmap]].

## Ver también

- [[KinematicsPort]]
- [[PoeKinematicsAdapter]]
- [[Commander y ControlSession]]

---
tags: [arquitectura, adaptador]
---

# CoppeliaSimRobotAdapter

Implementa [[RobotConnectorPort]] contra CoppeliaSim, vía su API remota
ZMQ directamente desde este proceso Python — sin pasar por ningún script
Lua embebido en la escena. Código:
`src/robot_node/robot_node/adapters/coppeliasim_adapter.py`.

## Decoración cosmética, NO parte del puerto

Métodos extra que usa `robot_node` solo si el adaptador los ofrece (duck
typing, `getattr(..., None)`):

- `mark_goal(goal)` — deja un dummy rojo en el objetivo cartesiano.
- (interno) trail de dummies azules por cada waypoint, si se le da
  `tip_name`.

Ninguno de los dos existe en [[Cr5RealRobotAdapter]] — son puramente
visuales, no algo que un robot real necesite.

## `scene_path` opcional

Si se le da, carga esa escena y arranca la simulación él mismo
(`_load_scene_and_play`). Sin él, asume que la escena ya está cargada y
en play — comportamiento de siempre, usado por
`coppeliasim_scene_builder.py` cuando la escena se construye por código
en vez de cargar un `.ttt`.

## `close()`

No-op deliberado, no un olvido: `RemoteAPIClient` no expone (ni se ha
verificado) un cierre propio, y la escena/simulación puede seguir
compartida por otras sesiones (`two_sessions_demo.py`) — cerrarla no
sería correcto aunque existiera el método. Existe solo para cumplir
[[RobotConnectorPort]] formalmente.

## Ver también

- [[RobotConnectorPort]]
- [[Cr5RealRobotAdapter]]
- [[Commander y ControlSession]]
- [[Herramientas de CoppeliaSim]] — `build_scene`/`build_cr5_scene`, cómo se construye la escena que este adaptador espera ya cargada

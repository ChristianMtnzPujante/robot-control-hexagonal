---
tags: [arquitectura, puerto]
---

# RobotConnectorPort

> `set_joints(configuration: JointConfiguration) -> None`
> `get_current_configuration() -> JointConfiguration`
> `close() -> None`

`shared_kernel/ports.py` — el "nodo robot": ejecuta comandos crudos sobre
un robot, simulado o real. Nunca calcula nada, solo obedece y reporta —
la cinemática (a dónde debe ir cada joint) la decide [[KinematicsPort]]
aguas arriba, en `controller_node`. `close()` es parte formal del
contrato desde el 07/09 (ver [[Decisiones de Diseño Clave]]): libera lo
que el adaptador tenga abierto y deja el robot en estado seguro si
aplica.

`RobotConnectorError` (mismo fichero) vive junto al puerto, no dentro de
un adaptador concreto — para que `robot_node` pueda capturar fallos de
cualquier adaptador sin conocer sus subclases (p. ej. `Cr5ProtocolError`).

## Adaptadores

- [[Cr5RealRobotAdapter]] — el CR5 físico real, protocolo TCP/IP directo.
  **Verificado contra el robot físico (07/09).**
- [[CoppeliaSimRobotAdapter]] — CoppeliaSim vía su API ZMQ.

## Cómo añadir un robot nuevo (guías)

Dos notas del vault, con el paso a paso concreto de qué hace falta para
dar de alta un `RobotConnectorPort` nuevo — no son teoría aparte, son la
referencia práctica de "toca aquí" para este puerto:

- **[[Conectar un Robot Nuevo]]** — el guion general (protocolo del
  fabricante + URDF como la combinación habitual, checklist físico/de
  red, dónde vive cada pieza) generalizado a partir de lo aprendido
  conectando el CR5 real — ver [[Cr5RealRobotAdapter]] para el caso
  concreto ya resuelto. Si ya es compatible con ros2 el trabajo se
  simplifica.

- **[[Anatomía de un Nodo]]** — el pipeline YAML → `node_config.py` →
  nodo → callbacks/`messages.py`: necesario para saber DÓNDE declarar el
  parámetro ROS2 nuevo que un adaptador nuevo probablemente necesite (p.
  ej. `cr5_host` en `robot_node.yaml` para [[Cr5RealRobotAdapter]]), no
  solo cómo escribir la clase del adaptador en sí.

## Ver también

- [[Puertos y Adaptadores]]
- [[Arquitectura Hexagonal]]

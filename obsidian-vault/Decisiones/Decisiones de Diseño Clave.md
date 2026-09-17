---
tags: [decisiones]
---

# Decisiones de Diseño Clave

Decisiones no triviales tomadas durante el proyecto, con fecha y motivo —
para no perder el "por qué" cuando el código ya solo muestra el "qué".
Fuente: `ROADMAP.md` y mensajes de commit. Orden cronológico, más recientes
al final. Añade una entrada nueva aquí (no una nota nueva) cada vez que
tomes una decisión de este tipo — ver [[Cómo usar este vault (Obsidian)]].

> [!question] (~01/09) `Scene.obstacles`: de `List` a `Dict[str, SphereObstacle]`
> Tomada al diseñar `perception_node`. Con nombre estable como clave, un
> productor externo puede releer su fuente entera en cada ciclo y
> reconstruir el diccionario sin llevar cuenta de qué era "nuevo" — y varias
> `Scene` parciales se combinan clave a clave vía `Scene.merge`. Ver
> [[Scene y Percepción]].

> [!question] (01/09) PoE y CGA son bounded contexts separados
> Cuando aterrice el álgebra geométrica conforme (Bloque 1), `Scene` y las
> primitivas de `geometry_kernel` **no** se reinterpretan por debajo con
> multivectores. En su lugar: una `Scene` conforme aparte, con sus propios
> tipos, y la tabla de traducción cartesiano→CGA como el punto único donde
> se traduce explícitamente entre ambas álgebras
> (`docs/algebra_geometrica_conforme.md` §2). Cada puerto consume la
> representación de su propia álgebra, no una forma neutra forzada entre
> las dos.

> [!question] (02/09→) Régimen 1 = LLM como consumidor de tools, no generador de código
> Revisión sobre el planteamiento original: el LLM no genera código libre
> que luego se ejecuta (no es *Code as Policies* literal). Actúa como
> consumidor de una API/tools que expone el propio backend
> (function-calling), eligiendo y encadenando llamadas a operaciones ya
> validadas del repo. Acota mucho la superficie de validación del Bloque 6
> y hace el sistema más entendible: lo que el LLM puede hacer está limitado,
> por diseño, a lo que la API expone.

> [!question] (04/09) Bloque 0 redefinido
> Deja de ser "la base teórica del sistema clásico" (esa base ya no depende
> de un robot fijo, ver Bloque 9) y pasa a ser la instanciación concreta y
> el ejemplo de uso real de la arquitectura hexagonal contra el CR5 —
> específico de este robot, no generalizable.

> [!question] (04/09) CR5 físico: TCP/IP directo, no puente ROS1
> Este entorno no tiene ROS1 Noetic instalado junto a ROS2 Humble (requisito
> duro de la vía puente, documentado en el propio `ros1_kit/bridge.py`),
> mientras que reimplementar el protocolo es puro Python/ROS2, sin
> dependencias nuevas. `ros1_kit/bridge.py` se deja tal cual, como boceto
> sin usar. Ver [[Arquitectura Hexagonal]].

> [!question] (04/09, corregida el mismo día) El protocolo real no es el del driver de 2021
> La primera versión de `Cr5RealRobotAdapter` se basó solo en el driver de
> referencia (`dobot_bringup`, `TCP-IP-ROS-6AXis`, fechado 2021) y asumía un
> puerto de movimiento aparte (30003) con `JointMovJ(...)`. Al leer el
> **manual oficial del fabricante** (2025, mucho más reciente que el
> driver) resultó que ese puerto no existe en el protocolo actual: solo hay
> 29999/30004/30005/30006, y el movimiento articular se manda como
> `MovJ(joint={...})` por el mismo puerto 29999 ("Dashboard"). El formato
> de la trama real-time (puerto 30004) sí coincidía entre driver y manual.
> Lección: preferir la fuente más reciente y oficial sobre un driver de
> referencia de terceros, aunque parezca más autorizado a primera vista.

> [!bug] (07/09, `e009321`) `ControlSession.stop()` dejaba nodos zombie
> `ros2 run` no hace `exec()` sobre el nodo real — el PID de `Popen` es el
> del lanzador, no el del nodo `rclpy`. Encontrado tras acumular media
> docena de nodos zombie en una sesión de trabajo, **dos con conexión TCP
> abierta al CR5 físico**. Fix: `start_new_session=True` al lanzar +
> `os.killpg` sobre el grupo de procesos entero en `stop()`. Ver
> [[Commander y ControlSession]].

> [!question] (07/09) `RobotConnectorPort` gana `close()` como parte formal del contrato
> Antes `close()` solo existía como método suelto en `Cr5RealRobotAdapter`,
> y `robot_node` no lo llamaba en ningún camino de cierre porque no
> formaba parte del contrato que `robot_node` conoce — cualquier adaptador
> sin nada que liberar (p. ej. `CoppeliaSimRobotAdapter`) lo implementa como
> no-op. Ver [[Puertos y Adaptadores]].

> [!question] `RobotControllerPort` renombrado a `RobotConnectorPort`
> "Controller" se confundía con `controller_node` (el nodo que de verdad
> calcula cinemática/planifica) — este puerto no controla nada, solo
> ejecuta comandos crudos sobre el robot y reporta ("obedece y reporta").
> Renombrado repo-wide (18 ficheros) para que el nombre deje claro que es
> el conector/ejecutor, no el cerebro.

> [!bug] (07/09) Los límites articulares de la URDF local NO eran de relleno
> Al diseñar la validación de límites de `Cr5RealRobotAdapter.set_joints`
> (ver más abajo), se descartaron en un primer momento los límites ya
> presentes en `~/ros2_ws/.../dobot_description/urdf/cr5_robot.urdf`
> (±360° la mayoría de joints, ±160° el codo) como "valores de relleno sin
> verificar". Error: verificados después contra TRES fuentes oficiales
> independientes (manual de usuario, manual de hardware, página de
> producto de Dobot), las tres coinciden exactamente con la URDF. Lección,
> en la línea de la de más abajo sobre el driver de 2021: **verificar
> contra la fuente oficial antes de descartar un dato existente como
> sospechoso**, no solo antes de confiar en uno nuevo.

> [!bug] (07/09) `MovJ` sin `cp` (suavizado) — vibración real en el robot físico
> `Cr5RealRobotAdapter.set_joints` mandaba `MovJ(joint={...})` sin el
> parámetro opcional `cp` (ratio de suavizado entre el movimiento actual y
> el siguiente de la cola, manual V4.6.5 §"平滑过渡参数", rango 0-100). Sin
> especificarlo, el robot usa 0 (sin suavizado) — el brazo para
> COMPLETAMENTE en cada uno de los 21 waypoints de una trayectoria antes
> de reanudar hacia el siguiente. Para un recorrido de pocos grados, eso
> es vibración y lentitud reales, reportadas por el usuario viendo el
> robot moverse. Fix: `cp=50` por defecto (valor intermedio deliberado,
> no 100 — con `cp>0` el robot no pasa exactamente por los puntos
> intermedios). El último waypoint de una trayectoria no se ve afectado
> (no hay comando siguiente con el que hacer transición).

> [!tip] (07/09) Errores de protocolo recurrentes que `ClearError()`/`Stop()` no arreglan de forma duradera → power-cycle
> Tras un día especialmente intenso probando contra el CR5 físico (corte
> de red, incidente de e-stop, procesos zombie con conexión TCP abierta),
> apareció un error "-7 robot en estado de script pausado" recurrente en
> casi todos los `MovJ` de una trayectoria — inmune a `Stop()`, que en un
> intento anterior solo lo había despejado temporalmente. El robot nunca
> llegó a moverse en ninguno de esos intentos (sin riesgo físico, solo
> bloqueo funcional). Un apagado/encendido físico de la caja de control lo
> resolvió del todo — probable estado interno acumulado que el software no
> estaba limpiando. Moraleja para sesiones reales largas: si el software
> (`ClearError`/`Stop`) no arregla un error de protocolo recurrente de
> forma duradera, un power-cycle del controlador es más rápido y más
> fiable que seguir depurando a ciegas.

> [!question] (07/09) Criterio config-vs-constante para datos de un adaptador
> Surgió al revisar qué de lo añadido ese día (límites articulares, `cp`)
> debía ir al YAML del pipeline (`robot_node.yaml`, como ya hace
> `cr5_host`). Primera versión del criterio: preferencia de despliegue/
> ajuste → parámetro ROS2; dato de SEGURIDAD verificado contra el
> fabricante → constante, nunca pisable por `-p` (mismo argumento que ya
> justificaba que los puertos TCP tampoco fueran configuración).
>
> Refinado el mismo día tras una pregunta directa: ¿y si un robot nuevo
> tiene otros límites, o interesa ser más restrictivo en algún despliegue
> del MISMO robot? La dicotomía "config o constante" era demasiado
> binaria para los límites articulares en concreto. Solución: el límite
> de FÁBRICA (`_FACTORY_JOINT_LIMITS_DEGREES`) se queda como techo fijo
> en el código, pero un override opcional (`cr5_joint_limits_degrees` en
> el YAML) SÍ puede pisarlo -- solo para ESTRECHARLO (`min()` joint a
> joint contra la fábrica, nunca una sustitución directa). Así un typo o
> un override descuidado nunca puede ampliar el límite por encima del
> real, pero sí se puede pedir un margen más cauto por sesión, y un robot
> nuevo define su propio techo en su propio adaptador sin tocar esta
> constante. Ver [[Cr5RealRobotAdapter]].

> [!question] (08/09) `forward_kinematics`/`link_poses` pasan a ser parte formal de `KinematicsPort`
> Señalado por el usuario al revisar la vault: la nota de
> [[KinematicsPort]] presentaba la cinemática directa como un "extra" que
> [[PoeKinematicsAdapter]] ofrece, cuando conceptualmente es algo que
> cualquier cinemática de cadena serie debería poder dar (mucho más
> simple que la inversa), no una capacidad especial de un adaptador
> concreto. Decisión: formalizarlo ya en el `Protocol` (`shared_kernel/ports.py`),
> en vez de dejarlo como capacidad opcional ad-hoc por consumidor.
>
> Efecto en cada adaptador de [[KinematicsPort]]: [[PoeKinematicsAdapter]]
> ya los tenía, sin cambio de comportamiento. [[CoppeliaSimIkKinematicsAdapter]]
> los implementa por primera vez, sobre el mismo entorno IK aislado que ya
> usaba `compute_trajectory` (`simIK.setJointPosition` para fijar el clon
> a la configuración pedida + `simIK.getObjectPose` para leer la pose
> resultante, sin tocar la escena real) — **sin verificar en vivo contra
> CoppeliaSim en esta sesión** (requiere GUI, no disponible en este
> entorno), pendiente de un smoke test contra `cr5_base.ttt` antes de
> confiar en ello en una sesión real. `GaKinematicsAdapter`/`DhKinematicsAdapter`
> (stubs) y `NaiveTestKinematicsAdapter`/`StraightLineKinematicsAdapter`
> (dobles de test) los declaran lanzando `NotImplementedError`, por
> consistencia con `compute_trajectory` — ninguno tiene un modelo
> geométrico real que darles.
>
> Como consecuencia, [[ObstacleAvoidingPlanningAdapter]] y
> [[WholeBodyObstacleAvoidingPlanningAdapter]] dejan de necesitar su
> propio `Protocol` local (`_KinematicsPortWithForward`/
> `_KinematicsPortWithLinkPoses`) — ahora aceptan `KinematicsPort`
> directamente. Verificado con mypy (instalado en esta sesión, no estaba
> en el entorno) que ningún sitio del repo que use `KinematicsPort` como
> tipo (el registro `_STRATEGIES` de `controller_node`, los dos
> planificadores, los demos) rompe con el contrato ampliado; suite de
> tests existente (97 tests) sigue en verde.

> [!bug] (08/09) Autocolisión real del CR5 durante `cr5_semicircle_demo.py` — `GetErrorID()`=[76]
> Al mandar 9 `MovJ` consecutivos de la semicircunferencia contra el
> robot físico, `DisableRobot()` falló al final con el código -2. Según el
> manual oficial y la tabla de alarmas del fabricante: `RobotMode()`
> devolvía 9 (alarma sin limpiar) y `GetErrorID()` devolvía `[76]` = "el
> extremo interfiere con el cuerpo del robot" (autocolisión, nivel de
> severidad 5) — confirmado que el hipótesis del usuario era correcta. El
> manual también explica el -2 en sí: en estado de alarma NINGÚN comando
> de control se ejecuta hasta `ClearError()`, algo que `cr5_disable_demo.py`
> no intentaba — corregido para probar `GetErrorID()`→`ClearError()`→
> `DisableRobot()`, en ese orden.
>
> Decisión de fondo: implementar autocolisión como comprobación de base,
> no ad-hoc — nuevo [[SelfCollisionAwarePlanningAdapter]] (tercer
> `PlanningPort`), cápsulas (segmento + radio) sobre `link_poses`,
> `_segment_geometry.segment_segment_distance` nueva (algoritmo cerrado
> clásico, Lumelsky/Ericson). Radio único calibrado contra dos puntos
> reales (home: 11.6cm de margen mínimo real; la secuencia que disparó la
> alarma: 7.7cm en el peor tramo) — no adivinado. Hallazgo intermedio real
> por el camino: excluir pares "adyacentes por índice" no basta —
> `joint1`/`joint2` del CR5 comparten el mismo punto físico (offset cero
> entre ellos en el URDF), así que la exclusión final es GEOMÉTRICA
> (`structural_adjacency_exclusions`, calculada una vez por ser propiedad
> del robot, no de la postura), no por índice. Verificado reproduciendo la
> secuencia real completa: la rechaza en el punto 8/9, antes del tramo
> donde ocurrió el incidente real.
>
> **(08/09, mismo día) Segunda vuelta — a petición del usuario: "necesitamos
> generar una trayectoria en la que no colisione, por los mismos puntos".**
> Nuevo `SelfCollisionAvoidingPlanningAdapter` (misma familia, no sustituye
> al de arriba): explota la MISMA singularidad de `joint5≈0` ya documentada
> en [[PoeKinematicsAdapter]] — cerca de ahí, `joint4`/`joint6` tienen un
> grado de libertad casi redundante para una orientación dada (moverlos en
> direcciones opuestas apenas cambia la orientación, pero SÍ desplaza el
> tramo intermedio, porque `joint4→joint5` no tiene longitud cero).
> Reintenta la IK del mismo objetivo desde una semilla con esos dos joints
> desplazados en pasos crecientes hasta encontrar una rama libre de
> autocolisión. Verificado contra el incidente real completo: un
> desplazamiento de solo 4° ya resuelve el punto que colisionaba, con un
> salto adicional de ~10° respecto al waypoint anterior — muy por debajo de
> la alternativa de "vuelta de muñeca" completa (~180°, saltos de 85-290°
> entre waypoints, inutilizable para un movimiento suave). Las 9
> configuraciones del arco real se alcanzan sin excepción, mismo punto
> cartesiano final exacto. Ya wireado en `cr5_semicircle_sim_demo.py`
> (verificado en vivo contra CoppeliaSim) y `cr5_semicircle_demo.py`
> (verificado contra el arnés de servidor TCP de mentira, pendiente de
> confirmación contra el robot físico real).

> [!bug] (08/09) `cr5_semicircle_demo.py`/`cr5_circle_demo.py` leían/cerraban antes de que el robot terminara de moverse
> Mismo patrón exacto ya conocido y corregido antes en
> `commander/poe_lift_and_wrist_demo.py`/`poe_sim_then_real_demo.py`
> (`_wait_until_robot_idle`) y en [[Commander y ControlSession]]
> (cierre de sesión prematuro, 07/09) — pero no se aplicó a estos dos
> scripts nuevos al escribirlos. Síntoma real: tras una ejecución
> "completa sin errores" del círculo, el usuario encontró el robot
> parado a mitad de camino (`joint4≈-66°`, sin ninguna alarma). Causa:
> el script solo pausaba `waypoint_pause_seconds` (0.3s) entre cada
> ENVÍO de `MovJ`, no entre el envío y que el robot terminara de
> EJECUTARLOS — con `cp=50` el controlador sigue procesando la cola a su
> propio ritmo real, así que tanto la lectura de "posición final" como
> el propio `DisableRobot()` podían llegar mientras el robot seguía en
> marcha. Corregido añadiendo `_wait_until_robot_idle` (poll de
> `RobotMode()` hasta 5, con timeout de seguridad de 15s) antes de leer
> la posición final y de des-energizar, en los dos scripts.

> [!tip] (14/09) Donde hay redundancia, anclar la rama explícitamente en vez de confiar en la IK
> Al montar el gesto de saludo (`cr5_wave_sim_demo.py`, ver
> [[Scripts de Demostración]]) hacía falta la herramienta inclinada hacia
> arriba — una rotación sobre el eje X del mundo que ningún joint del CR5
> da suelto en esa postura. El camino obvio, pedírsela a la IK rotando la
> pose y dejando que Newton-Raphson encontrase la postura, **converge y aun
> así no sirve**: llega a una rama contorsionada (`joint1`=-9°,
> `joint6`=-69°) desde la que el barrido posterior del saludo va saltando
> de rama punto a punto, con saltos de **hasta 51° entre waypoints
> consecutivos**. Lo insidioso es que ninguna métrica de "¿salió bien?"
> lo delataba: el recorrido cerraba sobre su partida con 0.00° de residuo.
>
> La solución no fue tocar la IK sino no usarla para eso: conjugar la
> rotación de `joint5` (eje Z del mundo) por ±90° de `joint4`/`joint6`
> (eje Y) la convierte exactamente en el cabeceo que se quería —
> Ry(−90)·Rz(β)·Ry(90) = Rx(−β) — con lo que la postura de partida queda
> definida **en espacio de articulaciones**, analítica y sin IK, y a la IK
> solo le queda seguir deltas pequeños (paso máximo: de 51° a ~7°).
>
> Es la misma lección que ya había dado `_snap_to_exact_start_if_needed`
> en el círculo (08/09), desde el otro lado: allí la redundancia de muñeca
> se corregía a POSTERIORI (un waypoint final a la configuración exacta),
> aquí se evita de ENTRADA. Generalizable: cuando una pose se puede
> alcanzar por varias ramas, elegir la rama es una decisión de diseño, no
> algo que delegar en el solucionador — y conviene tomarla en el espacio
> donde es exacta (articulaciones) en vez de en el espacio donde es
> ambigua (cartesiano). Ver [[PoeKinematicsAdapter]].

## Ver también

- [[Estado del Roadmap]]
- [[Arquitectura Hexagonal]]
- [[KinematicsPort]]
- [[SelfCollisionAwarePlanningAdapter]]
- [[Commander y ControlSession]]

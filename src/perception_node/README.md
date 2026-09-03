# perception_node

Adaptador de entrada/salida ROS2 alrededor de `PerceptionPort`: publica
periódicamente la `Scene` que reporte el adaptador elegido. A diferencia de
`robot_node`/`controller_node`, vive fuera del namespace de cualquier
`ControlSession` (ver `docs/nodos_ros2.md` §4).

## Configuración de hoy (implementado)

Parámetros, topics y cómo sobreescribirlos: **`docs/configuracion_nodos.md`**
(tabla de `perception_node`). No se duplica aquí para no tener dos fuentes
de la verdad.

## Qué más suele hacer falta configurar en un nodo de este tipo (NO implementado hoy)

- **Umbral de confianza/filtrado de detecciones** — solo tiene sentido en
  cuanto exista un adaptador que perciba de verdad (`CoppeliaSimPerceptionAdapter`,
  bloqueado hoy en cómo leer el radio de una esfera vía la API ZMQ, ver
  ROADMAP.md Bloque 3). `FilePerceptionAdapter`/`StaticPerceptionAdapter`
  no "detectan" nada, solo reportan lo que ya está escrito.
- **Qué partes de la `Scene` reportar** (planos/obstáculos/objetos, cada
  uno activable por separado) — hoy el nodo publica siempre la `Scene`
  completa que devuelva el adaptador, sin forma de pedir solo un subconjunto.
- **Frame de referencia de las coordenadas reportadas** — la `Scene` no
  declara en qué marco están sus puntos; hoy se asume implícitamente que es
  el mismo que usa el planificador (relativo a `base_link`, ver docstring
  de `controller_node/adapters/_segment_geometry.py`). Con más de un
  perceptor real, esto debería ser explícito, no una convención tácita.
- **Política de fusión entre varios perceptores** — YA está bocetada, pero
  como responsabilidad de `Commander` (vía `Scene.merge`), no de este nodo
  (ver `docs/nodos_ros2.md` §4: "el productor de `<ns>/scene` NO es un
  único perceptor... es `Commander`"). Si en algún momento la fusión se
  moviera aquí (varios `PerceptionPort` dentro del mismo proceso), haría
  falta una política de qué hacer cuando dos perceptores contradicen la
  misma zona.
- **Publicar solo al cambiar, no siempre por timer** — hoy
  `scene_publish_period_seconds` es un timer fijo que republica la última
  `Scene` conocida aunque no haya cambiado nada; un modo "solo si cambió"
  sería más barato para un sensor real de alta frecuencia.

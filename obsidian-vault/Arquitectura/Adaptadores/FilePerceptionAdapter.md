---
tags: [arquitectura, adaptador]
---

# FilePerceptionAdapter

Implementa [[PerceptionPort]] releyendo un fichero de texto plano ENTERO
en cada `get_scene()` — sin diff, sin llevar la cuenta de "qué línea es
nueva". Código:
`src/perception_node/perception_node/adapters/file_perception_adapter.py`.

Banco de pruebas mínimo de percepción sin depender de CoppeliaSim: editar
el fichero a mano desde otra terminal y que el siguiente `get_scene()` lo
refleje es la forma más simple de probar "el mundo cambió y el sistema se
entera", sin cámara, sin simulador.

## Formato

Una línea por entrada:

```
nombre x y z radio     # un obstáculo (Scene.obstacles)
objetivo x y z         # EL objetivo (Scene.objects["objetivo"], clave fija)
```

`"objetivo"` es palabra reservada — no nombres un obstáculo así. Líneas
vacías y las que empiezan por `#` se ignoran. Repetir un nombre se queda
con la ÚLTIMA línea (mismo criterio que `Scene.with_obstacle`).

## Auto-descripción (`description`)

Declara qué tipo de información reporta, al estilo del schema de una tool
de MCP — mismo mecanismo que [[PseudoPerceptionAdapter]]. Nadie lo
consume todavía (no hay LLM en el bucle, Bloque 6), pero nace con esta
forma para no tener que añadirla más tarde.

## Ver también

- [[PerceptionPort]]
- [[PseudoPerceptionAdapter]]
- [[Scene y Percepción]]

# CLAUDE.md

Instrucciones específicas de este proyecto para Claude Code.

## Mantener actualizadas TODAS las fuentes de seguimiento, no solo el código

Este proyecto se sigue en tres sitios a la vez, y los tres deben quedar
coherentes tras cualquier cambio significativo (código, decisión de
arquitectura, hallazgo real al probar contra hardware, avance/cierre de un
bloque del roadmap):

1. **`ROADMAP.md`** (este repo) — marcar tareas hechas, añadir hallazgos con
   fecha.
2. **Vikunja** (proyecto "ProyectoRobotica") — tareas y su estado, enlazadas
   entre sí cuando corresponda.
3. **Vault de Obsidian** (`obsidian-vault/`, dentro de este mismo repo
   desde el 07/09 — antes vivía fuera, en
   `~/Documents/ObsidianVaults/robot-control-hexagonal/`; si esa carpeta
   externa reaparece, es una copia obsoleta, no la fuente de verdad) —
   resumen de arquitectura, decisiones y estado del roadmap para explicar
   el proyecto sin leer el repo entero. En concreto, según qué cambió:
   - `Home.md` — la frase de estado en una línea, si cambia el titular.
   - `Roadmap/Estado del Roadmap.md` — la fila del bloque afectado en la
     tabla, y el callout/hitos si es un avance notable.
   - `Arquitectura/*.md` — si el cambio toca un puerto, un adaptador, o el
     flujo entre Commander/ControlSession/nodos (p. ej. `Puertos y
     Adaptadores.md` cuando un adaptador pasa de "en progreso" a
     "verificado", o gana/pierde un método del contrato).
   - `Decisiones/Decisiones de Diseño Clave.md` — cualquier decisión de
     diseño no trivial, con fecha y motivo.
   - `Diario/<AAAA-MM-DD>.md` — una entrada por cada día de trabajo real
     (no por cada mensaje), con el orden cronológico de lo que se hizo y
     enlaces `[[...]]` a las notas de arriba para el detalle, en vez de
     repetirlo. Añadir el enlace nuevo en `Diario/Diario.md` (más reciente
     primero).

**No es opcional ni algo a hacer "cuando se acuerde"**: es el mismo hábito
que ya existe para ROADMAP.md/Vikunja, aplicado también al vault. Motivo
concreto (07/09/2026): el vault llegó a decir "sin verificar todavía contra
el robot físico" y "sin commitear" sobre trabajo que ya estaba probado y
commiteado desde hacía rato — el usuario tuvo que señalarlo dos veces. Una
nota de seguimiento que no se actualiza es peor que no tenerla, porque
parece autoritativa y no lo es.

No hace falta tocar el vault para cambios triviales/internos sin relevancia
arquitectónica (un refactor interno, un typo, un test nuevo que no cambia
comportamiento) — sí para: puertos/adaptadores nuevos o que cambian de
estado, decisiones de diseño, un bloque del roadmap que se cierra o se
redefine, y hallazgos reales significativos al probar contra hardware
físico.

---
tags: [meta, obsidian]
---

# Cómo usar este vault

Chuleta rápida de Obsidian, escrita contra este vault concreto. No hace falta
leerla entera de una sentada — vuelve cuando algo no te cuadre.

## Qué es un "vault"

Ni más ni menos que esta carpeta: `obsidian-vault/`, dentro del propio repo
`~/Desktop/robot-control-hexagonal/` desde el 07/09 (antes vivía fuera, sin
versionar, en `~/Documents/ObsidianVaults/robot-control-hexagonal/`).
Todo son ficheros `.md` en texto plano — puedes editarlos con cualquier editor,
Obsidian solo añade la capa de enlaces/grafo/búsqueda encima. Al abrir esta
carpeta como vault en la app, Obsidian crea una subcarpeta `.obsidian/` con su
configuración (no la he creado yo a mano); el estado de pestañas abiertas
(`workspace.json`) está excluido del control de versiones vía `.gitignore`
del repo — es estado local de la app, no contenido.

## Enlaces `[[así]]`

`[[Arquitectura Hexagonal]]` enlaza a `Arquitectura/Arquitectura Hexagonal.md`
sin que haga falta escribir la ruta ni la carpeta — Obsidian resuelve por
nombre de nota en todo el vault. Si escribes `[[Nota Que No Existe]]` y haces
clic, Obsidian la crea vacía en el sitio donde tengas el cursor: así es como
se hace crecer un vault orgánicamente, sin planificar la estructura antes.

- Clic normal: navega a la nota.
- Ctrl/Cmd+clic: la abre en un panel al lado (split), para ver dos notas a la vez.
- `[[Nota|texto a mostrar]]`: enlaza pero muestra otro texto.

## Backlinks (el motivo real de usar enlaces)

Cada nota tiene, en el panel derecho ("Linked mentions" / "Backlinks"), la
lista de qué otras notas la enlazan. Por ejemplo, [[Scene y Percepción]] la
enlazan tanto [[Puertos y Adaptadores]] como [[Estado del Roadmap]] — abrir
"Scene y Percepción" y mirar sus backlinks te da gratis "quién depende de
esto", sin mantener esa lista a mano en ningún sitio.

## MOC (Map of Content)

[[Home]] es el MOC principal: una nota que no es "contenido" en sí, solo una
lista organizada de enlaces a las demás. Es la alternativa Obsidian a forzar
una jerarquía de carpetas rígida — las carpetas de este vault
(`Arquitectura/`, `Roadmap/`, `Decisiones/`) son solo para no tener 8 ficheros
sueltos en la raíz, la navegación real pasa por los enlaces, no por el árbol
de carpetas.

## Tags (`#etiqueta`)

Sirven para agrupar por *tipo* en vez de por *tema* (para tema ya están los
enlaces). En [[Estado del Roadmap]] uso tags jerárquicos:
`#bloque/hecho`, `#bloque/en-progreso`, `#bloque/pendiente` — el panel de
tags (izquierda) los agrupa bajo `#bloque` con sus tres variantes. Clic en un
tag = todas las notas/líneas que lo usan.

## Callouts

Bloques como este:

> [!info] Así se ve un callout
> Sintaxis: `> [!tipo] Título` seguido de líneas `>` normales. Tipos con
> estilo propio: `note`, `info`, `tip`, `warning`, `danger`, `bug`,
> `example`, `question`, `todo`. Un tipo inventado (como uso en
> [[Decisiones de Diseño Clave]]) también renderiza, con estilo genérico.

## Vista de grafo

Icono de grafo en la barra lateral: dibuja cada nota como nodo y cada
`[[enlace]]` como arista. Desde que [[Puertos y Adaptadores]] se
desglosó en una nota por puerto (`Puertos/`) y por adaptador con
sustancia real (`Adaptadores/`, 07/09), el grafo ya no es trivial — pero
[[Puertos y Adaptadores]] sigue siendo el nodo más central, coherente con
que es, literalmente, el punto de mayor acoplamiento del propio código
(todo el mundo depende de los puertos).

## Cómo mantener esto vivo (y no que se quede congelado hoy)

Cuando cierres un bloque de trabajo real en el repo:

1. Actualiza la fila correspondiente en [[Estado del Roadmap]].
2. Si tomaste una decisión no evidente (como las que ya hay en
   [[Decisiones de Diseño Clave]]), añade una entrada nueva ahí — no hace
   falta nota aparte, es una lista de entradas fechadas.
3. Si aparece un concepto de dominio nuevo que no encaja en ninguna nota
   existente, esa sí es una nota nueva de verdad — enlázala desde
   [[Arquitectura Hexagonal]] o desde donde tenga más sentido, y desde [[Home]]
   si es lo bastante central.
4. Añade (o actualiza) la entrada del día en [[Diario]] — el orden
   cronológico de los hechos, enlazando a las notas de arriba para el
   detalle en vez de repetirlo. Ver `CLAUDE.md` en la raíz del repo para la
   instrucción completa de qué mantener sincronizado y cuándo.

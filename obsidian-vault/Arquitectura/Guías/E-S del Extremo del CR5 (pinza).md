---
tags: [arquitectura, guia, cr5, hardware]
---

# E/S del extremo del CR5 (conector de 8 pines)

Nota de referencia del **End I/O**: el conector de 8 pines de la brida del
CR5, por el que va conectada la pinza del laboratorio. Escrita el 17/09
tras sondearlo contra el robot físico, porque el repo todavía **no tiene
ningún soporte de efector final** (ni puerto, ni método en
[[Cr5RealRobotAdapter]], ni comandos de E/S de herramienta) y lo primero
para diseñarlo es saber qué ofrece el hardware.

## Pinout

Fuente: manual de usuario del CR5, Tabla 3.5 "End I/O description". El
cable es el designado por el fabricante, modelo **Lumberg RKMV 8-354**.

| Pin | Señal | Función |
| --- | --- | --- |
| 1 | `AI_1/485A` | Entrada analógica 1 **o** 485A |
| 2 | `AI_2/485B` | Entrada analógica 2 **o** 485B |
| 3 | `DI_2` | Entrada digital 2 |
| 4 | `DI_1` | Entrada digital 1 |
| 5 | `24V` | Alimentación 24V (salida) |
| 6 | `DO_2` | Salida digital 2 |
| 7 | `DO_1` | Salida digital 1 |
| 8 | `GND` | Masa |

Por un solo cable van, entonces, **alimentación + 2 DI + 2 DO + RS485**.
Las analógicas y el 485 COMPARTEN los pines 1 y 2: son terminales
multiplexados, y el modo por defecto es **485**. Consecuencia práctica:
mientras no se conmute con `SetToolMode(2,type)`, lo que devuelva `ToolAI`
es ruido, no una medida.

## Comandos del protocolo TCP/IP que tocan estos pines

Del manual `Dobot TCP_IP二次开发接口文档 V4.6.5` (ver
[[_cr5_protocol (protocolo TCP del CR5)]] para el marco general del 29999/30004):

| Vía | Pines | Comandos |
| --- | --- | --- |
| Salidas digitales | 6, 7 | `ToolDO(index,status)` (encolado), `ToolDOInstant` (inmediato), `GetToolDO(index)` |
| Entradas digitales | 3, 4 | `ToolDI(index)` |
| Analógicas | 1, 2 | `SetToolMode(2,type)` y luego `ToolAI(index)` |
| RS485 / Modbus RTU | 1, 2 | `SetTool485(baud,parity,stopbit)`, **`ModbusCreate("127.0.0.1",60000,slave,1)`** (no `ModbusRTUCreate`, ver abajo), `GetHoldRegs`/`SetHoldRegs`, `ModbusClose` |
| Alimentación | 5 | `SetToolPower(0|1)` — el manual lo describe como "reiniciar la alimentación del extremo, p. ej. para re-inicializar la pinza" |

> [!warning] `SetHoldRegs` y `ToolDO` MUEVEN la pinza
> Todo lo sondeado el 17/09 fue de lectura a propósito. Escribir un
> registro Modbus o conmutar una salida digital cierra o abre los dedos:
> no se hace sin alguien delante del robot.

> [!success] Resuelto el 29/09: todo el sondeo de abajo usó el comando equivocado
> `ModbusRTUCreate` no llega al 485 de la brida; `ModbusCreate("127.0.0.1",
> 60000,9,1)` sí, y la Robotiq contestó a la primera. El cable, la
> polaridad y los parámetros estaban bien. Lo de abajo queda como historia
> de la investigación. Ver [[Manejar la pinza Robotiq 2F]] y [[2026-09-29]].

## Estado: la pinza no responde a ningún sondeo (17/09)

Con el robot **energizado** (`RequestControl()` + `EnableRobot()`, modo 5,
sin alarmas) y tras ciclar la alimentación del extremo:

- `ToolDI(1)` = `ToolDI(2)` = 0, `GetToolDO(1)` = `GetToolDO(2)` = 0.
- Ningún esclavo Modbus RTU contesta: slave id 1-15 a 115200 8N1, más
  9600/19200/38400/57600 y 115200 8E1 para los id 1 y 9. `GetHoldRegs`
  devuelve -1 en todos.
- Las E/S de la caja de control también están a cero
  (`digital_input_bits`/`digital_outputs` de la trama real-time).

Ojo con leer `ModbusRTUCreate` → `0,{índice}` como "hay pinza": ese
comando solo abre el maestro en el lado del robot, **no comprueba que haya
un esclavo al otro lado**. La prueba real es que conteste `GetHoldRegs`.

Se probó también conmutar las salidas digitales (`ToolDOInstant(1,1)` y
`(2,1)`, restaurando a 0 después): el controlador confirma el cambio con
`GetToolDO`, pero las `ToolDI` no se inmutaron.

## La pinza es una Robotiq 2F Adaptive — y el sondeo no valía

El usuario identificó el modelo al final de la sesión: **Robotiq 2F
Adaptive Gripper** (2F-85/2F-140). Sus valores de fábrica:

| Parámetro | Valor |
| --- | --- |
| Velocidad | 115200 bps, 8 bits, sin paridad, 1 stop (8N1) |
| Slave ID | **9** |
| Estado (lectura, FC03) | **0x07D0** (2000) en adelante |
| Petición de acción (escritura, FC16) | **0x03E8** (1000) en adelante |

Con eso, **el resultado negativo de arriba deja de ser concluyente**. El
barrido limpio probó el id 9 a 115200 8N1 —correcto— pero solo leyendo
0x0200 y 0x0000, direcciones que esta pinza no implementa. Una dirección
inexistente devuelve excepción Modbus, y `GetHoldRegs` la reporta como
-1: exactamente el mismo -1 que "no contesta nadie". Es decir, el sondeo
**no podía distinguir** una pinza sana de una ausente. La única pasada
que llegó a probar 0x07D0 con el id 9 fue la primera, la que iba
desincronizada y se descartó por poco fiable.

Prueba pendiente, con el robot encendido:

```
ModbusRTUCreate(9,115200,"N",8,1)   -> 0,{idx}
GetHoldRegs(idx,2000,3)            -> estado gACT/gGTO/gSTA/gOBJ
```

Si con la dirección correcta sigue sin contestar, el siguiente sospechoso
es el cableado del conector de 8 pines: **485A/485B cruzados** es el fallo
clásico de un cable a medida y encaja con "alimenta pero nunca responde".
Y para que la pinza MUEVA hay que activarla antes (escritura en 0x03E8);
para responder a una lectura de estado, no.

## Diagnóstico cerrado (18/09): es el cable

Con el extremo alimentado, la pinza enciende un **LED rojo FIJO**. Eso, en
la tabla de fallos del manual Robotiq, es fallo menor, y de los dos
candidatos solo encaja `gFLT = 0x09`, *"no communication during at least 1
second"*. Y por eliminación es concluyente: una pinza que comunicara pero
estuviese sin activar daría LED **azul** (`0x07`). **Está viva, alimentada,
y no le llegan los datos.**

Descartado por software, todo con resultado negativo:

| Prueba | Resultado |
| --- | --- |
| Dirección correcta (2000, FC03) | `-1` |
| Slave 9 + barrido 1-16 | ninguno contesta |
| 115200 8N1 y 8E1 | igual |
| `SetToolPower(1)` confirmado, ciclado off/on | 24V confirmados, sigue mudo |
| Robot energizado (modo 5) y des-energizado (4) | sin diferencia |
| `SetToolMode(1)` + `SetTool485` (forzar 485) | sin diferencia |

### El cable de Robotiq, y el mapeo que debería tener

De la figura 3-9 del manual (hay que ABRIR el PDF: pdftotext no renderiza
las figuras). El cable de dispositivo es de 8 polos pero solo usa cuatro
señales:

| Pin cable Robotiq | Señal | Va al pin de la brida |
| --- | --- | --- |
| 1 | `24V` | 5 (`24V`) |
| 2 | `GND` | 8 (`GND`) |
| 3 | `RS485 +` | 1 (`AI_1/485A`) ? |
| 4 | `RS485 −` | 2 (`AI_2/485B`) ? |

**Que la pinza encienda demuestra que 1 y 2 están bien mapeados.** Lo único
sin verificar es el par 3/4 — y ahí `485+`/`485-` frente a `485A`/`485B`
**no tiene correspondencia estándar** (en TIA/EIA-485 la "A" es la línea
invertida, pero buena parte del sector la etiqueta al revés). La propia
figura 3-9 dibuja el extremo macho y el hembra con los pines 3 y 4 en
posiciones **espejadas**: el fabricante avisa del error porque es el
habitual, y en el laboratorio hay además un cable INTERMEDIO, que duplica
la ocasión de equivocarse.

Un espejado COMPLETO está descartado: pondría los 24V del pin 5 en `DI_1`
y la pinza no encendería.

### Confirmado en vivo: la alimentación va por la brida, los datos no

Ciclando `SetToolPower` tres veces mientras el usuario miraba la pinza
(18/09):

- **El LED se apaga y enciende con cada ciclo** -> la pinza se alimenta
  por el **pin 5 de la brida**. Eso demuestra que el cable está
  **re-mapeado a mano**: el 24V de la Robotiq es su pin 1, no el 5, así
  que alguien tradujo el pinout a propósito (el intermedio es un alargador
  macho-hembra recto, y el conector lleva pestaña que impide enchufarlo
  girado, así que la traducción está en el cable de la pinza).
- **Al encender se ve azul/violeta un instante y luego rojo.** Es la
  secuencia normal del manual: arranca sin fallo (azul) y al pasar 1 s sin
  comunicación levanta `gFLT = 0x09` (rojo). Una pinza sana a la que nadie
  habla.

| Señal | Estado |
| --- | --- |
| 24V (Robotiq 1 -> brida 5) | verificado: responde a `SetToolPower` |
| GND (Robotiq 2 -> brida 8) | implícito: sin masa no arrancaría |
| RS485+/- (Robotiq 3/4 -> brida 1/2) | **lo único que falla** |

Siguiente paso, físico: intercambiar los dos hilos de datos, o comprobar
con multímetro que existen (la otra opción es que el cable a medida solo
llevara alimentación). **Confirmación visual inmediata: en cuanto lleguen
datos, el LED pasa de rojo a azul**, porque el `0x09` se borra solo al
restablecerse la comunicación.

### Los hilos de datos SÍ llegan: sonda analógica (18/09)

Sin multímetro en el laboratorio, se usaron las **entradas analógicas de la
propia brida** como sonda. Los pines 1 y 2 son multiplexados (485 o
analógica), así que `SetToolMode(2,0)` los conmuta a entrada 0-10V y
`ToolAI` los lee. Medida **diferencial**, ciclando la alimentación de la
pinza para distinguir lo que pone ella de lo que pone el robot:

| | `ToolAI(1)` (pin 1) | `ToolAI(2)` (pin 2) |
| --- | --- | --- |
| Pinza alimentada | **1.37 V** | **1.29 V** |
| Pinza sin alimentar | 0.06 V | 0.01 V |
| Pinza alimentada otra vez | 1.37 V | 1.29 V |

La tensión desaparece al quitar los 24V y vuelve al darlos: **la ponen los
transceptores de la pinza**. Conclusión firme: **el par de datos está
físicamente conectado a los pines 1 y 2**. El cable lleva las cuatro
señales (24V, GND, 485+, 485-), no solo alimentación.

Lo que la medida NO permite concluir: la polaridad. La diferencia es de
**80 mV**, por debajo del umbral de decisión de RS-485 (±200 mV), o sea
reposo en zona indeterminada; y qué signo debería tener depende de si el
"485A" de Dobot es la línea invertida o la no invertida, convenio que su
manual no documenta y que el sector usa en los dos sentidos.

**Queda una sola variable: el orden del par.** Intercambiar los dos hilos
ya no es un palo de ciego -- sabemos que existen, que llegan y que hay un
transceptor vivo al otro lado. Confirmación visual inmediata al acertar:
el LED pasa de rojo a azul.

### Colores de hilo: las dos tablas, y la trampa

Del **Dobot CR5 Hardware User Guide V2.3** (tabla 3.6), que documenta el
cable designado Lumberg RKMV 8-354 hilo a hilo — dato que el User Guide
V3.5.3.1 NO trae:

| Pin | Color | Señal |
| --- | --- | --- |
| 1 | Blanco | `485A` |
| 2 | Marrón | `485B` |
| 3 | Verde | `DI_2` |
| 4 | Amarillo | `DI_1` |
| 5 | Gris | `24V` |
| 6 | Rosa | `DO_2` |
| 7 | Azul | `DO_1` |
| 8 | Rojo | `GND` |

Y del lado Robotiq (manual del FT 300, fig. 3.2 — el de la 2F no documenta
colores en texto, así que vale como indicio fuerte, no como certeza):

| Pin | Color | Señal |
| --- | --- | --- |
| 1 | malla | `485 GND` |
| 2 | Rojo | `24V` |
| 3 | Negro | `GND` |
| 4 | Blanco | `485+` |
| 5 | Verde | `485−` |

> [!danger] El rojo significa lo contrario en cada cable
> **Rojo = 24V en Robotiq, pero = GND en Dobot.** Empalmar por color habría
> cortocircuitado la alimentación contra masa. Como la pinza funciona, el
> cable NO se hizo por colores sino por números de pin: hubo traducción
> consciente.

> [!note] El blanco sí coincide
> Blanco es `485+` en Robotiq y `485A` en Dobot. Si se unió blanco con
> blanco, el `+` quedó en `485A` — que es justo lo que la medida de tensión
> del 18/09 (pin 1 a 1 V, pin 2 a 0,02 V → pin 1 es la no invertida) dice
> que debe ser. Indicio a favor de que **la polaridad es correcta**.

Especificación del interfaz de extremo, del mismo manual: 2 DI, 2 DO, 2 AI
multiplexadas con RS485, y el RS485 declarado como **`ModBus_RTU`**. La
pinza está conectada exactamente donde el fabricante espera.

### Cableado del conversor RS-485/USB (FT 300, fig. 3.3)

```
485+     -> borne 1        485-  -> borne 2        485 GND -> borne 3
puente entre bornes 4 y 5  = resistencia de terminación de 120 Ohm
```

### El sensor de par no está en este bus (18/09)

Entre la brida y la pinza hay un **sensor de par**, lo que abrió la
hipótesis de que fuera él quien ocupaba el bus 485 (y que el transceptor
medido a 1.3 V fuera suyo, no de la pinza). **Descartada**, por dos vías:

- El **FT 300** tiene **cable propio**: un pigtail con conector **M12 de 5
  pines** (alimentación en 2 y 3, RS-485 en 1, 4 y 5), que según su manual
  va "directamente, a un conversor RS-485/USB o RS-485/RS-232". No es un
  pasamuros: no reenvía el bus de la brida.
- Sus parámetros exactos (**19200 8N1, slave ID 9**, fuerzas en el
  registro **180**, número de serie en el **510**) se barrieron contra el
  bus de la brida a seis velocidades: **ninguna respuesta**.

Topología real, confirmada por el usuario: el conector de 8 pines va
**directo de la brida a la pinza** (vía alargador); el sensor se interpone
solo mecánicamente y habla por su propio cable.

> [!note] El acoplamiento y sus 10 muelles no son sospechosos
> La pinza se une a su acoplamiento por un conector de 10 pines de muelle
> (`24V` en 3/4, `485+/485-` en 7/8), y el cable se enchufa al
> acoplamiento. Que la pinza ENCIENDA prueba que la cadena
> brida→alargador→cable→acoplamiento→muelles conduce; que los DOS hilos de
> datos midan tensión prueba que también conducen los muelles 7 y 8.
> Alimentación y datos comparten camino: si llega una, llegan los dos.

> [!tip] Un conversor RS-485/USB puede estar ya en el laboratorio
> El FT 300 se conecta normalmente a uno. Si aparece, permite hablar con
> la pinza SIN el robot — y como sus hilos van a bornes de tornillo,
> probar las dos polaridades es cuestión de un minuto.

### Superficie de software agotada (18/09)

Barrido exhaustivo con la dirección correcta, todo negativo:

| Qué se probó | Resultado |
| --- | --- |
| 36 combinaciones de línea: 115200/57600/38400/19200/9600/230400 × N/E/O × 1/2 stop | ninguna |
| FC03 (`GetHoldRegs`) y FC04 (`GetInRegs`), en 2000 y en 0 | `-1` |
| Slave id 1-16 | ninguno |
| `SetToolMode(1)` + `SetTool485` (forzar 485) | sin cambio |
| `SetTool485(...,identify=2)` (2º航插) | `-60003`: el CR5 solo tiene uno |
| Alimentación confirmada, robot energizado y des-energizado | sin cambio |
| (24/09) `SetToolPower(1)` **antes** de `SetToolMode(1)` + `SetTool485` + `ModbusRTUCreate` | `-1`, 0,50 s por petición: el orden no importa |
| (24/09) FC04 `GetInRegs`, FC01 `GetCoils`, FC02 `GetInBits`, FC16 `SetHoldRegs(1000,{0,0,0})` | `-1`, 0,50 s todas: ni siquiera una excepción Modbus |
| (24/09) Orden nuevo, 2400/4800/9600/19200/38400/57600/115200/230400 × N/E/O, slave 9 | `-1` en todas, 0,50 s fijos |

El razonamiento que cierra el caso, y que conviene no perder:

1. Si las tramas LLEGARAN con la dirección equivocada, la pinza
   respondería con una excepción Modbus — **eso es comunicación**, el
   `0x09` se borraría y el LED se pondría azul. No ocurre.
2. Si llegaran con parámetros de línea equivocados, vería basura y
   seguiría roja — pero ya se probaron **todas** las combinaciones
   plausibles, incluida la de fábrica.
3. Luego **no llega señal eléctrica válida en absoluto**: el par de datos
   está cruzado, cortado o sin conectar. Un cruce A/B produce exactamente
   esto (bits invertidos, ningún frame válido) y es lo más probable en un
   cable mapeado a mano.

> [!bug] `SetToolPower` resetea la conexión del 29999
> Sistemático: 17/09 y 18/09, muchas veces. El comando se ejecuta, pero el
> controlador cierra la conexión en vez de contestar, y los comandos
> siguientes se pierden en silencio -- una vez dejó la pinza APAGADA
> porque el `SetToolPower(1)` posterior nunca llegó, y el sondeo siguió
> corriendo sin alimentación. Forma fiable de usarlo: **una conexión nueva
> por comando**, comprobando el `0,{}`. `Robotiq2FGripperAdapter` tendrá
> que hacerlo así si alguna vez alimenta la pinza desde el nodo.

> [!warning] Corregido el 18/09
> Antes esta nota daba como segundo sospechoso el `485 GND` sin conectar.
> **No aplica**: el acoplamiento sí tiene ese pin (el 10 de su bornera de
> 10 muelles), pero el cable de dispositivo de Robotiq solo lleva 4
> conductores y no lo incluye. Su ausencia es lo normal.

> [!note] Moraleja
> Un "no responde" solo vale si consta que la pregunta se hizo bien. Aquí
> el barrido era correcto en velocidad, paridad y slave id, y aun así
> preguntaba por una dirección que este dispositivo no tiene.

## Relacionado

- [[Manejar la pinza Robotiq 2F]] — cómo mandarla por Modbus y cómo
  añadirla a la simulación

- [[Cr5RealRobotAdapter]] — el adaptador que tendría que ganar el soporte
  de pinza cuando se sepa por dónde se manda.
- [[Puertos y Adaptadores]] — dónde encajaría un puerto de efector final.
- [[2026-09-17]] — el día del sondeo.

## Qué dice fuera del repo (búsqueda del 24/09)

No hay ningún caso público de "Robotiq en la brida de un Dobot que devuelve
`-1`". Lo útil que salió:

- **Robotiq**: la conexión de muñeca del Dobot es la misma que la de un
  **Fanuc CRX**, y la del CRX es la misma que la de un **UR e-Series**. El
  kit que recomiendan para Dobot es el de CRX (`AGC-CRX-KIT-85`). O sea, el
  pinout de la fig. 3-9 del manual e-Series (el que ya usamos) aplica tal
  cual. Robotiq no da soporte del lado Dobot.
- **Dobot**: DobotStudio Pro **≥ 2.3.0** trae **preinstalado** un plugin
  Dobot+ para Robotiq 2F/Hand-E/EPick. Es la prueba con software oficial
  que falta.
- En UR y CRX **sí hay un paso de "activar"**: en UR, el *Tool
  Communication Interface* del Installation tab (si no, los pines son
  analógicos); en CRX, las variables `$TLIF` (tensión 24 V, baudios, modo
  DO). Aquí el equivalente es `SetToolMode(1)` + `SetTool485`, que ya se
  hace. No hay ningún ajuste documentado más allá de esos.
- UR avisa de que su 485 lleva **polarización de reposo** interna, y de que
  si falta hay que poner pull-up en 485+ y pull-down en 485-. El manual de
  Dobot no dice si la suya la tiene. Encaja con los ~20 mV de reposo que
  medimos (zona indeterminada), pero esa medida se hizo en modo analógico,
  con el transceptor del CR5 desconectado: no dice nada de la polarización
  en modo 485. El osciloscopio lo ve (paso 1 de la medida).

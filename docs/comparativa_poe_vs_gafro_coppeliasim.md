# Comparativa PoE vs GAFRO en CoppeliaSim — 2026-09-17

Generado por `commander/cr5_poe_vs_gafro_sim_demo.py` (ver su docstring para el método). Mismo `RobotDescription` del CR5, mismo Newton-Raphson amortiguado y mismas tolerancias (posición 1e-4 m, orientación 1e-3 rad) en ambos adaptadores; la única diferencia es el álgebra: matrices 4x4/se(3) en numpy (PoE) frente a motores CGA en gafro (GA).

Arco: semicircunferencia de radio 0.15 m hacia abajo desde la home, 10 puntos, orientación fija. Verdad de terreno: posición de `Link6_visual` calculada por CoppeliaSim (offset respecto del frame de joint6 medido en la home: [0.0, -0.0, 0.0] m).

## IK sobre el arco (por punto, media y máximo)

| Adaptador | Puntos resueltos | Tiempo IK (ms) | Iteraciones NR | Error FK propia (mm) | Error real en CoppeliaSim (mm) |
|---|---|---|---|---|---|
| POE | 9/9 | 0.89 (máx 3.09) | 3.1 (máx 8.0) | 0.0181 (máx 0.0795) | 0.018 (máx 0.080) |
| GA | 9/9 | 0.27 (máx 0.70) | 3.1 (máx 8.0) | 0.0288 (máx 0.0888) | 0.029 (máx 0.089) |

Diferencia articular máxima entre las dos soluciones, punto a punto (mrad): joint1 0.0, joint2 67.5, joint3 162.8, joint4 184.0, joint5 0.0, joint6 273.7; **joint4+joint6 101.3**.

## FK contra CoppeliaSim en 200 configuraciones aleatorias (sin IK)

| Adaptador | Error medio (µm) | Error máx (µm) | `forward_kinematics` (µs) | `link_poses` (µs) |
|---|---|---|---|---|
| POE | 0.00 | 0.00 | 183.7 | 115.3 |
| GA | 0.00 | 0.00 | 39.7 | 236.6 |

## Lectura (generada a partir de los números de arriba)

- **Misma cinemática.** Error de FK contra el modelo de CoppeliaSim: máx 0.00 µm (PoE) y 0.00 µm (GA) en 200 posturas aleatorias; y ambos dejan la punta sobre el arco con error real máx 0.080 mm (PoE) / 0.089 mm (GA), dentro de la tolerancia de 0,1 mm de las dos IK.
- **Misma pose, distinta solución articular.** Diferencias de hasta 274 mrad (`joint6`) con la punta en el mismo sitio. No es un error: todo el arco se recorre con joint1 = joint5 = 0, y con joint5 = 0 los ejes de joint2, joint3, joint4 y joint6 son paralelos (la singularidad de muñeca documentada en [[PoeKinematicsAdapter]]): un mecanismo planar de 4 revolutas para 3 restricciones (x, z y giro en el plano), es decir, una familia continua de soluciones (*self-motion*) para cada punto. Los dos Newton-Raphson usan el mismo Jacobiano y amortiguamiento, pero parametrizan el error de forma distinta (twist de se(3) frente a logaritmo del motor CGA) y avanzan por esa familia de forma distinta. Es la libertad que la IK numérica no fija -- la misma que explota `SelfCollisionAvoidingPlanningAdapter` a propósito.
- **Coste.** IK: GA 3.3× más rápida por punto (misma media de iteraciones, 3.1 frente a 3.1); `forward_kinematics`: GA 4.6× respecto de PoE en numpy. `link_poses` en GA paga hoy 6 conversiones motor→Pose en Python; es optimizable (una sola pasada en C++ con `computeKinematicChainMotor` por prefijo) si algún `PlanningPort` lo necesita en bucle.
- **Lo que GA aporta y PoE no:** `Motor.apply` sobre puntos/esferas/planos/líneas, jacobianos de primitivas y dinámica del mismo `System` -- lo que necesitan la escena conforme (Fase 4a) y el MPC (Fase 4b) de la tesis.

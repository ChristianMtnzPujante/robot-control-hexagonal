"""Ejemplo numérico: eje de herramienta del CR5 coaxial con una recta
(TCP sobre la recta + eje paralelo) y avance a lo largo de ella, con
pygafro para la geometría y numpy para el álgebra lineal.

Exploración del 23/09/2026 (decisión abierta "tareas CGA por primitivas",
ver ROADMAP Bloque 1). Ejecutar desde la raíz del repo:
`python3 docs/cga_tareas_linea_prioridades.py`.
"""
import sys
import numpy as np

sys.path[:0] = ["src/shared_kernel", "src/controller_node", "src/geometry_kernel"]
import pygafro as g
from controller_node.adapters.ga_adapter import GaKinematicsAdapter, _CHAIN_NAME

np.set_printoptions(precision=4, suppress=True, linewidth=120)
ga = GaKinematicsAdapter()
S = ga._system


def motor(th):
    return S.computeKinematicChainMotor(_CHAIN_NAME, list(map(float, th)))


def line_plucker(L):
    """u (dirección) y m (momento) de una pygafro.Line."""
    c = dict(zip(L.blades(), np.asarray(L.vector()).ravel()))
    # índices verificados: 19=e01i 21=e02i 25=e03i 22=e12i 26=e13i 28=e23i
    u = np.array([c[19], c[21], c[25]])
    m = np.array([c[28], -c[26], c[22]])
    return u, m


def tool_state(th):
    """Datos que tomamos del robot en θ: TCP p y dirección del eje u."""
    M = motor(th)
    L = M.apply(g.Line(g.Point(0, 0, 0), g.Point(0, 0, 1)))  # eje z de la brida
    u, _ = line_plucker(L)
    u = u / np.linalg.norm(u)
    p = np.asarray(M.toTransformationMatrix())[:3, 3]
    return p, u


def jacobians(th, p, u):
    """J_p (3xn): dp/dθ, J_u (3xn): du/dθ, a partir del Jacobiano geométrico
    de gafro. Columna i = generador [e12,e13,e23,e1i,e2i,e3i]."""
    cols = S.computeKinematicChainGeometricJacobian(_CHAIN_NAME, list(map(float, th)))
    Jp, Ju = [], []
    for c in cols:
        b = np.asarray(c.vector()).ravel()
        w = np.array([b[2], -b[1], b[0]])  # e23=wx, e13=-wy, e12=wz
        v = b[3:6]
        Jp.append(v + np.cross(w, p))
        Ju.append(np.cross(w, u))
    return np.array(Jp).T, np.array(Ju).T


def pinv_damped(J, lam=1e-3):
    return J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(J.shape[0]))


# --- objetivo: recta L_goal (punto a, dirección ug) -------------------------
th = np.array([0.3, -0.4, 1.4, 0.2, 1.3, 0.1])
p0, u0 = tool_state(th)
ug = np.array([0.3, 0.2, -1.0]); ug /= np.linalg.norm(ug)
a = p0 + np.array([0.05, -0.04, 0.03])           # recta desplazada del TCP
n1 = np.cross(ug, [1, 0, 0]); n1 /= np.linalg.norm(n1)
n2 = np.cross(ug, n1)
B = np.column_stack([n1, n2])                     # base 3x2 ⟂ a la recta

# --- comprobar Jacobianos por diferencias finitas --------------------------
Jp, Ju = jacobians(th, p0, u0)
h = 1e-7
fd = np.column_stack([(tool_state(th + h * np.eye(6)[i])[0] - p0) / h for i in range(6)])
print("error Jp vs dif. finitas:", np.abs(Jp - fd).max())
fd = np.column_stack([(tool_state(th + h * np.eye(6)[i])[1] - u0) / h for i in range(6)])
print("error Ju vs dif. finitas:", np.abs(Ju - fd).max())


def tasks(th, s_target):
    p, u = tool_state(th)
    Jp, Ju = jacobians(th, p, u)
    e_on = B.T @ (p - a);        J_on = B.T @ Jp          # 2 filas: TCP sobre la recta
    e_par = B.T @ u;             J_par = B.T @ Ju         # 2 filas: eje paralelo
    e_prog = np.array([ug @ (p - a) - s_target]); J_prog = (ug @ Jp)[None, :]  # 1 fila
    return (e_on, J_on), (e_par, J_par), (e_prog, J_prog), p, u


print("\n--- iteración 0, detalle ---")
(e1, J1), (e2, J2), (e3, J3), p, u = tasks(th, 0.0)
print("p =", p, " u =", u, " u·ug =", u @ ug)
print("e_on  =", e1, "\nJ_on  =\n", J1)
print("e_par =", e2, "\nJ_par =\n", J2)
print("e_prog=", e3, "\nJ_prog=\n", J3)

# --- A) apilado ponderado --------------------------------------------------
print("\n=== A) apilado (Newton amortiguado) ===")
t = th.copy()
for k in range(30):
    (e1, J1), (e2, J2), (e3, J3), p, u = tasks(t, 0.0)
    e = np.concatenate([e1, e2, e3]); J = np.vstack([J1, J2, J3])   # 5x6
    if k < 3 or np.linalg.norm(e) < 1e-10:
        print(f"it {k}: |e|={np.linalg.norm(e):.2e}  rango J={np.linalg.matrix_rank(J)}")
    if np.linalg.norm(e) < 1e-10:
        break
    t = t - pinv_damped(J) @ e

# --- B) prioridades con espacio nulo: avance a lo largo de la recta --------
print("\n=== B) prioridades: [sobre+paralelo] > avance s = 0 -> 0.10 m ===")
t = th.copy()
for k, s in enumerate(np.linspace(0.0, 0.10, 11).tolist() + [0.10] * 20):
    (e1, J1), (e2, J2), (e3, J3), p, u = tasks(t, s)
    E1, JA = np.concatenate([e1, e2]), np.vstack([J1, J2])         # prioridad 1 (4x6)
    JA_p = pinv_damped(JA)
    N = np.eye(6) - JA_p @ JA                                      # proyector al espacio nulo
    d1 = -JA_p @ E1
    d2 = N @ pinv_damped(J3 @ N) @ (-e3 - J3 @ d1)                 # prioridad 2 dentro de N
    t = t + d1 + d2
    if k % 5 == 0 or k == 30:
        print(f"paso {k:2d}: s*={s:.3f}  |e_on|={np.linalg.norm(e1):.1e} "
              f"|e_par|={np.linalg.norm(e2):.1e}  avance={ug @ (p - a):+.4f}")
print("DOF libres tras prioridad 1 (autovalores de N ~ 1):", int((np.linalg.eigvalsh((N + N.T) / 2) > 0.5).sum()), "| rango JA:", np.linalg.matrix_rank(JA))
print("u·ug final (B):", tool_state(t)[1] @ ug)

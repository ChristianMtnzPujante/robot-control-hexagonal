"""TCP del CR5 sobre plano z=h Y sobre esfera: apilado vs meet (círculo).

Exploración del 23/09/2026 (decisión abierta "tareas CGA por primitivas",
ver ROADMAP Bloque 1): mismo conjunto válido, soluciones algo distintas.
Ejecutar desde la raíz del repo: `python3 docs/cga_meet_vs_apilado.py`.
"""
import sys, numpy as np
sys.path[:0] = ["src/shared_kernel", "src/controller_node", "src/geometry_kernel"]
import pygafro as g
from controller_node.adapters.ga_adapter import GaKinematicsAdapter, _CHAIN_NAME
S = GaKinematicsAdapter()._system
P = g.Point

def tcp(th):
    return np.asarray(S.computeKinematicChainMotor(_CHAIN_NAME, list(map(float, th))).toTransformationMatrix())[:3, 3]

def coeffs(mv):
    return np.asarray(mv.vector(), dtype=float).ravel()

th0 = np.array([0.3, -0.4, 1.4, 0.2, 1.3, 0.1])
c = tcp(th0) + np.array([0.08, 0.05, 0.0]); r = 0.12; h = c[2] - 0.04
plane = g.Plane(P(0, 0, h), P(1, 0, h), P(0, 1, h))
sphere = g.Sphere(P(*(c + [r, 0, 0])), P(*(c - [r, 0, 0])), P(*(c + [0, r, 0])), P(*(c + [0, 0, r])))
circle = (plane.dual() ^ sphere.dual()).dual()

def res_stack(th):
    X = P(*tcp(th)); return np.concatenate([coeffs(X ^ plane), coeffs(X ^ sphere)])
def res_meet(th):
    X = P(*tcp(th)); return coeffs(X ^ circle)

def solve(res, th):
    th = th.copy(); path = [tcp(th)]
    for k in range(100):
        e = res(th)
        if np.linalg.norm(e) < 1e-12: break
        J = np.column_stack([(res(th + 1e-7 * np.eye(6)[i]) - e) / 1e-7 for i in range(6)])
        th = th - J.T @ np.linalg.solve(J @ J.T + 1e-8 * np.eye(len(e)), e)
        path.append(tcp(th))
    return th, k, np.array(path)

for name, res in [("apilado", res_stack), ("meet", res_meet)]:
    th, k, path = solve(res, th0)
    p = tcp(th)
    print(f"{name:8s} it={k:3d} filas={len(res(th0)):2d}  dist plano={p[2]-h:+.1e}  "
          f"dist esfera={np.linalg.norm(p-c)-r:+.1e}\n         θ={np.round(th,4)}  p={np.round(p,4)}  "
          f"longitud recorrido TCP={np.linalg.norm(np.diff(path,axis=0),axis=1).sum():.4f}")

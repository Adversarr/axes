from root import *
import pyax
from pyax import fem
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

pyax.init()
pyax.set_log_level('warning')

mesh = fem.make_mesh_3d()
# E = np.array([0, 1, 2, 3], dtype=np.int64).reshape(-1, 1)
# V = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64).T
E = np.load(AX_ROOT + "/asset/mesh/npy/beam_mid_res_elements.npy").T
V = np.load(AX_ROOT + "/asset/mesh/npy/beam_mid_res_vertices.npy").T

mesh.SetNumDofPerVertex(3)
mesh.SetMesh(E, V)

stepper = fem.make_timestepper_3d(mesh)
stepper.Initialize()
stepper.SetExternalAccelerationUniform(np.array([0, 0, -9.8]))
opt = pyax.JsonObject({
    "elasticity": "stable_neohookean",
    "device": "gpu",
    "youngs": 1e7,
    "poisson_ratio": 0.45
    })
print(opt)
stepper.SetOptions(opt)

moving_bc = []

for i in range(V.shape[1]):
    x = V[:, i]
    if x[0] > 4.9:
        mesh.MarkDirichletBoundary(i, 0, x[0])
        mesh.MarkDirichletBoundary(i, 1, x[1])
        mesh.MarkDirichletBoundary(i, 2, x[2])
    elif x[0] < -4.9:
        moving_bc.append([i, x])
        mesh.MarkDirichletBoundary(i, 0, x[0])
        mesh.MarkDirichletBoundary(i, 1, x[1])
        mesh.MarkDirichletBoundary(i, 2, x[2])

rotate_around_x = np.array([
    [1, 0, 0],
    [0, np.cos(0.03 / 4), -np.sin(0.03 / 4)],
    [0, np.sin(0.03 / 4), np.cos(0.03 / 4)]
    ])


stepper.BeginSimulation(0.01)

def draw_all_elements():
    vertices = stepper.GetPosition()
    x = vertices[0, :]
    y = vertices[1, :]
    z = vertices[2, :]
    # Just Plot all the points.
    ax.scatter(x, y, z, c='r', marker='o')
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 2])
    ax.set_zlim([-5, 5])


while True:
    draw_all_elements()
    plt.pause(0.01)
    ax.clear()

    for id, (i, x) in enumerate(moving_bc):
        x = rotate_around_x @ x
        moving_bc[id][1] = x
        mesh.MarkDirichletBoundary(i, 0, x[0])
        mesh.MarkDirichletBoundary(i, 1, x[1])
        mesh.MarkDirichletBoundary(i, 2, x[2])

    stepper.BeginTimestep(0.01)
    stepper.SolveTimestep()
    stepper.EndTimestep()


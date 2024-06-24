import pyax
import numpy as np
pyax.gl.init(True)
pyax.init()
mesh = pyax.gl.Mesh()

mesh.vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float64).T
mesh.indices = np.array([[0, 1, 2]], dtype=np.int64).T
mesh.colors = np.array([[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]], dtype=np.float64).T
mesh.flush = True

ent = pyax.create_named_entity("drawing_from_python")
print(f'ent: {ent}')

pyax.gl.entity_add_or_replace_mesh(ent, mesh)
pyax.gl.enter_main_loop()
pyax.clean_up()
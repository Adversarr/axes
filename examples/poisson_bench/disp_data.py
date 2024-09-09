import os
from pathlib import Path
cur_path = Path(os.path.dirname(os.path.realpath(__file__)))
outs = cur_path / '..' / '..' / 'outs_poisson'

import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument('--i', type=int, default=0)
args = parser.parse_args()
i = args.i

g = np.load(outs / f'rhs_{i}.npy') # (N, b), rhs of the poisson problem
print(g.shape, np.linalg.norm(g))

p = np.load(outs / f'u0_{i}.npy') # (N, b), solution of the poisson problem
print(p.shape, np.linalg.norm(p))

vert = np.load(outs / 'mesh_vertices.npy') # (3, N)
print(vert.shape)

# 3d plot
fig = plt.figure()
ax = fig.add_subplot(121, projection='3d')
color = p[:, 0]
ax.scatter(vert[0], vert[1], vert[2], c=color, cmap='viridis')
ax = fig.add_subplot(122, projection='3d')
color = g[:, 0]
ax.scatter(vert[0], vert[1], vert[2], c=color, cmap='viridis')
plt.show()
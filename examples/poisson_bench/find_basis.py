import numpy as np
import scipy as sp
from pathlib import Path
import os

from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument('--comp', type=int, default=128)
args = parser.parse_args()
comp = args.comp

data_folder = Path('/home/adversarr/Repo/axes/outs_poisson')
dx =[f for f in os.listdir(data_folder) if 'u0' in f]
print(f'Size of the data: {len(dx)}')

# Load the data
data = np.concatenate([np.load(data_folder / f).T for f in dx])
data = np.concatenate([data, -data])
print(f'Shape of the data: {data.shape}')
N = data.shape[0]

# we expect that we find the basis of the data
from sklearn.decomposition import PCA
pca = PCA(n_components=comp) # we expect that the data is low-dimensional, dimension is 32
coordinate = pca.fit_transform(data)
print(pca.explained_variance_ratio_) # explained variance ratio: how much variance is explained by each component
print(pca.components_.shape)

basis = pca.components_
np.save('basis.npy', basis)
print(f'coordinate: {coordinate.shape}')
# plot the result for sample 0
import matplotlib.pyplot as plt
idx = 0 # np.random.randint(0, N)
fig = plt.figure()
ax = fig.add_subplot(122, projection='3d')
vertices = np.load(data_folder / 'mesh_vertices.npy') # (3, N)

sp0 = data[idx] # (N, )
transformed_sp0 = (coordinate[idx].reshape(-1, 1) * basis).sum(axis=0)
# transformed_sp0 = pca.inverse_transform(coordinate[idx].reshape(1, -1)).reshape(-1)
ax.scatter(vertices[0], vertices[1], vertices[2], c=sp0, cmap='viridis')

ax = fig.add_subplot(121, projection='3d')
ax.scatter(vertices[0], vertices[1], vertices[2], c=transformed_sp0, cmap='viridis')

print(f'Relative error: {np.linalg.norm(sp0 - transformed_sp0) / np.linalg.norm(sp0)}')
plt.show()


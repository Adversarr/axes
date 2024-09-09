import numpy as np
import scipy as sp
import sklearn
from pathlib import Path
import os

from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument('--ncomp', type=int, default=32)
args = parser.parse_args()
ncomp = args.ncomp

data_folder = Path('/home/adversarr/Repo/axes/outs')
dx =[f for f in os.listdir(data_folder) if 'pos' in f]
print(f'Size of the data: {len(dx)}')

# Load the data
data = np.array([np.load(data_folder / f) for f in dx])
print(f'Shape of the data: {data.shape}')
N = data.shape[0]
data = data.reshape((N, -1)) # (N, dof)

# we expect that we find the basis of the data
from sklearn.decomposition import PCA
pca = PCA(n_components=ncomp) # we expect that the data is low-dimensional, dimension is 32
coordinate = pca.fit_transform(data)
print(pca.explained_variance_ratio_) # explained variance ratio: how much variance is explained by each component
print(pca.components_.shape)

basis = pca.components_
np.save('basis.npy', basis)
basis = pca.components_.reshape((ncomp, -1, 3))

# plot the result for sample 0
import matplotlib.pyplot as plt
idx = np.random.randint(0, N)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
vertices = np.load(data_folder / 'initial.npy') # (3, N)

sp0 = vertices +  data[idx].reshape((-1, 3)).T # (3, N)
transformed_sp0 = vertices + (coordinate[idx].reshape(-1, 1, 1) * basis).sum(axis=0).reshape((-1, 3)).T
ax.scatter(sp0[0], sp0[1], sp0[2], c='r')
ax.scatter(transformed_sp0[0], transformed_sp0[1], transformed_sp0[2], c='b')
plt.show()
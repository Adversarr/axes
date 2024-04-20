import numpy as np
import scipy as sp
from scipy.sparse import coo_matrix

def load_sparse_matrix_coo(filename):
    triplets = []
    row, col, nnz = 0, 0, 0
    with open(filename, 'r') as f:
        lines = f.readlines()
        i = 0
        for i, line in enumerate(lines):
            if line.startswith('%'): continue
            row, col, nnz = map(int, line.split())
            break
        for j in range(i+1, len(lines)):
            if lines[j].startswith('%'): continue
            i, j, val = lines[j].split()
            i, j = int(i), int(j)
            val = float(val)
            triplets.append((i - 1, j - 1, val))
        ret = coo_matrix((np.array([t[2] for t in triplets]),
                          (np.array([t[0] for t in triplets]),
                           np.array([t[1] for t in triplets]))), shape=(row, col))
    return ret



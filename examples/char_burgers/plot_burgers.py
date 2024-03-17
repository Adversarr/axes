import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('python')
from root import axes_build_directory

from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument('--input', type=str, default=f"{axes_build_directory()}/examples/result.npy")
args = parser.parse_args()
file = args.input

# Load the data
def load_data():
    data = np.load(file)
    return data



data = load_data()
x = np.linspace(0, np.pi * 2, 256)
t = np.linspace(0, 3, 257)
T, X = np.meshgrid(t, x)
plt.figure()
plt.contourf(X, T, data, 100, cmap='plasma')
plt.colorbar()
plt.xlabel('$x$')
plt.ylabel('$t$')
plt.plot([np.pi, np.pi + 3], [0, 3], 'r--')
plt.figure()
ti = int(0.4 * 257 / 3)
plt.plot(x, data[:, ti], label='$t$=0.4')
plt.xlabel('$x$')
plt.ylabel('$u$')
plt.grid()
plt.xlim(0, np.pi * 2)

ti = int(1.0 * 257 / 3)
plt.plot(x, data[:, ti], label='$t$=1.0')
plt.legend()



plt.show()
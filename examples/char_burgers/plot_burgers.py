import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('python')
from root import axes_build_directory

# Load the data
def load_data():
    data = np.load(f"{axes_build_directory()}/examples/result.npy")
    return data


data = load_data()
plt.figure()
plt.imshow(data)
plt.figure()
plt.plot(data[:, 100])
plt.show()
from root import get_executable
from subprocess import call

import numpy as np

executable = get_executable('optim_hw')

def run(fista=False, max_iter=3000, verbose=False, prefix='out'):
    cmds = [executable, f'--max_iter={max_iter}', f'--stderrthreshold=1', f'--export={prefix}']
    if fista:
        cmds.append('--fista')
    call(cmds)

    optimal = np.load(f'{prefix}_x.npy')
    diff = np.load(f'{prefix}_diff.npy')
    return optimal, diff

import matplotlib.pyplot as plt

o, d = run(fista=True)
plt.plot(np.log(d + 1e-20))
plt.show()

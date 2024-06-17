from root import get_executable
from subprocess import call
import os

import numpy as np


MU = 1e-2

executable = get_executable('optim_hw')

def get_accurate(mu=MU):
    if not os.path.exists(f'accurate_{int(mu*1000)}_x.npy'):
        call([executable, '--tol_var=1e-12', '--max_iter=10000', f'--export=accurate_{int(mu*1000)}', f'--mu={mu}', '--fista'])
    return np.load(f'accurate_{int(mu*1000)}_x.npy')[:, -1], np.load(f'accurate_{int(mu*1000)}_energy.npy')[-1]


def run(fista=False, max_iter=3000, mu=MU):
    prefix = f'fista_{int(mu*1000)}' if fista else f'no_fista_{int(mu*1000)}'
    if not os.path.exists(f'{prefix}_x.npy'):
        cmds = [executable, f'--max_iter={max_iter}', f'--stderrthreshold=1', f'--export={prefix}', '--tol_var=1e-6', f'--mu={mu}']
        if fista:
            cmds.append('--fista')
        call(cmds)
    x = np.load(f'{prefix}_x.npy')
    diff = np.load(f'{prefix}_diff.npy')
    energy = np.load(f'{prefix}_energy.npy')
    return x, diff, energy

import matplotlib.pyplot as plt


def distance_to_optimal(x, x_opt=None):
    # x is [n_feat, n_iter]
    _, n_iter = x.shape
    x_opt_local = x_opt if x_opt is not None else x[:, -1]
    dist = np.zeros(n_iter)
    for i in range(n_iter):
        dist[i] = np.linalg.norm(x[:, i] - x_opt_local)
    return dist

def sparsity(x):
    return np.sum(np.abs(x) <= 1e-12) / x.size

def experiment_mu(mu):
    x_opt, f_opt = get_accurate(mu)
    no_fista_x, no_fista_diff, no_fista_energy = run(fista=False, max_iter=10000, mu=mu)
    fista_x, fista_diff, fista_energy = run(fista=True, max_iter=10000, mu=mu)

    f_opt = min(f_opt, fista_energy.min(), no_fista_energy.min())

    plt.figure(dpi=200)
    plt.plot(np.log10(distance_to_optimal(no_fista_x, x_opt)), label='Proximal Gradient')
    plt.plot(np.log10(distance_to_optimal(fista_x, x_opt)), label='FISTA')
    plt.legend()
    plt.title('$\\log\\|x_k - x^*\\|$ ' + f'for $\\mu$={mu}')
    plt.grid()
    plt.savefig(f'x_star_{int(mu*1000)}.png')

    plt.figure(dpi=200)
    plt.plot(np.log10(no_fista_diff), label='Proximal Gradient')
    plt.plot(np.log10(fista_diff), label='FISTA')
    plt.legend()
    plt.title('$\\log\\|x_{k} - x_{k+1}\\|/t_k$ for ' + f'$\\mu$={mu}')
    plt.grid()
    plt.savefig(f'diff_{int(mu*1000)}.png')

    plt.figure(dpi=200)
    plt.plot(np.log10(1e-18 + no_fista_energy - f_opt), label='Proximal Gradient')
    plt.plot(np.log10(1e-18 + fista_energy - f_opt), label='FISTA')
    plt.legend()
    plt.title('log-loss for ' + f'$\\mu$={mu}')
    plt.grid()
    plt.savefig(f'energy_{int(mu*1000)}.png')

    print(f'mu = {mu}')
    print(f'Sparsity for No FISTA: {sparsity(no_fista_x[:, -1])}, Iterations: {no_fista_x.shape[1]}')
    print(f'Sparsity for FISTA: {sparsity(fista_x[:, -1])}, Iterations: {fista_x.shape[1]}')
    print(f'f_opt = {f_opt}')

def test_sparsity_only(mu):
    fista_x, _, _ = run(fista=True, max_iter=10000, mu=mu)
    no_fista_x, _, _ = run(fista=False, max_iter=10000, mu=mu)
    print(f'mu = {mu}')
    print(f'Sparsity for No FISTA: {sparsity(no_fista_x[:, -1])}, Iterations: {no_fista_x.shape[1]}')
    print(f'Sparsity for FISTA: {sparsity(fista_x[:, -1])}, Iterations: {fista_x.shape[1]}')

test_sparsity_only(0.001)
experiment_mu(1e-2)
test_sparsity_only(0.05)

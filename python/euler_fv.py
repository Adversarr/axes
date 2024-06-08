import numpy as np
import matplotlib.pyplot as plt
from subprocess import call

from root import *

fv = get_executable('euler_fv')

gamma = 1.4
FLUX_LAX_FRIDRICH = 0
FLUX_HLL = 1
PROB_SOD = 0
T = 0.1

def do_experiment(nx, tend, periodic, problem, flux):
    assert flux in [FLUX_LAX_FRIDRICH, FLUX_HLL]
    assert problem in [PROB_SOD]
    call([fv, f'--nx={nx}', f'--tend={tend}', f'--periodic={periodic}', f'--problem={problem}', f'--flux_type={flux}'])
    density = np.load('output_density.npy').flatten()
    momentum = np.load('output_momentum.npy').flatten()
    energy = np.load('output_energy.npy').flatten()
    return density, momentum, energy


def pressure(density, momentum, energy):
    return (gamma - 1) * (energy - 0.5 * momentum**2 / density)

def velocity(density, momentum):
    return momentum / density

def integrate(f):
    return np.mean(f)

exact_nx = 512
ref_density, ref_momentum, ref_energy = do_experiment(exact_nx, T, 0, PROB_SOD, FLUX_HLL)


def l2(inexact, exact):
    n_exact = exact.shape[0]
    n_inexact = inexact.shape[0]
    assert n_exact % n_inexact == 0
    factor = n_exact // n_inexact
    exact_to_eval = exact.reshape(n_inexact, factor).mean(axis=1)
    return np.sqrt(np.mean((inexact - exact_to_eval)**2))

def l1(inexact, exact):
    n_exact = exact.shape[0]
    n_inexact = inexact.shape[0]
    assert n_exact % n_inexact == 0
    factor = n_exact // n_inexact
    exact_to_eval = exact.reshape(n_inexact, factor).mean(axis=1)
    return np.mean(np.abs(inexact - exact_to_eval))

def linf(inexact, exact):
    n_exact = exact.shape[0]
    n_inexact = inexact.shape[0]
    assert n_exact % n_inexact == 0
    factor = n_exact // n_inexact
    exact_to_eval = exact.reshape(n_inexact, factor).mean(axis=1)
    return np.max(np.abs(inexact - exact_to_eval))

l, r = -np.pi, np.pi
x = np.linspace(l, r, exact_nx)
# density = 1 + 0.2 * np.sin(x - T), velocity = 1, pressure = 1
true_density = 1 + np.sin(x - T) * 0.2
true_momentum = true_density
true_energy = 2.5 + 0.5 * true_density

plt.figure(dpi=200)
plt.subplot(221)
plt.xlim(l, r)
plt.ylim(0, 1.25);
plt.plot(x, ref_density, label='computed')
plt.plot(x, true_density, label='true')
plt.legend()
plt.title('Density')
plt.subplot(222)
plt.xlim(l, r)
plt.ylim(-0.25, 1.25);
plt.plot(x, velocity(ref_density, ref_momentum), label='computed')
plt.plot(x, velocity(true_density, true_momentum), label='true')
plt.title('velocity')
plt.subplot(223)
plt.xlim(l, r)
plt.plot(x, ref_energy, label='computed')
plt.plot(x, true_energy, label='true')
plt.title('energy')
plt.subplot(224)
plt.xlim(l, r)
plt.ylim(0, 1.25);
plt.plot(x, pressure(ref_density, ref_momentum, ref_energy), label='computed')
plt.plot(x, pressure(true_density, true_momentum, true_energy), label='true')
plt.title('pressure')
plt.show()

test_nx = [32, 64, 128, 256, ]

errors = {
    'density': [],
    'momentum': [],
    'energy': [],
}

for i, nx in enumerate(test_nx):
    density, momentum, energy = do_experiment(nx, T, 1, PROB_SOD, FLUX_LAX_FRIDRICH)
    dl2 = l1(density, ref_density)
    ml2 = l1(momentum, ref_momentum)
    el2 = l1(energy, ref_energy)
    if i > 0:
        # compute order
        dl2_old = errors['density'][-1]
        ml2_old = errors['momentum'][-1]
        el2_old = errors['energy'][-1]
        order_density = np.log(dl2 / dl2_old) / np.log(2)
        order_momentum = np.log(ml2 / ml2_old) / np.log(2)
        order_energy = np.log(el2 / el2_old) / np.log(2)
        print(f'density: {dl2:.2e} order: {order_density:.2f}')
        print(f'momentum: {ml2:.2e} order: {order_momentum:.2f}')
        print(f'energy: {el2:.2e} order: {order_energy:.2f}')
    errors['density'].append(dl2)
    errors['momentum'].append(ml2)
    errors['energy'].append(el2)

plt.figure(dpi=200)
plt.loglog(test_nx, errors['density'], label='density')
plt.loglog(test_nx, errors['momentum'], label='momentum')
plt.loglog(test_nx, errors['energy'], label='energy')
plt.show()


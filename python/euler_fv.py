import numpy as np
import matplotlib.pyplot as plt
from subprocess import call

from root import *

fv = get_executable('euler_fv')

gamma = 1.4
FLUX_LAX_FRIDRICH = 0
FLUX_HLL = 1
PROB_SOD = 0
PROB_NONE = 1

def do_experiment(nx, tend, periodic, problem, flux):
    assert flux in [FLUX_LAX_FRIDRICH, FLUX_HLL]
    assert problem in [PROB_SOD, PROB_NONE]
    call([fv, f'--nx={nx}', f'--tend={tend}', f'--periodic={periodic}', f'--problem={problem}', f'--flux_type={flux}', '--stderrthreshold=0'])
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

def do_experiment_continuous():
    T = 1
    exact_nx = 1024

    ref_density, ref_momentum, ref_energy = do_experiment(exact_nx, T, 1, PROB_SOD, FLUX_HLL)
    l, r = -np.pi, np.pi
    dx = (r - l) / exact_nx
    x = np.arange(l, r, dx)
    # density = 1 + 0.2 * np.sin(x - T), velocity = 1, pressure = 1
    true_density = 1 + 0.2 * (np.cos(x - T) - np.cos(x-T + dx)) / dx
    true_momentum = true_density
    true_energy = 2.5 + 0.5 * true_density

    true_pressure = np.ones_like(true_density)
    true_velocity = np.ones_like(true_density)

    plt.figure(figsize=(12, 8), dpi=150)
    plt.subplot(221)
    plt.xlim(l, r)
    plt.ylim(0, 1.25);
    plt.plot(x, ref_density, label='computed')
    plt.plot(x, true_density, '--', label='true')
    plt.grid()
    plt.legend()
    plt.title('Density')
    plt.subplot(222)
    plt.xlim(l, r)
    plt.ylim(-0.25, 1.25);
    plt.plot(x, velocity(ref_density, ref_momentum), label='computed')
    plt.plot(x, velocity(true_density, true_momentum), '--', label='true')
    plt.grid()
    plt.legend()
    plt.title('Velocity')
    plt.subplot(223)
    plt.xlim(l, r)
    plt.plot(x, ref_energy, label='computed')
    plt.plot(x, true_energy, '--', label='true')
    plt.grid()
    plt.legend()
    plt.title('Energy')
    plt.subplot(224)
    plt.xlim(l, r)
    plt.ylim(0, 1.25);
    plt.plot(x, pressure(ref_density, ref_momentum, ref_energy), label='computed')
    plt.plot(x, pressure(true_density, true_momentum, true_energy),'--', label='true')
    plt.grid()
    plt.legend()
    plt.title('Pressure')


    test_nx = [32, 64, 128, 256, 512, ]

    linferrors = {
        'density': [],
        'momentum': [],
        'velocity': [],
        'energy': [],
        'pressure': [],
    }
    l1errors = {
        'density': [],
        'momentum': [],
        'velocity': [],
        'energy': [],
        'pressure': [],
    }

    for i, nx in enumerate(test_nx):
        density, momentum, energy = do_experiment(nx, T, 1, PROB_SOD, FLUX_LAX_FRIDRICH)
        dlinf = linf(density, true_density)
        mlinf = linf(momentum, true_momentum)
        vlinf = linf(velocity(density, momentum), true_velocity)
        elinf = linf(energy, true_energy)
        plinf = linf(pressure(density, momentum, energy), true_pressure)
        dl1 = l1(density, true_density)
        ml1 = l1(momentum, true_momentum)
        vl1 = l1(velocity(density, momentum), true_velocity)
        el1 = l1(energy, true_energy)
        pl1 = l1(pressure(density, momentum, energy), true_pressure)
        if i > 0:
            # compute order
            dlinf_old = linferrors['density'][-1]
            mlinf_old = linferrors['momentum'][-1]
            vlinf_old = linferrors['velocity'][-1]
            elinf_old = linferrors['energy'][-1]
            plinf_old = linferrors['pressure'][-1]
            order_density = np.log(dlinf / dlinf_old) / np.log(2)
            order_momentum = np.log(mlinf / mlinf_old) / np.log(2)
            order_velocity = np.log(vlinf / vlinf_old) / np.log(2)
            order_energy = np.log(elinf / elinf_old) / np.log(2)
            order_pressure = np.log(plinf / plinf_old) / np.log(2)
            print(f'density: {dlinf:.2e} order: {order_density:.2f}')
            print(f'momentum: {mlinf:.2e} order: {order_momentum:.2f}')
            print(f'velocity: {vlinf:.2e} order: {order_velocity:.2f}')
            print(f'energy: {elinf:.2e} order: {order_energy:.2f}')
            print(f'pressure: {plinf:.2e} order: {order_pressure:.2f}')
 
            dl1_old = l1errors['density'][-1]
            ml1_old = l1errors['momentum'][-1]
            vl1_old = l1errors['velocity'][-1]
            el1_old = l1errors['energy'][-1]
            pl1_old = l1errors['pressure'][-1]
            order_density = np.log(dl1 / dl1_old) / np.log(2)
            order_momentum = np.log(ml1 / ml1_old) / np.log(2)
            order_velocity = np.log(vl1 / vl1_old) / np.log(2)
            order_energy = np.log(el1 / el1_old) / np.log(2)
            order_pressure = np.log(pl1 / pl1_old) / np.log(2)
            print(f'density: {dl1:.2e} order: {order_density:.2f}')
            print(f'momentum: {ml1:.2e} order: {order_momentum:.2f}')
            print(f'velocity: {vl1:.2e} order: {order_velocity:.2f}')
            print(f'energy: {el1:.2e} order: {order_energy:.2f}')
            print(f'pressure: {pl1:.2e} order: {order_pressure:.2f}')

        linferrors['density'].append(dlinf)
        linferrors['momentum'].append(mlinf)
        linferrors['velocity'].append(vlinf)
        linferrors['energy'].append(elinf)
        linferrors['pressure'].append(plinf)
        l1errors['density'].append(dl1)
        l1errors['momentum'].append(ml1)
        l1errors['velocity'].append(vl1)
        l1errors['energy'].append(el1)
        l1errors['pressure'].append(pl1)

    plt.figure(figsize=(10, 4), dpi=200)
    plt.subplot(121)
    plt.loglog(test_nx, linferrors['density'], '+-',label='density')
    plt.loglog(test_nx, linferrors['momentum'], '*-', label='momentum')
    plt.loglog(test_nx, linferrors['energy'], 'x-', label='energy')
    plt.loglog(test_nx, np.array(test_nx, dtype=np.float64)**-3, '--', label='$1/h^3$')
    plt.legend()
    plt.grid()
    plt.title('$L^\\infty$ error')
    plt.subplot(122)
    plt.loglog(test_nx, l1errors['density'], '+-',label='density')
    plt.loglog(test_nx, l1errors['momentum'], '*-', label='momentum')
    plt.loglog(test_nx, l1errors['energy'], 'x-', label='energy')
    plt.loglog(test_nx, np.array(test_nx, dtype=np.float64)**-3, '--', label='$1/h^3$')
    plt.legend()
    plt.grid()
    plt.title('$L^1$ error')
    plt.show()

    # print the error table in typst
    # name,
    # l1e, l1o, linfe, linfo
    for name in ['density', 'momentum', 'velocity', 'energy', 'pressure']:
        print(f'[{test_nx[0]}], [{l1errors[name][0]}], [--], [{linferrors[name][0]}], [--],')
        for i, nx in enumerate(test_nx[1:]):
            print(f'[{nx}], ', end='')
            for errt in [l1errors, linferrors]:
                err = errt[name][i + 1]
                err_last = errt[name][i]
                order = np.log(err / err_last) / np.log(2)
                print(f'[{err}], [{-order}], ', end='')
            print('')





def do_experiment_discontinuous():
    compute_nx = 128
    ref_nx = 4096
    # standard SOD problem
    # l, r = -1, 1
    # density, momentum, energy = do_experiment(compute_nx, .2, 0, PROB_SOD, FLUX_HLL)
    # ref_density, ref_momentum, ref_energy = do_experiment(ref_nx, 0.2, 0, PROB_SOD, FLUX_HLL)

    l, r = -6, 6
    density, momentum, energy = do_experiment(compute_nx, 1.3, 0, PROB_NONE, FLUX_LAX_FRIDRICH)
    ref_density, ref_momentum, ref_energy = do_experiment(ref_nx, 1.3, 0, PROB_NONE, FLUX_HLL)

    x = np.linspace(l, r, compute_nx)
    ref_x = np.linspace(l, r, ref_nx)

    plt.figure(figsize=(12, 8), dpi=200)
    plt.subplot(221)
    plt.xlim(l, r)
    plt.ylim(0, 1.25);
    plt.plot(x, density, label='computed')
    plt.plot(ref_x, ref_density, '--', label='reference')
    plt.grid()
    plt.legend()
    plt.title('Density')
    plt.subplot(222)
    plt.xlim(l, r)
    plt.ylim(-0.25, 1.25);
    plt.plot(x, velocity(density, momentum), label='computed')
    plt.plot(ref_x, velocity(ref_density, ref_momentum), '--', label='reference')
    plt.legend()
    plt.grid()
    plt.title('Velocity')
    plt.subplot(223)
    plt.xlim(l, r)
    plt.plot(x, energy, label='computed')
    plt.plot(ref_x, ref_energy, '--', label='reference')
    plt.title('Energy')
    plt.grid()
    plt.legend()
    plt.subplot(224)
    plt.xlim(l, r)
    plt.ylim(0, 1.25);
    plt.plot(x, pressure(density, momentum, energy), label='computed')
    plt.plot(ref_x, pressure(ref_density, ref_momentum, ref_energy), '--', label='reference')
    plt.title('Pressure')
    plt.grid()
    plt.legend()
    plt.show()


# do_experiment_continuous()
do_experiment_discontinuous()

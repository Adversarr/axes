"""
Solves 1d scalar conservation law by method of lines, using WENO reconstruction

- should have 3rd order accuracy in space and time, if the solution is smooth
- should have at-least 1rd order accuracy in space and time anyway.
"""

import numpy as np
import matplotlib.pyplot as plt

# SECT: Eqn. Replace with your own one.
# u_t + f(u)_x = 0

# f
def fn(u):
    return 0.5 * u ** 2
# pfpx 
def pfn(u):
    return u


# SECT: WENO
def weno_left(u):
    u_ll = np.roll(u, 2)
    u_l = np.roll(u, 1)
    u_c = u
    u_r = np.roll(u, -1)
    u_rr = np.roll(u, -2)
    left0 = (-u_ll + 5 * u_l + 2 * u_c) / 6
    left1 = (2 * u_l + 5 * u_c - u_r) / 6
    left2 = (11 * u_c - 7 * u_r + 2 * u_rr) / 6
    return left0, left1, left2

def weno_right(u):
    u_ll = np.roll(u, 2)
    u_l = np.roll(u, 1)
    u_c = u
    u_r = np.roll(u, -1)
    u_rr = np.roll(u, -2)
    right0 = (2 * u_ll - 7 * u_l + 11 * u_c) / 6
    right1 = (-u_l + 5 * u_c + 2 * u_r) / 6
    right2 = (2 * u_c + 5 * u_r - u_rr) / 6
    return right0, right1, right2

def smooth_indicator(u):
    u_ll = np.roll(u, 2)
    u_l = np.roll(u, 1)
    u_c = u
    u_r = np.roll(u, -1)
    u_rr = np.roll(u, -2)
    d0 = 13.0 / 12.0 * (u_ll - 2 * u_l + u_c) ** 2 + 0.25 * (u_ll - 4 * u_l + 3 * u_c) ** 2
    d1 = 13.0 / 12.0 * (u_l - 2 * u_c + u_r) ** 2 + 0.25 * (u_l - u_r) ** 2
    d2 = 13.0 / 12.0 * (u_c - 2 * u_r + u_rr) ** 2 + 0.25 * (3 * u_c - 4 * u_r + u_rr) ** 2
    eps = 1e-12
    alpha0 = 0.1 / ((d0 + eps) ** 2)
    alpha1 = 0.6 / ((d1 + eps) ** 2)
    alpha2 = 0.3 / ((d2 + eps) ** 2)
    w0 = alpha0 / (alpha0 + alpha1 + alpha2)
    w1 = alpha1 / (alpha0 + alpha1 + alpha2)
    w2 = alpha2 / (alpha0 + alpha1 + alpha2)
    return w0, w1, w2

def weno(u):
    l0, l1, l2 = weno_left(u)
    r0, r1, r2 = weno_right(u)
    w0, w1, w2 = smooth_indicator(u)
    u_minus = w0 * r0 + w1 * r1 + w2 * r2
    u_plus  = w0 * l0 + w1 * l1 + w2 * l2
    u_plus = np.roll(u_plus, -1)
    return u_minus, u_plus

def lf_flux(u_minus, u_plus):
    pfn_m = 0.5 * (np.max(np.abs(pfn(u_minus))) + np.max(np.abs(pfn(u_plus))))
    return 0.5 * (fn(u_minus) + fn(u_plus) - pfn_m * (u_plus - u_minus))

# SECT: Time Stepping
def rk1(u, dtdx):
    flux = lf_flux(*weno(u))
    return u - dtdx * (flux - np.roll(flux, 1))

def rk3(u, dtdx):
    # These coefficients are optimal and SSP. Proved and Privided by C. W. Shu
    u1 = rk1(u, dtdx)
    u2 = rk1(u1, dtdx) * 0.25 + u * 0.75
    return 1/3 * (u + 2 * rk1(u2, dtdx))

# ensure stability
def cfl_dt(dx, u):
    return 0.5 * dx / np.max(np.abs(pfn(u)))

def main(require_T = 1.5, N=320):
    u0 = np.sin(np.linspace(0, 2 * np.pi, N)) + 0.5
    u = u0.copy()
    dx = 2 * np.pi / N
    T = 0
    while T < require_T:
        dt = cfl_dt(dx, u)
        if T + dt > require_T:
            dt = require_T - T
        T += dt
        dtdx = dt / dx
        u = rk3(u, dtdx)
    plt.plot(np.linspace(0, 2 * np.pi, N), u0, label='u0')
    plt.plot(np.linspace(0, 2 * np.pi, N), u, label='u')
    plt.title(f'T={T}')
    plt.legend()
    plt.show()

main()

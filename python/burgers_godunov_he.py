from root import *
import os
import subprocess

import numpy as np
import matplotlib.pyplot as plt
bg = get_executable('burgers_high_order')
accurate = get_executable('char_burgers')
Nx_list = [10, 20, 40, 80, 160, 320]

T = 0.5

for Nx in Nx_list:
  args = [bg, f"--Nx={Nx}", f'--T={T}']
  subprocess.call(args)
  args = [accurate, f'--alpha=0.5', f'--nx={1+Nx}', '--export', f'--export_file=exact_{Nx}_{int(10*T)}.npy', f'--t={T}']
  subprocess.call(args)

super_res = 640 * 2
args = [accurate, f'--alpha=0.5', f'--nx={super_res + 1}', '--export', f'--export_file=exact_{super_res}_{int(10*T)}.npy', f'--t={T}']
subprocess.call(args)
g_exact = list(np.load(f'exact_{super_res}_{int(10*T)}.npy')[:-1, -1])
def run_trapz_on_exact_solution(Nx):
  result = []
  interval = super_res // Nx
  for i in range(0, Nx):
    sum = 0.
    w = 0.
    left = i * interval - interval // 2
    right = i * interval + interval // 2
    for j in range(left, right + 1):
      if j == left or j == right:
        sum += g_exact[j] / 3
        w += 1/3
      elif (j + interval // 2) % 2 == 0:
        sum += g_exact[j] * 2.0 / 3.0
        w += 2/3
      else:
        sum += g_exact[j] * 4.0 / 3.0
        w += 4/3
    result.append(sum / w)
  result.append(result[0])
  return np.array(result)

# do compare
exact_for_plot = np.load(f'exact_{Nx_list[-1]}_{int(10*T)}.npy')[:, -1]
l1_list = []
l_inf_list = []
for Nx in Nx_list:
  # exact = np.load(f'exact_{Nx}_{int(10*T)}.npy')[:, -1]
  exact = run_trapz_on_exact_solution(Nx)
  godunov = np.load(f'u{Nx}_{int(10 * T)}.npy')[:, -1]
  l_inf = np.max(np.abs(godunov - exact))
  l1 = (np.sum(np.abs(godunov - exact)) / Nx) * 2 * np.pi
  l_inf_list.append(l_inf)
  l1_list.append(l1)
  plt.figure(dpi=200)
  plt.plot(np.linspace(0, np.pi * 2, 1+Nx), godunov, label=f'Godunov Nx={Nx}')
  # plt.plot(np.linspace(0, np.pi * 2, 1+Nx), exact, label=f'MY EXACT Nx={Nx}')
  plt.plot(np.linspace(0, np.pi * 2, 1+Nx_list[-1]), exact_for_plot, label=f'Exact Nx={Nx}',linestyle= '--')
  plt.legend()
  plt.title(f'Godunov vs Exact at T={T} $N_x$={Nx}')
  plt.annotate(f'$l_\\infty$={l_inf:.1e}', (1, 0), textcoords="offset points", xytext=(0, 10), ha='center')
  plt.annotate(f'$l_1$={l1:.1e}', (1, 0), textcoords="offset points", xytext=(0, 20), ha='center')
  plt.xlabel('x')
  plt.ylabel('u')
  plt.savefig(f'{Nx}_{int(10*T)}.png')

def plot_err_nx(Nx_list, l_inf_list, l1_list, T, prefix):
  plt.figure(dpi=300)
  plt.title('Error v.s. $N_x$')
  plt.loglog(Nx_list, l_inf_list, label='$l_\\infty$', marker='o')
  plt.loglog(Nx_list, l1_list, label='$l_1$', marker='o')

  # compute the order:
  for i in range(1, len(Nx_list)):
    l_inf_increased = l_inf_list[i] / l_inf_list[0]
    l1_increased = l1_list[i] / l1_list[0]
    Nx_increased = Nx_list[i] / Nx_list[0]
    l_inf_order = -np.log(l_inf_increased) / np.log(Nx_increased)
    l1_order = -np.log(l1_increased) / np.log(Nx_increased)
    print(f'[{prefix}]: Nx={Nx_list[i-1]} to Nx={Nx_list[i]}: l_inf_order={l_inf_order}, l1_order={l1_order}')


  # show the data around the markers.
  for i, txt in enumerate(l_inf_list):
    plt.annotate(f'{txt:.1e}', (Nx_list[i], l_inf_list[i]), textcoords="offset points", xytext=(0, 10), ha='center')
  for i, txt in enumerate(l1_list):
    plt.annotate(f'{txt:.1e}', (Nx_list[i], l1_list[i]), textcoords="offset points", xytext=(0, 10), ha='center')

  plt.xlabel('Nx')
  plt.ylabel('Error')
  plt.xticks(Nx_list, labels=Nx_list)  # Set the x-axis ticks to be the values in Nx_list
  plt.legend()
  plt.grid()
  plt.savefig(f'{prefix}_E_Nx_{int(10*T)}.png')
  plt.close()

plot_err_nx(Nx_list, l_inf_list, l1_list, T, 't05')

T = 1.5
Nx_list = [10, 20, 40, 80]

subprocess.call([accurate, f'--alpha=0.5', f'--nx={Nx_list[-1]+1}', '--export', 
                 f'--export_file=exact_{Nx_list[-1]}_{int(10*T)}.npy', f'--t={T}'])
args = [accurate, f'--alpha=0.5', f'--nx={super_res + 1}', '--export', 
        f'--export_file=exact_{super_res}_{int(10*T)}.npy', f'--t={T}']
subprocess.call(args)

g_exact = list(np.load(f'exact_{super_res}_{int(10*T)}.npy')[:-1, -1])
exact_for_plot = np.load(f'exact_{Nx_list[-1]}_{int(10*T)}.npy')[:, -1]
LIMITOR_NAME = ["None", "TVD", "TVB"]
M = [0.1, 1, 10]
FLUX = ['LF', 'Godunov']
for f, flux in enumerate(FLUX):
  for l, lname in enumerate(LIMITOR_NAME):
    for m in (M if l == 2 else [0.1]):
      l_inf_list = []
      l1_list = []
      for Nx in Nx_list:
        subprocess.call([bg, f"--Nx={Nx}", f'--T={T}', f'--limiter_type={l}', f'--M={m}', f'--flux_type={f}'])
        exact = run_trapz_on_exact_solution(Nx)
        godunov = np.load(f'u{Nx}_{int(10 * T)}.npy')[:, -1]

        l_inf = np.max(np.abs(godunov - exact))
        l1 = (np.sum(np.abs(godunov - exact)) / Nx) * 2 * np.pi
        l_inf_list.append(l_inf)
        l1_list.append(l1)

        plt.figure(dpi=200)
        plt.plot(np.linspace(0, np.pi * 2, 1+Nx), godunov, label=f'Godunov Nx={Nx}')
        # plt.plot(np.linspace(0, np.pi * 2, 1+Nx), exact, label=f'MY EXACT Nx={Nx}')
        plt.plot(np.linspace(0, np.pi * 2, 1+Nx_list[-1]), exact_for_plot, label=f'Exact Nx={Nx}',linestyle= '--')
        plt.legend()
        if lname == 'TVB':
          plt.title(f'Flux={flux} T={T} $N_x$={Nx} Limiter: {lname} M={m}')
        else:
          plt.title(f'Flux={flux} T={T} $N_x$={Nx} Limiter: {lname}')
        plt.annotate(f'$l_\\infty$={l_inf:.1e}', (1, 0), textcoords="offset points", xytext=(0, 10), ha='center')
        plt.annotate(f'$l_1$={l1:.1e}', (1, 0), textcoords="offset points", xytext=(0, 20), ha='center')
        plt.xlabel('x')
        plt.ylabel('u')
        plt.savefig(f'{lname}_{flux}_{int(np.log10(m))}_{Nx}_{int(10*T)}.png')
        plt.close()
      plot_err_nx(Nx_list, l_inf_list, l1_list, T, f'l{l}m{int(m*10)}f{flux}')


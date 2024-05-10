from root import *
import os
import subprocess

import numpy as np
import matplotlib.pyplot as plt
bg = get_executable('burgers_high_order')
accurate = get_executable('char_burgers')
Nx_list = [10, 20, 40, 80, 160, 320]

T = 1.5

for Nx in Nx_list:
    args = [bg, f"--Nx={Nx}", f'--T={T}', '--weno', '--flux_type=0', '--time_high_accuracy']
    subprocess.call(args)

# a extremely high resolution for the exact solution
super_res = 640 * 16
args = [bg, f"--Nx={super_res}", f'--T={T}', '--weno', '--flux_type=0']
subprocess.call(args)

accurate = np.load(f'u{super_res}_{int(10*T)}.npy')[:, -1]
def run_trapz_on_exact_solution(Nx):
    result = []
    interval = super_res // Nx
    for i in range(Nx):
        sum = 0.
        cnt = 0
        for j in range(i * interval - interval // 2, i * interval + interval // 2):
            sum += accurate[(j + super_res) % super_res]
            cnt += 1
        sum /= cnt
        result.append(sum)
    result.append(result[0])
    result = np.array(result)
    return result;


# args = [accurate, f'--alpha=0.5', f'--nx={1+Nx_list[-1]}', '--export', f'--export_file=exact_{Nx_list[-1]}_{int(10*T)}.npy', f'--t={T}']
# subprocess.call(args)
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
  lios = []
  l1os = []
  for i in range(1, len(Nx_list)):
    l_inf_increased = l_inf_list[i] / l_inf_list[0]
    l1_increased = l1_list[i] / l1_list[0]
    Nx_increased = Nx_list[i] / Nx_list[0]
    l_inf_order = -np.log(l_inf_increased) / np.log(Nx_increased)
    l1_order = -np.log(l1_increased) / np.log(Nx_increased)
    l1os.append(l1_order); lios.append(l_inf_order)
    print(f'[{prefix}]: Nx={Nx_list[i-1]} to Nx={Nx_list[i]}: l_inf_order={l_inf_order}, l1_order={l1_order}')

  for i in range(len(Nx_list)):
    if i == 0:
      print(f'[{Nx_list[i]}], [{l1_list[i]:.2e}], [{l_inf_list[i]:.2e}], [--], [--]')
    else:
      print(f'[{Nx_list[i]}], [{l1_list[i]:.2e}], [{l_inf_list[i]:.2e}], [{l1os[i-1]:.2e}], [{lios[i-1]:.2e}]')

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


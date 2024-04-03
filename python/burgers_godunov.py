from root import *
import os
import subprocess
bg = get_executable('burgers_godunov')
accurate = get_executable('char_burgers')
Nx_list = [10, 20, 40, 80, 160, 320]

T = 0.5

for Nx in Nx_list:
  args = [bg, f"--Nx={Nx}", f'--T={T}']
  print(args)
  subprocess.call(args)
  args = [accurate, f'--alpha=0.5', f'--nx={Nx}', '--export', f'--export_file=exact_{Nx}_{int(10*T)}.npy', f'--t={T}']
  print(args)
  subprocess.call(args)

import numpy as np
import matplotlib.pyplot as plt

# do compare
exact_for_plot = np.load(f'exact_{Nx_list[-1]}_{int(10*T)}.npy')
l2_list = []
l_inf_list = []
for Nx in Nx_list:
  exact = np.load(f'exact_{Nx}_{int(10*T)}.npy')
  godunov = np.load(f'u{Nx}_{int(10 * T)}.npy')
  l_inf = np.max(np.abs(godunov[:, -1] - exact[:, -1]))
  l2 = np.sqrt(np.sum((godunov[:, -1] - exact[:, -1])**2) / Nx) * 2 * np.pi
  l_inf_list.append(l_inf)
  l2_list.append(l2)
  plt.figure(dpi=300)
  plt.plot(np.linspace(0, np.pi * 2, Nx), godunov[:, -1], label=f'Godunov Nx={Nx}')
  plt.plot(np.linspace(0, np.pi * 2, Nx_list[-1]), exact_for_plot[:, -1], label=f'Exact Nx={Nx}',linestyle= '--')
  plt.legend()
  plt.title(f'Godunov vs Exact at T={T} $N_x$={Nx}')
  plt.annotate(f'$l_\infty$={l_inf:.1e}', (1, 0), textcoords="offset points", xytext=(0, 10), ha='center')
  plt.annotate(f'$l_2$={l2:.1e}', (1, 0), textcoords="offset points", xytext=(0, 20), ha='center')
  plt.xlabel('x')
  plt.ylabel('u')
  plt.savefig(f'godunov_vs_exact_{Nx}_{int(10*T)}.png')

plt.figure(dpi=300)
plt.title('Error v.s. $N_x$')
plt.ylim(6e-3, 1.5)
plt.xlim(8, 400)
plt.loglog(Nx_list, l_inf_list, label='$l_\infty$', marker='o')
plt.loglog(Nx_list, l2_list, label='$l_2$', marker='o')
# show the data around the markers.
for i, txt in enumerate(l_inf_list):
  plt.annotate(f'{txt:.1e}', (Nx_list[i], l_inf_list[i]), textcoords="offset points", xytext=(0, 10), ha='center')
for i, txt in enumerate(l2_list):
  plt.annotate(f'{txt:.1e}', (Nx_list[i], l2_list[i]), textcoords="offset points", xytext=(0, 10), ha='center')

plt.xlabel('Nx')
plt.ylabel('Error')
plt.xticks(Nx_list, labels=Nx_list)  # Set the x-axis ticks to be the values in Nx_list
plt.legend()
plt.grid()
plt.savefig(f'error_vs_Nx_{int(10*T)}.png')
# plt.show()

T = 1.5
Nx_list = [20, 80]

for Nx in Nx_list:
  subprocess.call([bg, f"--Nx={Nx}", f'--T={T}'])
  subprocess.call([accurate, f'--alpha=0.5', f'--nx={Nx}', '--export', f'--export_file=exact_{Nx}_{int(10*T)}.npy', f'--t={T}'])

exact_for_plot = np.load(f'exact_{Nx_list[-1]}_{int(10*T)}.npy')
for Nx in Nx_list:
  exact = np.load(f'exact_{Nx}_{int(10*T)}.npy')
  godunov = np.load(f'u{Nx}_{int(10 * T)}.npy')
  l_inf = np.max(np.abs(godunov[:, -1] - exact[:, -1]))
  l2 = np.sqrt(np.sum((godunov[:, -1] - exact[:, -1])**2) / Nx) * 2 * np.pi
  plt.figure(dpi=300)
  plt.plot(np.linspace(0, np.pi * 2, Nx), godunov[:, -1], label=f'Godunov Nx={Nx}')
  plt.plot(np.linspace(0, np.pi * 2, Nx_list[-1]), exact_for_plot[:, -1], label=f'Exact Nx={Nx}',linestyle= '--')
  plt.legend()
  plt.title(f'Godunov vs Exact at T={T} $N_x$={Nx}')
  plt.annotate(f'$l_\infty$={l_inf:.1e}', (1, 0), textcoords="offset points", xytext=(0, 10), ha='center')
  plt.annotate(f'$l_2$={l2:.1e}', (1, 0), textcoords="offset points", xytext=(0, 20), ha='center')
  plt.xlabel('x')
  plt.ylabel('u')
  plt.savefig(f'godunov_vs_exact_{Nx}_{int(10*T)}.png')
  # plt.show()

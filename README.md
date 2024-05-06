# axes

`axes` is a utility-library for Physical Simulation in Computer Graphics.

![](./asset/gh-images/framework.png)

System Requirement:

- gcc>=12, or
- clang>=16.0, or
- MSVC: VS2022

`axes` use C++20 code standard.

## What's in

1. A basic library for general purpose CG physical simulation, containing
   1. Math library using Eigen;
   2. Utilities library, such as abseil's Status, Logging, Flags;
   3. A tiny optimization library
   4. Basic geometry functionalities
2. Graphics for visulization. which supports 
   1. Trimesh/line/point visualization (with phong shading)
   2. ImGUI integration.
   3. and something more...
3. A trival node system, for fast prototype development, with moderate performance and high flexibility.

## System Dependencies

These settings are tested on:
1. MacOS-latest (m1)
2. Windows 11 latest, with Visual Studio 2022 latest (x64)
3. Arch Linux latest (x64)


### Common Dependencies

Refer to [axdeps](https://github.com/Adversarr/axdeps.git), and run the install script.

For Unix-like System
```bash
export SDK_PATH="path/to/axes"
export BUILD_TYPE=Release
path/to/axdeps/build.sh
```

For Windows
```powershell
$env:SDK_PATH="path/to/axes"
$env:BUILD_TYPE="Release"
path/to/axdeps/build.ps1
```

(Optional) Python binding, please refer to `requirements_cpu.txt`:

```bash
pip install -r requirements_cpu.txt
```

for additional GPU dependencies, please refer to `requirements_gpu.txt`:

```bash
pip install -r requirements_gpu.txt
```

The prefered Deep Learning Framework is [PyTorch](https://pytorch.org/), version=`2.3.0`.

### Platform Dependent Dependencies

For mac users, because we do not support Metal, nothing is required.

Windows:
- Install Visual Studio 2022 with C++ Development Kit;
- (Optional) CUDA Toolkit from NVIDIA.

Although the library is able to compile on Windows, the performance of both compilation and runtime is not guaranteed.

Arch Linux:
```
yay -S libcap gperf libsystemd libmount liblzma xz dbus
# If you prefer a clang compiler
yay -S clang15
# (Optional) CUDA Toolkit, However CUDA is always the latest from arch linux install.
yay -S cuda
```

For Ubuntu:
```bash
# CPU Version.
sudo apt install base-devel
```

CUDA is also available for boost up the performance. Refer to the CUDA official installation guide for more information.

- My CUDA version: `12.4.131`

# Performance Illustration

![fem3LBFGS](asset/gh-images/image.png)

FEM-3D, with NeoHookean energy, naive L-BFGS optimizer. (Youngs=1e7, density=1e1, poisson ratio=0.3) solves 32000 vertices @24 steps per second.

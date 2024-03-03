# axes

`axes` is a utility-library for Physical Simulation in Computer Graphics.

System Requirement:

- gcc>=12, or
- clang>=16.0, or
- MSVC: VS2022

`axes` use C++20 code standard.

You have to use `vcpkg` for CMake package management:

```sh
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
```

Without `vcpkg`, you have to make sure CMake can find the required packages listed in `vcpkg.json`.

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

## System Dependencies

Arch Linux:

```
yay -S libcap 
yay -S libcap gperf libsystemd
yay -S libmount
yay -S liblzma
yay -S clang15
yay -S xz dbus
```


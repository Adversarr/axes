# axes

`axes` is a utility-library for Physical Simulation in Computer Graphics.

System Requirement:

- gcc
- clang: 15.0

## What is in

1. A basic library for general purpose CG physical simulation, containing
   1. math library using Eigen;
   2. utilities library, such as abseil's Status, Logging, Flags;
   3. A tiny optimization library
2. Graphics for visulization. which supports trimesh visualization.
   1. ImGUI integration.
   2. and something more...

## Dependencies

Arch Linux:

```
yay -S libcap 
yay -S libcap gperf libsystemd
yay -S libmount
yay -S liblzma
yay -S clang15
yay -S xz dbus
```


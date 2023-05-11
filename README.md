# axes

Axes is yet another CG tool.

## Requirements

1. c++1z supported, gcc/clang compiler, msvc may be supported;
2. CMake, with vcpkg integration
3. Vulkan (for graphics, optional)

To build

```sh
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=${VCPKG_TOOLCHAIN_FILE}
```



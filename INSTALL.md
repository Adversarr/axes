## Trouble Shooting for cuda installation

Many system does not provide a stable cuda and its dependency such as `gcc`. 
This issue is frequently encountered under Arch Linux.

To fix this, you can use the following instruction to install the `gcc` 
and `cuda` in your `conda` environment. Install the `cuda` package should 
install the corresponding `gcc` at the same time.

```bash
conda install -c nvidia cuda
```

> See: [anaconda/nvidia/cuda](https://anaconda.org/nvidia/cuda) for more details

Then, you need to activate your environment, and set the `CMAKE_C_COMPILER`
and `CMAKE_CXX_COMPILER` to the `gcc` and `g++` in your `conda` environment.

```bash
cmake -S . -B build \
  -DCMAKE_C_COMPILER=gcc \
  -DCMAKE_CXX_COMPILER=g++
```


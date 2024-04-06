# Check Generator Ninja.
export CMAKE_GENERATOR="Unix Makefiles"
if [ -x "$(command -v ninja)" ]; then
  export CMAKE_GENERATOR=Ninja
fi

# Check Compiler.
if [ -z "$CC" ]; then
  export CC=$(which clang)
fi
if [ -z "$CXX" ]; then
  export CXX=$(which clang++)
fi
if ! [ -x "$(command -v clang)" ]; then
  export CC=gcc
  export CXX=g++
fi

if [ -z "$CC" ]; then
  echo "C compiler not found"
  exit 1
fi
if [ -z "$CXX" ]; then
  echo "C++ compiler not found"
  exit 1
fi

echo "Configuring project"
echo "C compiler: $CC"
echo "C++ compiler: $CXX"

cmake -S . -B build \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_C_COMPILER="$CC" \
  -DCMAKE_CXX_COMPILER="$CXX" \
  -DAX_SDK_PATH="$(pwd)/sdk" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -G "$CMAKE_GENERATOR"

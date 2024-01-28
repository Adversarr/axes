# TODO: Implement the build script.

cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=~/.local/utils/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang

cmake --build build -j $(nproc)

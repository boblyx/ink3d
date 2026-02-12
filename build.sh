#!/bin/bash
start_time=$(date +%s%N)
conan install . --output-folder=build -c tools.build:jobs=8 --build=missing
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build .
end_time=$(date +%s%N)
duration=$(( (end_time - start_time) / 1000000 ))

echo "-------------------------------"
echo "Execution Time: ${duration} ms"
echo "-------------------------------"
mkdir build
cd build
cmake -DCMAKE_BUID_TYPE=${BUILD_TYPE} ..
make -j4
make test

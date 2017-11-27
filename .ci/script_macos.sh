#!/usr/bin/env bash -e

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..
make -j4 tests
make test

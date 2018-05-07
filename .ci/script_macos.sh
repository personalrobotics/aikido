#!/usr/bin/env bash

set -ex

unset -f cd; # Disable rvm ovveride of cd (see https://github.com/travis-ci/travis-ci/issues/8703)

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..
make -j4 tests
make test

#!/usr/bin/env bash

set -ex

unset -f cd; # Disable rvm override of cd (see https://github.com/travis-ci/travis-ci/issues/8703)

mkdir build
cd build

if [ $BUILD_NAME = DOCS ]; then
  . "${GITHUB_WORKSPACE}/.ci/build_docs.sh"
  exit 0
fi

if [ $BUILD_AIKIDOPY = ON ]; then
  cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DTREAT_WARNINGS_AS_ERRORS=ON -DBUILD_AIKIDOPY=ON ..
  make -j4 aikidopy
  make pytest
else
  cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DTREAT_WARNINGS_AS_ERRORS=ON ..
  make -j4 tests
  make test
fi
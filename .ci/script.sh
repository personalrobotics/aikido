#!/usr/bin/env bash

set -ex

if [ "${USE_CATKIN}" = "ON" ]; then
  . "${TRAVIS_BUILD_DIR}/.ci/script_catkin.sh"
else
  . "${TRAVIS_BUILD_DIR}/.ci/script_cmake.sh"
fi

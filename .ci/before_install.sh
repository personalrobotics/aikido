#!/usr/bin/env bash

set -ex

if [ "${USE_CATKIN}" = "ON" ]; then
  . "${TRAVIS_BUILD_DIR}/.ci/before_install_catkin.sh";
fi

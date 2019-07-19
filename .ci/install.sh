#!/usr/bin/env bash

set -ex

if [ "${TRAVIS_OS_NAME}" = "linux" ]; then
  if [ "${USE_CATKIN}" = "ON" ]; then
    . "${TRAVIS_BUILD_DIR}/.ci/install_linux_catkin.sh"
  else
    . "${TRAVIS_BUILD_DIR}/.ci/install_linux_cmake.sh"
  fi
elif [ "${TRAVIS_OS_NAME}" = "osx"   ]; then
  . "${TRAVIS_BUILD_DIR}/.ci/install_macos.sh"
fi

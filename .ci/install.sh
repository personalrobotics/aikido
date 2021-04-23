#!/usr/bin/env bash

set -ex

if [ "${OS_NAME}" = "macos-latest" ]; then
  . "${GITHUB_WORKSPACE}/.ci/install_macos.sh"
else
  if [ "${USE_CATKIN}" = "ON" ]; then
    . "${GITHUB_WORKSPACE}/.ci/install_linux_catkin.sh"
  else
    . "${GITHUB_WORKSPACE}/.ci/install_linux_cmake.sh"
  fi
fi

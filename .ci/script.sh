#!/usr/bin/env bash

set -ex

if [ "${USE_CATKIN}" = "ON" ]; then
  . "${GITHUB_WORKSPACE}/.ci/script_catkin.sh"
else
  . "${GITHUB_WORKSPACE}/.ci/script_cmake.sh"
fi

#!/usr/bin/env bash

set -ex

if [ "${USE_CATKIN}" = "ON" ]; then
  . "${GITHUB_WORKSPACE}/.ci/before_install_catkin.sh";
fi

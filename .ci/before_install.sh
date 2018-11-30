#!/usr/bin/env bash

set -ex

if [ "${OS_NAME}" = "linux" ]; then
  . "${BUILD_DIR}/.ci/before_install_linux.sh"; 
fi

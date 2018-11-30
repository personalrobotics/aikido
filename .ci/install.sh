#!/usr/bin/env bash

set -ex

if [ "${OS_NAME}" = "linux" ]; then
  . "${BUILD_DIR}/.ci/install_linux.sh"
elif [ "${OS_NAME}" = "osx"   ]; then
  . "${BUILD_DIR}/.ci/install_macos.sh"
fi

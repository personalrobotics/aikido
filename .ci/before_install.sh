#!/usr/bin/env bash

set -ex

if [ "${TRAVIS_OS_NAME}" = "linux" ]; then
  . "${TRAVIS_BUILD_DIR}/.ci/before_install_linux.sh"; 
fi

#!/usr/bin/env bash

set -ex

if [ "${TRAVIS_OS_NAME}" = "linux" ]; then
  . "${TRAVIS_BUILD_DIR}/.ci/script_linux.sh"
elif [ "${TRAVIS_OS_NAME}" = "osx"   ]; then
  . "${TRAVIS_BUILD_DIR}/.ci/script_macos.sh"
fi

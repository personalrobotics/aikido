#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"
cp -r "${TRAVIS_BUILD_DIR}" src
./scripts/internal-distro.py --workspace=src distribution.yml --repository "${REPOSITORY}" ${REQUIRED_ONLY}

if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  sudo apt-get install -y clang-format-3.8
fi

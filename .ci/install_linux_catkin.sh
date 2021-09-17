#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"
cp -r "${GITHUB_WORKSPACE}" src
./scripts/internal-distro.py --workspace=src distribution.yml --repository "${REPOSITORY}" ${REQUIRED_ONLY}

# Catkin on Focal-only
$SUDO apt-get install -y clang-format-10

if [ "$BUILD_AIKIDOPY" = "ON" ]; then
  $SUDO apt-get -y install python3-dev python3-numpy
  $SUDO apt-get -y install python3-pip -y
  $SUDO pip3 install pytest -U

  $SUDO apt-get -y install pybind11-dev python3 libpython3-dev python3-pytest \
      python3-distutils
fi

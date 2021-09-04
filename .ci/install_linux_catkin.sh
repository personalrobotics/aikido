#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"
cp -r "${GITHUB_WORKSPACE}" src
./scripts/internal-distro.py --workspace=src distribution.yml --repository "${REPOSITORY}" ${REQUIRED_ONLY}

if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  sudo apt-get install -y clang-format-6.0
fi

if [ "$BUILD_AIKIDOPY" = "ON" ]; then
  $SUDO apt-get -y install python3-dev python3-numpy
  $SUDO apt-get -y install python3-pip -y
  $SUDO pip3 install pytest -U

  if [ $(lsb_release -sc) = "trusty" ] || [ $(lsb_release -sc) = "xenial" ] || [ $(lsb_release -sc) = "bionic" ]; then
    git clone https://github.com/pybind/pybind11 -b 'v2.3.0' --single-branch --depth 1
    cd pybind11
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF
    make -j4
    $SUDO make install
    cd ../..
  elif [ $(lsb_release -sc) = "focal" ]; then
    $SUDO apt-get -y install pybind11-dev python3 libpython3-dev python3-pytest \
      python3-distutils
  else
    echo -e "$(lsb_release -sc) is not supported."
    exit 1
  fi
fi

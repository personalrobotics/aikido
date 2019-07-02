#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"
cp -r "${TRAVIS_BUILD_DIR}" src
./scripts/internal-distro.py --workspace=src distribution.yml --repository "${REPOSITORY}" ${REQUIRED_ONLY}

if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  sudo apt-get install -y clang-format-3.8
fi

if [ $BUILD_AIKIDOPY = ON ]; then
  sudo apt-get -y install python3-dev python3-numpy
  sudo apt-get -y install python3-pip -y
  sudo pip3 install pytest -U

  if [ $(lsb_release -sc) = "xenial" ]; then
    git clone https://github.com/pybind/pybind11 -b 'v2.2.4' --single-branch --depth 1
    cd pybind11
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF
    make -j4
    sudo make install
    cd ../..
  elif [ $(lsb_release -sc) = "bionic" ]; then
    git clone https://github.com/pybind/pybind11 -b 'v2.2.4' --single-branch --depth 1
    cd pybind11
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF
    make -j4
    sudo make install
    cd ../..
  elif [ $(lsb_release -sc) = "cosmic" ]; then
    sudo apt-get -y install pybind11-dev python3 libpython3-dev python3-pytest \
      python3-distutils
  elif [ $(lsb_release -sc) = "disco" ]; then
    sudo apt-get -y install pybind11-dev python3 libpython3-dev python3-pytest \
      python3-distutils
  else
    echo -e "$(lsb_release -sc) is not supported."
    exit 1
  fi
fi

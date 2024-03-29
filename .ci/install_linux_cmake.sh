#!/usr/bin/env bash

set -ex

$SUDO apt-get -qq update
$SUDO apt-get -y install lsb-release software-properties-common

$SUDO apt-get -qq update

# Build tools
$SUDO apt-get -y install \
  sudo \
  build-essential \
  cmake \
  pkg-config \
  curl \
  git
if [ $COMPILER = clang ]; then
  $SUDO apt-get -qq -y install clang
fi

# Required dependencies
$SUDO apt-get -y install \
  libboost-filesystem-dev \
  libdart-all-dev \
  libdart-external-ikfast-dev \
  libdart-external-odelcpsolver-dev \
  libompl-dev





# Optional dependencies
$SUDO apt-get -y install \
  libtinyxml2-dev \
  libyaml-cpp-dev

if [ "$BUILD_AIKIDOPY" = "ON" ]; then
  $SUDO apt-get -y install python3-dev python3-numpy
  $SUDO apt-get -y install python3-pip -y
  $SUDO pip3 install pytest -U

  if [ $(lsb_release -sc) = "bionic" ]; then
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

if [ $BUILD_NAME = DOCS ]; then
  $SUDO apt-get -qq -y install doxygen
fi

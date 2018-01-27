#!/usr/bin/env bash

set -e

brew update > /dev/null

brew install dartsim/dart/dartsim6 --without-ipopt
brew install ompl --with-eigen
brew install tinyxml2
brew install yaml-cpp

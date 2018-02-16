#!/usr/bin/env bash

set -e

brew update > /dev/null

brew install dartsim
brew install ompl --with-eigen
brew install tinyxml2
brew install yaml-cpp

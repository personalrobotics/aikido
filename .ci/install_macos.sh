#!/usr/bin/env bash

set -ex

brew update > /dev/null
brew bundle || true
brew install --HEAD valgrind

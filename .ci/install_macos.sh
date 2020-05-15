#!/usr/bin/env bash

set -ex

brew update > /dev/null
brew bundle || true
brew install --HEAD https://raw.githubusercontent.com/sowson/valgrind/master/valgrind.rb

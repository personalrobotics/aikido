#!/usr/bin/env bash

set -ex

/usr/bin/yes | pip2 uninstall numpy
brew upgrade python

brew update > /dev/null
brew bundle || true

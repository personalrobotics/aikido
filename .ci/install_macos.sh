#!/usr/bin/env bash

set -ex

/usr/bin/yes | pip2 uninstall numpy  # see https://github.com/travis-ci/travis-ci/issues/6688

brew upgrade > /dev/null
brew bundle || true

#!/usr/bin/env bash

set -ex

/usr/bin/yes | pip2 uninstall numpy  # see https://github.com/travis-ci/travis-ci/issues/6688

# sudo npm uninstall npm -g
# sudo rm -rf /usr/local/lib/node_modules/npm
# brew link node

# brew upgrade > /dev/null
brew bundle || true

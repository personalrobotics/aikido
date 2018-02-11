#!/usr/bin/env bash

set -e

# Build documentation
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --make-args docs -- aikido > /dev/null

# Organize into "master" subdirectory
mkdir -p "${TRAVIS_BUILD_DIR}/gh-pages"
mv "${HOME}/workspace/build/aikido/doxygen" "${TRAVIS_BUILD_DIR}/gh-pages/master"

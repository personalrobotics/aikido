#!/usr/bin/env bash

set -ev

# Build documentation
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --make-args docs -- aikido > /dev/null

# Organize into "master" subdirectory
mkdir -p "${HOME}/gh-pages"
mv "${HOME}/workspace/build/aikido/doxygen" "${HOME}/gh-pages/master"

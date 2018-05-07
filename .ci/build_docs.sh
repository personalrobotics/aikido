#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

# Build documentation
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DDOWNLOAD_TAGFILES=$DOWNLOAD_TAGFILES --make-args docs -- aikido > /dev/null

# Organize into "master" subdirectory
mkdir -p "${TRAVIS_BUILD_DIR}/gh-pages"
mv "${HOME}/workspace/build/aikido/doxygen" "${TRAVIS_BUILD_DIR}/gh-pages/master"

# Generate hard-coded list of API versions
cat <<EOF > "${TRAVIS_BUILD_DIR}/gh-pages/README.md"
## API Documentation

* [master](https://personalrobotics.github.io/aikido/master/)
EOF

#!/usr/bin/env bash

set -ex

# Install test fixture dependencies.
mkdir -p "${HOME}/workspace/src"
cd "${HOME}/workspace"
git clone https://github.com/personalrobotics/pr-cleanroom.git -b focal_support_test scripts
curl -sS "${DISTRIBUTION}" > distribution.yml
./scripts/internal-setup.sh

#!/usr/bin/env bash

set -ex

# Install test fixture dependencies.
mkdir -p "${HOME}/workspace/src"
cd "${HOME}/workspace"
git clone https://github.com/personalrobotics/pr-cleanroom.git scripts
curl -sS "${DISTRIBUTION}" > distribution.yml
./scripts/internal-setup.sh

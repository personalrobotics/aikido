#!/usr/bin/env bash

set -e

# Install test fixture dependencies.
mkdir -p "${HOME}/workspace/src"
cd "${HOME}/workspace"
git clone https://github.com/personalrobotics/pr-cleanroom.git scripts
curl -sS "${DISTRIBUTION}" > distribution.yml
./scripts/internal-setup.sh
export PACKAGE_NAMES="$(./scripts/internal-get-packages.py distribution.yml ${REPOSITORY})"

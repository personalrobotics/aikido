#!/usr/bin/env bash

set -e

if [ ${TRAVIS} = "true" ]; then
  AIKIDO_BUILD_DIR="${TRAVIS_BUILD_DIR}"
elif [ ${APPVEYOR} = "True" ]; then
  AIKIDO_BUILD_DIR="${APPVEYOR_BUILD_FOLDER}"
fi

cp -r "${AIKIDO_BUILD_DIR}" src
./scripts/internal-distro.py --workspace=src distribution.yml --repository "${REPOSITORY}"

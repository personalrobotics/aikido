#!/usr/bin/env bash

set -e

if [ ${TRAVIS} ]; then
  AIKIDO_BUILD_DIR="${TRAVIS_BUILD_DIR}"
elif [ ${APPVEYOR} ]; then
  AIKIDO_BUILD_DIR="${APPVEYOR_BUILD_FOLDER}"
fi

# For test
echo ${TRAVIS_BUILD_DIR}
echo ${APPVEYOR_BUILD_FOLDER}
echo ${AIKIDO_BUILD_DIR}

cp -r "${AIKIDO_BUILD_DIR}" src
./scripts/internal-distro.py --workspace=src distribution.yml --repository "${REPOSITORY}"

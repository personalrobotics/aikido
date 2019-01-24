#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

if [ "${TRAVIS_OS_NAME}" = "linux" ]; then
  cat ./build/aikido/Testing/Temporary/LastTest.log
  cat ./build/aikido/Testing/Temporary/LastTestsFailed.log
elif [ "${TRAVIS_OS_NAME}" = "osx"   ]; then
  cat Testing/Temporary/LastTest.log
  cat Testing/Temporary/LastTestsFailed.log
fi

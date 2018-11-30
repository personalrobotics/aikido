#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

if [ "${OS_NAME}" = "linux" ]; then
  cat ./build/aikido/Testing/Temporary/LastTest.log
  cat ./build/aikido/Testing/Temporary/LastTestsFailed.log
elif [ "${OS_NAME}" = "osx"   ]; then
  cat Testing/Temporary/LastTest.log
  cat Testing/Temporary/LastTestsFailed.log
fi

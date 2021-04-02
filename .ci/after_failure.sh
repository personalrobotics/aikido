#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

if [ "${OS_NAME}" = "macos-latest" ]; then
  cat Testing/Temporary/LastTest.log
  cat Testing/Temporary/LastTestsFailed.log
else
  cat ./build/aikido/Testing/Temporary/LastTest.log
  cat ./build/aikido/Testing/Temporary/LastTestsFailed.log
fi

#!/usr/bin/env bash

set -ex

if [ "${USE_CATKIN}" = "ON" ]; then
  cd "${HOME}/workspace/build/aikido/"
fi

cat Testing/Temporary/LastTest.log
cat Testing/Temporary/LastTestsFailed.log

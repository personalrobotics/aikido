#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

if [ "${TRAVIS_OS_NAME}" = "linux" ]; then
  ./scripts/view-all-results.sh test_results
fi

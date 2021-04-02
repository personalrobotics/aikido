#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

if [ "${OS_NAME}" != "macos-latest" ]; then
  ./scripts/view-all-results.sh test_results
fi

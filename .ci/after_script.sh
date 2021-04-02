#!/usr/bin/env bash

set -ex
if [ "${USE_CATKIN}" = "ON" ]; then
	cd "${HOME}/workspace"

	if [ "${OS_NAME}" != "macos-latest" ]; then
	  ./scripts/view-all-results.sh test_results
	fi
fi

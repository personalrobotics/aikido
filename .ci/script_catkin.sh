#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

export PACKAGE_NAMES="$(./scripts/internal-get-packages.py distribution.yml ${REPOSITORY})"
./scripts/internal-build.sh ${PACKAGE_NAMES}

# Check code style (only one job needs to do this)
if [ $BUILD_NAME = FOCAL_CATKIN_DEBUG_CODECOV ]; then
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --make-args check-format -- aikido
fi

# Build aikidopy and pytest
if [ $BUILD_AIKIDOPY = ON ]; then
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DTREAT_WARNINGS_AS_ERRORS=OFF -DCODECOV=OFF --make-args aikidopy -- aikido
  $SUDO ./scripts/internal-run.sh make -C build/aikido pytest
  exit 0
fi

# Manually build Aikido's tests; they are not built automatically because it is not a Catkin package.
if [ $BUILD_NAME = FOCAL_CATKIN_DEBUG_CODECOV ]; then
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DTREAT_WARNINGS_AS_ERRORS=ON -DCODECOV=ON --make-args tests -- aikido
else
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DTREAT_WARNINGS_AS_ERRORS=ON -DCODECOV=OFF --make-args tests -- aikido
fi

# Run tests and measure test coverage if CodeCov is on.
if [ $BUILD_NAME = FOCAL_CATKIN_DEBUG_CODECOV ]; then
  ./scripts/internal-run.sh make -C build/aikido aikido_coverage
else
  ./scripts/internal-run.sh env CTEST_OUTPUT_ON_FAILURE=true make -C build/aikido test
fi

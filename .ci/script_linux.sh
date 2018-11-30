#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

export PACKAGE_NAMES="$(./scripts/internal-get-packages.py distribution.yml ${REPOSITORY})"
if [ $(lsb_release -sc) = "xenial" ]; then
  #. /opt/ros/kinetic/setup.bash
  ./scripts/internal-build.sh ${PACKAGE_NAMES} --cmake-args -DCMAKE_PREFIX_PATH=/usr
  #./scripts/internal-build.sh ${PACKAGE_NAMES}
else
  ./scripts/internal-build.sh ${PACKAGE_NAMES}
fi

if [ $BUILD_NAME = DOCS ]; then
  . "${TRAVIS_BUILD_DIR}/.ci/build_docs.sh"
  exit 0
fi

# Check code style
if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --make-args check-format -- aikido
fi

# Manually build Aikido's tests; they are not built automatically because it is not a Catkin package.
if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DTREAT_WARNINGS_AS_ERRORS=ON -DCODECOV=ON --make-args tests -- aikido
else
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DTREAT_WARNINGS_AS_ERRORS=ON -DCODECOV=OFF --make-args tests -- aikido
fi

# Run tests and measure test coverage if CodeCov is on.
if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  ./scripts/internal-run.sh make -C build/aikido aikido_coverage
else
  ./scripts/internal-run.sh env CTEST_OUTPUT_ON_FAILURE=true make -C build/aikido test
fi

# Uploading report to CodeCov
if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  bash <(curl -s https://codecov.io/bash) || echo "Codecov did not collect coverage reports"
fi

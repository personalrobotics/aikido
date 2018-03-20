#!/usr/bin/env bash

set -e

./scripts/internal-build.sh ${PACKAGE_NAMES}
./scripts/internal-test.sh ${PACKAGE_NAMES}

# Check code style
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --make-args check-format -- aikido

# Manually build Aikido's tests; they are not built automatically because it is not a Catkin package.
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DTREAT_WARNINGS_AS_ERRORS=ON -DCODECOV=$CODECOV --make-args tests -- aikido

# Run tests and measure test coverage if CodeCov is on.
if [ $CODECOV = ON ]; then ./scripts/internal-run.sh make -C build/aikido aikido_coverage; else ./scripts/internal-run.sh CTEST_OUTPUT_ON_FAILURE=true make -C build/aikido test; fi

# Uploading report to CodeCov
if [ $CODECOV = ON ]; then bash <(curl -s https://codecov.io/bash) || echo "Codecov did not collect coverage reports"; fi

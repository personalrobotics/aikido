./scripts/internal-build.sh ${PACKAGE_NAMES}
./scripts/internal-test.sh ${PACKAGE_NAMES}

# Check code style
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --make-args check-format -- aikido

# Manually build Aikido's tests; they are not built automatically because it is not a Catkin package.
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DTREAT_WARNINGS_AS_ERRORS=ON -DCOVERALLS=$COVERALLS --make-args tests -- aikido

# Run tests and measure test coverage if COVERALLS is on.
if [ $COVERALLS = ON ]; then . "/scripts/internal-run.sh make -C build/aikido coveralls"; else . "/scripts/internal-run.sh make -C build/aikido test"; fi


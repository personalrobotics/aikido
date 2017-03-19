./scripts/internal-build.sh ${PACKAGE_NAMES}
./scripts/internal-test.sh ${PACKAGE_NAMES}

# Manually run Aikido's tests; they are not run automatically because it is not a Catkin package.
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --make-args tests test -- aikido

# Rebuild Aikido in debug mode with coveralls enabled to measure test coverage.
./scripts/internal-run.sh catkin clean -y aikido
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCOVERALLS=ON -- --make-args all tests -- aikido
./scripts/internal-run.sh make -C build/aikido coveralls

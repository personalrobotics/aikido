#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

AIKIDO_DIR="${HOME}/workspace/src/aikido"
BUILD_DIR="${HOME}/workspace/build/aikido/doxygen"

# For branch builds, Travis only clones that branch with a fixed depth of 50
# commits. This means that the clone knows nothing about other Git branches or
# tags. We fix this by deleting and re-cloning the full repository.
rm -rf ${AIKIDO_DIR}
git clone "https://github.com/${TRAVIS_REPO_SLUG}.git" ${AIKIDO_DIR}

# Organize into "gh-pages" directory
mkdir -p ${HOME}/gh-pages

# Initialize list of API versions
cat <<EOF > ${HOME}/gh-pages/README.md
## API Documentation

EOF

while read version; do
  # Add entry to list of API versions
  echo "* [${version}](https://personalrobotics.github.io/aikido/${version}/)" >> ${HOME}/gh-pages/README.md

  # Build documentation
  git -C ${AIKIDO_DIR} checkout ${version}
  rm -rf ${BUILD_DIR}
  ./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DDOWNLOAD_TAGFILES=ON --make-args docs -- aikido > /dev/null

  mv ${BUILD_DIR} ${HOME}/gh-pages/${version}
done < ${TRAVIS_BUILD_DIR}/.ci/docs_versions.txt

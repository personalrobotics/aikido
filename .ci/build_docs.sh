#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

AIKIDO_DIR="${HOME}/workspace/src/aikido"

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

cd ${AIKIDO_DIR}
mkdir build

while read version; do
  # Add entry to list of API versions
  echo "* [${version}](https://personalrobotics.github.io/aikido/${version}/)" >> ${HOME}/gh-pages/README.md

  # Build documentation
  git checkout ${version}
  pushd build
  rm -rf *
  ./scripts/internal-run.sh cmake -DDOWNLOAD_TAGFILES=ON ..
  make docs
  mv doxygen ${HOME}/gh-pages/${version}
  popd
done < ${TRAVIS_BUILD_DIR}/.ci/docs_versions.txt

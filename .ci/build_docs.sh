#!/usr/bin/env bash

set -ex

# Organize into "gh-pages" directory
mkdir -p ${GITHUB_WORKSPACE}/gh-pages

# Initialize list of API versions
cat <<EOF > ${GITHUB_WORKSPACE}/gh-pages/README.md
## API Documentation

EOF

mkdir build_docs
cd build_docs

while read version; do
  # Add entry to list of API versions
  echo "* [${version}](https://personalrobotics.github.io/aikido/${version}/)" >> ${GITHUB_WORKSPACE}/gh-pages/README.md

  # Build documentation
  git checkout ${version}
  rm -rf *
  cmake -DDOWNLOAD_TAGFILES=ON ..
  make docs
  mv doxygen ${GITHUB_WORKSPACE}/gh-pages/${version}
done < ${GITHUB_WORKSPACE}/.ci/docs_versions.txt

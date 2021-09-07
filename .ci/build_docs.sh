#!/usr/bin/env bash

set -ex

AIKIDO_DIR="${HOME}/workspace/src/aikido"

# Github Actions doesn't do a full clone of the repository.
# We do it ourselves in a separate directory
# To access all branches and tags.
mkdir -p ${AIKIDO_DIR}
git clone "https://github.com/${GITHUB_REPOSITORY}.git" ${AIKIDO_DIR}

# Organize into "gh-pages" directory
mkdir -p ${GITHUB_WORKSPACE}/gh-pages

# Initialize list of API versions
cat <<EOF > ${GITHUB_WORKSPACE}/gh-pages/index.md
---
title: API Documentation
---

## API Documentation

EOF

mkdir build_docs
cd build_docs

while read version; do
  # Add entry to list of API versions
  echo "* [${version}](https://personalrobotics.github.io/aikido/${version}/)" >> ${GITHUB_WORKSPACE}/gh-pages/index.md

  # Build documentation
  git -C ${AIKIDO_DIR} checkout ${version}
  rm -rf *
  cmake -DDOWNLOAD_TAGFILES=ON ${AIKIDO_DIR}
  make docs
  mv doxygen ${GITHUB_WORKSPACE}/gh-pages/${version}
done < ${GITHUB_WORKSPACE}/.ci/docs_versions.txt

#!/usr/bin/env bash

set -ex

# Install OMPL package from PRL PPA
if [ $(lsb_release -sc) = "xenial" ]; then
  sudo apt-add-repository -y ppa:personalrobotics/ppa
  sudo apt update
  sudo apt install -y libompl-dev # OMPL (>= 1.2.1)
fi

# Install test fixture dependencies.
mkdir -p "${HOME}/workspace/src"
cd "${HOME}/workspace"
git clone https://github.com/personalrobotics/pr-cleanroom.git scripts
curl -sS "${DISTRIBUTION}" > distribution.yml
./scripts/internal-setup.sh

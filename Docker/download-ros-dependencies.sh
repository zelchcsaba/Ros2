#!/usr/bin/env bash
set -euo pipefail

rosdep update
sudo apt-get update
sudo apt upgrade -y
rosdep install --from-paths . --ignore-src --rosdistro humble -y

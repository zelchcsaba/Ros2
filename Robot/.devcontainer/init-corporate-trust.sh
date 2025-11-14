#!/usr/bin/env bash
set -euo pipefail

CA_SRC="/tmp/certs"
DEBIAN_DST_DIR="/usr/local/share/ca-certificates"

if [ -d $CA_SRC ]; then
    sudo mkdir -p $DEBIAN_DST_DIR
    sudo cp -r $CA_SRC/* $DEBIAN_DST_DIR
    sudo update-ca-certificates
fi

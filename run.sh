#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
SYMLINK_DIR=$(dirname $(readlink $0))
cd $SCRIPT_DIR
cd $SYMLINK_DIR/pviewer_electron
cordova run electron --release --nobuild

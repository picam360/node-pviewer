#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
SYMLINK_DIR=$(dirname $(readlink $0))
cd $SCRIPT_DIR
if [ -n "$SYMLINK_DIR" ]; then
    cd $SYMLINK_DIR
fi
cd pviewer_electron
npx electron platforms/electron/www "$@"

#!/bin/bash
#
#usage : source setup_nodebrew.sh
#

#node-wrtc supports up to node v14

if hash volta 2>/dev/null; then
    echo "volta is already installed."
else
    sudo apt-get -y install curl

    curl https://get.volta.sh | bash
    export PATH=$HOME/.volta/bin:$PATH
    volta install node@16
    volta intall npm
fi
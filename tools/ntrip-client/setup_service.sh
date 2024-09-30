#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
CONFIG_PATH=$1
NODE_PATH=$(which node)

sed -e "s%@CONFIG_PATH@%${CONFIG_PATH}%" \
    -e "s%@NODE_PATH@%${NODE_PATH}%" \
    -e "s%@SCRIPT_DIR@%${SCRIPT_DIR}%" \
    pserver-ntrip-client.in | sudo tee /usr/bin/pserver-ntrip-client
sudo chmod +x /usr/bin/pserver-ntrip-client

sed -e "s%@SCRIPT_DIR@%${SCRIPT_DIR}%" \
    pserver-ntrip-client.service.in | sudo tee /etc/systemd/system/pserver-ntrip-client.service

sudo systemctl daemon-reload
sudo systemctl restart pserver-ntrip-client.service
sudo systemctl enable pserver-ntrip-client.service
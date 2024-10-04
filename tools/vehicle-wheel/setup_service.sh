#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
CONFIG_PATH=$1
NODE_PATH=$(which node)

sed -e "s%@CONFIG_PATH@%${CONFIG_PATH}%" \
    -e "s%@NODE_PATH@%${NODE_PATH}%" \
    -e "s%@SCRIPT_DIR@%${SCRIPT_DIR}%" \
    pserver-vehicle-wheel.in | sudo tee /usr/bin/pserver-vehicle-wheel
sudo chmod +x /usr/bin/pserver-vehicle-wheel

sed -e "s%@SCRIPT_DIR@%${SCRIPT_DIR}%" \
    pserver-vehicle-wheel.service.in | sudo tee /etc/systemd/system/pserver-vehicle-wheel.service

sudo systemctl daemon-reload
sudo systemctl restart pserver-vehicle-wheel.service
sudo systemctl enable pserver-vehicle-wheel.service
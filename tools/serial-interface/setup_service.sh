#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
CONFIG_PATH=$1
NODE_PATH=$(which node)

sed -e "s%@CONFIG_PATH@%${CONFIG_PATH}%" \
    -e "s%@NODE_PATH@%${NODE_PATH}%" \
    -e "s%@SCRIPT_DIR@%${SCRIPT_DIR}%" \
    pserver-serial-interface.in | sudo tee /usr/bin/pserver-serial-interface
sudo chmod +x /usr/bin/pserver-serial-interface

sed -e "s%@SCRIPT_DIR@%${SCRIPT_DIR}%" \
    pserver-serial-interface.service.in | sudo tee /etc/systemd/system/pserver-serial-interface.service

sudo systemctl daemon-reload
sudo systemctl restart pserver-serial-interface.service
sudo systemctl enable pserver-serial-interface.service
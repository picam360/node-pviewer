#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
CONFIG_PATH=$1
PYTHON_PATH=$(which python3)
SERVICE_NAME=litime-bms

sed -e "s%@CONFIG_PATH@%${CONFIG_PATH}%" \
    -e "s%@PYTHON_PATH@%${PYTHON_PATH}%" \
    -e "s%@SCRIPT_DIR@%${SCRIPT_DIR}%" \
    -e "s%@SERVICE_NAME@%${SERVICE_NAME}%" \
    ${SERVICE_NAME}.in | sudo tee /usr/bin/${SERVICE_NAME}
sudo chmod +x /usr/bin/${SERVICE_NAME}

sed -e "s%@SCRIPT_DIR@%${SCRIPT_DIR}%" \
    -e "s%@SERVICE_NAME@%${SERVICE_NAME}%" \
    ${SERVICE_NAME}.service.in | sudo tee /etc/systemd/system/${SERVICE_NAME}.service

sudo systemctl daemon-reload
sudo systemctl restart ${SERVICE_NAME}.service
sudo systemctl enable ${SERVICE_NAME}.service
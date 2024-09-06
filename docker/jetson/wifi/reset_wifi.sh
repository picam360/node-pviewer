#!/bin/bash

sudo rm /etc/NetworkManager/system-connections/* 2>/dev/null
sudo systemctl restart NetworkManager
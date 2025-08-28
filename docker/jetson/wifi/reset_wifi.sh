#!/bin/bash

sudo systemctl stop dnsmasq
sudo systemctl disable dnsmasq
sudo systemctl stop hostapd
sudo systemctl disable hostapd

sudo rm /etc/NetworkManager/system-connections/* 2>/dev/null
sudo systemctl restart NetworkManager
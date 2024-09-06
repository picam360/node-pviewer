#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
cd "$SCRIPT_DIR" || exit

if [ "$1" = "" ]; then
    echo no address
    exit
fi

ADDRESS=$1

if [ "$2" = "" ]; then
    SSID=picam360
else
    SSID=$2
fi


if [ "$3" = "" ]; then
    PWD=picam360
else
    PWD=$3
fi

echo "SSID=$SSID"
echo "PWD=$PWD"
echo "ADDRESS=10.42.0.$ADDRESS"

MACADDRESS=$(cat /sys/class/net/wlan0/address)

. reset_wifi.sh
sleep 1

sudo nmcli device wifi hotspot ifname wlan0 con-name 'hotspot' ssid 'picam360' band 'bg' password 'picam360'
sudo cp ifcfg-hotspot /etc/NetworkManager/system-connections/hotspot
sudo sed -i "s/%SSID%/$SSID/g" /etc/NetworkManager/system-connections/hotspot
sudo sed -i "s/%PWD%/$PWD/g" /etc/NetworkManager/system-connections/hotspot
sudo sed -i "s/%ADDRESS%/$ADDRESS/g" /etc/NetworkManager/system-connections/hotspot
sudo sed -i "s/%MACADDRESS%/$MACADDRESS/g" /etc/NetworkManager/system-connections/hotspot
sudo chmod 600 /etc/NetworkManager/system-connections/hotspot
sudo nmcli connection load /etc/NetworkManager/system-connections/hotspot

sudo systemctl restart NetworkManager
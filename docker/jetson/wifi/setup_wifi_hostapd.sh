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

sudo systemctl stop NetworkManager
sudo systemctl disable NetworkManager

sudo cp hostapd.conf.tmp /etc/hostapd/hostapd.conf
sudo sed -i "s/%SSID%/$SSID/g" /etc/hostapd/hostapd.conf
sudo sed -i "s/%PWD%/$PWD/g" /etc/hostapd/hostapd.conf
#sudo sed -i "s/%ADDRESS%/$ADDRESS/g" /etc/hostapd/hostapd.conf
#sudo sed -i "s/%MACADDRESS%/$MACADDRESS/g" /etc/hostapd/hostapd.conf
sudo chmod 600 /etc/hostapd/hostapd.conf

sudo systemctl restart hostapd
sudo ip addr add 10.42.0.%ADDRESS%/24 dev wlan0
sudo ip link set wlan0 up
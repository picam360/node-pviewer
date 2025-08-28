#!/bin/bash

#sudo apt install dnsmasq
#sudo apt install hostapd

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

sudo cp dnsmasq.conf.tmp /etc/dnsmasq.conf
sudo sed -i "s/%ADDRESS%/$ADDRESS/g" /etc/dnsmasq.conf
sudo chmod 644 /etc/dnsmasq.conf

sudo cp hostapd.conf.tmp /etc/hostapd/hostapd.conf
sudo sed -i "s/%SSID%/$SSID/g" /etc/hostapd/hostapd.conf
sudo sed -i "s/%PWD%/$PWD/g" /etc/hostapd/hostapd.conf
sudo chmod 600 /etc/hostapd/hostapd.conf

sudo ip addr add 10.42.0.$ADDRESS/24 dev wlan0
sudo ip link set wlan0 up

sudo systemctl enable dnsmasq
sudo systemctl restart dnsmasq
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl restart hostapd
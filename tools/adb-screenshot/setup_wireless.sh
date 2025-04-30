#!/bin/bash

# Check if adb command is available
if ! command -v adb &> /dev/null; then
    echo "adb command not found. Please install Android Platform Tools."
    exit 1
fi

# Check if a device is connected via USB
adb devices | grep -w "device" > /dev/null
if [ $? -ne 0 ]; then
    echo "No device connected via USB."
    exit 1
fi

echo "Device detected. Retrieving IP address..."

# Get the device's IP address
IP=$(adb shell ip route | awk '{print $9}' | tr -d '\r')

if [ -z "$IP" ]; then
    echo "Failed to get IP address."
    exit 1
fi

echo "Device IP address: $IP"

# Switch the device to TCP/IP mode
adb tcpip 5555
sleep 1  # Wait briefly to ensure TCP mode is active

# Connect to the device over network
adb connect ${IP}:5555

echo "Connection complete!"
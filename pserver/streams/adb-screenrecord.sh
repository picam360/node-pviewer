#!/bin/bash

# -----------------------
# Initialization
# -----------------------
USB_PORT=""
BITRATE=""
SIZE=""
ADB_PORT=5555

# -----------------------
# Parse arguments
# -----------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --usb-port)
            USB_PORT="$2"
            shift 2
            ;;
        --bit-rate)
            BITRATE="$2"
            shift 2
            ;;
        --size)
            SIZE="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1" >&2
            echo "Usage: $0 --usb-port <BUS:DEVICE> [--bit-rate BPS] [--size WIDTHxHEIGHT]" >&2
            exit 1
            ;;
    esac
done

if [ -z "$USB_PORT" ]; then
    SERIAL=""
else
    REDIS_KEY="adb-usb-port['$USB_PORT']"

    # -----------------------
    # Get serial number from USB port
    # -----------------------
    SERIAL=$(lsusb -v -s "$USB_PORT" 2>/dev/null | grep -E "iSerial|Serial Number" | awk '{print $3}')

    if [ -z "$SERIAL" ]; then
        echo "No serial number found for port $USB_PORT" >&2

        # Retrieve value from Redis
        ADB_IP_PORT=$(redis-cli GET "$REDIS_KEY")

        if [ -z "$ADB_IP_PORT" ]; then
            echo "❌ Failed to retrieve value from Redis (key: $REDIS_KEY)" >&2
            exit 1
        else
            if adb devices | grep -q "^$ADB_IP_PORT[[:space:]]*device"; then
                echo "✅ $ADB_IP_PORT is connected via ADB" >&2
                SERIAL=$ADB_IP_PORT
            else
                redis-cli DEL "$REDIS_KEY"
                echo "❌ $ADB_IP_PORT is not connected via ADB" >&2
                exit 1
            fi
        fi

    else
        echo "ADB serial: $SERIAL" >&2

        # --- Get device IP address (Wi-Fi interface) ---
        IP=$(adb -s "$SERIAL" shell ip -f inet addr show wlan0 | grep inet | awk '{print $2}' | cut -d/ -f1)
        if [ -z "$IP" ]; then
            echo "Error: Failed to retrieve IP address" >&2
            exit 1
        fi
        echo "✅ Device IP = $IP" >&2

        # --- Switch to TCP/IP mode ---
        adb -s "$SERIAL" tcpip "$ADB_PORT"
        sleep 1

        # --- Connect via Wi-Fi ---
        adb connect "$IP:$ADB_PORT"
        if [ $? -ne 0 ]; then
            echo "Error: adb connect failed" >&2
            exit 1
        fi
        echo "✅ ADB Wi-Fi connection succeeded: $IP:$ADB_PORT" >&2

        # --- Save connection info to Redis ---
        REDIS_VALUE="$IP:$ADB_PORT"
        redis-cli SET "$REDIS_KEY" "$REDIS_VALUE"

        if [ $? -eq 0 ]; then
            echo "✅ Saved to Redis: $REDIS_KEY → $REDIS_VALUE" >&2
        else
            echo "Error: Failed to save to Redis" >&2
            exit 1
        fi
    fi
fi

# -----------------------
# Build screenrecord options
# -----------------------
CMD_OPTS="--output-format=h264"

if [ -n "$BITRATE" ]; then
    CMD_OPTS="$CMD_OPTS --bit-rate $BITRATE"
fi

if [ -n "$SIZE" ]; then
    CMD_OPTS="$CMD_OPTS --size $SIZE"
fi

SERIAL_OPTION=""
if [ -n "$SERIAL" ]; then
    SERIAL_OPTION="-s $SERIAL"
fi

# -----------------------
# Execute screenrecord
# -----------------------
echo "Executing: adb $SERIAL_OPTION exec-out screenrecord $CMD_OPTS -" >&2
adb $SERIAL_OPTION exec-out screenrecord $CMD_OPTS -

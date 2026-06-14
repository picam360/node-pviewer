import asyncio
import json
import time
import redis
from bleak import BleakClient, BleakScanner

TARGET_ADDRESS = "C8:47:80:46:03:C0"
TARGET_NAME = "L-12200BNN160-C00028"

READ_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
WRITE_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"

REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379
REDIS_CHANNEL = "litime-bms"

QUERY_BATTERY_STATUS = bytes([
    0x00, 0x00, 0x04, 0x01,
    0x13,
    0x55, 0xAA,
    0x17,
])


def hex_to_int(hexstr):
    return int(hexstr, 16)


def hex_to_bin(hexstr):
    return bin(int(hexstr, 16))[2:]


def parse_litime(data: bytes):
    s = data.hex()
    t = [s[i:i + 2] for i in range(0, len(s), 2)]

    def rev_hex(start, end):
        return "".join(reversed(t[start:end]))

    many_ff = 2 ** 16

    ret = {
        "timestamp": int(time.time() * 1000),
        "messured_total_voltage": hex_to_int(rev_hex(8, 12)) / 1000,
        "cells_added_together_voltage": hex_to_int(rev_hex(12, 16)) / 1000,
        "mosfet_temp": int(rev_hex(54, 56), 16),
        "remaining_amph": hex_to_int(rev_hex(62, 64)) / 100,
        "full_charge_capacity_amph": hex_to_int(rev_hex(64, 66)) / 100,
        "protection_state": rev_hex(76, 80),
        "heat": rev_hex(68, 72),
        "balance_memory_active": rev_hex(72, 76),
        "failure_state": rev_hex(80, 84)[-3:],
        "is_balancing": hex_to_bin(rev_hex(84, 88)),
        "battery_state": rev_hex(88, 90),
        "SOC": hex_to_int(rev_hex(90, 92)),
        "SOH": f"{hex_to_int(rev_hex(92, 96))}%",
        "discharges_count": hex_to_int(rev_hex(96, 100)),
        "discharges_amph_count": hex_to_int(rev_hex(100, 104)),
    }

    cell_temp = int(rev_hex(52, 54), 16)
    if cell_temp > many_ff / 2 - 1:
        cell_temp -= many_ff
    ret["cell_temp"] = cell_temp

    raw_current = int(rev_hex(48, 52), 16)
    r = ~raw_current
    ret["current"] = (-r if r > 0 else raw_current) / 1000

    cells = []
    a = t[16:48]
    for i in range(0, len(a), 2):
        if i + 1 < len(a) and a[i] != "00" and a[i + 1] != "00":
            cells.append(hex_to_int(a[i + 1] + a[i]) / 1000)

    ret["cellsVoltages"] = cells
    return ret


async def find_device():
    devices = await BleakScanner.discover(timeout=10)

    for d in devices:
        if d.address.upper() == TARGET_ADDRESS.upper():
            return d
        if d.name == TARGET_NAME:
            return d

    return None


async def main():
    r = redis.Redis(
        host=REDIS_HOST,
        port=REDIS_PORT,
        decode_responses=True,
    )

    while True:
        device = await find_device()

        if not device:
            await asyncio.sleep(5)
            continue

        try:
            async with BleakClient(device.address) as client:
                def on_notify(sender, data):
                    try:
                        parsed = parse_litime(data)
                        payload = json.dumps(parsed, ensure_ascii=False)
                        r.publish(REDIS_CHANNEL, payload)
                    except Exception as e:
                        err = json.dumps({
                            "timestamp": int(time.time() * 1000),
                            "error": str(e),
                        })
                        r.publish(REDIS_CHANNEL, err)

                await client.start_notify(READ_UUID, on_notify)

                while client.is_connected:
                    await client.write_gatt_char(
                        WRITE_UUID,
                        QUERY_BATTERY_STATUS,
                        response=True,
                    )
                    await asyncio.sleep(1)

        except Exception as e:
            r.publish(REDIS_CHANNEL, json.dumps({
                "timestamp": int(time.time() * 1000),
                "error": str(e),
            }))
            await asyncio.sleep(5)


if __name__ == "__main__":
    asyncio.run(main())

const i2c = require('i2c-bus');
const redis = require('redis');

const ICM_ADDR = 0x68;

// ---- registers ----
const REG_WHO_AM_I   = 0x75;
const REG_PWR_MGMT_1 = 0x6B;
const REG_ACCEL_XOUT_H = 0x3B;

// ---- scales ----
const ACCEL_SCALE = 9.80665 / 16384.0;   // ±2g
const GYRO_SCALE  = Math.PI / 180 / 131; // ±250 dps

// ---- timestamp ----
let t0_hr = 0n;
let t0_epoch_ns = 0n;

function now_ns() {
    const hr = process.hrtime.bigint();
    let now_ns = t0_epoch_ns + (hr - t0_hr);
    const date_ns = BigInt(Date.now()) * 1_000_000n;

    if ((now_ns > date_ns ? now_ns - date_ns : date_ns - now_ns) > 1_000_000_000n) {
        t0_hr = hr;
        t0_epoch_ns = date_ns;
        now_ns = date_ns;
    }

    return {
        sec: Number(now_ns / 1_000_000_000n),
        nanosec: Number(now_ns % 1_000_000_000n),
    };
}

// ---- init ----
function initICM(bus) {
    const who = bus.readByteSync(ICM_ADDR, REG_WHO_AM_I);
    if (who !== 0xE1) throw new Error("ICM20498 WHO_AM_I mismatch");

    bus.writeByteSync(ICM_ADDR, REG_PWR_MGMT_1, 0x01); // PLL
    console.log("ICM20498 initialized");
}

// ---- burst read ----
function readImu(bus) {
    const buf = Buffer.alloc(14);
    bus.readI2cBlockSync(ICM_ADDR, REG_ACCEL_XOUT_H, 14, buf);

    const i16 = (o) => {
        let v = (buf[o] << 8) | buf[o + 1];
        if (v & 0x8000) v -= 65536;
        return v;
    };

    const accel = {
        x: i16(0) * ACCEL_SCALE,
        y: i16(2) * ACCEL_SCALE,
        z: i16(4) * ACCEL_SCALE,
    };

    const gyro = {
        x: i16(8)  * GYRO_SCALE,
        y: i16(10) * GYRO_SCALE,
        z: i16(12) * GYRO_SCALE,
    };

    return { accel, gyro };
}

// ---- main ----
async function main() {
    const bus = i2c.openSync(7);
    initICM(bus);

    const client = redis.createClient();
    await client.connect();

    setInterval(() => {
        const timestamp = now_ns();
        const { accel, gyro } = readImu(bus);

        client.publish(
            "pserver-imu",
            JSON.stringify({
                timestamp,
                accel,
                gyro,
            })
        );
    }, 10); // 100Hz
}

if (require.main === module) {
    main();
}

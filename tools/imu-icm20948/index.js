const i2c = require('i2c-bus');
const redis = require('redis');

const ICM_ADDR = 0x68;

// ======================================================
// ICM-20948 registers
// ======================================================
const REG_BANK_SEL = 0x7F;

// Bank 0
const REG_WHO_AM_I       = 0x00;
const REG_PWR_MGMT_1     = 0x06;
const REG_ACCEL_XOUT_H   = 0x2D;

// Expected WHO_AM_I value
const WHO_AM_I_ICM20948 = 0xEA;

// ======================================================
// Scale factors (default full-scale settings)
// Accel: ±2g, Gyro: ±250 dps
// ======================================================
const ACCEL_SCALE = 9.80665 / 16384.0;   // m/s^2 per LSB
const GYRO_SCALE  = Math.PI / 180 / 131; // rad/s per LSB

// ======================================================
// Timestamp helper (same as original code)
// ======================================================
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

// ======================================================
// Helper: select register bank
// ======================================================
function selectBank(bus, bank) {
    bus.writeByteSync(ICM_ADDR, REG_BANK_SEL, bank << 4);
}

// ======================================================
// Initialization
// ======================================================
function initICM20948(bus) {
    // Select bank 0
    selectBank(bus, 0);

    const who = bus.readByteSync(ICM_ADDR, REG_WHO_AM_I);
    if (who !== WHO_AM_I_ICM20948) {
        throw new Error(`ICM20948 WHO_AM_I mismatch: 0x${who.toString(16)}`);
    }

    // Wake up device and select best clock source
    bus.writeByteSync(ICM_ADDR, REG_PWR_MGMT_1, 0x01);

    console.log('ICM20948 initialized');
}

// ======================================================
// Burst read: accel + gyro (Bank 0)
// ======================================================
function readImu(bus) {
    const buf = Buffer.alloc(12);
    bus.readI2cBlockSync(ICM_ADDR, REG_ACCEL_XOUT_H, 12, buf);

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
        x: i16(6)  * GYRO_SCALE,
        y: i16(8)  * GYRO_SCALE,
        z: i16(10) * GYRO_SCALE,
    };

    return { accel, gyro };
}

// ======================================================
// Main
// ======================================================
async function main() {
    const bus = i2c.openSync(7);
    initICM20948(bus);

    const client = redis.createClient();
    await client.connect();

    setInterval(() => {
        const timestamp = now_ns();
        const { accel, gyro } = readImu(bus);
        const imu_value = {
            timestamp,
            accel,
            gyro,
        };
        client.publish(
            'pserver-imu',
            JSON.stringify(imu_value, (key, value) => {
                if (typeof value === "number") {
                    return Math.round(value * 1e9) / 1e9;
                }else{
                    return value;
                }
            }, 2)
        );
    }, 1000/500); // 500 Hz
}

if (require.main === module) {
    main();
}

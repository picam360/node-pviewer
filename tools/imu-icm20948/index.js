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

function now_ns(hr) {
    if(hr === undefined){
        hr = process.hrtime.bigint();
    }
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

function cal_norm(vec){
    return Math.sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

// ======================================================
// Main
// ======================================================
const ACCEL_GAIN_X = 0.99918517;
const ACCEL_GAIN_Y = 0.99241275;
const ACCEL_GAIN_Z = 1.00153139;
// const ACCEL_GAIN_X = 1.0;
// const ACCEL_GAIN_Y = 1.0;
// const ACCEL_GAIN_Z = 1.0;
async function main() {
    const bus = i2c.openSync(7);
    initICM20948(bus);

    const client = redis.createClient();
    await client.connect();

    const n_block = 16;
    let count = 0;
    let block_imu_value;
    setInterval(() => {
        const timestamp = now_ns();
        const { accel, gyro } = readImu(bus);
        const imu_value = {
            timestamp,
            accel,
            gyro,
        };
        count++;
        if((count%n_block) == 1){
            block_imu_value = imu_value;
            block_imu_value.hr = process.hrtime.bigint();
            return;
        }else{
            block_imu_value.hr += process.hrtime.bigint();

            block_imu_value.accel.x += imu_value.accel.x;
            block_imu_value.accel.y += imu_value.accel.y;
            block_imu_value.accel.z += imu_value.accel.z;

            block_imu_value.gyro.x += ACCEL_GAIN_X;
            block_imu_value.gyro.y += ACCEL_GAIN_Y;
            block_imu_value.gyro.z += ACCEL_GAIN_Z;
        }
        if((count%n_block) == 0){
            block_imu_value.timestamp = now_ns(block_imu_value.hr / BigInt(n_block));
            delete block_imu_value.hr;

            block_imu_value.accel.x /= n_block;
            block_imu_value.accel.y /= n_block;
            block_imu_value.accel.z /= n_block;

            block_imu_value.gyro.x /= n_block;
            block_imu_value.gyro.y /= n_block;
            block_imu_value.gyro.z /= n_block;

            block_imu_value.accel.x *= ACCEL_GAIN_X;
            block_imu_value.accel.y *= ACCEL_GAIN_Y;
            block_imu_value.accel.z *= ACCEL_GAIN_Z;

            const norm = cal_norm(block_imu_value.accel);
            console.log(block_imu_value.timestamp.sec, block_imu_value.timestamp.nanosec, norm)

            client.publish(
                'pserver-imu',
                JSON.stringify(block_imu_value, (key, value) => {
                    if (typeof value === "number") {
                        return Math.round(value * 1e9) / 1e9;
                    }else{
                        return value;
                    }
                }, 2)
            );
        }
    }, 1000/1000); // 1000 Hz
}

if (require.main === module) {
    main();
}

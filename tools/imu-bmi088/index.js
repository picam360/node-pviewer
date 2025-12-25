const i2c = require('i2c-bus');
const redis = require('redis');

// ===============================
// I2C addresses
// ===============================
const BMI088_ACC_ADDR  = 0x19;
const BMI088_GYRO_ADDR = 0x69;

// ===============================
// Accelerometer registers
// ===============================
const ACC_CHIP_ID     = 0x00;
const ACC_CHIP_ID_VAL = 0x1E;

const ACC_PWR_CTRL    = 0x7D;
const ACC_PWR_CONF    = 0x7C;
const ACC_CONF        = 0x40;
const ACC_RANGE       = 0x41;
const ACC_X_LSB       = 0x12;

// ===============================
// Gyroscope registers
// ===============================
const GYRO_CHIP_ID     = 0x00;
const GYRO_CHIP_ID_VAL = 0x0F;

const GYRO_RANGE      = 0x0F;
const GYRO_BW         = 0x10;
const GYRO_LPM1       = 0x11;
const GYRO_X_LSB      = 0x02;

// ===============================
// scales
// ===============================
// accel ±3g
const ACCEL_SCALE = 9.80665 / 1024.0;
// gyro ±2000 dps
const GYRO_SCALE  = Math.PI / 180 / 16.4;

// ===============================
// timestamp
// ===============================
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

// ===============================
// init BMI088
// ===============================
function initBMI088(bus) {

    // ---- accel ----
    const acc_id = bus.readByteSync(BMI088_ACC_ADDR, ACC_CHIP_ID);
    if (acc_id !== ACC_CHIP_ID_VAL)
        throw new Error(`BMI088 accel WHO_AM_I mismatch: ${acc_id}`);

    bus.writeByteSync(BMI088_ACC_ADDR, ACC_PWR_CTRL, 0x04); // active
    bus.writeByteSync(BMI088_ACC_ADDR, ACC_PWR_CONF, 0x00); // normal
    bus.writeByteSync(BMI088_ACC_ADDR, ACC_CONF, 0xA8);     // 800Hz
    bus.writeByteSync(BMI088_ACC_ADDR, ACC_RANGE, 0x01);    // ±3g

    // ---- gyro ----
    const gyro_id = bus.readByteSync(BMI088_GYRO_ADDR, GYRO_CHIP_ID);
    if (gyro_id !== GYRO_CHIP_ID_VAL)
        throw new Error(`BMI088 gyro WHO_AM_I mismatch: ${gyro_id}`);

    bus.writeByteSync(BMI088_GYRO_ADDR, GYRO_RANGE, 0x00); // ±2000 dps
    bus.writeByteSync(BMI088_GYRO_ADDR, GYRO_BW, 0x02);    // 400Hz
    bus.writeByteSync(BMI088_GYRO_ADDR, GYRO_LPM1, 0x00);  // normal

    console.log("BMI088 initialized");
}

// ===============================
// burst read
// ===============================
function readBMI088(bus) {

    const acc_buf = Buffer.alloc(6);
    bus.readI2cBlockSync(BMI088_ACC_ADDR, ACC_X_LSB, 6, acc_buf);

    const gyro_buf = Buffer.alloc(6);
    bus.readI2cBlockSync(BMI088_GYRO_ADDR, GYRO_X_LSB, 6, gyro_buf);

    const i16 = (buf, o) => {
        let v = (buf[o + 1] << 8) | buf[o];
        if (v & 0x8000) v -= 65536;
        return v;
    };

    const accel = {
        x: i16(acc_buf, 0) * ACCEL_SCALE,
        y: i16(acc_buf, 2) * ACCEL_SCALE,
        z: i16(acc_buf, 4) * ACCEL_SCALE,
    };

    const gyro = {
        x: i16(gyro_buf, 0) * GYRO_SCALE,
        y: i16(gyro_buf, 2) * GYRO_SCALE,
        z: i16(gyro_buf, 4) * GYRO_SCALE,
    };

    return { accel, gyro };
}

// ===============================
// main
// ===============================
async function main() {
    const bus = i2c.openSync(7);
    initBMI088(bus);

    const client = redis.createClient();
    await client.connect();

    setInterval(() => {
        const timestamp = now_ns();
        const { accel, gyro } = readBMI088(bus);

        client.publish(
            "pserver-imu",
            JSON.stringify({ timestamp, accel, gyro })
        );
    }, 10); // 100Hz
}

if (require.main === module) {
    main();
}

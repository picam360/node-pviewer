const i2c = require('i2c-bus');
const AHRS = require('ahrs');

// ======================================================
// ICM-20948 definitions
// ======================================================
const ICM_ADDR = 0x68;

// Bank select
const REG_BANK_SEL = 0x7F;

// Bank 0 registers
const REG_WHO_AM_I     = 0x00;
const REG_PWR_MGMT_1   = 0x06;
const REG_ACCEL_XOUT_H = 0x2D;

// WHO_AM_I expected value
const WHO_AM_I_ICM20948 = 0xEA;

// Scale factors (default settings)
// Accelerometer: ±2g
// Gyroscope: ±250 dps
const ACCEL_SCALE = 9.80665 / 16384.0;   // m/s^2 per LSB
const GYRO_SCALE  = Math.PI / 180 / 131; // rad/s per LSB

// ======================================================
// I2C bus
// ======================================================
const i2cBus = i2c.openSync(7);

// ======================================================
// AHRS (Madgwick filter)
// ======================================================
const ahrs = new AHRS({
  algorithm: 'Madgwick',
  sampleInterval: 1000, // ms (1 Hz, same as BNO055 test)
  beta: 0.1
});

// ======================================================
// Helper: select register bank
// ======================================================
function selectBank(bank) {
  i2cBus.writeByteSync(ICM_ADDR, REG_BANK_SEL, bank << 4);
}

// ======================================================
// Initialization
// ======================================================
function initICM20948() {
  // Select bank 0
  selectBank(0);

  const who = i2cBus.readByteSync(ICM_ADDR, REG_WHO_AM_I);
  if (who !== WHO_AM_I_ICM20948) {
    throw new Error(`ICM20948 WHO_AM_I mismatch: 0x${who.toString(16)}`);
  }

  // Wake up device, auto select best clock
  i2cBus.writeByteSync(ICM_ADDR, REG_PWR_MGMT_1, 0x01);

  console.log('ICM20948 initialized');
}

// ======================================================
// Read accelerometer and gyroscope (Bank 0)
// ======================================================
function readImu() {
  const buffer = Buffer.alloc(12);
  i2cBus.readI2cBlockSync(ICM_ADDR, REG_ACCEL_XOUT_H, 12, buffer);

  const int16 = (o) => {
    let v = (buffer[o] << 8) | buffer[o + 1];
    if (v & 0x8000) v -= 65536;
    return v;
  };

  const accel = {
    x: int16(0) * ACCEL_SCALE,
    y: int16(2) * ACCEL_SCALE,
    z: int16(4) * ACCEL_SCALE,
  };

  const gyro = {
    x: int16(6)  * GYRO_SCALE,
    y: int16(8)  * GYRO_SCALE,
    z: int16(10) * GYRO_SCALE,
  };

  return { accel, gyro };
}

// ======================================================
// Quaternion to heading (yaw)
// ======================================================
function quaternionToHeading(q) {
  const { w, x, y, z } = q;

  const t3 = 2.0 * (w * z + x * y);
  const t4 = 1.0 - 2.0 * (y * y + z * z);
  let heading = Math.atan2(t3, t4) * 180 / Math.PI;

  // Match BNO055 behavior
  heading = -heading;
  if (heading < 0) heading += 360;

  return heading;
}

// ======================================================
// Main
// ======================================================
try {
  initICM20948();

  setInterval(() => {
    const { accel, gyro } = readImu();

    // Update AHRS
    // gyro: rad/s, accel: m/s^2
    ahrs.update(
      gyro.x, gyro.y, gyro.z,
      accel.x, accel.y, accel.z
    );

    const q = ahrs.getQuaternion();

    console.log('Quaternion');
    console.log('W:', q.w);
    console.log('X:', q.x);
    console.log('Y:', q.y);
    console.log('Z:', q.z);

    const heading = quaternionToHeading(q);
    console.log('Heading:', heading);
    console.log('---');

  }, 1000); // 1 second interval

} catch (err) {
  console.error('Error:', err.message);
  i2cBus.closeSync();
}

const i2c = require('i2c-bus');
const AHRS = require('ahrs');

// ======================================================
// BMI088 definitions
// ======================================================
const BMI088_ACC_ADDR  = 0x19;
const BMI088_GYRO_ADDR = 0x69;

// ---------- Accelerometer registers ----------
const ACC_CHIP_ID     = 0x00;
const ACC_CHIP_ID_VAL = 0x1E;

const ACC_PWR_CTRL    = 0x7D;
const ACC_PWR_CONF    = 0x7C;
const ACC_CONF        = 0x40;
const ACC_RANGE       = 0x41;
const ACC_X_LSB       = 0x12;

// ---------- Gyroscope registers ----------
const GYRO_CHIP_ID     = 0x00;
const GYRO_CHIP_ID_VAL = 0x0F;

const GYRO_RANGE      = 0x0F;
const GYRO_BW         = 0x10;
const GYRO_LPM1       = 0x11;
const GYRO_X_LSB      = 0x02;

// ======================================================
// Scale factors
// ======================================================
// Accelerometer: ±3g
const ACCEL_SCALE = 9.80665 / 1024.0; // m/s^2 per LSB

// Gyroscope: ±2000 dps
const GYRO_SCALE = Math.PI / 180 / 16.4; // rad/s per LSB

// ======================================================
// I2C bus
// ======================================================
const i2cBus = i2c.openSync(7);

// ======================================================
// AHRS (Madgwick)
// ======================================================
const ahrs = new AHRS({
  algorithm: 'Madgwick',
  sampleInterval: 1000, // ms
  beta: 0.1
});

// ======================================================
// Initialization
// ======================================================
function initBMI088() {

  // ---- Accelerometer ----
  const accId = i2cBus.readByteSync(BMI088_ACC_ADDR, ACC_CHIP_ID);
  if (accId !== ACC_CHIP_ID_VAL) {
    throw new Error(`BMI088 accel WHO_AM_I mismatch: 0x${accId.toString(16)}`);
  }

  i2cBus.writeByteSync(BMI088_ACC_ADDR, ACC_PWR_CTRL, 0x04); // active
  i2cBus.writeByteSync(BMI088_ACC_ADDR, ACC_PWR_CONF, 0x00); // normal
  i2cBus.writeByteSync(BMI088_ACC_ADDR, ACC_CONF, 0xA8);     // 800 Hz
  i2cBus.writeByteSync(BMI088_ACC_ADDR, ACC_RANGE, 0x01);    // ±3g

  // ---- Gyroscope ----
  const gyroId = i2cBus.readByteSync(BMI088_GYRO_ADDR, GYRO_CHIP_ID);
  if (gyroId !== GYRO_CHIP_ID_VAL) {
    throw new Error(`BMI088 gyro WHO_AM_I mismatch: 0x${gyroId.toString(16)}`);
  }

  i2cBus.writeByteSync(BMI088_GYRO_ADDR, GYRO_RANGE, 0x00); // ±2000 dps
  i2cBus.writeByteSync(BMI088_GYRO_ADDR, GYRO_BW, 0x02);    // 400 Hz
  i2cBus.writeByteSync(BMI088_GYRO_ADDR, GYRO_LPM1, 0x00);  // normal

  console.log('BMI088 initialized');
}

// ======================================================
// Read accelerometer + gyroscope
// ======================================================
function readImu() {

  const accBuf = Buffer.alloc(6);
  i2cBus.readI2cBlockSync(BMI088_ACC_ADDR, ACC_X_LSB, 6, accBuf);

  const gyroBuf = Buffer.alloc(6);
  i2cBus.readI2cBlockSync(BMI088_GYRO_ADDR, GYRO_X_LSB, 6, gyroBuf);

  const int16 = (buf, o) => {
    let v = (buf[o + 1] << 8) | buf[o];
    if (v & 0x8000) v -= 65536;
    return v;
  };

  const accel = {
    x: int16(accBuf, 0) * ACCEL_SCALE,
    y: int16(accBuf, 2) * ACCEL_SCALE,
    z: int16(accBuf, 4) * ACCEL_SCALE,
  };

  const gyro = {
    x: int16(gyroBuf, 0) * GYRO_SCALE,
    y: int16(gyroBuf, 2) * GYRO_SCALE,
    z: int16(gyroBuf, 4) * GYRO_SCALE,
  };

  return { accel, gyro };
}

// ======================================================
// Quaternion → heading (yaw)
// ======================================================
function quaternionToHeading(q) {
  const { w, x, y, z } = q;

  const t3 = 2.0 * (w * z + x * y);
  const t4 = 1.0 - 2.0 * (y * y + z * z);
  let heading = Math.atan2(t3, t4) * 180 / Math.PI;

  // BNO055互換
  heading = -heading;
  if (heading < 0) heading += 360;

  return heading;
}

// ======================================================
// Main
// ======================================================
try {
  initBMI088();

  setInterval(() => {
    const { accel, gyro } = readImu();

    // Update AHRS (gyro: rad/s, accel: m/s^2)
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

  }, 1000);

} catch (err) {
  console.error('Error:', err.message);
  i2cBus.closeSync();
}

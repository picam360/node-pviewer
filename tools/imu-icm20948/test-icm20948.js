const i2c = require('i2c-bus');
const Madgwick = require('madgwick');

// =======================
// ICM-20498 定義
// =======================
const ICM_ADDR = 0x68;

// レジスタ
const REG_WHO_AM_I   = 0x75;
const REG_PWR_MGMT_1 = 0x6B;
const REG_ACCEL_XOUT_H = 0x3B;

// スケール
const ACCEL_SCALE = 9.80665 / 16384.0;   // ±2g
const GYRO_SCALE  = Math.PI / 180 / 131; // ±250 dps

// =======================
// I2C 初期化
// =======================
const i2cBus = i2c.openSync(1);

// Madgwick フィルタ
const madgwick = new Madgwick({
  sampleInterval: 1000, // ms
  beta: 0.1
});

// =======================
// 初期化
// =======================
function initICM20498() {
  const who = i2cBus.readByteSync(ICM_ADDR, REG_WHO_AM_I);
  if (who !== 0xE1) {
    throw new Error('ICM20498 WHO_AM_I mismatch');
  }

  // PLL 使用
  i2cBus.writeByteSync(ICM_ADDR, REG_PWR_MGMT_1, 0x01);
  console.log('ICM20498 初期化完了');
}

// =======================
// IMU 読み取り
// =======================
function readImu() {
  const buf = Buffer.alloc(14);
  i2cBus.readI2cBlockSync(ICM_ADDR, REG_ACCEL_XOUT_H, 14, buf);

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

// =======================
// Quaternion → Heading
// =======================
function quaternionToHeading(q) {
  const { w, x, y, z } = q;

  const t3 = 2.0 * (w * z + x * y);
  const t4 = 1.0 - 2.0 * (y * y + z * z);
  let heading = Math.atan2(t3, t4) * 180 / Math.PI;

  heading = -heading;
  if (heading < 0) heading += 360;
  return heading;
}

// =======================
// メイン
// =======================
try {
  initICM20498();

  setInterval(() => {
    const { accel, gyro } = readImu();

    // Madgwick 更新（rad/s, m/s²）
    madgwick.update(
      gyro.x, gyro.y, gyro.z,
      accel.x, accel.y, accel.z
    );

    const q = madgwick.getQuaternion();

    console.log('クォータニオン');
    console.log('W:', q.w);
    console.log('X:', q.x);
    console.log('Y:', q.y);
    console.log('Z:', q.z);

    const heading = quaternionToHeading(q);
    console.log('方位 (Heading):', heading);
    console.log('---');

  }, 1000);

} catch (err) {
  console.error('エラー:', err.message);
  i2cBus.closeSync();
}

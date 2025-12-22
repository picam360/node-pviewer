
const fs = require('fs');
const i2c = require('i2c-bus');
let m_options = {
    bus_num : 7,
    calib_path : "/etc/pserver/imu-bno055.calib",
    fps : 100,
};
let m_mode = "CONFIG";

let t0_hr = 0n;
let t0_epoch_ns = 0n;
function now_ns() {
    const hr = process.hrtime.bigint();
    let now_ns = t0_epoch_ns + (hr - t0_hr);

    const date_now_ns = BigInt(Date.now()) * 1_000_000n;

    const diff_ns = now_ns > date_now_ns
        ? now_ns - date_now_ns
        : date_now_ns - now_ns;

    if (diff_ns > 1_000_000_000n) {
        t0_hr = hr;
        t0_epoch_ns = date_now_ns;
        now_ns = date_now_ns;
        console.log("now_ns base updated");
    }

    return {
        sec: Number(now_ns / 1_000_000_000n),
        nanosec: Number(now_ns % 1_000_000_000n)
    };
}

// BNO055 I2C address
const BNO055_I2C_ADDR = 0x28; // (Primary address: 0x28, Secondary: 0x29)

// BNO055 registers
const BNO055_CHIP_ID = 0x00;
const BNO055_OPR_MODE = 0x3D;
const BNO055_PWR_MODE = 0x3E;
const BNO055_SYS_TRIGGER = 0x3F;
const BNO055_UNIT_SEL = 0x3B;
const BNO055_EULER_H_LSB = 0x1A;
// Accelerometer (LSB start)
const BNO055_ACCEL_DATA_X_LSB = 0x08;
// Gyroscope (LSB start)
const BNO055_GYRO_DATA_X_LSB  = 0x14;
// Quaternion (already defined)
const BNO055_QUATERNION_DATA_W_LSB = 0x20; // Starting register for quaternion data

// BNO055 operating mode (NDOF: 9DOF fusion)
const OPERATION_MODE_CONFIG = 0x00;
const OPERATION_MODE_NDOF = 0x0C;

function saveCalibrationData(i2cBus, calib_path) {
    const buffer = Buffer.alloc(22);
    i2cBus.readI2cBlockSync(BNO055_I2C_ADDR, 0x55, 22, buffer); // Read calibration data
    fs.writeFileSync(calib_path, buffer);
}

function loadCalibrationData(i2cBus, calib_path) {
    if (fs.existsSync(calib_path)) {
        const buffer = fs.readFileSync(calib_path);
        i2cBus.writeI2cBlockSync(BNO055_I2C_ADDR, 0x55, buffer.length, buffer); // Write back calibration data
        console.log('Calibration data loaded successfully');
    } else {
        console.log('Calibration data not found');
    }
}

function getCalibrationStatus(i2cBus) {
    const CALIB_STAT_ADDR = 0x35; // Calibration status register
    const status = i2cBus.readByteSync(BNO055_I2C_ADDR, CALIB_STAT_ADDR);

    const sys = (status >> 6) & 0x03;   // System calibration (2 bits)
    const gyro = (status >> 4) & 0x03;  // Gyroscope calibration (2 bits)
    const accel = (status >> 2) & 0x03; // Accelerometer calibration (2 bits)
    const mag = status & 0x03;          // Magnetometer calibration (2 bits)

    return {
        sys: sys,
        gyro: gyro,
        accel: accel,
        mag: mag,
    };
}

function switchConfigMode(i2cBus) {
    return new Promise((resolve, reject) => {
        i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_OPR_MODE, OPERATION_MODE_CONFIG); // Set to CONFIG mode
        console.log('BNO055 switch config mode successfully');
        setTimeout(() => {
            resolve();
        }, 10);
    });
}

// Initialization function
function initBNO055(i2cBus) {
    return new Promise((resolve, reject) => {
        try {
            // Check chip ID
            const chipId = i2cBus.readByteSync(BNO055_I2C_ADDR, BNO055_CHIP_ID);
            if (chipId !== 0xA0) {
                throw new Error('BNO055 chip ID does not match');
            }

            switchConfigMode(i2cBus).then(() => {
                if(m_options.calib_path){
                    loadCalibrationData(i2cBus, m_options.calib_path);
                }
        
                // Initialize the sensor
                i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_PWR_MODE, 0x00); // Normal operating mode
                i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 0x00); // Reset release
                i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_OPR_MODE, OPERATION_MODE_NDOF); // Set to NDOF mode
                console.log('BNO055 initialized successfully');
        
                resolve();
            }).catch((err) => {
                reject(err);
            });
        } catch (err) {
            reject(err);
        }
    });
}

// Function to retrieve quaternion data
function readQuaternion(i2cBus) {
    const buffer = Buffer.alloc(8); // Buffer for 8 bytes
    i2cBus.readI2cBlockSync(BNO055_I2C_ADDR, BNO055_QUATERNION_DATA_W_LSB, 8, buffer);

    // Retrieve raw 16-bit integer values
    let w = (buffer[1] << 8) | buffer[0];
    let x = (buffer[3] << 8) | buffer[2];
    let y = (buffer[5] << 8) | buffer[4];
    let z = (buffer[7] << 8) | buffer[6];

    // Handle negative values properly as two's complement
    if (w & 0x8000) w -= 65536;
    if (x & 0x8000) x -= 65536;
    if (y & 0x8000) y -= 65536;
    if (z & 0x8000) z -= 65536;

    // Normalize quaternion (1 unit = 1/2^14)
    const scale = 1.0 / (1 << 14); // 1/16384
    return {
        w: w * scale,
        x: x * scale,
        y: y * scale,
        z: z * scale,
    };
}

function readVec3Int16(i2cBus, startReg) {
    const buffer = Buffer.alloc(6);
    i2cBus.readI2cBlockSync(
        BNO055_I2C_ADDR,
        startReg,
        6,
        buffer
    );

    let x = (buffer[1] << 8) | buffer[0];
    let y = (buffer[3] << 8) | buffer[2];
    let z = (buffer[5] << 8) | buffer[4];

    if (x & 0x8000) x -= 65536;
    if (y & 0x8000) y -= 65536;
    if (z & 0x8000) z -= 65536;

    return { x, y, z };
}

function readAccel(i2cBus) {
    const raw = readVec3Int16(
        i2cBus,
        BNO055_ACCEL_DATA_X_LSB
    );

    // 1 LSB = 1 mg = 0.00981 m/s²
    const scale = 0.00981;

    return {
        x: raw.x * scale,
        y: raw.y * scale,
        z: raw.z * scale,
    };
}

function readGyro(i2cBus) {
    const raw = readVec3Int16(
        i2cBus,
        BNO055_GYRO_DATA_X_LSB
    );

    // 1 LSB = 1/16 deg/s
    const scale = (1.0 / 16.0) * (Math.PI / 180.0); // rad/s

    return {
        x: raw.x * scale,
        y: raw.y * scale,
        z: raw.z * scale,
    };
}

function readImuBurst(i2cBus) {
    const BNO055_BURST_START = 0x08;
    const BNO055_BURST_LEN   = 0x20; // 32 bytes
    const buffer = Buffer.alloc(BNO055_BURST_LEN);

    i2cBus.readI2cBlockSync(
        BNO055_I2C_ADDR,
        BNO055_BURST_START,
        BNO055_BURST_LEN,
        buffer
    );

    const readInt16 = (offset) => {
        let v = (buffer[offset + 1] << 8) | buffer[offset];
        if (v & 0x8000) v -= 65536;
        return v;
    };

    // ---- Accel (m/s²) ----
    const accel = {
        x: readInt16(0)  * 0.00981,
        y: readInt16(2)  * 0.00981,
        z: readInt16(4)  * 0.00981,
    };

    // ---- Gyro (rad/s) ----
    const DEG2RAD = Math.PI / 180.0;
    const gyro = {
        x: readInt16(12) * (1 / 16) * DEG2RAD,
        y: readInt16(14) * (1 / 16) * DEG2RAD,
        z: readInt16(16) * (1 / 16) * DEG2RAD,
    };

    // ---- Quaternion ----
    const scaleQuat = 1.0 / (1 << 14);
    const quaternion = {
        w: readInt16(24) * scaleQuat,
        x: readInt16(26) * scaleQuat,
        y: readInt16(28) * scaleQuat,
        z: readInt16(30) * scaleQuat,
    };

    return { accel, gyro, quaternion };
}


// Function to calculate heading (Yaw) from quaternion
function quaternionToHeading(quaternion) {
    const w = quaternion.w;
    const x = quaternion.x;
    const y = quaternion.y;
    const z = quaternion.z;

    // Calculate heading (rotation around Z axis)
    const t3 = +2.0 * (w * z + x * y);
    const t4 = +1.0 - 2.0 * (y * y + z * z);
    let heading = Math.atan2(t3, t4) * (180 / Math.PI); // Convert radians to degrees

    // Reverse the heading (invert the sign)
    heading = -heading;

    // Adjust to range of 0–360 degrees
    if (heading < 0) {
        return heading + 360;
    }
    return heading;
}

function main(){
    
    let i2cBus;
    let last_calib_saved = 0;

    // Main process
    i2cBus = i2c.openSync(m_options.bus_num);

    const redis = require('redis');
    const client = redis.createClient({
        host: 'localhost',
        port: 6379,
    });
    client.on('error', (err) => {
        console.error('redis error:', err);
        process.exit(-1);
    });
    client.connect().then(() => {
        console.log('redis connected:');
    });

    initBNO055(i2cBus).then(() => {
        return new Promise((resolve, reject) => {
            setInterval(() => {
                try {

                    const timestamp = now_ns();
                    const { accel, gyro, quaternion } = readImuBurst(i2cBus);
                    const heading = quaternionToHeading(quaternion);

                    const status = getCalibrationStatus(i2cBus);
                    if(m_options.calib_path){
                        if (status.sys == 3 && status.gyro == 3 && status.accel == 3 && status.mag == 3) {
                            if (timestamp - last_calib_saved >= 60) {
                                switchConfigMode(i2cBus).then(() => {
                                    saveCalibrationData(i2cBus, m_options.calib_path);
                            
                                    i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_OPR_MODE, OPERATION_MODE_NDOF); // Set to NDOF mode
                                    console.log('Calibration data saved successfully');

                                    last_calib_saved = timestamp;
                                }).catch((err) => {
                                    reject(err);
                                });
                            }
                        }
                    }

                    const imu_value = {
                        timestamp,
                        gyro,
                        accel,
                        quaternion,
                        heading,
                        status,
                    };
                    if (m_options.debug) {
                        console.log('calibration status:', imu_value);
                    }

                    client.publish(`pserver-imu`, JSON.stringify(imu_value), (err, reply) => {
                        if (err) {
                            console.error('Error publishing message:', err);
                        } else {
                            //console.log(`Message published to ${reply} subscribers.`);
                        }
                    });
                } catch (err) {
                    reject(err);
                }
            }, 1000 / m_options.fps); // Retrieve data every second
        });
    }).catch((err) => {
        console.error('Error:', err);
        process.exit(-1);
    });
}

if (require.main === module) {
    main();
}

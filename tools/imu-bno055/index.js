
const fs = require('fs');
const i2c = require('i2c-bus');
let m_options = {
    bus_num : 0,
    calib_path : "/etc/pserver/imu-bno055.calib"
};
let m_mode = "CONFIG";

// BNO055 I2C address
const BNO055_I2C_ADDR = 0x28; // (Primary address: 0x28, Secondary: 0x29)

// BNO055 registers
const BNO055_CHIP_ID = 0x00;
const BNO055_OPR_MODE = 0x3D;
const BNO055_PWR_MODE = 0x3E;
const BNO055_SYS_TRIGGER = 0x3F;
const BNO055_UNIT_SEL = 0x3B;
const BNO055_EULER_H_LSB = 0x1A;
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
                    const quaternion = readQuaternion(i2cBus);
                    
                    const heading = quaternionToHeading(quaternion);

                    const status = getCalibrationStatus(i2cBus);
                    if(m_options.calib_path){
                        const ts = Date.now();
                        if (status.sys == 3 && status.gyro == 3 && status.accel == 3 && status.mag == 3) {
                            if (ts - last_calib_saved >= 60000) {
                                switchConfigMode(i2cBus).then(() => {
                                    saveCalibrationData(i2cBus, m_options.calib_path);
                            
                                    i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_OPR_MODE, OPERATION_MODE_NDOF); // Set to NDOF mode
                                    console.log('Calibration data saved successfully');

                                    last_calib_saved = ts;
                                }).catch((err) => {
                                    reject(err);
                                });
                            }
                        }
                    }

                    const imu_value = {
                        heading,
                        quaternion,
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
            }, 1000); // Retrieve data every second
        });
    }).catch((err) => {
        console.error('Error:', err);
        process.exit(-1);
    });
}

if (require.main === module) {
    main();
}

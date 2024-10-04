module.exports = {
    create_plugin: function (plugin_host) {
        console.log("create imu_bno055 plugin");
        const fs = require('fs');
        const i2c = require('i2c-bus');
        let m_options = {
            bus_num : 0,
        };

        // BNO055 I2C address
        const BNO055_I2C_ADDR = 0x29; // (Primary address: 0x28, Secondary: 0x29)
        
        // BNO055 registers
        const BNO055_CHIP_ID = 0x00;
        const BNO055_OPR_MODE = 0x3D;
        const BNO055_PWR_MODE = 0x3E;
        const BNO055_SYS_TRIGGER = 0x3F;
        const BNO055_UNIT_SEL = 0x3B;
        const BNO055_EULER_H_LSB = 0x1A;
        const BNO055_QUATERNION_DATA_W_LSB = 0x20; // Starting register for quaternion data
        
        // BNO055 operating mode (NDOF: 9DOF fusion)
        const OPERATION_MODE_NDOF = 0x0C;

        function openclose_i2c(bus_num, task) {
            let i2cBus;
            try {
                i2cBus = i2c.openSync(bus_num);
                return task(i2cBus);
            } catch (error) {
                console.error("Error:", error);
            } finally {
                if (i2cBus) {
                    try {
                        i2cBus.closeSync();
                    } catch (closeError) {
                        console.error("Failed to close I2C bus:", closeError);
                    }
                }
            }
        }

        function saveCalibrationData(bus_num, calib_path) {
            return openclose_i2c(bus_num, (i2cBus) => {
                const buffer = Buffer.alloc(22);
                i2cBus.readI2cBlockSync(BNO055_I2C_ADDR, 0x55, 22, buffer); // Read calibration data
                // Save buffer to a file or memory here. Example: save to a file.
                fs.writeFileSync(calib_path, buffer);
            });
        }

        function loadCalibrationData(bus_num, calib_path) {
            return openclose_i2c(bus_num, (i2cBus) => {
                if (fs.existsSync(calib_path)) {
                    const buffer = fs.readFileSync(calib_path);
                    i2cBus.writeI2cBlockSync(BNO055_I2C_ADDR, 0x55, buffer.length, buffer); // Write back calibration data
                    console.log('Calibration data loaded successfully');
                } else {
                    console.log('Calibration data not found');
                }
            });
        }

        function getCalibrationStatus(bus_num) {
            return openclose_i2c(bus_num, (i2cBus) => {
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
            });
        }

        // Initialization function
        function initBNO055(bus_num) {
            return openclose_i2c(bus_num, (i2cBus) => {
                // Check chip ID
                const chipId = i2cBus.readByteSync(BNO055_I2C_ADDR, BNO055_CHIP_ID);
                if (chipId !== 0xA0) {
                    throw new Error('BNO055 chip ID does not match');
                }

                // Initialize the sensor
                i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_PWR_MODE, 0x00); // Normal operating mode
                i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 0x00); // Reset release
                i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_OPR_MODE, OPERATION_MODE_NDOF); // Set to NDOF mode
                console.log('BNO055 initialized successfully');
            });
        }

        // Function to retrieve quaternion data
        function readQuaternion(bus_num) {
            return openclose_i2c(bus_num, (i2cBus) => {
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
            });
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

        var plugin = {
            name: "imu_bno055",
            init_options: function (options) {
                m_options = Object.assign(m_options, options["imu_bno055"] || {});

                if (m_options && m_options.enabled) {
                    plugin.start_read();
                }
            },
            start_read: () => {

                // Main process
                initBNO055(m_options.bus_num);
                setInterval(() => {
                    try {
                        // Retrieve quaternion
                        const quaternion = readQuaternion(m_options.bus_num);
                        if (m_options.debug) {
                            console.log('Quaternion');
                            console.log('W:', quaternion.w);
                            console.log('X:', quaternion.x);
                            console.log('Y:', quaternion.y);
                            console.log('Z:', quaternion.z);
                        }

                        // Calculate heading from quaternion
                        const heading = quaternionToHeading(quaternion);
                        if (m_options.debug) {
                            console.log('Heading:', heading);
                        }

                        const imu_value = {
                            heading,
                            quaternion,
                        };

                        if (m_plugin_host.get_redis_client) {
                            const client = m_plugin_host.get_redis_client();
                            if (client) {
                                client.publish(`pserver-imu`, JSON.stringify(imu_value), (err, reply) => {
                                    if (err) {
                                        console.error('Error publishing message:', err);
                                    } else {
                                        //console.log(`Message published to ${reply} subscribers.`);
                                    }
                                });
                            }
                        }
                    } catch (error) {
                        console.error('Error:', error.message);
                    }
                }, 1000); // Retrieve data every second

                if(m_options.calib_path){
                    loadCalibrationData(m_options.bus_num, m_options.calib_path);
                    setInterval(() => {
                        const status = getCalibrationStatus(m_options.bus_num);
                        if (m_options.debug) {
                            console.log('calibration status:', status);
                        }
                        if (status.sys == 3 && status.gyro == 3 && status.accel == 3 && status.mag == 3) {
                            saveCalibrationData(m_options.bus_num, m_options.calib_path);
                            if (m_options.debug) {
                                console.log('Calibration data saved successfully');
                            }
                        }
                    }, 10000);
                }
            }
        };
        return plugin;
    }
};

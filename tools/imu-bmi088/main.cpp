#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/i2c-dev.h>

#include <hiredis/hiredis.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ===============================
// I2C addresses
// ===============================
#define BMI088_ACC_ADDR   0x19
#define BMI088_GYRO_ADDR  0x69

// ===============================
// Accelerometer registers
// ===============================
#define ACC_CHIP_ID       0x00
#define ACC_CHIP_ID_VAL   0x1E
#define ACC_PWR_CTRL      0x7D
#define ACC_PWR_CONF      0x7C
#define ACC_CONF          0x40
#define ACC_RANGE         0x41
#define ACC_X_LSB         0x12

// ===============================
// Gyroscope registers
// ===============================
#define GYRO_CHIP_ID      0x00
#define GYRO_CHIP_ID_VAL  0x0F
#define GYRO_RANGE        0x0F
#define GYRO_BW           0x10
#define GYRO_LPM1         0x11
#define GYRO_X_LSB        0x02

#define SAMPLING_RATE 800

// ===============================
// scales
// ===============================
// accel ±3g
const double ACCEL_SCALE = 9.80665 / 10920.0;
// gyro ±2000 dps
const double GYRO_SCALE  = M_PI / 180.0 / 16.4;

// ===============================
// I2C helpers
// ===============================
static void i2c_set_slave(int fd, uint8_t addr) {
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        perror("I2C_SLAVE");
        exit(1);
    }
}

static uint8_t i2c_read8(int fd, uint8_t reg) {
    write(fd, &reg, 1);
    uint8_t v;
    read(fd, &v, 1);
    return v;
}

static void i2c_write8(int fd, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    write(fd, buf, 2);
}

static void i2c_read_block(int fd, uint8_t reg, uint8_t *buf, size_t len) {
    write(fd, &reg, 1);
    read(fd, buf, len);
}

static inline int16_t i16(const uint8_t *b, int o) {
    return (int16_t)((b[o + 1] << 8) | b[o]);
}

// ===============================
// BMI088 init
// ===============================
static void init_bmi088(int fd) {
    // ---- accel ----
    i2c_set_slave(fd, BMI088_ACC_ADDR);
    if (i2c_read8(fd, ACC_CHIP_ID) != ACC_CHIP_ID_VAL) {
        fprintf(stderr, "BMI088 accel WHO_AM_I mismatch\n");
        exit(1);
    }

    i2c_write8(fd, ACC_PWR_CTRL, 0x04); // active
    i2c_write8(fd, ACC_PWR_CONF, 0x00); // normal
    switch(SAMPLING_RATE){
    case 100:
        i2c_write8(fd, ACC_CONF, 0xA8);
        break;
    case 200:
        i2c_write8(fd, ACC_CONF, 0xA9);
        break;
    case 400:
        i2c_write8(fd, ACC_CONF, 0xAA);
        break;
    case 800:
        i2c_write8(fd, ACC_CONF, 0xAB);
        break;
    case 1600:
        i2c_write8(fd, ACC_CONF, 0xAC);
        break;
    }
    i2c_write8(fd, ACC_RANGE,    0x00); // ±3g

    // ---- gyro ----
    i2c_set_slave(fd, BMI088_GYRO_ADDR);
    if (i2c_read8(fd, GYRO_CHIP_ID) != GYRO_CHIP_ID_VAL) {
        fprintf(stderr, "BMI088 gyro WHO_AM_I mismatch\n");
        exit(1);
    }

    i2c_write8(fd, GYRO_RANGE, 0x00); // ±2000 dps
    i2c_write8(fd, GYRO_BW,    0x02); // 400Hz
    i2c_write8(fd, GYRO_LPM1,  0x00); // normal

    printf("BMI088 initialized\n");
}

// ===============================
// main
// ===============================
int main() {
    int fd = open("/dev/i2c-7", O_RDWR);
    if (fd < 0) {
        perror("open /dev/i2c-7");
        return 1;
    }

    init_bmi088(fd);

    redisContext *redis = redisConnect("127.0.0.1", 6379);
    if (!redis || redis->err) {
        fprintf(stderr, "Redis connection failed\n");
        return 1;
    }

    const int n_block = 8;
    int count = 0;

    double ax=0, ay=0, az=0;
    double gx=0, gy=0, gz=0;

    uint64_t time_sum_us = 0;

    while (1) {
        uint8_t acc_buf[6];
        uint8_t gyro_buf[6];

        i2c_set_slave(fd, BMI088_ACC_ADDR);
        i2c_read_block(fd, ACC_X_LSB, acc_buf, 6);

        i2c_set_slave(fd, BMI088_GYRO_ADDR);
        i2c_read_block(fd, GYRO_X_LSB, gyro_buf, 6);

        double lax = i16(acc_buf,0) * ACCEL_SCALE;
        double lay = i16(acc_buf,2) * ACCEL_SCALE;
        double laz = i16(acc_buf,4) * ACCEL_SCALE;

        double lgx = i16(gyro_buf,0) * GYRO_SCALE;
        double lgy = i16(gyro_buf,2) * GYRO_SCALE;
        double lgz = i16(gyro_buf,4) * GYRO_SCALE;

        struct timeval tv;
        gettimeofday(&tv, nullptr);
        uint64_t now_us = (uint64_t)tv.tv_sec * 1000000ull + tv.tv_usec;

        count++;
        if (count % n_block == 1 || n_block == 1) {
            ax=lax; ay=lay; az=laz;
            gx=lgx; gy=lgy; gz=lgz;
            time_sum_us = now_us;
        } else {
            ax+=lax; ay+=lay; az+=laz;
            gx+=lgx; gy+=lgy; gz+=lgz;
            time_sum_us += now_us;
        }

        if (count % n_block == 0) {
            ax /= n_block;
            ay /= n_block;
            az /= n_block;
            gx /= n_block;
            gy /= n_block;
            gz /= n_block;

            uint64_t avg_us = time_sum_us / n_block;
            uint32_t sec  = avg_us / 1000000ull;
            uint32_t nsec = (avg_us % 1000000ull) * 1000;

            double norm = std::sqrt(ax*ax + ay*ay + az*az);
            printf("%u %u %f\n", sec, nsec, norm);

            json j;
            j["timestamp"]["sec"]     = sec;
            j["timestamp"]["nanosec"] = nsec;

            j["accel"]["x"] = ax;
            j["accel"]["y"] = ay;
            j["accel"]["z"] = az;

            j["gyro"]["x"] = gx;
            j["gyro"]["y"] = gy;
            j["gyro"]["z"] = gz;

            std::string payload = j.dump(2); // compact JSON
            redisCommand(redis, "PUBLISH pserver-imu %s", payload.c_str());
        }

        usleep(1000*1000/SAMPLING_RATE);
    }
}

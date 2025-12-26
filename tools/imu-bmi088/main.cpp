#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <string>
#include <atomic>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/i2c-dev.h>

#include <hiredis/hiredis.h>
#include <nlohmann/json.hpp>

using json = nlohmann::ordered_json;

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

// ===============================
// Sampling rates
// ===============================
#define SAMPLING_RATE     100
#define SAMPLING_RATE_ACT 100

// ===============================
// Configuration
// ===============================
constexpr size_t QUEUE_MAX = 2;                 // Maximum number of queued messages

// ===============================
// Thread-safe bounded queue
// ===============================
std::mutex queue_mutex;
std::condition_variable queue_cv;
std::deque<std::string> message_queue;

std::atomic<bool> running{true};

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
static void init_bmi088(int fd_acc, int fd_gyro) {
    // ---- accel ----
    i2c_set_slave(fd_acc, BMI088_ACC_ADDR);
    if (i2c_read8(fd_acc, ACC_CHIP_ID) != ACC_CHIP_ID_VAL) {
        fprintf(stderr, "BMI088 accel WHO_AM_I mismatch\n");
        exit(1);
    }

    i2c_write8(fd_acc, ACC_PWR_CTRL, 0x04); // active
    i2c_write8(fd_acc, ACC_PWR_CONF, 0x00); // normal
    switch(SAMPLING_RATE_ACT){
    case 100:
        i2c_write8(fd_acc, ACC_CONF, 0xA8);
        break;
    case 200:
        i2c_write8(fd_acc, ACC_CONF, 0xA9);
        break;
    case 400:
        i2c_write8(fd_acc, ACC_CONF, 0xAA);
        break;
    case 800:
        i2c_write8(fd_acc, ACC_CONF, 0xAB);
        break;
    case 1600:
        i2c_write8(fd_acc, ACC_CONF, 0xAC);
        break;
    }
    i2c_write8(fd_acc, ACC_RANGE,    0x00); // ±3g

    // ---- gyro ----
    i2c_set_slave(fd_gyro, BMI088_GYRO_ADDR);
    if (i2c_read8(fd_gyro, GYRO_CHIP_ID) != GYRO_CHIP_ID_VAL) {
        fprintf(stderr, "BMI088 gyro WHO_AM_I mismatch\n");
        exit(1);
    }

    i2c_write8(fd_gyro, GYRO_RANGE, 0x00); // ±2000 dps
    i2c_write8(fd_gyro, GYRO_BW,    0x02); // 400Hz
    i2c_write8(fd_gyro, GYRO_LPM1,  0x00); // normal

    printf("BMI088 initialized\n");
}

// ===============================
// Redis publishing thread
// ===============================
void redis_thread_func() {
    // Create Redis connection (blocking context)
    redisContext* redis = redisConnect("127.0.0.1", 6379);
    if (!redis || redis->err) {
        fprintf(stderr, "Failed to connect to Redis\n");
        return;
    }

    while (running) {
        std::string payload;

        // Wait until a message is available or shutdown is requested
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            queue_cv.wait(lock, [] {
                return !message_queue.empty() || !running;
            });

            if (!running) {
                break;
            }

            payload = std::move(message_queue.front());
            message_queue.pop_front();
        }

        // Blocking publish (safe because this thread is dedicated to Redis)
        redisCommand(
            redis,
            "PUBLISH pserver-imu %b",
            payload.data(),
            payload.size()
        );
    }

    redisFree(redis);
}

// ===============================
// main
// ===============================
int main() {
    int fd_acc  = open("/dev/i2c-7", O_RDWR);
    if (fd_acc < 0) {
        perror("open /dev/i2c-7");
        return 1;
    }
    int fd_gyro = open("/dev/i2c-7", O_RDWR);
    if (fd_gyro < 0) {
        perror("open /dev/i2c-7");
        return 1;
    }

    init_bmi088(fd_acc, fd_gyro);

    // Start Redis publishing thread
    std::thread redis_thread(redis_thread_func);

    const int n_block = SAMPLING_RATE_ACT/SAMPLING_RATE;
    int count = 0;

    double ax=0, ay=0, az=0;
    double gx=0, gy=0, gz=0;

    uint64_t time_sum_us = 0;

    while (1) {
        struct timeval t0, t1;
        gettimeofday(&t0, nullptr);

        uint8_t acc_buf[6];
        uint8_t gyro_buf[6];

        i2c_read_block(fd_acc, ACC_X_LSB, acc_buf, 6);
        i2c_read_block(fd_gyro, GYRO_X_LSB, gyro_buf, 6);

        // {
        //     gettimeofday(&t1, nullptr);

        //     const int PERIOD_US = 1000*1000/SAMPLING_RATE_ACT;
        //     int elapsed_us =
        //         (t1.tv_sec - t0.tv_sec) * 1000000 +
        //         (t1.tv_usec - t0.tv_usec);
    
        //     int sleep_us = PERIOD_US - elapsed_us;
        //     if (sleep_us > 0) {
        //     }else{
        //         printf("!!!!!!!!!!!!!!!!!!loop process(%dus) exceed %dus\n", elapsed_us, PERIOD_US);
        //     }
        // }

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

            if((count%SAMPLING_RATE_ACT) == 0){
                double norm = std::sqrt(ax*ax + ay*ay + az*az);
                printf("%u %u %f\n", sec, nsec, norm);
                printf("%s\n", payload.c_str());
                fflush(stdout);
            }

            // Push payload into the bounded queue (non-blocking for IMU loop)
            {
                std::lock_guard<std::mutex> lock(queue_mutex);

                // Drop the oldest message if the queue is full
                if (message_queue.size() >= QUEUE_MAX) {
                    message_queue.pop_front();
                }

                message_queue.emplace_back(std::move(payload));
            }
            queue_cv.notify_one();
        }

        gettimeofday(&t1, nullptr);

        const int PERIOD_US = 1000*1000/SAMPLING_RATE_ACT;
        int elapsed_us =
            (t1.tv_sec - t0.tv_sec) * 1000000 +
            (t1.tv_usec - t0.tv_usec);

        int sleep_us = PERIOD_US - elapsed_us;
        if (sleep_us > 0) {
            usleep(sleep_us);
        }else{
            printf("loop process(%dus) exceed %dus\n", elapsed_us, PERIOD_US);
        }
    }
    
    // Shutdown
    queue_cv.notify_all();
    redis_thread.join();
    return 0;
}

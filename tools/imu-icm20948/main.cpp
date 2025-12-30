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
// I2C addresses (ICM-20948)
// ===============================
// AD0=0 -> 0x68, AD0=1 -> 0x69
#define ICM20948_ADDR     0x68

// ===============================
// ICM-20948 register map (banked)
// ===============================
#define REG_BANK_SEL      0x7F

// Bank 0
#define WHO_AM_I          0x00
#define WHO_AM_I_VAL      0xEA

#define PWR_MGMT_1        0x06
#define PWR_MGMT_2        0x07

// Data registers (Bank 0) : contiguous burst read
// ACCEL_XOUT_H .. ACCEL_ZOUT_L (6 bytes)
// GYRO_XOUT_H  .. GYRO_ZOUT_L  (6 bytes)
// TEMP_OUT_H   .. TEMP_OUT_L   (2 bytes)
// Total 14 bytes starting at 0x2D
#define ACCEL_XOUT_H      0x2D

// Bank 2 (config)
#define GYRO_SMPLRT_DIV   0x00
#define GYRO_CONFIG_1     0x01
#define GYRO_CONFIG_2     0x02

#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG      0x14
#define ACCEL_CONFIG_2    0x15

// ===============================
// Sampling rates
// ===============================
#define SAMPLING_RATE     100
#define SAMPLING_RATE_ACT 100

// ===============================
// Configuration
// ===============================
constexpr size_t QUEUE_MAX = 2; // Maximum number of queued messages

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
// ICM-20948 accel sensitivity (LSB/g):
//  ±2g: 16384, ±4g: 8192, ±8g: 4096, ±16g: 2048
// ここでは ±4g (8192 LSB/g) に設定
const double ACCEL_SCALE = 9.80665 / 8192.0;

// ICM-20948 gyro sensitivity (LSB/dps):
//  ±250: 131, ±500: 65.5, ±1000: 32.8, ±2000: 16.4
// ここでは ±2000 dps (16.4 LSB/dps) に設定
const double GYRO_SCALE  = (M_PI / 180.0) / 16.4;

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
    if (write(fd, &reg, 1) != 1) {
        perror("i2c_write(reg)");
        exit(1);
    }
    uint8_t v;
    if (read(fd, &v, 1) != 1) {
        perror("i2c_read8");
        exit(1);
    }
    return v;
}

static void i2c_write8(int fd, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (write(fd, buf, 2) != 2) {
        perror("i2c_write8");
        exit(1);
    }
}

static void i2c_read_block(int fd, uint8_t reg, uint8_t *buf, size_t len) {
    if (write(fd, &reg, 1) != 1) {
        perror("i2c_write(reg)");
        exit(1);
    }
    ssize_t r = read(fd, buf, len);
    if (r != (ssize_t)len) {
        perror("i2c_read_block");
        exit(1);
    }
}

static inline int16_t be_i16(const uint8_t *b, int o) {
    // ICM-20948 data registers are big-endian (H then L)
    return (int16_t)((b[o] << 8) | b[o + 1]);
}

// ===============================
// ICM-20948 bank select
// ===============================
static void icm_set_bank(int fd, uint8_t bank) {
    // bank: 0..3
    i2c_write8(fd, REG_BANK_SEL, (uint8_t)(bank << 4));
}

// ===============================
// ICM-20948 init
// ===============================
static void init_icm20948(int fd) {
    i2c_set_slave(fd, ICM20948_ADDR);

    icm_set_bank(fd, 0);
    uint8_t who = i2c_read8(fd, WHO_AM_I);
    if (who != WHO_AM_I_VAL) {
        fprintf(stderr, "ICM-20948 WHO_AM_I mismatch: 0x%02X (expected 0x%02X)\n", who, WHO_AM_I_VAL);
        exit(1);
    }

    // Reset device
    // PWR_MGMT_1: DEVICE_RESET (bit7)
    i2c_write8(fd, PWR_MGMT_1, 0x80);
    usleep(100 * 1000);

    // Wake up, set clock source (best available, e.g. auto selects)
    // PWR_MGMT_1:
    //  bit6 SLEEP=0
    //  bits[2:0] CLKSEL=1 (Auto best clock is typical; many implementations use 1)
    i2c_write8(fd, PWR_MGMT_1, 0x01);
    usleep(10 * 1000);

    // Enable accel + gyro (clear disables in PWR_MGMT_2)
    // 0x00 = all enabled
    i2c_write8(fd, PWR_MGMT_2, 0x00);
    usleep(10 * 1000);

    // ---------- configure sample rate & filters ----------
    // ICM-20948 typical internal ODR (with DLPF enabled) is 1125 Hz for both gyro/accel.
    // OutputRate = 1125 / (1 + DIV)
    auto calc_div_1125 = [](int target_hz) -> uint16_t {
        if (target_hz <= 0) target_hz = 1;
        int div = (int)std::lround((1125.0 / (double)target_hz) - 1.0);
        if (div < 0) div = 0;
        if (div > 0xFFFF) div = 0xFFFF;
        return (uint16_t)div;
    };

    uint16_t div = calc_div_1125(SAMPLING_RATE_ACT);

    // Bank 2 for gyro/accel configs
    icm_set_bank(fd, 2);

    // Gyro sample rate divider (8-bit)
    // GYRO_SMPLRT_DIV is 8-bit in many docs; clamp
    uint8_t gdiv = (div > 255) ? 255 : (uint8_t)div;
    i2c_write8(fd, GYRO_SMPLRT_DIV, gdiv);

    // Gyro config:
    // GYRO_CONFIG_1:
    //  bits[2:0]  GYRO_DLPFCFG
    //  bit3       GYRO_FCHOICE (0 = enable DLPF)
    //  bits[6:5]  GYRO_FS_SEL (00=250, 01=500, 10=1000, 11=2000 dps)
    //  bit7       reserved
    //
    // ここでは:
    //  FS_SEL=11 (±2000dps)
    //  FCHOICE=0 (DLPF on)
    //  DLPFCFG=3 (例: 適度なLPF)
    uint8_t gyro_fs_sel = 0x03;   // 11b
    uint8_t gyro_dlpf   = 0x03;   // 好みで調整
    uint8_t gyro_cfg1   = (uint8_t)((gyro_fs_sel << 1) << 4); // ざっくり: FSは[6:5]に入れたい
    // 正確に: FS_SEL を [2 bits] で [6:5] へ
    gyro_cfg1 = (uint8_t)((gyro_fs_sel & 0x03) << 5);         // FS_SEL
    gyro_cfg1 |= (0 << 3);                                    // FCHOICE=0 (DLPF on)
    gyro_cfg1 |= (gyro_dlpf & 0x07);                          // DLPFCFG
    i2c_write8(fd, GYRO_CONFIG_1, gyro_cfg1);

    // Gyro config 2: leave default (or set averaging if desired)
    i2c_write8(fd, GYRO_CONFIG_2, 0x00);

    // Accel sample rate divider is 12-bit split into two regs (DIV[11:0])
    // ACCEL_SMPLRT_DIV_1: high, _2: low
    uint16_t adiv = div; // reuse
    i2c_write8(fd, ACCEL_SMPLRT_DIV_1, (uint8_t)((adiv >> 8) & 0x0F));
    i2c_write8(fd, ACCEL_SMPLRT_DIV_2, (uint8_t)(adiv & 0xFF));

    // Accel config:
    // ACCEL_CONFIG:
    //  bits[6:5] ACCEL_FS_SEL (00=2g,01=4g,10=8g,11=16g)
    //  bit3      ACCEL_FCHOICE (0=DLPF on)
    //  bits[2:0] ACCEL_DLPFCFG
    //
    // ここでは:
    //  FS_SEL=01 (±4g)
    //  FCHOICE=0 (DLPF on)
    //  DLPFCFG=3 (例)
    uint8_t accel_fs_sel = 0x01; // ±4g
    uint8_t accel_dlpf   = 0x03;
    uint8_t accel_cfg = (uint8_t)((accel_fs_sel & 0x03) << 5);
    accel_cfg |= (0 << 3);
    accel_cfg |= (accel_dlpf & 0x07);
    i2c_write8(fd, ACCEL_CONFIG, accel_cfg);

    // ACCEL_CONFIG_2: leave default (or set averaging)
    i2c_write8(fd, ACCEL_CONFIG_2, 0x00);

    // Back to Bank 0 for reading
    icm_set_bank(fd, 0);

    printf("ICM-20948 initialized (addr=0x%02X, rate=%dHz)\n", ICM20948_ADDR, SAMPLING_RATE_ACT);
}

// ===============================
// Redis publishing thread
// ===============================
[[noreturn]] static void redis_fatal(const char* msg, redisContext* c = nullptr) {
    fprintf(stderr, "Redis fatal: %s\n", msg);
    if (c && c->err) {
        fprintf(stderr, "Redis error: %s\n", c->errstr);
    }
    std::abort(); // 即プロセス終了
}

void redis_thread_func() {
    redisContext* redis = redisConnect("127.0.0.1", 6379);
    if (!redis || redis->err) {
        redis_fatal("connect failed", redis);
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

        redisReply* reply = (redisReply*)redisCommand(
            redis,
            "PUBLISH pserver-imu %b",
            payload.data(),
            payload.size()
        );

        if (!reply) {
            redis_fatal("redisCommand returned NULL", redis);
        }

        freeReplyObject(reply);

        if (redis->err) {
            redis_fatal("redis runtime error", redis);
        }
    }

    redisFree(redis);
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

    init_icm20948(fd);

    std::thread redis_thread(redis_thread_func);

    const int n_block = SAMPLING_RATE_ACT / SAMPLING_RATE;
    int count = 0;

    double ax=0, ay=0, az=0;
    double gx=0, gy=0, gz=0;

    uint64_t time_sum_us = 0;

    while (1) {
        struct timeval t0, t1;
        gettimeofday(&t0, nullptr);

        uint8_t buf[14];
        i2c_read_block(fd, ACCEL_XOUT_H, buf, sizeof(buf));

        // layout:
        // 0..5  : accel X/Y/Z (H,L)
        // 6..11 : gyro  X/Y/Z (H,L)
        // 12..13: temp  (H,L)
        int16_t raw_ax = be_i16(buf, 0);
        int16_t raw_ay = be_i16(buf, 2);
        int16_t raw_az = be_i16(buf, 4);

        int16_t raw_gx = be_i16(buf, 6);
        int16_t raw_gy = be_i16(buf, 8);
        int16_t raw_gz = be_i16(buf,10);

        // int16_t raw_temp = be_i16(buf,12); // 必要なら使う

        double lax = (double)raw_ax * ACCEL_SCALE;
        double lay = (double)raw_ay * ACCEL_SCALE;
        double laz = (double)raw_az * ACCEL_SCALE;

        double lgx = (double)raw_gx * GYRO_SCALE;
        double lgy = (double)raw_gy * GYRO_SCALE;
        double lgz = (double)raw_gz * GYRO_SCALE;

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
            uint32_t sec  = (uint32_t)(avg_us / 1000000ull);
            uint32_t nsec = (uint32_t)((avg_us % 1000000ull) * 1000ull);

            json j;
            j["timestamp"]["sec"]     = sec;
            j["timestamp"]["nanosec"] = nsec;

            j["accel"]["x"] = ax;
            j["accel"]["y"] = ay;
            j["accel"]["z"] = az;

            j["gyro"]["x"] = gx;
            j["gyro"]["y"] = gy;
            j["gyro"]["z"] = gz;

            std::string payload = j.dump(2);

            if ((count % SAMPLING_RATE_ACT) == 0) {
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

        const int PERIOD_US = 1000*1000 / SAMPLING_RATE_ACT;
        int elapsed_us =
            (t1.tv_sec - t0.tv_sec) * 1000000 +
            (t1.tv_usec - t0.tv_usec);

        int sleep_us = PERIOD_US - elapsed_us;
        if (sleep_us > 0) {
            usleep(sleep_us);
        } else {
            printf("loop process(%dus) exceed %dus\n", elapsed_us, PERIOD_US);
        }
    }

    running = false;
    queue_cv.notify_all();
    redis_thread.join();
    return 0;
}

// basalt_redis_vio.cpp
//
// Redis PubSub から
//  - stereo 512x512 (OpenCV Mat)
//  - IMU acc xyz, gyro xyz
// を受け取り、Basalt VIO に投入し、推定 pose を Redis に publish する。
//
// 注意: Basalt API はビルド構成/バージョンでクラス名や初期化手順が変わるため、
//       "BasaltBridge" クラス内の 3 関数だけあなたの環境に合わせて差し替える設計。
//       (Redis/OpenCV/同期/スレッド/キューはこのまま実用)
//

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <hiredis/hiredis.h>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ===============================
// Base64 decode (minimal)
// ===============================
static inline int b64_index(unsigned char c) {
  if ('A' <= c && c <= 'Z') return c - 'A';
  if ('a' <= c && c <= 'z') return c - 'a' + 26;
  if ('0' <= c && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  return -1;
}

static bool base64_decode(const std::string& in, std::vector<uint8_t>& out) {
  out.clear();
  out.reserve(in.size() * 3 / 4);

  int val = 0;
  int valb = -8;
  for (unsigned char c : in) {
    if (c == '=') break;
    int idx = b64_index(c);
    if (idx < 0) {
      if (c == '\n' || c == '\r' || c == ' ' || c == '\t') continue;
      return false;
    }
    val = (val << 6) + idx;
    valb += 6;
    if (valb >= 0) {
      out.push_back((uint8_t)((val >> valb) & 0xFF));
      valb -= 8;
    }
  }
  return true;
}

// ===============================
// Data types
// ===============================
struct ImuSample {
  int64_t t_ns = 0;
  double acc[3]{0, 0, 0};   // m/s^2
  double gyro[3]{0, 0, 0};  // rad/s
};

struct StereoFrame {
  int64_t t_ns = 0;
  cv::Mat left;   // CV_8UC1 recommended
  cv::Mat right;  // CV_8UC1 recommended
};

struct PoseOut {
  int64_t t_ns = 0;
  double p[3]{0, 0, 0};       // x y z
  double q[4]{1, 0, 0, 0};    // qw qx qy qz
  std::string status = "INIT";
};

// ===============================
// Thread-safe queue
// ===============================
template <typename T>
class TSQueue {
 public:
  void push(T&& v) {
    {
      std::lock_guard<std::mutex> lk(m_);
      q_.push_back(std::move(v));
    }
    cv_.notify_one();
  }

  std::optional<T> pop_wait(std::atomic<bool>& running, int timeout_ms) {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] {
      return !q_.empty() || !running.load();
    });

    if (!running.load()) return std::nullopt;
    if (q_.empty()) return std::nullopt;

    T v = std::move(q_.front());
    q_.pop_front();
    return v;
  }

 private:
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<T> q_;
};

// ===============================
// Redis client (hiredis)
// ===============================
class RedisClient {
 public:
  RedisClient(std::string host, int port) : host_(std::move(host)), port_(port) {}

  bool connect_pub() {
    pub_ = redisConnect(host_.c_str(), port_);
    if (!pub_ || pub_->err) {
      std::cerr << "[redis] pub connect error: " << (pub_ ? pub_->errstr : "null") << "\n";
      if (pub_) { redisFree(pub_); pub_ = nullptr; }
      return false;
    }
    return true;
  }

  bool connect_sub() {
    sub_ = redisConnect(host_.c_str(), port_);
    if (!sub_ || sub_->err) {
      std::cerr << "[redis] sub connect error: " << (sub_ ? sub_->errstr : "null") << "\n";
      if (sub_) { redisFree(sub_); sub_ = nullptr; }
      return false;
    }
    return true;
  }

  bool publish(const std::string& channel, const std::string& payload) {
    if (!pub_) return false;
    redisReply* r = (redisReply*)redisCommand(pub_, "PUBLISH %s %b",
                                             channel.c_str(),
                                             payload.data(), payload.size());
    if (!r) return false;
    freeReplyObject(r);
    return true;
  }

  bool subscribe(const std::string& channel) {
    if (!sub_) return false;
    redisReply* r = (redisReply*)redisCommand(sub_, "SUBSCRIBE %s", channel.c_str());
    if (!r) return false;
    freeReplyObject(r);
    return true;
  }

  bool recv_message(std::string& out_channel, std::string& out_payload) {
    if (!sub_) return false;

    void* reply = nullptr;
    if (redisGetReply(sub_, &reply) != REDIS_OK || !reply) return false;

    redisReply* r = (redisReply*)reply;
    // pubsub message: ["message", channel, payload]
    if (r->type == REDIS_REPLY_ARRAY && r->elements >= 3 &&
        r->element[0]->type == REDIS_REPLY_STRING &&
        std::string(r->element[0]->str, r->element[0]->len) == "message") {
      out_channel = std::string(r->element[1]->str, r->element[1]->len);
      out_payload = std::string(r->element[2]->str, r->element[2]->len);
    }
    freeReplyObject(r);
    return true;
  }

  ~RedisClient() {
    if (pub_) redisFree(pub_);
    if (sub_) redisFree(sub_);
  }

 private:
  std::string host_;
  int port_;
  redisContext* pub_{nullptr};
  redisContext* sub_{nullptr};
};

// ===============================
// JSON decode
// ===============================
static bool decode_stereo_frame(const std::string& payload, StereoFrame& out) {
  json j;
  try { j = json::parse(payload); } catch (...) { return false; }

  if (!j.contains("t_ns") || !j.contains("left_b64") || !j.contains("right_b64")) return false;

  out.t_ns = j["t_ns"].get<int64_t>();

  std::vector<uint8_t> lbin, rbin;
  if (!base64_decode(j["left_b64"].get<std::string>(), lbin)) return false;
  if (!base64_decode(j["right_b64"].get<std::string>(), rbin)) return false;

  cv::Mat lbuf(1, (int)lbin.size(), CV_8UC1, lbin.data());
  cv::Mat rbuf(1, (int)rbin.size(), CV_8UC1, rbin.data());

  cv::Mat left = cv::imdecode(lbuf, cv::IMREAD_GRAYSCALE);
  cv::Mat right = cv::imdecode(rbuf, cv::IMREAD_GRAYSCALE);
  if (left.empty() || right.empty()) return false;

  if (left.rows != 512 || left.cols != 512) cv::resize(left, left, {512, 512}, 0, 0, cv::INTER_AREA);
  if (right.rows != 512 || right.cols != 512) cv::resize(right, right, {512, 512}, 0, 0, cv::INTER_AREA);

  out.left = left;
  out.right = right;
  return true;
}

static bool decode_imu_sample(const std::string& payload, ImuSample& out) {
  json j;
  try { j = json::parse(payload); } catch (...) { return false; }

  if (!j.contains("t_ns") || !j.contains("acc") || !j.contains("gyro")) return false;

  out.t_ns = j["t_ns"].get<int64_t>();

  auto acc = j["acc"];
  auto gyro = j["gyro"];
  if (!acc.is_array() || acc.size() != 3) return false;
  if (!gyro.is_array() || gyro.size() != 3) return false;

  for (int i = 0; i < 3; i++) out.acc[i] = acc[i].get<double>();
  for (int i = 0; i < 3; i++) out.gyro[i] = gyro[i].get<double>();
  return true;
}

// ===============================
// Basalt bridge (差し替えポイント)
// ===============================
class BasaltBridge {
 public:
  struct Config {
    std::string basalt_config_path;  // Basalt VIO config (yaml/toml/json etc)
    std::string calib_path;          // camera/imu calibration file
  };

  bool init(const Config& cfg) {
    // ======= ここが vio.cpp 相当の初期化（あなたの Basalt ビルドに合わせて実装） =======
    // 例: vio.cpp では config/calib を読み、推定器を構築し、内部スレッドや状態を初期化します。
    //      (入力: IMU + stereo frame)
    //
    // TODO:
    //  - Basalt の config を読み込む
    //  - calibration を読み込む
    //  - estimator を生成
    //
    // ここでは“初期化成功”としてダミーを返します（実動作には下の3関数を実装してください）。
    (void)cfg;
    initialized_ = true;
    last_.status = "INIT_OK";
    return true;
  }

  // IMU を投入（t_ns: 単調増加が望ましい）
  void feedImu(const ImuSample& s) {
    if (!initialized_) return;

    // ======= ここが vio.cpp の IMU 取り込み部分に対応 =======
    // Basalt は多くの場合:
    //   timestamp(sec) + gyro(rad/s) + acc(m/s^2)
    // を push します。
    //
    // TODO: あなたの Basalt API に合わせて実装
    (void)s;
  }

  // Stereo frame を投入し、推定 pose が更新されたら true
  bool feedStereoAndGetPose(const StereoFrame& f, PoseOut& out) {
    if (!initialized_) return false;

    // ======= ここが vio.cpp の frame 処理に対応 =======
    // 典型:
    //  - left/right を Basalt の image 型に変換
    //  - timestamp を与えて処理を進める
    //  - 最新状態 (T_WI 等) を取得
    //
    // TODO: あなたの Basalt API に合わせて実装

    // ---- ダミー実装（とりあえず流れ確認用）----
    last_.t_ns = f.t_ns;
    last_.status = "OK_DUMMY";
    out = last_;
    return true;
  }

 private:
  bool initialized_{false};
  PoseOut last_{};
};

// ===============================
// App config
// ===============================
struct AppConfig {
  std::string redis_host = "127.0.0.1";
  int redis_port = 6379;

  std::string ch_stereo = "stereo512";
  std::string ch_imu = "imu";
  std::string ch_pose = "vio_pose";

  // Basalt
  std::string basalt_config_path = "./basalt_config.yaml";
  std::string calib_path = "./calib.json";

  // Sync policy
  int64_t max_imu_buffer_ns = 500'000'000;  // 0.5s
};

static std::string pose_to_json(const PoseOut& p) {
  json j;
  j["t_ns"] = p.t_ns;
  j["p"] = {p.p[0], p.p[1], p.p[2]};
  j["q"] = {p.q[0], p.q[1], p.q[2], p.q[3]};
  j["status"] = p.status;
  return j.dump();
}

// ===============================
// main
// ===============================
int main(int argc, char** argv) {
  (void)argc; (void)argv;

  AppConfig cfg;

  RedisClient redis(cfg.redis_host, cfg.redis_port);
  if (!redis.connect_pub()) return 1;
  if (!redis.connect_sub()) return 1;

  if (!redis.subscribe(cfg.ch_stereo)) return 1;
  if (!redis.subscribe(cfg.ch_imu)) return 1;

  TSQueue<StereoFrame> stereo_q;
  TSQueue<ImuSample> imu_q;

  std::atomic<bool> running{true};

  // Subscriber thread
  std::thread th_sub([&] {
    std::string ch, payload;
    while (running.load()) {
      if (!redis.recv_message(ch, payload)) {
        std::cerr << "[redis] recv error\n";
        running.store(false);
        break;
      }

      if (ch == cfg.ch_stereo) {
        StereoFrame f;
        if (decode_stereo_frame(payload, f)) {
          stereo_q.push(std::move(f));
        } else {
          std::cerr << "[stereo] decode failed\n";
        }
      } else if (ch == cfg.ch_imu) {
        ImuSample s;
        if (decode_imu_sample(payload, s)) {
          imu_q.push(std::move(s));
        } else {
          std::cerr << "[imu] decode failed\n";
        }
      }
    }
  });

  // Basalt init
  BasaltBridge vio;
  BasaltBridge::Config bcfg;
  bcfg.basalt_config_path = cfg.basalt_config_path;
  bcfg.calib_path = cfg.calib_path;

  if (!vio.init(bcfg)) {
    std::cerr << "[basalt] init failed\n";
    running.store(false);
    th_sub.join();
    return 1;
  }

  std::deque<ImuSample> imu_buf;

  auto drain_imu = [&] {
    while (true) {
      auto sopt = imu_q.pop_wait(running, 0);
      if (!sopt.has_value()) break;
      imu_buf.push_back(std::move(*sopt));
    }
  };

  while (running.load()) {
    auto fopt = stereo_q.pop_wait(running, 50);
    if (!running.load()) break;

    // idle: just keep IMU buffer small
    if (!fopt.has_value()) {
      drain_imu();
      continue;
    }

    StereoFrame frame = std::move(*fopt);

    // drain IMU
    drain_imu();

    // drop too old IMU samples (to avoid unbounded memory)
    while (!imu_buf.empty() && (frame.t_ns - imu_buf.front().t_ns) > cfg.max_imu_buffer_ns) {
      imu_buf.pop_front();
    }

    // feed IMU up to frame timestamp
    while (!imu_buf.empty() && imu_buf.front().t_ns <= frame.t_ns) {
      vio.feedImu(imu_buf.front());
      imu_buf.pop_front();
    }

    // feed stereo & get pose
    PoseOut pose;
    if (vio.feedStereoAndGetPose(frame, pose)) {
      const std::string msg = pose_to_json(pose);
      if (!redis.publish(cfg.ch_pose, msg)) {
        std::cerr << "[redis] publish failed\n";
      }
    }
  }

  running.store(false);
  if (th_sub.joinable()) th_sub.join();
  return 0;
}

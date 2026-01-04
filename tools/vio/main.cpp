/*
 * Live Basalt VIO from Redis
 *
 * Based on basalt/src/vio.cpp structure
 * Dataset IO is replaced by Redis stereo + IMU input
 */

#include <atomic>
#include <thread>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <iostream>

#include <hiredis/hiredis.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/calibration/calibration.hpp>
#include <basalt/utils/time_utils.hpp>
#include <basalt/serialization/headers_serialization.h>

using json = nlohmann::json;

// ============================================================
// Redis config
// ============================================================
static const char *REDIS_HOST = "127.0.0.1";
static const int REDIS_PORT = 6379;

static const char *CH_STEREO = "stereo512";
static const char *CH_IMU = "pserver-imu";
static const char *CH_POSE = "vio_pose";

// ============================================================
// Thread-safe queue
// ============================================================
template <typename T>
class TSQueue
{
public:
  void push(T &&v)
  {
    std::lock_guard<std::mutex> lk(m);
    q.push_back(std::move(v));
    cv.notify_one();
  }

  bool pop(T &out)
  {
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, [&]
            { return !q.empty() || terminate; });
    if (q.empty())
      return false;
    out = std::move(q.front());
    q.pop_front();
    return true;
  }

  void stop()
  {
    terminate = true;
    cv.notify_all();
  }

private:
  std::deque<T> q;
  std::mutex m;
  std::condition_variable cv;
  bool terminate = false;
};

// ============================================================
// Data types
// ============================================================
struct StereoMsg
{
  int64_t t_ns;
  cv::Mat left;
  cv::Mat right;
};

struct ImuMsg
{
  int64_t t_ns;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
};

// ============================================================
// Redis helpers
// ============================================================
redisContext *redis_sub(const char *ch)
{
  redisContext *c = redisConnect(REDIS_HOST, REDIS_PORT);
  if (!c || c->err)
  {
    std::cerr << "Redis connect error\n";
    std::exit(1);
  }
  redisCommand(c, "SUBSCRIBE %s", ch);
  return c;
}

void redis_publish_pose(const Sophus::SE3d &T_w_i, int64_t t_ns)
{
  redisContext *c = redisConnect(REDIS_HOST, REDIS_PORT);
  if (!c)
    return;

  json j;
  j["t_ns"] = t_ns;
  j["p"] = {T_w_i.translation().x(),
            T_w_i.translation().y(),
            T_w_i.translation().z()};
  j["q"] = {T_w_i.unit_quaternion().w(),
            T_w_i.unit_quaternion().x(),
            T_w_i.unit_quaternion().y(),
            T_w_i.unit_quaternion().z()};

  std::string s = j.dump();
  redisCommand(c, "PUBLISH %s %b", CH_POSE, s.data(), s.size());
  redisFree(c);
}

// ============================================================
// Globals
// ============================================================
TSQueue<StereoMsg> stereo_q;
TSQueue<ImuMsg> imu_q;

basalt::Calibration<double> calib;
basalt::VioConfig vio_config;
basalt::OpticalFlowBase::Ptr opt_flow;
basalt::VioEstimatorBase::Ptr vio;

// ============================================================
// Redis IMU thread (feed_imu equivalent)
// ============================================================
void imu_thread()
{
  redisContext *c = redis_sub(CH_IMU);
  redisReply *r;

  while (redisGetReply(c, (void **)&r) == REDIS_OK)
  {
    if (!r || r->type != REDIS_REPLY_ARRAY)
      continue;

    json j = json::parse(r->element[2]->str);

    ImuMsg m;
    const auto &ts = j["timestamp"];
    m.t_ns =
        static_cast<int64_t>(ts["sec"].get<int64_t>()) * 1000000000LL +
        static_cast<int64_t>(ts["nanosec"].get<int64_t>());

    // accel [m/s^2]
    const auto &acc = j["accel"];
    m.acc = Eigen::Vector3d(
        acc["x"].get<double>(),
        acc["y"].get<double>(),
        acc["z"].get<double>());

    // gyro [rad/s]
    const auto &gyro = j["gyro"];
    m.gyro = Eigen::Vector3d(
        gyro["x"].get<double>(),
        gyro["y"].get<double>(),
        gyro["z"].get<double>());

    imu_q.push(std::move(m));
    freeReplyObject(r);
  }
}

// Base64 index table
static inline int base64_index(unsigned char c)
{
  if (c >= 'A' && c <= 'Z')
    return c - 'A';
  if (c >= 'a' && c <= 'z')
    return c - 'a' + 26;
  if (c >= '0' && c <= '9')
    return c - '0' + 52;
  if (c == '+')
    return 62;
  if (c == '/')
    return 63;
  return -1;
}

// Decode base64 string to binary buffer
// return true on success
bool base64_decode(const std::string &input, std::vector<uint8_t> &output)
{
  output.clear();
  output.reserve(input.size() * 3 / 4);

  int val = 0;
  int valb = -8;

  for (unsigned char c : input)
  {
    if (c == '=')
      break;

    int idx = base64_index(c);
    if (idx < 0)
    {
      // ignore whitespace
      if (c == '\n' || c == '\r' || c == ' ' || c == '\t')
        continue;
      return false;
    }

    val = (val << 6) + idx;
    valb += 6;

    if (valb >= 0)
    {
      output.push_back(uint8_t((val >> valb) & 0xFF));
      valb -= 8;
    }
  }
  return true;
}

static cv::Mat decode_image_from_base64(const std::string &b64)
{
  std::vector<uint8_t> bin;
  if (!base64_decode(b64, bin))
  {
    return cv::Mat();
  }

  cv::Mat buf(1, bin.size(), CV_8UC1, bin.data());
  return cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
}

// ============================================================
// Redis stereo thread (feed_images equivalent)
// ============================================================
void stereo_thread()
{
  redisContext *c = redis_sub(CH_STEREO);
  redisReply *r;

  while (redisGetReply(c, (void **)&r) == REDIS_OK)
  {
    if (!r || r->type != REDIS_REPLY_ARRAY)
      continue;

    json j = json::parse(r->element[2]->str);

    StereoMsg s;
    s.t_ns = j["t_ns"];

    s.left = decode_image_from_base64(j["left"]);
    s.right = decode_image_from_base64(j["right"]);

    stereo_q.push(std::move(s));
    freeReplyObject(r);
  }
}

// ============================================================
// Main
// ============================================================
int main(int argc, char **argv)
{

  // ---- load calibration (same as vio.cpp) ----
  {
    std::ifstream is("calib.json");
    cereal::JSONInputArchive ar(is);
    ar(calib);
  }

  vio_config.load("vio_config.json");

  opt_flow = basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
  vio = basalt::VioEstimatorFactory::getVioEstimator(
      vio_config, calib, basalt::constants::g, true, true);

  vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  opt_flow->output_queue = &vio->vision_data_queue;

  std::thread t_imu(imu_thread);
  std::thread t_img(stereo_thread);

  // ---- Main processing loop ----
  while (true)
  {

    // IMU feed
    ImuMsg im;
    while (imu_q.pop(im))
    {
      basalt::ImuData<double>::Ptr d(new basalt::ImuData<double>);
      d->t_ns = im.t_ns;
      d->accel = im.acc;
      d->gyro = im.gyro;
      vio->imu_data_queue.push(d);
    }

    // Stereo feed
    StereoMsg sm;
    if (!stereo_q.pop(sm))
      continue;

    basalt::OpticalFlowInput::Ptr in(new basalt::OpticalFlowInput);
    in->t_ns = sm.t_ns;
    basalt::ImageData l, r;

    l.img.reset(new basalt::ManagedImage<uint16_t>(512, 512));
    r.img.reset(new basalt::ManagedImage<uint16_t>(512, 512));

    //  for (int y = 0; y < 512; y++) {
    //      const uint8_t* sl = s.left.ptr<uint8_t>(y);
    //      const uint8_t* sr = s.right.ptr<uint8_t>(y);

    //      uint16_t* dl = l.img->ptr + y * 512;
    //      uint16_t* dr = r.img->ptr + y * 512;

    //      for (int x = 0; x < 512; x++) {
    //          dl[x] = uint16_t(sl[x]) << 8;
    //          dr[x] = uint16_t(sr[x]) << 8;
    //      }
    //  }

    in->img_data = {l, r};

    std::memcpy(l.img->ptr, sm.left.data, 512 * 512);
    std::memcpy(r.img->ptr, sm.right.data, 512 * 512);

    in->img_data = {l, r};
    opt_flow->input_queue.push(in);

    // Get pose
    basalt::PoseVelBiasState<double>::Ptr st;
    if (vio->out_state_queue->pop(st))
    {
      redis_publish_pose(st->T_w_i, st->t_ns);
    }
  }

  t_imu.join();
  t_img.join();
  return 0;
}

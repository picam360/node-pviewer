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
#include <regex>

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

static const char *CH_STEREO = "pserver-forward-pst";
static const char *CH_IMU = "pserver-imu";
static const char *CH_POSE = "vio_pose";

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
basalt::Calibration<double> calib;
basalt::VioConfig vio_config;
basalt::OpticalFlowBase::Ptr opt_flow;
basalt::VioEstimatorBase::Ptr vio;
tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> out_state_queue;

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

        basalt::ImuData<double>::Ptr d(new basalt::ImuData<double>);

        const auto &ts = j["timestamp"];
        d->t_ns =
            static_cast<int64_t>(ts["sec"].get<int64_t>()) * 1000000000LL +
            static_cast<int64_t>(ts["nanosec"].get<int64_t>());

        const auto &acc = j["accel"];
        d->accel = Eigen::Vector3d(
            acc["x"].get<double>(),
            acc["y"].get<double>(),
            acc["z"].get<double>());

        const auto &gyro = j["gyro"];
        d->gyro = Eigen::Vector3d(
            gyro["x"].get<double>(),
            gyro["y"].get<double>(),
            gyro["z"].get<double>());

        vio->imu_data_queue.push(d);

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

bool parse_xml_timestamp(
    const std::vector<uint8_t> &xml_bytes,
    int64_t &sec,
    int64_t &nsec)
{
    if (xml_bytes.size() <= 4)
        return false;

    // Python: xml_bytes[4:].decode("utf-8")
    std::string xml(
        reinterpret_cast<const char *>(xml_bytes.data() + 4),
        xml_bytes.size() - 4);

    // remove namespace ( <ns:tag> -> <tag> )
    xml = std::regex_replace(xml, std::regex("</?\\w+:(\\w+)"), "<$1");

    // timestamp="sec,usec"
    std::smatch m;
    std::regex r("timestamp\\s*=\\s*\"(\\d+),(\\d+)\"");

    if (!std::regex_search(xml, m, r))
        return false;

    sec = std::stoll(m[1].str());
    nsec = std::stoll(m[2].str()) * 1000; // usec ? nsec
    return true;
}

bool split_stereo_image(
    const std::vector<uint8_t> &jpeg,
    cv::Mat &left,
    cv::Mat &right)
{
    cv::Mat buf(1, jpeg.size(), CV_8UC1,
                const_cast<uint8_t *>(jpeg.data()));

    cv::Mat img = cv::imdecode(buf, cv::IMREAD_COLOR);
    if (img.empty())
        return false;

    int h = img.rows;
    int w = img.cols;
    int half = w / 2;

    left = img(cv::Rect(0, 0, half, h)).clone();
    right = img(cv::Rect(half, 0, half, h)).clone();
    return true;
}

// ============================================================
// Redis stereo thread (feed_images equivalent)
// ============================================================
void stereo_thread()
{
    redisContext *c = redis_sub(CH_STEREO);
    redisReply *r;

    std::vector<std::vector<uint8_t>> tmp_img;

    while (redisGetReply(c, (void **)&r) == REDIS_OK)
    {
        if (!r || r->type != REDIS_REPLY_ARRAY)
        {
            if (r)
                freeReplyObject(r);
            continue;
        }

        // Redis pub/sub: ["message", channel, payload]
        std::string data;
        if (r->elements >= 3 && r->element[2]->type == REDIS_REPLY_STRING)
        {
            data.assign(r->element[2]->str, r->element[2]->len);
        }

        if (data.empty() && !tmp_img.empty())
        {
            if (tmp_img.size() != 3)
            {
                std::cerr << "chunk size must be 3\n";
                tmp_img.clear();
                freeReplyObject(r);
                continue;
            }

            int64_t sec = 0, nsec = 0;
            if (!parse_xml_timestamp(tmp_img[0], sec, nsec))
            {
                std::cerr << "[stereo] XML timestamp parse failed\n";
                tmp_img.clear();
                freeReplyObject(r);
                continue;
            }

            cv::Mat left, right;
            if (!split_stereo_image(tmp_img[2], left, right))
            {
                std::cerr << "[stereo] stereo jpeg decode failed\n";
                tmp_img.clear();
                freeReplyObject(r);
                continue;
            }

            // cv::imshow("stereo_left", left);
            // cv::imshow("stereo_right", right);
            // cv::waitKey(1);

            cv::Mat left_gray8, right_gray8;
            cv::cvtColor(left, left_gray8, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right, right_gray8, cv::COLOR_BGR2GRAY);

            cv::Mat left_gray16, right_gray16;
            left_gray8.convertTo(left_gray16, CV_16U, 256.0);
            right_gray8.convertTo(right_gray16, CV_16U, 256.0);

            basalt::ImageData img_l, img_r;

            img_l.img.reset(new basalt::ManagedImage<uint16_t>(512, 512));
            img_r.img.reset(new basalt::ManagedImage<uint16_t>(512, 512));

            std::memcpy(img_l.img->ptr, left_gray16.data, 512 * 512 * sizeof(uint16_t));
            std::memcpy(img_r.img->ptr, right_gray16.data, 512 * 512 * sizeof(uint16_t));

            basalt::OpticalFlowInput::Ptr in(new basalt::OpticalFlowInput);
            in->t_ns = sec * 1000000000LL + nsec;
            in->img_data = {img_l, img_r};
            opt_flow->input_queue.push(in);

            tmp_img.clear();

            freeReplyObject(r);
            continue;
        }

        std::vector<uint8_t> bin;
        if (!base64_decode(data, bin))
        {
            std::cerr << "[stereo] base64 decode failed\n";
            freeReplyObject(r);
            continue;
        }

        tmp_img.emplace_back(std::move(bin));
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
    vio->out_state_queue = &out_state_queue;
    opt_flow->output_queue = &vio->vision_data_queue;

    std::thread t_imu(imu_thread);
    std::thread t_img(stereo_thread);

    // ---- Main processing loop ----
    while (true)
    { // Get pose
        const size_t n = vio->out_state_queue->size();
        for (size_t i = 0; i < n; ++i)
        {
            basalt::PoseVelBiasState<double>::Ptr st;
            vio->out_state_queue->pop(st);

            if (!st.get())
                break;

            redis_publish_pose(st->T_w_i, st->t_ns);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    t_imu.join();
    t_img.join();
    return 0;
}

/*
 * Live Basalt VIO from Redis + Differential-drive Encoder EKF fusion
 *
 * Adds:
 *  - SUBSCRIBE "pserver-encoder"
 *  - Compute v (forward) and yaw_rate from 2-wheel encoder
 *  - EKF to fuse encoder dead-reckoning (predict) + VIO pose (update)
 *
 * Extended:
 *  - Online calibration with EKF states:
 *      kv      : wheel radius / distance scale factor
 *      kw      : wheel base factor
 *      lrlatio : left-right relative scale factor
 *
 * Time offset (TOFF):
 *  - REMOVED from EKF state
 *  - Estimated by SlidingXcorrToffEstimator (ENC yawrate vs VIO yawrate)
 *  - Applied only to encoder timestamps in predict stage
 */

#include <atomic>
#include <thread>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <regex>
#include <fstream>
#include <cmath>
#include <vector>
#include <cstring>
#include <sys/time.h>

#include <hiredis/hiredis.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tbb/concurrent_queue.h>

#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/calibration/calibration.hpp>
#include <basalt/utils/time_utils.hpp>
#include <basalt/serialization/headers_serialization.h>

#include <sophus/se3.hpp>
#include <CLI/CLI.hpp>

#include "toff_estimator.h"
#include "zupt_drift_canceller.h"
#include "ekf.h"
#include "stabilizer.h"

using json = nlohmann::json;

// ============================================================
// Redis config
// ============================================================
static const char *REDIS_HOST = "127.0.0.1";
static const int REDIS_PORT = 6379;

static const char *CH_STEREO = "pserver-forward-pst";
static const char *CH_IMU = "pserver-imu-raw";
static const char *CH_ENC = "pserver-enc-raw";
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

    {
        json j;
        j["t_ns"] = t_ns;
        j["p"] = {T_w_i.translation().x(),
                  T_w_i.translation().y(),
                  T_w_i.translation().z()};
        j["q"] = {T_w_i.unit_quaternion().w(),
                  T_w_i.unit_quaternion().x(),
                  T_w_i.unit_quaternion().y(),
                  T_w_i.unit_quaternion().z()};

        std::string s = j.dump(2);
        redisCommand(c, "PUBLISH %s %b", CH_POSE, s.data(), s.size());
    }

    {
        Eigen::Quaterniond q = T_w_i.unit_quaternion();

        double yaw_ccw = std::atan2(
            2.0 * (q.w() * q.z() + q.x() * q.y()),
            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        double heading = -yaw_ccw * 180.0 / M_PI;

        json j;
        j["mode"] = "odom";
        j["state"] = "UPDATE_ODOMETRY";
        j["odom"] = {
            {"x", -T_w_i.translation().y()},
            {"y", T_w_i.translation().x()},
            {"z", T_w_i.translation().z()},
            {"heading", heading}};
        j["timestamp"] = (double)t_ns / 1e9;

        std::string s = j.dump(2);
        redisCommand(c, "PUBLISH %s %b", "pserver-odometry-info", s.data(), s.size());
    }

    {
        Eigen::Quaterniond q = T_w_i.unit_quaternion();

        double yaw_ccw = std::atan2(
            2.0 * (q.w() * q.z() + q.x() * q.y()),
            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        double yaw_cw = -yaw_ccw;
        double yaw_deg = yaw_cw * 180.0 / M_PI;

        double pitch = std::asin(
            std::clamp(2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0));
        double pitch_deg = pitch * 180.0 / M_PI;

        double roll = std::atan2(
            2.0 * (q.w() * q.x() + q.y() * q.z()),
            1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
        double roll_deg = roll * 180.0 / M_PI;

        char buff[1024];
        int size = snprintf(buff, 1024, "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                            -T_w_i.translation().y(), 0.13, T_w_i.translation().x(),
                            yaw_deg, (double)t_ns / 1e9, pitch_deg, roll_deg);

        redisCommand(c, "PUBLISH %s %b", "vehicle_pos", buff, size);
    }

    redisFree(c);
}

// ============================================================
// Globals (Basalt)
// ============================================================
basalt::Calibration<double> calib;
basalt::VioConfig vio_config;
basalt::OpticalFlowBase::Ptr opt_flow;
basalt::VioEstimatorBase::Ptr vio;
tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> out_state_queue;

// ============================================================
// TOFF Estimator (ENC yawrate vs VIO yawrate)
// ============================================================
static SlidingXcorrToffEstimator g_toff_est;
static std::atomic<int64_t> g_toff_ns{0}; // encoder -> vio

// Global EKF instance
static DiffDriveEkf g_ekf;

// ============================================================
// Redis IMU thread (feed_imu equivalent)
// ============================================================
struct ImuSample
{
    int64_t t_ns;
    double ax, ay, az;
    double gx, gy, gz;
};

static inline bool parse_imu_message_array(const std::string &payload,
                                           std::vector<ImuSample> &out_list)
{
    out_list.clear();

    json j;
    try
    {
        j = json::parse(payload);
    }
    catch (...)
    {
        return false;
    }

    if (!j.is_array())
        return false;

    auto is_num = [](const json &v)
    {
        return v.is_number_integer() || v.is_number_unsigned() || v.is_number_float();
    };

    out_list.reserve(j.size());

    for (const auto &item : j)
    {
        if (!item.is_array() || item.size() < 7)
            continue;
        if (!is_num(item[0]))
            continue;
        if (!is_num(item[1]) || !is_num(item[2]) || !is_num(item[3]) ||
            !is_num(item[4]) || !is_num(item[5]) || !is_num(item[6]))
            continue;

        uint64_t ts_us = item[0].get<uint64_t>();

        ImuSample out;
        out.t_ns = ts_us * 1000;

        out.ax = item[1].get<double>();
        out.ay = item[2].get<double>();
        out.az = item[3].get<double>();

        out.gx = item[4].get<double>();
        out.gy = item[5].get<double>();
        out.gz = item[6].get<double>();

        out_list.push_back(out);
    }

    return true;
}

void imu_thread()
{
    redisContext *c = redis_sub(CH_IMU);
    redisReply *r;

    bool have_prev = false;
    ImuSample prev;

    while (redisGetReply(c, (void **)&r) == REDIS_OK)
    {
        if (!r || r->type != REDIS_REPLY_ARRAY)
        {
            if (r)
                freeReplyObject(r);
            continue;
        }

        std::string payload;
        if (r->elements >= 3 && r->element[2]->type == REDIS_REPLY_STRING)
            payload.assign(r->element[2]->str, r->element[2]->len);

        freeReplyObject(r);

        if (payload.empty())
            continue;

        std::vector<ImuSample> imus;
        try
        {
            if (!parse_imu_message_array(payload, imus))
                continue;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[enc] json parse error: " << e.what() << "\n";
            continue;
        }

        for (auto &cur : imus)
        {

            if (!have_prev)
            {
                prev = cur;
                have_prev = true;
                continue;
            }

            int64_t dt_ns = cur.t_ns - prev.t_ns;
            double dt = dt_ns * 1e-9;
            if (!(dt > 0.0 && dt < 0.2))
            {
                prev = cur;
                continue;
            }

            basalt::ImuData<double>::Ptr d(new basalt::ImuData<double>);

            d->t_ns = cur.t_ns;
            d->accel = Eigen::Vector3d(cur.ax, cur.ay, cur.az);
            d->gyro = Eigen::Vector3d(cur.gx, cur.gy, cur.gz);

            vio->imu_data_queue.push(d);
        }
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

bool parse_xml_timestamp(const std::vector<uint8_t> &xml_bytes, int64_t &sec, int64_t &nsec)
{
    if (xml_bytes.size() <= 4)
        return false;

    std::string xml(reinterpret_cast<const char *>(xml_bytes.data() + 4), xml_bytes.size() - 4);
    xml = std::regex_replace(xml, std::regex("</?\\w+:(\\w+)"), "<$1");

    std::smatch m;
    std::regex r("timestamp\\s*=\\s*\"(\\d+),(\\d+)\"");
    if (!std::regex_search(xml, m, r))
        return false;

    sec = std::stoll(m[1].str());
    nsec = std::stoll(m[2].str()) * 1000; // usec -> nsec
    return true;
}

bool split_stereo_image(const std::vector<uint8_t> &jpeg, cv::Mat &left, cv::Mat &right)
{
    cv::Mat buf(1, (int)jpeg.size(), CV_8UC1, const_cast<uint8_t *>(jpeg.data()));
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


static Mat g_left;
static Mat g_right;

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

        std::string data;
        if (r->elements >= 3 && r->element[2]->type == REDIS_REPLY_STRING)
            data.assign(r->element[2]->str, r->element[2]->len);

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

            int w = left.cols;
            int h = left.rows;

            cv::Mat left_gray8, right_gray8;
            cv::cvtColor(left, left_gray8, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right, right_gray8, cv::COLOR_BGR2GRAY);

            cv::Mat left_gray16, right_gray16;
            left_gray8.convertTo(left_gray16, CV_16U, 256.0);
            right_gray8.convertTo(right_gray16, CV_16U, 256.0);

            basalt::ImageData img_l, img_r;
            img_l.img.reset(new basalt::ManagedImage<uint16_t>(w, h));
            img_r.img.reset(new basalt::ManagedImage<uint16_t>(w, h));

            std::memcpy(img_l.img->ptr, left_gray16.data, w * h * sizeof(uint16_t));
            std::memcpy(img_r.img->ptr, right_gray16.data, w * h * sizeof(uint16_t));

            basalt::OpticalFlowInput::Ptr in(new basalt::OpticalFlowInput);
            in->t_ns = sec * 1000000000LL + nsec;
            in->img_data = {img_l, img_r};
            opt_flow->input_queue.push(in);

            {//stabilizer
                if (g_left.empty()){
                    g_left = left_gray8;
                    g_right = right_gray8;
                }
                Point2d shift;
                Mat affine_out;
                estimateShift(g_left, left_gray8, 32, 32, shift, affine_out);

                std::cout << "shift: " << shift << std::endl;
                std::cout << "Affine:\n" << affine_out << std::endl;

                g_left = left_gray8;
                g_right = right_gray8;
            }

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
// Encoder parsing + thread
// ============================================================
struct EncOdom
{
    int64_t t_ns = 0; // epoch time [ns]
    int64_t left_cnt = 0;
    int64_t right_cnt = 0;
};

static inline bool parse_enc_message_array(const std::string &payload,
                                           std::vector<EncOdom> &out_list)
{
    out_list.clear();

    json j = json::parse(payload);

    if (!j.is_array())
    {
        return false;
    }

    out_list.reserve(j.size());
    for (const auto &item : j)
    {
        // item must be array [ts_us, encL, encR]
        if (!item.is_array() || item.size() < 3)
            continue;

        // 0: ts_us
        if (!item[0].is_number_unsigned() && !item[0].is_number_integer())
            continue;
        uint64_t ts_us = item[0].get<uint64_t>();

        // 1: left, 2: right
        if (!item[1].is_number_integer() || !item[2].is_number_integer())
            continue;

        EncOdom out;
        out.t_ns = ts_us * 1000;

        out.left_cnt = item[1].get<int64_t>();
        out.right_cnt = item[2].get<int64_t>();

        out_list.push_back(out);
    }

    return true;
}

void enc_thread()
{
    redisContext *c = redis_sub(CH_ENC);
    redisReply *r;

    bool have_prev = false;
    EncOdom prev;

    while (redisGetReply(c, (void **)&r) == REDIS_OK)
    {
        if (!r || r->type != REDIS_REPLY_ARRAY)
        {
            if (r)
                freeReplyObject(r);
            continue;
        }

        std::string payload;
        if (r->elements >= 3 && r->element[2]->type == REDIS_REPLY_STRING)
            payload.assign(r->element[2]->str, r->element[2]->len);

        freeReplyObject(r);

        if (payload.empty())
            continue;

        std::vector<EncOdom> encs;
        try
        {
            if (!parse_enc_message_array(payload, encs))
                continue;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[enc] json parse error: " << e.what() << "\n";
            continue;
        }

        for (auto &cur : encs)
        {

            if (!have_prev)
            {
                prev = cur;
                have_prev = true;
                continue;
            }

            int64_t dt_ns = cur.t_ns - prev.t_ns;
            double dt = dt_ns * 1e-9;
            if (!(dt > 0.0 && dt < 0.2))
            {
                prev = cur;
                continue;
            }

            int64_t dL = cur.left_cnt - prev.left_cnt;
            int64_t dR = cur.right_cnt - prev.right_cnt;

            if (std::llabs(dL) == 0 && std::llabs(dR) == 0)
            {
                g_ZUPT = true;
            }
            else if (std::llabs(dL) <= 5 && std::llabs(dR) <= 5)
            {
                continue;
            }
            else
            {
                g_ZUPT = false;
            }

            // ---- encoder yawrate for TOFF estimator ----
            // Use nominal baseline for this estimator (scale doesn't matter much).
            double w_enc = 0.0;
            if (!g_ZUPT)
            {
                w_enc = g_ekf.cal_yaw(dL, dR) / dt;
                w_enc = clamp(w_enc, -10.0, 10.0);
            }

            if (!g_ZUPT)
            {
                g_toff_est.push_encoder(cur.t_ns, w_enc);
            }

            // ---- estimate toff online ----
            int64_t toff_ns_new;
            if (!g_ZUPT && g_toff_est.estimate(toff_ns_new))
            {
                // clamp
                toff_ns_new = std::max<int64_t>(-500000000LL, std::min<int64_t>(500000000LL, toff_ns_new));
                g_toff_ns.store(toff_ns_new, std::memory_order_relaxed);
            }

            // EKF predict (apply external toff)
            const int64_t toff_ns = g_toff_ns.load(std::memory_order_relaxed);
            g_ekf.predict_from_encoder_counts(cur.t_ns, dL, dR, dt, toff_ns);

            prev = cur;
        }
    }
}

// ============================================================
// Main
// ============================================================
int main(int argc, char **argv)
{

    CLI::App app{"basalt helper"};

    // ---- options ----
    std::string config_path = "vio_config.json";
    std::string calib_path = "calib.json";
    bool enable_zupt = true;

    app.add_option("--config", config_path, "Configuration json file path");
    app.add_option("--calib", calib_path, "Calibration json file path");
    app.add_flag("--zupt", enable_zupt, "Enable ZUPT (default: true)");

    CLI11_PARSE(app, argc, argv);

    std::cout << "[INFO] config_path=" << config_path << "\n";
    std::cout << "[INFO] calib_path=" << calib_path << "\n";
    std::cout << "[INFO] zupt=" << (enable_zupt ? "true" : "false") << "\n";

    // ---- load calibration (same as vio.cpp) ----
    {
        std::ifstream is(calib_path);
        cereal::JSONInputArchive ar(is);
        ar(calib);
    }
    vio_config.load(config_path);

    opt_flow = basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
    vio = basalt::VioEstimatorFactory::getVioEstimator(
        vio_config, calib, basalt::constants::g, true, true);

    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    vio->out_state_queue = &out_state_queue;
    opt_flow->output_queue = &vio->vision_data_queue;

    std::thread t_imu(imu_thread);
    std::thread t_img(stereo_thread);
    std::thread t_enc(enc_thread);

    // ---- for VIO yawrate ----
    bool vio_have_prev = false;
    double vio_yaw_prev = 0.0;
    int64_t vio_t_prev = 0;

    ZuptDriftCanceller g_zupt_cancel;
    int pose_count = 0;
    while (true)
    {
        const size_t n = vio->out_state_queue->size();
        for (size_t i = 0; i < n; ++i)
        {
            basalt::PoseVelBiasState<double>::Ptr st;
            vio->out_state_queue->pop(st);

            if (!st.get())
                break;

            pose_count++;

            if(enable_zupt){
                Sophus::SE3d T_corr = g_zupt_cancel.apply(st->T_w_i, g_ZUPT);
    
                // publish corrected pose
                redis_publish_pose(T_corr, st->t_ns);
            }else{
                redis_publish_pose(st->T_w_i, st->t_ns);
            }

            // ---- VIO yawrate for TOFF estimator ----
            {
                Eigen::Quaterniond q = st->T_w_i.unit_quaternion();
                Eigen::Vector3d ypr = quat_to_ypr_zyx(q);
                double yaw = ypr(0);

                if (!g_ZUPT && vio_have_prev)
                {
                    double dyaw = std::atan2(std::sin(yaw - vio_yaw_prev),
                                             std::cos(yaw - vio_yaw_prev));
                    double dt = (st->t_ns - vio_t_prev) * 1e-9;
                    if (dt > 0.0 && dt < 0.2)
                    {
                        double w_vio = dyaw / dt;
                        w_vio = clamp(w_vio, -10.0, 10.0);
                        g_toff_est.push_vio(st->t_ns, w_vio);

                        // estimate here too (helps when encoder thread is quiet)
                        int64_t toff_ns_new;
                        if (g_toff_est.estimate(toff_ns_new))
                        {
                            toff_ns_new = std::max<int64_t>(-500000000LL, std::min<int64_t>(500000000LL, toff_ns_new));
                            g_toff_ns.store(toff_ns_new, std::memory_order_relaxed);
                        }
                    }
                }

                vio_yaw_prev = yaw;
                vio_t_prev = st->t_ns;
                vio_have_prev = true;
            }

            // EKF update with VIO measurement
            Eigen::Vector3d p = st->T_w_i.translation();
            Eigen::Quaterniond q = st->T_w_i.unit_quaternion();
            g_ekf.update_from_vio(st->t_ns, p, q);

            Sophus::SE3d T_fused = g_ekf.get_fused_T_w_i();
            // redis_publish_pose(T_fused, st->t_ns);

            // debug print
            if ((pose_count % 60) == 0)
            {
                double kv, kw, lr;
                g_ekf.get_calib(kv, kw, lr);
                const int64_t toff_ns = g_toff_ns.load(std::memory_order_relaxed);

                std::cout << "[EKF] toff_ns=" << toff_ns
                          << " kv=" << kv << " kw=" << kw << " lr=" << lr
                          << " ZUPT=" << (g_ZUPT ? 1 : 0)
                          << "\n";
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    t_imu.join();
    t_img.join();
    t_enc.join();
    return 0;
}

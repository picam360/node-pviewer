/*
 * Live Basalt VIO from Redis + Differential-drive Encoder EKF fusion
 *
 * Adds:
 *  - SUBSCRIBE "pserver-enc"
 *  - Compute v (forward) and yaw_rate from 2-wheel encoder
 *  - EKF to fuse encoder dead-reckoning (predict) + VIO pose (update)
 *
 * Notes / TODOs:
 *  - You MUST set your wheel/encoder parameters (WHEEL_RADIUS_M, WHEEL_BASE_M, TICKS_PER_REV).
 *  - This EKF is pragmatic: it fuses position+yaw strongly; pitch/roll/pz mostly follow VIO.
 *  - State includes yawoffsetgain and encoder_vio_timeoffset_ns, but the timeoffset update is a
 *    lightweight approximation (enough to “pull” offset slowly if there is consistent lag).
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

using json = nlohmann::json;

// ============================================================
// Redis config
// ============================================================
static const char *REDIS_HOST = "127.0.0.1";
static const int REDIS_PORT = 6379;

static const char *CH_STEREO = "pserver-forward-pst";
static const char *CH_IMU    = "pserver-imu";
static const char *CH_ENC    = "pserver-enc";
static const char *CH_POSE   = "vio_pose";

// ============================================================
// Encoder / robot parameters (!!! SET THESE !!!)
// ============================================================
// Wheel radius [m]
static constexpr double WHEEL_RADIUS_M = 0.050;      // TODO: set
// Wheel baseline (distance between left/right wheel contact points) [m]
static constexpr double WHEEL_BASE_M   = 0.300;      // TODO: set
// Encoder ticks per revolution [ticks/rev]
static constexpr double TICKS_PER_REV  = 2048.0;     // TODO: set

// If your encoder publishes already "wheel_rad_per_sec" or "wheel_mps", you can bypass ticks-based conversion.
// This implementation assumes it publishes cumulative counts for left/right wheels.

static inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

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
            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())
        );
        double heading = -yaw_ccw * 180.0 / M_PI;

        json j;
        j["mode"] = "odom";
        j["state"] = "UPDATE_ODOMETRY";
        j["odom"] = {
            {"x", -T_w_i.translation().y()},
            {"y", T_w_i.translation().x()},
            {"z", T_w_i.translation().z()},
            {"heading", heading}
        };
        j["timestamp"] = (double)t_ns / 1e9;
    
        std::string s = j.dump(2);
        redisCommand(c, "PUBLISH %s %b", "pserver-odometry-info", s.data(), s.size());
    }
    // {
    //     Eigen::Quaterniond q = T_w_i.unit_quaternion();

    //     double yaw_ccw = std::atan2(
    //         2.0 * (q.w() * q.z() + q.x() * q.y()),
    //         1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())
    //     );
    //     double yaw_cw = -yaw_ccw;
    //     double yaw_deg = yaw_cw * 180.0 / M_PI;
        
    //     char buff[1024];
    //     int size = snprintf(buff, 1024, "%lf,0.13,%lf,%lf,%lf", -T_w_i.translation().y(), T_w_i.translation().x(), yaw_deg, (double)t_ns / 1e9);
    
    //     redisCommand(c, "PUBLISH %s %b", "vehicle_pos", buff, size);
    // }
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
// Utilities: quaternion <-> yaw/pitch/roll (ZYX)
// ============================================================
static inline Eigen::Vector3d quat_to_ypr_zyx(const Eigen::Quaterniond& q)
{
    // yaw(Z), pitch(Y), roll(X)
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    double yaw   = std::atan2(R(1,0), R(0,0));
    double pitch = std::asin(clamp(-R(2,0), -1.0, 1.0));
    double roll  = std::atan2(R(2,1), R(2,2));
    return Eigen::Vector3d(yaw, pitch, roll);
}

static inline Eigen::Quaterniond ypr_zyx_to_quat(double yaw, double pitch, double roll)
{
    Eigen::AngleAxisd az(yaw,   Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd ay(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd ax(roll,  Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = az * ay * ax;
    return q.normalized();
}

// ============================================================
// EKF state:
//   [px, py, pz, yaw, pitch, roll, yawoffsetgain, encoder_vio_timeoffset_ns]
// ============================================================
struct EkfState
{
    Eigen::Matrix<double, 8, 1> x;   // state
    Eigen::Matrix<double, 8, 8> P;   // covariance
    int64_t last_pred_t_ns = 0;
    double last_v_mps = 0.0;         // most recent v used for prediction (for yawoffsetgain, timeoffset update)
    double last_w_rps = 0.0;

    EkfState()
    {
        x.setZero();
        P.setIdentity();
        P *= 1e-2;
        // timeoffset initial uncertainty large (ns^2)
        P(7,7) = 1e16; // (1e8 ns)^2 = 0.1s^2, tune as needed
    }
};

class DiffDriveEkf
{
public:
    DiffDriveEkf()
    {
        reset();
    }

    void reset()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        st_ = EkfState();

        // Reasonable initial covariance:
        st_.P.setZero();
        st_.P.diagonal() <<
            1.0,   // px [m^2]
            1.0,   // py
            4.0,   // pz
            0.5,   // yaw [rad^2]
            0.5,   // pitch
            0.5,   // roll
            1.0,   // yawoffsetgain
            1e16;  // timeoffset [ns^2]
    }

    // Called from encoder thread: predict with v,w and encoder timestamp (raw encoder time)
    void predict_from_encoder(int64_t enc_t_ns, double v_mps, double yawrate_rps)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        // Apply estimated time offset to align encoder time to VIO time domain:
        const int64_t toff_ns = (int64_t)std::llround(st_.x(7));
        const int64_t t_ns = enc_t_ns + toff_ns;

        if (st_.last_pred_t_ns == 0)
        {
            st_.last_pred_t_ns = t_ns;
            st_.last_v_mps = v_mps;
            st_.last_w_rps = yawrate_rps;
            return;
        }

        double dt = (t_ns - st_.last_pred_t_ns) * 1e-9;
        // guard against weird timestamps
        if (!(dt > 0.0 && dt < 0.2))
        {
            st_.last_pred_t_ns = t_ns;
            st_.last_v_mps = v_mps;
            st_.last_w_rps = yawrate_rps;
            return;
        }

        // State indices
        const int PX=0, PY=1, PZ=2, YAW=3, PITCH=4, ROLL=5, GAIN=6, TOFF=7;

        double px = st_.x(PX);
        double py = st_.x(PY);
        double pz = st_.x(PZ);
        double yaw = st_.x(YAW);
        double pitch = st_.x(PITCH);
        double roll = st_.x(ROLL);
        (void)pz; (void)pitch; (void)roll;

        // Simple planar motion model:
        // yaw_{k+1} = yaw_k + w*dt
        // p_{k+1} = p_k + Rz(yaw_k) * [v*dt, 0]
        double dyaw = yawrate_rps * dt;
        double vdt  = v_mps * dt;

        double c = std::cos(yaw);
        double s = std::sin(yaw);

        st_.x(PX) = px + c * vdt;
        st_.x(PY) = py + s * vdt;
        st_.x(YAW)= yaw + dyaw;

        // Jacobian F = d f / d x
        Eigen::Matrix<double,8,8> F = Eigen::Matrix<double,8,8>::Identity();
        // d(px)/d(yaw) = -sin(yaw)*vdt
        F(PX,YAW) = -s * vdt;
        // d(py)/d(yaw) =  cos(yaw)*vdt
        F(PY,YAW) =  c * vdt;

        // (Optional) yawoffsetgain and timeoffset are random walk -> keep identity.

        // Process noise Q
        // Tune these based on your encoder quality & slip.
        Eigen::Matrix<double,8,8> Q = Eigen::Matrix<double,8,8>::Zero();
        const double sigma_v = 0.20;          // m/s noise
        const double sigma_w = 0.50;          // rad/s noise
        const double sigma_z = 0.05;          // m/sqrt(s) for pz random walk (small)
        const double sigma_pr = 0.02;         // rad/sqrt(s) for pitch/roll random walk (small)
        const double sigma_gain = 0.02;       // gain random walk
        const double sigma_toff = 5e5;        // ns/sqrt(s) random walk (slow)

        // Approximate: Q position from v noise
        double q_pos = (sigma_v*sigma_v) * dt*dt;
        Q(PX,PX) = q_pos;
        Q(PY,PY) = q_pos;
        Q(YAW,YAW) = (sigma_w*sigma_w) * dt*dt;

        Q(PZ,PZ)       = (sigma_z*sigma_z) * dt;
        Q(PITCH,PITCH) = (sigma_pr*sigma_pr) * dt;
        Q(ROLL,ROLL)   = (sigma_pr*sigma_pr) * dt;

        Q(GAIN,GAIN)   = (sigma_gain*sigma_gain) * dt;
        Q(TOFF,TOFF)   = (sigma_toff*sigma_toff) * dt;

        st_.P = F * st_.P * F.transpose() + Q;

        st_.last_pred_t_ns = t_ns;
        st_.last_v_mps = v_mps;
        st_.last_w_rps = yawrate_rps;
    }

    // Called when VIO pose is available (measurement update).
    // Uses VIO position + yaw/pitch/roll from quaternion.
    void update_from_vio(int64_t vio_t_ns,
                         const Eigen::Vector3d& p_w_i,
                         const Eigen::Quaterniond& q_w_i)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        // If we never predicted yet, initialize state from VIO directly.
        if (st_.last_pred_t_ns == 0)
        {
            init_from_vio_locked(vio_t_ns, p_w_i, q_w_i);
            return;
        }

        // Measurement z: [px, py, pz, yaw_meas, pitch_meas, roll_meas]
        // With yaw_meas model including speed-proportional offset:
        //   yaw_meas = yaw + yawoffsetgain * |v|
        // This helps absorb "moving-only yaw drift" into gain rather than corrupting yaw.
        Eigen::Vector3d ypr = quat_to_ypr_zyx(q_w_i);
        double yaw_meas   = ypr(0);
        double pitch_meas = ypr(1);
        double roll_meas  = ypr(2);

        const int PX=0, PY=1, PZ=2, YAW=3, PITCH=4, ROLL=5, GAIN=6, TOFF=7;

        Eigen::Matrix<double,6,1> z;
        z << p_w_i.x(), p_w_i.y(), p_w_i.z(),
             yaw_meas, pitch_meas, roll_meas;

        // Predicted measurement h(x)
        double vabs = std::abs(st_.last_v_mps);
        Eigen::Matrix<double,6,1> h;
        h << st_.x(PX), st_.x(PY), st_.x(PZ),
             st_.x(YAW) + st_.x(GAIN) * vabs,
             st_.x(PITCH),
             st_.x(ROLL);

        // Innovation
        Eigen::Matrix<double,6,1> r = z - h;
        // normalize yaw residual to [-pi,pi]
        r(3) = std::atan2(std::sin(r(3)), std::cos(r(3)));

        // Measurement Jacobian H (6x8)
        Eigen::Matrix<double,6,8> H = Eigen::Matrix<double,6,8>::Zero();
        H(0,PX) = 1.0;
        H(1,PY) = 1.0;
        H(2,PZ) = 1.0;
        H(3,YAW)= 1.0;
        H(3,GAIN)= vabs;
        H(4,PITCH)=1.0;
        H(5,ROLL)=1.0;

        // --- Timeoffset correction (approx) ---
        // If encoder is consistently delayed/advanced vs VIO, the EKF should shift TOFF.
        // We approximate that changing TOFF shifts prediction time by dT, which shifts px/py/yaw.
        // Use last v,w and current yaw for derivative:
        //
        // d(px)/d(toff_ns) ≈ (d(px)/dt) * 1e-9 = v*cos(yaw)*1e-9
        // d(py)/d(toff_ns) ≈ v*sin(yaw)*1e-9
        // d(yaw)/d(toff_ns)≈ w*1e-9
        //
        // This is not perfect, but works as a slow “alignment” term.
        {
            double yaw = st_.x(YAW);
            double c = std::cos(yaw), s = std::sin(yaw);
            double dpx_dtoff = st_.last_v_mps * c * 1e-9;
            double dpy_dtoff = st_.last_v_mps * s * 1e-9;
            double dyaw_dtoff= st_.last_w_rps * 1e-9;

            // Put into H for corresponding measurement rows (px,py,yaw_meas)
            H(0,TOFF) = dpx_dtoff;
            H(1,TOFF) = dpy_dtoff;
            H(3,TOFF) = dyaw_dtoff; // yaw_meas depends on yaw via TOFF-shifted prediction
        }

        // Measurement noise R
        // Tune: VIO position/orientation noise
        Eigen::Matrix<double,6,6> R = Eigen::Matrix<double,6,6>::Zero();
        const double sigma_p_xy = 0.05;   // m
        const double sigma_p_z  = 0.10;   // m
        const double sigma_yaw  = 0.05;   // rad
        const double sigma_pr   = 0.08;   // rad

        R(0,0) = sigma_p_xy*sigma_p_xy;
        R(1,1) = sigma_p_xy*sigma_p_xy;
        R(2,2) = sigma_p_z*sigma_p_z;
        R(3,3) = sigma_yaw*sigma_yaw;
        R(4,4) = sigma_pr*sigma_pr;
        R(5,5) = sigma_pr*sigma_pr;

        // EKF update
        Eigen::Matrix<double,6,6> S = H * st_.P * H.transpose() + R;
        Eigen::Matrix<double,8,6> K = st_.P * H.transpose() * S.inverse();

        st_.x = st_.x + K * r;

        Eigen::Matrix<double,8,8> I = Eigen::Matrix<double,8,8>::Identity();
        st_.P = (I - K * H) * st_.P;

        // keep TOFF in a sane range: e.g., +/- 0.5s
        st_.x(TOFF) = clamp(st_.x(TOFF), -5e8, 5e8);

        // Optional: keep gain in reasonable range
        st_.x(GAIN) = clamp(st_.x(GAIN), -2.0, 2.0);

        // Also lightly “snap” pz/pitch/roll toward VIO by having them in measurement.
        // Done already.

        // If VIO time jumps a lot, re-init
        if (std::llabs(vio_t_ns - st_.last_pred_t_ns) > (int64_t)2e9) // >2s
        {
            init_from_vio_locked(vio_t_ns, p_w_i, q_w_i);
        }
    }

    // Get fused pose (as SE3) from EKF state
    Sophus::SE3d get_fused_T_w_i() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        Eigen::Quaterniond q = ypr_zyx_to_quat(st_.x(3), st_.x(4), st_.x(5));
        Eigen::Vector3d p(st_.x(0), st_.x(1), st_.x(2));
        return Sophus::SE3d(q, p);
    }

    int64_t get_timeoffset_ns() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return (int64_t)std::llround(st_.x(7));
    }

    double get_yawoffsetgain() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return st_.x(6);
    }

private:
    void init_from_vio_locked(int64_t vio_t_ns,
                              const Eigen::Vector3d& p_w_i,
                              const Eigen::Quaterniond& q_w_i)
    {
        const int PX=0, PY=1, PZ=2, YAW=3, PITCH=4, ROLL=5;

        Eigen::Vector3d ypr = quat_to_ypr_zyx(q_w_i);

        st_.x(PX) = p_w_i.x();
        st_.x(PY) = p_w_i.y();
        st_.x(PZ) = p_w_i.z();
        st_.x(YAW)   = ypr(0);
        st_.x(PITCH) = ypr(1);
        st_.x(ROLL)  = ypr(2);

        st_.last_pred_t_ns = vio_t_ns;

        // shrink covariance after init
        st_.P.setIdentity();
        st_.P.diagonal() <<
            0.1, 0.1, 0.2,
            0.1, 0.2, 0.2,
            0.5, 1e16; // keep timeoffset uncertain initially
    }

    mutable std::mutex mtx_;
    EkfState st_;
};

// Global EKF instance
DiffDriveEkf g_ekf;

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

            if(false){
                struct timeval tv;
                gettimeofday(&tv, nullptr);
    
                int64_t now_ns =
                    static_cast<int64_t>(tv.tv_sec) * 1000000000LL +
                    static_cast<int64_t>(tv.tv_usec) * 1000LL;

                int64_t t_ns = sec * 1000000000LL + nsec;
    
                double diff_ms = (now_ns - t_ns) * 1e-6;
    
                std::cout << "[VIO] latency = " << diff_ms << " ms" << std::endl;
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

            // cv::imshow("stereo_left", left_gray8);
            // cv::imshow("stereo_right", right_gray8);
            // cv::waitKey(1);

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

            if(false){
                struct timeval tv;
                gettimeofday(&tv, nullptr);
    
                int64_t now_ns =
                    static_cast<int64_t>(tv.tv_sec) * 1000000000LL +
                    static_cast<int64_t>(tv.tv_usec) * 1000LL;
    
                double diff_ms = (now_ns - in->t_ns) * 1e-6;
    
                std::cout << "[VIO] latency = " << diff_ms << " ms" << std::endl;
            }

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
    int64_t t_ns = 0;
    int64_t left_cnt = 0;
    int64_t right_cnt = 0;
};

static inline bool parse_enc_message(const std::string& payload, EncOdom& out)
{
    // EXPECTED JSON EXAMPLE (you may need to adapt):
    // {
    //   "timestamp": {"sec": 123, "nanosec": 456},
    //   "left":  1234567,
    //   "right": 1234999
    // }
    //
    // If your fields are named differently, change here.

    json j = json::parse(payload);

    const auto& ts = j["timestamp"];
    int64_t sec  = ts["sec"].get<int64_t>();
    int64_t nsec = ts["nanosec"].get<int64_t>();

    out.t_ns = sec * 1000000000LL + nsec;
    out.left_cnt  = j["left"].get<int64_t>();
    out.right_cnt = j["right"].get<int64_t>();
    return true;
}

static inline void diffdrive_counts_to_vw(int64_t dL, int64_t dR, double dt,
                                         double& v_mps, double& yawrate_rps)
{
    // Convert ticks delta to distance
    const double meters_per_tick = (2.0 * M_PI * WHEEL_RADIUS_M) / TICKS_PER_REV;
    double dSl = (double)dL * meters_per_tick;
    double dSr = (double)dR * meters_per_tick;

    double v = (dSr + dSl) * 0.5 / dt;
    double w = (dSr - dSl) / (WHEEL_BASE_M * dt);

    v_mps = v;
    yawrate_rps = w;
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
            if (r) freeReplyObject(r);
            continue;
        }

        std::string payload;
        if (r->elements >= 3 && r->element[2]->type == REDIS_REPLY_STRING)
            payload.assign(r->element[2]->str, r->element[2]->len);

        freeReplyObject(r);

        if (payload.empty())
            continue;

        EncOdom cur;
        try
        {
            if (!parse_enc_message(payload, cur))
                continue;
        }
        catch (const std::exception& e)
        {
            std::cerr << "[enc] json parse error: " << e.what() << "\n";
            continue;
        }

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

        int64_t dL = cur.left_cnt  - prev.left_cnt;
        int64_t dR = cur.right_cnt - prev.right_cnt;

        double v_mps = 0.0, w_rps = 0.0;
        diffdrive_counts_to_vw(dL, dR, dt, v_mps, w_rps);

        // Optional: clamp extreme spikes
        v_mps = clamp(v_mps, -5.0, 5.0);
        w_rps = clamp(w_rps, -10.0, 10.0);

        // EKF predict
        g_ekf.predict_from_encoder(cur.t_ns, v_mps, w_rps);

        prev = cur;
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
    std::thread t_enc(enc_thread);

    // ---- Main processing loop ----
    int pose_count = 0;
    while (true)
    {
        // Drain out_state_queue (VIO output)
        const size_t n = vio->out_state_queue->size();
        for (size_t i = 0; i < n; ++i)
        {
            basalt::PoseVelBiasState<double>::Ptr st;
            vio->out_state_queue->pop(st);

            if (!st.get())
                break;

            // 1) Publish raw VIO pose (as before)
            redis_publish_pose(st->T_w_i, st->t_ns);
            pose_count++;

            // 2) EKF update with VIO measurement
            Eigen::Vector3d p = st->T_w_i.translation();
            Eigen::Quaterniond q = st->T_w_i.unit_quaternion();
            g_ekf.update_from_vio(st->t_ns, p, q);

            // 3) (Optional) publish fused pose too (re-using CH_POSE is confusing; use another channel)
            //    If you want, uncomment and change channel name:
            // Sophus::SE3d T_fused = g_ekf.get_fused_T_w_i();
            // redis_publish_pose(T_fused, st->t_ns);

            // Debug: show estimated timeoffset/gain occasionally
            // static int ctr = 0;
           s                // if ((++ctr % 200) == 0) {
            //     std::cout << "[EKF] toff_ns=" << g_ekf.get_timeoffset_ns()
            //               << " gain=" << g_ekf.get_yawoffsetgain() << "\n";
            // }

            if(false){
                struct timeval tv;
                gettimeofday(&tv, nullptr);
    
                int64_t now_ns =
                    static_cast<int64_t>(tv.tv_sec) * 1000000000LL +
                    static_cast<int64_t>(tv.tv_usec) * 1000LL;
    
                double diff_ms = (now_ns - st->t_ns) * 1e-6;
    
                std::cout << "[VIO] latency = " << diff_ms << " ms" << std::endl;
            }
            // if((pose_count%30) == 0){
            //     std::cout << "[VIO] velocity  =" << st->vel_w_i.x() << "," << st->vel_w_i.y() << "," << st->vel_w_i.z() << std::endl;
            //     std::cout << "[VIO] gyro_bias =" << st->bias_gyro.x() << "," << st->bias_gyro.y() << "," << st->bias_gyro.z() << std::endl;
            //     std::cout << "[VIO] accel_bias=" << st->bias_accel.x() << "," << st->bias_accel.y() << "," << st->bias_accel.z() << std::endl;
            // }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    t_imu.join();
    t_img.join();
    t_enc.join();
    return 0;
}

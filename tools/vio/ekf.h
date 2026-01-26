#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;


// ============================================================
// Encoder / robot parameters (nominal)
// ============================================================
// Encoder ticks per revolution [ticks/rev]
static constexpr double TICKS_PER_REV = 4096.0; // 12bit

// Nominal wheel radius [m]
static constexpr double WHEEL_RADIUS_M_NOM = 0.027;

// Nominal wheel baseline [m]
static constexpr double WHEEL_BASE_M_NOM = 0.200;

static bool g_ZUPT = false;

static inline double clamp(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

// ============================================================
// EKF state (TOFF removed):
//   [px, py, pz, yaw, pitch, roll, kv, kw, lrlatio]  (9 states)
// ============================================================
struct EkfState
{
    Eigen::Matrix<double, 9, 1> x;
    Eigen::Matrix<double, 9, 9> P;
    int64_t last_pred_t_ns = 0;

    EkfState()
    {
        x.setZero();
        P.setIdentity();
        P *= 1e-2;

        // init calib params
        x(6) = 1.0; // kv
        x(7) = 1.0; // kw
        x(8) = 0.0; // lrlatio
    }
};

class DiffDriveEkf
{
public:
    DiffDriveEkf() { reset(); }

    void reset()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        st_ = EkfState();

        st_.P.setZero();
        st_.P.diagonal() << 1.0, // px
            1.0,                 // py
            4.0,                 // pz
            0.5,                 // yaw
            0.5,                 // pitch
            0.5,                 // roll
            0.05 * 0.05,         // kv (5% sigma)
            0.05 * 0.05,         // kw (5% sigma)
            0.02 * 0.02;         // lrlatio (2% sigma)
    }

    // predict with encoder counts delta (best for calibrating kv/kw/lr)
    // ? toff_ns is provided externally (encoder -> vio)
    void predict_from_encoder_counts(int64_t enc_t_ns, int64_t dL, int64_t dR, double dt, int64_t toff_ns)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        // Apply external time offset: encoder time -> VIO time
        const int64_t t_ns = enc_t_ns + toff_ns;

        if (st_.last_pred_t_ns == 0)
        {
            st_.last_pred_t_ns = t_ns;
            return;
        }

        if (!(dt > 0.0 && dt < 0.2))
        {
            st_.last_pred_t_ns = t_ns;
            return;
        }

        const int PX = 0, PY = 1, PZ = 2, YAW = 3, PITCH = 4, ROLL = 5, KV = 6, KW = 7, LR = 8;

        double px = st_.x(PX);
        double py = st_.x(PY);
        double yaw = st_.x(YAW);

        // calibration params
        double kv = st_.x(KV);
        double kw = st_.x(KW);
        double lr = st_.x(LR);

        kv = clamp(kv, 0.5, 1.5);
        kw = clamp(kw, 0.5, 1.5);
        lr = clamp(lr, -0.2, 0.2);
        st_.x(KV) = kv;
        st_.x(KW) = kw;
        st_.x(LR) = lr;

        // meters_per_tick nominal
        const double mpt = (2.0 * M_PI * WHEEL_RADIUS_M_NOM) / TICKS_PER_REV;

        // Apply kv (distance scale) and lrlatio (left/right relative)
        const double sL = kv * (1.0 - lr);
        const double sR = kv * (1.0 + lr);

        double dSl = (double)dL * mpt * sL;
        double dSr = (double)dR * mpt * sR;

        // Apply kw to wheelbase
        const double B = WHEEL_BASE_M_NOM * kw;

        // v,w
        double v = (dSr + dSl) * 0.5 / dt;
        double w = (dSr - dSl) / (B * dt);

        if (g_ZUPT)
        {
            v = 0.0;
            w = 0.0;
        }

        v = clamp(v, -5.0, 5.0);
        w = clamp(w, -10.0, 10.0);

        // propagate state
        double vdt = v * dt;
        double dyaw = w * dt;

        double c = std::cos(yaw);
        double s = std::sin(yaw);

        st_.x(PX) = px + c * vdt;
        st_.x(PY) = py + s * vdt;
        st_.x(YAW) = yaw + dyaw;

        // ------------------------------------------------------------
        // Jacobian F = df/dx (9x9)
        // ------------------------------------------------------------
        Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();

        F(PX, YAW) = -s * vdt;
        F(PY, YAW) = c * vdt;

        // derivatives of v and w wrt kv/kw/lr
        const double A = (mpt / (2.0 * dt)) * ((double)dR * (1.0 + lr) + (double)dL * (1.0 - lr));
        const double dv_dkv = A;
        const double dv_dlr = kv * (mpt / (2.0 * dt)) * ((double)dR - (double)dL);

        const double C = (mpt / (WHEEL_BASE_M_NOM * dt)) * ((double)dR * (1.0 + lr) - (double)dL * (1.0 - lr));
        const double dw_dkv = C / kw;
        const double dw_dkw = -w / kw;
        const double dw_dlr = (kv / kw) * (mpt / (WHEEL_BASE_M_NOM * dt)) * ((double)dR + (double)dL);

        F(PX, KV) = c * (dv_dkv * dt);
        F(PY, KV) = s * (dv_dkv * dt);
        F(YAW, KV) = (dw_dkv * dt);

        F(YAW, KW) = (dw_dkw * dt);

        F(PX, LR) = c * (dv_dlr * dt);
        F(PY, LR) = s * (dv_dlr * dt);
        F(YAW, LR) = (dw_dlr * dt);

        // ------------------------------------------------------------
        // Process noise Q (9x9)
        // ------------------------------------------------------------
        Eigen::Matrix<double, 9, 9> Q = Eigen::Matrix<double, 9, 9>::Zero();

        double sigma_v = 0.5;
        double sigma_w = 1.0;
        if (g_ZUPT)
        {
            sigma_v = 0.0;
            sigma_w = 0.0;
        }

        double sigma_z = 0.5;  // m/sqrt(s)
        double sigma_pr = 0.2; // rad/sqrt(s)

        // calibration random walk
        double sigma_kv = 2e-4; // 1/sqrt(s)
        double sigma_kw = 2e-4; // 1/sqrt(s)
        double sigma_lr = 5e-5; // 1/sqrt(s)

        double q_pos = (sigma_v * sigma_v) * dt * dt;
        Q(PX, PX) = q_pos;
        Q(PY, PY) = q_pos;
        Q(YAW, YAW) = (sigma_w * sigma_w) * dt * dt;

        Q(PZ, PZ) = (sigma_z * sigma_z) * dt;
        Q(PITCH, PITCH) = (sigma_pr * sigma_pr) * dt;
        Q(ROLL, ROLL) = (sigma_pr * sigma_pr) * dt;

        Q(KV, KV) = (sigma_kv * sigma_kv) * dt;
        Q(KW, KW) = (sigma_kw * sigma_kw) * dt;
        Q(LR, LR) = (sigma_lr * sigma_lr) * dt;

        st_.P = F * st_.P * F.transpose() + Q;

        st_.last_pred_t_ns = t_ns;
    }

    // VIO measurement update
    void update_from_vio(int64_t vio_t_ns,
                         const Eigen::Vector3d &p_w_i,
                         const Eigen::Quaterniond &q_w_i)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        if (st_.last_pred_t_ns == 0)
        {
            init_from_vio_locked(vio_t_ns, p_w_i, q_w_i);
            return;
        }

        const int PX = 0, PY = 1, PZ = 2, YAW = 3, PITCH = 4, ROLL = 5, KV = 6, KW = 7, LR = 8;

        Eigen::Vector3d ypr = quat_to_ypr_zyx(q_w_i);
        double yaw_meas = ypr(0);
        double pitch_meas = ypr(1);
        double roll_meas = ypr(2);

        Eigen::Matrix<double, 6, 1> z;
        z << p_w_i.x(), p_w_i.y(), p_w_i.z(),
            yaw_meas, pitch_meas, roll_meas;

        Eigen::Matrix<double, 6, 1> h;
        h << st_.x(PX), st_.x(PY), st_.x(PZ),
            st_.x(YAW),
            st_.x(PITCH),
            st_.x(ROLL);

        Eigen::Matrix<double, 6, 1> r = z - h;
        r(3) = std::atan2(std::sin(r(3)), std::cos(r(3)));

        // H: 6x9
        Eigen::Matrix<double, 6, 9> H = Eigen::Matrix<double, 6, 9>::Zero();
        H(0, PX) = 1.0;
        H(1, PY) = 1.0;
        H(2, PZ) = 1.0;
        H(3, YAW) = 1.0;
        H(4, PITCH) = 1.0;
        H(5, ROLL) = 1.0;

        // Measurement noise R
        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
        const double sigma_p_xy = 0.0005;
        const double sigma_p_z = 0.010;
        const double sigma_yaw = 0.001;
        const double sigma_pr = 0.08;

        R(0, 0) = sigma_p_xy * sigma_p_xy;
        R(1, 1) = sigma_p_xy * sigma_p_xy;
        R(2, 2) = sigma_p_z * sigma_p_z;
        R(3, 3) = sigma_yaw * sigma_yaw;
        R(4, 4) = sigma_pr * sigma_pr;
        R(5, 5) = sigma_pr * sigma_pr;

        Eigen::Matrix<double, 6, 6> S = H * st_.P * H.transpose() + R;
        Eigen::Matrix<double, 9, 6> K = st_.P * H.transpose() * S.inverse();

        st_.x = st_.x + K * r;

        Eigen::Matrix<double, 9, 9> I = Eigen::Matrix<double, 9, 9>::Identity();
        st_.P = (I - K * H) * st_.P;

        // clamp parameters to sane ranges
        st_.x(KV) = clamp(st_.x(KV), 0.5, 1.5);
        st_.x(KW) = clamp(st_.x(KW), 0.5, 1.5);
        st_.x(LR) = clamp(st_.x(LR), -0.2, 0.2);

        if (std::llabs(vio_t_ns - st_.last_pred_t_ns) > (int64_t)2e9)
        {
            init_from_vio_locked(vio_t_ns, p_w_i, q_w_i);
        }
    }

    Sophus::SE3d get_fused_T_w_i() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        Eigen::Quaterniond q = ypr_zyx_to_quat(st_.x(3), st_.x(4), st_.x(5));
        Eigen::Vector3d p(st_.x(0), st_.x(1), st_.x(2));
        return Sophus::SE3d(q, p);
    }

    void get_calib(double &kv, double &kw, double &lr) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        kv = st_.x(6);
        kw = st_.x(7);
        lr = st_.x(8);
    }

    double cal_yaw(int64_t dL, int64_t dR){
        double yaw = 0;
        const double mpt = (2.0 * M_PI * WHEEL_RADIUS_M_NOM) / TICKS_PER_REV;
        const double dSl = (double)dL * mpt;
        const double dSr = (double)dR * mpt;
        const double Bnom = WHEEL_BASE_M_NOM;
        yawrate = (dSr - dSl) / Bnom;
        return yawrate;
    }

private:
    void init_from_vio_locked(int64_t vio_t_ns,
                              const Eigen::Vector3d &p_w_i,
                              const Eigen::Quaterniond &q_w_i)
    {
        const int PX = 0, PY = 1, PZ = 2, YAW = 3, PITCH = 4, ROLL = 5, KV = 6, KW = 7, LR = 8;

        Eigen::Vector3d ypr = quat_to_ypr_zyx(q_w_i);

        st_.x(PX) = p_w_i.x();
        st_.x(PY) = p_w_i.y();
        st_.x(PZ) = p_w_i.z();
        st_.x(YAW) = ypr(0);
        st_.x(PITCH) = ypr(1);
        st_.x(ROLL) = ypr(2);

        st_.last_pred_t_ns = vio_t_ns;

        st_.P.setIdentity();
        st_.P.diagonal() << 0.1, 0.1, 0.2,
            0.1, 0.2, 0.2,
            0.05 * 0.05,
            0.05 * 0.05,
            0.02 * 0.02;
    }

    mutable std::mutex mtx_;
    EkfState st_;
};
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
 * Notes:
 *  - kv/kw/lrlatio are observable only with enough motion excitation (turns etc).
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
 
 using json = nlohmann::json;
 
 // ============================================================
 // Redis config
 // ============================================================
 static const char *REDIS_HOST = "127.0.0.1";
 static const int REDIS_PORT = 6379;
 
 static const char *CH_STEREO = "pserver-forward-pst";
 static const char *CH_IMU    = "pserver-imu";
 static const char *CH_ENC    = "pserver-encoder";
 static const char *CH_POSE   = "vio_pose";
 
 // ============================================================
 // Encoder / robot parameters (nominal)
 // ============================================================
 // Encoder ticks per revolution [ticks/rev]
 static constexpr double TICKS_PER_REV  = 4096.0;   // 12bit
 
 // Nominal wheel radius [m]
 static constexpr double WHEEL_RADIUS_M_NOM = 0.027;
 
 // Nominal wheel baseline [m]
 static constexpr double WHEEL_BASE_M_NOM   = 0.200;
 
 static bool g_ZUPT = false;
 
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
   if (!c) return;
 
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
 
   {
     Eigen::Quaterniond q = T_w_i.unit_quaternion();
 
     double yaw_ccw = std::atan2(
       2.0 * (q.w() * q.z() + q.x() * q.y()),
       1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())
     );
     double yaw_cw = -yaw_ccw;
     double yaw_deg = yaw_cw * 180.0 / M_PI;
 
     char buff[1024];
     int size = snprintf(buff, 1024, "%lf,%lf,%lf,%lf,%lf",
         //   -T_w_i.translation().y(), T_w_i.translation().z(), T_w_i.translation().x(), yaw_deg, (double)t_ns / 1e9);
              -T_w_i.translation().y(), 0.13, T_w_i.translation().x(), yaw_deg, (double)t_ns / 1e9);
 
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
 // Utilities: quaternion <-> yaw/pitch/roll (ZYX)
 // ============================================================
 static inline Eigen::Vector3d quat_to_ypr_zyx(const Eigen::Quaterniond& q)
 {
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
 //   [px, py, pz, yaw, pitch, roll, toff_ns, kv, kw, lrlatio]
 // ============================================================
 struct EkfState
 {
   Eigen::Matrix<double, 10, 1> x;
   Eigen::Matrix<double, 10, 10> P;
   int64_t last_pred_t_ns = 0;
 
   // for timeoffset Jacobian approx
   double last_v_mps = 0.0;
   double last_w_rps = 0.0;
 
   EkfState()
   {
     x.setZero();
     P.setIdentity();
     P *= 1e-2;
     P(6,6) = 1e16; // toff variance huge initially
 
     // init calib params
     x(7) = 1.0; // kv
     x(8) = 1.0; // kw
     x(9) = 0.0; // lrlatio
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
     st_.P.diagonal() <<
       1.0,   // px
       1.0,   // py
       4.0,   // pz
       0.5,   // yaw
       0.5,   // pitch
       0.5,   // roll
       1e16,  // toff_ns
       0.05*0.05,  // kv (5% sigma)
       0.05*0.05,  // kw (5% sigma)
       0.02*0.02;  // lrlatio (2% sigma)
   }
 
   // predict with encoder counts delta (best for calibrating kv/kw/lr)
   void predict_from_encoder_counts(int64_t enc_t_ns, int64_t dL, int64_t dR, double dt)
   {
     std::lock_guard<std::mutex> lk(mtx_);
 
     // Apply estimated time offset (encoder -> VIO time domain)
     const int64_t toff_ns = (int64_t)std::llround(st_.x(6));
     const int64_t t_ns = enc_t_ns + toff_ns;
 
     if (st_.last_pred_t_ns == 0)
     {
       st_.last_pred_t_ns = t_ns;
       st_.last_v_mps = 0.0;
       st_.last_w_rps = 0.0;
       return;
     }
 
     // dt is already computed from encoder timestamps (before toff),
     // but toff affects absolute time; for stability, keep dt from encoder stream.
     if (!(dt > 0.0 && dt < 0.2))
     {
       st_.last_pred_t_ns = t_ns;
       return;
     }
 
     const int PX=0, PY=1, PZ=2, YAW=3, PITCH=4, ROLL=5, TOFF=6, KV=7, KW=8, LR=9;
 
     double px  = st_.x(PX);
     double py  = st_.x(PY);
     double yaw = st_.x(YAW);
 
     // calibration params
     double kv = st_.x(KV);
     double kw = st_.x(KW);
     double lr = st_.x(LR);
 
     // keep them in reasonable range during predict too
     kv = clamp(kv, 0.5, 1.5);
     kw = clamp(kw, 0.5, 1.5);
     lr = clamp(lr, -0.2, 0.2);
     st_.x(KV) = kv;
     st_.x(KW) = kw;
     st_.x(LR) = lr;
 
     // meters_per_tick nominal
     const double mpt = (2.0 * M_PI * WHEEL_RADIUS_M_NOM) / TICKS_PER_REV;
 
     // Apply kv (distance scale) and lrlatio (left/right relative)
     // left  scale = kv*(1 - lr)
     // right scale = kv*(1 + lr)
     const double sL = kv * (1.0 - lr);
     const double sR = kv * (1.0 + lr);
 
     double dSl = (double)dL * mpt * sL;
     double dSr = (double)dR * mpt * sR;
 
     // Apply kw to wheelbase
     const double B = WHEEL_BASE_M_NOM * kw;
 
     // v,w
     double v = (dSr + dSl) * 0.5 / dt;
     double w = (dSr - dSl) / (B * dt);
 
     // ZUPT behavior: if stopped, trust v,w = 0 strongly
     if (g_ZUPT) {
       v = 0.0;
       w = 0.0;
     }
 
     // clamp spikes
     v = clamp(v, -5.0, 5.0);
     w = clamp(w, -10.0, 10.0);
 
     // propagate state
     double vdt  = v * dt;
     double dyaw = w * dt;
 
     double c = std::cos(yaw);
     double s = std::sin(yaw);
 
     st_.x(PX)  = px + c * vdt;
     st_.x(PY)  = py + s * vdt;
     st_.x(YAW) = yaw + dyaw;
 
     // ------------------------------------------------------------
     // Jacobian F = df/dx (10x10)
     // ------------------------------------------------------------
     Eigen::Matrix<double,10,10> F = Eigen::Matrix<double,10,10>::Identity();
 
     // w.r.t yaw (same as before)
     F(PX,YAW) = -s * vdt;
     F(PY,YAW) =  c * vdt;
 
     // derivatives of v and w wrt kv/kw/lr
     // v = kv * A, where
     // A = mpt/(2dt) * ( dR*(1+lr) + dL*(1-lr) )
     // dv/dkv = A
     // dv/dlr = kv*mpt/(2dt) * (dR - dL)
     const double A = (mpt / (2.0*dt)) * ( (double)dR*(1.0+lr) + (double)dL*(1.0-lr) );
     const double dv_dkv = A;
     const double dv_dlr = kv * (mpt / (2.0*dt)) * ( (double)dR - (double)dL );
 
     // w = (kv/kw) * C, where
     // C = mpt/(WHEEL_BASE_NOM*dt) * ( dR*(1+lr) - dL*(1-lr) )
     // dw/dkv = C/kw
     // dw/dkw = -w/kw
     // dw/dlr = (kv/kw)* mpt/(WHEEL_BASE_NOM*dt) * (dR + dL)
     const double C = (mpt / (WHEEL_BASE_M_NOM*dt)) * ( (double)dR*(1.0+lr) - (double)dL*(1.0-lr) );
     const double dw_dkv = C / kw;
     const double dw_dkw = -w / kw;
     const double dw_dlr = (kv / kw) * (mpt / (WHEEL_BASE_M_NOM*dt)) * ( (double)dR + (double)dL );
 
     // px += cos(yaw)*v*dt
     // py += sin(yaw)*v*dt
     // yaw += w*dt
     F(PX,KV) = c * (dv_dkv * dt);
     F(PY,KV) = s * (dv_dkv * dt);
     F(YAW,KV)= (dw_dkv * dt);
 
     F(PX,KW) = c * (0.0);        // v doesn't depend on kw
     F(PY,KW) = s * (0.0);
     F(YAW,KW)= (dw_dkw * dt);
 
     F(PX,LR) = c * (dv_dlr * dt);
     F(PY,LR) = s * (dv_dlr * dt);
     F(YAW,LR)= (dw_dlr * dt);
 
     // ------------------------------------------------------------
     // Process noise Q (10x10)
     // ------------------------------------------------------------
     Eigen::Matrix<double,10,10> Q = Eigen::Matrix<double,10,10>::Zero();
 
     // encoder motion noise (m/s, rad/s)
     double sigma_v = 0.2;
     double sigma_w = 0.5;
 
     if (g_ZUPT) {
       sigma_v = 0.0;
       sigma_w = 0.0;
     }
 
     // small random walk for unobserved states
     double sigma_z     = 0.5;   // m/sqrt(s)
     double sigma_pr    = 0.2;   // rad/sqrt(s)
     double sigma_toff  = 5e5;   // ns/sqrt(s)
 
     // calibration random walk (tune!)
     // ?: kv/kw ??1?? 0.02% ??????????????
     double sigma_kv = 2e-4;     // 1/sqrt(s)
     double sigma_kw = 2e-4;     // 1/sqrt(s)
     double sigma_lr = 5e-5;     // 1/sqrt(s)
 
     // position noise from v noise (rough)
     double q_pos = (sigma_v*sigma_v) * dt*dt;
     Q(PX,PX)   = q_pos;
     Q(PY,PY)   = q_pos;
     Q(YAW,YAW) = (sigma_w*sigma_w) * dt*dt;
 
     Q(PZ,PZ)         = (sigma_z*sigma_z) * dt;
     Q(PITCH,PITCH)   = (sigma_pr*sigma_pr) * dt;
     Q(ROLL,ROLL)     = (sigma_pr*sigma_pr) * dt;
     Q(TOFF,TOFF)     = (sigma_toff*sigma_toff) * dt;
 
     Q(KV,KV)         = (sigma_kv*sigma_kv) * dt;
     Q(KW,KW)         = (sigma_kw*sigma_kw) * dt;
     Q(LR,LR)         = (sigma_lr*sigma_lr) * dt;
 
     st_.P = F * st_.P * F.transpose() + Q;
 
     st_.last_pred_t_ns = t_ns;
     st_.last_v_mps = v;
     st_.last_w_rps = w;
   }
 
   // VIO measurement update
   void update_from_vio(int64_t vio_t_ns,
                        const Eigen::Vector3d& p_w_i,
                        const Eigen::Quaterniond& q_w_i)
   {
     std::lock_guard<std::mutex> lk(mtx_);
 
     if (st_.last_pred_t_ns == 0)
     {
       init_from_vio_locked(vio_t_ns, p_w_i, q_w_i);
       return;
     }
 
     const int PX=0, PY=1, PZ=2, YAW=3, PITCH=4, ROLL=5, TOFF=6, KV=7, KW=8, LR=9;
 
     Eigen::Vector3d ypr = quat_to_ypr_zyx(q_w_i);
     double yaw_meas   = ypr(0);
     double pitch_meas = ypr(1);
     double roll_meas  = ypr(2);
 
     Eigen::Matrix<double,6,1> z;
     z << p_w_i.x(), p_w_i.y(), p_w_i.z(),
          yaw_meas, pitch_meas, roll_meas;
 
     Eigen::Matrix<double,6,1> h;
     h << st_.x(PX), st_.x(PY), st_.x(PZ),
          st_.x(YAW),
          st_.x(PITCH),
          st_.x(ROLL);
 
     Eigen::Matrix<double,6,1> r = z - h;
     r(3) = std::atan2(std::sin(r(3)), std::cos(r(3)));
 
     // H: 6x10
     Eigen::Matrix<double,6,10> H = Eigen::Matrix<double,6,10>::Zero();
     H(0,PX) = 1.0;
     H(1,PY) = 1.0;
     H(2,PZ) = 1.0;
     H(3,YAW)= 1.0;
     H(4,PITCH)=1.0;
     H(5,ROLL)=1.0;
 
     // --- timeoffset approx jacobian ---
     {
       double yaw = st_.x(YAW);
       double c = std::cos(yaw), s = std::sin(yaw);
 
       double dpx_dtoff = st_.last_v_mps * c * 1e-9;
       double dpy_dtoff = st_.last_v_mps * s * 1e-9;
       double dyaw_dtoff= st_.last_w_rps * 1e-9;
 
       H(0,TOFF) = dpx_dtoff;
       H(1,TOFF) = dpy_dtoff;
       H(3,TOFF) = dyaw_dtoff;
     }
 
     // Measurement noise R
     Eigen::Matrix<double,6,6> R = Eigen::Matrix<double,6,6>::Zero();
     const double sigma_p_xy = 0.005;
     const double sigma_p_z  = 0.010;
     const double sigma_yaw  = 0.05;
     const double sigma_pr   = 0.08;
 
     R(0,0) = sigma_p_xy*sigma_p_xy;
     R(1,1) = sigma_p_xy*sigma_p_xy;
     R(2,2) = sigma_p_z*sigma_p_z;
     R(3,3) = sigma_yaw*sigma_yaw;
     R(4,4) = sigma_pr*sigma_pr;
     R(5,5) = sigma_pr*sigma_pr;
 
     Eigen::Matrix<double,6,6> S = H * st_.P * H.transpose() + R;
     Eigen::Matrix<double,10,6> K = st_.P * H.transpose() * S.inverse();
 
     st_.x = st_.x + K * r;
 
     Eigen::Matrix<double,10,10> I = Eigen::Matrix<double,10,10>::Identity();
     st_.P = (I - K * H) * st_.P;
 
     // clamp parameters to sane ranges
     st_.x(TOFF) = clamp(st_.x(TOFF), -5e8, 5e8); // +/-0.5s
     st_.x(KV)   = clamp(st_.x(KV),   0.5, 1.5);
     st_.x(KW)   = clamp(st_.x(KW),   0.5, 1.5);
     st_.x(LR)   = clamp(st_.x(LR),  -0.2, 0.2);
 
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
 
   int64_t get_timeoffset_ns() const
   {
     std::lock_guard<std::mutex> lk(mtx_);
     return (int64_t)std::llround(st_.x(6));
   }
 
   void get_calib(double& kv, double& kw, double& lr) const
   {
     std::lock_guard<std::mutex> lk(mtx_);
     kv = st_.x(7);
     kw = st_.x(8);
     lr = st_.x(9);
   }
 
 private:
   void init_from_vio_locked(int64_t vio_t_ns,
                             const Eigen::Vector3d& p_w_i,
                             const Eigen::Quaterniond& q_w_i)
   {
     const int PX=0, PY=1, PZ=2, YAW=3, PITCH=4, ROLL=5, TOFF=6, KV=7, KW=8, LR=9;
 
     Eigen::Vector3d ypr = quat_to_ypr_zyx(q_w_i);
 
     st_.x(PX) = p_w_i.x();
     st_.x(PY) = p_w_i.y();
     st_.x(PZ) = p_w_i.z();
     st_.x(YAW)   = ypr(0);
     st_.x(PITCH) = ypr(1);
     st_.x(ROLL)  = ypr(2);
 
     st_.last_pred_t_ns = vio_t_ns;
 
     // keep toff uncertain; keep calib moderate uncertainty
     st_.P.setIdentity();
     st_.P.diagonal() <<
       0.1, 0.1, 0.2,
       0.1, 0.2, 0.2,
       1e16,
       0.05*0.05,
       0.05*0.05,
       0.02*0.02;
 
     // do not reset kv/kw/lr unless you want hard reset:
     // st_.x(KV)=1.0; st_.x(KW)=1.0; st_.x(LR)=0.0;
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
   if (c >= 'A' && c <= 'Z') return c - 'A';
   if (c >= 'a' && c <= 'z') return c - 'a' + 26;
   if (c >= '0' && c <= '9') return c - '0' + 52;
   if (c == '+') return 62;
   if (c == '/') return 63;
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
     if (c == '=') break;
 
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
   if (xml_bytes.size() <= 4) return false;
 
   std::string xml(reinterpret_cast<const char *>(xml_bytes.data() + 4), xml_bytes.size() - 4);
   xml = std::regex_replace(xml, std::regex("</?\\w+:(\\w+)"), "<$1");
 
   std::smatch m;
   std::regex r("timestamp\\s*=\\s*\"(\\d+),(\\d+)\"");
   if (!std::regex_search(xml, m, r)) return false;
 
   sec = std::stoll(m[1].str());
   nsec = std::stoll(m[2].str()) * 1000; // usec -> nsec
   return true;
 }
 
 bool split_stereo_image(const std::vector<uint8_t> &jpeg, cv::Mat &left, cv::Mat &right)
 {
   cv::Mat buf(1, (int)jpeg.size(), CV_8UC1, const_cast<uint8_t *>(jpeg.data()));
   cv::Mat img = cv::imdecode(buf, cv::IMREAD_COLOR);
   if (img.empty()) return false;
 
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
       if (r) freeReplyObject(r);
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
   json j = json::parse(payload);
 
   const auto& ts = j["timestamp"];
   int64_t sec  = ts["sec"].get<int64_t>();
   int64_t nsec = ts["nanosec"].get<int64_t>();
 
   out.t_ns = sec * 1000000000LL + nsec;
   out.left_cnt  = j["left"].get<int64_t>();
   out.right_cnt = j["right"].get<int64_t>();
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
       if (r) freeReplyObject(r);
       continue;
     }
 
     std::string payload;
     if (r->elements >= 3 && r->element[2]->type == REDIS_REPLY_STRING)
       payload.assign(r->element[2]->str, r->element[2]->len);
 
     freeReplyObject(r);
 
     if (payload.empty()) continue;
 
     EncOdom cur;
     try
     {
       if (!parse_enc_message(payload, cur)) continue;
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
 
     if (std::llabs(dL) <= 1) dL = 0;
     if (std::llabs(dR) <= 1) dR = 0;
 
     g_ZUPT = (dL == 0 && dR == 0);
 
     // EKF predict (counts-based, best for kv/kw/lr estimation)
     g_ekf.predict_from_encoder_counts(cur.t_ns, dL, dR, dt);
 
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
 
   int pose_count = 0;
   while (true)
   {
     const size_t n = vio->out_state_queue->size();
     for (size_t i = 0; i < n; ++i)
     {
       basalt::PoseVelBiasState<double>::Ptr st;
       vio->out_state_queue->pop(st);
 
       if (!st.get()) break;
 
       pose_count++;

        redis_publish_pose(st->T_w_i, st->t_ns);
       
 
       // EKF update with VIO measurement
       Eigen::Vector3d p = st->T_w_i.translation();
       Eigen::Quaterniond q = st->T_w_i.unit_quaternion();
       g_ekf.update_from_vio(st->t_ns, p, q);
 
       Sophus::SE3d T_fused = g_ekf.get_fused_T_w_i();
       //redis_publish_pose(T_fused, st->t_ns);
 
       // debug print
       if ((pose_count % 60) == 0) {
         double kv, kw, lr;
         g_ekf.get_calib(kv, kw, lr);
         std::cout << "[EKF] toff_ns=" << g_ekf.get_timeoffset_ns()
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
 
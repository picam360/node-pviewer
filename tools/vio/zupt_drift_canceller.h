#include <cmath>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

// =============================================
// ZUPT drift canceller (delta-bias accumulation)
// - During ZUPT: Freeze XY and Yaw (no drift output)
// - At ZUPT falling edge: compute drift between start/end and accumulate bias
// - Bias is accumulated over multiple intermittent ZUPT segments
// =============================================
class ZuptDriftCanceller
{
public:
    ZuptDriftCanceller() { reset(); }

    void reset()
    {
        in_zupt_ = false;

        bias_xy_.setZero();
        bias_yaw_ = 0.0;

        // hold pose in corrected frame
        hold_xy_.setZero();
        hold_yaw_ = 0.0;

        // for drift measurement at ZUPT end
        end_xy_.setZero();
        end_yaw_ = 0.0;
    }

    Sophus::SE3d apply(const Sophus::SE3d &T_raw, bool zupt_active)
    {
        // Always start from "bias-applied" pose
        Sophus::SE3d T_corr = apply_xy_yaw_bias(T_raw, bias_xy_, bias_yaw_);

        // -------------------------
        // ZUPT rising edge
        // -------------------------
        if (zupt_active && !in_zupt_)
        {
            in_zupt_ = true;

            // Freeze reference pose (corrected pose) at the beginning of ZUPT
            hold_xy_ = T_corr.translation().head<2>();
            hold_yaw_ = yaw_from_se3(T_corr);

            // initialize end pose as hold pose
            end_xy_ = hold_xy_;
            end_yaw_ = hold_yaw_;
        }

        // -------------------------
        // ZUPT falling edge
        // -------------------------
        if (!zupt_active && in_zupt_)
        {
            // We exit ZUPT now.
            // Drift observed during ZUPT (in corrected frame):
            //   drift = end - hold
            // To cancel drift in future, accumulate bias += (hold - end)
            Eigen::Vector2d dxy = hold_xy_ - end_xy_;
            double dyaw = wrap_pi(hold_yaw_ - end_yaw_);

            bias_xy_ += dxy;
            bias_yaw_ = wrap_pi(bias_yaw_ + dyaw);

            in_zupt_ = false;

            // Recompute output after updating bias (continuity)
            T_corr = apply_xy_yaw_bias(T_raw, bias_xy_, bias_yaw_);
        }

        // -------------------------
        // While in ZUPT
        // -------------------------
        if (in_zupt_)
        {
            // Update "end" measurement each frame (how the corrected pose drifts)
            end_xy_ = T_corr.translation().head<2>();
            end_yaw_ = yaw_from_se3(T_corr);

            // Output pose with XY + Yaw frozen at hold
            // (Pitch/Roll/Z are kept as current corrected pose to avoid unnatural freeze)
            Sophus::SE3d T_frozen = freeze_xy_yaw(T_corr, hold_xy_, hold_yaw_);
            zupt_prev_ = zupt_active;
            return T_frozen;
        }
        else
        {
            return T_corr;
        }
    }

    Eigen::Vector2d biasXY() const { return bias_xy_; }
    double biasYaw() const { return bias_yaw_; }

private:
    bool zupt_prev_ = false;
    bool in_zupt_ = false;

    // accumulated bias (persistent across intermittent ZUPT)
    Eigen::Vector2d bias_xy_;
    double bias_yaw_ = 0.0;

    // frozen reference pose at ZUPT start (corrected frame)
    Eigen::Vector2d hold_xy_;
    double hold_yaw_ = 0.0;

    // last corrected pose in ZUPT (used to measure drift)
    Eigen::Vector2d end_xy_;
    double end_yaw_ = 0.0;

private:
    static double wrap_pi(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a < -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

    static double yaw_from_se3(const Sophus::SE3d &T)
    {
        const Eigen::Matrix3d R = T.so3().matrix();
        return std::atan2(R(1, 0), R(0, 0));
    }

    static Sophus::SE3d apply_xy_yaw_bias(const Sophus::SE3d &Tin,
                                          const Eigen::Vector2d &bias_xy,
                                          double bias_yaw)
    {
        Eigen::Vector3d t = Tin.translation();
        t.x() += bias_xy.x();
        t.y() += bias_xy.y();

        Eigen::AngleAxisd aa(bias_yaw, Eigen::Vector3d::UnitZ());
        Sophus::SO3d Rcorr(aa.toRotationMatrix());
        Sophus::SO3d Rout = Rcorr * Tin.so3();

        return Sophus::SE3d(Rout, t);
    }

    static Sophus::SE3d freeze_xy_yaw(const Sophus::SE3d &Tin,
                                      const Eigen::Vector2d &xy_hold,
                                      double yaw_hold)
    {
        Eigen::Vector3d t = Tin.translation();
        t.x() = xy_hold.x();
        t.y() = xy_hold.y();

        // keep pitch/roll from Tin, overwrite yaw only
        Eigen::Matrix3d R = Tin.so3().matrix();
        double pitch = std::asin(-R(2, 0));
        double roll = std::atan2(R(2, 1), R(2, 2));

        // reconstruct rotation from (yaw_hold, pitch, roll) using ZYX
        Eigen::AngleAxisd Rz(yaw_hold, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());

        Eigen::Matrix3d Rnew = Rz.toRotationMatrix() * Ry.toRotationMatrix() * Rx.toRotationMatrix();
        Sophus::SO3d Rout(Rnew);

        return Sophus::SE3d(Rout, t);
    }
};

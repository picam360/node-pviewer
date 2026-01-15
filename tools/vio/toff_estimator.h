#pragma once
#include <deque>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

class SlidingXcorrToffEstimator
{
public:
    struct Sample {
        int64_t t_ns;
        double  w;
    };

    SlidingXcorrToffEstimator()
    {
        window_ns_   = (int64_t)(3.0 * 1e9);   // 3秒窓
        max_lag_ns_  = (int64_t)(0.5 * 1e9);   // ±0.5秒探索
        step_ns_     = (int64_t)(5e6);         // 5ms step
        min_energy_  = 1e-3;                  // 運動判定
    }

    // ----------------------------------
    // push data
    // ----------------------------------
    void push_encoder(int64_t t_ns, double yawrate)
    {
        enc_.push_back({t_ns, yawrate});
        prune(enc_);
    }

    void push_vio(int64_t t_ns, double yawrate)
    {
        vio_.push_back({t_ns, yawrate});
        prune(vio_);
    }

    // ----------------------------------
    // estimate toff (encoder -> vio)
    // ----------------------------------
    bool estimate(int64_t& out_toff_ns)
    {
        if (enc_.size() < 10 || vio_.size() < 10)
            return false;

        // check motion energy
        if (signal_energy(enc_) < min_energy_ ||
            signal_energy(vio_) < min_energy_)
            return false;

        double best_corr = -1e30;
        int64_t best_lag = 0;

        for (int64_t lag = -max_lag_ns_; lag <= max_lag_ns_; lag += step_ns_)
        {
            double c = correlation_at_lag(lag);
            if (c > best_corr)
            {
                best_corr = c;
                best_lag = lag;
            }
        }

        // sanity
        if (!std::isfinite(best_corr))
            return false;

        // low-pass filter
        if (!initialized_) {
            toff_ns_ = best_lag;
            initialized_ = true;
        } else {
            const double alpha = 0.1; // smooth
            toff_ns_ = (int64_t)((1.0 - alpha) * toff_ns_ + alpha * best_lag);
        }

        out_toff_ns = toff_ns_;
        return true;
    }

private:
    // ----------------------------------
    // internals
    // ----------------------------------
    std::deque<Sample> enc_, vio_;

    int64_t window_ns_;
    int64_t max_lag_ns_;
    int64_t step_ns_;
    double  min_energy_;

    bool initialized_ = false;
    int64_t toff_ns_ = 0;

    // ----------------------------------
    void prune(std::deque<Sample>& q)
    {
        if (q.empty()) return;

        int64_t t_latest = q.back().t_ns;
        while (!q.empty() && (t_latest - q.front().t_ns) > window_ns_)
            q.pop_front();
    }

    // ----------------------------------
    static double signal_energy(const std::deque<Sample>& q)
    {
        double e = 0.0;
        for (auto& s : q)
            e += s.w * s.w;
        return e / std::max<size_t>(1, q.size());
    }

    // ----------------------------------
    double correlation_at_lag(int64_t lag_ns)
    {
        // corr( enc(t+lag), vio(t) )
        double sum = 0.0;
        double sum_e = 0.0;
        double sum_v = 0.0;

        size_t cnt = 0;
        size_t j = 0;

        for (size_t i = 0; i < enc_.size(); i++)
        {
            int64_t t = enc_[i].t_ns + lag_ns;

            // advance vio index
            while (j + 1 < vio_.size() && vio_[j + 1].t_ns < t)
                j++;

            if (j + 1 >= vio_.size())
                break;

            // linear interpolation
            const auto& v0 = vio_[j];
            const auto& v1 = vio_[j + 1];

            double alpha =
                (double)(t - v0.t_ns) /
                (double)(v1.t_ns - v0.t_ns);

            if (alpha < 0.0 || alpha > 1.0)
                continue;

            double wv = v0.w * (1.0 - alpha) + v1.w * alpha;
            double we = enc_[i].w;

            sum   += we * wv;
            sum_e += we * we;
            sum_v += wv * wv;
            cnt++;
        }

        if (cnt < 5)
            return -1e30;

        double denom = std::sqrt(sum_e * sum_v) + 1e-12;
        return sum / denom;
    }
};

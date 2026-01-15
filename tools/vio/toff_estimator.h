#pragma once
#include <deque>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <mutex>

// Sliding window time-offset estimator using normalized cross-correlation
// between encoder yaw-rate and VIO yaw-rate.
//
// "Follow-mode" version (prevents JUMP):
//  - Thread-safe (push_* and estimate can be called from different threads)
//  - ZUPT / low-omega gating (do not push, do not estimate)
//  - Motion-energy gating
//  - Peak uniqueness check (best vs second-best) with "different-peak" lag-gap rule
//  - Local search around current toff (optional) + full search when uninitialized
//  - Step-limited update: toff is adjusted gradually (max_step_ns per estimate)
//  - Optional EWMA smoothing on top of step-limited update
//  - Optional "hold" mode
//
// Convention:
//  - toff_ns means: encoder_time + toff_ns ~= vio_time
//  - Correlation computed as corr( enc(t+lag), vio(t) ), so lag is encoder->vio shift.
class SlidingXcorrToffEstimator
{
public:
    struct Sample {
        int64_t t_ns; // epoch time [ns]
        double  w;    // yaw rate [rad/s]
    };

    struct Result {
        bool    ok = false;
        int64_t toff_ns = 0;

        // Diagnostics
        int64_t best_lag_ns = 0;
        double  best_corr = -1e30;
        int64_t second_lag_ns = 0;
        double  second_corr = -1e30;
        size_t  used_pairs = 0;      // overlap count at best lag
        bool    accepted = false;    // accepted after gating/rules
        const char* reason = "";     // "ok" or rejection reason
        int64_t target_lag_ns = 0;   // lag we want to move toward (best peak)
        int64_t applied_step_ns = 0; // actual step applied this estimate (follow mode)
        int64_t search_lo_ns = 0;    // search range used
        int64_t search_hi_ns = 0;
    };

    SlidingXcorrToffEstimator()
    {
        // Window/search parameters
        window_ns_   = (int64_t)(30.0 * 1e9);   // sliding window length
        max_lag_ns_  = (int64_t)(0.5 * 1e9);   // search range +/- max_lag
        step_ns_     = (int64_t)(5e6);         // search step (5 ms)

        // Motion gates
        min_energy_  = 1e-3;     // mean(w^2) threshold
        w_min_push_  = 0.05;     // rad/s: ignore near-zero yawrate samples when pushing

        // Estimate acceptance gates
        min_corr_            = 0.10; // reject if best correlation is too low
        min_peak_separation_ = 0.00; // reject if best-second is too small (ambiguous)
        min_used_pairs_      = 15;   // reject if overlap count too small

        // Stability controls
        jump_limit_ns_ = (int64_t)(1e8); // 100 ms: sanity bound vs previous (optional)
        alpha_         = 0.05;           // EWMA smoothing factor (applied to step-limited update)
        hold_mode_     = false;          // if true, do not update even if estimate is good (manual hold)

        // ZUPT behavior
        zupt_ = false;

        // Follow-mode (prevents JUMP)
        follow_mode_     = true;              // enable gradual update by default
        local_search_ns_ = (int64_t)(50e6);   // +/-50ms around current toff (when initialized)
        max_step_ns_     = (int64_t)(1e6);    // max change per estimate: 1ms

        // Peak ambiguity handling
        different_peak_gap_steps_ = 3; // treat second as "different peak" only if lag_gap >= 3*step
        high_corr_allow_ambig_    = 0.60; // if best_corr >= this, allow ambiguity (optional)
    }

    // -----------------------------
    // Configuration (thread-safe)
    // -----------------------------
    void set_window_seconds(double sec)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        window_ns_ = (int64_t)(sec * 1e9);
        prune_locked(enc_);
        prune_locked(vio_);
    }

    void set_search(double max_lag_sec, double step_ms)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        max_lag_ns_ = (int64_t)(max_lag_sec * 1e9);
        step_ns_    = (int64_t)(step_ms * 1e6);
        if (step_ns_ <= 0) step_ns_ = (int64_t)1e6;
    }

    void set_motion_gates(double min_energy, double w_min_push)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        min_energy_ = min_energy;
        w_min_push_ = w_min_push;
    }

    void set_acceptance_gates(double min_corr, double min_peak_separation, size_t min_used_pairs)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        min_corr_            = min_corr;
        min_peak_separation_ = min_peak_separation;
        min_used_pairs_      = min_used_pairs;
    }

    void set_jump_limit_ms(double ms)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        jump_limit_ns_ = (int64_t)(ms * 1e6);
    }

    void set_smoothing_alpha(double alpha)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        alpha_ = alpha;
        if (alpha_ < 0.0) alpha_ = 0.0;
        if (alpha_ > 1.0) alpha_ = 1.0;
    }

    void set_hold_mode(bool hold)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        hold_mode_ = hold;
    }

    // Set ZUPT externally (recommended).
    // When ZUPT is true, we skip push and estimate (we hold last toff).
    void set_zupt(bool zupt)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        zupt_ = zupt;
    }

    // Follow-mode controls
    void set_follow_mode(bool on)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        follow_mode_ = on;
    }

    // local_search_ms: local search half-range around current toff (when initialized)
    // max_step_ms: maximum toff change per estimate
    void set_follow_params(double local_search_ms, double max_step_ms)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        local_search_ns_ = (int64_t)(local_search_ms * 1e6);
        max_step_ns_     = (int64_t)(max_step_ms * 1e6);
        if (local_search_ns_ < 0) local_search_ns_ = 0;
        if (max_step_ns_ < 0) max_step_ns_ = 0;
    }

    // Peak ambiguity handling controls
    void set_peak_gap_steps(int steps)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        different_peak_gap_steps_ = std::max(0, steps);
    }

    void set_high_corr_allow_ambiguity(double thr)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        high_corr_allow_ambig_ = thr;
    }

    // Reset internal state and buffers.
    void reset()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        enc_.clear();
        vio_.clear();
        initialized_ = false;
        toff_ns_ = 0;
    }

    // -----------------------------
    // Push data (thread-safe)
    // -----------------------------
    // If you already have a ZUPT flag, call set_zupt() before pushing.
    void push_encoder(int64_t t_ns, double yawrate)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        if (zupt_) return;                 // Hold during ZUPT
        if (!std::isfinite(yawrate)) return;

        // Ignore near-zero yaw rate samples to reduce ambiguity.
        if (std::abs(yawrate) < w_min_push_)
            return;

        enc_.push_back({t_ns, yawrate});
        prune_locked(enc_);
    }

    void push_vio(int64_t t_ns, double yawrate)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        if (zupt_) return;                 // Hold during ZUPT
        if (!std::isfinite(yawrate)) return;

        // Ignore near-zero yaw rate samples to reduce ambiguity.
        if (std::abs(yawrate) < w_min_push_)
            return;

        vio_.push_back({t_ns, yawrate});
        prune_locked(vio_);
    }

    // -----------------------------
    // Estimate (thread-safe)
    // -----------------------------
    // Returns a full diagnostic result. If result.accepted==true,
    // result.toff_ns is the updated toff.
    Result estimate()
    {
        // Copy buffers and params under lock
        std::deque<Sample> enc_copy, vio_copy;

        int64_t max_lag_ns, step_ns;
        double  min_energy;
        double  min_corr, min_peak_sep;
        size_t  min_used_pairs;
        int64_t jump_limit_ns;
        double  alpha;
        bool    hold;
        bool    zupt;

        bool    follow;
        int64_t local_search_ns;
        int64_t max_step_ns;
        int     peak_gap_steps;
        double  high_corr_allow_ambig;

        int64_t prev_toff_ns = 0;
        bool    had_prev = false;

        {
            std::lock_guard<std::mutex> lk(mtx_);

            zupt = zupt_;
            hold = hold_mode_;

            follow           = follow_mode_;
            local_search_ns  = local_search_ns_;
            max_step_ns      = max_step_ns_;
            peak_gap_steps   = different_peak_gap_steps_;
            high_corr_allow_ambig = high_corr_allow_ambig_;

            if (zupt) {
                Result r;
                r.ok = false;
                r.accepted = false;
                r.reason = "zupt_hold";
                if (initialized_) {
                    r.ok = true;
                    r.toff_ns = toff_ns_;
                }
                return r;
            }

            if (enc_.size() < 10 || vio_.size() < 10) {
                Result r;
                r.ok = false;
                r.accepted = false;
                r.reason = "not_enough_samples";
                return r;
            }

            enc_copy = enc_;
            vio_copy = vio_;

            max_lag_ns     = max_lag_ns_;
            step_ns        = step_ns_;
            min_energy     = min_energy_;
            min_corr       = min_corr_;
            min_peak_sep   = min_peak_separation_;
            min_used_pairs = min_used_pairs_;
            jump_limit_ns  = jump_limit_ns_;
            alpha          = alpha_;

            had_prev = initialized_;
            prev_toff_ns = toff_ns_;
        }

        if (step_ns <= 0) step_ns = (int64_t)1e6;

        Result res;
        res.ok = true;

        // Motion energy gate: if no excitation, peak becomes ambiguous.
        const double e_enc = signal_energy(enc_copy);
        const double e_vio = signal_energy(vio_copy);
        if (e_enc < min_energy || e_vio < min_energy) {
            res.accepted = false;
            res.reason = "low_energy_hold";
            if (had_prev) res.toff_ns = prev_toff_ns;
            return res;
        }

        // Decide search range
        int64_t search_lo = -max_lag_ns;
        int64_t search_hi =  max_lag_ns;

        // Follow-mode: search only around current estimate, to avoid jumping to a different peak.
        if (had_prev && follow && local_search_ns > 0) {
            search_lo = std::max<int64_t>(-max_lag_ns, prev_toff_ns - local_search_ns);
            search_hi = std::min<int64_t>( max_lag_ns, prev_toff_ns + local_search_ns);
        }

        res.search_lo_ns = search_lo;
        res.search_hi_ns = search_hi;

        // Search for best and second-best correlation peaks within chosen range.
        double  best_corr = -1e30;
        int64_t best_lag  = 0;
        size_t  best_cnt  = 0;

        double  second_corr = -1e30;
        int64_t second_lag  = 0;
        size_t  second_cnt  = 0;

        for (int64_t lag = search_lo; lag <= search_hi; lag += step_ns)
        {
            size_t cnt = 0;
            double c = correlation_at_lag(enc_copy, vio_copy, lag, cnt);

            if (c > best_corr)
            {
                // Demote current best to second
                second_corr = best_corr;
                second_lag  = best_lag;
                second_cnt  = best_cnt;

                best_corr = c;
                best_lag  = lag;
                best_cnt  = cnt;
            }
            else if (c > second_corr)
            {
                second_corr = c;
                second_lag  = lag;
                second_cnt  = cnt;
            }
        }

        res.best_corr      = best_corr;
        res.best_lag_ns    = best_lag;
        res.second_corr    = second_corr;
        res.second_lag_ns  = second_lag;
        res.used_pairs     = best_cnt;

        if (!std::isfinite(best_corr)) {
            res.accepted = false;
            res.reason = "nan_corr";
            if (had_prev) res.toff_ns = prev_toff_ns;
            return res;
        }

        // Acceptance gate 1: overlap count
        if (best_cnt < min_used_pairs) {
            res.accepted = false;
            res.reason = "too_few_pairs";
            if (had_prev) res.toff_ns = prev_toff_ns;
            return res;
        }

        // Acceptance gate 2: absolute correlation threshold
        if (best_corr < min_corr) {
            res.accepted = false;
            res.reason = "corr_too_low";
            if (had_prev) res.toff_ns = prev_toff_ns;
            return res;
        }

        // Acceptance gate 3: peak uniqueness (avoid truly ambiguous peaks)
        // Only reject if:
        //  (a) second peak is close in correlation, AND
        //  (b) second peak is far enough in lag to be a different peak,
        //  AND (c) best_corr is not already very strong.
        if (std::isfinite(second_corr) && min_peak_sep > 0.0)
        {
            const double diff = best_corr - second_corr;
            const int64_t lag_gap = std::llabs(best_lag - second_lag);
            const int64_t diff_peak_gap = (int64_t)peak_gap_steps * step_ns;

            const bool is_strong = (best_corr >= high_corr_allow_ambig);

            if (!is_strong && lag_gap >= diff_peak_gap && diff < min_peak_sep)
            {
                res.accepted = false;
                res.reason = "peak_ambiguous";
                if (had_prev) res.toff_ns = prev_toff_ns;
                return res;
            }
        }

        // Optional sanity gate: if best peak is extremely far from previous (even within local search),
        // reject and hold. In follow-mode with local search, this rarely triggers unless local_search is large.
        if (had_prev && jump_limit_ns > 0) {
            const int64_t jump = std::llabs(best_lag - prev_toff_ns);
            if (jump > jump_limit_ns) {
                res.accepted = false;
                res.reason = "jump_rejected";
                res.toff_ns = prev_toff_ns;
                return res;
            }
        }

        // Optional manual hold
        if (hold) {
            res.accepted = false;
            res.reason = "manual_hold";
            if (had_prev) res.toff_ns = prev_toff_ns;
            return res;
        }

        // ------------------------------------------------------------
        // Follow-mode update (prevents JUMP):
        // Move gradually toward the best_lag (target), limited by max_step_ns per estimate.
        // ------------------------------------------------------------
        const int64_t target = best_lag;
        res.target_lag_ns = target;

        int64_t updated = target;
        int64_t applied_step = 0;

        if (!had_prev) {
            updated = target; // initialize to the best peak at startup
        } else if (follow) {
            const int64_t err = target - prev_toff_ns;

            // Step-limited movement toward target
            int64_t step = err;
            if (max_step_ns > 0) {
                if (step >  max_step_ns) step =  max_step_ns;
                if (step < -max_step_ns) step = -max_step_ns;
            }
            applied_step = step;

            int64_t stepped = prev_toff_ns + step;

            // Optional EWMA smoothing on top of step-limited update
            if (alpha > 0.0) {
                updated = (int64_t)((1.0 - alpha) * (double)prev_toff_ns + alpha * (double)stepped);
            } else {
                updated = stepped;
            }
        } else {
            // Non-follow mode: directly move to target with optional EWMA
            if (alpha > 0.0) {
                updated = (int64_t)((1.0 - alpha) * (double)prev_toff_ns + alpha * (double)target);
            } else {
                updated = target;
            }
        }

        res.applied_step_ns = applied_step;

        // Store updated toff in shared state
        {
            std::lock_guard<std::mutex> lk(mtx_);
            toff_ns_ = updated;
            initialized_ = true;
        }

        res.accepted = true;
        res.reason = "ok";
        res.toff_ns = updated;
        return res;
    }

    // Convenience: returns true only if accepted (updated).
    bool estimate(int64_t& out_toff_ns)
    {
        Result r = estimate();
        if (r.ok && r.accepted) {
            out_toff_ns = r.toff_ns;
            return true;
        }
        return false;
    }

    // Get last toff without updating.
    bool get_last(int64_t& out_toff_ns) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!initialized_) return false;
        out_toff_ns = toff_ns_;
        return true;
    }

private:
    // ----------------------------------
    // internals
    // ----------------------------------
    std::deque<Sample> enc_, vio_;

    // Window/search
    int64_t window_ns_;
    int64_t max_lag_ns_;
    int64_t step_ns_;

    // Motion gates
    double  min_energy_;
    double  w_min_push_;

    // Acceptance gates
    double  min_corr_;
    double  min_peak_separation_;
    size_t  min_used_pairs_;

    // Stability controls
    int64_t jump_limit_ns_;
    double  alpha_;
    bool    hold_mode_;
    bool    zupt_;

    // Follow-mode controls
    bool    follow_mode_;
    int64_t local_search_ns_;
    int64_t max_step_ns_;

    // Peak ambiguity controls
    int     different_peak_gap_steps_;
    double  high_corr_allow_ambig_;

    // Output state
    bool    initialized_ = false;
    int64_t toff_ns_ = 0;

    mutable std::mutex mtx_;

    // ----------------------------------
    // prune old samples (must be called under lock)
    // ----------------------------------
    void prune_locked(std::deque<Sample>& q)
    {
        if (q.empty()) return;
        const int64_t t_latest = q.back().t_ns;
        while (!q.empty() && (t_latest - q.front().t_ns) > window_ns_)
            q.pop_front();
    }

    // ----------------------------------
    // mean square energy (yawrate^2)
    // ----------------------------------
    static double signal_energy(const std::deque<Sample>& q)
    {
        if (q.empty()) return 0.0;
        double e = 0.0;
        size_t n = 0;
        for (const auto& s : q)
        {
            if (!std::isfinite(s.w)) continue;
            e += s.w * s.w;
            n++;
        }
        if (n == 0) return 0.0;
        return e / (double)n;
    }

    // ----------------------------------
    // normalized cross correlation at a given lag
    // corr( enc(t+lag), vio(t) ) with linear interpolation of vio
    // Also returns overlap count in out_cnt.
    // ----------------------------------
    static double correlation_at_lag(const std::deque<Sample>& enc,
                                     const std::deque<Sample>& vio,
                                     int64_t lag_ns,
                                     size_t& out_cnt)
    {
        out_cnt = 0;

        if (enc.size() < 2 || vio.size() < 2)
            return -1e30;

        double sum   = 0.0;
        double sum_e = 0.0;
        double sum_v = 0.0;

        size_t j = 0;

        for (size_t i = 0; i < enc.size(); i++)
        {
            const double we = enc[i].w;
            if (!std::isfinite(we))
                continue;

            const int64_t t = enc[i].t_ns + lag_ns;

            // Advance vio index so that vio[j].t_ns <= t <= vio[j+1].t_ns
            while (j + 1 < vio.size() && vio[j + 1].t_ns < t)
                j++;

            if (j + 1 >= vio.size())
                break;

            const auto& v0 = vio[j];
            const auto& v1 = vio[j + 1];

            const int64_t dt_ns = v1.t_ns - v0.t_ns;
            if (dt_ns <= 0) {
                // Duplicated or non-monotonic timestamps -> skip
                continue;
            }

            const double alpha = (double)(t - v0.t_ns) / (double)dt_ns;
            if (!(alpha >= 0.0 && alpha <= 1.0))
                continue;

            const double w0 = v0.w;
            const double w1 = v1.w;
            if (!std::isfinite(w0) || !std::isfinite(w1))
                continue;

            const double wv = w0 * (1.0 - alpha) + w1 * alpha;

            sum   += we * wv;
            sum_e += we * we;
            sum_v += wv * wv;
            out_cnt++;
        }

        if (out_cnt < 5)
            return -1e30;

        const double denom = std::sqrt(sum_e * sum_v) + 1e-12;
        const double corr  = sum / denom;

        if (!std::isfinite(corr))
            return -1e30;

        return corr;
    }
};

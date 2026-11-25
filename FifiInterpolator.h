// x3 interpolation of interleaved 16-bit I/Q (96 kHz) to interleaved float I/Q (288 kHz), full scale +-1.0.
//
// Polyphase FIR: a 72-tap Kaiser-windowed sinc prototype at the output rate
// (cutoff 48 kHz, beta 5.65), split into 3 phases of 24 taps, each phase
// normalised to unity DC gain. A 24-frame history is kept between calls, so
// the output does not depend on how the input is cut into pieces.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

class FifiInterpolator {
public:
    FifiInterpolator() { designFilter(); reset(); }

    // Forget all history.
    void reset() { m_history.assign(2 * kTapsPerPhase, 0.0f); }

    // frames in -> 3*frames out; out must hold 6*frames floats. Returns 3*frames.
    std::size_t process(const std::int16_t* in, std::size_t frames, float* out)
    {
        const std::size_t hist = kTapsPerPhase;
        m_work.resize(2 * (hist + frames));
        std::copy(m_history.begin(), m_history.end(), m_work.begin());
        for (std::size_t i = 0; i < 2 * frames; ++i) { m_work[2 * hist + i] = in[i] / 32768.0f; }

        for (std::size_t n = 0; n < frames; ++n) {
            const float* newest = &m_work[2 * (hist + n)];   // frame n; older frames lie below it
            for (std::size_t p = 0; p < kPhases; ++p) {
                float i = 0.0f, q = 0.0f;
                for (std::size_t k = 0; k < kTapsPerPhase; ++k) {
                    const float h = m_phase[p][k];
                    i += h * newest[-2 * static_cast<std::ptrdiff_t>(k)];
                    q += h * newest[-2 * static_cast<std::ptrdiff_t>(k) + 1];
                }
                out[2 * (kPhases * n + p)] = i;
                out[2 * (kPhases * n + p) + 1] = q;
            }
        }
        std::copy(m_work.end() - 2 * static_cast<std::ptrdiff_t>(hist), m_work.end(), m_history.begin());
        return kPhases * frames;
    }

private:
    static constexpr std::size_t kPhases = 3;
    static constexpr std::size_t kTapsPerPhase = 24;
    static constexpr std::size_t kTaps = kPhases * kTapsPerPhase;
    static constexpr double kCutoffHz = 48000.0;
    static constexpr double kOutRateHz = 288000.0;
    static constexpr double kBeta = 5.65;

    static double besselI0(double x)
    {
        double sum = 1.0, term = 1.0;
        for (int k = 1; k < 50; ++k) {
            term *= (x / (2.0 * k)) * (x / (2.0 * k));
            sum += term;
        }
        return sum;
    }

    void designFilter()
    {
        const double pi = 3.14159265358979323846;
        const double fc = kCutoffHz / kOutRateHz;   // cycles per output sample
        const double mid = (kTaps - 1) / 2.0;
        std::array<double, kTaps> h{};
        for (std::size_t n = 0; n < kTaps; ++n) {
            const double t = double(n) - mid;
            const double sinc = t == 0.0 ? 2.0 * fc : std::sin(2.0 * pi * fc * t) / (pi * t);
            const double r = t / mid;
            h[n] = sinc * besselI0(kBeta * std::sqrt(std::max(0.0, 1.0 - r * r))) / besselI0(kBeta);
        }
        for (std::size_t p = 0; p < kPhases; ++p) {
            double sum = 0.0;
            for (std::size_t k = 0; k < kTapsPerPhase; ++k) { sum += h[kPhases * k + p]; }
            for (std::size_t k = 0; k < kTapsPerPhase; ++k) { m_phase[p][k] = static_cast<float>(h[kPhases * k + p] / sum); }
        }
    }

    std::array<std::array<float, kTapsPerPhase>, kPhases> m_phase{};
    std::vector<float> m_history;   // last 24 input frames, interleaved
    std::vector<float> m_work;
};

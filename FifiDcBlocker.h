// DC offset estimate for interleaved 16-bit I/Q.
//
// A zero-IF sound-card receiver carries a DC offset at the frequency it is
// tuned to. Each channel has a one-pole tracker of its mean (time constant
// 4800 frames, 50 ms at 96 kHz), fed with the raw samples, which it does not
// change. The estimate starts at the first frame after reset(), so a start
// has no step.
//
// The offset is not a whole number of LSBs, so it is removed after the
// samples are converted to float (subtractFrom): subtracting it from the
// 16-bit samples and rounding back would leave up to half an LSB of DC, a
// clear spike against the FiFi-SDR's noise floor of about 2 LSB.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <cstdint>

class FifiDcBlocker {
public:
    // The next sample (re)initialises the estimate.
    void reset() { m_started = false; }

    // The estimate follows the samples with a time constant of 4800 frames.
    void track(const std::int16_t* iq, std::size_t frames)
    {
        if (frames == 0) { return; }
        if (!m_started) {
            m_dc[0] = iq[0];
            m_dc[1] = iq[1];
            m_started = true;
        }
        for (std::size_t n = 0; n < frames; ++n) {
            for (std::size_t c = 0; c < 2; ++c) { m_dc[c] += kAlpha * (double(iq[2 * n + c]) - m_dc[c]); }
        }
    }

    // The estimate in LSB of the 16-bit samples; 0 before the first frame.
    double dcI() const { return m_started ? m_dc[0] : 0.0; }
    double dcQ() const { return m_started ? m_dc[1] : 0.0; }

    // Subtract the estimate from interleaved float I/Q at full scale +-1.0
    // (sample / 32768). swapped: the frames carry Q first, then I.
    void subtractFrom(float* iq, std::size_t frames, bool swapped) const
    {
        const float a = static_cast<float>((swapped ? dcQ() : dcI()) / 32768.0);
        const float b = static_cast<float>((swapped ? dcI() : dcQ()) / 32768.0);
        for (std::size_t n = 0; n < frames; ++n) {
            iq[2 * n] -= a;
            iq[2 * n + 1] -= b;
        }
    }

private:
    static constexpr double kAlpha = 1.0 / 4800.0;
    double m_dc[2]{0.0, 0.0};
    bool m_started{false};
};

// Test of FifiInterpolator (x3, 96 kHz -> 288 kHz). Plain main: prints PASS/FAIL, exits non-zero on failure.
// Builds alone: g++ -std=c++17 -O2 fifi_interpolator_test.cpp
// SPDX-License-Identifier: GPL-3.0-or-later
#include "FifiInterpolator.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInRate = 96000.0;
constexpr double kOutRate = 288000.0;
constexpr std::size_t kSkip = 200;
int g_failures = 0;

void check(bool ok, const char* what, double value, const char* unit)
{
    std::printf("%s: %s (%.3f %s)\n", ok ? "PASS" : "FAIL", what, value, unit);
    if (!ok) { ++g_failures; }
}

std::vector<std::int16_t> tone(double hz, double amplitude, std::size_t frames)
{
    std::vector<std::int16_t> x(2 * frames);
    for (std::size_t n = 0; n < frames; ++n) {
        const double ph = 2.0 * kPi * hz / kInRate * double(n);
        x[2 * n] = static_cast<std::int16_t>(std::lround(amplitude * std::cos(ph)));
        x[2 * n + 1] = static_cast<std::int16_t>(std::lround(amplitude * std::sin(ph)));
    }
    return x;
}

// Amplitude (full scale 1.0) of the component at hz in interleaved output, after the skip. Hann window.
double amplitudeAt(const std::vector<float>& y, double hz)
{
    const std::size_t frames = y.size() / 2;
    std::complex<double> acc{0, 0};
    double wsum = 0;
    const std::size_t n = frames - kSkip;
    for (std::size_t k = 0; k < n; ++k) {
        const double w = 0.5 - 0.5 * std::cos(2.0 * kPi * double(k) / double(n));
        const std::size_t i = k + kSkip;
        const double ph = -2.0 * kPi * hz / kOutRate * double(i);
        acc += w * std::complex<double>(y[2 * i], y[2 * i + 1]) * std::complex<double>(std::cos(ph), std::sin(ph));
        wsum += w;
    }
    return std::abs(acc) / wsum;
}

double db(double ratio) { return 20.0 * std::log10(ratio + 1e-30); }

std::vector<float> run(const std::vector<std::int16_t>& x)
{
    FifiInterpolator f;
    std::vector<float> y(6 * (x.size() / 2));
    f.process(x.data(), x.size() / 2, y.data());
    return y;
}

} // namespace

int main()
{
    const double expected = 8000.0 / 32768.0;
    {
        const auto x = tone(10000.0, 8000.0, 4096);
        FifiInterpolator f;
        std::vector<float> y(6 * 4096);
        const std::size_t n = f.process(x.data(), 4096, y.data());
        check(n == 12288, "4096 frames in give 12288 out", double(n), "frames");

        const double a = amplitudeAt(y, 10000.0);
        check(std::fabs(db(a / expected)) <= 0.5, "+10 kHz tone amplitude within 0.5 dB", db(a / expected), "dB");
        check(db(amplitudeAt(y, -86000.0) / a) <= -55.0, "image at -86 kHz at least 55 dB down", db(amplitudeAt(y, -86000.0) / a), "dB");
        check(db(amplitudeAt(y, 106000.0) / a) <= -55.0, "image at +106 kHz at least 55 dB down", db(amplitudeAt(y, 106000.0) / a), "dB");
    }
    {
        const auto x = tone(40000.0, 8000.0, 4096);
        const auto y = run(x);
        const double a = amplitudeAt(y, 40000.0);
        check(std::fabs(db(a / expected)) <= 1.0, "+40 kHz tone amplitude within 1 dB", db(a / expected), "dB");
        check(db(amplitudeAt(y, -56000.0) / a) <= -50.0, "image at -56 kHz at least 50 dB down", db(amplitudeAt(y, -56000.0) / a), "dB");
    }
    {
        const auto x = tone(10000.0, 8000.0, 4096);
        const auto whole = run(x);
        FifiInterpolator f;
        std::vector<float> pieces(whole.size());
        std::size_t done = 0;
        for (std::size_t pos = 0; pos < 4096; pos += 7) {
            const std::size_t n = std::min<std::size_t>(7, 4096 - pos);
            done += f.process(x.data() + 2 * pos, n, pieces.data() + 2 * done);
        }
        double worst = 0;
        for (std::size_t i = 0; i < whole.size(); ++i) { worst = std::max(worst, double(std::fabs(whole[i] - pieces[i]))); }
        check(done == 12288 && worst <= 1e-6, "pieces of 7 frames equal one call", worst, "max abs diff");
    }
    {
        const auto x = tone(10000.0, 8000.0, 512);
        const auto fresh = run(x);
        FifiInterpolator f;
        std::vector<float> junk(6 * 512), again(6 * 512);
        const auto other = tone(33000.0, 20000.0, 512);
        f.process(other.data(), 512, junk.data());
        f.reset();
        f.process(x.data(), 512, again.data());
        double worst = 0;
        for (std::size_t i = 0; i < fresh.size(); ++i) { worst = std::max(worst, double(std::fabs(fresh[i] - again[i]))); }
        check(worst <= 1e-6, "reset equals a new object", worst, "max abs diff");
    }
    std::printf("%s\n", g_failures == 0 ? "ALL PASS" : "FAILURES");
    return g_failures == 0 ? 0 : 1;
}

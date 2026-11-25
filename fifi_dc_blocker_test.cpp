// Test of FifiDcBlocker (DC offset estimate on 16-bit I/Q, removed in float). Plain main: prints PASS/FAIL, exits non-zero on failure.
// Builds alone: g++ -std=c++17 -O2 -Wall -Wextra fifi_dc_blocker_test.cpp
// SPDX-License-Identifier: GPL-3.0-or-later
#include "FifiDcBlocker.h"
#include "FifiInterpolator.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRate = 96000.0;
int g_failures = 0;

void check(bool ok, const char* what, double value, const char* unit)
{
    std::printf("%s: %s (%.3f %s)\n", ok ? "PASS" : "FAIL", what, value, unit);
    if (!ok) { ++g_failures; }
}

// Tone of the given amplitude at hz plus a fixed offset per channel, interleaved.
std::vector<std::int16_t> toneWithOffset(double hz, double amplitude, double offsetI, double offsetQ, std::size_t frames)
{
    std::vector<std::int16_t> x(2 * frames);
    for (std::size_t n = 0; n < frames; ++n) {
        const double ph = 2.0 * kPi * hz / kRate * double(n);
        x[2 * n] = static_cast<std::int16_t>(std::lround(offsetI + amplitude * std::cos(ph)));
        x[2 * n + 1] = static_cast<std::int16_t>(std::lround(offsetQ + amplitude * std::sin(ph)));
    }
    return x;
}

// Amplitude (in LSB) of the component at hz in frames [from, end) of float I/Q at full scale +-1.0. Hann window.
double amplitudeAt(const std::vector<float>& y, double hz, std::size_t from)
{
    const std::size_t frames = y.size() / 2;
    const std::size_t n = frames - from;
    std::complex<double> acc{0, 0};
    double wsum = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const double w = 0.5 - 0.5 * std::cos(2.0 * kPi * double(k) / double(n));
        const std::size_t i = from + k;
        const double ph = -2.0 * kPi * hz / kRate * double(i);
        acc += w * 32768.0 * std::complex<double>(y[2 * i], y[2 * i + 1]) * std::complex<double>(std::cos(ph), std::sin(ph));
        wsum += w;
    }
    return std::abs(acc) / wsum;
}

// Largest difference between the estimates of two blockers, in LSB.
double estimateDiff(const FifiDcBlocker& a, const FifiDcBlocker& b)
{
    return std::max(std::fabs(a.dcI() - b.dcI()), std::fabs(a.dcQ() - b.dcQ()));
}

// In-place radix-2 FFT (n a power of two).
void fft(std::vector<std::complex<double>>& a)
{
    const std::size_t n = a.size();
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        std::size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) { j ^= bit; }
        j ^= bit;
        if (i < j) { std::swap(a[i], a[j]); }
    }
    for (std::size_t len = 2; len <= n; len <<= 1) {
        const std::complex<double> wl(std::cos(-2.0 * kPi / double(len)), std::sin(-2.0 * kPi / double(len)));
        for (std::size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (std::size_t k = 0; k < len / 2; ++k) {
                const std::complex<double> u = a[i + k];
                const std::complex<double> v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

// The 288000 path of the driver: 96000 frames of I = 300.4, Q = -200.3 LSB plus Gaussian noise of 2.3 LSB rms
// per channel, read in pieces of 341 frames, DC removed and interpolated x3. Output in LSB (x 32768).
std::vector<std::complex<double>> driverPath288(std::size_t frames)
{
    std::mt19937 rng(12345);
    std::normal_distribution<double> noise(0.0, 2.3);
    FifiDcBlocker blocker;
    FifiInterpolator interpolator;
    const std::size_t chunk = 341;
    std::vector<std::int16_t> raw(2 * chunk);
    std::vector<float> up(6 * chunk);
    std::vector<std::complex<double>> out;
    out.reserve(3 * frames);
    for (std::size_t done = 0; done < frames; done += chunk) {
        const std::size_t n = std::min(chunk, frames - done);
        for (std::size_t k = 0; k < n; ++k) {
            raw[2 * k] = static_cast<std::int16_t>(std::lround(300.4 + noise(rng)));
            raw[2 * k + 1] = static_cast<std::int16_t>(std::lround(-200.3 + noise(rng)));
        }
        blocker.track(raw.data(), n);
        const std::size_t outFrames = interpolator.process(raw.data(), n, up.data());
        blocker.subtractFrom(up.data(), outFrames, false);
        for (std::size_t k = 0; k < outFrames; ++k) { out.emplace_back(up[2 * k] * 32768.0, up[2 * k + 1] * 32768.0); }
    }
    return out;
}

// The 96000 CF32 path of the driver: estimate, convert, subtract. Full scale +-1.0.
std::vector<float> run(const std::vector<std::int16_t>& x)
{
    FifiDcBlocker b;
    b.track(x.data(), x.size() / 2);
    std::vector<float> y(x.size());
    for (std::size_t i = 0; i < x.size(); ++i) { y[i] = x[i] / 32768.0f; }
    b.subtractFrom(y.data(), x.size() / 2, false);
    return y;
}

} // namespace

int main()
{
    // DC offset with a fraction of an LSB, under noise, through the interpolator as the driver does at 288000.
    {
        const std::size_t frames = 480000;
        const auto y = driverPath288(frames);
        const std::size_t skip = 3 * 96000;
        double sumI = 0, sumQ = 0;
        for (std::size_t n = skip; n < y.size(); ++n) { sumI += y[n].real(); sumQ += y[n].imag(); }
        const double meanI = sumI / double(y.size() - skip);
        const double meanQ = sumQ / double(y.size() - skip);
        check(std::fabs(meanI) <= 0.05, "I mean of the 288000 output within 0.05 LSB after 96000 frames", meanI, "LSB");
        check(std::fabs(meanQ) <= 0.05, "Q mean of the 288000 output within 0.05 LSB after 96000 frames", meanQ, "LSB");
        // 4096-point Hann periodograms, averaged (one periodogram's bins scatter by several dB).
        const std::size_t n = 4096;
        std::vector<double> power(n, 0.0);
        std::vector<std::complex<double>> block(n);
        std::size_t blocks = 0;
        for (std::size_t from = skip; from + n <= y.size(); from += n, ++blocks) {
            for (std::size_t k = 0; k < n; ++k) { block[k] = y[from + k] * (0.5 - 0.5 * std::cos(2.0 * kPi * double(k) / double(n))); }
            fft(block);
            for (std::size_t k = 0; k < n; ++k) { power[k] += std::norm(block[k]); }
        }
        std::vector<double> inBand;   // bin k is k * 288000 / 4096 Hz from the centre (negative above n/2)
        for (std::size_t k = 0; k < n; ++k) {
            const double hz = (k < n / 2 ? double(k) : double(k) - double(n)) * 288000.0 / double(n);
            if (std::fabs(hz) < 40000.0) { inBand.push_back(power[k]); }
        }
        std::nth_element(inBand.begin(), inBand.begin() + inBand.size() / 2, inBand.end());
        const double centreDb = 10.0 * std::log10(power[0] / inBand[inBand.size() / 2]);
        std::printf("averaged %zu periodograms\n", blocks);
        check(std::fabs(centreDb) <= 3.0, "the centre bin of the 288000 output within 3 dB of the median floor (+-40 kHz)", centreDb, "dB");
    }
    {
        const auto x = toneWithOffset(0.0, 0.0, 300.4, -200.3, 1);
        FifiDcBlocker b;
        check(b.dcI() == 0.0 && b.dcQ() == 0.0, "no estimate before the first frame", b.dcI(), "LSB");
        b.track(x.data(), 1);
        check(b.dcI() == double(x[0]) && b.dcQ() == double(x[1]), "the estimate starts at the first frame", b.dcI(), "LSB");
        b.track(x.data(), 0);
        check(b.dcI() == double(x[0]), "an empty read leaves the estimate", b.dcI(), "LSB");
    }
    {
        const std::size_t frames = 96000;
        const std::size_t skip = 48000;
        const auto y = run(toneWithOffset(10000.0, 8000.0, 300.0, -200.0, frames));
        const double db = 20.0 * std::log10(amplitudeAt(y, 10000.0, skip) / 8000.0);
        check(std::fabs(db) <= 0.1, "+10 kHz tone amplitude within 0.1 dB", db, "dB");
    }
    {
        const auto x = toneWithOffset(10000.0, 8000.0, 300.4, -200.3, 9600);
        FifiDcBlocker whole;
        whole.track(x.data(), 9600);
        FifiDcBlocker pieces;
        for (std::size_t pos = 0; pos < 9600; pos += 7) { pieces.track(x.data() + 2 * pos, std::min<std::size_t>(7, 9600 - pos)); }
        const double worst = estimateDiff(whole, pieces);
        check(worst == 0.0, "pieces of 7 frames equal one call", worst, "LSB");
    }
    {
        const auto x = toneWithOffset(10000.0, 8000.0, 300.4, -200.3, 4800);
        FifiDcBlocker fresh;
        fresh.track(x.data(), 4800);
        const auto other = toneWithOffset(33000.0, 20000.0, -900.0, 700.0, 4800);
        FifiDcBlocker b;
        b.track(other.data(), 4800);
        b.reset();
        b.track(x.data(), 4800);
        const double worst = estimateDiff(fresh, b);
        check(worst == 0.0, "reset equals a new object", worst, "LSB");
    }
    {
        const auto x = toneWithOffset(0.0, 0.0, 300.4, -200.3, 4800);
        FifiDcBlocker b;
        b.track(x.data(), 4800);
        float plain[2] = {0.0f, 0.0f};
        float swapped[2] = {0.0f, 0.0f};
        b.subtractFrom(plain, 1, false);
        b.subtractFrom(swapped, 1, true);
        check(plain[0] == swapped[1] && plain[1] == swapped[0] && plain[0] < 0.0f && plain[1] > 0.0f,
              "swapped I/Q gets the swapped estimate", double(plain[0]) * 32768.0, "LSB");
    }
    std::printf("%s\n", g_failures == 0 ? "ALL PASS" : "FAILURES");
    return g_failures == 0 ? 0 : 1;
}

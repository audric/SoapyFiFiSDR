// SoapySDR module for the FiFi-SDR receiver (driver name "fifisdr").
//
// The FiFi-SDR is a USB sound card that delivers I/Q (left = I, right = Q,
// 16 bit, 48 or 96 kHz) and a SoftRock-style USB control channel for tuning.
// The stream offers 48000, 96000 and 288000 S/s; 288000 is the sound card at
// 96000 interpolated x3 (FifiInterpolator.h), so a receiver that needs a wider
// capture than 96 kHz can use it. At 288000 the passband is limited by the
// card's 96 kHz sampling, not by the interpolation.
// The sound card's DC offset is removed (FifiDcBlocker.h) at every rate unless
// it is switched off with setDCOffsetMode or the device argument dc_removal=0:
// it is estimated on the raw samples and subtracted after the conversion to
// float (at 288000 after the interpolation), so CF32 at every rate and CS16 at
// 288000 keep no DC. CS16 at 48000 and 96000 has the estimate subtracted
// rounded to whole LSBs, which leaves up to half an LSB of DC.
// This module captures from the sound card with ALSA and tunes with libusb
// vendor requests on the device (no interface is claimed, so the kernel's
// audio driver stays attached). Linux only. One FiFi-SDR per computer.
//
// Control requests follow Hamlib's rigs/kit/fifisdr.c:
//   0x32 out, 4 bytes LE: frequency in MHz x multiplier, 11.21 fixed point
//   0x3A in,  4 bytes LE: the same, read back
//   0xAB in,  index 11, 4 bytes LE: multiplier (normally 4); index 19: preamp
//   0xAC out, index 19, 1 byte: preamp, 0 = 0 dB, 1 = +6 dB
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>

#include <alsa/asoundlib.h>
#include <libusb.h>

#include "FifiDcBlocker.h"
#include "FifiInterpolator.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint16_t kVid = 0x16C0;
constexpr uint16_t kPid = 0x05DC;
constexpr const char* kProduct = "FiFi-SDR";
constexpr uint8_t kSetFreq = 0x32;
constexpr uint8_t kReadFreq = 0x3A;
constexpr uint8_t kFifiRead = 0xAB;
constexpr uint8_t kFifiWrite = 0xAC;
constexpr uint16_t kIndexMultiplier = 11;
constexpr uint16_t kIndexPreamp = 19;
constexpr unsigned kUsbTimeoutMs = 1000;
constexpr double kMinHz = 200e3;
constexpr double kMaxHz = 30e6;
constexpr size_t kMtu = 1024;   // frames per read
constexpr int kWaitSliceMs = 20;   // longest time readStream holds the sound card lock

uint32_t fromLe32(const unsigned char* b)
{
    return uint32_t(b[0]) | uint32_t(b[1]) << 8 | uint32_t(b[2]) << 16 | uint32_t(b[3]) << 24;
}

// The first FiFi-SDR on the bus, opened; nullptr when there is none or it
// cannot be opened. serial is filled when the device reports one.
libusb_device_handle* openFifi(libusb_context* ctx, std::string* serial)
{
    libusb_device** list = nullptr;
    const ssize_t n = libusb_get_device_list(ctx, &list);
    libusb_device_handle* found = nullptr;
    for (ssize_t i = 0; i < n && !found; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(list[i], &desc) != 0) { continue; }
        if (desc.idVendor != kVid || desc.idProduct != kPid) { continue; }
        libusb_device_handle* h = nullptr;
        if (libusb_open(list[i], &h) != 0) { continue; }
        unsigned char text[128] = {0};
        libusb_get_string_descriptor_ascii(h, desc.iProduct, text, sizeof(text));
        if (std::strcmp(reinterpret_cast<const char*>(text), kProduct) != 0) {
            libusb_close(h);   // another device on the shared VID/PID
            continue;
        }
        if (serial && desc.iSerialNumber) {
            unsigned char sn[128] = {0};
            if (libusb_get_string_descriptor_ascii(h, desc.iSerialNumber, sn, sizeof(sn)) > 0) {
                *serial = reinterpret_cast<const char*>(sn);
            }
        }
        found = h;
    }
    if (list) { libusb_free_device_list(list, 1); }
    return found;
}

// "hw:<n>,0" of the FiFi-SDR's sound card; empty when there is none.
std::string fifiPcmName()
{
    int card = -1;
    while (snd_card_next(&card) == 0 && card >= 0) {
        char* name = nullptr;
        if (snd_card_get_name(card, &name) != 0 || !name) { continue; }
        const bool match = std::strstr(name, kProduct) != nullptr;
        free(name);
        if (match) { return "hw:" + std::to_string(card) + ",0"; }
    }
    return {};
}

class SoapyFifiSdr : public SoapySDR::Device
{
public:
    explicit SoapyFifiSdr(const SoapySDR::Kwargs& args)
    {
        if (args.count("iq_swap")) { m_iqSwap = args.at("iq_swap") == "true" || args.at("iq_swap") == "1"; }
        if (args.count("dc_removal")) { m_dcRemoval = !(args.at("dc_removal") == "false" || args.at("dc_removal") == "0"); }
        if (libusb_init(&m_usb) != 0) { throw std::runtime_error("fifisdr: libusb_init failed"); }
        m_handle = openFifi(m_usb, &m_serial);
        if (!m_handle) {
            libusb_exit(m_usb);
            throw std::runtime_error("fifisdr: no FiFi-SDR found on USB, or no permission to open it");
        }
        m_pcmName = fifiPcmName();
        if (m_pcmName.empty()) {
            libusb_close(m_handle);
            libusb_exit(m_usb);
            throw std::runtime_error("fifisdr: the FiFi-SDR's sound card was not found");
        }
        unsigned char b[4] = {0};
        if (controlIn(kFifiRead, kIndexMultiplier, b, 4) == 4) {
            const uint32_t m = fromLe32(b);
            if (m >= 1 && m <= 16) { m_multiplier = m; }
        }
        if (controlIn(kReadFreq, 0, b, 4) == 4) { m_frequency = fromLe32(b) / 2097152.0 * 1e6 / m_multiplier; }
        unsigned char p = 0;
        if (controlIn(kFifiRead, kIndexPreamp, &p, 1) == 1) { m_preampDb = p ? 6.0 : 0.0; }
    }

    ~SoapyFifiSdr() override
    {
        if (m_pcm) { snd_pcm_close(m_pcm); }
        libusb_close(m_handle);
        libusb_exit(m_usb);
    }

    std::string getDriverKey() const override { return "fifisdr"; }
    std::string getHardwareKey() const override { return "FiFi-SDR"; }
    SoapySDR::Kwargs getHardwareInfo() const override
    {
        SoapySDR::Kwargs info;
        info["serial"] = m_serial;
        info["multiplier"] = std::to_string(m_multiplier);
        info["pcm"] = m_pcmName;
        unsigned char b[4] = {0};
        if (controlIn(kFifiRead, 0, b, 4) == 4) { info["firmware"] = std::to_string(fromLe32(b)); }
        return info;
    }

    size_t getNumChannels(const int direction) const override { return direction == SOAPY_SDR_RX ? 1 : 0; }
    std::vector<std::string> listAntennas(const int, const size_t) const override { return {"RX"}; }
    std::string getAntenna(const int, const size_t) const override { return "RX"; }

    std::vector<std::string> listGains(const int, const size_t) const override { return {"PREAMP"}; }
    SoapySDR::Range getGainRange(const int, const size_t, const std::string&) const override
    {
        return SoapySDR::Range(0.0, 6.0, 6.0);
    }
    SoapySDR::Range getGainRange(const int, const size_t) const override { return SoapySDR::Range(0.0, 6.0, 6.0); }
    void setGain(const int direction, const size_t channel, const double value) override
    {
        setGain(direction, channel, "PREAMP", value);
    }
    void setGain(const int, const size_t, const std::string& name, const double value) override
    {
        if (name != "PREAMP") { throw std::runtime_error("fifisdr: unknown gain " + name); }
        unsigned char p = value >= 3.0 ? 1 : 0;
        if (controlOut(kFifiWrite, kIndexPreamp, &p, 1) != 1) { throw std::runtime_error("fifisdr: setting the preamp failed"); }
        m_preampDb = p ? 6.0 : 0.0;
    }
    double getGain(const int, const size_t) const override { return m_preampDb; }
    double getGain(const int, const size_t, const std::string&) const override { return m_preampDb; }

    bool hasDCOffsetMode(const int direction, const size_t) const override { return direction == SOAPY_SDR_RX; }
    void setDCOffsetMode(const int, const size_t, const bool automatic) override { m_dcRemoval = automatic; }
    bool getDCOffsetMode(const int, const size_t) const override { return m_dcRemoval; }

    void setFrequency(const int, const size_t, const std::string& name, const double frequency,
                      const SoapySDR::Kwargs&) override
    {
        if (name != "RF") { return; }
        const double hz = std::clamp(frequency, kMinHz, kMaxHz);
        const uint32_t value = static_cast<uint32_t>(std::llround(hz * m_multiplier / 1e6 * 2097152.0));
        unsigned char b[4] = {static_cast<unsigned char>(value), static_cast<unsigned char>(value >> 8),
                              static_cast<unsigned char>(value >> 16), static_cast<unsigned char>(value >> 24)};
        if (controlOut(kSetFreq, 0, b, 4) != 4) { throw std::runtime_error("fifisdr: setting the frequency failed"); }
        m_frequency = hz;
    }
    double getFrequency(const int, const size_t, const std::string& name) const override
    {
        return name == "RF" ? m_frequency.load() : 0.0;
    }
    std::vector<std::string> listFrequencies(const int, const size_t) const override { return {"RF"}; }
    SoapySDR::RangeList getFrequencyRange(const int, const size_t, const std::string& name) const override
    {
        return name == "RF" ? SoapySDR::RangeList{SoapySDR::Range(kMinHz, kMaxHz)} : SoapySDR::RangeList{};
    }

    std::vector<double> listSampleRates(const int, const size_t) const override { return {48000.0, 96000.0, 288000.0}; }
    SoapySDR::RangeList getSampleRateRange(const int, const size_t) const override
    {
        return {SoapySDR::Range(48000.0, 48000.0), SoapySDR::Range(96000.0, 96000.0), SoapySDR::Range(288000.0, 288000.0)};
    }
    void setSampleRate(const int, const size_t, const double rate) override
    {
        const unsigned r = rate < 72000.0 ? 48000 : rate < 192000.0 ? 96000 : 288000;
        const PcmLock lock(*this);
        if (m_pcm && r != m_rate) { throw std::runtime_error("fifisdr: the sample rate cannot change while streaming"); }
        m_rate = r;
    }
    double getSampleRate(const int, const size_t) const override { return m_rate; }

    std::vector<std::string> getStreamFormats(const int, const size_t) const override
    {
        return {SOAPY_SDR_CS16, SOAPY_SDR_CF32};
    }
    std::string getNativeStreamFormat(const int, const size_t, double& fullScale) const override
    {
        fullScale = 32768.0;
        return SOAPY_SDR_CS16;
    }

    SoapySDR::Stream* setupStream(const int direction, const std::string& format, const std::vector<size_t>& channels,
                                  const SoapySDR::Kwargs&) override
    {
        if (direction != SOAPY_SDR_RX) { throw std::runtime_error("fifisdr: receive only"); }
        if (channels.size() > 1 || (channels.size() == 1 && channels[0] != 0)) {
            throw std::runtime_error("fifisdr: only channel 0 exists");
        }
        if (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32) {
            throw std::runtime_error("fifisdr: stream format must be CS16 or CF32");
        }
        const PcmLock lock(*this);
        if (m_pcm) { throw std::runtime_error("fifisdr: a stream is already open"); }
        snd_pcm_t* pcm = nullptr;
        int err = snd_pcm_open(&pcm, m_pcmName.c_str(), SND_PCM_STREAM_CAPTURE, 0);
        if (err < 0) { throw std::runtime_error(std::string("fifisdr: opening the sound card failed: ") + snd_strerror(err)); }
        // 2 channels, 16 bit, no resampling, about 0.1 s of buffer
        err = snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED, 2, cardRate(), 0, 100000);
        if (err < 0) {
            snd_pcm_close(pcm);
            throw std::runtime_error(std::string("fifisdr: configuring the sound card failed: ") + snd_strerror(err));
        }
        m_pcm = pcm;
        m_float = format == SOAPY_SDR_CF32;
        m_raw.resize(kMtu * 2);
        return reinterpret_cast<SoapySDR::Stream*>(this);
    }

    void closeStream(SoapySDR::Stream*) override
    {
        const PcmLock lock(*this);
        if (m_pcm) { snd_pcm_close(m_pcm); m_pcm = nullptr; }
        m_active = false;
    }

    size_t getStreamMTU(SoapySDR::Stream*) const override { return kMtu; }

    int activateStream(SoapySDR::Stream*, const int, const long long, const size_t) override
    {
        const PcmLock lock(*this);
        if (!m_pcm) { return SOAPY_SDR_STREAM_ERROR; }
        snd_pcm_prepare(m_pcm);
        snd_pcm_start(m_pcm);
        m_interpolator.reset();
        m_dcBlocker.reset();
        m_active = true;
        return 0;
    }

    int deactivateStream(SoapySDR::Stream*, const int, const long long) override
    {
        const PcmLock lock(*this);
        if (m_pcm) { snd_pcm_drop(m_pcm); }
        m_active = false;
        return 0;
    }

    int readStream(SoapySDR::Stream*, void* const* buffs, const size_t numElems, int& flags, long long&,
                   const long timeoutUs) override
    {
        flags = 0;
        // The caller reads in a tight loop. Hold the lock for one short slice
        // at a time and stand back while a control call is waiting for it.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(std::max(1000L, timeoutUs));
        for (;;) {
            while (m_controlWaiting.load() > 0) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
            const int ret = readSlice(buffs, numElems);
            if (ret != SOAPY_SDR_TIMEOUT || std::chrono::steady_clock::now() >= deadline) { return ret; }
        }
    }

private:
    // RAII lock for calls that open, close, start or stop the sound card.
    struct PcmLock {
        explicit PcmLock(SoapyFifiSdr& d) : dev(d) { ++dev.m_controlWaiting; dev.m_pcmMutex.lock(); --dev.m_controlWaiting; }
        ~PcmLock() { dev.m_pcmMutex.unlock(); }
        SoapyFifiSdr& dev;
    };

    // One wait of at most kWaitSliceMs and one read, under the lock.
    int readSlice(void* const* buffs, const size_t numElems)
    {
        std::lock_guard<std::mutex> lock(m_pcmMutex);
        if (!m_pcm) {
            // the device is gone: wait one slice so a caller that keeps reading does not spin
            m_pcmMutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(kWaitSliceMs));
            m_pcmMutex.lock();
            return SOAPY_SDR_STREAM_ERROR;
        }
        if (!m_active) {
            m_pcmMutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(kWaitSliceMs));
            m_pcmMutex.lock();
            return SOAPY_SDR_TIMEOUT;
        }
        const bool interpolate = m_rate == 288000;
        if (interpolate && numElems < 3) {
            // no room for the 3 frames one card frame becomes: wait one slice, do not spin
            m_pcmMutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(kWaitSliceMs));
            m_pcmMutex.lock();
            return SOAPY_SDR_TIMEOUT;
        }
        const int ready = snd_pcm_wait(m_pcm, kWaitSliceMs);
        if (ready == 0) { return SOAPY_SDR_TIMEOUT; }
        size_t want = std::min(numElems, kMtu);
        if (interpolate) { want = want / 3; }
        snd_pcm_sframes_t got = ready < 0 ? ready : snd_pcm_readi(m_pcm, m_raw.data(), want);
        if (got == -EAGAIN) { return SOAPY_SDR_TIMEOUT; }
        if (got < 0) {
            // overrun or suspend: restart the capture and report the gap
            if (snd_pcm_recover(m_pcm, static_cast<int>(got), 1) < 0) { return SOAPY_SDR_STREAM_ERROR; }
            snd_pcm_start(m_pcm);
            m_interpolator.reset();
            m_dcBlocker.reset();
            return SOAPY_SDR_OVERFLOW;
        }
        // The estimate is taken on the samples as the card delivers them (I, Q)
        // and subtracted in float, after the swap; see FifiDcBlocker.h.
        const bool dcRemoval = m_dcRemoval;
        if (dcRemoval) { m_dcBlocker.track(m_raw.data(), static_cast<size_t>(got)); }
        const size_t i0 = m_iqSwap ? 1 : 0;
        const size_t q0 = m_iqSwap ? 0 : 1;
        if (interpolate) {
            if (m_iqSwap) {
                for (snd_pcm_sframes_t k = 0; k < got; ++k) { std::swap(m_raw[2 * k], m_raw[2 * k + 1]); }
            }
            m_up.resize(6 * static_cast<size_t>(got));
            const size_t outFrames = m_interpolator.process(m_raw.data(), static_cast<size_t>(got), m_up.data());
            // each phase of the interpolator has unity DC gain: the offset is the same after it
            if (dcRemoval) { m_dcBlocker.subtractFrom(m_up.data(), outFrames, m_iqSwap); }
            if (m_float) {
                std::memcpy(buffs[0], m_up.data(), outFrames * 2 * sizeof(float));
            } else {
                auto* out = static_cast<int16_t*>(buffs[0]);
                for (size_t k = 0; k < outFrames * 2; ++k) {
                    out[k] = static_cast<int16_t>(std::clamp(std::lround(m_up[k] * 32768.0f), -32768L, 32767L));
                }
            }
            return static_cast<int>(outFrames);
        }
        if (m_float) {
            auto* out = static_cast<float*>(buffs[0]);
            for (snd_pcm_sframes_t k = 0; k < got; ++k) {
                out[2 * k] = m_raw[2 * k + i0] / 32768.0f;
                out[2 * k + 1] = m_raw[2 * k + q0] / 32768.0f;
            }
            if (dcRemoval) { m_dcBlocker.subtractFrom(out, static_cast<size_t>(got), m_iqSwap); }
        } else {
            // whole LSBs only: up to half an LSB of DC stays (the app does not use these rates)
            const long dc[2] = {dcRemoval ? std::lround(m_dcBlocker.dcI()) : 0L, dcRemoval ? std::lround(m_dcBlocker.dcQ()) : 0L};
            auto* out = static_cast<int16_t*>(buffs[0]);
            for (snd_pcm_sframes_t k = 0; k < got; ++k) {
                out[2 * k] = static_cast<int16_t>(std::clamp(long(m_raw[2 * k + i0]) - dc[i0], -32768L, 32767L));
                out[2 * k + 1] = static_cast<int16_t>(std::clamp(long(m_raw[2 * k + q0]) - dc[q0], -32768L, 32767L));
            }
        }
        return static_cast<int>(got);
    }

    int controlIn(uint8_t request, uint16_t index, unsigned char* data, uint16_t length) const
    {
        std::lock_guard<std::mutex> lock(m_usbMutex);
        return libusb_control_transfer(m_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN,
                                       request, 0, index, data, length, kUsbTimeoutMs);
    }
    int controlOut(uint8_t request, uint16_t index, unsigned char* data, uint16_t length)
    {
        std::lock_guard<std::mutex> lock(m_usbMutex);
        return libusb_control_transfer(m_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT,
                                       request, 0, index, data, length, kUsbTimeoutMs);
    }

    libusb_context* m_usb{nullptr};
    libusb_device_handle* m_handle{nullptr};
    mutable std::mutex m_usbMutex;
    std::string m_serial;
    std::string m_pcmName;
    uint32_t m_multiplier{4};
    std::atomic<double> m_frequency{0.0};   // written by RPC threads, read by any
    std::atomic<double> m_preampDb{0.0};
    bool m_iqSwap{false};
    std::atomic<bool> m_dcRemoval{true};   // read on the stream thread, written by RPC threads

    std::mutex m_pcmMutex;
    std::atomic<int> m_controlWaiting{0};
    bool m_active{false};   // guarded by m_pcmMutex
    snd_pcm_t* m_pcm{nullptr};
    unsigned cardRate() const { const unsigned r = m_rate; return r == 288000 ? 96000 : r; }

    std::atomic<unsigned> m_rate{96000};   // the stream's rate; the card runs at cardRate()
    FifiInterpolator m_interpolator;
    FifiDcBlocker m_dcBlocker;
    std::vector<float> m_up;
    bool m_float{false};
    std::vector<int16_t> m_raw;
};

std::vector<SoapySDR::Kwargs> findFifiSdr(const SoapySDR::Kwargs& args)
{
    std::vector<SoapySDR::Kwargs> results;
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) != 0) { return results; }
    std::string serial;
    libusb_device_handle* h = openFifi(ctx, &serial);
    if (h) {
        libusb_close(h);
        if ((!args.count("serial") || args.at("serial") == serial) && !fifiPcmName().empty()) {
            SoapySDR::Kwargs dev;
            dev["driver"] = "fifisdr";
            dev["label"] = serial.empty() ? std::string("FiFi-SDR") : "FiFi-SDR :: " + serial;
            dev["serial"] = serial;
            results.push_back(dev);
        }
    }
    libusb_exit(ctx);
    return results;
}

SoapySDR::Device* makeFifiSdr(const SoapySDR::Kwargs& args) { return new SoapyFifiSdr(args); }

SoapySDR::Registry registerFifiSdr("fifisdr", &findFifiSdr, &makeFifiSdr, SOAPY_SDR_ABI_VERSION);

} // namespace

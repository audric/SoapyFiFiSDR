# SoapySDR driver for the FiFi-SDR

A SoapySDR module (`fifisdrSupport`, driver `fifisdr`) for the FiFi-SDR, a USB
receiver that is a 96 kHz I/Q sound card plus a USB tuning channel. I/Q is
captured with ALSA; tuning and the preamp go through libusb vendor requests
(no interface is claimed, the kernel's audio driver stays attached).
Linux only, one FiFi-SDR per computer.

## Build and install

    ./setup.sh --install            # add --server to share it with SoapyRemote

Run `./setup.sh` with no options to see the commands it would run. It installs
the build packages (Debian and Ubuntu), builds and installs the module and ends
by listing the receiver. By hand:

    sudo apt install libsoapysdr-dev libusb-1.0-0-dev libasound2-dev cmake g++
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build
    sudo cmake --install build

Run `SoapySDRUtil --find="driver=fifisdr"` to check. Pass
`-DFIFISDR_BUILD_TESTS=OFF` to skip the interpolator and DC blocker tests
(`build/fifi_interpolator_test`, `build/fifi_dc_blocker_test`).

## Use

- Device arguments: `driver=fifisdr`, `serial=<serial>` (only filters the
  search: a FiFi-SDR with another serial is not listed; opening always takes
  the first FiFi-SDR found), `iq_swap=1` (swap I and Q, to mirror the
  spectrum), `dc_removal=0` (start with DC removal off).
- Sample rates: 48000, 96000 (the sound card's own rates) and 288000, which is
  the card at 96000 interpolated x3 for receivers that need a wider capture.
  The usable width stays that of the 96 kHz sampling. Set the rate before
  setting up the stream: it is fixed once a stream is set up.
- DC removal: the sound card's DC offset, which shows as a spur at the tuned
  frequency, is removed at every rate. Each channel's offset is tracked on the
  raw samples (time constant 4800 frames) and, as it is not a whole number of
  LSBs, subtracted after the conversion to float: at 288000 after the
  interpolation, at 48000 and 96000 from the CF32 samples. CS16 at 288000 is
  rounded from that result, so it keeps no DC either. CS16 at 48000 and 96000
  has the offset subtracted rounded to whole LSBs, which leaves up to half an
  LSB of DC. It is on by default and uses SoapySDR's standard switch
  (`setDCOffsetMode`, `getDCOffsetMode`); the device argument `dc_removal=0`
  (or `false`) starts with it off.
- Stream formats: CS16 (native, full scale 32768) and CF32.
- Tuning 0.2 to 30 MHz; gain element `PREAMP` (0 or 6 dB).
- The process needs write access to the device's USB node (a udev rule for
  16c0:05dc, or run it as a user in the right group) and read access to the
  sound card.

The control requests follow Hamlib's `rigs/kit/fifisdr.c`.

License: GPL-3.0-or-later.

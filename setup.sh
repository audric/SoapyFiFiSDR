#!/usr/bin/env bash
# Build and install the SoapySDR driver for the FiFi-SDR receiver.
#
#   ./setup.sh                   print the commands only
#   ./setup.sh --install         run them (uses sudo)
#   ./setup.sh --install --server
#                                also enable the SoapyRemote server (port 55132)
#
# Run it on the computer the FiFi-SDR is plugged into (Linux only). The driver
# is built from this checkout. Always ends by running
# SoapySDRUtil --find="driver=fifisdr"; exits non-zero when no device is listed.
# Works unchanged on amd64 and arm64 (Debian/Ubuntu).
set -euo pipefail

INSTALL=0
SERVER=0
while [ $# -gt 0 ]; do
    case "$1" in
        --install) INSTALL=1 ;;
        --server) SERVER=1 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/soapy-fifisdr/build"

run() {
    echo "+ $*"
    if [ "$INSTALL" -eq 1 ]; then "$@"; fi
}

run sudo apt-get install -y libsoapysdr-dev soapysdr-tools libusb-1.0-0-dev \
    libasound2-dev cmake g++
if [ "$SERVER" -eq 1 ]; then
    run sudo apt-get install -y soapyremote-server
fi

run cmake -S "$SRC_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
run cmake --build "$BUILD_DIR" -j"$(nproc)"
run sudo cmake --install "$BUILD_DIR"
run sudo ldconfig

if [ "$SERVER" -eq 1 ]; then
    # The soapyremote-server package ships its own unit, listening on every
    # interface, port 55132. Restart it so it loads the new module.
    run sudo systemctl enable soapyremote-server.service
    run sudo systemctl restart soapyremote-server.service
fi

echo "+ SoapySDRUtil --find=\"driver=fifisdr\""
if ! OUT="$(SoapySDRUtil --find="driver=fifisdr" 2>&1)"; then
    echo "$OUT"; echo "SoapySDRUtil failed" >&2; exit 1
fi
echo "$OUT"
if ! grep -q "driver *= *fifisdr" <<<"$OUT"; then
    echo "No FiFi-SDR found (is it plugged in, is the driver installed, and may this user open the USB device?)" >&2
    exit 1
fi

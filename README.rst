ESPARGOS packet decoders
=======================

Portable C99 WiFi and Bluetooth LE IQ qualification, shared by sensor firmware,
WebAssembly and Python. Licensed under LGPL-3.0-or-later; see COPYING.LESSER and
COPYING. The initial decoders and RF fixtures were extracted from ESPARGOS
sensor-firmware, preserving its signal conventions and validation thresholds.

Hardware first
--------------

Use modem-provided MAC/header/CRC results whenever available. This library does
not access radio registers, acquire IQ, associate packets between sensors, or
perform phase calibration. A native FCS-valid completion and valid captured
training are separate evidence. Native WiFi reception should call only
``epd_wifi_qualify_native`` to check its training, then use the modem's metadata.
A training match alone never asserts a decoded address or payload CRC.

WiFi input is 20 MS/s signed 10-bit I and Q in bits 0..9 and 10..19, in the
ESPARGOS raw tap's Q polarity. Upper debug bits are ignored. BLE input uses the
same packing at IF +2 MHz, sampled at 4, 8 or 16 MS/s (2M needs at least 8 MS/s).
The C API supports circular input for full qualification. Callers own the
workspace and output; separate workspaces allow independent concurrent calls.

WiFi stages include bounded legacy OFDM/DSSS/CCK decoding, OFDM training-only,
extended PHY-header validation, and native-timestamp-guided OFDM/DSSS training.
HT multi-stream captures can expose legacy training and HT-SIG without decoding
multi-stream payloads. BLE 1M/2M supports arbitrary valid access addresses;
advertising CRC is checked against its known seed. Unknown connection CRC seeds
remain unverified. A coded-PHY preamble may be admitted without identity; full
coded-PHY decoding remains outside this initial C implementation.

Build and use
-------------

Host C library::

    cmake -S . -B build -DBUILD_SHARED_LIBS=ON
    cmake --build build

ESP-IDF: place this repository in ``components/espargos-packet-decoders`` and
add ``espargos-packet-decoders`` to the consuming component's requirements.
Public headers are ``epd/wifi.h``, ``epd/wifi_mac.h`` and ``epd/ble.h``.

Python (a C compiler and Python development headers are required)::

    python -m pip install .
    python -c 'from espargos_packet_decoders import wifi, ble'

``wifi(iq, stage="native_ofdm", training_hint=80)`` validates known training
without decoding the header again. ``wifi(iq)`` performs bounded full decoding.
``ble(iq, sample_rate=4000000, phy=0, channel=37)`` qualifies BLE. Input can be an
iterable of packed uint32 samples or native-endian packed bytes. Results are
Python dictionaries, or None on failed qualification. ``crc_checked`` is
independent of ``crc_valid``. The binding releases the GIL while decoding.

WebAssembly (Emscripten required)::

    python bindings/build_wasm.py dist

Serve ``epd.js``, ``epd.wasm`` and ``packet_decoders.js`` together, load the two
scripts, then ``await PacketDecoders.create()``. The returned reusable decoder
has ``wifi(Uint32Array, options)`` and ``ble(Uint32Array, options)`` methods,
matching Python's fields and stages. Call ``destroy()`` when finished. The
WASM module is self-hosted; it needs no CDN or network service.

Tests::

    python setup.py build_ext --inplace
    python bindings/build_wasm.py dist
    python -m unittest discover -s tests

Tests require NumPy, SciPy and Node. They cover real RF recordings, synthesized
modulations, circular input, noise and CRC failures. Binding tests compare
native Python and WebAssembly results on identical samples. CFO may differ by
one hertz due to libm rounding; decoded bytes and validation flags must agree.

The API is initial and not yet a stable binary ABI. Rebuild consumers when
updating the submodule. This repository is currently local and unpublished by user choice. Consumer
submodules point at the local development checkout; supply it locally before
initializing them on another machine.

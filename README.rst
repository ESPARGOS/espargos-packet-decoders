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
independent of ``crc_valid``. Results name their ``protocol``. BLE
``identity_valid`` means a decoded access address and header; WiFi
``mac_valid`` means a recovered transmitter MAC. The binding releases the GIL
while decoding.

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
updating the submodule.

The source repository is https://github.com/ESPARGOS/espargos-packet-decoders.
Consumers pin it as a Git submodule. After cloning a consuming project, run
``git submodule update --init --recursive`` to fetch its pinned dependencies.
For an existing checkout whose submodule URL has changed, first run
``git submodule sync --recursive``.

Soft 11b constellations
----------------------

``epd_wifi_dsss_symbols`` exposes measured complex samples, never decoded ideal
points. Python ``dsss_symbols`` and JavaScript ``dsssSymbols`` use the same C
implementation: CFO correction, a regularized seven-tap equalizer trained on
Barker chips, differential despreading for DBPSK/DQPSK, and chip sampling for
5.5/11-Mbps CCK. CCK output is a QPSK **chip** constellation, not one QAM point
per eight-chip codeword. Payload extraction requires a captured CRC-valid PLCP;
short preamble-only captures can instead select ``barker``. PBCC is excluded.
The Barker option expects already validated training offsets from the packet
metadata. ``epd_wifi_qualify_dsss_training`` additionally checks known chip
waveform correlation when there is no native modem packet verdict, rejecting
CW and other coherent signals that differential phase alone cannot reject.

``tests/test_wifi_symbols.py`` checks all four rates, long/short PLCP, an echo,
truncation, and native/WebAssembly parity. Set ``EPD_PYTHON_DIR`` and
``EPD_WASM_DIR`` when testing separate build directories.

ESPARGOS packet decoders
========================

A portable WiFi and Bluetooth LE decoder library for captured I/Q samples.
The same C implementation can run in embedded firmware, in a web browser
through WebAssembly, or in Python through a native extension.

The library provides the common decoding layer for the
`ESPARGOS <https://espargos.net/>`_ ecosystem:

* **Sensor firmware:** identify and validate packets in captured samples.
* **Web interface:** decode samples in the browser, including soft symbols for
  constellation plots. The controller serves the WebAssembly assets.
* **Python / pyespargos:** use the Python binding for offline analysis or client
  integration. The binding is usable independently of pyespargos; device access
  and array processing belong to the client.

The decoder has no hardware, network, or device-discovery dependency. It accepts
samples supplied by the caller and returns packet metadata and validation results.
Acquisition, cross-antenna association, MRC, and array calibration remain with
the application. Other receivers can use it by adapting their samples to the
input format below.

Supported signals
-----------------

* **WiFi 802.11b:** long/short preambles, DSSS and CCK at 1, 2, 5.5 and 11 Mbps.
* **Legacy OFDM:** 20 MHz, 6–54 Mbps, with soft demapping and Viterbi decoding.
* **HT:** legacy training and HT-SIG recovery; HT/MIMO payload decoding is not
  provided by this library.
* **Bluetooth LE:** LE1M and LE2M packet decoding. Advertising CRC is checked;
  an unknown connection CRC seed remains unverified. LE Coded support is limited
  to preamble detection, without FEC or payload decoding.

Captures may contain only part of a packet. Decoded identity, checked CRC, and
valid CRC are separate results: finding training or a header does not imply a
valid full packet. When native modem metadata is available, applications can
use the training-only entry points instead of decoding the packet again.

Input format
------------

Raw input uses packed 32-bit words: signed 10-bit I in bits 0–9 and signed
10-bit Q in bits 10–19. Higher bits are ignored. Q uses the ESPARGOS ADC tap's
inverted polarity relative to conventional complex baseband.

WiFi expects 20 MSa/s. BLE expects an IF of +2 MHz at 4, 8 or 16 MSa/s;
LE2M requires at least 8 MSa/s. C callers can supply circular buffers and own
all workspaces, so independent decoders can run concurrently.

C / embedded firmware
---------------------

Build the host library with a C99 compiler and CMake::

    cmake -S . -B build -DBUILD_SHARED_LIBS=ON
    cmake --build build

For ESP-IDF, add this repository as ``components/espargos-packet-decoders``
and list ``espargos-packet-decoders`` in the consuming component's requirements.
The public API is in ``include/epd/wifi.h`` and ``include/epd/ble.h``.
The ABI is not version-stable yet; rebuild consumers when updating the library.

Python
------

Requires Python 3.9+, a C compiler, and Python development headers::

    python -m pip install .

Decode an iterable of packed sample words or native-endian packed bytes::

    from espargos_packet_decoders import wifi, ble

    packet = wifi(iq_words)
    advertisement = ble(ble_iq_words, sample_rate=4_000_000, channel=37)

Results are dictionaries, or ``None`` when qualification fails. Check
``crc_checked`` and ``crc_valid`` separately. ``dsss_symbols()`` returns soft
DSSS symbols or CCK chips for constellation plots. Decoding releases the GIL.

WebAssembly / JavaScript
------------------------

With Emscripten's ``emcc`` on PATH::

    python bindings/build_wasm.py dist

Serve ``epd.js``, ``epd.wasm``, and ``packet_decoders.js`` from ``dist/``.
Load the two scripts in that order, then::

    const decoder = await PacketDecoders.create();
    const packet = decoder.wifi(iqWords); // Uint32Array of packed samples
    decoder.destroy();

The wrapper also provides ``ble()``, ``dsssSymbols()``, and ``wifiOfdm()``.
The latter accepts equalized OFDM symbols and per-carrier inverse noise variance
from an external combiner; see ``epd_wifi_decode_ofdm`` in the C header for the
layout. CCK constellations show measured QPSK chips, not decoded codewords.

The build includes license files and ``source.json`` with the source revision.
Firmware repositories pin the source as a submodule and distribute the generated
browser assets; their normal builds do not need Emscripten.

Development
-----------

Tests run without radio hardware. They use synthetic waveforms and small recorded
regression fixtures to check decoding, invalid inputs, and native/WebAssembly
agreement. With Node.js and Emscripten available::

    python -m pip install -e '.[test]'
    python bindings/build_wasm.py dist
    EPD_WASM_DIR=dist python -m unittest discover -s tests

``tools/wifi_ppdu.py`` generates deterministic WiFi test waveforms. It does not
control or transmit through any radio hardware.

License
-------

LGPL-3.0-or-later. See ``COPYING.LESSER`` and ``COPYING``.

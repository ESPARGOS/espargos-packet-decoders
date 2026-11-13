ESPARGOS Packet Decoders
========================

.. image:: img/espargos-logo.png
   :width: 240px
   :align: right
   :alt: ESPARGOS
   :target: https://espargos.net/

This library decodes WiFi and Bluetooth LE packets from I/Q samples. It is
written in C and builds for embedded processors, WebAssembly, and Python.

We use it in the `ESPARGOS <https://espargos.net/>`_ sensor firmware and web
interface. The Python binding makes the same decoder available for offline
analysis and integration with `pyespargos <https://github.com/ESPARGOS/pyespargos>`_.
In the web interface, decoding runs in the browser; the controller only serves
the WASM files.

You can also use it with samples from another receiver. The library takes a
buffer of samples and returns what it can decode: packet headers, addresses,
CRC results, and soft symbols. It leaves sample acquisition and array processing
to the application. See the sample format below.

What can it decode?
-------------------

* **802.11b:** DSSS and CCK at 1, 2, 5.5 and 11 Mbps, with long or short preambles.
* **Legacy OFDM:** 20 MHz WiFi at 6–54 Mbps.
* **802.11n:** legacy training and HT-SIG. HT payload decoding is not implemented.
* **Bluetooth LE:** LE1M and LE2M packets. LE Coded support currently stops at
  preamble detection.

Short captures are useful too: the decoder can recover training or a header
without having the whole packet. Results distinguish a decoded address from a
verified CRC. BLE advertising CRCs are checked; connection CRCs remain unverified
when their initialization value is unknown.

Using the library
-----------------

C and ESP-IDF
~~~~~~~~~~~~~

For a host build, you need a C99 compiler and CMake::

    cmake -S . -B build -DBUILD_SHARED_LIBS=ON
    cmake --build build

For ESP-IDF, add the repository under ``components/espargos-packet-decoders``
and add ``espargos-packet-decoders`` to your component's requirements.
The API is declared in ``include/epd/wifi.h`` and ``include/epd/ble.h``.
C callers provide the workspace; separate workspaces allow concurrent decoding.
Rebuild your application when updating the library, since the ABI may change.

Python
~~~~~~

Install from this checkout with Python 3.9 or newer. Building the extension
requires a C compiler and Python development headers::

    python -m pip install .

Pass an iterable of packed sample words, or native-endian packed bytes::

    from espargos_packet_decoders import wifi, ble

    packet = wifi(iq_words)
    advertisement = ble(ble_iq_words, sample_rate=4_000_000, channel=37)

Both functions return a dictionary, or ``None`` if decoding fails.
``crc_checked`` tells you whether enough data was available to check the CRC;
``crc_valid`` gives the result. ``dsss_symbols()`` returns soft DSSS symbols or
CCK chips for constellation plots. The extension releases the GIL while decoding.

JavaScript and WebAssembly
~~~~~~~~~~~~~~~~~~~~~~~~~~

With Emscripten's ``emcc`` on your PATH, build the browser files::

    python bindings/build_wasm.py dist

Serve ``epd.js``, ``epd.wasm``, and ``packet_decoders.js`` from ``dist/``.
Load the two scripts in that order, then::

    const decoder = await PacketDecoders.create();
    const packet = decoder.wifi(iqWords); // Uint32Array of packed samples
    decoder.destroy();

The wrapper also has ``ble()``, ``dsssSymbols()``, and ``wifiOfdm()``.
``wifiOfdm()`` accepts equalized OFDM symbols and their inverse noise variance,
so an application can combine antenna signals before decoding. Its input layout
is documented alongside ``epd_wifi_decode_ofdm`` in the C header.

The build copies the license files and records the source revision in
``source.json``. ESPARGOS firmware repositories include these generated assets,
so a normal firmware build does not need Emscripten.

Sample format
-------------

Each raw sample is a packed 32-bit word: signed 10-bit I in bits 0–9 and signed
10-bit Q in bits 10–19. The upper bits are ignored. Q follows the ESPARGOS ADC
tap's convention, with the opposite sign to conventional complex baseband.

WiFi samples must be at 20 MSa/s. BLE expects an IF of +2 MHz and a sample rate
of 4, 8 or 16 MSa/s; LE2M needs at least 8 MSa/s. The C API also accepts circular
buffers. If your radio already supplies packet metadata, the training-only
functions can check the captured training without decoding the header again.

Running the tests
-----------------

The tests use generated waveforms and recorded samples; no radio is needed.
With Node.js and Emscripten installed::

    python -m pip install -e '.[test]'
    python bindings/build_wasm.py dist
    EPD_WASM_DIR=dist python -m unittest discover -s tests

The tests cover decoding and malformed inputs, and compare native and WASM
results. ``tools/wifi_ppdu.py`` contains the WiFi waveform generator.

License
-------

LGPL-3.0-or-later. See ``COPYING.LESSER`` and ``COPYING``.

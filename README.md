<a href="https://espargos.net/"><img src="img/espargos-logo.png" width="240" align="right" alt="ESPARGOS"></a>

# ESPARGOS Packet Decoders

This library decodes WiFi and Bluetooth LE packets from I/Q samples. It is
written in C and builds for embedded processors, WebAssembly, and Python.

We use it in the [ESPARGOS](https://espargos.net/) sensor firmware and web
interface. The Python binding makes the same decoder available for offline
analysis and integration with [pyespargos](https://github.com/ESPARGOS/pyespargos).
In the web interface, decoding runs in the browser; the controller only serves
the WASM files.

You can also use it with samples from another receiver. The library takes a
buffer of samples and returns what it can decode: packet headers, addresses,
CRC results, and soft symbols. It leaves sample acquisition and array processing
to the application. See the sample format below.

## What can it decode?

* **802.11b:** DSSS and CCK at 1, 2, 5.5 and 11 Mbps, with long or short preambles.
* **802.11a/g (OFDM):** 20 MHz WiFi at 6–54 Mbps.
* **802.11n:** legacy training and HT-SIG. HT payload decoding is not implemented.
* **Bluetooth LE:** LE1M and LE2M packets. LE Coded support currently stops at
  preamble detection.

Short captures are useful too: the decoder can recover training or a header
without having the whole packet. Results distinguish a decoded address from a
verified CRC. BLE advertising CRCs are checked; connection CRCs remain unverified
when their initialization value is unknown.

## Using the library

### C and ESP-IDF

For a host build, you need a C99 compiler and CMake:

```bash
cmake -S . -B build -DBUILD_SHARED_LIBS=ON
cmake --build build
```

For ESP-IDF, add the repository under `components/espargos-packet-decoders`
and add `espargos-packet-decoders` to your component's requirements.
The API is declared in `include/epd/wifi.h` and `include/epd/ble.h`.
C callers provide the workspace; separate workspaces allow concurrent decoding.
Rebuild your application when updating the library, since the ABI may change.

### Python

Install from this checkout with Python 3.9 or newer. Building the extension
requires a C compiler and Python development headers:

```bash
python -m pip install .
```

Pass an iterable of packed sample words, or native-endian packed bytes:

```python
from espargos_packet_decoders import wifi, ble

packet = wifi(iq_words)
advertisement = ble(ble_iq_words, sample_rate=4_000_000, channel=37)
```

Both functions return a dictionary, or `None` if decoding fails.
`crc_checked` tells you whether enough data was available to check the CRC;
`crc_valid` gives the result. `dsss_symbols()` returns soft DSSS symbols or
CCK chips for constellation plots. The extension releases the GIL while decoding.

### JavaScript and WebAssembly

With Emscripten's `emcc` on your PATH, build the browser files:

```bash
python bindings/build_wasm.py dist
```

Serve `epd.js`, `epd.wasm`, and `packet_decoders.js` from `dist/`.
Load the two scripts in that order, then:

```javascript
const decoder = await PacketDecoders.create();
const packet = decoder.wifi(iqWords); // Uint32Array of packed samples
decoder.destroy();
```

The wrapper also has `ble()`, `dsssSymbols()`, and `wifiOfdm()`.
`wifiOfdm()` accepts equalized OFDM symbols and their inverse noise variance,
so an application can combine antenna signals before decoding. Its input layout
is documented alongside `epd_wifi_decode_ofdm` in the C header.

The build copies the license files and records the source revision in
`source.json`. ESPARGOS firmware repositories include these generated assets,
so a normal firmware build does not need Emscripten.

## Sample format

Each raw sample is a packed 32-bit word: signed 10-bit I in bits 0–9 and signed
10-bit Q in bits 10–19. The upper bits are ignored. Q follows the ESPARGOS ADC
tap's convention, with the opposite sign to conventional complex baseband.

WiFi samples must be at 20 MSa/s. BLE expects an IF of +2 MHz and a sample rate
of 4, 8 or 16 MSa/s; LE2M needs at least 8 MSa/s. The C API also accepts circular
buffers. If your radio already supplies packet metadata, the training-only
functions can check the captured training without decoding the header again.

## API conventions

Python and JavaScript calls are synchronous once the WASM module has loaded.
They copy the input before decoding and return independent results: a later
call or `destroy()` does not invalidate earlier results. Keep a JavaScript
decoder for repeated calls, then call `destroy()` to release its workspace.
Calling `destroy()` twice is safe; decoding after destruction raises an error.

Raw input contains 1–16384 sample words. Python accepts unsigned-word iterables
or packed `bytes`, `bytearray`, and `memoryview` buffers; JavaScript requires a
`Uint32Array`. Invalid input sizes or option values raise `ValueError` in Python
or `RangeError` in JavaScript; wrong option types raise `TypeError`. A valid
capture that cannot be decoded returns `None` / `null` instead.

`wifi()` accepts stages `full`, `training`, `extended_header`, `native_ofdm`, and
`native_dsss`. The native stages use `training_hint`, a signed sample offset.
BLE uses `phy=0` for LE1M or `phy=1` for LE2M, with logical channel numbers 0–39.
Offsets and lengths in results are measured in samples; an unavailable frame or
training offset is 65535. `rate_half_mbps` is in units of 0.5 Mbps; `cfo_hz` is in
Hz. A WiFi transmitter address requires `mac_valid`; BLE identity fields require
`identity_valid`. The returned prefix can be shorter than the full packet.

The constellation helpers return `{rate, points}`, with rate in Mbps and at
most 1024 complex points. Python uses `complex` values; JavaScript uses `{re, im}`.
No available symbols gives an empty list and rate zero. For Barker extraction,
Python names the options `training`, `training_words`, and `cfo`; JavaScript uses
`training`, `trainingWords`, and `cfo`. CFO is in Hz, rounded to the nearest integer
(ties toward positive infinity). These helpers require previously validated
training offsets. CCK points represent chips, not eight-chip codewords.

The C headers specify workspace ownership, buffer bounds, and result validity.
A failed C call returns false (or zero symbols); its output must be ignored.

## Running the tests

The tests use generated waveforms and recorded samples; no radio is needed.
With Node.js and Emscripten installed:

```bash
python -m pip install -e '.[test]'
python bindings/build_wasm.py dist
EPD_WASM_DIR=dist python -m unittest discover -s tests
```

The tests cover decoding and malformed inputs, and compare native and WASM
results. `tools/wifi_ppdu.py` contains the WiFi waveform generator.

## License

LGPL-3.0-or-later. See `COPYING.LESSER` and `COPYING`.

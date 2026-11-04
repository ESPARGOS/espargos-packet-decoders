WiFi header IQ fixtures
=======================

``wifi_header_iq.npz`` contains raw 32-bit C61 ADC words at 20 MS/s, captured
on 2026-11-03. I occupies bits 0..9; Q occupies bits 10..19 with the raw tap's
inverted complex-baseband convention. Higher bits contain gain/AGC telemetry.

* ``native_ofdm``: ESPARGOS sensor 0 receives a standalone C61's 6 Mbps,
  68-byte PSDU. Sensor 1 independently decoded that test transmitter. This
  short IQ excerpt contains L-LTF and L-SIG but insufficient MAC data.
* ``long_ofdm``: standalone C61 bounded IQ capture of a HackRF-generated
  6 Mbps, 1445-byte PSDU. Only its header is retained. The transmitter is
  02:00:00:00:00:01; sequence control is 0x00b0. No whole-frame FCS is present.
* ``long_dsss``: standalone C61 bounded IQ capture of a HackRF-generated
  long-preamble 1 Mbps, 232-byte PSDU. PLCP duration is 1856 us, transmitter
  02:00:00:00:00:01 and sequence control 0x0060. No whole-frame FCS is present.

The synthetic vectors in the test supplement these independent RF captures;
they do not replace them. These fixtures do not establish full MIMO payload
decoding, which a single receive chain cannot generally provide.

* ``legacy_ht_false_match``: ESPARGOS sensor 5 receives a 1445-byte legacy
  OFDM test frame. An early decoder accidentally accepted a rotated-noise
  HT-SIG CRC and reported reserved MCS 100. This regression fixture requires
  legacy classification, a defined HT MCS and rotated-BPSK evidence. Its
  packed stream was unpacked; upper AGC bits are unavailable.

``wifi_cck_iq.npz`` adds eight 20 MS/s CCK excerpts from ESPARGOS antennas 2
and 5, receiving HackRF-generated 5.5/11 Mbps packets with long and short
preambles. These channels broke a bare codeword correlator; PLCP-trained
seven-tap equalization recovers their transmitter and full packet FCS. The
records were unpacked from WFQ1; upper gain/AGC bits are not present. Some
stream excerpts start within the preamble; the test adds leading idle samples
to let the general acquisition search inspect the retained SFD/PLCP history.

Four additional ``native_rate2/3/6/7`` records come from the ESP32-C61 native
WiFi transmitter, independently of the waveform generator. They cover long and
short preamble 5.5/11 Mbps CCK, with transmitter 02:45:53:50:49:51 and a 68-byte
PSDU. All four require a correct MAC and full FCS.

``wifi_ofdm_native.npz`` contains eight native C61 OFDM captures at 6–54 Mbps,
one for each legacy rate. These exposed an I/Q bit-order error shared by the
initial synthetic generator and decoder: QPSK swapped the earliest I/Q bits,
and QAM indexed the sign bit as the last bit on each axis. All retained packets
except 6 Mbps contain enough payload for a full FCS. The 6-Mbps excerpt retains
the transmitter MAC but its 102.4-us window ends before the payload FCS.
Mapping is checked against IEEE 802.11-2012, 18.3.5.8, Tables 18-9 through 18-11.

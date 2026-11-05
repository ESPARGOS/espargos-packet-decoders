High-gain Barker training
========================

``wifi_barker_dc.bin`` contains 1536 little-endian raw C61 ADC words from
antenna 2 of oldspargel.local, channel 11, automatic gain, on 2026-11-05.
This is a preamble-only shared history window associated with a native
MAC/FCS-validated 1-Mbps reception. Upper bits retain gain/AGC state; I and Q
are the signed low ten-bit fields. It contains no decoded MAC header or payload.

The native-backed window failed the standalone Barker check because the
matched-filter numerator canceled DC but its energy denominator did not.
The DC-corrected check accepts the same unmodified samples at hint 800,
without asserting MAC or payload-FCS evidence. This fixture demonstrates
DC normalization, not the recoverability of every weak array antenna.

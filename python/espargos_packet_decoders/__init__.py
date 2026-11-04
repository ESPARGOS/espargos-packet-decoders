# SPDX-License-Identifier: LGPL-3.0-or-later
"""Portable WiFi/BLE decoding of ESPARGOS packed signed 10-bit IQ samples.

WiFi: 20 MS/s, raw tap Q polarity. BLE: IF +2 MHz, 4/8/16 MS/s.
Native modem metadata should be used directly when available; select a training
stage to validate IQ without decoding headers or FCS again.
"""
import array
import struct
from . import _native

STAGES = {'full': 0, 'training': 1, 'extended_header': 2,
          'native_ofdm': 3, 'native_dsss': 4}

def _samples(iq):
    if isinstance(iq, bytes):
        return iq
    return array.array('I', iq).tobytes()

def _result(raw):
    if raw is None:
        return None
    v = struct.unpack('=26I', raw)
    if v[0] != 1:
        raise RuntimeError('Unsupported decoder ABI')
    return dict(phy=v[1], rate_half_mbps=v[2], mcs=v[3], stbc=v[4],
                frame_start=v[5], training_start=v[6], training_words=v[7],
                length_bytes=v[8], duration_us=v[9],
                cfo_hz=struct.unpack('=i', raw[40:44])[0], quality=v[11],
                mac_valid=bool(v[12]&1), crc_checked=bool(v[12]&2), crc_valid=bool(v[12]&4),
                prefix=raw[64:64+v[13]], access_address=v[14], packet_crc=v[15],
                transmitter=raw[96:102])

def wifi(iq, *, stage='full', training_hint=0):
    """Return a partial/complete decoded result, or None when qualification fails."""
    return _result(_native.decode(_samples(iq), 0, STAGES[stage], training_hint, 0))

def ble(iq, *, sample_rate=4_000_000, phy=0, channel=37):
    """Qualify BLE 1M/2M; unknown connection CRC seeds remain unverified."""
    return _result(_native.decode(_samples(iq), 1, sample_rate, phy, channel))

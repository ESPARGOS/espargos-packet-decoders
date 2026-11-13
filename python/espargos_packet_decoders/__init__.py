# SPDX-License-Identifier: LGPL-3.0-or-later
"""Portable WiFi/BLE decoding of ESPARGOS packed signed 10-bit IQ samples.

WiFi: 20 MS/s, raw tap Q polarity. BLE: IF +2 MHz, 4/8/16 MS/s.
Native modem metadata should be used directly when available; select a training
stage to validate IQ without decoding headers or FCS again.
"""
import array
import math
import operator
from numbers import Real
import struct
from . import _native

STAGES = {'full': 0, 'training': 1, 'extended_header': 2,
          'native_ofdm': 3, 'native_dsss': 4}

def _samples(iq):
    if isinstance(iq, (bytes, bytearray, memoryview)):
        return bytes(iq)
    try:
        return array.array('I', iq).tobytes()
    except OverflowError as error:
        raise ValueError('IQ words must be in 0..4294967295') from error

def _integer(value, name, low, high):
    if isinstance(value, bool):
        raise TypeError(name + ' must be an integer')
    value = operator.index(value)
    if not low <= value <= high:
        raise ValueError(f'{name} must be in {low}..{high}')
    return value


def _cfo(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError('cfo must be a finite number in Hz')
    if not math.isfinite(value) or not -2147483648 <= value <= 2147483647:
        raise ValueError('cfo must fit a signed 32-bit integer in Hz')
    return math.floor(value + .5)


def _result(raw):
    if raw is None:
        return None
    v = struct.unpack('=26I', raw)
    if v[0] != 1:
        raise RuntimeError('Unsupported decoder ABI')
    if v[1] >= 100:
        return dict(protocol='ble', phy=v[1]-100, identity_valid=bool(v[12]&1),
                    crc_checked=bool(v[12]&2), crc_valid=bool(v[12]&4),
                    prefix=raw[64:64+v[13]], access_address=v[14], packet_crc=v[15])
    return dict(protocol='wifi', phy=v[1], rate_half_mbps=v[2], mcs=v[3], stbc=v[4],
                frame_start=v[5], training_start=v[6], training_words=v[7],
                length_bytes=v[8], duration_us=v[9],
                cfo_hz=struct.unpack('=i', raw[40:44])[0], quality=v[11],
                mac_valid=bool(v[12]&1), crc_checked=bool(v[12]&2), crc_valid=bool(v[12]&4),
                prefix=raw[64:64+v[13]],
                transmitter=raw[96:102])

def wifi(iq, *, stage='full', training_hint=0):
    """Return a partial/complete decoded result, or None when qualification fails."""
    if not isinstance(stage, str):
        raise TypeError('stage must be a string')
    if stage not in STAGES:
        raise ValueError('Unknown WiFi decoder stage')
    hint = _integer(training_hint, 'training_hint', -2147483648, 2147483647)
    return _result(_native.decode(_samples(iq), 0, STAGES[stage], hint, 0))

def ble(iq, *, sample_rate=4_000_000, phy=0, channel=37):
    """Qualify BLE 1M/2M; unknown connection CRC seeds remain unverified."""
    sample_rate = _integer(sample_rate, 'sample_rate', 4000000, 16000000)
    phy = _integer(phy, 'phy', 0, 1)
    channel = _integer(channel, 'channel', 0, 39)
    if sample_rate not in (4000000, 8000000, 16000000) or (phy == 1 and sample_rate < 8000000):
        raise ValueError('Unsupported BLE sample rate for this PHY')
    return _result(_native.decode(_samples(iq), 1, sample_rate, phy, channel))


def dsss_symbols(iq, *, field='payload', training=0, training_words=0, cfo=0):
    """Soft equalized CCK chips or differential DSSS/Barker symbols, never decisions."""
    if field not in ('payload', 'barker'):
        raise ValueError('Unknown DSSS field')
    training = _integer(training, 'training', 0, 4294967295)
    training_words = _integer(training_words, 'training_words', 0, 4294967295)
    rate, raw = _native.dsss_symbols(_samples(iq), int(field == 'barker'), training, training_words, _cfo(cfo))
    return dict(rate=rate / 2, points=[complex(r, i) for r, i in struct.iter_unpack('=ff', raw)])

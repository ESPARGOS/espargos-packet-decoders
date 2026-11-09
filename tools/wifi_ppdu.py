#!/usr/bin/env python3
"""Synthesize reusable 802.11 PPDU baseband waveforms.

The implementation covers non-HT OFDM, long-preamble 11b DSSS/CCK and HT20
MCS0, including a one-chain virtual-channel HT-STBC waveform. It deliberately
exposes the intermediate bit/symbol pipeline so later formats can share the
same hardware-tested coding primitives.
"""

from __future__ import annotations

import argparse
import binascii
import math
import pathlib
import struct

import numpy as np


POLARITY = np.array(
    [
        1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1,
        -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1,
        1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1,
        -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1,
        -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1,
        -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, -1, 1,
        -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, -1,
        -1, 1, -1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1,
    ],
    dtype=np.float32,
)

LONG = np.array(
    [
        0, 0, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1,
        1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1,
        0, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1,
        1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    ],
    dtype=np.complex64,
)

SHORT = np.zeros(64, dtype=np.complex64)
for index, sign in zip(
    (8, 12, 16, 20, 24, 28, 36, 40, 44, 48, 52, 56),
    (1, -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, 1),
):
    SHORT[index] = sign * (1 + 1j) * math.sqrt(13 / 6)

# Phase-ramped first L-LTF symbol used by gr-ieee802-11.  Together with LONG
# and a 16-sample CP on each symbol this forms the standard 32-sample L-LTF GI
# followed by two identical long-training symbols.
LONG_RAMPED = np.array(
    [
        0, 0j, 0, 0j, 0, 0j, -1, 1j, -1, 1j, -1, 1j, -1, -1j,
        1, 1j, 1, -1j, -1, 1j, 1, 1j, 1, 1j, 1, 1j, -1, -1j,
        1, -1j, -1, 1j, 0, -1j, 1, -1j, 1, -1j, 1, 1j, -1, -1j,
        1, -1j, -1, 1j, 1, 1j, 1, 1j, 1, 1j, -1, -1j, 1, 1j,
        1, -1j, -1, 0j, 0, 0j, 0, 0j,
    ],
    dtype=np.complex64,
)

RATE_PARAMETERS = {
    6: (0xD, 1, 48, 24, "1/2"),
    9: (0xF, 1, 48, 36, "3/4"),
    12: (0x5, 2, 96, 48, "1/2"),
    18: (0x7, 2, 96, 72, "3/4"),
    24: (0x9, 4, 192, 96, "1/2"),
    36: (0xB, 4, 192, 144, "3/4"),
    48: (0x1, 6, 288, 192, "2/3"),
    54: (0x3, 6, 288, 216, "3/4"),
}

DATA_SUBCARRIERS = [
    k for k in range(-26, 27) if k not in (0, -21, -7, 7, 21)
]
PILOT_SUBCARRIERS = (-21, -7, 7, 21)
HT_DATA_SUBCARRIERS = [
    k for k in range(-28, 29) if k not in (0, -21, -7, 7, 21)
]

# HT20 extends the legacy LTF with two +1 and two -1 edge tones.
# Derive it to avoid a separately transcribed sequence (notably k=-3).
HT_LTF_SIGNS = (1, 1, *LONG[6:32].real.astype(int), *LONG[33:59].real.astype(int), -1, -1)

VIRTUAL_HT_STBC_H1 = 1.0 + 0.0j
VIRTUAL_HT_STBC_H2 = 0.0 + 0.5j
# With two HT space-time streams, stream 2 has a -400 ns cyclic shift.  In
# frequency order -32..31 this is exp(+j*2*pi*k/8).
HT_STBC_STREAM2_CSD = np.exp(
    1j * 2 * math.pi * np.arange(-32, 32) / 8
).astype(np.complex64)

BARKER = np.array((1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1), dtype=np.complex64)


def bits_from_bytes(data: bytes) -> np.ndarray:
    return np.unpackbits(np.frombuffer(data, dtype=np.uint8), bitorder="little")


def convolutional_encode(bits: np.ndarray) -> np.ndarray:
    state = 0
    out = np.empty(len(bits) * 2, dtype=np.uint8)
    for i, bit in enumerate(bits):
        state = ((state << 1) & 0x7E) | int(bit)
        out[2 * i] = (state & 0o155).bit_count() & 1
        out[2 * i + 1] = (state & 0o117).bit_count() & 1
    return out


def puncture(bits: np.ndarray, code_rate: str) -> np.ndarray:
    if code_rate == "1/2":
        return bits
    if code_rate == "2/3":
        pattern = np.array([1, 1, 1, 0], dtype=bool)
    elif code_rate == "3/4":
        pattern = np.array([1, 1, 1, 0, 0, 1], dtype=bool)
    else:
        raise ValueError(f"unsupported code rate {code_rate}")
    return bits[np.resize(pattern, len(bits))]


def interleave(bits: np.ndarray, n_cbps: int, n_bpsc: int) -> np.ndarray:
    if len(bits) % n_cbps:
        raise ValueError("coded bit count is not a whole number of symbols")
    s = max(n_bpsc // 2, 1)
    first = np.array(
        [s * (j // s) + ((j + (16 * j) // n_cbps) % s) for j in range(n_cbps)]
    )
    second = np.array([16 * i - (n_cbps - 1) * ((16 * i) // n_cbps) for i in range(n_cbps)])
    permutation = second[first]
    return np.concatenate([symbol[permutation] for symbol in bits.reshape(-1, n_cbps)])


def ht_interleave(bits: np.ndarray, n_cbps: int, n_bpsc: int) -> np.ndarray:
    if len(bits) % n_cbps:
        raise ValueError("coded HT bit count is not a whole number of symbols")
    n_col = 13
    n_row = n_cbps // n_col
    s = max(n_bpsc // 2, 1)
    result = []
    for symbol in bits.reshape(-1, n_cbps):
        out = np.empty(n_cbps, dtype=np.uint8)
        for k in range(n_cbps):
            i = n_row * (k % n_col) + k // n_col
            j = s * (i // s) + (i + n_cbps - (n_col * i) // n_cbps) % s
            out[j] = symbol[k]
        result.append(out)
    return np.concatenate(result)


def scramble(bits: np.ndarray, initial_state: int) -> np.ndarray:
    if not 1 <= initial_state <= 127:
        raise ValueError("scrambler seed must be 1..127")
    state = initial_state
    out = np.empty_like(bits)
    for i, bit in enumerate(bits):
        feedback = ((state >> 6) ^ (state >> 3)) & 1
        out[i] = int(bit) ^ feedback
        state = ((state << 1) & 0x7E) | feedback
    return out


def map_constellation(groups: np.ndarray, n_bpsc: int) -> np.ndarray:
    weights = 1 << np.arange(n_bpsc, dtype=np.uint8)
    values = groups.reshape(-1, n_bpsc).dot(weights)
    if n_bpsc == 1:
        table = np.array([-1, 1], dtype=np.complex64)
    elif n_bpsc == 2:
        table = np.array(
            [-1 - 1j, 1 - 1j, -1 + 1j, 1 + 1j], dtype=np.complex64
        ) / math.sqrt(2)
    else:
        # IEEE bit ordering is Gray-coded independently on I and Q.  Build the
        # table with the earliest/sign bit in the least significant index
        # position (IEEE 802.11-2012 Tables 18-10/18-11).
        axis_bits = n_bpsc // 2
        levels = {
            2: (-3, 3, -1, 1),
            3: (-7, 7, -1, 1, -5, 5, -3, 3),
        }[axis_bits]
        norm = math.sqrt(10 if n_bpsc == 4 else 42)
        table = np.array(
            [
                levels[value & ((1 << axis_bits) - 1)]
                + 1j * levels[value >> axis_bits]
                for value in range(1 << n_bpsc)
            ],
            dtype=np.complex64,
        ) / norm
    return table[values]


def _ifft_symbol(freq_shifted: np.ndarray, cp: int = 16) -> np.ndarray:
    time_domain = np.fft.ifft(np.fft.ifftshift(freq_shifted))
    return np.concatenate((time_domain[-cp:], time_domain)).astype(np.complex64)


def legacy_preamble() -> np.ndarray:
    return np.concatenate(
        tuple(_ifft_symbol(freq) for freq in (SHORT, SHORT, LONG_RAMPED, LONG))
    )


def _signal_bits(rate_field: int, psdu_length: int) -> np.ndarray:
    bits = np.zeros(24, dtype=np.uint8)
    bits[:4] = [(rate_field >> shift) & 1 for shift in (3, 2, 1, 0)]
    bits[5:17] = [(psdu_length >> shift) & 1 for shift in range(12)]
    bits[17] = int(bits[:17].sum()) & 1
    return bits


def _ofdm_symbols(
    mapped: np.ndarray, first_symbol_index: int, data_rotation: complex = 1
) -> np.ndarray:
    result = []
    for symbol_index, data in enumerate(mapped.reshape(-1, 48), first_symbol_index):
        freq = np.zeros(64, dtype=np.complex64)
        for k, value in zip(DATA_SUBCARRIERS, data):
            freq[(k + 32) % 64] = value * data_rotation
        pilot_polarity = POLARITY[symbol_index % len(POLARITY)]
        for k, value in zip(PILOT_SUBCARRIERS, (1, 1, 1, -1)):
            freq[(k + 32) % 64] = value * pilot_polarity
        result.append(_ifft_symbol(freq))
    return np.concatenate(result)


def _ht_ofdm_symbols(
    mapped: np.ndarray,
    first_symbol_index: int = 0,
    stbc_first_chain: bool = False,
    stbc_collapsed: bool = False,
) -> np.ndarray:
    if stbc_collapsed and not stbc_first_chain:
        raise ValueError("collapsed STBC requires STBC mapping")
    frequencies = []
    # Table 20-19 pilot sequences for one and two HT space-time streams.
    # A symbol uses four consecutive entries, so rotating the base sequence by
    # the symbol index produces the pilots at (-21, -7, 7, 21).
    pilot_sequences = {
        1: ((1, 1, 1, -1),),
        2: ((1, 1, -1, -1), (1, -1, -1, 1)),
    }
    for symbol_index, data in enumerate(mapped.reshape(-1, 52), first_symbol_index):
        freq = np.zeros(64, dtype=np.complex64)
        for k, value in zip(HT_DATA_SUBCARRIERS, data):
            freq[(k + 32) % 64] = value
        frequencies.append(freq)
    if stbc_first_chain:
        if len(frequencies) & 1:
            raise ValueError("STBC mapping requires an even number of OFDM symbols")
        transformed = []
        h1 = VIRTUAL_HT_STBC_H1
        h2 = VIRTUAL_HT_STBC_H2
        for pair_index, (first, second) in enumerate(
            zip(frequencies[::2], frequencies[1::2])
        ):
            stream_frequencies = (
                (first, second),
                (-np.conj(second), np.conj(first)),
            )
            if stbc_collapsed:
                mapped_pair = [
                    h1 * stream_frequencies[0][time_index]
                    + h2 * HT_STBC_STREAM2_CSD
                    * stream_frequencies[1][time_index]
                    for time_index in range(2)
                ]
            else:
                mapped_pair = list(stream_frequencies[0])
            for time_index, freq in enumerate(mapped_pair):
                symbol_index = first_symbol_index + 2 * pair_index + time_index
                pilot_polarity = POLARITY[(symbol_index + 3) % len(POLARITY)]
                for pilot_index, k in enumerate(PILOT_SUBCARRIERS):
                    values = [
                        sequence[(symbol_index + pilot_index) % 4]
                        for sequence in pilot_sequences[2]
                    ]
                    pilot = (
                        h1 * values[0]
                        + h2 * HT_STBC_STREAM2_CSD[(k + 32) % 64] * values[1]
                    )
                    if not stbc_collapsed:
                        pilot = values[0]
                    freq[(k + 32) % 64] = pilot * pilot_polarity
            transformed.extend(mapped_pair)
        frequencies = transformed
    else:
        for symbol_index, freq in enumerate(frequencies, first_symbol_index):
            pilot_polarity = POLARITY[(symbol_index + 3) % len(POLARITY)]
            sequence = pilot_sequences[1][0]
            for pilot_index, k in enumerate(PILOT_SUBCARRIERS):
                freq[(k + 32) % 64] = (
                    sequence[(symbol_index + pilot_index) % 4] * pilot_polarity
                )
    return np.concatenate([_ifft_symbol(freq) for freq in frequencies])


def make_probe_request(sequence: int = 0, payload: bytes = b"ESPARGOS-PPDU",
                       corrupt_payload: bool = False) -> bytes:
    header = struct.pack(
        "<HH6s6s6sH",
        0x0040,
        0,
        b"\xff" * 6,
        b"\x02\x00\x00\x00\x00\x01",
        b"\xff" * 6,
        (sequence & 0xFFF) << 4,
    )
    body = b"\x00\x00" + bytes((221, len(payload))) + payload
    mpdu = header + body
    fcs = struct.pack("<I", binascii.crc32(mpdu) & 0xFFFFFFFF)
    if corrupt_payload:
        # Flip bits INSIDE the FCS-protected MPDU while keeping the FCS field
        # itself a well-formed CRC of the ORIGINAL frame: the receiver's CRC
        # then mismatches, exactly like a real payload bit error, whereas
        # --corrupt-fcs damages only the trailing checksum.
        damaged = bytearray(mpdu)
        damaged[-1] ^= 0xA5
        mpdu = bytes(damaged)
    return mpdu + fcs


def dsss_scramble(bits: np.ndarray) -> np.ndarray:
    """Apply the 11b self-synchronizing x^-4 + x^-7 scrambler."""
    history = [1] * 7
    out = np.empty_like(bits)
    for index, bit in enumerate(bits):
        coded = int(bit) ^ history[-4] ^ history[-7]
        out[index] = coded
        history.append(coded)
    return out


def dsss_plcp_crc_bits(header_bits: np.ndarray) -> np.ndarray:
    """Return the 16 serial CRC bits for the 11b SIGNAL/SERVICE/LENGTH fields."""
    if len(header_bits) != 32:
        raise ValueError("11b PLCP CRC covers exactly 32 bits")
    state = [1] * 16  # index 0 is x^0, index 15 is x^15
    for bit in header_bits:
        feedback = state[15] ^ int(bit)
        old = state
        state = [0] * 16
        state[15] = old[14]
        state[14] = old[13]
        state[13] = old[12]
        state[12] = old[11] ^ feedback
        state[11] = old[10]
        state[10] = old[9]
        state[9] = old[8]
        state[8] = old[7]
        state[7] = old[6]
        state[6] = old[5]
        state[5] = old[4] ^ feedback
        state[4] = old[3]
        state[3] = old[2]
        state[2] = old[1]
        state[1] = old[0]
        state[0] = feedback
    complemented = [bit ^ 1 for bit in state]
    return np.array(
        [complemented[15 - i] for i in range(8)]
        + [complemented[7 - i] for i in range(8)],
        dtype=np.uint8,
    )


def _differential_dsss(bits: np.ndarray, rate_mbps: int, phase: complex = 1) -> tuple[np.ndarray, complex]:
    chips = []
    if rate_mbps == 1:
        for bit in bits:
            if bit:
                phase = -phase
            chips.append(phase * BARKER)
    elif rate_mbps == 2:
        if len(bits) & 1:
            bits = np.append(bits, 0)
        # Serial dibits 00, 01, 11, 10 produce 0, +pi/2, pi, -pi/2.
        # ``dibit`` stores the first transmitted bit in bit 0.
        phase_steps = (0, -math.pi / 2, math.pi / 2, math.pi)
        for first, second in bits.reshape(-1, 2):
            dibit = int(first) | (int(second) << 1)
            phase *= np.exp(1j * phase_steps[dibit])
            chips.append(phase * BARKER)
    else:
        raise ValueError("DSSS implementation supports 1 or 2 Mbps")
    return np.concatenate(chips).astype(np.complex64), phase


def _dibit_phase(first: int, second: int) -> float:
    return {
        (0, 0): 0,
        (0, 1): math.pi / 2,
        (1, 1): math.pi,
        (1, 0): -math.pi / 2,
    }[(int(first), int(second))]


CCK_CODE_PHASES = {
    (0, 0): 0,
    (0, 1): math.pi / 2,
    (1, 0): math.pi,
    (1, 1): -math.pi / 2,
}


def _cck(bits: np.ndarray, rate_mbps: float, phase: complex) -> tuple[np.ndarray, complex]:
    bits_per_symbol = 4 if rate_mbps == 5.5 else 8
    if len(bits) % bits_per_symbol:
        bits = np.pad(bits, (0, bits_per_symbol - len(bits) % bits_per_symbol))
    chips = []
    previous_phase = float(np.angle(phase))
    for symbol_index, symbol_bits in enumerate(bits.reshape(-1, bits_per_symbol)):
        phi1 = previous_phase + _dibit_phase(symbol_bits[0], symbol_bits[1])
        if symbol_index & 1:
            phi1 += math.pi
        if rate_mbps == 5.5:
            phi2 = math.pi / 2 + math.pi * int(symbol_bits[2])
            phi3 = 0
            phi4 = math.pi * int(symbol_bits[3])
        else:
            phi2 = CCK_CODE_PHASES[(int(symbol_bits[2]), int(symbol_bits[3]))]
            phi3 = CCK_CODE_PHASES[(int(symbol_bits[4]), int(symbol_bits[5]))]
            phi4 = CCK_CODE_PHASES[(int(symbol_bits[6]), int(symbol_bits[7]))]
        codeword = np.exp(
            1j
            * np.array(
                (
                    phi1 + phi2 + phi3 + phi4,
                    phi1 + phi3 + phi4,
                    phi1 + phi2 + phi4,
                    phi1 + phi4 + math.pi,
                    phi1 + phi2 + phi3,
                    phi1 + phi3,
                    phi1 + phi2 + math.pi,
                    phi1,
                )
            )
        )
        chips.append(codeword)
        previous_phase = phi1
    return np.concatenate(chips).astype(np.complex64), complex(np.exp(1j * previous_phase))


def synthesize_dsss(psdu: bytes, rate_mbps: float = 1, short_preamble: bool = False) -> np.ndarray:
    if rate_mbps not in (1, 2, 5.5, 11):
        raise ValueError("11b implementation supports 1, 2, 5.5 or 11 Mbps")
    if short_preamble and rate_mbps == 1:
        raise ValueError("Short PLCP requires at least 2 Mbps")
    signal = {1: 0x0A, 2: 0x14, 5.5: 0x37, 11: 0x6E}[rate_mbps]
    length_us = math.ceil(8 * len(psdu) / rate_mbps)
    service = 0
    if rate_mbps == 11 and length_us * 11 - 8 * len(psdu) >= 8:
        service |= 0x80
    header = struct.pack("<BBH", signal, service, length_us)
    header_bits = bits_from_bytes(header)
    serial_bits = np.concatenate(
        (
            np.zeros(56, dtype=np.uint8) if short_preamble else np.ones(128, dtype=np.uint8),
            bits_from_bytes(struct.pack("<H", 0x05CF if short_preamble else 0xF3A0)),
            header_bits,
            dsss_plcp_crc_bits(header_bits),
            bits_from_bytes(psdu),
        )
    )
    scrambled = dsss_scramble(serial_bits)
    header_end = (56 if short_preamble else 128) + 16 + 48
    if short_preamble:
        sync, phase = _differential_dsss(scrambled[:72], 1)
        header_wave, phase = _differential_dsss(scrambled[72:header_end], 2, phase)
        preamble = np.concatenate((sync, header_wave))
    else:
        preamble, phase = _differential_dsss(scrambled[:header_end], 1)
    if rate_mbps in (1, 2):
        payload, _ = _differential_dsss(scrambled[header_end:], int(rate_mbps), phase)
    else:
        payload, _ = _cck(scrambled[header_end:], rate_mbps, phase)
    return np.concatenate((preamble, payload))


def synthesize_nonht(psdu: bytes, rate_mbps: int = 6, scrambler_seed: int = 93) -> np.ndarray:
    rate_field, n_bpsc, n_cbps, n_dbps, code_rate = RATE_PARAMETERS[rate_mbps]
    signal_coded = interleave(convolutional_encode(_signal_bits(rate_field, len(psdu))), 48, 1)
    signal = _ofdm_symbols(map_constellation(signal_coded, 1), first_symbol_index=0)

    n_sym = math.ceil((16 + 8 * len(psdu) + 6) / n_dbps)
    data_bits = np.zeros(n_sym * n_dbps, dtype=np.uint8)
    psdu_bits = bits_from_bytes(psdu)
    data_bits[16 : 16 + len(psdu_bits)] = psdu_bits
    scrambled = scramble(data_bits, scrambler_seed)
    tail_start = 16 + len(psdu_bits)
    scrambled[tail_start : tail_start + 6] = 0
    coded = puncture(convolutional_encode(scrambled), code_rate)
    interleaved = interleave(coded, n_cbps, n_bpsc)
    mapped = map_constellation(interleaved, n_bpsc)
    data = _ofdm_symbols(mapped, first_symbol_index=1)
    return np.concatenate((legacy_preamble(), signal, data))


def ht_sig_crc(bits: np.ndarray) -> np.ndarray:
    if len(bits) != 34:
        raise ValueError("HT-SIG CRC covers exactly 34 bits")
    state = [1] * 8
    for bit in bits:
        feedback = state[7] ^ int(bit)
        old = state
        state = [
            feedback,
            old[0] ^ feedback,
            old[1] ^ feedback,
            old[2],
            old[3],
            old[4],
            old[5],
            old[6],
        ]
    return np.array([state[7 - i] ^ 1 for i in range(8)], dtype=np.uint8)


def _ht_sig_bits(psdu_length: int, stbc: int = 0) -> np.ndarray:
    bits = np.zeros(48, dtype=np.uint8)
    bits[8:24] = [(psdu_length >> shift) & 1 for shift in range(16)]
    bits[24:27] = 1  # smoothing, not sounding, reserved
    bits[28:30] = [(stbc >> shift) & 1 for shift in range(2)]
    bits[34:42] = ht_sig_crc(bits[:34])
    return bits


def _ht_ltf_frequency() -> np.ndarray:
    freq = np.zeros(64, dtype=np.complex64)
    signs = iter(HT_LTF_SIGNS)
    for k in range(-28, 0):
        freq[k + 32] = next(signs)
    for k in range(1, 29):
        freq[k + 32] = next(signs)
    return freq


def _ht_training_fields() -> tuple[np.ndarray, np.ndarray]:
    return _ifft_symbol(SHORT), _ifft_symbol(_ht_ltf_frequency())


def synthesize_ht20(
    psdu: bytes, scrambler_seed: int = 93, stbc: int = 0,
    stbc_ltf2_sign: int = 1, stbc_collapsed: bool = False,
) -> np.ndarray:
    if stbc not in (0, 1):
        raise ValueError("HT20 STBC field must be 0 or 1")
    if stbc_collapsed and not stbc:
        raise ValueError("collapsed STBC requires the STBC field")
    n_dbps = 26
    n_sym = math.ceil((16 + 8 * len(psdu) + 6) / n_dbps)
    if stbc:
        n_sym = 2 * math.ceil(n_sym / 2)
    if stbc_ltf2_sign not in (-1, 1):
        raise ValueError("second STBC HT-LTF sign must be -1 or 1")
    l_length = 3 * (4 + n_sym + stbc) - 3
    l_signal_bits = interleave(
        convolutional_encode(_signal_bits(RATE_PARAMETERS[6][0], l_length)), 48, 1
    )
    l_signal = _ofdm_symbols(map_constellation(l_signal_bits, 1), 0)

    ht_sig_encoded = convolutional_encode(_ht_sig_bits(len(psdu), stbc))
    ht_sig_interleaved = np.concatenate(
        [interleave(part, 48, 1) for part in ht_sig_encoded.reshape(2, 48)]
    )
    ht_sig = _ofdm_symbols(
        map_constellation(ht_sig_interleaved, 1), 1, data_rotation=1j
    )
    ht_stf, ht_ltf = _ht_training_fields()

    data_bits = np.zeros(n_sym * n_dbps, dtype=np.uint8)
    psdu_bits = bits_from_bytes(psdu)
    data_bits[16 : 16 + len(psdu_bits)] = psdu_bits
    scrambled = scramble(data_bits, scrambler_seed)
    tail_start = 16 + len(psdu_bits)
    scrambled[tail_start : tail_start + 6] = 0
    data_encoded = convolutional_encode(scrambled)
    data_interleaved = ht_interleave(data_encoded, 52, 1)
    data = _ht_ofdm_symbols(
        map_constellation(data_interleaved, 1),
        stbc_first_chain=bool(stbc),
        stbc_collapsed=stbc_collapsed,
    )
    fields = [legacy_preamble(), l_signal, ht_sig, ht_stf, ht_ltf]
    if stbc:
        if stbc_collapsed:
            h1 = VIRTUAL_HT_STBC_H1
            h2 = VIRTUAL_HT_STBC_H2
            stream2 = h2 * HT_STBC_STREAM2_CSD
            ltf_frequency = _ht_ltf_frequency()
            fields[-2] = _ifft_symbol(h1 * SHORT + stream2 * SHORT)
            fields[-1] = _ifft_symbol(h1 * ltf_frequency + stream2 * ltf_frequency)
            fields.append(
                _ifft_symbol(-h1 * ltf_frequency + stream2 * ltf_frequency)
            )
        else:
            fields.append(stbc_ltf2_sign * ht_ltf)
    fields.append(data)
    return np.concatenate(fields)


def synthesize_ht20_mcs8(psdu: bytes, h1: complex = 1, h2: complex = .5j) -> np.ndarray:
    """Two BPSK spatial streams summed through a fixed one-receiver channel.

    This provides a reproducible 2SS header/payload stimulus for a single RF
    output. It does not emulate two independently positioned transmitters.
    Legacy fields use the first-chain mapping; HT training maps both streams.
    """
    n_sym = math.ceil((16 + 8 * len(psdu) + 6) / 52)
    length = 3 * (5 + n_sym) - 3  # two HT-LTFs
    signal = interleave(convolutional_encode(_signal_bits(RATE_PARAMETERS[6][0], length)), 48, 1)
    sig = _ht_sig_bits(len(psdu))
    sig[:7] = [(8 >> bit) & 1 for bit in range(7)]
    sig[34:42] = ht_sig_crc(sig[:34])
    coded_sig = convolutional_encode(sig)
    sig_bits = np.concatenate([interleave(part, 48, 1) for part in coded_sig.reshape(2, 48)])
    stream2 = h2 * HT_STBC_STREAM2_CSD
    ltf = _ht_ltf_frequency()
    fields = [h1 * legacy_preamble(), h1 * _ofdm_symbols(map_constellation(signal, 1), 0),
              h1 * _ofdm_symbols(map_constellation(sig_bits, 1), 1, data_rotation=1j),
              _ifft_symbol((h1 + stream2) * SHORT),
              _ifft_symbol((h1 + stream2) * ltf), _ifft_symbol((-h1 + stream2) * ltf)]
    data = np.zeros(n_sym * 52, dtype=np.uint8)
    data[16:16+8*len(psdu)] = bits_from_bytes(psdu)
    data = scramble(data, 93)
    data[16+8*len(psdu):22+8*len(psdu)] = 0
    coded = convolutional_encode(data)
    streams = [ht_interleave(coded[ss::2], 52, 1).reshape(n_sym, 52) for ss in range(2)]
    # Third permutation: frequency rotation of the second stream, N_ROT=11.
    streams[1] = np.roll(streams[1], -22, axis=1)
    pilots = ((1, 1, -1, -1), (1, -1, -1, 1))
    for n in range(n_sym):
        bins = []
        for ss in range(2):
            f = np.zeros(64, dtype=np.complex64)
            f[np.array(HT_DATA_SUBCARRIERS)+32] = map_constellation(streams[ss][n], 1)
            for j, k in enumerate(PILOT_SUBCARRIERS):
                f[k+32] = pilots[ss][(n+j)%4] * POLARITY[(n+3)%127]
            bins.append(f)
        fields.append(_ifft_symbol(h1*bins[0]+stream2*bins[1]))
    return np.concatenate(fields)


def with_guard_and_gap(
    packet: np.ndarray, sample_rate: int, guard_us: float, gap_ms: float
) -> np.ndarray:
    guard = np.zeros(round(guard_us * sample_rate / 1e6), dtype=np.complex64)
    gap = np.zeros(round(gap_ms * sample_rate / 1e3), dtype=np.complex64)
    waveform = np.concatenate((guard, packet, guard, gap))
    peak = np.abs(waveform).max(initial=0)
    if peak:
        waveform *= np.float32(0.7 / peak)
    return waveform


def write_waveform(path: pathlib.Path, waveform: np.ndarray, output_type: str) -> None:
    if output_type == "cf32":
        waveform.astype(np.complex64).tofile(path)
    elif output_type == "hackrf":
        interleaved = np.empty(waveform.size * 2, dtype=np.int8)
        scaled = np.clip(np.column_stack((waveform.real, waveform.imag)) * 120, -127, 127)
        interleaved[:] = scaled.astype(np.int8).ravel()
        interleaved.tofile(path)
    else:
        raise ValueError(output_type)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--format", choices=("nonht", "11b", "ht20"), default="nonht")
    parser.add_argument("--rate", type=float, default=6)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--output-type", choices=("cf32", "hackrf"), default="cf32")
    parser.add_argument("--sequence", type=int, default=0)
    parser.add_argument("--scrambler-seed", type=int, default=93)
    parser.add_argument("--stbc", action="store_true", help="assert STBC and add HT-LTF2")
    parser.add_argument(
        "--stbc-collapsed",
        action="store_true",
        help="apply a nondegenerate virtual 2x1 channel and transmit its sum on one RF chain",
    )
    parser.add_argument("--stbc-ltf2-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--guard-us", type=float, default=20.0)
    parser.add_argument("--gap-ms", type=float, default=20.0)
    parser.add_argument(
        "--corrupt-payload",
        action="store_true",
        help="flip bits in the FCS-protected payload but leave the FCS field a "
             "valid checksum of the original frame (payload bit error)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="number of PPDUs per waveform, with consecutive sequence numbers "
             "(so receivers can cluster by sequence number instead of seeing "
             "one endlessly repeated frame)",
    )
    parser.add_argument(
        "--corrupt-fcs",
        action="store_true",
        help="invert the FCS so the frame is otherwise valid but fails the CRC check",
    )
    args = parser.parse_args()

    if args.format == "nonht":
        if args.rate not in RATE_PARAMETERS:
            parser.error("nonht rate must be one of: " + ", ".join(map(str, RATE_PARAMETERS)))
        sample_rate = 20_000_000
        synth = lambda psdu: synthesize_nonht(psdu, args.rate, args.scrambler_seed)
    elif args.format == "11b":
        if args.rate not in (1, 2, 5.5, 11):
            parser.error("11b rate must be 1, 2, 5.5 or 11")
        sample_rate = 11_000_000
        synth = lambda psdu: synthesize_dsss(psdu, args.rate)
    else:
        if args.rate != 0:
            parser.error("ht20 currently supports MCS 0; use --rate 0")
        sample_rate = 20_000_000
        synth = lambda psdu: synthesize_ht20(
            psdu,
            args.scrambler_seed,
            int(args.stbc),
            args.stbc_ltf2_sign,
            args.stbc_collapsed,
        )

    if args.count < 1:
        parser.error("--count must be >= 1")

    # One waveform carries --count PPDUs with CONSECUTIVE sequence numbers, so a
    # receiver clustering by (source, sequence) sees distinct packets instead of
    # one frame repeated forever by the cyclic transmit buffer.
    packets = []
    for index in range(args.count):
        psdu = make_probe_request(sequence=args.sequence + index,
                                  corrupt_payload=args.corrupt_payload)
        if args.corrupt_fcs:
            psdu = psdu[:-4] + bytes(b ^ 0xFF for b in psdu[-4:])
        packets.append(synth(psdu))
    packet = packets[0]

    waveform = np.concatenate([
        with_guard_and_gap(p, sample_rate, args.guard_us, args.gap_ms) for p in packets
    ])
    write_waveform(args.output, waveform, args.output_type)
    print(
        f"wrote {args.output}: {len(psdu)}-byte PSDU x{args.count} (seq {args.sequence}..{args.sequence + args.count - 1}), "
        f"{len(packet)} packet samples, "
        f"{len(waveform)} cyclic samples at {sample_rate / 1e6:g} Msps"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

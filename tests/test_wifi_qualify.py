# SPDX-License-Identifier: LGPL-3.0-or-later
"""Exercise sensor WiFi qualification against waveforms and real RF captures."""
import ctypes as C
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from scipy.signal import resample_poly

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import wifi_ppdu as phy


class Packet(C.Structure):
    _fields_ = [(name, C.c_uint16) for name in
                ('frame_start', 'training_start', 'training_words', 'length_bytes', 'duration_us')]
    _fields_ += [('cfo_hz', C.c_int32)]
    _fields_ += [(name, C.c_uint8) for name in
                 ('phy', 'rate_half_mbps', 'mcs', 'stbc', 'quality', 'prefix_length')]
    _fields_ += [('prefix', C.c_uint8 * 24), ('transmitter', C.c_uint8 * 6)]
    _fields_ += [(name, C.c_bool) for name in ('mac_valid', 'crc_checked', 'crc_valid')]


def packed(samples, cfo=0, noise=0, dc=0):
    z = np.asarray(samples, dtype=np.complex128)
    z = z / max(abs(z).max(), 1e-9) * 240
    z *= np.exp(2j * np.pi * cfo * np.arange(len(z)) / 20e6)
    rng = np.random.default_rng(79)
    z += dc + noise * (rng.normal(size=len(z)) + 1j * rng.normal(size=len(z)))
    i = np.clip(np.rint(z.real), -512, 511).astype(np.int32)
    q = np.clip(np.rint(-z.imag), -512, 511).astype(np.int32)
    return ((i & 1023) | ((q & 1023) << 10)).astype(np.uint32)


class WiFiQualificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        library = Path(cls.directory.name) / 'qualify.so'
        subprocess.run(['cc', '-I'+str(ROOT/'include'), '-shared', '-fPIC', '-O2', '-Wall', '-Wextra', '-Werror',
                        str(ROOT / 'src/wifi.c'), '-lm', '-o', str(library)], check=True)
        cls.lib = C.CDLL(str(library))
        cls.lib.epd_wifi_qualify_workspace_size.restype = C.c_size_t
        cls.scratch = C.create_string_buffer(cls.lib.epd_wifi_qualify_workspace_size())
        cls.fn = cls.lib.epd_wifi_qualify
        cls.fn.argtypes = [C.POINTER(C.c_uint32), C.c_uint32, C.c_uint32, C.c_uint32, C.c_void_p, C.POINTER(Packet)]
        cls.fn.restype = C.c_bool

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def admit(self, samples, offset=0):
        self.assertLessEqual(len(samples), 16384)
        ring = np.full(16384, 0xdeadbeef, dtype=np.uint32)
        ring[(offset + np.arange(len(samples))) % len(ring)] = samples
        packet = Packet()
        valid = self.fn(ring.ctypes.data_as(C.POINTER(C.c_uint32)), len(ring), offset,
                        len(samples), self.scratch, C.byref(packet))
        return valid, packet

    def test_standard_constellation_bit_order(self):
        # IEEE 802.11-2012 18.3.5.8: b0 arrives first and controls I's sign.
        for bits, expected, norm in (([1,0],1-1j,2),([1,0,0,1],3-1j,10),
                                      ([0,1,1,0],-1+3j,10),
                                      ([1,0,1,0,1,0],5-1j,42),
                                      ([0,1,1,1,0,0],-3+7j,42)):
            point=phy.map_constellation(np.array(bits,dtype=np.uint8),len(bits))[0]
            self.assertAlmostEqual(abs(point-expected/np.sqrt(norm)),0,places=6)

    def test_native_ofdm_modulations(self):
        with np.load(ROOT/'tests/fixtures/wifi_ofdm_native.npz') as captures:
            for name, samples in captures.items():
                rate=int(name.rsplit('_',1)[1])
                valid,p=self.admit(samples,16031)
                self.assertTrue(valid,name)
                self.assertEqual(p.rate_half_mbps,rate*2,name)
                self.assertTrue(p.mac_valid,name)
                self.assertEqual(bytes(p.transmitter),bytes.fromhex('024553504951'),name)
                # The 102.4-us stream window omits the last 6-Mbps payload bits.
                self.assertEqual(p.crc_valid,rate!=6,name)
                self.assertEqual(p.crc_checked,rate!=6,name)

    def test_control_frame_transmitter_identity(self):
        import zlib
        receiver = bytes.fromhex('024553504950')
        transmitter = bytes.fromhex('024553504951')
        for subtype, has_ta in ((8, True), (9, True), (10, True), (11, True),
                                (12, False), (13, False)):
            frame = bytes([(subtype << 4) | 4, 0, 0, 0]) + receiver
            if has_ta:
                frame += transmitter
            frame += zlib.crc32(frame).to_bytes(4, 'little')
            valid, packet = self.admit(packed(np.r_[np.zeros(83),
                                                   phy.synthesize_nonht(frame, 6),
                                                   np.zeros(100)]))
            self.assertTrue(valid, subtype)
            self.assertTrue(packet.crc_valid, subtype)
            self.assertEqual(packet.mac_valid, has_ta, subtype)
            if has_ta:
                self.assertEqual(bytes(packet.transmitter), transmitter)

    def test_bounded_extended_headers(self):
        fn = self.lib.epd_wifi_qualify_extended_header
        fn.argtypes = [C.POINTER(C.c_uint32), C.c_uint32, C.c_void_p, C.POINTER(Packet)]
        fn.restype = C.c_bool
        training_fn = self.lib.epd_wifi_qualify_ofdm_training
        training_fn.argtypes = fn.argtypes
        training_fn.restype = C.c_bool
        at_fn = self.lib.epd_wifi_qualify_extended_at
        at_fn.argtypes = [C.POINTER(C.c_uint32), C.c_uint32, C.POINTER(Packet),
                          C.c_void_p, C.POINTER(Packet)]
        at_fn.restype = C.c_bool
        psdu = phy.make_probe_request(sequence=59)
        for mcs, synth in ((0, phy.synthesize_ht20), (8, phy.synthesize_ht20_mcs8)):
            wave = synth(psdu)
            # Early ADC windows omit most STF and all payload. Retain enough
            # L-LTF and HT-SIG to validate MCS independently of native RX.
            for begin in (20, 100, 160):
                samples = packed(wave, cfo=21000, noise=.5)[begin:begin+768].copy()
                packet = Packet()
                self.assertTrue(training_fn(samples.ctypes.data_as(C.POINTER(C.c_uint32)),
                                            len(samples), self.scratch, C.byref(packet)), (mcs, begin))
                self.assertEqual(packet.training_words, 128)
                self.assertFalse(packet.mac_valid)
                self.assertFalse(packet.crc_checked)
                self.assertTrue(at_fn(samples.ctypes.data_as(C.POINTER(C.c_uint32)),
                                      len(samples), C.byref(packet), self.scratch,
                                      C.byref(packet)), (mcs, begin))
                self.assertEqual(packet.phy, 2)
                self.assertEqual(packet.mcs, mcs)
                self.assertTrue(fn(samples.ctypes.data_as(C.POINTER(C.c_uint32)),
                                   len(samples), self.scratch, C.byref(packet)), (mcs, begin))
                self.assertEqual(packet.phy, 2)
                self.assertEqual(packet.mcs, mcs)
                self.assertEqual(packet.length_bytes, len(psdu))
                self.assertFalse(packet.mac_valid)
                self.assertFalse(packet.crc_checked)
        # A valid L-SIG followed by damaged HT-SIG is not sufficient.
        damaged = phy.synthesize_ht20_mcs8(psdu).copy()
        damaged[400:560] = 0
        samples = packed(damaged)[20:788].copy()
        packet = Packet()
        self.assertTrue(training_fn(samples.ctypes.data_as(C.POINTER(C.c_uint32)),
                                    len(samples), self.scratch, C.byref(packet)))
        self.assertFalse(at_fn(samples.ctypes.data_as(C.POINTER(C.c_uint32)),
                               len(samples), C.byref(packet), self.scratch, C.byref(packet)))
        self.assertFalse(fn(samples.ctypes.data_as(C.POINTER(C.c_uint32)),
                            len(samples), self.scratch, C.byref(packet)))
        legacy = packed(phy.synthesize_nonht(psdu, 6))[:768].copy()
        packet = Packet()
        self.assertFalse(fn(legacy.ctypes.data_as(C.POINTER(C.c_uint32)),
                            len(legacy), self.scratch, C.byref(packet)))

    def test_native_training_validation(self):
        fn = self.lib.epd_wifi_qualify_native
        fn.argtypes = [C.POINTER(C.c_uint32), C.c_uint32, C.c_bool, C.c_int,
                       C.c_void_p, C.POINTER(Packet)]
        fn.restype = C.c_bool
        for fixture, dsss in (('wifi_ofdm_native.npz', False), ('wifi_cck_iq.npz', True)):
            with np.load(ROOT / 'tests/fixtures' / fixture) as captures:
                for name, samples in captures.items():
                    padded = np.r_[np.zeros(400, dtype=np.uint32), samples]
                    valid, original = self.admit(padded)
                    self.assertTrue(valid, name)
                    begin = max(0, original.training_start - 80)
                    count = 1000 if dsss else 288
                    window = padded[begin:begin+count].copy()
                    p = Packet()
                    self.assertTrue(fn(window.ctypes.data_as(C.POINTER(C.c_uint32)),
                                       len(window), dsss, original.training_start-begin,
                                       self.scratch, C.byref(p)), name)
                    if not dsss:
                        for shift in (-96, 96):
                            fallback = Packet()
                            self.assertTrue(fn(window.ctypes.data_as(C.POINTER(C.c_uint32)),
                                               len(window), False, original.training_start-begin+shift,
                                               self.scratch, C.byref(fallback)), (name, shift))
                            self.assertLessEqual(fallback.training_start+fallback.training_words, len(window))
                            self.assertFalse(fallback.mac_valid)
                            self.assertFalse(fallback.crc_checked)
                    self.assertFalse(p.mac_valid, name)
                    self.assertFalse(p.crc_checked, name)
                    self.assertLess(abs(p.cfo_hz-original.cfo_hz), 2000, name)
                    self.assertLessEqual(p.training_start+p.training_words, len(window))
        noise = np.random.default_rng(128).integers(0, 1 << 20, 1280, dtype=np.uint32)
        for dsss in (False, True):
            p = Packet()
            self.assertFalse(fn(noise.ctypes.data_as(C.POINTER(C.c_uint32)),
                                len(noise), dsss, 320, self.scratch, C.byref(p)))

    def test_control_frame_addresses(self):
        import binascii
        address = bytes.fromhex('020000000001')
        for fc, body, expected_mac in ((0xb4, bytes.fromhex('ffffffffffff')+address, True),
                                       (0xd4, address, False)):
            frame = bytes([fc, 0, 0, 0]) + body
            frame += binascii.crc32(frame).to_bytes(4, 'little')
            valid, p = self.admit(packed(np.r_[np.zeros(83), phy.synthesize_nonht(frame), np.zeros(100)]))
            self.assertTrue(valid)
            self.assertTrue(p.crc_valid)
            self.assertEqual(p.mac_valid, expected_mac)
            if expected_mac:
                self.assertEqual(bytes(p.transmitter), address)

    def test_two_spatial_stream_header(self):
        psdu = phy.make_probe_request(sequence=59)
        valid, p = self.admit(packed(np.r_[np.zeros(83), phy.synthesize_ht20_mcs8(psdu), np.zeros(100)], cfo=27000))
        self.assertTrue(valid)
        self.assertEqual((p.phy, p.mcs, p.stbc), (2, 8, 0))
        self.assertEqual(p.length_bytes, len(psdu))
        self.assertFalse(p.mac_valid)
        self.assertFalse(p.crc_checked)

    def test_cck_captured_channels(self):
        # Stream crops can begin within the preamble; idle padding lets the
        # general acquisition search inspect the retained SFD/PLCP history.
        with np.load(ROOT / 'tests/fixtures/wifi_cck_iq.npz') as captures:
            for name, samples in captures.items():
                valid, p = self.admit(np.r_[np.zeros(400, dtype=np.uint32), samples], 15931)
                self.assertTrue(valid, name)
                self.assertEqual(p.rate_half_mbps, 11 if '55' in name or name in ('native_rate2','native_rate6') else 22)
                self.assertTrue(p.crc_valid, (name, bytes(p.prefix)))
                self.assertEqual(bytes(p.transmitter), bytes.fromhex('024553504951' if name.startswith('native') else '020000000001'))

    def test_cck_payload(self):
        psdu = phy.make_probe_request(sequence=61)
        for short in (False, True):
            for rate in (5.5, 11):
                for cfo in (0, 18000, -23000):
                    wave = resample_poly(phy.synthesize_dsss(psdu, rate, short_preamble=short), 20, 11)
                    valid, p = self.admit(packed(np.r_[np.zeros(93), wave, np.zeros(100)], cfo=cfo))
                    self.assertTrue(valid, (short, rate, cfo))
                    self.assertTrue(p.mac_valid, (short, rate, cfo, bytes(p.prefix)))
                    self.assertEqual(bytes(p.transmitter), bytes.fromhex('020000000001'))
                    self.assertTrue(p.crc_valid, (short, rate, cfo, bytes(p.prefix)))

    def test_cck_bad_fcs_and_partial_payload(self):
        psdu = phy.make_probe_request(sequence=62)
        for short in (False, True):
            for rate in (5.5, 11):
                damaged = psdu[:-1] + bytes([psdu[-1] ^ 0x80])
                wave = resample_poly(phy.synthesize_dsss(damaged, rate, short_preamble=short), 20, 11)
                valid, p = self.admit(packed(np.r_[np.zeros(93), wave, np.zeros(100)]))
                self.assertTrue(valid)
                self.assertTrue(p.crc_checked)
                self.assertFalse(p.crc_valid)
                self.assertFalse(p.mac_valid)
                wave = resample_poly(phy.synthesize_dsss(psdu, rate, short_preamble=short), 20, 11)
                # Retain the MAC header but omit the FCS and final payload bytes.
                wave = wave[:-int(16 * 8 / rate * 20)]
                valid, p = self.admit(packed(np.r_[np.zeros(93), wave]))
                self.assertTrue(valid)
                self.assertFalse(p.crc_checked)
                self.assertTrue(p.mac_valid)
                self.assertEqual(bytes(p.transmitter), bytes.fromhex('020000000001'))

    def test_short_dsss(self):
        psdu = phy.make_probe_request(sequence=57)
        for rate in (2, 5.5, 11):
            wave = resample_poly(phy.synthesize_dsss(psdu, rate, short_preamble=True), 20, 11)
            valid, p = self.admit(packed(np.r_[np.zeros(93), wave, np.zeros(100)], cfo=18000))
            self.assertTrue(valid, rate)
            self.assertEqual(p.phy, 1)
            self.assertEqual(p.rate_half_mbps, rate*2)
            self.assertEqual(p.length_bytes, len(psdu))
            if rate == 2:
                self.assertTrue(p.mac_valid)
                self.assertTrue(p.crc_valid)

    def test_legacy_rates_mac_and_crc(self):
        psdu = phy.make_probe_request(sequence=351)
        for rate in phy.RATE_PARAMETERS:
            samples = packed(np.r_[np.zeros(83), phy.synthesize_nonht(psdu, rate), np.zeros(100)],
                             cfo=43000, noise=.5, dc=17+9j)
            for offset in (0, 16001):
                ok, packet = self.admit(samples, offset)
                self.assertTrue(ok, (rate, offset))
                self.assertEqual(packet.rate_half_mbps, rate * 2)
                self.assertTrue(packet.mac_valid)
                self.assertEqual(bytes(packet.transmitter), bytes.fromhex('020000000001'))
                self.assertEqual(bytes(packet.prefix), psdu[:24])
                self.assertTrue(packet.crc_checked)
                self.assertTrue(packet.crc_valid, rate)

    def test_standalone_barker_rejects_tones(self):
        fn = self.lib.epd_wifi_qualify_dsss_training
        fn.argtypes = [C.POINTER(C.c_uint32), C.c_uint, C.c_int, C.c_void_p, C.POINTER(Packet)]
        fn.restype = C.c_bool
        wave = resample_poly(phy.synthesize_dsss(phy.make_probe_request(), 11), 20, 11)
        for z, expected in [(wave[200:1224], True),
                            (np.exp(2j*np.pi*.1*np.arange(1024)), False),
                            (np.ones(1024), False)]:
            iq = packed(z, cfo=27000, noise=.5, dc=10+7j)
            p = Packet()
            self.assertEqual(fn(iq.ctypes.data_as(C.POINTER(C.c_uint32)), len(iq), 100, self.scratch, C.byref(p)), expected)

    def test_11b_headers_all_rates(self):
        for rate in (1, 2, 5.5, 11):
            waveform = resample_poly(phy.synthesize_dsss(phy.make_probe_request(), rate), 20, 11)
            ok, packet = self.admit(packed(np.r_[np.zeros(83), waveform, np.zeros(100)],
                                          cfo=27000, noise=.5, dc=20-11j), 15000)
            self.assertTrue(ok, rate)
            self.assertEqual(packet.phy, 1)
            self.assertEqual(packet.rate_half_mbps, rate * 2)
            if rate in (1, 2, 5.5, 11):
                self.assertTrue(packet.mac_valid)
                self.assertTrue(packet.crc_valid)
            else:
                self.assertFalse(packet.mac_valid)
                self.assertFalse(packet.crc_checked)

    def test_bad_payload_crc_is_not_claimed_valid(self):
        psdu = phy.make_probe_request()
        bad = psdu[:-4] + bytes(byte ^ 255 for byte in psdu[-4:])
        for waveform in (phy.synthesize_nonht(bad), resample_poly(phy.synthesize_dsss(bad), 20, 11)):
            ok, packet = self.admit(packed(np.r_[np.zeros(83), waveform, np.zeros(100)]))
            self.assertTrue(ok)
            self.assertTrue(packet.crc_checked)
            self.assertFalse(packet.crc_valid)

    def test_invalid_lsig_and_negative_rf(self):
        bits = phy._signal_bits(13, 45)
        bits[17] ^= 1
        signal = phy._ofdm_symbols(phy.map_constellation(phy.interleave(phy.convolutional_encode(bits), 48, 1), 1), 0)
        bad = np.r_[np.zeros(83), phy.legacy_preamble(), signal, np.zeros(200)]
        self.assertFalse(self.admit(packed(bad))[0])
        rng = np.random.default_rng(349)
        for n in range(12):
            z = rng.normal(size=8192) + 1j * rng.normal(size=8192)
            self.assertFalse(self.admit(packed(z))[0])
        for frequency in (0, .003, .05, .2):
            self.assertFalse(self.admit(packed(np.exp(2j*np.pi*frequency*np.arange(8192))))[0])

    def test_ht_training_does_not_invent_mac(self):
        for stbc in (0, 1):
            waveform = phy.synthesize_ht20(phy.make_probe_request(), stbc=stbc, stbc_collapsed=bool(stbc))
            ok, packet = self.admit(packed(np.r_[np.zeros(83), waveform, np.zeros(100)]))
            self.assertTrue(ok)
            self.assertEqual(packet.phy, 2)
            self.assertEqual(packet.stbc, stbc)
            self.assertFalse(packet.mac_valid)

    def test_real_captures_and_truncation(self):
        with np.load(ROOT / 'tests/fixtures/wifi_header_iq.npz') as captures:
            expected = {'native_ofdm': (0, 68), 'long_ofdm': (0, 1445), 'long_dsss': (1, 232), 'legacy_ht_false_match': (0, 1445)}
            for name, samples in captures.items():
                ok, packet = self.admit(samples, 15999)
                self.assertTrue(ok, name)
                self.assertEqual((packet.phy, packet.length_bytes), expected[name])
                self.assertFalse(packet.crc_checked)
                self.assertEqual(packet.mac_valid, name != 'native_ofdm')
                if packet.mac_valid:
                    self.assertEqual(bytes(packet.transmitter), bytes.fromhex('020000000001'))
                for count in (0, 1, 127, 128):
                    self.assertFalse(self.admit(samples[:count])[0])


if __name__ == '__main__':
    unittest.main()

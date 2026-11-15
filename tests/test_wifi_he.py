# SPDX-License-Identifier: LGPL-3.0-or-later
"""HE signaling from independent hardware and generated coding cases."""
import ctypes as C
import unittest
import numpy as np
from test_wifi_qualify import Packet, ROOT, packed
import test_wifi_qualify as qualification
import wifi_he

class HESignalingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        qualification.WiFiQualificationTests.setUpClass()
        cls.lib=qualification.WiFiQualificationTests.lib
        cls.scratch=qualification.WiFiQualificationTests.scratch
        cls.fn=cls.lib.epd_wifi_qualify_extended_header
        cls.fn.argtypes=[C.c_void_p,C.c_uint,C.c_void_p,C.POINTER(Packet)]
        cls.fn.restype=C.c_bool
    @classmethod
    def tearDownClass(cls):qualification.WiFiQualificationTests.tearDownClass()
    def decode(self,raw):
        p=Packet();ok=self.fn(raw.ctypes.data,len(raw),self.scratch,C.byref(p));return ok,p
    def test_generated_header(self):
        for streams in range(1,9):
            for mcs in range(12):
                bits=wifi_he.header_bits(streams,mcs,cp=mcs%4)
                wave=wifi_he.preamble(bits)
                raw=packed(np.r_[np.zeros(32),wave,np.zeros(16)],cfo=45000,noise=.4)
                ok,p=self.decode(raw)
                self.assertTrue(ok);self.assertEqual(p.phy,4,(streams,mcs))
                self.assertEqual(p.mcs,mcs);self.assertEqual(p.prefix_length,7)
                self.assertEqual(bytes(p.prefix[:7]),np.packbits(np.r_[bits,np.zeros(4,dtype='u1')],bitorder='little').tobytes())
                self.assertFalse(p.crc_checked);self.assertFalse(p.mac_valid);self.assertEqual(p.length_bytes,0)
    def test_invalid_and_truncated(self):
        for mode in ['crc','reserved','mcs']:
            bits=wifi_he.header_bits(bad_crc=mode=='crc',mcs=15 if mode=='mcs' else 0)
            if mode=='reserved':bits[14]=0
            ok,p=self.decode(packed(wifi_he.preamble(bits)))
            self.assertTrue(ok);self.assertEqual(p.phy,3)
        raw=packed(wifi_he.preamble(wifi_he.header_bits()))
        for length in [560,639]:
            ok,p=self.decode(raw[:length]);self.assertTrue(ok);self.assertNotEqual(p.phy,4)
    def test_stbc_dcm_constraints(self):
        for params in [dict(stbc=True,streams=1), dict(stbc=True,streams=4),
                       dict(dcm=True,mcs=9), dict(dcm=True,streams=3),
                       dict(dcm=True,stbc=True,cp=1,streams=2)]:
            ok,p=self.decode(packed(wifi_he.preamble(wifi_he.header_bits(**params))))
            self.assertTrue(ok);self.assertEqual(p.phy,3,params)
        ok,p=self.decode(packed(wifi_he.preamble(wifi_he.header_bits(stbc=True,streams=2))))
        self.assertTrue(ok);self.assertEqual(p.phy,4);self.assertEqual(p.stbc,1)

    def test_special_short_gi(self):
        from espargos_packet_decoders import wifi
        for streams in range(1,9):
            raw=packed(wifi_he.preamble(wifi_he.header_bits(streams,cp=4)))
            ok,p=self.decode(raw)
            self.assertTrue(ok);self.assertEqual(p.phy,4);self.assertEqual(p.stbc,0)
            packet=wifi(raw,stage='extended_header')
            self.assertEqual(packet['he']['spatial_streams'],streams)
            self.assertEqual(packet['he']['guard_interval_ns'],800)
            self.assertEqual(packet['he']['ltf_size'],4)
            self.assertFalse(packet['he']['dcm'])

    def test_real_sensor(self):
        data=np.load(ROOT/'tests/fixtures/wifi_he_sensor.npz')
        for raw in data.values():
            ok,p=self.decode(raw);self.assertTrue(ok);self.assertEqual(p.phy,4)
            self.assertEqual(p.mcs,0);self.assertEqual(bytes(p.prefix[:7]).hex(),'074060fc250500')

    def test_python_metadata(self):
        from espargos_packet_decoders import wifi
        for streams in range(1,9):
            raw=packed(wifi_he.preamble(wifi_he.header_bits(streams,mcs=7,cp=1)))
            packet=wifi(raw,stage='extended_header')
            self.assertEqual(packet['he']['spatial_streams'],streams)
            self.assertEqual(packet['he']['ltf_size'],2)
            self.assertEqual(packet['he']['guard_interval_ns'],800)
            self.assertFalse(packet['crc_checked'])

if __name__=='__main__':unittest.main()

# SPDX-License-Identifier: LGPL-3.0-or-later
"""Exercise real IQ admission, including circular input and negative RF controls."""
import ctypes as C
from pathlib import Path
import subprocess
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
class Packet(C.Structure):
    _fields_=[('aa',C.c_uint32),('crc',C.c_uint32),('prefix',C.c_uint8*32),('length',C.c_uint8),('valid',C.c_bool),('identity',C.c_bool)]

def waveform(rate=4_000_000,phy=0,aa=0x8e89bed6,channel=37,bad=False,noise=1,cfo=0):
    pdu=bytes.fromhex('0215c8620ebdf5f00e09')+b'ESPARGOS-TEST'
    crc=0xaaaaaa
    for b in pdu:
        for k in range(8):crc=(crc>>1)^(0xda6000 if (crc^(b>>k))&1 else 0)
    if bad:crc^=0x10101
    state=channel|64;bits=[]
    prefix=bytes([0x55 if aa&1 else 0xaa])*(2 if phy else 1)+aa.to_bytes(4,'little')
    bits=[(b>>k)&1 for b in prefix for k in range(8)]
    for b in pdu+crc.to_bytes(3,'little'):
        for k in range(8):
            bits.append(((b>>k)^state)&1)
            state=(state>>1)^(0x44 if state&1 else 0)
    sps=rate//(2_000_000 if phy else 1_000_000)
    symbols=np.repeat(np.array(bits)*2.-1,sps)
    t=np.arange(-4*sps,4*sps+1)/sps
    taps=np.exp(-2*(np.pi*.5*t)**2/np.log(2));taps/=taps.sum()
    shaped=np.convolve(np.pad(symbols,(4*sps,4*sps),mode='edge'),taps,'same')
    phase=np.cumsum(shaped)*2*np.pi*(500_000 if phy else 250_000)/rate
    signal=180*np.exp(-1j*phase)
    z=np.pad(signal,(int(rate*32e-6),int(rate*16e-6)))
    z*=np.exp(2j*np.pi*(2_000_000+cfo)*np.arange(len(z))/rate)
    rng=np.random.default_rng(85);z+=noise*(rng.normal(size=len(z))+1j*rng.normal(size=len(z)))
    return packed(z),pdu

def packed(z):
    i=np.clip(np.rint(z.real),-512,511).astype(np.int32)
    q=np.clip(np.rint(z.imag),-512,511).astype(np.int32)
    return ((i&1023)|((q&1023)<<10)).astype(np.uint32)

class AdmissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();so=Path(cls.tmp.name)/'qualify.so'
        subprocess.run(['cc','-I'+str(ROOT/'include'), '-shared','-fPIC','-O2','-Wall','-Wextra','-Werror',str(ROOT/'src/ble.c'),'-o',str(so)],check=True)
        cls.lib=C.CDLL(str(so));cls.fn=cls.lib.epd_ble_qualify
        cls.fn.argtypes=[C.POINTER(C.c_uint32)]+[C.c_uint]*6+[C.POINTER(C.c_int16),C.c_size_t,C.POINTER(Packet)]
        cls.fn.restype=C.c_bool
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def admit(self,z,rate=4_000_000,phy=0,channel=37,offset=0):
        buf=np.zeros(16384,dtype=np.uint32);buf[(offset+np.arange(len(z)))%len(buf)]=z
        scratch=(C.c_int16*16384)();p=Packet()
        ok=self.fn(buf.ctypes.data_as(C.POINTER(C.c_uint32)),len(buf),offset,len(z),rate,phy,channel,scratch,len(scratch),C.byref(p))
        return ok,p
    def test_rates_phys_wrap_and_frequency_error(self):
        for phy,rates in [(0,[4_000_000,8_000_000,16_000_000]),(1,[8_000_000,16_000_000])]:
            for rate in rates:
                for offset in [0,16000]:
                    z,pdu=waveform(rate,phy,cfo=30_000)
                    ok,p=self.admit(z,rate,phy,offset=offset)
                    self.assertTrue(ok,(rate,phy,offset));self.assertTrue(p.valid);self.assertTrue(p.identity)
                    self.assertEqual(bytes(p.prefix[:p.length]),pdu)
    def test_arbitrary_addresses_and_whitening(self):
        for aa in [0x12345678,0xd3a5c79b,0x71f26c84,0x9b58d2e7]:
            for channel in [0,19,37,39]:
                z,_=waveform(aa=aa,channel=channel)
                ok,p=self.admit(z,channel=channel)
                self.assertTrue(ok,(hex(aa),channel));self.assertEqual(p.aa,aa);self.assertFalse(p.valid)
    def test_coded_preamble_requires_host_identity(self):
        rng=np.random.default_rng(12)
        symbols=np.concatenate((np.tile([-1,-1,1,1,1,1,-1,-1],10),rng.choice([-1,1],520)))
        sps=4;t=np.arange(-16,17)/sps
        taps=np.exp(-2*(np.pi*.5*t)**2/np.log(2));taps/=taps.sum()
        shaped=np.convolve(np.repeat(symbols,sps),taps,'same')
        phase=np.cumsum(shaped)*2*np.pi*250_000/4_000_000
        z=np.pad(180*np.exp(-1j*phase),(128,64))
        z*=np.exp(1j*np.pi*np.arange(len(z)))
        ok,p=self.admit(packed(z))
        self.assertTrue(ok);self.assertFalse(p.identity);self.assertFalse(p.valid)
    def test_bad_crc_is_never_marked_valid(self):
        z,_=waveform(bad=True);ok,p=self.admit(z)
        self.assertTrue(ok);self.assertFalse(p.valid)
    def test_truncated_and_wrong_whitening(self):
        z,_=waveform();self.assertFalse(self.admit(z[:len(z)//2])[0])
        self.assertFalse(self.admit(z,channel=19)[0])
    def test_energy_is_not_admission(self):
        rng=np.random.default_rng(47)
        for _ in range(40):
            noise=packed((rng.normal(size=1600)+1j*rng.normal(size=1600))*100)
            self.assertFalse(self.admit(noise)[0])
        for freq in [0,.05,.25,.49]:
            tone=packed(200*np.exp(2j*np.pi*freq*np.arange(1600)))
            self.assertFalse(self.admit(tone)[0])
        carriers=(rng.choice([-1,1],(50,64))+1j*rng.choice([-1,1],(50,64)))
        ofdm=np.fft.ifft(carriers,axis=1)*600
        burst=np.concatenate((ofdm[:,-16:],ofdm),axis=1).ravel()
        self.assertFalse(self.admit(packed(burst))[0])

if __name__=='__main__':unittest.main()

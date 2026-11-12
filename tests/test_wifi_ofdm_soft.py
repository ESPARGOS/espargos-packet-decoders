# SPDX-License-Identifier: LGPL-3.0-or-later
"""Independent frequency-domain observations exercise MRC's sufficient statistics."""
import ctypes as C
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import numpy as np
from test_wifi_qualify import Packet, ROOT, phy, packed


def symbols(rate=12, corrupt=False):
    wave=phy.synthesize_nonht(phy.make_probe_request(corrupt_payload=corrupt),rate)
    bins=np.array([np.fft.fft(wave[start:start+64]) for start in range(336,len(wave)-63,80)])
    values=np.zeros((*bins.shape,3),dtype='f4')
    values[:,:,0]=bins.real;values[:,:,1]=bins.imag
    values[:,:,2]=np.abs(bins)>1e-5
    return values


class SoftOFDMTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory()
        library=Path(cls.tmp.name)/'soft.so'
        subprocess.run(['cc','-std=c99','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',
                        '-I'+str(ROOT/'include'),str(ROOT/'src/wifi.c'),'-lm','-o',str(library)],check=True)
        cls.lib=C.CDLL(str(library));cls.lib.epd_wifi_qualify_workspace_size.restype=C.c_size_t
        cls.scratch=C.create_string_buffer(cls.lib.epd_wifi_qualify_workspace_size())
        cls.fn=cls.lib.epd_wifi_decode_ofdm
        cls.fn.argtypes=[C.POINTER(C.c_float),C.c_uint,C.c_void_p,C.POINTER(Packet)]
        cls.fn.restype=C.c_bool

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def decode(self,values):
        values=np.ascontiguousarray(values,dtype='f4');p=Packet()
        ok=self.fn(values.ctypes.data_as(C.POINTER(C.c_float)),len(values),self.scratch,C.byref(p))
        return ok,p

    def test_rates_phase_and_evidence(self):
        for rate in [6,9,12,18,24,36,48,54]:
            x=symbols(rate)
            phase=np.exp(1j*np.linspace(-1.8,2.1,len(x)))[:,None]
            z=(x[:,:,0]+1j*x[:,:,1])*phase
            x[:,:,0]=z.real;x[:,:,1]=z.imag
            ok,p=self.decode(x)
            self.assertTrue(ok and p.crc_valid,rate)
            self.assertEqual(p.rate_half_mbps,2*rate)
            self.assertEqual(bytes(p.transmitter),bytes.fromhex('020000000001'))
            self.assertEqual(p.training_words,0)
            self.assertEqual(p.training_start,65535)
            ok,p=self.decode(x[:1]);self.assertTrue(ok);self.assertFalse(p.crc_checked)
        ok,p=self.decode(symbols(12,True));self.assertTrue(ok and p.crc_checked);self.assertFalse(p.crc_valid)

    def test_mixed_format_headers(self):
        for stbc in [0,1]:
            wave=phy.synthesize_ht20(phy.make_probe_request(),stbc=stbc)
            bins=np.array([np.fft.fft(wave[start:start+64]) for start in [336,416,496]])
            x=np.zeros((3,64,3),dtype='f4')
            x[:,:,0]=bins.real;x[:,:,1]=bins.imag;x[:,:,2]=np.abs(bins)>1e-5
            ok,p=self.decode(x)
            self.assertTrue(ok);self.assertEqual(p.phy,2);self.assertEqual(p.stbc,stbc)
            self.assertFalse(p.crc_checked,'HT-SIG CRC is not a payload FCS')
            self.assertEqual(p.training_words,0)
            ok,p=self.decode(x[:1]);self.assertTrue(ok);self.assertNotEqual(p.phy,2)
        # Extended OFDM recognition is a repeated L-SIG, not payload decoding.
        x=symbols()[:3].copy();x[1]=x[0]
        ok,p=self.decode(x);self.assertTrue(ok);self.assertEqual(p.phy,3)
        self.assertFalse(p.crc_checked)

    def test_reliability_not_constellation_snapping(self):
        weighted=uniform=0
        for seed in range(60):
            rng=np.random.default_rng(seed);x=symbols(12)
            active=np.flatnonzero(x[0,:,2]);data=[k for k in active if k not in [7,21,43,57]]
            bad=rng.choice(data,8,replace=False)
            # A frequency-selective fade yields larger equalized noise and a
            # correspondingly lower inverse variance. No bits are replaced.
            noise=(rng.normal(size=(len(x),len(bad)))+1j*rng.normal(size=(len(x),len(bad))))*1.5
            x[:,bad,0]+=noise.real;x[:,bad,1]+=noise.imag;x[:,bad,2]=.01
            ok,p=self.decode(x);weighted+=bool(ok and p.crc_valid)
            x[:,bad,2]=1
            ok,p=self.decode(x);uniform+=bool(ok and p.crc_valid)
        self.assertGreater(weighted,50)
        self.assertGreater(weighted,uniform+25)
        print('Soft OFDM reliability recovery:',weighted,'vs uniform',uniform,'of 60')

    def test_invalid_and_erasure_inputs(self):
        for value in [np.nan,np.inf,-np.inf]:
            for field in range(3):
                x=symbols();x[0,7,field]=value;self.assertFalse(self.decode(x)[0])
        x=symbols();x[0,7,2]=-1;self.assertFalse(self.decode(x)[0])
        x=symbols();x[:,:,2]=0;self.assertFalse(self.decode(x)[0])
        self.assertFalse(self.decode(np.zeros((201,64,3),dtype='f4'))[0])
        # One erased data carrier must be an erasure, not a fabricated bit.
        x=symbols();x[:,3,:]=0;self.assertTrue(self.decode(x)[1].crc_valid)

    def test_workspace_reuse_and_truncation(self):
        raw=self.lib.epd_wifi_qualify
        raw.argtypes=[C.POINTER(C.c_uint32),C.c_uint32,C.c_uint32,C.c_uint32,C.c_void_p,C.POINTER(Packet)]
        raw.restype=C.c_bool
        for rate in [6,9,12,18,24,36,48,54]:
            x=symbols(rate)
            # Every possible symbol boundary, including header-only, exercises
            # the symbol-count-to-capture-length mapping without trailing data.
            for end in range(1,len(x)+1):
                ok,p=self.decode(x[:end]);self.assertTrue(ok)
                if end==len(x):self.assertTrue(p.crc_valid)
            iq=packed(phy.synthesize_nonht(phy.make_probe_request(),rate))
            p=Packet()
            self.assertTrue(raw(iq.ctypes.data_as(C.POINTER(C.c_uint32)),len(iq),0,len(iq),self.scratch,C.byref(p)))
            self.assertTrue(p.crc_valid)
            self.assertTrue(self.decode(x)[1].crc_valid)
            bad=x.copy();bad[0,7,2]=-1
            self.assertFalse(self.decode(bad)[0])
            self.assertTrue(raw(iq.ctypes.data_as(C.POINTER(C.c_uint32)),len(iq),0,len(iq),self.scratch,C.byref(p)))
            self.assertTrue(p.crc_valid)

    def test_wasm_binding(self):
        directory=os.environ.get('EPD_WASM_DIR')
        if not directory:self.skipTest('Set EPD_WASM_DIR to exercise the WASM binding')
        cases=[symbols(rate).reshape(-1).tolist() for rate in [6,12,24,54]]
        source=Path(self.tmp.name)/'cases.json';source.write_text(json.dumps(cases))
        script="""const D=require(process.argv[1]+'/packet_decoders.js');
D.create().then(d=>{const a=require(process.argv[2]);const out=a.map(x=>d.wifiOfdm(Float32Array.from(x)));
for(const x of [new Float32Array(),new Float32Array(191),new Float32Array(201*192)]) {
let rejected=false;try{d.wifiOfdm(x)}catch(e){rejected=e instanceof RangeError}if(!rejected)throw Error('bounds');}
d.destroy();let rejected=false;try{d.wifiOfdm(Float32Array.from(a[0]))}catch(e){rejected=true}if(!rejected)throw Error('destroy');
console.log(JSON.stringify(out));});"""
        result=json.loads(subprocess.check_output(['node','-e',script,directory,str(source)]))
        self.assertEqual([p['rate_half_mbps'] for p in result],[12,24,48,108])
        self.assertTrue(all(p['crc_valid'] and not p['training_words'] for p in result))

if __name__=='__main__':unittest.main()

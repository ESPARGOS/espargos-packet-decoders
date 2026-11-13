# SPDX-License-Identifier: LGPL-3.0-or-later
"""Cross-runtime fixture parity. Build Python extension and WASM before running.

EPD_WASM_DIR points at build_wasm.py output (defaults to dist/).
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, os.environ.get('EPD_PYTHON_DIR', str(ROOT/'python')))
from espargos_packet_decoders import wifi, ble, dsss_symbols
from test_ble_qualify import waveform
from test_wifi_qualify import colored_ofdm_iq

class BindingTests(unittest.TestCase):
    def test_host_wasm_parity(self):
        wasm = Path(os.environ.get('EPD_WASM_DIR', ROOT/'dist'))
        self.assertTrue((wasm/'epd.wasm').exists(), 'Build WASM before cross-runtime tests')
        cases = []
        for fixture in ['wifi_ofdm_native.npz', 'wifi_cck_iq.npz', 'wifi_header_iq.npz']:
            with np.load(ROOT/'tests/fixtures'/fixture) as captures:
                for name, iq in captures.items():
                    cases.append(dict(kind='wifi', iq=iq.tolist(), options={}, name=name))
                    decoded = wifi(iq)
                    if decoded:
                        stage = 'native_dsss' if decoded['phy'] == 1 else 'native_ofdm'
                        cases.append(dict(kind='wifi', iq=iq.tolist(), options=dict(stage=stage, training_hint=decoded['training_start']), name=name+' training'))
        for rate, snr, seed in [(6,8,1), (12,14,4), (24,20,3)]:
            cases.append(dict(kind='wifi', iq=colored_ofdm_iq(rate,snr,seed).tolist(),
                              options={}, name=f'noise-weighted retry {rate}'))
        for rate in (4_000_000, 8_000_000, 16_000_000):
            for phy in (0, 1):
                if phy == 1 and rate == 4_000_000:
                    continue
                for bad in (False, True):
                    iq, _ = waveform(rate=rate, phy=phy, bad=bad)
                    cases.append(dict(kind='ble', iq=iq.tolist(), options=dict(sample_rate=rate, phy=phy), name=f'BLE {rate} {phy} bad={bad}'))
        noise = np.random.default_rng(91).integers(0, 1<<20, 2048, dtype=np.uint32)
        for kind in ['wifi', 'ble']:
            cases.append(dict(kind=kind, iq=noise.tolist(), options={}, name=kind+' noise'))
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp)/'cases.json'; source.write_text(json.dumps(cases))
            results = json.loads(subprocess.check_output(['node', str(ROOT/'tests/wasm_decode.js'), str(wasm), str(source)]))
        for test, actual in zip(cases, results, strict=True):
            expected = (wifi if test['kind']=='wifi' else ble)(test['iq'], **test['options'])
            if expected:
                for field in ['prefix', 'transmitter']:
                    if field in expected: expected[field] = list(expected[field])
            # Different libm implementations may round CFO by 1 Hz.
            if expected and actual and 'cfo_hz' in expected:
                self.assertLessEqual(abs(actual['cfo_hz'] - expected['cfo_hz']), 1, test['name'])
                actual['cfo_hz'] = expected['cfo_hz']
            self.assertEqual(actual, expected, test['name'])
        self.assertIsNone(results[-1]); self.assertIsNone(results[-2])

    def test_binding_bounds(self):
        for iq in ([], [0]*16385, b'bad'):
            with self.assertRaises(ValueError): wifi(iq)
        self.assertIsNone(wifi([0]*512, stage='native_ofdm', training_hint=-1000))
        with self.assertRaises(ValueError): ble([0]*512, channel=40)

    def test_input_buffers_and_options(self):
        iq, _ = waveform()
        expected = ble(iq)
        self.assertTrue(expected['crc_valid'])
        for raw in (iq.tobytes(), bytearray(iq.tobytes()), memoryview(iq), memoryview(iq.tobytes())):
            self.assertEqual(ble(raw), expected)
        for stage in ('unknown', 'constructor', 'toString'):
            with self.assertRaises(ValueError): wifi([0]*512, stage=stage)
        for options in ({'channel': -1}, {'channel': 40}, {'sample_rate': 5000000},
                        {'phy': 2}, {'phy': 1, 'sample_rate': 4000000}):
            with self.assertRaises(ValueError): ble([0]*512, **options)
        with self.assertRaises(TypeError): wifi([0]*512, training_hint=1.5)
        with self.assertRaises(ValueError): wifi([0]*512, training_hint=2**32)
        for options in ({'training': -1}, {'training_words': 2**32}, {'cfo': float('nan')}):
            with self.assertRaises(ValueError): dsss_symbols([0]*512, **options)

    def test_javascript_options_and_ownership(self):
        directory = str(Path(os.environ.get('EPD_WASM_DIR', ROOT/'dist')).resolve())
        script = r"""
const assert=require('assert'),D=require(process.argv[1]+'/packet_decoders.js');
(async()=>{
 const d=await D.create(),raw=new Uint32Array(512);
 for(const stage of ['unknown','constructor','toString','__proto__'])
  assert.throws(()=>d.wifi(raw,{stage}),RangeError);
 assert.throws(()=>d.wifi(raw,{training_hint:1.5}),RangeError);
 assert.throws(()=>d.wifi(raw,{training_hint:2**32}),RangeError);
 assert.throws(()=>d.wifi(raw,{stage:['full']}),TypeError);
 for(const options of [{channel:-1},{channel:40},{sample_rate:5000000},{phy:2},{phy:1,sample_rate:4000000}])
  assert.throws(()=>d.ble(raw,options),RangeError);
 for(const options of [{training:-1},{trainingWords:2**32},{cfo:NaN}])
  assert.throws(()=>d.dsssSymbols(raw,options),RangeError);
 assert.strictEqual(d.wifi(raw),null);
 // Result byte arrays must survive later writes and workspace destruction.
 const m=d.module,r=d.result;m.HEAPU32.fill(0,r/4,r/4+26);
 m.HEAPU32[r/4]=1;m.HEAPU32[r/4+13]=1;m.HEAPU8[r+64]=42;
 const result=d.readResult(true);m.HEAPU8[r+64]=99;
 d.destroy();d.destroy();assert.strictEqual(result.prefix[0],42);
 for(const fn of [()=>d.wifi(raw),()=>d.ble(raw),()=>d.dsssSymbols(raw),()=>d.wifiOfdm(new Float32Array(192))])
  assert.throws(fn,/destroyed/);
})().catch(e=>{console.error(e);process.exitCode=1});
"""
        subprocess.run(['node', '-e', script, directory], check=True)

if __name__ == '__main__': unittest.main()

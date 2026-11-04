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
sys.path.insert(0, str(ROOT/'python'))
from espargos_packet_decoders import wifi, ble
from test_ble_qualify import waveform

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
                for field in ['prefix', 'transmitter']: expected[field] = list(expected[field])
            # Different libm implementations may round CFO by 1 Hz.
            if expected and actual:
                self.assertLessEqual(abs(actual['cfo_hz'] - expected['cfo_hz']), 1, test['name'])
                actual['cfo_hz'] = expected['cfo_hz']
            self.assertEqual(actual, expected, test['name'])
        self.assertIsNone(results[-1]); self.assertIsNone(results[-2])

    def test_binding_bounds(self):
        for iq in ([], [0]*16385, b'bad'):
            with self.assertRaises(ValueError): wifi(iq)
        self.assertIsNone(wifi([0]*512, stage='native_ofdm', training_hint=-1000))
        self.assertIsNone(ble([0]*512, channel=40))

if __name__ == '__main__': unittest.main()

# SPDX-License-Identifier: LGPL-3.0-or-later
"""Soft constellation accuracy and native/WASM parity, including truncated captures."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from scipy.signal import resample_poly
from espargos_packet_decoders import dsss_symbols, wifi
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import wifi_ppdu as phy


def packed(z):
    z = np.asarray(z, dtype=complex)
    z = z / max(abs(z)) * 200 * np.exp(2j*np.pi*40000*np.arange(len(z))/20e6)
    return ((np.rint(z.real).astype('i4') & 1023) |
            ((np.rint(-z.imag).astype('i4') & 1023) << 10)).astype('u4')


class SymbolsTests(unittest.TestCase):
    def test_modulations_and_parity(self):
        cases = []
        for short in (False, True):
            for rate in (1, 2, 5.5, 11):
                if short and rate == 1: continue
                wave = resample_poly(phy.synthesize_dsss(phy.make_probe_request(), rate, short_preamble=short), 20, 11)
                # Known echo; constellation equalization must not hide errors
                # by replacing samples with hard-decision ideal points.
                wave = wave + .12j*np.r_[np.zeros(2), wave[:-2]]
                iq = packed(np.r_[np.zeros(83), wave, np.zeros(100)])
                packet = wifi(iq)
                self.assertIsNotNone(packet)
                options = dict(field='payload')
                payload = dsss_symbols(iq, **options)
                points = np.array(payload['points'])
                self.assertEqual(payload['rate'], rate)
                self.assertGreater(len(points), 20)
                ideal = np.array([1,-1] if rate == 1 else [1,1j,-1,-1j])
                evm = np.sqrt(np.mean(np.min(abs(points[:,None]-ideal)**2, axis=1)))
                self.assertLess(evm, .22)
                self.assertGreater(evm, .001)
                cases.append(dict(iq=iq.tolist(), options=options))
                start = packet['training_start']; count = packet['training_words']
                training = iq[start-20:start+count+20]
                options = dict(field='barker', training=20, trainingWords=count, cfo=packet['cfo_hz'])
                barker = dsss_symbols(training, field='barker', training=20, training_words=count, cfo=packet['cfo_hz'])
                bp = np.array(barker['points']); self.assertGreater(len(bp), 10)
                self.assertLess(np.sqrt(np.mean(bp.imag**2)), .05)
                self.assertFalse(dsss_symbols(training)['points'])
                cases.append(dict(iq=training.tolist(), options=options))
        wasm = Path(os.environ['EPD_WASM_DIR']).resolve()
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'symbols.json'; source.write_text(json.dumps(cases))
            script = "const D=require(process.argv[1]+'/packet_decoders.js');D.create().then(d=>{const c=require(process.argv[2]);console.log(JSON.stringify(c.map(x=>d.dsssSymbols(Uint32Array.from(x.iq),x.options))));d.destroy()})"
            results = json.loads(subprocess.check_output(['node','-e',script,str(wasm),str(source)]))
        for case, actual in zip(cases,results,strict=True):
            options = dict(case['options'])
            if 'trainingWords' in options: options['training_words'] = options.pop('trainingWords')
            expected = dsss_symbols(case['iq'], **options)
            self.assertEqual(actual['rate'],expected['rate'])
            np.testing.assert_allclose([complex(p['re'],p['im']) for p in actual['points']],expected['points'],atol=.004)

    def test_recorded_cck_chips(self):
        with np.load(ROOT/'tests/fixtures/wifi_cck_iq.npz') as captures:
            for name, iq in captures.items():
                result = dsss_symbols(iq)
                points = np.array(result['points'])
                self.assertGreater(len(points), 100, name)
                self.assertIn(result['rate'], (5.5,11))
                error = np.sqrt(np.mean(np.min(abs(points[:,None]-np.array([1,1j,-1,-1j]))**2,axis=1)))
                self.assertLess(error, .5, name)

    def test_bounds_and_noise(self):
        noise = np.random.default_rng(91).integers(0,1<<20,2048,dtype=np.uint32)
        self.assertFalse(dsss_symbols(noise)['points'])
        for hint, length in [(2048,200),(0,4096),(2**32-1,200),(20,100)]:
            self.assertFalse(dsss_symbols(noise,field='barker',training=hint,training_words=length)['points'])

if __name__ == '__main__': unittest.main()

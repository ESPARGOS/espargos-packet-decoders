#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-3.0-or-later
"""Compare full-packet CRC recovery on fixed noise/multipath cases.

Example: git show HEAD:src/wifi.c > /tmp/wifi-baseline.c
         python tools/benchmark_wifi_noise.py /tmp/wifi-baseline.c
Requires NumPy/SciPy, a C compiler, and the repository test helpers.
"""
import argparse
import ctypes as C
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tests'))
from test_wifi_qualify import Packet, colored_ofdm_iq


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('--candidate', type=Path, default=ROOT/'src/wifi.c')
    parser.add_argument('--trials', type=int, default=20)
    args = parser.parse_args()
    if args.trials < 1: parser.error('--trials must be positive')
    with tempfile.TemporaryDirectory() as directory:
        decoders = []
        for name, source in [('baseline',args.baseline), ('candidate',args.candidate)]:
            library = Path(directory)/(name+'.so')
            subprocess.run(['cc','-shared','-fPIC','-O2','-I'+str(ROOT/'include'),
                            '-I'+str(ROOT/'src'),str(source),'-lm','-o',str(library)],check=True)
            lib = C.CDLL(str(library))
            lib.epd_wifi_qualify_workspace_size.restype = C.c_size_t
            scratch = C.create_string_buffer(lib.epd_wifi_qualify_workspace_size())
            fn = lib.epd_wifi_qualify
            fn.argtypes = [C.POINTER(C.c_uint32),C.c_uint32,C.c_uint32,C.c_uint32,C.c_void_p,C.POINTER(Packet)]
            fn.restype = C.c_bool
            decoders.append((lib,fn,scratch))
        report = {}
        for interference in [False,True]:
            row = {'cases':0,'baseline':0,'candidate':0,'recovered':0,'lost':0,'milliseconds':[0,0]}
            for rate in [6,12,24,36,48,54]:
                for snr in [8,14,20,26]:
                    for seed in range(args.trials):
                        iq = colored_ofdm_iq(rate,snr,seed,interference)
                        success = []
                        for n,(_,fn,scratch) in enumerate(decoders):
                            packet = Packet()
                            start = time.perf_counter()
                            valid = fn(iq.ctypes.data_as(C.POINTER(C.c_uint32)),len(iq),0,len(iq),scratch,C.byref(packet))
                            row['milliseconds'][n] += (time.perf_counter()-start)*1000
                            success.append(bool(valid and packet.crc_valid))
                            if success[-1]:
                                assert bytes(packet.transmitter) == bytes.fromhex('020000000001')
                                assert packet.rate_half_mbps == rate*2
                        row['cases'] += 1
                        row['baseline'] += success[0]; row['candidate'] += success[1]
                        row['recovered'] += success[1] and not success[0]
                        row['lost'] += success[0] and not success[1]
            report['colored' if interference else 'white'] = row
        print(json.dumps(report,indent=2))
        return any(row['lost'] for row in report.values())

if __name__ == '__main__':
    sys.exit(main())

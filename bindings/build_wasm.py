#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-3.0-or-later
"""Build browser/Node artifacts with Emscripten; output directory is the argument."""
from pathlib import Path
import json
import shutil
import subprocess
import sys
root = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1] if len(sys.argv) > 1 else root / 'dist').resolve()
out.mkdir(parents=True, exist_ok=True)
subprocess.run(['emcc', '-O3', '-std=c99', '-I'+str(root/'include'),
    str(root/'src/wifi.c'), str(root/'src/ble.c'), str(root/'bindings/bridge.c'),
    '-sMODULARIZE=1', '-sEXPORT_NAME=createEPDModule', '-sENVIRONMENT=web,node',
    '-sALLOW_MEMORY_GROWTH=1', '-sFILESYSTEM=0',
    '-sEXPORTED_RUNTIME_METHODS=["HEAPU32","HEAPU8"]',
    '-sEXPORTED_FUNCTIONS=["_malloc","_free","_epd_workspace_size","_epd_decode_wifi","_epd_decode_ble","_epd_wifi_dsss_symbols"]',
    '-o', str(out/'epd.js')], check=True)
shutil.copyfile(root/'bindings/packet_decoders.js', out/'packet_decoders.js')

for name in ('COPYING', 'COPYING.LESSER'):
    shutil.copyfile(root/name, out/name)
try:
    revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
except (OSError, subprocess.CalledProcessError):
    revision = None
(out/'source.json').write_text(json.dumps({'revision':revision, 'license':'LGPL-3.0-or-later'}, indent=2)+'\n')

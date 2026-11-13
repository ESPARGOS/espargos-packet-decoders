import sys
from pathlib import Path

from setuptools import Extension, find_packages, setup

ROOT = Path(__file__).parent
REPOSITORY = 'https://github.com/ESPARGOS/espargos-packet-decoders'

setup(
    name='espargos-packet-decoders',
    version='0.1.0',
    description='Portable WiFi and Bluetooth LE IQ decoders for C, WebAssembly and Python',
    long_description=(ROOT / 'README.rst').read_text(encoding='utf-8'),
    long_description_content_type='text/x-rst',
    url=REPOSITORY,
    project_urls={'Source': REPOSITORY, 'Issues': REPOSITORY + '/issues'},
    license='LGPL-3.0-or-later',
    license_files=['COPYING', 'COPYING.LESSER'],
    python_requires='>=3.9',
    extras_require={'test': ['numpy', 'scipy']},
    packages=find_packages('python'),
    package_dir={'': 'python'},
    ext_modules=[Extension(
        'espargos_packet_decoders._native',
        sources=['bindings/python.c', 'bindings/bridge.c', 'src/wifi.c', 'src/ble.c'],
        include_dirs=['include', 'bindings'],
        libraries=[] if sys.platform == 'win32' else ['m'],
        extra_compile_args=['/std:c11'] if sys.platform == 'win32' else ['-std=c99'],
    )],
)

from setuptools import Extension, setup, find_packages
setup(name='espargos-packet-decoders', version='0.1.0',
      description='Portable C WiFi and BLE IQ packet decoders',
      license='LGPL-3.0-or-later', python_requires='>=3.9',
      packages=find_packages('python'), package_dir={'': 'python'},
      ext_modules=[Extension('espargos_packet_decoders._native',
          sources=['bindings/python.c', 'bindings/bridge.c', 'src/wifi.c', 'src/ble.c'],
          include_dirs=['include', 'bindings'], libraries=['m'], extra_compile_args=['-std=c99'])])

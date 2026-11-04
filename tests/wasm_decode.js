// SPDX-License-Identifier: LGPL-3.0-or-later
const fs = require('fs'), path = require('path');
(async () => {
    const root = path.resolve(process.argv[2]);
    const Decoder = require(path.join(root, 'packet_decoders.js'));
    const decoder = await Decoder.create({wasmBinary:fs.readFileSync(path.join(root,'epd.wasm'))});
    const cases = JSON.parse(fs.readFileSync(process.argv[3], 'utf8'));
    const results = cases.map(test => {
        const iq = Uint32Array.from(test.iq);
        const r = decoder[test.kind](iq, test.options);
        if (r) for (const field of ['prefix', 'transmitter']) if (r[field]) r[field] = Array.from(r[field]);
        return r;
    });
    decoder.destroy();
    process.stdout.write(JSON.stringify(results));
})().catch(e => {console.error(e); process.exitCode = 1;});

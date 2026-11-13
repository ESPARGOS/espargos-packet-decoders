// SPDX-License-Identifier: LGPL-3.0-or-later
/* Reusable synchronous decoder after asynchronous WASM loading. IQ input is
 * Uint32Array in ESPARGOS tap convention; results use the same fields as Python. */
class PacketDecoders {
    static async create(options = {}) {
        const factory = typeof createEPDModule === 'function' ? createEPDModule : require('./epd.js');
        return new PacketDecoders(await factory(options));
    }
    constructor(module) {
        this.module = module;
        this.iq = module._malloc(16384 * 4);
        this.work = module._malloc(module._epd_workspace_size());
        this.result = module._malloc(104);
        this.symbols = module._malloc(1024 * 8 + 4);
        if (!this.iq || !this.work || !this.result || !this.symbols) {
            this.destroy();
            throw Error('Cannot allocate decoder workspace');
        }
    }
    destroy() {
        for (const key of ['iq', 'work', 'result', 'symbols', 'ofdm']) {
            if (this[key]) this.module._free(this[key]);
            this[key] = 0;
        }
    }
    decode(iq, kind, a, b, c = 0) {
        if (!this.work) throw Error('Decoder has been destroyed');
        if (!(iq instanceof Uint32Array) || !iq.length || iq.length > 16384)
            throw RangeError('Expected 1..16384 packed uint32 IQ samples');
        const m = this.module;
        m.HEAPU32.set(iq, this.iq / 4);
        const valid = kind === 0 ? m._epd_decode_wifi(this.iq, iq.length, a, b, this.work, this.result)
            : m._epd_decode_ble(this.iq, iq.length, a, b, c, this.work, this.result);
        return this.readResult(valid);
    }
    readResult(valid) {
        if (!valid) return null;
        const m = this.module;
        const r = m.HEAPU32.subarray(this.result / 4, this.result / 4 + 26);
        if (r[0] !== 1) throw Error('Unsupported decoder ABI');
        const bytes = m.HEAPU8;
        if (r[1] >= 100) return {protocol:'ble', phy:r[1]-100, identity_valid:!!(r[12]&1),
            crc_checked:!!(r[12]&2), crc_valid:!!(r[12]&4),
            prefix:bytes.slice(this.result+64,this.result+64+r[13]),
            access_address:r[14], packet_crc:r[15]};
        return {protocol:'wifi', phy:r[1], rate_half_mbps:r[2], mcs:r[3], stbc:r[4], frame_start:r[5],
            training_start:r[6], training_words:r[7], length_bytes:r[8], duration_us:r[9],
            cfo_hz:r[10]|0, quality:r[11], mac_valid:!!(r[12]&1), crc_checked:!!(r[12]&2),
            crc_valid:!!(r[12]&4), prefix:bytes.slice(this.result+64,this.result+64+r[13]),
            transmitter:bytes.slice(this.result+96,this.result+102)};
    }
    static integer(value, name, low, high) {
        if (typeof value !== 'number') throw TypeError(name + ' must be an integer');
        if (!Number.isInteger(value) || value < low || value > high)
            throw RangeError(`${name} must be an integer in ${low}..${high}`);
        return value;
    }
    wifi(iq, {stage = 'full', training_hint = 0} = {}) {
        const stages = {full:0, training:1, extended_header:2, native_ofdm:3, native_dsss:4};
        if (typeof stage !== 'string') throw TypeError('stage must be a string');
        if (!Object.prototype.hasOwnProperty.call(stages, stage)) throw RangeError('Unknown WiFi decoder stage');
        PacketDecoders.integer(training_hint, 'training_hint', -2147483648, 2147483647);
        return this.decode(iq, 0, stages[stage], training_hint);
    }
    wifiOfdm(symbols) {
        if (!this.work) throw Error('Decoder has been destroyed');
        if (!(symbols instanceof Float32Array) || !symbols.length || symbols.length%192 || symbols.length>200*192)
            throw RangeError('Expected 1..200 OFDM symbols of 64 (I,Q,inverse-variance) triples');
        const m = this.module;
        if (!(this.ofdmCapacity >= symbols.byteLength)) {
            const next = m._malloc(symbols.byteLength);
            if (!next) throw Error('Cannot allocate OFDM symbol input');
            if (this.ofdm) m._free(this.ofdm);
            this.ofdm = next;
            this.ofdmCapacity = symbols.byteLength;
        }
        new Float32Array(m.HEAPU8.buffer,this.ofdm,symbols.length).set(symbols);
        return this.readResult(m._epd_decode_wifi_ofdm(this.ofdm,symbols.length/192,this.work,this.result));
    }
    ble(iq, {sample_rate = 4000000, phy = 0, channel = 37} = {}) {
        PacketDecoders.integer(sample_rate, 'sample_rate', 4000000, 16000000);
        PacketDecoders.integer(phy, 'phy', 0, 1);
        PacketDecoders.integer(channel, 'channel', 0, 39);
        if (![4000000, 8000000, 16000000].includes(sample_rate) || (phy === 1 && sample_rate < 8000000))
            throw RangeError('Unsupported BLE sample rate for this PHY');
        return this.decode(iq, 1, sample_rate, phy, channel);
    }
    dsssSymbols(iq, {field = 'payload', training = 0, trainingWords = 0, cfo = 0} = {}) {
        if (!this.work) throw Error('Decoder has been destroyed');
        if (!(iq instanceof Uint32Array) || !iq.length || iq.length > 16384)
            throw RangeError('Expected 1..16384 packed uint32 IQ samples');
        if (!['payload', 'barker'].includes(field)) throw RangeError('Unknown DSSS field');
        PacketDecoders.integer(training, 'training', 0, 4294967295);
        PacketDecoders.integer(trainingWords, 'trainingWords', 0, 4294967295);
        if (typeof cfo !== 'number') throw TypeError('cfo must be a number in Hz');
        if (!Number.isFinite(cfo) || cfo < -2147483648 || cfo > 2147483647)
            throw RangeError('cfo must fit a signed 32-bit integer in Hz');
        const m = this.module;
        m.HEAPU32.set(iq, this.iq / 4);
        const count = m._epd_wifi_dsss_symbols(this.iq, iq.length, field === 'barker' ? 1 : 0,
            training, trainingWords, Math.round(cfo), this.work, this.symbols, 1024, this.symbols + 8192);
        const values = new Float32Array(m.HEAPU8.buffer, this.symbols, count * 2);
        return {points : Array.from({length : count}, (_, n) => ({re : values[n*2], im : values[n*2+1]})),
            rate : count ? m.HEAPU32[(this.symbols + 8192) / 4] / 2 : 0};
    }
}
if (typeof module !== 'undefined') module.exports = PacketDecoders;

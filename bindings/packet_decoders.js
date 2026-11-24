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
    // Each receiver supplies CFO/gain-corrected FFTs from its own sample grid.
    spatial(receivers, {phy, paths, ltf, gi, alamouti = false, maxSymbols = 8}) {
        const m = this.module;
        if (!this.work) throw Error('Decoder has been destroyed');
        if (!Number.isInteger(maxSymbols) || maxSymbols < 0 || maxSymbols > 8)
            throw RangeError('Expected 0..8 payload symbols');
        if (![phy, paths, ltf, gi].every(x => Number.isInteger(x) && x > 0 && x <= 64) || m._epd_spatial_layout(phy, paths, ltf, gi, this.result))
            throw RangeError('Unsupported spatial training configuration');
        const plan = m.HEAPU32.slice(this.result / 4, this.result / 4 + 9);
        const [, , count, size, first, stride, fft, data, period] = plan;
        const rows = [];
        let nsym = 0;
        for (const receiver of receivers) {
            if (!(receiver.noise > 0) || !Number.isFinite(receiver.noise)) continue;
            const training = Array.from({length: count}, (_, i) => receiver.fft(first + stride*i, size));
            if (training.some(x => !x || x.length !== size)) continue;
            const gain = receiver.gain?.();
            const payload = [];
            for (let i = 0; i < maxSymbols; i++) {
                const y = receiver.fft(data + period*i, fft);
                payload.push(y?.length === fft ? y : null);
                if (y?.length === fft) nsym = Math.max(nsym, i + 1);
            }
            rows.push({training, gain, payload, ant: receiver.ant, noise: receiver.noise});
        }
        const streams = alamouti ? 1 : paths;
        const channels = [], points = [], streamPoints = Array.from({length: streams}, () => []);
        const result = {channels, points, streamPoints, antennas: rows.length, symbols: 0, rankFailures: 0, pilotFailures: 0};
        if (!rows.length) return result;
        const length = count*size + nsym*fft, nr = rows.length;
        const inputBytes = 16*nr*length, channelBytes = 16*nr*fft*paths, dataBytes = 16*nsym*fft*streams;
        const bytes = nr*8 + inputBytes + channelBytes + dataBytes + 8;
        if (!Number.isSafeInteger(bytes) || bytes >= 0x80000000) throw RangeError('Spatial packet is too large');
        if (bytes > (this.spatialCapacity || 0)) {
            const ptr = m._malloc(bytes);
            if (!ptr) throw Error('Cannot allocate spatial decoder workspace');
            m._free(this.spatialWork || 0);
            this.spatialWork = ptr;
            this.spatialCapacity = bytes;
        }
        const noise = this.spatialWork, input = noise+nr*8, h = input+inputBytes;
        const soft = h+channelBytes, stats = soft+dataBytes;
        let memory = new Float64Array(m.HEAPU8.buffer);
        for (const [r,row] of rows.entries()) {
            memory[noise/8+r] = row.noise;
            let i = input/8 + 2*r*length;
            for (const spectrum of [...row.training,...row.payload.slice(0,nsym)]) {
                if (!spectrum) { memory[i] = NaN; i += 2*fft; continue; }
                for (const z of spectrum) { memory[i++] = z.re; memory[i++] = z.im; }
            }
        }
        m.HEAPU32.set(plan, this.result / 4);
        if (m._epd_spatial_decode(this.result,nr,nsym,+alamouti,noise,input,h,soft,stats))
            throw Error('Spatial decoder failed');
        memory = new Float64Array(m.HEAPU8.buffer);
        const read = i => ({re: memory[i], im: memory[i+1]});
        const edge = phy === 4 ? 122 : 28, spacing = 20e6/fft;
        const pilots = phy === 4 ? [-116,-90,-48,-22,22,48,90,116] : [-21,-7,7,21];
        for (const [r,row] of rows.entries()) {
            const tones = [];
            for (let k = -edge; k <= edge; k++) {
                const i = h/8 + 2*(r*fft+(k+fft)%fft)*paths;
                if (Number.isFinite(memory[i])) tones.push({hz:k*spacing, streams:Array.from({length:paths}, (_,s)=>read(i+2*s))});
            }
            channels.push({antenna:row.ant, gain:row.gain, tones});
        }
        for (let j = 0; j < nsym; j++) {
            const begin = points.length;
            for (let k = -edge; k <= edge; k++) {
                if (pilots.includes(k)) continue;
                const i = soft/8 + 2*(j*fft+(k+fft)%fft)*streams;
                if (!Number.isFinite(memory[i])) continue;
                for (let s = 0; s < streams; s++) {
                    const point = {...read(i+2*s),stream:s,hz:k*spacing};
                    points.push(point);
                    streamPoints[s].push(point);
                }
            }
            if (points.length > begin) result.symbols++;
        }
        [result.rankFailures,result.pilotFailures] = m.HEAPU32.subarray(stats/4,stats/4+2);
        return result;
    }
    destroy() {
        for (const key of ['iq', 'work', 'result', 'symbols', 'ofdm', 'spatialWork']) {
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
        const packet = {protocol:'wifi', phy:r[1], rate_half_mbps:r[2], mcs:r[3], stbc:r[4], frame_start:r[5],
            training_start:r[6], training_words:r[7], length_bytes:r[8], duration_us:r[9],
            cfo_hz:r[10]|0, quality:r[11], mac_valid:!!(r[12]&1), crc_checked:!!(r[12]&2),
            crc_valid:!!(r[12]&4), prefix:bytes.slice(this.result+64,this.result+64+r[13]),
            transmitter:bytes.slice(this.result+96,this.result+102)};
        if (packet.phy === 4 && packet.prefix.length === 7) {
            const bit = (start, n=1) => {let v=0;for(let i=0;i<n;i++)v|=((packet.prefix[(start+i)>>3]>>((start+i)&7))&1)<<i;return v;};
            const nsts=bit(23,bit(41)?2:3)+1, cp=bit(21,2), special=cp===3&&bit(7)&&bit(35);
            packet.he={format:'SU',bandwidth_mhz:20*(1<<bit(19,2)),space_time_streams:nsts,
                spatial_streams:bit(35)&&!special?nsts/2:nsts,ltf_size:[1,2,2,4][cp],
                guard_interval_ns:special?800:[800,800,1600,3200][cp],
                bss_color:bit(8,6),dcm:!special&&!!bit(7),doppler:!!bit(41),ldpc:!!bit(33)};
        }
        return packet;
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

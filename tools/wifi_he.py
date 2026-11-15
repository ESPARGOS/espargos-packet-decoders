# SPDX-License-Identifier: LGPL-3.0-or-later
"""HE SU20 PHY test stimuli: valid signaling/training and known soft symbols.

Payload symbols exercise demodulation, not MAC/FEC decoding. They are deliberately
uncoded and do not claim a valid PSDU/FCS. HE-LTF sequences follow IEEE TGax
11-15/0132r17, PHY motions 80/109. FFTs and channel application use NumPy.
"""
import numpy as np
import wifi_ppdu as legacy

TRAINING = {4: [-1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 0, 0, 0, -1, 1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1], 2: [-1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 0, 1, -1, 1, 1, -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1], 1: [-1, 1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 0, -1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1]}
PILOTS = [-116, -90, -48, -22, 22, 48, 90, 116]
PILOT_BASE = [1, 1, 1, -1, -1, 1, 1, 1]
P = {1:[[1]],2:[[1,-1],[1,1]],4:[[1,-1,1,1],[1,1,-1,1],[1,1,1,-1],[-1,1,1,1]]}

P[8]=np.kron([[1,1],[1,-1]],np.array(P[4]))
P[6]=np.exp(-2j*np.pi*np.outer(np.arange(6),np.arange(6))/6)*[1,-1,1,1,1,-1]

def header_bits(streams=1, mcs=0, cp=3, bandwidth=0, bad_crc=False, stbc=False, dcm=False):
    bits=np.zeros(52,dtype='u1')
    def put(start,width,value): bits[start:start+width]=[(value>>i)&1 for i in range(width)]
    put(0,1,1);put(3,4,mcs);put(8,6,17);put(14,1,1)
    put(19,2,bandwidth);put(21,2,3 if cp==4 else cp);put(23,3,streams-1)
    put(7,1,dcm or cp==4);put(35,1,stbc or cp==4)
    put(26,7,127);put(34,1,1);put(40,1,1)
    # Serial shift-register form, independent of the decoder's integer CRC.
    state=[1]*8
    for bit in bits[:42]:
        feedback=state[7]^int(bit)
        state=[feedback,state[0]^feedback,state[1]^feedback,*state[2:7]]
    bits[42:46]=[1^state[i] for i in [7,6,5,4]]
    if bad_crc:bits[42]^=1
    return bits

def preamble(bits):
    sig=legacy._signal_bits(13,100) # mod 3 = 1 identifies HE SU/TB
    coded=legacy.interleave(legacy.convolutional_encode(sig),48,1)
    fields=[legacy.legacy_preamble()]
    for s in range(2):
        f=np.zeros(64,complex)
        f[np.array(legacy.DATA_SUBCARRIERS)%64]=legacy.map_constellation(coded,1)
        f[np.array(legacy.PILOT_SUBCARRIERS)%64]=[1,1,1,-1]
        f[np.array([-28,-27,27,28])%64]=[-1,-1,-1,1]
        x=np.fft.ifft(f);fields.append(np.r_[x[-16:],x])
    coded=legacy.ht_interleave(legacy.convolutional_encode(bits),52,1)
    for s in range(2):
        f=np.zeros(64,complex)
        f[np.array(legacy.HT_DATA_SUBCARRIERS)%64]=legacy.map_constellation(coded[52*s:52*(s+1)],1)
        f[np.array(legacy.PILOT_SUBCARRIERS)%64]=[1,1,1,-1]
        x=np.fft.ifft(f);fields.append(np.r_[x[-16:],x])
    return np.concatenate(fields)

def stimulus(streams=2, cp=3, symbols=4, mcs=3, seed=4):
    """Return TX streams [sample, stream], ideal symbols [symbol,tone,stream]."""
    if streams not in range(1,9):raise ValueError('1..8 streams')
    ltf=[1,2,2,4,4][cp];gi=[16,16,32,64,16][cp];n=64*ltf;step=4//ltf
    count=1 if streams==1 else 2 if streams==2 else 4 if streams<=4 else 6 if streams<=6 else 8
    seq=TRAINING[ltf];tones=np.arange(-120 if ltf==1 else -122,123,step)
    matrix=np.array(P[count]);bits=header_bits(streams,mcs,cp)
    pre=preamble(bits)
    # HE SU STF: 16-sample periodicity, 4 us duration.
    stf=np.zeros(256,complex)
    stf[np.arange(-112,113,16)%256]=np.array([-1,-1,-1,1,1,1,-1,1,1,1,-1,1,1,-1,1])*(1+1j)*np.sqrt(242/30)
    stf=np.tile(np.fft.ifft(stf)[:16],5)
    fields=[np.tile(np.r_[pre,stf][:,None],(1,streams))]
    for s in range(count):
        f=np.zeros((n,streams),complex)
        for k,sign in zip(tones,seq):
            if k in PILOTS:f[(k//step)%n,:]=sign*matrix[0,s]
            else:f[(k//step)%n,:]=sign*matrix[:streams,s]
        x=np.fft.ifft(f,axis=0)/np.sqrt(4/ltf)
        fields.append(np.vstack([x[-gi:],x]))
    data=[k for k in range(-122,123) if abs(k)>1 and k not in PILOTS]
    rng=np.random.default_rng(seed);bpsc=[1,2,2,4,4,6,6,6,8,8,10,10][mcs]
    expected=np.empty((symbols,len(data),streams),complex)
    for s in range(symbols):
        f=np.zeros((256,streams),complex)
        for stream in range(streams):
            values=legacy.map_constellation(rng.integers(0,2,len(data)*bpsc,dtype='u1'),bpsc)
            expected[s,:,stream]=values;f[np.array(data)%256,stream]=values
        for j,k in enumerate(PILOTS):f[k%256,:]=PILOT_BASE[(s+j)%8]*legacy.POLARITY[(s+1)%127]
        x=np.fft.ifft(f,axis=0);fields.append(np.vstack([x[-gi:],x]))
    return np.vstack(fields),expected,bits

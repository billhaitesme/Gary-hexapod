import struct, os

FE = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin"
MP3DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "george_mp3")
fe = open(FE, "rb").read()
VBASE = 0x160000

def parse_mp3(b, label):
    print(f"--- {label} ({len(b)}B) ---")
    p = 0
    if b[:3] == b"ID3":
        tsz = (b[6]<<21)|(b[7]<<14)|(b[8]<<7)|b[9]
        print(f"  ID3v2.{b[3]}.{b[4]} tag, size {tsz}; tag body: {b[10:10+tsz].hex()}")
        p = 10 + tsz
    frames = 0
    first = True
    while p < len(b) - 4 and frames < 4:
        if b[p] == 0xFF and (b[p+1] & 0xE0) == 0xE0:
            h = b[p:p+4]
            ver_b = (h[1] >> 3) & 3
            layer_b = (h[1] >> 1) & 3
            br_i = (h[2] >> 4) & 0xF
            sr_i = (h[2] >> 2) & 3
            pad = (h[2] >> 1) & 1
            ver = {0: "2.5", 2: "2", 3: "1"}.get(ver_b, "?")
            layer = {1: 3, 2: 2, 3: 1}.get(layer_b, 0)
            BRv2 = [0,8,16,24,32,40,48,56,64,80,96,112,128,144,160,0]
            SRv2 = [22050,24000,16000,0]; SRv25 = [11025,12000,8000,0]
            sr = SRv2[sr_i] if ver == "2" else (SRv25[sr_i] if ver == "2.5" else 0)
            br = BRv2[br_i]
            flen = (144000 * br // sr + pad) if sr and br else 0
            body = b[p+4:p+40]
            xing = b"Xing" in b[p:p+flen] or b"Info" in b[p:p+flen] or b"LAME" in b[p:p+flen]
            print(f"  frame@{p}: hdr={h.hex()} MPEG{ver} L{layer} {br}kbps {sr}Hz pad={pad} len={flen} "
                  f"{'<< XING/INFO/LAME TAG FRAME' if xing and first else ''}")
            p += flen if flen else 1
            frames += 1
            first = False
        else:
            p += 1
    print()

# factory clip id 1 and id 36 (small)
n = struct.unpack_from("<H", fe, VBASE)[0]
for want in (1, 36):
    for i in range(n):
        cid, off, sz = struct.unpack_from("<HII", fe, VBASE+2+10*i)
        if cid == want:
            parse_mp3(fe[VBASE+off:VBASE+off+sz], f"FACTORY id {cid}")
            break

# our lame output
for f in ("[10]YESSIR.mp3", "_silence.mp3"):
    parse_mp3(open(os.path.join(MP3DIR, f), "rb").read(), f"OURS {f}")

# April voice.bin clip 1 (the format the SDK pipeline produced -- also useful)
VB = r"C:\Users\BillH\Documents\Arduino\gary_v1\WonderEcjho Firmwares\CI130X_SDK_Offline_uart_1_12_16\projects\cwsl_A_sample\firmware\voice\voice.bin"
vb = open(VB, "rb").read()
n2 = struct.unpack_from("<H", vb, 0)[0]
cid, off, sz = struct.unpack_from("<HII", vb, 2)
parse_mp3(vb[off:off+sz], f"APRIL voice.bin clip id {cid}")

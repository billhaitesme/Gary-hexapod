import subprocess, os, struct, wave, json

LAME = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 Firmware Flash Tool\lame.exe"
SRC = r"C:\Users\BillH\Documents\Arduino\gary_v1\WonderEcjho Firmwares\CI130X_SDK_Offline_uart_1_12_16\projects\cwsl_A_sample\firmware\voice\src"
SCRATCH = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(SCRATCH, "george_ci")
os.makedirs(OUT, exist_ok=True)

sil = os.path.join(OUT, "_silence.wav")
with wave.open(sil, "wb") as w:
    w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
    w.writeframes(b"\x00\x00" * 2400)

def ci_wrap(mp3_path):
    b = open(mp3_path, "rb").read()
    # strip ID3v2 (standard: size excludes 10-byte header)
    if b[:3] == b"ID3":
        tsz = (b[6]<<21)|(b[7]<<14)|(b[8]<<7)|b[9]
        b = b[10+tsz:]
    # strip ID3v1 trailer
    if len(b) > 128 and b[-128:-125] == b"TAG":
        b = b[:-128]
    # find first sync
    p = 0
    while p < len(b)-1 and not (b[p] == 0xFF and (b[p+1] & 0xE0) == 0xE0):
        p += 1
    b = b[p:]
    # drop leading Xing/Info/LAME tag frame (one 108-byte frame)
    if any(m in b[:108] for m in (b"Xing", b"Info", b"LAME")):
        b = b[108:]
    # validate: every 108 bytes must start with sync, length divides evenly
    assert len(b) % 108 == 0, f"{mp3_path}: audio {len(b)} not multiple of 108"
    for q in range(0, len(b), 108):
        assert b[q] == 0xFF and (b[q+1] & 0xE0) == 0xE0, f"{mp3_path}: no sync at {q}"
        assert b[q:q+4] == b"\xff\xf3\x38\xc4", f"{mp3_path}: unexpected frame header at {q}: {b[q:q+4].hex()}"
    frames = len(b) // 108
    samples = frames * 576
    total = 30 + len(b)
    hdr = (b"ID3\x03\x00\x00" + bytes([0,0,0,30]) +
           b"PRIV" + b"\x00\x00\x00\x0A" + b"\x00\x00" +
           b"CI" + struct.pack("<II", total, samples))
    assert len(hdr) == 30
    return hdr + b

results = {}
jobs = [(f, os.path.join(SRC, f)) for f in os.listdir(SRC) if f.lower().endswith(".wav") and not f.startswith("[65535]")]
jobs.append(("_silence.wav", sil))
for name, path in jobs:
    tmp = os.path.join(OUT, "_tmp.mp3")
    # -t: no Xing/Info frame; no id3 options: lame still writes none without tag input? use --noreplaygain and strip anyway
    r = subprocess.run([LAME, "--silent", "-t", "--resample", "16000", "-b", "24", "-m", "m", path, tmp],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(f"FAIL {name}: {r.stderr[-200:]}"); continue
    wrapped = ci_wrap(tmp)
    outp = os.path.join(OUT, name.replace(".wav", ".bin"))
    open(outp, "wb").write(wrapped)
    results[name] = len(wrapped)
os.remove(os.path.join(OUT, "_tmp.mp3"))
for k in sorted(results):
    print(f"{results[k]:7d}B  {k}")
print(f"TOTAL: {sum(results.values())}")

# structural self-check against a real factory clip profile
FE = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin"
fe = open(FE, "rb").read()
fac = fe[0x160000+0x630 : 0x160000+0x630+1974]
ours = open(os.path.join(OUT, "[10]YESSIR.bin"), "rb").read()
print("\nfactory header:", fac[:30].hex())
print("ours    header:", ours[:30].hex())
print("factory frame0:", fac[30:34].hex(), " ours frame0:", ours[30:34].hex())
fs = struct.unpack_from("<II", fac, 22); os_ = struct.unpack_from("<II", ours, 22)
print(f"factory PRIV: size={fs[0]} (actual {len(fac)}) samples={fs[1]}")
print(f"ours    PRIV: size={os_[0]} (actual {len(ours)}) samples={os_[1]}")

import struct

FE = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin"
with open(FE, "rb") as f:
    fe = f.read()
N = len(fe)
print(f"size {N} = {hex(N)}")

# 1) find candidate voice index tables: u16 count n, then n * (u16 id, u32 off, u32 size),
#    with off[0] == 2 + 10*n (allow up to 16 bytes padding), offsets ascending & contiguous-ish
def check_index(pos):
    if pos + 2 > N: return None
    n = struct.unpack_from("<H", fe, pos)[0]
    if not (3 <= n <= 300): return None
    need = 2 + 10 * n
    if pos + need > N: return None
    recs = []
    prev_end = None
    for i in range(n):
        cid, off, sz = struct.unpack_from("<HII", fe, pos + 2 + 10 * i)
        if sz == 0 or sz > 0x40000 or off > 0x200000: return None
        recs.append((cid, off, sz))
    if not (need <= recs[0][1] <= need + 64): return None
    for i in range(1, n):
        if recs[i][1] <= recs[i-1][1]: return None
        gap = recs[i][1] - (recs[i-1][1] + recs[i-1][2])
        if gap < 0 or gap > 64: return None
    total = recs[-1][1] + recs[-1][2]
    return n, recs, total

hits = []
for pos in range(0, N - 12, 2):
    r = check_index(pos)
    if r:
        hits.append((pos, r[0], r[2], r[1][:3], r[1][-2:]))
        # skip past this table to avoid overlapping rescans
if not hits:
    print("no index tables found")
for pos, n, total, first3, last2 in hits[:20]:
    print(f"INDEX at {hex(pos)}: {n} clips, content span {hex(total)} bytes")
    print(f"   first: {[(cid, hex(o), hex(s)) for cid,o,s in first3]}")
    print(f"   last : {[(cid, hex(o), hex(s)) for cid,o,s in last2]}")

# 2) MP3 sync density map (64KB buckets) to see where audio lives
print("\nMP3-ish sync density per 64KB bucket (count of FF Ex/FF Fx pairs):")
for b in range(0, N, 0x10000):
    chunk = fe[b:b+0x10000]
    c = 0
    for i in range(len(chunk) - 1):
        if chunk[i] == 0xFF and (chunk[i+1] & 0xE0) == 0xE0:
            c += 1
    if c > 50:
        print(f"  {hex(b)}: {c}")

# 3) ASCII strings of interest
import re
for m in re.finditer(rb"[ -~]{8,}", fe):
    s = m.group().decode()
    if any(k in s.lower() for k in ("voice", "ver", "sfw", "ci13", "hiwonder", "wake", "study")):
        print(f"STR @{hex(m.start())}: {s[:80]}")

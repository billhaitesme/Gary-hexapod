import struct, os, json

SCRATCH = os.path.dirname(os.path.abspath(__file__))
FE = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin"
MP3DIR = os.path.join(SCRATCH, "george_ci")
OUTBIN = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\voice\wonderecho\GARY_K2SO_v5_factory_swap.bin"

VBASE, VMAX_END = 0x160000, 0x1DE000
SIZE_FIELD, CRC_FIELD = 0x20F2, 0x20F6

fe = bytearray(open(FE, "rb").read())
orig = bytes(fe)

# factory index
n = struct.unpack_from("<H", fe, VBASE)[0]
fidx = []
for i in range(n):
    cid, off, sz = struct.unpack_from("<HII", fe, VBASE + 2 + 10 * i)
    fidx.append((cid, off, sz))
fsizes = {c: (o, s) for c, o, s in fidx}
assert n == 157

def george(name):
    p = os.path.join(MP3DIR, name + ".bin")
    return open(p, "rb").read()

G = {k: george(k) for k in [
    "[300]GARYONLINE", "[301]GOINGTOSLEEP", "[255]HEY-GARY", "[1]MOVINGFORWARD",
    "[2]MOVINGBACK", "[3]TURNINGLEFT", "[4]TURNINGRIGHT", "[9]STOPPING",
    "[10]YESSIR", "[12]SITTINGDOWN", "[15]RAISINGUP", "[26]HELLOTHERE",
    "[27]IAMGARY", "[28]WATCHTHIS", "[29]MARCHING", "[135]ATTACKING",
    "[130]GOODBYE", "[132]THANKS", "_silence"]}

PRIMARY = {
    1: "[300]GARYONLINE", 2: "[301]GOINGTOSLEEP", 3: "[255]HEY-GARY",
    11: "[1]MOVINGFORWARD", 12: "[2]MOVINGBACK", 13: "[3]TURNINGLEFT",
    14: "[4]TURNINGRIGHT", 19: "[9]STOPPING", 20: "[10]YESSIR",
    21: "[12]SITTINGDOWN", 22: "[12]SITTINGDOWN", 26: "[12]SITTINGDOWN", 44: "[12]SITTINGDOWN",
    25: "[15]RAISINGUP", 74: "[15]RAISINGUP",
    36: "[26]HELLOTHERE", 37: "[27]IAMGARY", 38: "[28]WATCHTHIS",
    39: "[29]MARCHING", 73: "[29]MARCHING",
    43: "[135]ATTACKING", 76: "[135]ATTACKING", 77: "[135]ATTACKING",
    98: "[135]ATTACKING", 99: "[135]ATTACKING",
    93: "[130]GOODBYE", 94: "[301]GOINGTOSLEEP", 95: "[132]THANKS",
}
YESSIR_IDS = [15,16,17,18,23,24,27,34,35,40,41,42,45,46,47,48,49,50,51,52,53,
              61,62,63,64,65,66,67,68,69,70,71,72,75,78,79,91,92,96,97,
              100,101,102,103,104,105,106,107,108,109,110,
              111,112,113,114,115,116,117,118,119,120]
SILENCE_IDS = [28,29,30,31,32,33,54,55,56,57,58,59,60,150,151,152,153,154,155,156,157,158]

assign = {}
for cid, _, _ in fidx:
    if cid == 65535:
        assign[cid] = ("factory", cid)
    elif cid in PRIMARY:
        assign[cid] = ("george", PRIMARY[cid])
    elif cid in YESSIR_IDS:
        assign[cid] = ("george", "[10]YESSIR")
    elif cid in SILENCE_IDS:
        assign[cid] = ("george", "_silence")
    else:
        assign[cid] = ("factory", cid)

# build region: index header then unique blobs (dedupe), clip order = factory order
hdr_len = 2 + 10 * n
data_start = (hdr_len + 15) // 16 * 16
blob_off = {}
data = bytearray()
entries = []
for cid, foff, fsz in fidx:
    kind, key = assign[cid]
    blob = bytes(orig[VBASE+foff : VBASE+foff+fsz]) if kind == "factory" else G[key]
    bkey = (kind, key)
    if bkey not in blob_off:
        pos = data_start + len(data)
        blob_off[bkey] = (pos, len(blob))
        data += blob
        pad = (-len(data)) % 16
        data += b"\xFF" * pad
    entries.append((cid, *blob_off[bkey]))

region = bytearray()
region += struct.pack("<H", n)
for cid, off, sz in entries:
    region += struct.pack("<HII", cid, off, sz)
region += b"\xFF" * (data_start - len(region))
region += data
new_size = len(region)
budget = VMAX_END - VBASE
print(f"new voice region: {new_size} bytes ({hex(new_size)}), budget {budget} ({hex(budget)}), "
      f"free {budget - new_size}")
assert new_size <= budget, "OVER BUDGET"

def crc16_ccitt(b):
    c = 0
    for byte in b:
        c ^= byte << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c

# splice: region, then 0xFF fill to partition end
fe[VBASE:VBASE+new_size] = region
fe[VBASE+new_size:VMAX_END] = b"\xFF" * (VMAX_END - VBASE - new_size)
struct.pack_into("<I", fe, SIZE_FIELD, new_size)
struct.pack_into("<I", fe, CRC_FIELD, crc16_ccitt(bytes(fe[VBASE:VBASE+new_size])))

# ============ VERIFICATION ============
out = bytes(fe)
assert len(out) == len(orig), "file length changed!"
# 1) outside voice window + 2 manifest fields: byte-identical
diffs = []
for i in range(len(orig)):
    if orig[i] != out[i]:
        if not (VBASE <= i < VMAX_END or SIZE_FIELD <= i < SIZE_FIELD+4 or CRC_FIELD <= i < CRC_FIELD+4):
            diffs.append(i)
assert not diffs, f"unexpected diffs at {[hex(d) for d in diffs[:5]]}"
print("VERIFY 1: all bytes outside voice window + 2 manifest fields identical  OK")

# 2) re-validate all partition CRCs from the manifest of the NEW file
parts = [(0x20AA,), (0x20CC,), (0x20DD,), (0x20EE,), (0x20FF,), (0x210C,)]
ok = True
for (p,) in parts:
    addr, size = struct.unpack_from("<II", out, p)
    stored = struct.unpack_from("<H", out, p+8)[0]
    if addr >= len(out):
        # region beyond EOF (NV): source data not in the image -- require the
        # manifest entry to be byte-identical to the factory bin instead
        good = out[p:p+12] == orig[p:p+12]
        print(f"  part addr={hex(addr)} (beyond EOF): entry identical to factory  {'OK' if good else 'MISMATCH'}")
        ok &= good
        continue
    seg = out[addr:addr+size]
    calc = crc16_ccitt(seg)
    good = (calc == stored)
    ok &= good
    print(f"  part addr={hex(addr)} size={hex(size)} crc stored={stored:#06x} calc={calc:#06x} {'OK' if good else 'MISMATCH'}")
assert ok
print("VERIFY 2: every manifest partition CRC validates on the new file  OK")

# 3) index integrity of the new region
n2 = struct.unpack_from("<H", out, VBASE)[0]
prev_end = None
for i in range(n2):
    cid, off, sz = struct.unpack_from("<HII", out, VBASE + 2 + 10*i)
    assert off + sz <= new_size
assert n2 == 157
print("VERIFY 3: new index has 157 entries, all inside the region  OK")

# 4) spot-check: wake clip (id 3) bytes == our HEY-GARY mp3
for want_id, want_key in ((3, "[255]HEY-GARY"), (1, "[300]GARYONLINE"), (19, "[9]STOPPING"), (37, "[27]IAMGARY")):
    for i in range(n2):
        cid, off, sz = struct.unpack_from("<HII", out, VBASE + 2 + 10*i)
        if cid == want_id:
            assert out[VBASE+off:VBASE+off+sz] == G[want_key], f"id {want_id} mismatch"
            print(f"VERIFY 4: id {want_id} == {want_key} ({sz}B)  OK")
            break

os.makedirs(os.path.dirname(OUTBIN), exist_ok=True)
open(OUTBIN, "wb").write(out)
import hashlib
print(f"\nWROTE {OUTBIN}")
print(f"size {len(out)}  md5 {hashlib.md5(out).hexdigest()}")


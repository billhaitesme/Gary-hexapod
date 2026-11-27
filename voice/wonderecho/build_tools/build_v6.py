import struct, hashlib

EN = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin"
V5 = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\voice\wonderecho\GARY_K2SO_v5_factory_swap.bin"
V6 = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\voice\wonderecho\GARY_K2SO_v6_factory_swap.bin"

en = open(EN, "rb").read()
v5 = bytearray(open(V5, "rb").read())

# calibrate: find table length L such that sum(en[0x2000:0x2000+L]) & 0xFFFF == stored u16 at 0x2000+L
stored_candidates = []
for L in range(260, 300):
    s = sum(en[0x2000:0x2000+L]) & 0xFFFF
    stored = struct.unpack_from("<H", en, 0x2000+L)[0]
    if s == stored:
        stored_candidates.append((L, s))
print("factory calibration matches (len, checksum):", [(l, hex(c)) for l, c in stored_candidates])
assert len(stored_candidates) >= 1
L = stored_candidates[0][0]
CK_OFF = 0x2000 + L
print(f"table length {L} (0x{L:x}), checksum field at {hex(CK_OFF)}, factory stored {hex(struct.unpack_from('<H', en, CK_OFF)[0])}")

# sanity: v5's table currently MISMATCHES (the bug)
v5sum = sum(v5[0x2000:0x2000+L]) & 0xFFFF
v5stored = struct.unpack_from("<H", v5, CK_OFF)[0]
print(f"v5: computed {hex(v5sum)} vs stored {hex(v5stored)}  -> {'BUG CONFIRMED (mismatch)' if v5sum != v5stored else 'already matches?!'}")

# patch
struct.pack_into("<H", v5, CK_OFF, v5sum)
out = bytes(v5)

# verify: v6 differs from v5 in exactly 2 bytes (the checksum), and from factory only in
# voice window + voice size/crc fields + checksum
v5_orig = open(V5, "rb").read()
d5 = [i for i in range(len(out)) if out[i] != v5_orig[i]]
print(f"v6 vs v5 diff bytes: {[hex(i) for i in d5]}")
allowed = set(range(0x160000, 0x1DE000)) | set(range(0x20F2, 0x20FA)) | {CK_OFF, CK_OFF+1}
bad = [i for i in range(len(en)) if en[i] != out[i] and i not in allowed]
assert not bad, f"unexpected diffs vs factory: {[hex(b) for b in bad[:5]]}"
print("v6 vs factory: only voice window + size/crc + table checksum differ  OK")

# final: table self-consistent on v6
assert (sum(out[0x2000:0x2000+L]) & 0xFFFF) == struct.unpack_from("<H", out, CK_OFF)[0]
print("v6 partition table checksum validates  OK")

open(V6, "wb").write(out)
print(f"\nWROTE {V6}")
print(f"md5 {hashlib.md5(out).hexdigest()}")

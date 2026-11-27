import struct

EN = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin"
CN = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\04 CI1302_中文_单麦_V00729_UART1_115200_2M.bin"

en = open(EN, "rb").read()
cn = open(CN, "rb").read()
L = min(len(en), len(cn))
print(f"EN {hex(len(en))}  CN {hex(len(cn))}")

# coarse diff map per 4KB block
same_blocks, diff_regions = 0, []
cur = None
for b in range(0, L, 0x1000):
    if en[b:b+0x1000] == cn[b:b+0x1000]:
        same_blocks += 1
        if cur is not None:
            diff_regions.append((cur, b)); cur = None
    else:
        if cur is None: cur = b
if cur is not None:
    diff_regions.append((cur, L))
print(f"identical 4KB blocks: {same_blocks}")
print("diff regions (4KB granularity):")
for a, b in diff_regions:
    print(f"  {hex(a)} .. {hex(b)}  ({(b-a)//1024} KB)")

# exact byte diffs in the first 0x3000 (header/metadata zone)
print("\nbyte diffs in first 0x3000:")
i = 0
while i < 0x3000:
    if en[i] != cn[i]:
        j = i
        while j < 0x3000 and en[j] != cn[j]:
            j += 1
        print(f"  {hex(i)}..{hex(j)}: EN={en[i:j].hex()} CN={cn[i:j].hex()}")
        i = j
    else:
        i += 1

# dump the info block area of EN (0x2000-0x2100)
def hexdump(b, base, n):
    for i in range(0, n, 16):
        chunk = b[base+i:base+i+16]
        hexs = " ".join(f"{c:02X}" for c in chunk)
        asc = "".join(chr(c) if 32 <= c <= 126 else "." for c in chunk)
        print(f"{base+i:08X}  {hexs:<48}  {asc}")
print("\nEN 0x2000-0x2100:")
hexdump(en, 0x2000, 0x100)

# does CN have voice index at 0x160000 too?
for name, d in (("EN", en), ("CN", cn)):
    n = struct.unpack_from("<H", d, 0x160000)[0]
    cid0, off0, sz0 = struct.unpack_from("<HII", d, 0x160002)
    print(f"{name} @0x160000: count={n} first=({cid0},{hex(off0)},{sz0})")

# what's after EN voice content end 0x1dd383 -> next non-FF/00 aligned boundary
print("\nEN tail region boundaries:")
for p in (0x1dd383, 0x1dd400, 0x1de000, 0x1df000, 0x1e0000):
    print(f"  {hex(p)}: {en[p:p+16].hex()}")

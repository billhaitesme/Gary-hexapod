import struct

EN = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin"
V4 = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\voice\wonderecho\GARY_K2SO_v4_factory_swap.bin"
en = open(EN, "rb").read()
v4 = open(V4, "rb").read()

def hexdump(b, base, n):
    for i in range(0, n, 16):
        chunk = b[base+i:base+i+16]
        hexs = " ".join(f"{c:02X}" for c in chunk)
        asc = "".join(chr(c) if 32 <= c <= 126 else "." for c in chunk)
        print(f"{base+i:08X}  {hexs:<48}  {asc}")

# 1) all occurrences of the FACTORY voice entry bytes (addr 0x160000, size 0x7d383)
needle = struct.pack("<II", 0x160000, 0x7D383)
print("factory voice (addr,size) occurrences in EN:")
i = -1
while True:
    i = en.find(needle, i + 1)
    if i < 0: break
    print(f"  @{hex(i)}")

# 2) all occurrences of the sfw name
name = b"sfw2025030518523949901915"
print("\nsfw name occurrences in EN:")
i = -1
while True:
    i = en.find(name, i + 1)
    if i < 0: break
    print(f"  @{hex(i)}")

# 3) all occurrences of DEMO_Board
nb = b"DEMO_Board"
print("\nDEMO_Board occurrences in EN:")
i = -1
while True:
    i = en.find(nb, i + 1)
    if i < 0: break
    print(f"  @{hex(i)}")

# 4) dump 0x20000 (the 0x100-byte 'Config' partition candidate)
print("\nEN @0x20000 (0x100 bytes):")
hexdump(en, 0x20000, 0x100)

# 5) dump 0x3000 (region between manifest and user code)
print("\nEN @0x3000 (0x60 bytes):")
hexdump(en, 0x3000, 0x60)
print("\nEN @0x1000 (0x60 bytes):")
hexdump(en, 0x1000, 0x60)

# 6) where do EN and V4 differ overall (should be voice window + 2 fields only)
print("\nEN vs V4 diff regions (4KB blocks):")
cur = None
for b in range(0, len(en), 0x1000):
    if en[b:b+0x1000] != v4[b:b+0x1000]:
        if cur is None: cur = b
    else:
        if cur is not None:
            print(f"  {hex(cur)}..{hex(b)}"); cur = None
if cur is not None: print(f"  {hex(cur)}..{hex(len(en))}")

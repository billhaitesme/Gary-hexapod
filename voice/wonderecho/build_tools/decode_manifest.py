import struct, zlib

EN = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin"
en = open(EN, "rb").read()

def hexdump(b, base, n):
    lines = []
    for i in range(0, n, 16):
        chunk = b[base+i:base+i+16]
        hexs = " ".join(f"{c:02X}" for c in chunk)
        asc = "".join(chr(c) if 32 <= c <= 126 else "." for c in chunk)
        lines.append(f"{base+i:08X}  {hexs:<48}  {asc}")
    return "\n".join(lines)

print(hexdump(en, 0x2090, 0x130))

# candidate partition entries: scan 0x2090..0x2200 for u32 addr fields that are
# 0x1000-aligned and < 0x200000, with a plausible u32 size after them
print("\ncandidate (addr,size) pairs:")
cands = []
for p in range(0x2090, 0x2200):
    addr, size = struct.unpack_from("<II", en, p)
    if addr % 0x1000 == 0 and 0x1000 <= addr < 0x200000 and 0 < size < 0x200000 and addr + size <= 0x200000:
        cands.append((p, addr, size))
        print(f"  @{hex(p)}: addr={hex(addr)} size={hex(size)} ({size})")

# CRC brute-match: for each candidate, compute various checksums of en[addr:addr+size]
# and search for the result in 0x2090..0x2200 as u16/u32
def sum32(b): return sum(b) & 0xFFFFFFFF
def crc16_ccitt(b, init=0x0000, poly=0x1021):
    c = init
    for byte in b:
        c ^= byte << 8
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c
def crc16_modbus(b):
    c = 0xFFFF
    for byte in b:
        c ^= byte
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

meta = en[0x2090:0x2200]
for p, addr, size in cands:
    seg = en[addr:addr+size]
    results = {
        "crc32": zlib.crc32(seg) & 0xFFFFFFFF,
        "sum32": sum32(seg),
        "crc16ccitt0": crc16_ccitt(seg, 0x0000),
        "crc16ccittF": crc16_ccitt(seg, 0xFFFF),
        "crc16modbus": crc16_modbus(seg),
    }
    found = []
    for name, val in results.items():
        b4 = struct.pack("<I", val)
        b2 = struct.pack("<H", val & 0xFFFF)
        i4 = meta.find(b4)
        i2 = meta.find(b2)
        if i4 >= 0:
            found.append(f"{name}=U32@{hex(0x2090+i4)}")
        elif name.startswith("crc16") and i2 >= 0:
            found.append(f"{name}=U16@{hex(0x2090+i2)}")
    print(f"addr={hex(addr)} size={hex(size)}: {found if found else 'no checksum match'} "
          f"[crc32={results['crc32']:08x} sum32={results['sum32']:08x}]")

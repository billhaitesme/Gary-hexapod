import sys, hashlib, struct

BINS = {
    "FACTORY_EN": r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin",
    "FACTORY_CN": r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\04 CI1302_中文_单麦_V00729_UART1_115200_2M.bin",
    "K2SO_V3":    r"C:\Users\BillH\Documents\Arduino\gary_v1\WonderEcjho Firmwares\CI130X_SDK_Offline_uart_1_12_16\projects\cwsl_A_sample\firmware\GARY_K2SO_build\GARY_K2SO_v3.bin",
    "APRIL_V2":   r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\voice\wonderecho\CI1302_GARY_v2_firmware.bin",
}
VOICE_BIN = r"C:\Users\BillH\Documents\Arduino\gary_v1\WonderEcjho Firmwares\CI130X_SDK_Offline_uart_1_12_16\projects\cwsl_A_sample\firmware\voice\voice.bin"

def hexdump(b, off=0, n=256):
    out = []
    for i in range(0, min(n, len(b)), 16):
        chunk = b[i:i+16]
        hexs = " ".join(f"{c:02X}" for c in chunk)
        asc = "".join(chr(c) if 32 <= c <= 126 else "." for c in chunk)
        out.append(f"{off+i:08X}  {hexs:<48}  {asc}")
    return "\n".join(out)

data = {}
for name, path in BINS.items():
    try:
        with open(path, "rb") as f:
            data[name] = f.read()
        print(f"{name}: {len(data[name])} bytes  md5={hashlib.md5(data[name]).hexdigest()}")
    except Exception as e:
        print(f"{name}: FAILED {e}")

with open(VOICE_BIN, "rb") as f:
    vb = f.read()
print(f"voice.bin: {len(vb)} bytes md5={hashlib.md5(vb).hexdigest()}")

print("\n=== HEADER OF EACH BIN (first 256 bytes) ===")
for name in data:
    print(f"\n--- {name} ---")
    print(hexdump(data[name], 0, 256))

print("\n=== voice.bin head (64 bytes) ===")
print(hexdump(vb, 0, 64))

print("\n=== Where does voice.bin content sit inside K2SO_V3? ===")
k = data.get("K2SO_V3")
if k is not None:
    idx = k.find(vb[:4096])
    print(f"voice.bin[0:4096] found at offset: {hex(idx) if idx >= 0 else 'NOT FOUND'}")
    if idx >= 0:
        full = k.find(vb)
        print(f"full voice.bin at: {hex(full) if full >= 0 else 'only prefix found'}")

print("\n=== Same voice content in FACTORY_EN? (should NOT be) ===")
fe = data.get("FACTORY_EN")
if fe is not None and k is not None:
    idx = fe.find(vb[:4096])
    print(f"our voice.bin prefix in factory: {hex(idx) if idx >= 0 else 'not present (expected)'}")

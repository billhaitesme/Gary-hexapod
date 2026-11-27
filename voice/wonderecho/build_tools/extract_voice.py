import struct, re, zipfile, os, json

FE = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin"
XLSX = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\03 Command Word and Playback Phrase Protocol List-V3_20250305.xlsx"
OUT = os.path.dirname(os.path.abspath(__file__))
VBASE = 0x160000

with open(FE, "rb") as f:
    fe = f.read()

n = struct.unpack_from("<H", fe, VBASE)[0]
recs = []
for i in range(n):
    cid, off, sz = struct.unpack_from("<HII", fe, VBASE + 2 + 10 * i)
    recs.append((cid, off, sz))
print(f"{n} entries; region content ends at {hex(recs[-1][1] + recs[-1][2])} (file {hex(VBASE + recs[-1][1] + recs[-1][2])})")

# codec check on first few clips
for cid, off, sz in recs[:5] + recs[-3:]:
    d = fe[VBASE+off : VBASE+off+16]
    tag = "MP3" if d[0] == 0xFF and (d[1] & 0xE0) == 0xE0 else ("ID3" if d[:3] == b"ID3" else d[:4].hex())
    print(f"  id {cid:5d} off {hex(off):>9} size {sz:6d}  starts: {tag} {d.hex()[:24]}")

# param entry (65535) content
pid, poff, psz = recs[-1]
if pid == 65535:
    print("param entry text:", fe[VBASE+poff:VBASE+poff+psz])

# read xlsx shared strings + first sheet to map IDs -> phrases
z = zipfile.ZipFile(XLSX)
print("\nxlsx parts:", [i.filename for i in z.infolist()])
shared = []
try:
    ss = z.read("xl/sharedStrings.xml").decode("utf-8", "replace")
    shared = [re.sub(r"<.*?>", "", m) for m in re.findall(r"<si>(.*?)</si>", ss, re.S)]
    print(f"{len(shared)} shared strings")
except KeyError:
    print("no sharedStrings")

sheets = [i.filename for i in z.infolist() if i.filename.startswith("xl/worksheets/sheet")]
rows_out = []
for sh in sheets:
    xml = z.read(sh).decode("utf-8", "replace")
    rows = re.findall(r"<row[^>]*>(.*?)</row>", xml, re.S)
    print(f"\n--- {sh}: {len(rows)} rows, first 25: ---")
    for r_i, row in enumerate(rows[:25]):
        cells = re.findall(r'<c r="([A-Z]+)\d+"(?:[^>]*t="(\w+)")?[^>]*>(?:<v>(.*?)</v>)?', row)
        vals = []
        for col, t, v in cells:
            if t == "s" and v:
                vals.append(shared[int(v)] if int(v) < len(shared) else v)
            else:
                vals.append(v)
        print(f"  row{r_i}: {vals}")
    rows_out.append((sh, len(rows)))

# dump index to json for later use
with open(os.path.join(OUT, "factory_voice_index.json"), "w") as f:
    json.dump([{"id": c, "off": o, "size": s} for c, o, s in recs], f, indent=1)
print("\nindex saved to factory_voice_index.json")

import re, zipfile, json, os, struct

XLSX = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\1. Tutorials\8. AI Voice Interaction Course\8.1 Introduction and Installation of WonderEcho\03 Command Word and Playback Phrase Protocol List-V3_20250305.xlsx"
OUT = os.path.dirname(os.path.abspath(__file__))

z = zipfile.ZipFile(XLSX)
ss = z.read("xl/sharedStrings.xml").decode("utf-8", "replace")
shared = [re.sub(r"<.*?>", "", m) for m in re.findall(r"<si>(.*?)</si>", ss, re.S)]

xml = z.read("xl/worksheets/sheet1.xml").decode("utf-8", "replace")
rows = re.findall(r"<row[^>]*>(.*?)</row>", xml, re.S)
table = []
for row in rows:
    cells = re.findall(r'<c r="([A-Z]+)(\d+)"(?:[^>]*t="(\w+)")?[^>]*>(?:<v>(.*?)</v>)?', row)
    d = {}
    for col, rownum, t, v in cells:
        val = shared[int(v)] if (t == "s" and v) else v
        d[col] = val
    if d.get("A") and d.get("A").isdigit():
        table.append({
            "id": int(d["A"]),
            "word": d.get("B", ""),
            "type": d.get("C", ""),
            "phrase": d.get("D", ""),
            "proto": d.get("F", ""),
        })
print(f"{len(table)} phrase rows")
with open(os.path.join(OUT, "phrase_table.json"), "w", encoding="utf-8") as f:
    json.dump(table, f, ensure_ascii=False, indent=1)

# join with factory index sizes
idx = json.load(open(os.path.join(OUT, "factory_voice_index.json")))
sizes = {e["id"]: e["size"] for e in idx}
missing = [t["id"] for t in table if t["id"] not in sizes]
print("xlsx ids missing from factory index:", missing)
for t in table:
    t["slot_bytes"] = sizes.get(t["id"], 0)

# print full table compactly
for t in table:
    print(f'{t["id"]:4d} {t["slot_bytes"]:6d}B  {t["word"]:<28} {t["phrase"][:44]}')
extra = sorted(set(sizes) - {t["id"] for t in table} - {65535})
print("factory index ids not in xlsx:", extra)

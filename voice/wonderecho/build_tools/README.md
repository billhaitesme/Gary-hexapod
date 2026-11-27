# WonderEcho K-2SO voice — factory-swap build pipeline (2026-08-30)

**Shipped artifact: `../GARY_K2SO_v6_factory_swap.bin` — CONFIRMED PLAYING GEORGE
on the module (Bill's ear, 2026-08-30 ~02:40).** The failed `GARY_K2SO_v3.bin`
(April `ci-tool-kit make-firmware` output, not a valid container, never booted)
and v4/v5 (see bugs below) are history; v6 is the keeper.

## The approach

Do NOT rebuild firmware from the April SDK project. Instead take the **proven
factory bin** (`05 CI1302_En_SingleMic_V00729_UART1_115200_2M.bin` in
`1. Tutorials/8. AI Voice Interaction Course/8.1 .../`) and surgically replace
only the **voice partition**, leaving bootloader/user_code/ASR/DNN/cmd/user_file
byte-identical. Recognition, wake word (HELLO-HIWONDER), STUDY-WAKE self-learn,
and the AA 55 protocol Gary's ESP32 polls all stay factory.

## Container format (reverse-engineered, verified against SDK source)

- The bin is a raw flash image. `partition_table_t` (SDK
  `ci_flash_data_info.h`) sits at **0x2000**, 278 bytes packed:
  product/hw/sw names + per-partition {version u32, offset u32, size u32,
  crc u32-stored-low16, status u8} for user_code1/2, asr, dnn, **voice**,
  user_file, then nv offset/size, then **`patitiontablechecksum` u16 at
  0x2114 = plain byte-sum of the first 276 bytes** (`get_partition_list_
  checksum` in `ci_flash_data_info.c`). Boot AND the flash tool's no-force
  validation both check it.
- Partition CRC = **CRC16-CCITT, init 0x0000, poly 0x1021** over the
  partition's `size` bytes at `offset`.
- Factory EN voice partition: offset 0x160000, budget to 0x1DE000 (516,096 B).
- Voice partition layout: u16 clip_count, then count × {u16 id, u32 off,
  u32 size} (10-byte packed records), offsets relative to partition start,
  clips 16-byte aligned. Entries MAY share data (dedupe verified working on
  chip via v6's shared YESSIR/silence slots).
- **Every clip needs the vendor wrapper**: 30-byte header
  `"ID3\x03\x00\x00" + \x00\x00\x00\x1E + "PRIV" + be32(10) + \x00\x00 +
  "CI" + le32(total_clip_bytes) + le32(total_samples)` followed by raw
  MPEG-2 Layer III frames, 24 kbps 16 kHz mono (108-byte frames, 576
  samples each; total_samples = frames×576). Plain lame output (LAME tag,
  Xing/Info frame) does NOT play — strip and rewrap (`encode_v5.py`).

## The two bugs that cost the night (both in v4)

1. Clips lacked the `CI` PRIV wrapper (lame's own tags instead) → player
   skips them.
2. Manifest voice size/CRC were patched but `patitiontablechecksum` was
   not → boot rejects the whole partition table → total silence, and the
   tool refuses the file without force-all. **A bin that passes file-select
   with force UNTICKED has been validated by the vendor's own check.**

## Pipeline (re-run order)

1. `phrase_table.py` — clip-ID ↔ phrase/protocol map from the vendor xlsx
   (`phrase_table.json`); `extract_voice.py` — factory index
   (`factory_voice_index.json`).
2. `encode_v5.py` — WAV → lame 24k/16kHz mono (`-t`, no id3) → strip → CI
   wrap → `george_ci/*.bin` (self-checks every frame).
3. `assemble_v5.py` — build the new voice region (slot mapping + dedupe/
   sharing lives HERE), splice at 0x160000, patch voice size+CRC, verify
   byte-identity outside the window + all partition CRCs.
4. `build_v6.py` — recompute `patitiontablechecksum` (calibrates length
   against the factory bin first), final verify, write the flashable bin.

Slot mapping in `assemble_v5.py`: George primaries (welcome=1, wake=3,
movement 11-14/19, sit family, hello/intro/skill 36-38, march, combat+hooks,
goodbye/goodnight, thanks), YESSIR shared across all generic "Copy that"
acks, silence for CV/waste/parking junk Gary can't do, factory David kept
for volume acks and unknown IDs (80-90, 121-149).

Forensics helpers: `dissect.py`, `scan_factory.py`, `diff_bins.py`,
`decode_manifest.py`, `clip_forensics.py`, `find_backup_manifest.py`.

Flash: PACK_UPDATE_TOOL (chip CI1302, 921600). Select bin with force
unticked = validation test. After success UNTICK THE COM ROW immediately —
an armed row catches the post-flash reboot and reflashes forever.

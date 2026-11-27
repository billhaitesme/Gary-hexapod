# Gary — sensor expansion, decided 2026-08-23

Came out of the Ø2-02 Jetson shopping; recorded here so it is not buried in that project.

## The blocker has always been ADDRESSES, not wiring

Every VL53L0X (and the VL53L5CX/L8CX) ships at I2C address **0x29**. Five of them on one bus means
five devices answering to the same name — which is what "needs I2C splitter + addressing" in the
April handover actually meant. **A passive splitter does not fix this.** Two real options:

* **TCA9548A 8-channel multiplexer** — each sensor on its own channel, mux at 0x70. Buy this.
* XSHUT sequencing — boot them one at a time on GPIOs and reassign addresses. Free, fiddly, costs
  five GPIOs.

**Buy a GENERIC TCA9548A with 0.1in header pins**, not SparkFun's Qwiic Mux ($15.12 + $15.12
delivery, and it is Qwiic-only). Qwiic is 1.0 mm JST-SH; Hiwonder's ecosystem conventionally uses
2.0 mm JST-PH — **physically incompatible**, and the ToF below has header pins anyway.

✅ **CONFIRMED (Bill, 2026-08-23): Gary's I2C ports are PH2.0 4-pin, 2.0 mm pitch, polarised /
anti-reverse.** So Qwiic (1.0 mm JST-SH) does not mate, and neither does the **JST-XH (2.5 mm)**
kit already in the parts bin — three similar-looking 4-pin connectors, none interchangeable:

| standard | pitch | who uses it |
|---|---|---|
| Qwiic / STEMMA QT (JST-SH) | 1.0 mm | SparkFun, Adafruit |
| **JST-PH — Gary** | **2.0 mm** | Hiwonder |
| JST-XH (owned) | 2.5 mm | the duck's shared kit |

**Missing link: ALREADY SOLVED, buy nothing.** Gary's kit cables are 4-pin **PH2.0 200 mm
module-to-ESP32** leads. Cut one in half and it becomes two PH2.0 pigtails with bare wire — solder
those onto the mux's 0.1in headers. Check the silkscreen on Gary's port for the SDA/SCL/VCC/GND
order first; Hiwonder's pin order does not necessarily match the mux's header labels.

## Which sensor for what

| job | sensor | status |
|---|---|---|
| **cliff detection** (deferred since April) | **VL53L0X**, single point, angled down | **2 already owned, free** |
| **wall climb** — seeing rung geometry | **VL53L8CX** 8x8 multizone | ordered 2026-08-23 |

The L8CX was chosen over the L5CX (newer, cheaper delivered) — note it is a **different driver**
from the L5CX pair the duck owns, and confirm the listing offers **I2C** mode (it supports SPI too).

## The D500 is NOT blind up close — I was wrong about that

Looked up rather than assumed (2026-08-23). **D500 minimum range is 0.03 m**, max 12 m; it is a
DToF unit in the LD19 / STL-19P family. The claim that "spinning LiDARs are typically blind inside
10-15 cm" is a generic assumption that does **not** hold here. At the climb's **9.5-17 cm** gap the
D500 sees perfectly well.

### ...and it is not single-plane either — NOD SCAN IS IMPLEMENTED

Corrected again after Bill pointed out the gimbal-nod. The "nod scan designed not implemented"
line is from the **11 July** manual; it was **implemented on 13 July** and iterated four times
after three failed hardware tests, with AUTOSCAN wired to trigger it. `gary_config.h:205` is the
proof in code: the tip alarm was raised 30 -> 45 deg specifically so "intentional tilting
(including the **nod-scan sweep**)" would not trip it.

| | status |
|---|---|
| nod-scan **pass 1** (pitch fan) | implemented, iterated 4x, working |
| nod-scan **pass 2** (16-point circular pitch/roll sweep) | implemented, **never had a clean hardware run**; radius held at 20 deg pending one |

So pitching the body sweeps the scan plane and the D500 **can** recover rung height, from hardware
already mounted and already coded.

**Where the multizone might still earn its place, narrowly:**

* **Surveying before a climb** — stand off, nod-scan, build a rung map. **The D500 does this today,
  for free.** Validating nod-scan pass 2 is already on the to-do list.
* **Real-time feedback DURING a climb** — a nod sweep takes seconds and needs body pitch, which
  mid-climb is busy doing the climbing. A fixed 8x8 returns geometry instantly without moving.

That second case is the honest argument for the VL53L8CX. It is a real one, but it is much narrower
than "the LiDAR cannot see rungs", which was wrong twice over.

**Superseded (kept so the reasoning is not repeated):** the D500 scans one horizontal plane.
It measures distance-to-wall superbly — that is exactly the `gap 17 -> needs <=9.5` variable the
wall work is stuck on — and it cannot resolve **rung HEIGHT**, because vertical structure is not in
its plane.

So the multizone's advantage is **dimensionality, not range**:

| | D500 | VL53L8CX |
|---|---|---|
| sees at 10 cm | yes (3 cm min) | yes (2 cm) |
| distance to the wall | excellent | yes |
| **rung height** | **no — single plane** | **yes, 8x8 zones** |
| accuracy below 30 cm | unspecified (spec tiers start at 300 mm) | — |

**Cheapest experiment first:** the D500 can already feed the gap-closure loop, and that costs
nothing but firmware. Sweeping the plane by pitching the body is the other free option. Reach for
the multizone when the problem is *where is the rung*, not *how far is the wall*.

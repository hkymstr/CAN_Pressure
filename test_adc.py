"""
ADC MISO Pin Hunt — run with:  mpremote run test_adc.py

Probes every possible SPI0_RX candidate (GP0, GP4, GP8, GP16, GP20)
plus GP6 and GP7 using SoftSPI, so you can see which pin actually
carries the ADC128S022 DOUT signal without guessing.

Also tests GP7 explicitly since the board builder reported seeing
MISO there in the schematic.
"""

import machine
import utime

# ---------- known-correct pins ----------
SPI0_SCK_PIN  = 2
SPI0_TX_PIN   = 3    # MOSI
ADC_CS_PIN    = 5
SD_CS_PIN     = 1
MUX_CS_PIN    = 14
MUX_S0_PIN    = 16
MUX_S1_PIN    = 17
MUX_S2_PIN    = 18
MUX_S3_PIN    = 19

SEP  = "=" * 60
DASH = "-" * 60

# De-select all SPI0 devices immediately
_sd_cs  = machine.Pin(SD_CS_PIN,  machine.Pin.OUT, value=1)
_adc_cs = machine.Pin(ADC_CS_PIN, machine.Pin.OUT, value=1)
_mux_cs = machine.Pin(MUX_CS_PIN, machine.Pin.OUT, value=1)

# ==========================================================
# PHASE 0: GPIO state of every MISO candidate
# ==========================================================
print(SEP)
print("  Phase 0 — GPIO state of every MISO candidate")
print(SEP)

# Valid RP2040 SPI0_RX pins: 0, 4, 8, 16, 20
# Schematic-reported candidate: 7
CANDIDATES = [0, 4, 6, 7, 8, 16, 20]

for gp in CANDIDATES:
    try:
        p = machine.Pin(gp, machine.Pin.IN, machine.Pin.PULL_UP)
        pu = p.value()
        p = machine.Pin(gp, machine.Pin.IN, machine.Pin.PULL_DOWN)
        pd = p.value()
        driven = ""
        if pu == 0:
            driven = "  ← driven LOW (something active)"
        elif pd == 1:
            driven = "  ← driven HIGH (something active)"
        else:
            driven = "  ← floating"
        print(f"  GP{gp:>2}: pull-up={pu}  pull-down={pd}{driven}")
    except Exception as e:
        print(f"  GP{gp:>2}: ERROR {e}")

# ==========================================================
# PHASE 1: SoftSPI read from every candidate MISO pin
# ==========================================================
print()
print(SEP)
print("  Phase 1 — SoftSPI read attempt from each candidate MISO")
print("  (SoftSPI works regardless of hardware SPI pin limits)")
print(SEP)

adc_cs = machine.Pin(ADC_CS_PIN, machine.Pin.OUT, value=1)

def softread(miso_pin, ch=1, label=""):
    """Do a prime + read using SoftSPI with the given MISO pin."""
    try:
        spi = machine.SoftSPI(
            baudrate=500_000,
            sck=machine.Pin(SPI0_SCK_PIN),
            mosi=machine.Pin(SPI0_TX_PIN),
            miso=miso_pin)
        cmd = bytes([(ch & 0x07) << 3, 0x00])
        rx  = bytearray(2)
        # prime
        adc_cs.value(0); utime.sleep_us(1)
        spi.write_readinto(cmd, rx)
        adc_cs.value(1)
        # real read
        adc_cs.value(0); utime.sleep_us(1)
        spi.write_readinto(cmd, rx)
        adc_cs.value(1)
        raw = ((rx[0] & 0x0F) << 8) | rx[1]
        hit = "  <<< NON-ZERO — THIS IS YOUR MISO PIN" if raw != 0 else ""
        print(f"  MISO=GP{miso_pin.id():>2} ({label:<18}): RX=[{rx[0]:02X} {rx[1]:02X}]  raw={raw:4d}{hit}")
    except Exception as e:
        print(f"  MISO=GP{miso_pin.id():>2}: ERROR {e}")

# RP2040 valid SPI0_RX assignments
softread(machine.Pin(0),  ch=1, label="SPI0_RX option")
softread(machine.Pin(4),  ch=1, label="SPI0_RX option")
softread(machine.Pin(6),  ch=1, label="(not SPI0_RX)")
softread(machine.Pin(7),  ch=1, label="schematic claim")
softread(machine.Pin(8),  ch=1, label="SPI1_RX / CAN")
softread(machine.Pin(16), ch=1, label="SPI0_RX option")
softread(machine.Pin(20), ch=1, label="SPI0_RX option")

# ==========================================================
# PHASE 2: Continuous sweep with the most likely pin
#          Change MISO_PIN below to whichever showed NON-ZERO
# ==========================================================
print()
print(SEP)
print("  Phase 2 — Continuous sweep (edit MISO_PIN if Phase 1 found it)")
print(SEP)

MISO_PIN = 4    # ← change this to whichever pin Phase 1 showed non-zero

mux_s = [
    machine.Pin(MUX_S0_PIN, machine.Pin.OUT),
    machine.Pin(MUX_S1_PIN, machine.Pin.OUT),
    machine.Pin(MUX_S2_PIN, machine.Pin.OUT),
    machine.Pin(MUX_S3_PIN, machine.Pin.OUT),
]

spi = machine.SoftSPI(
    baudrate=1_000_000,
    sck=machine.Pin(SPI0_SCK_PIN),
    mosi=machine.Pin(SPI0_TX_PIN),
    miso=machine.Pin(MISO_PIN))

def adc_read(ch):
    cmd = bytes([(ch & 0x07) << 3, 0x00])
    rx  = bytearray(2)
    adc_cs.value(0); utime.sleep_us(1)
    spi.write_readinto(cmd, rx)
    adc_cs.value(1)
    adc_cs.value(0); utime.sleep_us(1)
    spi.write_readinto(cmd, rx)
    adc_cs.value(1)
    return ((rx[0] & 0x0F) << 8) | rx[1]

def mux_read(ch):
    _mux_cs.value(0)
    for i, pin in enumerate(mux_s):
        pin.value((ch >> i) & 1)
    utime.sleep_us(50)
    result = adc_read(0)
    _mux_cs.value(1)
    return result

print(f"  Using MISO=GP{MISO_PIN}. Ctrl-C to stop.\n")
sweep = 0
while True:
    sweep += 1
    direct = [adc_read(ch) for ch in range(1, 8)]
    mux4   = [mux_read(ch) for ch in range(4)]
    nonzero = any(v != 0 for v in direct + mux4)
    status  = "*** NON-ZERO ***" if nonzero else "all zero"
    print(f"Sweep {sweep:4d}: IN1-7={direct}  MUX0-3={mux4}  [{status}]")
    utime.sleep_ms(500)

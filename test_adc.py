"""
ADC Diagnostic v2 — run with:  mpremote run test_adc.py

Tests SPI0 + ADC128S022 + CD74HC4067 MUX in isolation.
Includes MISO state probing and SD-card CS isolation so you
can tell the difference between a firmware bug and a hardware fault.
"""

import machine
import utime

# ---------- pin constants ----------
SPI0_SCK_PIN = 2
SPI0_TX_PIN  = 3
SPI0_RX_PIN  = 4
ADC_CS_PIN   = 5
SD_CS_PIN    = 1   # SAME SPI0 bus — must be held HIGH
MUX_CS_PIN   = 14
MUX_S0_PIN   = 16
MUX_S1_PIN   = 17
MUX_S2_PIN   = 18
MUX_S3_PIN   = 19

SEP  = "=" * 60
DASH = "-" * 60

# ==========================================================
# PHASE 1: MISO state check BEFORE touching SPI hardware
# ==========================================================
print(SEP)
print("  Phase 1 — MISO (GP4) state as plain GPIO")
print(SEP)

# De-select every SPI0 device immediately so nothing fights MISO
# GP1 = SD_CS (active-low), GP5 = ADC_CS (active-low)
_sd_cs  = machine.Pin(SD_CS_PIN,  machine.Pin.OUT, value=1)
_adc_cs = machine.Pin(ADC_CS_PIN, machine.Pin.OUT, value=1)

utime.sleep_ms(10)

miso = machine.Pin(SPI0_RX_PIN, machine.Pin.IN)
print(f"  GP4 no-pull   : {miso.value()}")

miso = machine.Pin(SPI0_RX_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
print(f"  GP4 pull-up   : {miso.value()}  (0 = something driving LOW)")

miso = machine.Pin(SPI0_RX_PIN, machine.Pin.IN, machine.Pin.PULL_DOWN)
print(f"  GP4 pull-down : {miso.value()}  (1 = something driving HIGH)")

print()
print("  Expected when ADC is powered and CS de-asserted:")
print("    pull-up → 1  (MISO high-Z, internal pull wins)")
print("  If pull-up → 0:")
print("    MISO is being driven LOW → SD_CS or ADC_CS stuck asserted,")
print("    OR short to GND, OR ADC not powered.")

# ==========================================================
# PHASE 2: SPI transfer with each CS, show raw bytes
# ==========================================================
print()
print(SEP)
print("  Phase 2 — Raw SPI bytes (100 kHz, slow for scope view)")
print(SEP)

spi0 = machine.SPI(
    0, baudrate=100_000, polarity=0, phase=0, bits=8,
    sck=machine.Pin(SPI0_SCK_PIN),
    mosi=machine.Pin(SPI0_TX_PIN),
    miso=machine.Pin(SPI0_RX_PIN))

sd_cs  = machine.Pin(SD_CS_PIN,  machine.Pin.OUT, value=1)
adc_cs = machine.Pin(ADC_CS_PIN, machine.Pin.OUT, value=1)

def spi_xfer(cs, tx_bytes):
    rx = bytearray(len(tx_bytes))
    cs.value(0)
    utime.sleep_us(2)
    spi0.write_readinto(bytes(tx_bytes), rx)
    cs.value(1)
    return rx

# Send ch=1 command  (0x08 0x00) — asking for ADC IN1
for label, cs in [("ADC_CS (GP5)", adc_cs), ("SD_CS  (GP1)", sd_cs)]:
    rx = spi_xfer(cs, [0x08, 0x00])
    raw = ((rx[0] & 0x0F) << 8) | rx[1]
    print(f"  {label}: TX=[08 00]  RX=[{rx[0]:02X} {rx[1]:02X}]  raw={raw:4d}")

# Also with CS de-asserted (baseline — what does MISO float to?)
rx = bytearray(2)
spi0.write_readinto(bytes([0x08, 0x00]), rx)
raw = ((rx[0] & 0x0F) << 8) | rx[1]
print(f"  No CS asserted: TX=[08 00]  RX=[{rx[0]:02X} {rx[1]:02X}]  raw={raw:4d}")

print()
print("  If RX is always [00 00] regardless of CS:")
print("    → MISO is stuck LOW (hardware, not firmware)")
print("  If RX differs between CS=ADC and CS=none:")
print("    → ADC is responding on the SPI bus ✓")

# ==========================================================
# PHASE 3: Multi-speed sweep with ADC_CS
# ==========================================================
print()
print(SEP)
print("  Phase 3 — ADC CH1 at multiple SPI speeds")
print(SEP)
for baud in [100_000, 500_000, 1_000_000, 2_000_000]:
    spi0 = machine.SPI(
        0, baudrate=baud, polarity=0, phase=0, bits=8,
        sck=machine.Pin(SPI0_SCK_PIN),
        mosi=machine.Pin(SPI0_TX_PIN),
        miso=machine.Pin(SPI0_RX_PIN))
    # prime
    spi_xfer(adc_cs, [0x08, 0x00])
    rx = spi_xfer(adc_cs, [0x08, 0x00])
    raw = ((rx[0] & 0x0F) << 8) | rx[1]
    print(f"  {baud//1000:5d} kHz  RX=[{rx[0]:02X} {rx[1]:02X}]  raw={raw:4d}")

# ==========================================================
# PHASE 4: Continuous sweep (for live scope / LA probing)
# ==========================================================
print()
print(SEP)
print("  Phase 4 — Continuous ADC sweep at 2 MHz (Ctrl-C to stop)")
print("  Attach scope/logic analyser to GP2(SCK) GP4(MISO) GP5(CS)")
print(SEP)

mux_cs = machine.Pin(MUX_CS_PIN, machine.Pin.OUT, value=1)
mux_s  = [
    machine.Pin(MUX_S0_PIN, machine.Pin.OUT),
    machine.Pin(MUX_S1_PIN, machine.Pin.OUT),
    machine.Pin(MUX_S2_PIN, machine.Pin.OUT),
    machine.Pin(MUX_S3_PIN, machine.Pin.OUT),
]

spi0 = machine.SPI(
    0, baudrate=2_000_000, polarity=0, phase=0, bits=8,
    sck=machine.Pin(SPI0_SCK_PIN),
    mosi=machine.Pin(SPI0_TX_PIN),
    miso=machine.Pin(SPI0_RX_PIN))

def read_adc(ch):
    spi_xfer(adc_cs, [(ch & 0x07) << 3, 0x00])          # prime
    rx = spi_xfer(adc_cs, [(ch & 0x07) << 3, 0x00])
    return ((rx[0] & 0x0F) << 8) | rx[1]

def read_mux(ch):
    mux_cs.value(0)
    for i, pin in enumerate(mux_s):
        pin.value((ch >> i) & 1)
    utime.sleep_us(50)
    result = read_adc(0)
    mux_cs.value(1)
    return result

sweep = 0
while True:
    sweep += 1
    vals_direct = [read_adc(ch) for ch in range(1, 8)]
    vals_mux    = [read_mux(ch) for ch in range(16)]
    any_nonzero = any(v != 0 for v in vals_direct + vals_mux)
    status = "*** NON-ZERO ***" if any_nonzero else "all zero"
    print(f"Sweep {sweep:4d}: direct={vals_direct}  mux0-3={vals_mux[:4]}  [{status}]")
    utime.sleep_ms(500)

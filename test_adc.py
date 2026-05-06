"""
ADC Diagnostic — run with:  mpremote run test_adc.py

Tests SPI0 + ADC128S022 + CD74HC4067 MUX in isolation.
No SD card, CAN, or RTC involved.  Prints raw 12-bit counts
and converted voltages for every sample so you can verify
each channel is alive.
"""

import machine
import utime

# ---------- pin constants (must match schematic) ----------
SPI0_SCK_PIN = 2
SPI0_TX_PIN  = 3
SPI0_RX_PIN  = 4
ADC_CS_PIN   = 5
MUX_CS_PIN   = 14
MUX_S0_PIN   = 16
MUX_S1_PIN   = 17
MUX_S2_PIN   = 18
MUX_S3_PIN   = 19

# ---------- hardware init ----------
print("Initialising SPI0 at 2 MHz mode-0 …")
spi0 = machine.SPI(
    0, baudrate=2_000_000, polarity=0, phase=0, bits=8,
    sck=machine.Pin(SPI0_SCK_PIN),
    mosi=machine.Pin(SPI0_TX_PIN),
    miso=machine.Pin(SPI0_RX_PIN))

adc_cs = machine.Pin(ADC_CS_PIN, machine.Pin.OUT, value=1)
mux_cs = machine.Pin(MUX_CS_PIN, machine.Pin.OUT, value=1)
mux_s  = [
    machine.Pin(MUX_S0_PIN, machine.Pin.OUT),
    machine.Pin(MUX_S1_PIN, machine.Pin.OUT),
    machine.Pin(MUX_S2_PIN, machine.Pin.OUT),
    machine.Pin(MUX_S3_PIN, machine.Pin.OUT),
]
print("SPI0 and pins ready.")

# ---------- helpers ----------

def read_adc_raw(ch):
    """Single 16-bit SPI transaction; returns raw 12-bit result."""
    cmd = (ch & 0x07) << 3   # DIN[13:11] = channel address
    rx  = bytearray(2)
    adc_cs.value(0)
    utime.sleep_us(1)
    spi0.write_readinto(bytes([cmd, 0x00]), rx)
    adc_cs.value(1)
    return ((rx[0] & 0x0F) << 8) | rx[1]

def read_adc(ch):
    """Double-read to flush ADC pipeline after any channel switch."""
    read_adc_raw(ch)          # prime
    return read_adc_raw(ch)   # valid result

def set_mux(ch):
    """Assert MUX enable and set channel address (0-15)."""
    mux_cs.value(0)
    for i, pin in enumerate(mux_s):
        pin.value((ch >> i) & 1)
    utime.sleep_us(50)        # MUX propagation + signal settle

def read_mux(ch):
    """Read one MUX+ADC channel; returns 12-bit integer."""
    set_mux(ch)
    result = read_adc(0)      # ADC IN0 is wired to MUX output
    mux_cs.value(1)
    return result

# ---------- main diagnostic loop ----------

SEP  = "=" * 56
DASH = "-" * 56

print()
print(SEP)
print("  ADC128S022 + CD74HC4067 Diagnostic  (Ctrl-C to stop)")
print(SEP)

sweep = 0
while True:
    sweep += 1
    print(f"\n--- Sweep {sweep} ---")

    # Direct ADC channels 1-7 (not MUXed)
    print("  Direct ADC IN1-IN7:")
    for ch in range(1, 8):
        raw = read_adc(ch)
        v   = raw / 4095.0 * 3.3
        print(f"    IN{ch}: raw={raw:4d}  {v:.3f} V")

    # MUX channels 0-15 (all via ADC IN0)
    print("  MUX channels 0-15 (via ADC IN0):")
    for mux_ch in range(16):
        raw = read_mux(mux_ch)
        v   = raw / 4095.0 * 3.3
        bar = "#" * (raw * 20 // 4095)
        print(f"    MUX{mux_ch:>2}: raw={raw:4d}  {v:.3f} V  |{bar}")

    utime.sleep_ms(1000)

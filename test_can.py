"""
MCP2515 CAN loopback test — run with:  mpremote run test_can.py

No external CAN bus or adapter required.  Puts the MCP2515 in
loopback mode so every transmitted frame is immediately received
back through the internal path, verifying SPI comms, bit-timing
registers, and frame formatting end-to-end.
"""

import machine
import utime

# ---------- pin constants ----------
CAN_CLK_PIN = 7
CAN_RX_PIN  = 8
CAN_CS_PIN  = 9
CAN_SCK_PIN = 10
CAN_TX_PIN  = 11

# ---------- MCP2515 registers / commands ----------
MCP_RESET    = 0xC0
MCP_READ     = 0x03
MCP_WRITE    = 0x02
MCP_LOAD_TX0 = 0x40
MCP_RTS_TX0  = 0x81
MCP_READ_RX0 = 0x90   # read RX buffer 0 starting at RXB0SIDH

MCP_CANCTRL  = 0x0F
MCP_CANSTAT  = 0x0E
MCP_CANINTF  = 0x2C
MCP_CNF1     = 0x2A
MCP_CNF2     = 0x29
MCP_CNF3     = 0x28
MCP_RXB0CTRL = 0x60   # receive buffer 0 control

SEP  = "=" * 56
PASS = "PASS"
FAIL = "FAIL"

# ---------- hardware init ----------
can_clk = machine.PWM(machine.Pin(CAN_CLK_PIN))
can_clk.freq(8_000_000)
can_clk.duty_u16(32768)

spi1 = machine.SPI(
    1, baudrate=10_000_000, polarity=0, phase=0, bits=8,
    sck=machine.Pin(CAN_SCK_PIN),
    mosi=machine.Pin(CAN_TX_PIN),
    miso=machine.Pin(CAN_RX_PIN))

cs = machine.Pin(CAN_CS_PIN, machine.Pin.OUT, value=1)

# ---------- helpers ----------
def wr(addr, val):
    cs.value(0); spi1.write(bytes([MCP_WRITE, addr, val])); cs.value(1)

def rd(addr):
    buf = bytearray(1)
    cs.value(0); spi1.write(bytes([MCP_READ, addr])); spi1.readinto(buf); cs.value(1)
    return buf[0]

def reset():
    cs.value(0); spi1.write(bytes([MCP_RESET])); cs.value(1)
    utime.sleep_ms(10)

def check(label, result):
    status = PASS if result else FAIL
    mark   = "✓" if result else "✗"
    print(f"  [{mark}] {label:<40} {status}")
    return result

# ---------- test ----------
print(SEP)
print("  MCP2515 CAN Loopback Test")
print(SEP)

all_pass = True

# 1. Reset and enter config mode
print("\n1. Reset and configuration mode")
reset()
wr(MCP_CANCTRL, 0x80)   # REQOP = 100 → config mode
utime.sleep_ms(5)
stat = rd(MCP_CANSTAT)
mode = (stat >> 5) & 0x07
all_pass &= check("CANSTAT OPMODE = 0b100 (config)", mode == 0b100)

# 2. Set 500 kbps bit timing for 8 MHz oscillator
print("\n2. Bit-timing registers (500 kbps @ 8 MHz)")
wr(MCP_CNF1, 0x00)
wr(MCP_CNF2, 0x90)
wr(MCP_CNF3, 0x02)
all_pass &= check("CNF1 readback = 0x00", rd(MCP_CNF1) == 0x00)
all_pass &= check("CNF2 readback = 0x90", rd(MCP_CNF2) == 0x90)
all_pass &= check("CNF3 readback = 0x02", rd(MCP_CNF3) == 0x02)

# 3. Set loopback mode (REQOP = 010)
print("\n3. Enter loopback mode")
wr(MCP_CANCTRL, 0x40)   # REQOP = 010 → loopback
utime.sleep_ms(5)
stat = rd(MCP_CANSTAT)
mode = (stat >> 5) & 0x07
all_pass &= check("CANSTAT OPMODE = 0b010 (loopback)", mode == 0b010)

# Allow all messages into RXB0 (no filter)
wr(MCP_RXB0CTRL, 0x60)   # RXM = 11 → accept all

# 4. Transmit a test frame with known payload and check loopback receive
print("\n4. TX/RX loopback frames")

test_cases = [
    (0x200, bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])),
    (0x205, bytes([0xFF, 0x00, 0xAA, 0x55, 0x12, 0x34, 0x56, 0x78])),
    (0x123, bytes([0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00])),
]

for can_id, payload in test_cases:
    # Clear RX interrupt flag
    wr(MCP_CANINTF, 0x00)

    # Load and send frame
    sidh = (can_id >> 3) & 0xFF
    sidl = (can_id & 0x07) << 5
    dlc  = len(payload)
    buf  = bytearray([MCP_LOAD_TX0, sidh, sidl, 0x00, 0x00, dlc])
    buf.extend(payload)
    cs.value(0); spi1.write(buf); cs.value(1)
    cs.value(0); spi1.write(bytes([MCP_RTS_TX0])); cs.value(1)

    # Wait up to 10 ms for RX0IF flag
    deadline = utime.ticks_add(utime.ticks_ms(), 10)
    while not (rd(MCP_CANINTF) & 0x01):
        if utime.ticks_diff(deadline, utime.ticks_ms()) <= 0:
            break

    rx_flag = bool(rd(MCP_CANINTF) & 0x01)

    # Read received frame
    rx = bytearray(13)
    cs.value(0)
    spi1.write(bytes([MCP_READ_RX0]))
    spi1.readinto(rx)
    cs.value(1)

    rx_id   = ((rx[0] << 3) | (rx[1] >> 5)) & 0x7FF
    rx_dlc  = rx[4] & 0x0F
    rx_data = bytes(rx[5:5 + rx_dlc])

    id_ok   = rx_id   == can_id
    data_ok = rx_data == payload
    ok      = rx_flag and id_ok and data_ok
    all_pass &= ok

    status = PASS if ok else FAIL
    print(f"  ID=0x{can_id:03X}  flag={rx_flag}  "
          f"rx_id=0x{rx_id:03X}  data_match={data_ok}  → {status}")

# 5. Summary
print()
print(SEP)
if all_pass:
    print("  ALL TESTS PASSED — MCP2515 SPI, config, and CAN framing OK")
else:
    print("  FAILURES DETECTED — check SPI wiring, CS pin, and 8 MHz clock")
print(SEP)

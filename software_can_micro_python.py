"""
CAN Pressure Board — MicroPython firmware
Raspberry Pi Pico (RP2040)

Hardware pin assignments (verified from schematic CAN_adc.pdf):
  SPI0  SCK/MOSI/MISO : GP2 / GP3 / GP4
  SD card CS           : GP1
  ADC128S022 CS        : GP5
  SPI1  SCK/MOSI/MISO : GP10 / GP11 / GP8
  MCP2515 CS           : GP9
  MCP2515 8 MHz clock  : GP7  (PWM)
  CD74HC4067 enable    : GP14
  CD74HC4067 S0-S3     : GP16-GP19
  RP2040 built-in RTC used for timestamps
  Menu / UART interface via USB-CDC (115200 baud at host)

Channel layout (23 total):
  adc_data[0..15]  — MUX channels 0-15  (ADC IN0 through CD74HC4067)
  adc_data[16..22] — Direct ADC IN1-IN7

CAN telemetry (4 ch × 2 bytes = 8 bytes per frame):
  0x200 : CH1-4   | 0x201 : CH5-8   | 0x202 : CH9-12
  0x203 : CH13-16 | 0x204 : CH17-20 | 0x205 : CH21-23 + pad
"""

import machine
import utime
import ustruct
import uos
import sys
import uselect

try:
    import sdcard  # community MicroPython SD-card driver
    _SDCARD_AVAILABLE = True
except ImportError:
    _SDCARD_AVAILABLE = False
    print("WARNING: sdcard module not found — SD logging disabled.")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# SPI0 — ADC + SD card
SD_CS_PIN    = 1
SPI0_SCK_PIN = 2
SPI0_TX_PIN  = 3
SPI0_RX_PIN  = 4
ADC_CS_PIN   = 5

# SPI1 — MCP2515 CAN controller
CAN_CLK_PIN  = 7   # 8 MHz PWM output
CAN_RX_PIN   = 8   # SPI1 MISO
CAN_CS_PIN   = 9
CAN_SCK_PIN  = 10
CAN_TX_PIN   = 11  # SPI1 MOSI

# CD74HC4067 MUX
MUX_CS_PIN   = 14
MUX_S0_PIN   = 16
MUX_S1_PIN   = 17
MUX_S2_PIN   = 18
MUX_S3_PIN   = 19

# Sampling / display
SAMPLE_HZ       = 5
SAMPLE_PERIOD   = 1000 // SAMPLE_HZ   # 200 ms
CAN_TX_PERIOD   = 500                 # ms  (2 Hz)
DISPLAY_PERIOD  = 1000                # ms  (1 Hz)

# Display modes
DISP_OFF   = 0
DISP_TABLE = 1
DISP_CSV   = 2
DISP_BOTH  = 3
_DISP_NAMES = {DISP_OFF: "OFF", DISP_TABLE: "TABLE", DISP_CSV: "CSV", DISP_BOTH: "BOTH"}

# MCP2515 SPI commands
MCP_RESET    = 0xC0
MCP_READ     = 0x03
MCP_WRITE    = 0x02
MCP_LOAD_TX0 = 0x40   # load TX buffer 0, start at SIDH
MCP_RTS_TX0  = 0x81   # request-to-send TX0

# MCP2515 registers
MCP_CANCTRL = 0x0F
MCP_CNF3    = 0x28
MCP_CNF2    = 0x29
MCP_CNF1    = 0x2A

# CAN bit-timing for 8 MHz oscillator
# Each config = (CNF1, CNF2, CNF3)
# 8 TQ/bit scheme: SYNC=1 PROP=1 PS1=3 PS2=3, BRP varies
# 4 TQ/bit scheme for 1 Mbps: SYNC=1 PROP=1 PS1=1 PS2=1
_CAN_BAUD = {
    125:  (0x03, 0x90, 0x02),   # BRP=3  → TQ=1 µs  → 8 TQ → 125 kbps
    250:  (0x01, 0x90, 0x02),   # BRP=1  → TQ=500 ns → 8 TQ → 250 kbps
    500:  (0x00, 0x90, 0x02),   # BRP=0  → TQ=250 ns → 8 TQ → 500 kbps
    1000: (0x00, 0x80, 0x00),   # BRP=0  → TQ=250 ns → 4 TQ → 1 Mbps
}
DEFAULT_BAUD = 500

# CAN base ID and channel count
CAN_BASE_ID = 0x200
NUM_CH      = 23

# OBD channel definitions: (name, unit, min, max)
# Temperature scaling: thermocouple amp output = 10 mV/°C → 0–330 °C over 0–3.3 V
# Current scaling:     INA4180A2 (50 V/V) + 50 mΩ shunt → 0–1.32 A over 0–3.3 V
# Voltage_5V scaling:  0.5 resistor divider → ×2 → 0–6.6 V over 0–3.3 V ADC
_OBD_CHANNELS = [
    # CH1–6: thermocouple inputs (via MUX)
    ("Temperature_1",   "C",     0, 330),   # index 0
    ("Temperature_2",   "C",     0, 330),   # index 1
    ("Temperature_3",   "C",     0, 330),   # index 2
    ("Temperature_4",   "C",     0, 330),   # index 3
    ("Temperature_5",   "C",     0, 330),   # index 4
    ("Temperature_6",   "C",     0, 330),   # index 5
    # CH7–9: spare MUX inputs (unconnected)
    ("Spare_1",         "V",     0, 3.3),   # index 6
    ("Spare_2",         "V",     0, 3.3),   # index 7
    ("Spare_3",         "V",     0, 3.3),   # index 8
    # CH10–14: analog inputs
    ("Analog_In_1",     "V",     0, 3.3),   # index 9
    ("Analog_In_2",     "V",     0, 3.3),   # index 10
    ("Analog_In_3",     "V",     0, 3.3),   # index 11
    ("Analog_In_4",     "V",     0, 3.3),   # index 12
    ("Analog_In_5",     "V",     0, 3.3),   # index 13
    # CH15: spare (unused)
    ("Spare_4",         "V",     0, 3.3),   # index 14
    # CH16: 3.3 V rail monitor via 0.5 resistor divider (×2 to recover voltage)
    ("Voltage_3V3",     "V",     0, 6.6),   # index 15
    # CH17: analog input (cont.)
    ("Analog_In_6",     "V",     0, 3.3),   # index 16
    # CH18: differential pressure sensor (SSCDRRN005PDAA5, 0–15 psi)
    ("Diff_Pressure",   "psi",   0,  15),   # index 17
    # CH19: gauge pressure sensor (SSCDANND015PGAA5, 0–150 psi)
    ("Pressure",        "psi",   0, 150),   # index 18
    # CH20–22: current monitors (INA4180A2, 50 V/V gain, 50 mΩ shunt)
    ("Input_Current",   "A",     0, 1.32),  # index 19
    ("Curr_5V",         "A",     0, 1.32),  # index 20
    ("Curr_3V3",        "A",     0, 1.32),  # index 21
    # CH23: 5 V rail monitor via 0.5 resistor divider (×2 to recover voltage)
    ("Voltage_5V",      "V",     0, 6.6),   # index 22
]


# ---------------------------------------------------------------------------
# Data Acquisition System
# ---------------------------------------------------------------------------

class DataAcquisitionSystem:
    def __init__(self):
        self._init_spi()
        self._init_pins()
        self._init_can_clock()
        self.rtc = machine.RTC()
        self.adc_data     = [0] * NUM_CH
        self.can_baud     = DEFAULT_BAUD
        self.display_mode = DISP_CSV   # send $DATA, lines by default
        self._sd_ok       = False
        self._init_sd()
        self._init_can(self.can_baud)
        # Non-blocking stdin poll for menu trigger
        self._poll = uselect.poll()
        self._poll.register(sys.stdin, uselect.POLLIN)

    # ------------------------------------------------------------------
    # Hardware init
    # ------------------------------------------------------------------

    def _init_spi(self):
        self.spi0 = machine.SPI(
            0, baudrate=2_000_000, polarity=0, phase=0, bits=8,
            sck=machine.Pin(SPI0_SCK_PIN),
            mosi=machine.Pin(SPI0_TX_PIN),
            miso=machine.Pin(SPI0_RX_PIN))

        self.spi1 = machine.SPI(
            1, baudrate=10_000_000, polarity=0, phase=0, bits=8,
            sck=machine.Pin(CAN_SCK_PIN),
            mosi=machine.Pin(CAN_TX_PIN),
            miso=machine.Pin(CAN_RX_PIN))

    def _init_pins(self):
        self.sd_cs  = machine.Pin(SD_CS_PIN,  machine.Pin.OUT, value=1)
        self.adc_cs = machine.Pin(ADC_CS_PIN, machine.Pin.OUT, value=1)
        self.can_cs = machine.Pin(CAN_CS_PIN, machine.Pin.OUT, value=1)
        self.mux_cs = machine.Pin(MUX_CS_PIN, machine.Pin.OUT, value=1)
        self.mux_s  = [
            machine.Pin(MUX_S0_PIN, machine.Pin.OUT),
            machine.Pin(MUX_S1_PIN, machine.Pin.OUT),
            machine.Pin(MUX_S2_PIN, machine.Pin.OUT),
            machine.Pin(MUX_S3_PIN, machine.Pin.OUT),
        ]

    def _init_can_clock(self):
        """Generate 8 MHz clock on GP7 for MCP2515."""
        self._can_clk = machine.PWM(machine.Pin(CAN_CLK_PIN))
        self._can_clk.freq(8_000_000)
        self._can_clk.duty_u16(32768)  # 50 % duty cycle

    def _init_sd(self):
        if not _SDCARD_AVAILABLE:
            return
        try:
            sd = sdcard.SDCard(self.spi0, self.sd_cs)
            uos.mount(sd, '/sd')
            self._sd_ok = True
            print("SD card mounted.")
            self._init_log_header()
            self._write_obd_file()
        except Exception as exc:
            print(f"SD card unavailable: {exc}")
        finally:
            # sdcard driver reconfigures SPI0 baudrate/mode during init;
            # always reinitialise so ADC reads are not corrupted.
            self.spi0 = machine.SPI(
                0, baudrate=2_000_000, polarity=0, phase=0, bits=8,
                sck=machine.Pin(SPI0_SCK_PIN),
                mosi=machine.Pin(SPI0_TX_PIN),
                miso=machine.Pin(SPI0_RX_PIN))

    # ------------------------------------------------------------------
    # MCP2515
    # ------------------------------------------------------------------

    def _can_write(self, addr, val):
        self.can_cs.value(0)
        self.spi1.write(bytes([MCP_WRITE, addr, val]))
        self.can_cs.value(1)

    def _can_read(self, addr):
        buf = bytearray(1)
        self.can_cs.value(0)
        self.spi1.write(bytes([MCP_READ, addr]))
        self.spi1.readinto(buf)
        self.can_cs.value(1)
        return buf[0]

    def _init_can(self, baud_kbps):
        # Hardware reset
        self.can_cs.value(0)
        self.spi1.write(bytes([MCP_RESET]))
        self.can_cs.value(1)
        utime.sleep_ms(10)

        # Enter configuration mode
        self._can_write(MCP_CANCTRL, 0x80)
        utime.sleep_ms(1)

        cnf1, cnf2, cnf3 = _CAN_BAUD.get(baud_kbps, _CAN_BAUD[500])
        self._can_write(MCP_CNF3, cnf3)
        self._can_write(MCP_CNF2, cnf2)
        self._can_write(MCP_CNF1, cnf1)

        # Normal mode
        self._can_write(MCP_CANCTRL, 0x00)
        utime.sleep_ms(1)

    def _send_can_frame(self, can_id, data):
        """Load and transmit one standard CAN frame via TX buffer 0."""
        sidh = (can_id >> 3) & 0xFF
        sidl = (can_id & 0x07) << 5
        dlc  = len(data) & 0x0F
        # Single CS-asserted transaction: LOAD_TX0 + header + payload
        buf = bytearray([MCP_LOAD_TX0, sidh, sidl, 0x00, 0x00, dlc])
        buf.extend(data)
        self.can_cs.value(0)
        self.spi1.write(buf)
        self.can_cs.value(1)
        # Request to send
        self.can_cs.value(0)
        self.spi1.write(bytes([MCP_RTS_TX0]))
        self.can_cs.value(1)

    # ------------------------------------------------------------------
    # ADC128S022 + CD74HC4067 MUX
    # ------------------------------------------------------------------

    def _set_mux(self, ch):
        """Select MUX channel 0-15 and assert MUX enable."""
        self.mux_cs.value(0)
        for i, pin in enumerate(self.mux_s):
            pin.value((ch >> i) & 1)
        utime.sleep_us(10)  # MUX propagation settle

    def _read_adc(self, ch):
        """
        Read one ADC128S022 channel (0-7), return 12-bit integer.
        Command word DIN[13:11] = channel address (see ADC128S022 datasheet).
        Result is pipelined one frame behind; prime the pipeline by calling
        this twice when switching channels (handled in sample_all_channels).
        """
        cmd = (ch & 0x07) << 3   # DIN[13:11] = channel
        rx  = bytearray(2)
        self.adc_cs.value(0)
        utime.sleep_us(1)
        self.spi0.write_readinto(bytes([cmd, 0x00]), rx)
        self.adc_cs.value(1)
        return ((rx[0] & 0x0F) << 8) | rx[1]

    def sample_all_channels(self):
        """
        Acquire all 23 channels at 5 Hz.
        adc_data[0..15]  = MUX ch 0-15 (via ADC IN0)
        adc_data[16..22] = ADC IN1-IN7 (direct)
        """
        # MUX channels: prime ADC pipeline after each MUX switch
        for i in range(16):
            self._set_mux(i)
            utime.sleep_us(50)   # MUX + signal settle
            self._read_adc(0)    # prime (pipeline lag)
            self.adc_data[i] = self._read_adc(0)
        self.mux_cs.value(1)     # de-assert MUX enable

        # Direct ADC channels 1-7 — double-read each to flush pipeline
        for i in range(1, 8):
            self._read_adc(i)                    # prime
            self.adc_data[15 + i] = self._read_adc(i)

    # ------------------------------------------------------------------
    # CAN telemetry transmit + SD log
    # ------------------------------------------------------------------

    def _timestamp(self):
        y, mo, d, _, h, mi, s, _ = self.rtc.datetime()
        return f"{y}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d}"

    def send_telemetry(self):
        """Pack 23 channels into 6 CAN frames (0x200-0x205) and log each."""
        ts = self._timestamp()
        for msg_idx in range(6):
            payload = bytearray(8)
            for j in range(4):
                ch = msg_idx * 4 + j
                val = self.adc_data[ch] if ch < NUM_CH else 0
                ustruct.pack_into('>H', payload, j * 2, val)
            can_id = CAN_BASE_ID + msg_idx
            self._send_can_frame(can_id, payload)
            self._log_can_row(ts, can_id, payload)
            utime.sleep_ms(1)   # brief gap between frames

    # ------------------------------------------------------------------
    # SD card logging
    # ------------------------------------------------------------------

    def _init_log_header(self):
        """Write CSV header if log file does not yet exist."""
        path = '/sd/can_log.csv'
        try:
            uos.stat(path)
        except OSError:
            with open(path, 'w') as f:
                f.write("Time Stamp,ID,Extended,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n")

    def _log_can_row(self, ts, can_id, data):
        if not self._sd_ok:
            return
        try:
            with open('/sd/can_log.csv', 'a') as f:
                row = f"{ts},0x{can_id:03X},0,0,{len(data)}"
                for b in data:
                    row += f",{b}"
                f.write(row + "\n")
        except Exception as exc:
            print(f"Log write error: {exc}")

    def _write_obd_file(self):
        """Write OBD channel definition file (created once)."""
        path = '/sd/channels.obd'
        try:
            uos.stat(path)
            return
        except OSError:
            pass
        with open(path, 'w') as f:
            for i, (name, unit, mn, mx) in enumerate(_OBD_CHANNELS, 1):
                can_id     = CAN_BASE_ID + (i - 1) // 4
                byte_off   = ((i - 1) % 4) * 2
                f.write(
                    f"CH{i} = {name}, {unit}, {mn}, {mx}, "
                    f"CAN_ID=0x{can_id:03X}, Byte={byte_off}, Len=2, "
                    f"Scale=1, BigEndian=1, Signed=0\n"
                )
        print("OBD file written to /sd/channels.obd")

    # ------------------------------------------------------------------
    # Serial telemetry display (1 Hz)
    # ------------------------------------------------------------------

    def _print_table(self):
        """Human-readable channel table printed to USB-CDC at 1 Hz."""
        ts   = self._timestamp()
        sep  = "=" * 58
        dash = "-" * 58
        print(sep)
        print(f"  CAN Pressure Board  |  {ts}  |  {self.can_baud} kbps")
        print(sep)
        print(f"  {'CH':>2}  {'Channel':<18}  {'Raw':>5}  {'Value':>9}  Unit")
        print(dash)
        scaled = []
        for i, (name, unit, mn, mx) in enumerate(_OBD_CHANNELS):
            raw = self.adc_data[i]
            if mx != mn:
                v = mn + (raw / 4095.0) * (mx - mn)
                scaled.append(v)
                val_str = f"{v:9.2f}"
            else:
                scaled.append(None)
                val_str = f"{'---':>9}"
            print(f"  {i+1:>2}  {name:<18}  {raw:>5}  {val_str}  {unit}")
        print(dash)
        # Derived power (V × I) for 5 V and 3.3 V rails
        # Indices: Voltage_5V=22, Curr_5V=20, Voltage_3V3=15, Curr_3V3=21
        v5  = scaled[22]; i5  = scaled[20]
        v3  = scaled[15]; i3  = scaled[21]
        p5  = v5 * i5 if (v5 is not None and i5 is not None) else None
        p3  = v3 * i3 if (v3 is not None and i3 is not None) else None
        p5s = f"{p5:9.3f}" if p5 is not None else f"{'---':>9}"
        p3s = f"{p3:9.3f}" if p3 is not None else f"{'---':>9}"
        print(f"  {'':2}  {'Power_5V':<18}  {'':>5}  {p5s}  W")
        print(f"  {'':2}  {'Power_3V3':<18}  {'':>5}  {p3s}  W")
        print(sep)

    def _print_csv(self):
        """
        Single CSV line prefixed with $DATA, for machine parsing.
        Format: $DATA,<timestamp>,<CH1>,...,<CH23>
        The PC monitor script filters for lines starting with $DATA,.
        """
        ts = self._timestamp()
        vals = ",".join(str(v) for v in self.adc_data)
        print(f"$DATA,{ts},{vals}")

    def display_telemetry(self):
        if self.display_mode == DISP_TABLE:
            self._print_table()
        elif self.display_mode == DISP_CSV:
            self._print_csv()
        elif self.display_mode == DISP_BOTH:
            self._print_table()
            self._print_csv()

    # ------------------------------------------------------------------
    # Menu (accessed via USB-CDC serial — press Ctrl-C or Enter)
    # ------------------------------------------------------------------

    def _menu_set_rtc(self):
        print("\nSet RTC — enter UTC date/time:")
        try:
            year = int(input("  Year  (YYYY) : "))
            mon  = int(input("  Month (1-12) : "))
            day  = int(input("  Day   (1-31) : "))
            hour = int(input("  Hour  (0-23) : "))
            mins = int(input("  Min   (0-59) : "))
            secs = int(input("  Sec   (0-59) : "))
            self.rtc.datetime((year, mon, day, 0, hour, mins, secs, 0))
            print(f"  RTC set → {self._timestamp()}")
        except Exception as exc:
            print(f"  Error: {exc}")

    def _menu_set_baud(self):
        print("\nSelect CAN baud rate:")
        opts = {1: 125, 2: 250, 3: 500, 4: 1000}
        for k, v in opts.items():
            mark = " ←" if v == self.can_baud else ""
            print(f"  {k}. {v} kbps{mark}")
        choice = input("Choice [1-4]: ").strip()
        baud = opts.get(int(choice) if choice.isdigit() else 0)
        if baud:
            self.can_baud = baud
            self._init_can(baud)
            print(f"  CAN set to {baud} kbps")
        else:
            print("  Invalid choice, no change.")

    def _menu_set_display(self):
        modes = {1: DISP_OFF, 2: DISP_TABLE, 3: DISP_CSV, 4: DISP_BOTH}
        print("\nSelect display mode:")
        for k, v in modes.items():
            mark = " ←" if v == self.display_mode else ""
            print(f"  {k}. {_DISP_NAMES[v]}{mark}")
        choice = input("Choice [1-4]: ").strip()
        mode = modes.get(int(choice) if choice.isdigit() else 0)
        if mode is not None:
            self.display_mode = mode
            print(f"  Display set to {_DISP_NAMES[mode]}")
        else:
            print("  Invalid choice, no change.")

    def _run_menu(self):
        mode_name = _DISP_NAMES[self.display_mode]
        print("\n=== CAN Pressure Board ===")
        print("  1. Set RTC time")
        print("  2. Set CAN baud rate")
        print(f"  3. Set display mode  [{mode_name}]")
        print("  4. Resume acquisition")
        while True:
            choice = input("> ").strip()
            if choice == '1':
                self._menu_set_rtc()
            elif choice == '2':
                self._menu_set_baud()
            elif choice == '3':
                self._menu_set_display()
            elif choice in ('4', '', 'q'):
                print("Resuming acquisition...\n")
                return
            else:
                print("  Enter 1, 2, 3, or 4.")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        print("CAN Pressure Board v2 — running.  Press Enter for menu.")
        print(f"Display mode: {_DISP_NAMES[self.display_mode]}  |  CAN: {self.can_baud} kbps  |  SD: {'OK' if self._sd_ok else 'none'}")
        t_sample  = utime.ticks_ms()
        t_can_tx  = utime.ticks_ms()
        t_display = utime.ticks_ms()

        while True:
            now = utime.ticks_ms()

            # Check for Enter key (non-blocking)
            if self._poll.poll(0):
                ch = sys.stdin.read(1)
                if ch in ('\r', '\n'):
                    self._run_menu()
                    t_sample  = utime.ticks_ms()
                    t_can_tx  = utime.ticks_ms()
                    t_display = utime.ticks_ms()
                    continue

            # Sample at 5 Hz
            if utime.ticks_diff(now, t_sample) >= SAMPLE_PERIOD:
                self.sample_all_channels()
                t_sample = utime.ticks_add(t_sample, SAMPLE_PERIOD)

            # Transmit CAN + log at 2 Hz
            if utime.ticks_diff(now, t_can_tx) >= CAN_TX_PERIOD:
                self.send_telemetry()
                t_can_tx = utime.ticks_add(t_can_tx, CAN_TX_PERIOD)

            # Display telemetry at 1 Hz
            if self.display_mode != DISP_OFF:
                if utime.ticks_diff(now, t_display) >= DISPLAY_PERIOD:
                    self.display_telemetry()
                    t_display = utime.ticks_add(t_display, DISPLAY_PERIOD)


if __name__ == "__main__":
    das = DataAcquisitionSystem()
    try:
        das.run()
    except KeyboardInterrupt:
        das._run_menu()
        das.run()
    finally:
        try:
            uos.umount('/sd')
        except Exception:
            pass

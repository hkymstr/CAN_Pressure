#!/usr/bin/env python3
"""
CAN Pressure Board — Serial Monitor and Data Logger

Connects to the board's USB-CDC port, displays live telemetry, and logs
all data rows to a timestamped CSV file.

Data rows from the board are prefixed with '$DATA,' — all other lines
(boot messages, table display, menu text) are printed as-is and not logged.

Usage examples:
    python can_monitor.py --port COM3
    python can_monitor.py --port /dev/ttyACM0 --output run1.csv
    python can_monitor.py --port COM3 --quiet
    python can_monitor.py --list

Requirements:
    pip install pyserial
"""

import argparse
import csv
import datetime
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed.  Run:  pip install pyserial")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Channel metadata — must match firmware _OBD_CHANNELS
# ---------------------------------------------------------------------------

CHANNELS = [
    # CH1–6: thermocouple inputs (10 mV/°C amp → 0–330 °C over 0–3.3 V)
    ("Temperature_1",  "C",      0,  330),
    ("Temperature_2",  "C",      0,  330),
    ("Temperature_3",  "C",      0,  330),
    ("Temperature_4",  "C",      0,  330),
    ("Temperature_5",  "C",      0,  330),
    ("Temperature_6",  "C",      0,  330),
    # CH7–9: spare MUX inputs (unconnected)
    ("Spare_1",        "V",      0,  3.3),
    ("Spare_2",        "V",      0,  3.3),
    ("Spare_3",        "V",      0,  3.3),
    # CH10–16: direct analog inputs (buffered, 0–3.3 V)
    ("Analog_In_1",    "V",      0,  3.3),
    ("Analog_In_2",    "V",      0,  3.3),
    ("Analog_In_3",    "V",      0,  3.3),
    ("Analog_In_4",    "V",      0,  3.3),
    ("Analog_In_5",    "V",      0,  3.3),
    ("Analog_In_6",    "V",      0,  3.3),
    ("Analog_In_7",    "V",      0,  3.3),
    # CH17: differential pressure (SSCDRRN005PDAA5, 0–15 psi)
    ("Diff_Pressure",  "psi",    0,   15),
    # CH18: gauge pressure (SSCDANND015PGAA5, 0–150 psi)
    ("Pressure",       "psi",    0,  150),
    # CH19–21: current monitors (INA4180A2, 50 V/V, 50 mΩ shunt → 0–1.32 A)
    ("Input_Current",  "A",      0, 1.32),
    ("Curr_5V",        "A",      0, 1.32),
    ("Curr_3V3",       "A",      0, 1.32),
    # CH22: 5 V rail via 0.5 divider (×2 recovery → 0–6.6 V)
    ("Voltage_5V",     "V",      0,  6.6),
    # CH23: spare
    ("Spare_4",        "V",      0,  3.3),
]

NUM_CH       = len(CHANNELS)
CH_NAMES     = [c[0] for c in CHANNELS]
CH_UNITS     = [c[1] for c in CHANNELS]
CH_MINS      = [c[2] for c in CHANNELS]
CH_MAXS      = [c[3] for c in CHANNELS]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def raw_to_scaled(raw, mn, mx):
    """Convert 12-bit raw count to engineering units."""
    if mx == mn:
        return None
    return mn + (int(raw) / 4095.0) * (mx - mn)


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for p in sorted(ports):
        print(f"  {p.device:<15}  {p.description}")


def open_port(port, baud, retries=5):
    for attempt in range(1, retries + 1):
        try:
            ser = serial.Serial(port, baud, timeout=2)
            print(f"Connected to {port} at {baud} baud.")
            return ser
        except serial.SerialException as exc:
            wait = 2 ** attempt
            print(f"  [{attempt}/{retries}] {exc}  — retrying in {wait}s")
            if attempt < retries:
                time.sleep(wait)
    print("Could not open port.  Exiting.")
    sys.exit(1)


def auto_filename():
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"can_pressure_{ts}.csv"


# ---------------------------------------------------------------------------
# Live display helpers
# ---------------------------------------------------------------------------

_CLEAR = "\033[2J\033[H"   # ANSI clear screen + cursor home

def print_live_table(timestamp, raw_vals):
    """Render a full-screen table of all 23 channels."""
    sep  = "=" * 62
    dash = "-" * 62
    print(_CLEAR, end="")
    print(sep)
    print(f"  CAN Pressure Board  |  {timestamp}")
    print(sep)
    print(f"  {'CH':>2}  {'Channel':<18}  {'Raw':>5}  {'Value':>10}  Unit")
    print(dash)
    for i, raw in enumerate(raw_vals):
        mn, mx = CH_MINS[i], CH_MAXS[i]
        scaled = raw_to_scaled(raw, mn, mx)
        val_str = f"{scaled:10.3f}" if scaled is not None else f"{'---':>10}"
        print(f"  {i+1:>2}  {CH_NAMES[i]:<18}  {int(raw):>5}  {val_str}  {CH_UNITS[i]}")
    print(sep)


def print_live_compact(timestamp, raw_vals, row_count):
    """One-line summary printed inline (for non-ANSI terminals)."""
    temp1   = raw_to_scaled(raw_vals[0],  CH_MINS[0],  CH_MAXS[0])
    dp      = raw_to_scaled(raw_vals[16], CH_MINS[16], CH_MAXS[16])
    press   = raw_to_scaled(raw_vals[17], CH_MINS[17], CH_MAXS[17])
    t_str   = f"{temp1:6.1f}°C"  if temp1  is not None else "  ---  "
    dp_str  = f"{dp:5.2f}psi"    if dp     is not None else " --- "
    p_str   = f"{press:6.1f}psi" if press  is not None else "  ---  "
    print(f"\r{timestamp}  T1={t_str}  DiffP={dp_str}  P={p_str}  "
          f"[{row_count} rows]", end="", flush=True)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run(port, baud, output_file, quiet, full_screen):
    ser = open_port(port, baud)

    if output_file is None:
        output_file = auto_filename()

    print(f"Logging to: {output_file}")
    if not quiet:
        mode = "full-screen table" if full_screen else "compact one-line"
        print(f"Display:    {mode}")
    print("Press Ctrl-C to stop.\n")

    row_count = 0

    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)
        # Header: Timestamp + one column per channel (raw) + one per channel (scaled)
        raw_header    = [f"{n}_raw" for n in CH_NAMES]
        scaled_header = [f"{n}_{u}" if u else n for n, u, *_ in CHANNELS]
        writer.writerow(["Timestamp"] + raw_header + scaled_header)
        f.flush()

        try:
            while True:
                try:
                    line = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
                except serial.SerialException:
                    print("\nPort disconnected — reconnecting...")
                    ser.close()
                    time.sleep(2)
                    ser = open_port(port, baud)
                    continue

                if not line:
                    continue

                if line.startswith("$DATA,"):
                    # Parse: $DATA,<timestamp>,<v0>,<v1>,...,<v22>
                    parts = line[6:].split(",")
                    if len(parts) < NUM_CH + 1:
                        continue   # malformed line

                    timestamp = parts[0]
                    try:
                        raw_vals = [int(p) for p in parts[1 : NUM_CH + 1]]
                    except ValueError:
                        continue

                    scaled_vals = [
                        raw_to_scaled(r, CH_MINS[i], CH_MAXS[i])
                        for i, r in enumerate(raw_vals)
                    ]
                    scaled_strs = [
                        f"{v:.4f}" if v is not None else "" for v in scaled_vals
                    ]

                    writer.writerow([timestamp] + raw_vals + scaled_strs)
                    f.flush()
                    row_count += 1

                    if not quiet:
                        if full_screen:
                            print_live_table(timestamp, raw_vals)
                        else:
                            print_live_compact(timestamp, raw_vals, row_count)

                else:
                    # Non-data line (boot message, table header, menu text)
                    if not quiet:
                        if not full_screen:
                            print()   # newline after compact row
                        print(line)

        except KeyboardInterrupt:
            if not quiet and not full_screen:
                print()
            print(f"\nStopped.  {row_count} rows written to {output_file}")
        finally:
            ser.close()


def main():
    parser = argparse.ArgumentParser(
        description="CAN Pressure Board — serial monitor and data logger",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  %(prog)s --port COM3
  %(prog)s --port /dev/ttyACM0 --output run1.csv
  %(prog)s --port COM3 --full-screen
  %(prog)s --port COM3 --quiet --output background.csv
  %(prog)s --list
        """)

    parser.add_argument("--port",  "-p",
                        help="Serial port  (e.g. COM3 or /dev/ttyACM0)")
    parser.add_argument("--baud",  "-b", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--output", "-o",
                        help="Output CSV file  (default: auto-timestamped)")
    parser.add_argument("--quiet", "-q", action="store_true",
                        help="Suppress all screen output — log only")
    parser.add_argument("--full-screen", "-f", action="store_true",
                        help="Clear screen and redraw full channel table each second")
    parser.add_argument("--list", "-l", action="store_true",
                        help="List available serial ports and exit")

    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        parser.print_help()
        print("\nERROR: --port is required.")
        print()
        list_ports()
        sys.exit(1)

    run(args.port, args.baud, args.output, args.quiet, args.full_screen)


if __name__ == "__main__":
    main()

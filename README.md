# CAN Pressure Board

A multi-channel telemetry data acquisition board built around the
**Raspberry Pi Pico (RP2040)**.  It reads K-type thermocouples, analog voltage
inputs, and pressure sensors, multiplexes all 23 channels through a 16:1 MUX
and 12-bit SPI ADC, and streams the results over a 500 kbps CAN bus while
simultaneously logging to a microSD card.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Architecture](#2-hardware-architecture)
3. [Bill of Materials (Key ICs)](#3-bill-of-materials-key-ics)
4. [Connector Reference](#4-connector-reference)
5. [Pin Assignments](#5-pin-assignments)
6. [Signal Chain](#6-signal-chain)
7. [Channel Map (23 channels)](#7-channel-map-23-channels)
8. [CAN Bus Interface](#8-can-bus-interface)
9. [SD Card Logging](#9-sd-card-logging)
10. [Power Supply](#10-power-supply)
11. [Firmware](#11-firmware)
12. [Serial Menu Interface](#12-serial-menu-interface)
13. [MCP2515 Bit Timing](#13-mcp2515-bit-timing)
14. [Repository Files](#14-repository-files)

---

## 1. System Overview

```
 Thermocouples (×6)        ┌──────────────┐
 Analog inputs  (×8)  ────►│ CD74HC4067   │ 16:1 MUX
 Spare MUX inputs(×2) ────►│ (ADC IN0)    │
                            └──────┬───────┘
                                   │
 Pressure sensors (×2) ────────────┤  ADC IN1-IN7 (direct)
                                   │
                            ┌──────▼───────┐
                            │ ADC128S022   │ 8-ch, 12-bit SPI ADC
                            └──────┬───────┘
                                   │ SPI0
                            ┌──────▼───────┐     ┌──────────┐
                            │  RP2040      ├────►│ MCP2515  │ CAN controller
                            │  (Pico)      │SPI1 │ TJA1050  │ CAN transceiver
                            └──────┬───────┘     └──────────┘
                                   │ SPI0
                            ┌──────▼───────┐
                            │ microSD card │ CSV log
                            └──────────────┘
```

**Key specifications:**

| Parameter | Value |
|---|---|
| Microcontroller | Raspberry Pi Pico (RP2040, dual-core Cortex-M0+, 133 MHz) |
| Total telemetry channels | 23 |
| Sample rate | 5 Hz (all 23 channels) |
| CAN transmit rate | 2 Hz (6 frames per cycle) |
| CAN baud rate (default) | 500 kbps |
| ADC resolution | 12-bit (0–4095 counts) |
| SD log format | CAN message CSV |
| Serial interface | USB-CDC, 115200 baud |

---

## 2. Hardware Architecture

### Schematic hierarchy

| Sheet | File | Description |
|---|---|---|
| Top level | `CAN_adc.kicad_sch` | Interconnects all sub-sheets |
| Pico interface | `Pico_interface.kicad_sch` | RP2040 Pico header and GPIO routing |
| ADC | `adc.kicad_sch` | ADC128S022 and SPI0 connections |
| MUX | `mux.kicad_sch` | CD74HC4067 16:1 analog MUX |
| CAN bus | `canbus.kicad_sch` | MCP2515 + TJA1050 with 120 Ω termination |
| Thermocouple ×2 | `thermo_revised.kicad_sch` | 3-ch K-type front-end per instance |
| Analog input ×2 | `analog_in.kicad_sch` | 4-ch differential buffer per instance |
| Input power | `input_power.kicad_sch` | Buck regulator, LDO, current sense |

### Block diagram detail

**Thermocouple front-end (×2 boards, 3 channels each = 6 total)**

Each board uses an **LM324** quad op-amp and an **LT1004 1.2 V reference** for
cold-junction compensation and signal conditioning.  Each board drives three
single-ended output signals (THERM_OUT1–3) which feed into the analog MUX.

**Analog input buffers (×2 boards, 4 channels each = 8 total)**

Each board uses an **LM324** quad op-amp configured as a unity-gain differential
buffer.  Differential inputs IN1+–IN4+ are converted to single-ended outputs
OUTPUT1–4 which feed into the analog MUX.

**Analog MUX (CD74HC4067)**

A single 16:1 analog MUX selects one of 16 inputs and routes it to
ADC channel IN0.  The MUX is enabled by a dedicated GPIO (GP14) and controlled
by four address lines (GP16–GP19).

```
MUX I0–I5   : Thermocouple conditioned outputs (6 channels)
MUX I6–I13  : Analog input buffered outputs (8 channels)
MUX I14–I15 : Spare MUX inputs (SPARE_MUX_INPUT_1, _2)
```

**ADC (ADC128S022)**

8-channel, 12-bit successive-approximation ADC with pipelined SPI interface.
Operates at up to 2 MSPS.  Channel IN0 receives the MUX output;
channels IN1–IN7 are wired directly to sensor signals.

```
ADC IN0  → CD74HC4067 MUX output (16 sub-channels)
ADC IN1  → Differential pressure sensor output
ADC IN2  → Pressure sensor output
ADC IN3–IN7 → TBD / spare analog inputs
```

> **ADC pipeline note:** The ADC128S022 outputs the result of the *previous*
> conversion while clocking in the *current* channel address.  The firmware
> performs one dummy read after each MUX channel switch to flush the pipeline
> before capturing data.

**CAN bus (MCP2515 + TJA1050)**

The MCP2515 CAN controller communicates with the RP2040 over SPI1.  The
RP2040 generates the required 8 MHz oscillator signal on GP7 using its PWM
peripheral.  The TJA1050 high-speed CAN transceiver drives the differential
CAN bus lines (CANH/CANL).  A 120 Ω termination resistor (R122) is fitted
on-board.

---

## 3. Bill of Materials (Key ICs)

| Reference | Part | Description |
|---|---|---|
| U9 | ADC128S022CIMT | 8-ch, 12-bit, 500 kSPS–2 MSPS SPI ADC |
| U14 | CD74HC4067SM96E4 | 16:1 single-channel analog MUX/DEMUX |
| U15 | MCP2515-I/P | Stand-alone CAN 2.0B controller, SPI interface |
| U16 | TJA1050 | High-speed CAN transceiver (ISO 11898) |
| U3, U11 | LT1004CDR-1.2 | 1.2 V micropower voltage reference |
| U4, U10, U12, U13 | LM324ADR | Quad op-amp (thermocouple + analog buffers) |
| U5 | MBRM140T3G | 1 A Schottky diode (reverse-polarity / flyback) |
| U6 | LT1121IS8-3.3 | 150 mA low-dropout 3.3 V regulator |
| U7 | LT1933IS6 | 500 mA synchronous step-down switching regulator |
| U8 | INA4180A2IPWR | 26 V, 20-bit current-sense amplifier |
| U1 (×2) | SSCDRRN005PDAA5 | Honeywell TruStability, ±5 psi differential pressure |
| — | 15psig sensor | 0–15 psig single-port pressure sensor |
| D1, D2 | LG_L29K-G2J1-24-Z | Würth dual-color LED indicator |

---

## 4. Connector Reference

| Ref | Part | Position | Signals |
|---|---|---|---|
| J1, J10 | Würth 691137710002 | Power input | VIN+ / GND (5.08 mm screw terminal) |
| J2, J3 | TE 5747840-3 | Thermocouple | THERM_x+, THERM_x−, GND (9-pos, 2.54 mm) |
| J4, J5 | Phoenix 1989764 | Analog inputs | IN1–IN4, GND (4-pos, 3.81 mm screw terminal) |
| J6 | TE 2201778-1 | Digital outputs | OUTPUT1–4 (4-pos, 2.54 mm header) |
| J8, J9 | Samtec PPPC201LFBN-RC | Pico socket | 2×20-pos elevated socket, 2.54 mm |
| J10 | — | CAN bus | CANH, CANL, GND |

---

## 5. Pin Assignments

All GPIO numbers are RP2040 physical GPIO, verified from schematic `CAN_adc.pdf`.

### SPI0 — ADC128S022 + SD card (shared bus)

| GPIO | Function |
|---|---|
| GP1 | SD card chip select (active-low) |
| GP2 | SPI0 SCK |
| GP3 | SPI0 MOSI |
| GP4 | SPI0 MISO |
| GP5 | ADC128S022 chip select (active-low) |

### SPI1 — MCP2515 CAN controller

| GPIO | Function |
|---|---|
| GP7 | 8 MHz PWM clock output → MCP2515 OSC1 |
| GP8 | SPI1 MISO (MCP2515 SO) |
| GP9 | MCP2515 chip select (active-low) |
| GP10 | SPI1 SCK (MCP2515 SCK) |
| GP11 | SPI1 MOSI (MCP2515 SI) |

### CD74HC4067 MUX control

| GPIO | Function |
|---|---|
| GP14 | MUX enable, active-low (~E) |
| GP16 | MUX address S0 (LSB) |
| GP17 | MUX address S1 |
| GP18 | MUX address S2 |
| GP19 | MUX address S3 (MSB) |

### Unassigned / spare

GP0, GP6, GP12, GP13, GP15, GP20–GP22 are not connected to board functions
and are available for expansion.

> **UART note:** An earlier revision of the requirements document listed GP16/17
> as a hardware UART interface.  The schematic shows these pins drive MUX_S0/S1.
> The serial menu is therefore provided via the Pico's USB-CDC port
> (appears as a virtual COM port on the host).

---

## 6. Signal Chain

```
K-type thermocouple
    │
    ▼
Differential pair ──► LM324 cold-junction amp ──► single-ended voltage
                       LT1004 1.2V reference
    │
    ▼
CD74HC4067 MUX (I0–I5)
    │
    ▼
ADC128S022 IN0 ──► RP2040 (SPI0) ──► 12-bit integer


0–3.3 V analog sensor
    │
    ▼
LM324 unity-gain buffer ──► single-ended voltage
    │
    ▼
CD74HC4067 MUX (I6–I13)
    │
    ▼
ADC128S022 IN0 ──► RP2040 (SPI0) ──► 12-bit integer


Pressure sensor (SSCDRRN005PDAA5 / 15psig)
    │
    ▼
ADC128S022 IN1–IN2 (direct, no MUX) ──► RP2040 (SPI0) ──► 12-bit integer
```

---

## 7. Channel Map (23 channels)

Raw ADC values are 12-bit unsigned integers (0–4095).
Engineering-unit conversion factors are defined in `channels.obd` on the SD card.

### MUX channels (via ADC IN0, CD74HC4067 I0–I15)

| adc_data index | MUX input | Channel | Default sensor | Unit | Min | Max |
|---|---|---|---|---|---|---|
| 0 | I0 | CH1 | Temperature 1 (K-type TC) | °C | −40 | 200 |
| 1 | I1 | CH2 | Temperature 2 (K-type TC) | °C | −40 | 200 |
| 2 | I2 | CH3 | Temperature 3 (K-type TC) | °C | −40 | 200 |
| 3 | I3 | CH4 | Temperature 4 (K-type TC) | °C | −40 | 200 |
| 4 | I4 | CH5 | Temperature 5 (K-type TC) | °C | −40 | 200 |
| 5 | I5 | CH6 | Temperature 6 (K-type TC) | °C | −40 | 200 |
| 6 | I6 | CH7 | Temperature 7 (K-type TC) | °C | −40 | 200 |
| 7 | I7 | CH8 | Temperature 8 (K-type TC) | °C | −40 | 200 |
| 8 | I8 | CH9 | Analog In 1 | V | 0 | 3.3 |
| 9 | I9 | CH10 | Analog In 2 | V | 0 | 3.3 |
| 10 | I10 | CH11 | Analog In 3 | V | 0 | 3.3 |
| 11 | I11 | CH12 | Analog In 4 | V | 0 | 3.3 |
| 12 | I12 | CH13 | Analog In 5 | V | 0 | 3.3 |
| 13 | I13 | CH14 | Analog In 6 | V | 0 | 3.3 |
| 14 | I14 | CH15 | Analog In 7 | V | 0 | 3.3 |
| 15 | I15 | CH16 | Analog In 8 | V | 0 | 3.3 |

### Direct ADC channels (ADC IN1–IN7)

| adc_data index | ADC input | Channel | Default sensor | Unit | Min | Max |
|---|---|---|---|---|---|---|
| 16 | IN1 | CH17 | Differential Pressure (SSCDRRN005PDAA5) | psi | 0 | 15 |
| 17 | IN2 | CH18 | Pressure (15 psig sensor) | psi | 0 | 150 |
| 18 | IN3 | CH19 | TBD | — | — | — |
| 19 | IN4 | CH20 | TBD | — | — | — |
| 20 | IN5 | CH21 | TBD | — | — | — |
| 21 | IN6 | CH22 | TBD | — | — | — |
| 22 | IN7 | CH23 | TBD | — | — | — |

> **Thermocouple note:** The six thermocouple channels (CH1–CH6) are conditioned
> by two `thermo_revised` boards (3 channels each).  Channels CH7–CH8 are
> conditioned by the same circuit on spare MUX inputs.  If fewer than eight
> thermocouples are installed, unused MUX inputs float; tie unused inputs to GND
> via a 49.9 Ω resistor to avoid noise.

---

## 8. CAN Bus Interface

### Physical layer

| Parameter | Value |
|---|---|
| Standard | CAN 2.0B (ISO 11898) |
| Transceiver | TJA1050 |
| Bus termination | 120 Ω on-board (R122) |
| Connector | Screw terminal (CANH, CANL, GND) |
| Default baud rate | 500 kbps |
| Oscillator | 8 MHz PWM from RP2040 GP7 |

### CAN frame layout

Each CAN frame carries **4 channels × 2 bytes = 8 bytes**, big-endian unsigned
16-bit integers (raw 12-bit ADC counts, zero-padded to 16 bits).

| CAN ID | Byte 0–1 | Byte 2–3 | Byte 4–5 | Byte 6–7 |
|---|---|---|---|---|
| **0x200** | CH1 (Temp 1) | CH2 (Temp 2) | CH3 (Temp 3) | CH4 (Temp 4) |
| **0x201** | CH5 (Temp 5) | CH6 (Temp 6) | CH7 (Temp 7) | CH8 (Temp 8) |
| **0x202** | CH9 (AIn 1) | CH10 (AIn 2) | CH11 (AIn 3) | CH12 (AIn 4) |
| **0x203** | CH13 (AIn 5) | CH14 (AIn 6) | CH15 (AIn 7) | CH16 (AIn 8) |
| **0x204** | CH17 (Diff P) | CH18 (Press) | CH19 (TBD) | CH20 (TBD) |
| **0x205** | CH21 (TBD) | CH22 (TBD) | CH23 (TBD) | 0x0000 (pad) |

All 6 frames are transmitted at **2 Hz**.  There is a 1 ms inter-frame gap
between each frame.

### Selectable baud rates

| Rate | CNF1 | CNF2 | CNF3 | TQ | Bit time |
|---|---|---|---|---|---|
| 125 kbps | 0x03 | 0x90 | 0x02 | 8 | 8 µs |
| 250 kbps | 0x01 | 0x90 | 0x02 | 8 | 4 µs |
| **500 kbps** (default) | 0x00 | 0x90 | 0x02 | 8 | 2 µs |
| 1000 kbps | 0x00 | 0x80 | 0x00 | 4 | 1 µs |

All rates use BRP = 0…3 with an 8 MHz oscillator.
Bit timing: SYNC=1 TQ, PROP=1 TQ, PS1=3 TQ, PS2=3 TQ (8 TQ scheme),
or SYNC=1, PROP=1, PS1=1, PS2=1 (4 TQ for 1 Mbps).

---

## 9. SD Card Logging

Two files are written to the SD card root.

### `can_log.csv`

Created at first boot; every CAN frame transmitted is appended as one CSV row.
At 2 Hz × 6 frames, this produces **12 rows per second**.

**Format:**
```
Time Stamp,ID,Extended,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8
```

**Example rows:**
```
2025-06-01 09:15:32,0x200,0,0,8,0,124,0,98,1,12,0,77
2025-06-01 09:15:32,0x201,0,0,8,0,88,0,91,0,75,0,83
2025-06-01 09:15:32,0x202,0,0,8,7,208,6,144,5,96,4,16
...
```

- `ID` — standard 11-bit CAN ID in hex (0x200–0x205)
- `Extended` — always 0 (standard frame)
- `Bus` — always 0 (single CAN bus)
- `LEN` — always 8 (DLC)
- `D1–D8` — raw payload bytes (decimal)

**Decoding a value:**  
`value = (D_n << 8) | D_(n+1)` for each big-endian uint16 pair.

### `channels.obd`

Written once at first boot; not overwritten on subsequent boots.
Defines all 23 channels for use with CAN data logging / analysis software.

**Format (one line per channel):**
```
CH<n> = <name>, <unit>, <min>, <max>, CAN_ID=0x<id>, Byte=<offset>, Len=2, Scale=1, BigEndian=1, Signed=0
```

**Example:**
```
CH1 = Temperature_1, C, -40, 200, CAN_ID=0x200, Byte=0, Len=2, Scale=1, BigEndian=1, Signed=0
CH17 = Diff_Pressure, psi, 0, 15, CAN_ID=0x204, Byte=0, Len=2, Scale=1, BigEndian=1, Signed=0
```

---

## 10. Power Supply

Designed by the `input_power.kicad_sch` sub-sheet.

| Rail | Source | Max current | Notes |
|---|---|---|---|
| VIN | External (J1/J10 screw terminals) | — | 5–26 V input range (LT1933 input limit) |
| 5 V | LT1933IS6 step-down regulator | 500 mA | Powers Pico VSYS, CAN transceiver |
| 3.3 V | LT1121IS8-3.3 LDO | 150 mA | Powers ADC, MUX, MCP2515 logic |
| 3.3 V (Pico) | Pico on-board LDO (from 5 V) | 300 mA | Powers RP2040 core |

**Current monitoring:** An **INA4180** current-sense amplifier monitors supply
rail currents.  Sense resistors of 50 mΩ are used (R36, R45, R46).  Current
data is brought out as `INPUT_CURR`, `5V_CURR`, `SPARE_CURR` net labels.

**Protection:** An MBRM140 Schottky diode (U5) provides reverse-polarity
protection on the input rail.

**Indicators:** Two dual-color LEDs (LG_L29K-G2J1-24-Z, D1/D2) are provided
for power or status indication.

---

## 11. Firmware

**File:** `software_can_micro_python.py`  
**Runtime:** MicroPython on RP2040

### Timing

| Event | Rate | Period |
|---|---|---|
| Sample all 23 channels | 5 Hz | 200 ms |
| Transmit CAN + log SD | 2 Hz | 500 ms |
| Menu check (non-blocking) | Every loop iteration | — |

### Sample sequence

```
For MUX channels 0–15:
  1. Assert MUX enable (GP14 low)
  2. Set MUX address (GP16–19)
  3. Wait 10 µs for MUX propagation
  4. Dummy SPI read  ← flush ADC pipeline after channel switch
  5. Real SPI read   ← store 12-bit result
  6. De-assert MUX enable

For direct ADC channels IN1–7:
  1. One priming read on IN1
  2. Pipelined read sequence IN1→IN7
```

### Required library

The community `sdcard` MicroPython driver must be present on the Pico:

```
# Install via mpremote
mpremote mip install sdcard
```

Or manually copy `sdcard.py` from
[micropython/micropython-lib](https://github.com/micropython/micropython-lib/blob/master/micropython/drivers/storage/sdcard/sdcard.py)
to the Pico alongside the main firmware file.

### Deploying to the Pico

```bash
# Copy firmware (requires mpremote or Thonny)
mpremote cp sdcard.py :
mpremote cp software_can_micro_python.py :main.py
```

Rename to `main.py` so it runs automatically on power-up.

---

## 12. Serial Menu Interface

Connect to the Pico's USB port.  The Pico appears as a USB-CDC virtual serial
port.  Open any terminal at **115200 baud** (baud rate is set at the host;
USB-CDC is not rate-limited).

**To open the menu:** press **Enter** in the terminal while the firmware is
running.

```
=== CAN Pressure Board ===
  1. Set RTC time
  2. Set CAN baud rate
  3. Resume acquisition
>
```

### Option 1 — Set RTC time

Sets the RP2040's internal real-time clock.  The clock is battery-backed only
if an external battery is connected to the Pico's RTC; otherwise time resets
on power-cycle.

```
Set RTC — enter UTC date/time:
  Year  (YYYY) : 2025
  Month (1-12) : 6
  Day   (1-31) : 1
  Hour  (0-23) : 09
  Min   (0-59) : 15
  Sec   (0-59) : 00
  RTC set → 2025-06-01 09:15:00
```

### Option 2 — Set CAN baud rate

Re-initialises the MCP2515 immediately with the new bit timing.

```
Select CAN baud rate:
  1. 125 kbps
  2. 250 kbps
  3. 500 kbps ←
  4. 1000 kbps (1 Mbps)
Choice [1-4]: 2
  CAN set to 250 kbps
```

---

## 13. MCP2515 Bit Timing

The MCP2515 oscillator is driven by GP7 at exactly **8 MHz** (RP2040 PWM,
50 % duty cycle).  Register values for each supported baud rate:

```
Bit time = TQ × N_TQ
TQ = 2 × (BRP + 1) / Fosc

Segment layout (8 TQ mode):
  SYNC  = 1 TQ  (fixed)
  PROP  = 1 TQ  (PRSEG  = 0 → 1 TQ)
  PS1   = 3 TQ  (PHSEG1 = 2 → 3 TQ)
  PS2   = 3 TQ  (PHSEG2 = 2 → 3 TQ)
  Total = 8 TQ
  Sample point = 62.5 % (after PS1)

CNF2 = 0x90  (BTLMODE=1, SAM=0, PHSEG1=2, PRSEG=0)
CNF3 = 0x02  (PHSEG2=2)
```

| Baud | BRP | TQ (ns) | CNF1 |
|---|---|---|---|
| 125 kbps | 3 | 1000 | 0x03 |
| 250 kbps | 1 | 500 | 0x01 |
| 500 kbps | 0 | 250 | 0x00 |
| 1 Mbps | 0 (4 TQ) | 250 | 0x00 |

For 1 Mbps, CNF2 = 0x80 (PS1=1 TQ) and CNF3 = 0x00 (PS2=1 TQ).

---

## 14. Repository Files

| File | Description |
|---|---|
| `software_can_micro_python.py` | Primary MicroPython firmware (deploy as `main.py`) |
| `software_can_circuit_python.py` | CircuitPython port (reference / alternative) |
| `requirements_functionality` | Original functional requirements specification |
| `CAN_adc.pdf` | Full schematic PDF (all sheets) |
| `CAN_adc.zip` | KiCad 9 project: schematics, PCB layout, Gerber files |
| `CANBUS_pressure_monitor.drawio` | System-level block diagram |

# CAN Pressure Board

Telemetry data acquisition board based on the **Raspberry Pi Pico (RP2040)**.
Reads thermocouples, analog inputs, and pressure sensors through a 16:1 multiplexer
and a 12-bit SPI ADC, then streams all 23 channels over CAN bus and logs to SD card.

---

## Hardware Overview

| Block | Part | Interface |
|---|---|---|
| Microcontroller | RP2040 (Raspberry Pi Pico) | — |
| CAN controller | MCP2515 | SPI1 |
| CAN transceiver | TJA1050 | — |
| ADC | ADC128S022 (8-ch, 12-bit) | SPI0 |
| Analog MUX | CD74HC4067 (16:1) | GPIO |
| SD card | Standard microSD | SPI0 |
| Thermocouple front-end | ×2 boards, 3 ch each | Analog → MUX |
| Differential pressure | Honeywell SSCDRRN005PDAA5 (±5 psi) | Analog → ADC |
| Pressure | 2-port sensor (0–150 psi) | Analog → ADC |

---

## Pin Assignments (RP2040)

| GPIO | Function |
|---|---|
| GP1 | SD card CS |
| GP2 | SPI0 SCK (ADC + SD) |
| GP3 | SPI0 MOSI |
| GP4 | SPI0 MISO |
| GP5 | ADC128S022 CS |
| GP7 | 8 MHz PWM clock → MCP2515 OSC1 |
| GP8 | SPI1 MISO (CAN) |
| GP9 | MCP2515 CS |
| GP10 | SPI1 SCK (CAN) |
| GP11 | SPI1 MOSI (CAN) |
| GP14 | CD74HC4067 enable (~E) |
| GP16 | MUX S0 |
| GP17 | MUX S1 |
| GP18 | MUX S2 |
| GP19 | MUX S3 |

> **Note:** The requirements document listed GPIO16/17 as UART pins. The schematic
> shows these are the MUX select lines. The menu/UART interface runs over the Pico's
> USB-CDC port (appears as a serial port on the host PC at 115200 baud).

---

## Channel Map (23 channels total)

ADC IN0 is routed through the CD74HC4067 MUX (16 sub-channels).
ADC IN1–7 are direct inputs.

| Index | Channel | Source | Default assignment |
|---|---|---|---|
| 0 | CH1 | MUX 0 | Temperature 1 |
| 1 | CH2 | MUX 1 | Temperature 2 |
| 2 | CH3 | MUX 2 | Temperature 3 |
| 3 | CH4 | MUX 3 | Temperature 4 |
| 4 | CH5 | MUX 4 | Temperature 5 |
| 5 | CH6 | MUX 5 | Temperature 6 |
| 6 | CH7 | MUX 6 | Temperature 7 |
| 7 | CH8 | MUX 7 | Temperature 8 |
| 8 | CH9 | MUX 8 | Analog In 1 |
| 9 | CH10 | MUX 9 | Analog In 2 |
| 10 | CH11 | MUX 10 | Analog In 3 |
| 11 | CH12 | MUX 11 | Analog In 4 |
| 12 | CH13 | MUX 12 | Analog In 5 |
| 13 | CH14 | MUX 13 | Analog In 6 |
| 14 | CH15 | MUX 14 | Analog In 7 |
| 15 | CH16 | MUX 15 | Analog In 8 |
| 16 | CH17 | ADC IN1 | Differential Pressure (0–15 psi) |
| 17 | CH18 | ADC IN2 | Pressure (0–150 psi) |
| 18 | CH19 | ADC IN3 | TBD |
| 19 | CH20 | ADC IN4 | TBD |
| 20 | CH21 | ADC IN5 | TBD |
| 21 | CH22 | ADC IN6 | TBD |
| 22 | CH23 | ADC IN7 | TBD |

All values are raw 12-bit ADC counts (0–4095). Scale factors are defined in
`channels.obd` on the SD card.

---

## CAN Bus

| Parameter | Value |
|---|---|
| Default baud rate | 500 kbps |
| Frame type | Standard (11-bit ID) |
| Base ID | 0x200 |
| Frames per cycle | 6 (0x200–0x205) |
| Transmit rate | 2 Hz |
| Bytes per frame | 8 (4 channels × 2 bytes, big-endian uint16) |

### Frame layout

| CAN ID | Bytes 0–1 | Bytes 2–3 | Bytes 4–5 | Bytes 6–7 |
|---|---|---|---|---|
| 0x200 | CH1 | CH2 | CH3 | CH4 |
| 0x201 | CH5 | CH6 | CH7 | CH8 |
| 0x202 | CH9 | CH10 | CH11 | CH12 |
| 0x203 | CH13 | CH14 | CH15 | CH16 |
| 0x204 | CH17 | CH18 | CH19 | CH20 |
| 0x205 | CH21 | CH22 | CH23 | 0x0000 |

---

## SD Card Logging

### `can_log.csv` — CAN message log (written at 2 Hz)

```
Time Stamp,ID,Extended,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8
2025-01-15 10:23:45,0x200,0,0,8,0,124,0,98,1,12,0,77
...
```

### `channels.obd` — Channel definition file (written once at startup)

One line per channel in the format:

```
CH1 = Temperature_1, C, -40, 200, CAN_ID=0x200, Byte=0, Len=2, Scale=1, BigEndian=1, Signed=0
```

---

## Firmware

File: `software_can_micro_python.py`
Runtime: **MicroPython** on RP2040

### Sampling

All 23 channels are sampled at **5 Hz** (200 ms cycle).
The ADC128S022 pipeline is primed with one dummy read each time the MUX channel changes.

### Defaults

| Parameter | Value |
|---|---|
| Sampling rate | 5 Hz |
| CAN transmit rate | 2 Hz |
| SD log rate | 2 Hz (one row per CAN frame) |
| CAN baud rate | 500 kbps |
| RTC | RP2040 internal RTC (set via menu) |

### Menu Interface

Press **Enter** in the USB serial terminal to open the menu:

```
=== CAN Pressure Board ===
  1. Set RTC time
  2. Set CAN baud rate
  3. Resume acquisition
```

**Set RTC time** — prompts for year/month/day/hour/min/sec (UTC).
Timestamps are stored by the RP2040's internal RTC and written to each log row.

**Set CAN baud rate** — choose 125 / 250 / 500 / 1000 kbps.
Re-initialises the MCP2515 with correct CNF1/CNF2/CNF3 registers for the 8 MHz oscillator.

### Required MicroPython libraries

- `sdcard` — community SD card driver
  (e.g. from [micropython/micropython-lib](https://github.com/micropython/micropython-lib))

Copy `sdcard.py` to the Pico alongside `software_can_micro_python.py`, or install via `mip`.

---

## Repository Files

| File | Description |
|---|---|
| `software_can_micro_python.py` | Main MicroPython firmware |
| `software_can_circuit_python.py` | CircuitPython port (reference) |
| `requirements_functionality` | Original functional requirements |
| `CAN_adc.pdf` | Full KiCad schematic |
| `CAN_adc.zip` | KiCad project files and Gerbers |
| `CANBUS_pressure_monitor.drawio` | System block diagram |

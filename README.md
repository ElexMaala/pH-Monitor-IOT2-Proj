# pH-Monitor-IOT2-Proj
A Raspberry Pi-based pH monitoring and automatic dosing system. Uses an ADS1115 ADC for pH sensing, peristaltic pumps for acid/base dosing, a TFT display, and a Flask + React web dashboard for live remote monitoring and control.
# pH Adjustment System

A Raspberry Pi-based pH monitoring and automatic dosing system, built for biological applications such as aquarium management and hydroponic plant irrigation. The system continuously measures pH using an ADS1115 ADC, automatically doses acid or base to reach a target pH, and exposes a live web dashboard for remote monitoring and control.

This repository contains my individual contribution to a group project: the pH calibration model, the ADS1115 hardware interface, the Flask/React web dashboard, and the 3D-printed enclosure design.

## Features

- **Piecewise linear pH calibration** using three standard buffer solutions (pH 4, 7, 10)
- **Adaptive dosing** that calibrates pump effectiveness at startup, adapting to solution concentration each session
- **Anti-overshoot control** using live pH rate estimation to stop dosing before the target is reached
- **TFT display** showing live pH, waveform history, dosing events, and system status
- **Web dashboard** (Flask + React) for live monitoring and remote control over the local network
- **Rotary encoder and touch sensor** for manual target pH adjustment and system locking

## Hardware

| Component | Purpose |
|---|---|
| Raspberry Pi 5 | Main controller |
| ADS1115 | Analog-to-digital converter for the pH probe |
| pH probe module | pH sensing |
| ST7735 TFT display (160x128) | Live status display |
| Rotary encoder + push button | Target pH adjustment |
| Capacitive touch sensor | Lock/unlock controls |
| 2x MOSFET drivers | Switch 12V dosing pumps from GPIO signals |
| 2x 12V peristaltic pumps | Acid/base dosing |
| RGB LED + buzzer | Status indication and audio feedback |
| 12V battery | Pump power supply |

### Pin Map

| Signal | GPIO |
|---|---|
| Rotary encoder CLK | 17 |
| Rotary encoder DT | 18 |
| Rotary encoder SW | 27 |
| Touch sensor | 22 |
| Buzzer | 23 |
| RGB LED (R/G/B) | 5 / 6 / 13 |
| Acid pump (MOSFET) | 19 |
| Base pump (MOSFET) | 26 |

ADS1115 and TFT display communicate over I2C and SPI respectively.

## Software Requirements

- Raspberry Pi OS (Bookworm or later recommended)
- Python 3.9+
- I2C and SPI enabled via `raspi-config`

### Python dependencies

```bash
pip3 install adafruit-circuitpython-ads1x15 RPi.GPIO Pillow flask
```

`board`, `busio`, and `ST7735` are provided through CircuitPython and the display driver library. Install per your display module's documentation if not already present.

## Calibration

Before running the main system, the pH probe must be calibrated against known buffer solutions.

1. Run the standalone voltage reader to find each buffer's voltage:

   ```bash
   python3 read_voltage.py
   ```

2. Submerge the probe in the pH 4, pH 7, and pH 10 buffer solutions one at a time, rinsing with distilled water between each. Record the stable voltage shown in the terminal for each.

3. Update the calibration constants at the top of `pHmonitor_v11.py`:

   ```python
   CAL_PH4_V  = 1.42436
   CAL_PH7_V  = 1.77000
   CAL_PH10_V = 2.08568
   ```
   Note:  Replace these with the values you recorded. `read_voltage.py` is a standalone tool only and is not part of the deployed system.

  

## Running

```bash
python3 pHmonitor.py
```

On startup, the system stabilises the sensor, calibrates pump sensitivity for both acid and base, then enters normal monitoring. The Raspberry Pi's local IP address is printed to the log and shown on the TFT display, used to access the web dashboard.

## Web Dashboard

Once running, the dashboard is available at:

```
http://<raspberry-pi-ip>:5000
```

From the dashboard you can view live pH, adjust the target pH, toggle motor and rotary locks, and view dose history.

## Repository Structure

```
.
├── pHmonitor.py     # Main system: state machine, sensing, dosing, display, dashboard
├── read_voltage.py      # Standalone tool used only during calibration
└── README.md
```

## Project Context

This system was developed as part of a group engineering project. My individual contributions were the pH calibration model, the ADS1115 hardware interface, the web dashboard, and the 3D-printed enclosure. The state machine, adaptive dosing algorithm, and TFT display rendering were developed by a teammate, and the physical hardware assembly and wiring by another.

## License

This project was developed for academic purposes.

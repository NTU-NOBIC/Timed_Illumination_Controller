# Timed Illumination Controller

Firmware for an **Arduino Teensy 4.0** that coordinates two LED-control outputs with camera acquisition. In timed acquisition, the Teensy generates an external camera trigger and monitors the camera's trigger-ready feedback. A second mode uses camera feedback to control LED illumination during externally controlled continuous streaming.

## Features

- **Timed acquisition:** programmable camera trigger intervals (`t1`–`t4`) and LED lead/lag intervals (`t5`–`t8`).
- **Streaming mode:** LED control based on the camera feedback input and configurable `ts` interval.
- **Two active-HIGH channels:** Channel A and Channel B, with firmware-enforced mutual exclusion.
- **Camera monitoring:** frame-event counting from the camera feedback signal (mode-dependent validation).
- **Serial interface:** status, timing, acquisition, streaming, manual LED control, and configuration commands at **115200 baud**.
- **EEPROM presets:** save/load timing, cycle count, and selected channel in slots **0–10**; slot **0** is read at startup.

## Firmware and connections

Main sketch: [`Timed_Cam_Trig_LED_Control.ino`](Timed_Cam_Trig_LED_Control.ino)

| Teensy 4.0 pin | Firmware name | Direction | Purpose |
|---|---|---|---|
| 9 | `CAM_TRIG_OUT` | Output | Active-HIGH camera external trigger |
| 10 | `CAM_TRIG_READY` | Input (`INPUT_PULLUP`) | Camera feedback |
| 13 (`LED_BUILTIN`) | `GATE_A` | Output | Active-HIGH Channel A gate (also onboard LED) |
| 14 | `GATE_B` | Output | Active-HIGH Channel B gate |

**Important:** Confirm interface voltage levels, signal polarity, camera feedback configuration, and external LED driver requirements before connecting hardware. The sketch's naming of its camera and signals does not establish compatibility with every camera configuration.
Note: Teensy 4.0 GPIO input and output logic voltage is 3.3V.

## Quick start

1. Open `Timed_Cam_Trig_LED_Control.ino` in an Arduino environment configured for **Teensy 4.0**.
2. Compile and upload the sketch to the board.
3. Open a serial terminal at **115200 baud** and send commands terminated by a newline.
4. Send `help` to display the implemented commands or `?` to view status and timing settings.
5. For timed acquisition, configure the camera for the expected external-trigger/ready-signal operation, then issue `r`. Enter `q` followed by Enter to abort the timed run.
6. For streaming mode, ensure the camera feedback input is configured for the exposure signal expected by the firmware, then issue `stm on`; use `stm off` to disable the mode.

See [**USER_GUIDE.md**](./firmware/USER_GUIDE.md) under firmware folder for full commands, parameter definitions, default values, operating procedures, and EEPROM details.

## Documentation scope

This README describes functionality present in the supplied `.ino` sketch. Channel names A/B are firmware labels; the sketch does **not** assign LED colours or fluorescence filter bands. Camera operation and image saving must be verified separately on the actual system.

## Disclaimer and licensing

The firmware is provided without any assurance of suitability or safe operation with a particular hardware configuration. Review and test signal timing and electrical interfaces before use.

**License:** No license grant has been inferred from the `.ino` source. Add an appropriate `LICENSE` file only after confirming the intended licensing terms with the code's rights holders.

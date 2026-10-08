# Demo UI User Guide

This guide explains how to install and use the Python demo UI for the Timed Illumination Controller. The UI provides serial control of the Teensy running `Timed_Cam_Trig_LED_Control.ino`.

## Recommended GitHub layout

Use the folder name `demo_ui` because it is short, descriptive, and follows common lowercase repository naming conventions.

```text
Timed_Illumination_Controller/
|-- README.md
|-- firmware/
|   |-- USER_GUIDE.md
|   `-- Timed_Cam_Trig_LED_Control.ino
`-- demo_ui/
    |-- Demo_UI_USER_GUIDE.md
    |-- Time_Illumination_GUI.py
    |-- requirements.txt
    `-- Connection_MM_setup.pdf
```

### Files to upload

- `Demo_UI_USER_GUIDE.md` - installation and operating instructions for the demo UI.
- `Time_Illumination_GUI.py` - the main Python/Tkinter application.
- `requirements.txt` - Python package dependency list. The current application requires `pyserial>=3.5,<4`.
- `Connection_MM_setup.pdf` - connection and Micro-Manager setup instructions opened by the UI's **Connection & Micro-Manager Setup** button.


## Requirements

- Windows computer (the UI uses Windows to open the bundled setup PDF).
- Python 3.10 or later recommended.
- Tkinter, normally included with the standard Windows Python installer.
- A Teensy programmed with `firmware/Timed_Cam_Trig_LED_Control.ino`.
- A USB data cable.
- Micro-Manager and the microscope hardware configuration described in `Connection_MM_setup.pdf`, when camera acquisition is required.

## Install from source

Open PowerShell in the repository root and run:

```powershell
cd demo_ui
py -m pip install --upgrade pip
py -m pip install -r requirements.txt
```

## Start the UI

```powershell
py Time_Illumination_GUI.py
```

## Connect to the controller

1. Upload the firmware to the Teensy and connect it to the computer by USB.
2. Close Arduino Serial Monitor and any other application using the Teensy's serial port.
3. Start the demo UI.
4. Select the Teensy's **COM port**. Use the refresh button if the port does not appear.
5. Click **Connect**. The UI connects at 115200 baud and reads the controller's current settings.
6. Confirm that the status reports `Connected to COM...` and that received messages appear in the serial log.

If more than one COM port is shown, disconnect and reconnect the Teensy and note which port disappears and returns.

## Main controls

### Timing settings

- `t1` to `t4` define the primary camera/illumination timing sequence in milliseconds.
- `t5` to `t8` define the alternating LED-channel timing sequence in milliseconds.
- `ts` is the expected Micro-Manager exposure time. It must be an integer of at least 10 ms.
- **Total Cycle** sets the controller cycle count. Accepted values are 0 through 2,147,483,647.

Edit a value and click the corresponding **Set** button, or press Enter in the field. A value is treated as applied only after the controller confirms it. Refer to the timing diagram in the UI and the firmware guide for the exact meaning of each interval.

### Illumination and operating mode

- **Illumination ON/OFF** manually controls illumination.
- **LED Channel A/B** selects an illumination channel.
- **Time Illumination** runs the programmed timed sequence.
- **Continuous Streaming** coordinates illumination with continuous camera streaming. Set `ts` to match the Micro-Manager camera exposure.
- **Start** begins a test run using the current controller settings; while a run is active, the same button stops it.

### Timing presets

The controller provides EEPROM preset slots `#0` through `#10`.

- **Save** overwrites the selected slot after confirmation.
- **Read** loads the selected slot and refreshes the displayed settings.
- Preset `#0` is the controller's power-on default.

Saving writes to the controller's non-volatile memory. Verify all timing values before overwriting a preset.

### Serial logger and command prompt

- **Help** requests the firmware command list.
- **Refresh Status** requests the current controller state.
- **Manual Command** sends a firmware serial command directly.
- **Clear Log** clears only the on-screen log; it does not change controller settings.

## Suggested demo procedure

1. Connect the Teensy and confirm the current values populate the UI.
2. Load a known EEPROM preset.
3. Select LED Channel A or B and briefly test **Illumination ON/OFF**.
4. Select the required operating mode.
5. Verify `t1` through `t8`, `ts`, and **Total Cycle** against the experiment plan.
6. For Micro-Manager acquisition, open **Connection & Micro-Manager Setup**, complete the hardware configuration, and set the camera exposure to the same value as `ts`.
7. Click **Start** and watch the status and serial log for controller errors.
8. Click **Stop** before disconnecting or changing hardware.

## Troubleshooting

### No COM port appears

- Confirm that the USB cable supports data, not charging only.
- Install the appropriate Teensy/USB driver if Windows does not recognize the board.
- Click refresh after reconnecting the controller.
- Check Windows Device Manager for the assigned COM port.

### Connection fails or the port is busy

- Close Arduino Serial Monitor, another copy of the demo UI, and other serial tools.
- Disconnect and reconnect the Teensy, refresh the list, and try again.
- Confirm that the firmware is running and uses 115200 baud.

### `PySerial is required`

Run this command using the same Python interpreter that starts the UI:

```powershell
python -m pip install -r requirements.txt
```

### The setup PDF does not open

Keep `Connection_MM_setup.pdf` in the `demo_ui` directory beside `Time_Illumination_GUI.py`. The UI locates the PDF relative to the Python script or packaged executable.

### Values do not update

- Check the serial log for `Error` or `Invalid` responses.
- Ensure values are non-negative integers; `ts` must be at least 10 ms.
- Use **Refresh Status** to read the controller state again.
- Confirm that the firmware version matches the UI's serial command protocol.

## Safety and shutdown

Before connecting illumination hardware, verify the driver voltage, current limit, channel wiring, and active logic levels. Use conservative timing values for the first test. Stop the sequence, turn illumination off, disconnect the UI, and then power down the external hardware.


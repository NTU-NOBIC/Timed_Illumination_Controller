# Timed Illumination Controller — User Guide

This guide documents the behaviour implemented in [`Timed_Cam_Trig_LED_Control.ino`](Timed_Cam_Trig_LED_Control.ino).

## 1. System overview

The **Teensy 4.0** controls two active-HIGH outputs (`GATE_A`, `GATE_B`) and uses a camera feedback input (`CAM_TRIG_READY`). Its own external-trigger output (`CAM_TRIG_OUT`) is used in **timed acquisition**. In **continuous streaming**, camera acquisition is controlled externally and the feedback input is treated as an exposure-activity signal.

The main loop writes the gate outputs so **only the selected channel can be HIGH**. The `gateA_on()` and `gateB_on()` functions also force the opposite gate LOW before turning on their selected gate.

## 2. Hardware signals

| Symbol | Teensy pin | Configuration | Code-defined function |
|---|---:|---|---|
| `CAM_TRIG_OUT` | 9 | Digital output, initially LOW | Active-HIGH signal to camera EXT TRIGGER input |
| `CAM_TRIG_READY` | 10 | `INPUT_PULLUP` | Trigger-ready feedback in timed mode; exposure feedback in streaming mode |
| `GATE_A` | 13 (`LED_BUILTIN`) | Digital output, initially LOW | Active-HIGH Channel A gate |
| `GATE_B` | 14 | Digital output, initially LOW | Active-HIGH Channel B gate |

**Electrical check:** Ensure the connected camera input/output and LED-driver interfaces are electrically compatible with Teensy 4.0 GPIO. Pin 13 also controls the onboard LED. The code does not define an external electrical schematic or a specific LED-driver circuit.

## 3. Serial connection

The code calls `Serial.begin(115200)`. Open a serial monitor at **115200 baud** and send each command as a line (newline-terminated). The exact available commands can also be listed using `help`.

### Command reference

| Command | Firmware action |
|---|---|
| `help` | Show command help |
| `?` | Show current status and timing settings |
| `ch?` | Show selected channel |
| `on?` or `off?` | Show Channel A / Channel B status |
| `A` | Select Channel A and deselect Channel B |
| `B` | Select Channel B and deselect Channel A |
| `on` | Enable LED output for currently selected channel |
| `off` | Disable LED output for currently selected channel |
| `t?` | Show current timing settings |
| `t a/b/c/d` | Set `t1/t2/t3/t4` in milliseconds |
| `p a/b/c/d` | Set `t5/t6/t7/t8` in milliseconds |
| `c?` | Display configured total cycles |
| `c N` | Set total acquisition cycles |
| `r` | Start timed exposure/LED acquisition |
| `q` | Abort a running timed acquisition when prompted (follow with Enter) |
| `stm?` | Show streaming mode status |
| `stm on` | Enable continuous streaming LED response |
| `stm off` | Disable continuous streaming mode and clear LED enables |
| `ts?` | Display `ts` in milliseconds |
| `ts N` | Set `ts` in milliseconds (requires at least 10 ms) |
| `s N` | Save configuration to EEPROM slot `N`, from 0 through 10 |
| `l N` | Load configuration from EEPROM slot `N`, from 0 through 10 |

Examples:

```text
help
?
A
on
off
t 10/15/40/100
p 0/0/0/0
c 100
r
```

Do not send commands intended for the normal command parser while a timed acquisition is running; the timed routine separately checks serial input for `q` to abort.

## 4. Timing model

The firmware stores timing in **microseconds** (`unsigned long`), while serial `t`, `p`, and `ts` values are entered in **milliseconds**.

### Compiled initial values

| Parameter | Initial value | Meaning |
|---|---:|---|
| `t1` | 10 ms | Channel A camera trigger HIGH interval |
| `t2` | 15 ms | Following interval after `t1` |
| `t3` | 40 ms | Channel B camera trigger HIGH interval |
| `t4` | 100 ms | Following interval after `t3` |
| `t5` | 15 µs | Channel A LED pre-trigger interval (displayed as 0 ms) |
| `t6` | 0 ms | Channel A LED post-trigger interval |
| `t7` | 15 µs | Channel B LED pre-trigger interval (displayed as 0 ms) |
| `t8` | 0 ms | Channel B LED post-trigger interval |
| `ts` | 10 ms | Streaming exposure timing setting |
| `cycle_Total` | 1000 | Number of requested timed cycles |

**Startup note:** `setup()` loads **EEPROM slot 0** after initialization, so stored settings can replace these compiled values. Do not assume the values above are active without checking `?` or `t?`.

The firmware displays these timing relationships:

```text
Cycle duration = t1 + t2 + t3 + t4
Channel A LED ON time = t5 + t1 + t6
Channel B LED ON time = t7 + t3 + t8
```

For two-channel timing, it calculates the waits as `t_wait1 = t2 - t6 - t7` and `t_wait2 = t4 - t8 - t5`. There are alternate wait expressions when `t1` or `t3` is zero, in which case the corresponding channel is skipped. Timing-setting commands validate some relationships between these parameters.

A zero-millisecond request for `t5` or `t7` is represented internally as **15 µs** when the corresponding channel's exposure interval is active, to accommodate MCU response time. Consult `t?` for how the sketch reports its current settings.

## 5. Timed acquisition (`r`)

In this mode the Teensy drives `CAM_TRIG_OUT` and monitors `CAM_TRIG_READY` as the camera's trigger-ready signal.

1. Configure the camera to respond to the external trigger and provide the trigger-ready feedback expected by the firmware.
2. Check the current timing settings using `t?`; adjust with `t ...`, `p ...`, and `c N` as needed.
3. Send `r` to run the requested cycles.
4. The firmware first waits for `CAM_TRIG_READY` to be HIGH, aborting after **15 seconds** if that condition does not occur.
5. During each cycle, a nonzero `t1` enables the Channel A trigger segment; a nonzero `t3` enables the Channel B trigger segment. The appropriate LED gate is switched according to the configured pre-/post-trigger timing.
6. Enter `q` followed by Enter to abort during the run, where the routine checks serial input.

### Feedback-based frame count

The timed-mode interrupt first captures a **falling edge** on `CAM_TRIG_READY`, then measures the LOW pulse until the **rising edge**. The frame counter increments only when the pulse duration lies within the mode's channel-specific bounds:

```text
Channel A: t1 <= pulse duration <= t1 + 17,000 µs
Channel B: t3 <= pulse duration <= t3 + 17,000 µs
```

These bounds come from `TRIG_RDY_AFTER_EXPO` (15,000 µs) plus a further 2,000 µs allowance. An out-of-range pulse produces an alert. **The count indicates accepted feedback pulses; it does not verify that image data were successfully saved.**

A full cycle can issue up to **two exposure triggers** (one for each channel), or one if one of the corresponding exposure intervals is zero. The sketch displays an expected image count derived from the cycle count and the enabled intervals.

## 6. Continuous streaming (`stm on`, `stm off`)

In streaming mode, the code describes the camera as using **internal triggering** with its output configured for **EXPOSURE** activity. Acquisition is controlled by the camera / Micro-Manager, not by the Teensy's timed `CAM_TRIG_OUT` sequence.

1. Select the required channel with `A` or `B`.
2. Configure a matching exposure time with `ts N` (minimum **10 ms**).
3. Configure the camera feedback interface as required by this mode.
4. Send `stm on` to arm the feedback interrupt.
5. The streaming interrupt increments `frameNo` on each detected **falling edge** and enables the selected channel.
6. The main loop refreshes a timer for **`ts + 100,000 µs`** after a detected falling edge. When the timer expires, the channel is disabled and a frame total is printed; the firmware subtracts one from the counted edges in that timeout handler.
7. Send `stm off` to disable the streaming interrupt and LED enables.

**Code-specific distinction:** streaming mode does **not** perform the timed mode's falling-to-rising pulse-width validation. Its edge count is not equivalent to confirmed saved camera frames. The comment near the streaming code mentions another timer allowance, but the executable `cam_ExpoTimer.begin(...)` call uses `ts + 100000` µs; that executable value is documented here.

## 7. Manual channel operation

Use `A` or `B` to select the channel, then `on` or `off` to control the selected LED:

```text
A
on
on?
off
B
on
on?
off
```

Only one gate is intended to be active at a time. `GATE_A` is also the Teensy's onboard LED pin.

## 8. EEPROM presets

The sketch reads and writes preset slots **0 through 10**. Each slot uses a 100-byte address stride (`address = slot * 100`). The stored data are:

- `t1`–`t8`
- `ts`
- `cycle_Total`
- selected channel (`ch`)

Use `s 0` to save the currently active configuration as the setting loaded at boot. Use `s 5` and `l 5`, for example, to save and recall a separate configuration.

```text
s 0
s 5
l 5
?
```

The code does not implement a preset name or signature, nor an explicit validity check before using values read from EEPROM. After first programming or changing hardware, inspect current settings before running acquisition.

## 9. Implementation cautions

- **Camera signals:** timed mode expects trigger-ready behaviour, whereas streaming mode expects exposure activity on the same input. Configure the camera for the mode being used.
- **Signal compatibility:** verify voltage, polarity and source/sink limits of every connected interface. The firmware itself supplies no verified wiring diagram or isolation circuit.
- **Serial responsiveness:** `r` executes a blocking acquisition routine; normal command handling resumes after it finishes or aborts.
- **Timing limitations:** software execution, interrupt latency, camera behaviour and peripheral response can affect actual timing. Validate waveforms before production use.
- **Frame accounting:** reported frames are based on camera signal events, not independently verified captured/saved image files.
- **Optical assignment:** the source labels only Channel A and Channel B. It does not identify LED colours, filters or fluorophores.
- **License:** this `.ino` source does not establish license terms. Do not assume a particular software license solely from this guide.

## 10. Source of truth

If documentation and behaviour disagree, inspect the exact version of [`Timed_Cam_Trig_LED_Control.ino`](Timed_Cam_Trig_LED_Control.ino) deployed on the Teensy. This guide reflects the supplied sketch rather than a separately verified hardware test.

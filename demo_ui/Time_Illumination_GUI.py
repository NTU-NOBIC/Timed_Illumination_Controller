# Tk/ttk serial controller for Timed_Cam_Trig_LED_Control.ino.
#  python -m pip install pyserial


from __future__ import annotations

import os
import queue
import re
import threading
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Keep the module importable so a useful GUI error can be shown.
    serial = None
    list_ports = None


BAUD_RATE = 115200
POLL_INTERVAL_MS = 50


class TimeIlluminationApp(tk.Tk):
    """Thread-safe ttk front end for the Teensy serial command interface."""

    def __init__(self) -> None:
        super().__init__()
        self.title("NOBIC/SCELSE/NTU - Time Illumination Controller")
        self.geometry("850x1100")
        self.minsize(850, 700)

        self.serial_port = None
        self.reader_thread: threading.Thread | None = None
        self.stop_reader = threading.Event()
        self.rx_queue: queue.Queue[tuple[int, str, str]] = queue.Queue()
        self.cpu_parse_buffer = ""
        self.cpu_stream_tail = ""
        self.write_lock = threading.Lock()
        self.connected = False
        self.connection_generation = 0

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="Disconnected")
        self.channel_var = tk.StringVar(value="A")
        self.led_states = {"A": "OFF", "B": "OFF"}
        self.led_status_var = tk.StringVar(value="")
        self.mode_var = tk.StringVar(value="")
        self.run_status_var = tk.StringVar(value="")
        self.test_running = False
        self.camera_wait_seconds = 0
        self.camera_countdown_after_id: str | None = None
        self.command_var = tk.StringVar()
        self.diagram_var = tk.StringVar()
        self.timing_vars = {
            name: tk.StringVar(value="—") for name in [
                "t1", "t2", "t3", "t4", "t5", "t6", "t7", "t8"
            ]
        }
        self.ts_var = tk.StringVar(value="—")
        self.cycles_var = tk.StringVar(value="—")
        self.confirmed_values = {
            name: "—" for name in [
                "t1", "t2", "t3", "t4", "t5", "t6", "t7", "t8", "ts", "cycles"
            ]
        }
        self.setting_retry_required: set[str] = set()
        self.pending_setting_group: str | None = None
        self.diagram_values = {
            name: "—" for name in ["t1", "t2", "t3", "t4", "t5", "t6", "t7", "t8"]
        }

        self.connected_widgets: list[tk.Widget] = []
        self._build_ui()
        for name, variable in self.timing_vars.items():
            variable.trace_add("write", lambda *_args, n=name: self._on_setting_changed(n))
        self.ts_var.trace_add("write", lambda *_args: self._on_setting_changed("ts"))
        self.cycles_var.trace_add(
            "write", lambda *_args: self._on_setting_changed("cycles")
        )
        self.mode_var.trace_add(
            "write", lambda *_args: self._on_mode_changed()
        )
        self._set_connected_state(False)
        self.refresh_ports()
        self._update_diagram()
        self._show_idle_status()
        self.after(POLL_INTERVAL_MS, self._process_rx_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        if serial is None:
            self.after(
                100,
                lambda: messagebox.showerror(
                    "Missing dependency",
                    "PySerial is required. Install it with:\n\npython -m pip install pyserial",
                ),
            )

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)

        connection = ttk.LabelFrame(self, text="Serial Connection", padding=10)
        connection.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))
        connection.columnconfigure(5, weight=1)
        ttk.Label(connection, text="COM port:").grid(row=0, column=0, padx=(0, 6))
        self.port_combo = ttk.Combobox(
            connection, textvariable=self.port_var, state="readonly", width=10
        )
        self.port_combo.grid(row=0, column=1, padx=(0, 6))
        self.refresh_button = ttk.Button(
            connection, text="↻", width=3, command=self.refresh_ports
        )
        self.refresh_button.grid(row=0, column=2, padx=(0, 6))
        self.connect_button = ttk.Button(
            connection, text="Connect", width=14, command=self.toggle_connection
        )
        self.connect_button.grid(row=0, column=3, padx=(0, 4))
        ttk.Separator(connection, orient="vertical").grid(
            row=0, column=4, sticky="ns", padx=(4, 4)
        )
        ttk.Label(
            connection,
            textvariable=self.connection_var,
            anchor="w",
            foreground="green",
        ).grid(
            row=0, column=5, sticky="w"
        )
        setup_button = ttk.Button(
            connection,
            text="Connection & Micro-Manager Setup",
            width=35,
            command=self._open_setup_pdf,
        )
        setup_button.grid(row=0, column=6, padx=(8, 6))
        ttk.Button(connection, text="Exit", width=8, command=self._on_close).grid(
            row=0, column=7
        )

        diagram_frame = ttk.LabelFrame(self, text="Timing Configuration", padding=8)
        diagram_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
        diagram_frame.columnconfigure(0, minsize=220)
        diagram_frame.columnconfigure(1, weight=1)
        ttk.Label(
            diagram_frame,
            textvariable=self.diagram_var,
            font=("Consolas", 10),
            anchor="center",
            justify="left",
        ).grid(row=0, column=1, sticky="ew")
        run_controls = ttk.Frame(diagram_frame)
        run_controls.grid(row=0, column=0, padx=(2, 2), pady=(15, 0))
        run_controls.columnconfigure(0, minsize=210)
        self.start_button = ttk.Button(
            run_controls, text="Start", width=12, command=self._toggle_test_run
        )
        # ttk.Button has no height option; ipady gives it an approximately
        # three-text-row height while preserving the native ttk appearance.
        self.start_button.grid(row=0, column=0, ipady=12, pady=(0, 1))
        ttk.Label(
            run_controls,
            textvariable=self.run_status_var,
            width=34,
            anchor="center",
            justify="center",
            foreground="green",
        ).grid(row=1, column=0)
        self.connected_widgets.append(self.start_button)

        controls = ttk.Frame(self, padding=(10, 5))
        controls.grid(row=2, column=0, sticky="ew")
        controls.columnconfigure(0, weight=1)
        controls.columnconfigure(1, weight=1)

        timings = ttk.LabelFrame(controls, text="Timing Settings (ms)", padding=10)
        timings.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        for column in range(9):
            timings.columnconfigure(column, weight=1 if column in (1, 3, 5, 7) else 0)

        primary_timing_entries = []
        for index, name in enumerate(("t1", "t2", "t3", "t4")):
            column = index * 2
            ttk.Label(timings, text=f"{name} :").grid(row=0, column=column, padx=(0, 3))
            entry = ttk.Entry(timings, textvariable=self.timing_vars[name], width=8)
            entry.grid(row=0, column=column + 1, sticky="w", padx=(0, 4))
            primary_timing_entries.append(entry)
            self.connected_widgets.append(entry)
        self.set_t_button = ttk.Button(
            timings, text="Set t1–t4", command=self.set_primary_timing
        )
        self.set_t_button.grid(row=0, column=8, sticky="ew")
        for entry in primary_timing_entries:
            self._bind_enter_to_button(entry, self.set_t_button)

        secondary_timing_entries = []
        for index, name in enumerate(("t5", "t6", "t7", "t8")):
            column = index * 2
            ttk.Label(timings, text=f"{name} :").grid(row=1, column=column, padx=(0, 3), pady=(8, 0))
            entry = ttk.Entry(timings, textvariable=self.timing_vars[name], width=8)
            entry.grid(row=1, column=column + 1, sticky="w", padx=(0, 4), pady=(8, 0))
            secondary_timing_entries.append(entry)
            self.connected_widgets.append(entry)
        self.set_p_button = ttk.Button(
            timings, text="Set t5–t8", command=self.set_secondary_timing
        )
        self.set_p_button.grid(row=1, column=8, sticky="w", pady=(8, 0))
        for entry in secondary_timing_entries:
            self._bind_enter_to_button(entry, self.set_p_button)

        miscellaneous = ttk.Frame(timings)
        miscellaneous.grid(row=2, column=0, columnspan=9, sticky="ew", pady=(10, 0))
        for column in range(8):
            miscellaneous.columnconfigure(column, weight=1 if column in (1, 4) else 0)
        ttk.Label(miscellaneous, text="ts :").grid(row=0, column=0, padx=(2, 2))
        ts_entry = ttk.Entry(miscellaneous, textvariable=self.ts_var, width=8)
        ts_entry.grid(row=0, column=1, sticky="w", padx=(0, 5))
        self.set_ts_button = ttk.Button(miscellaneous, text="Set ts", width=6, command=self.set_ts)
        self.set_ts_button.grid(row=0, column=1, padx=(0, 2))
        self._bind_enter_to_button(ts_entry, self.set_ts_button)
        ttk.Label(miscellaneous, text="Total Cycle :").grid(row=0, column=2, padx=(0, 3))
        cycles_entry = ttk.Entry(miscellaneous, textvariable=self.cycles_var, width=8)
        cycles_entry.grid(row=0, column=4, sticky="w", padx=(0, 5))
        self.set_cycles_button = ttk.Button(
            miscellaneous, text="Set Cycles", width=10, command=self.set_cycles
        )
        self.set_cycles_button.grid(row=0, column=4)
        self._bind_enter_to_button(cycles_entry, self.set_cycles_button)
        self.connected_widgets.extend([ts_entry, cycles_entry])

        actions = ttk.LabelFrame(controls, text="Device Controls", padding=10)
        actions.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        for column in range(4):
            actions.columnconfigure(column, weight=1)

        illumination_on_button = ttk.Button(
            actions,
            text="Illumination ON",
            command=lambda: self.send_command("on"),
        )
        illumination_on_button.grid(row=0, column=0, sticky="ew", padx=3)
        illumination_off_button = ttk.Button(
            actions,
            text="Illumination OFF",
            command=lambda: self.send_command("off"),
        )
        illumination_off_button.grid(row=0, column=1, sticky="ew", padx=3)
        ttk.Label(
            actions,
            textvariable=self.led_status_var,
            foreground="#008000",
            anchor="w",
        ).grid(row=0, column=2, columnspan=2, sticky="w", padx=(10, 0))
        self.connected_widgets.extend([illumination_on_button, illumination_off_button])

        ttk.Label(actions, text="Illumination : ").grid(row=1, column=0, sticky="e", pady=(10, 0))
        for column, channel in ((1, "A"), (2, "B")):
            radio = ttk.Radiobutton(
                actions,
                text=f"LED Channel {channel}",
                value=channel,
                variable=self.channel_var,
                command=lambda c=channel: self.send_command(c),
            )
            radio.grid(row=1, column=column, sticky="w", pady=(10, 0))
            self.connected_widgets.append(radio)

        ttk.Label(actions, text="Mode : ").grid(row=2, column=0, sticky="e", pady=(10, 0))
        mode_timed = ttk.Radiobutton(
            actions,
            text="Time Illumination",
            value="off",
            variable=self.mode_var,
            command=self._set_mode,
        )
        mode_timed.grid(row=2, column=1, sticky="w", pady=(10, 0))
        mode_stream = ttk.Radiobutton(
            actions,
            text="Continuous Streaming",
            value="on",
            variable=self.mode_var,
            command=self._set_mode,
        )
        mode_stream.grid(row=2, column=2, columnspan=2, sticky="e", pady=(10, 0))
        self.connected_widgets.extend([mode_timed, mode_stream])

        eeprom = ttk.LabelFrame(controls, text="Timing Presets (Preset #0 as MCU power-ON default)", padding=8)
        eeprom.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        ttk.Label(eeprom, text="Save :").grid(row=0, column=0, padx=(0, 5))
        ttk.Label(eeprom, text="Read :").grid(row=1, column=0, padx=(0, 5), pady=(5, 0))
        for slot in range(11):
            eeprom.columnconfigure(slot + 1, weight=1)
            save = ttk.Button(
                eeprom, text=f"#{slot}", width=6, command=lambda n=slot: self.save_eeprom(n)
            )
            save.grid(row=0, column=slot + 1, sticky="ew", padx=2)
            load = ttk.Button(
                eeprom, text=f"#{slot}", width=6, command=lambda n=slot: self.load_eeprom(n)
            )
            load.grid(row=1, column=slot + 1, sticky="ew", padx=2, pady=(5, 0))
            self.connected_widgets.extend([save, load])

        logger = ttk.LabelFrame(self, text="Serial Logger / Command Prompt", padding=8)
        logger.grid(row=3, column=0, sticky="nsew", padx=10, pady=(5, 10))
        logger.columnconfigure(0, weight=1)
        logger.rowconfigure(1, weight=1)
        command_bar = ttk.Frame(logger)
        command_bar.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        command_bar.columnconfigure(3, weight=1)
        ttk.Label(command_bar, text="Manual Command :").grid(
            row=0, column=0, padx=(0, 5)
        )
        self.command_entry = ttk.Entry(command_bar, textvariable=self.command_var, width=40)
        self.command_entry.grid(row=0, column=1, padx=(0, 5))
        self.command_entry.bind("<Return>", self._send_prompt)
        help_button = ttk.Button(
            command_bar,
            text="Help",
            width=8,
            command=lambda: self.send_command("help"),
        )
        help_button.grid(row=0, column=4, padx=(0, 5))
        refresh_status_button = ttk.Button(
            command_bar,
            text="Refresh Status",
            command=lambda: self.send_command("?"),
        )
        refresh_status_button.grid(row=0, column=5, padx=(0, 5))
        self.send_button = ttk.Button(
            command_bar, text="Send Command", width=18, command=self._send_prompt
        )
        self.send_button.grid(row=0, column=2)
        ttk.Button(command_bar, text="Clear Log", command=self.clear_log).grid(
            row=0, column=6
        )
        self.log = scrolledtext.ScrolledText(
            logger, wrap=tk.WORD, height=14, font=("Consolas", 10), state="disabled"
        )
        self.log.grid(row=1, column=0, columnspan=3, sticky="nsew")
        self.log.tag_configure("tx", foreground="#1f5fa8")
        self.log.tag_configure("rx", foreground="#177245")
        self.log.tag_configure("system", foreground="#8a4f00")
        self.log.tag_configure("error", foreground="#d00000")
        self.connected_widgets.extend(
            [self.command_entry, help_button, refresh_status_button, self.send_button]
        )

    @staticmethod
    def _bind_enter_to_button(entry: ttk.Entry, button: ttk.Button) -> None:
        """Make both Enter keys invoke a ttk button when it is enabled."""
        entry.bind("<Return>", lambda _event: button.invoke())
        entry.bind("<KP_Enter>", lambda _event: button.invoke())

    def _open_setup_pdf(self) -> None:
        """Open the connection and Micro-Manager setup guide."""
        pdf_path = Path(__file__).resolve().with_name("Connection_MM_setup.pdf")
        if not pdf_path.is_file():
            messagebox.showerror(
                "Setup guide not found",
                f"Could not find:\n\n{pdf_path}",
            )
            return
        try:
            os.startfile(str(pdf_path))
        except (AttributeError, OSError) as exc:
            messagebox.showerror(
                "Unable to open setup guide",
                f"Could not open:\n\n{pdf_path}\n\n{exc}",
            )

    def refresh_ports(self) -> None:
        """Refresh the serial-port dropdown while preserving the current choice."""
        old_selection = self.port_var.get()
        devices = [port.device for port in list_ports.comports()] if list_ports else []
        self.port_combo["values"] = devices
        if old_selection in devices:
            self.port_var.set(old_selection)
        elif devices:
            self.port_var.set(devices[0])
        else:
            self.port_var.set("")
        if not self.connected:
            self.connect_button.configure(state="normal" if devices and serial else "disabled")

    def toggle_connection(self) -> None:
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        port = self.port_var.get().strip()
        if not port or serial is None:
            messagebox.showwarning("Serial port", "Select an available COM port first.")
            return
        try:
            connection = serial.Serial(port=port, baudrate=BAUD_RATE, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            messagebox.showerror("Connection failed", f"Could not open {port}:\n\n{exc}")
            self.refresh_ports()
            return

        self.serial_port = connection
        self.connected = True
        self.connection_generation += 1
        generation = self.connection_generation
        self.cpu_parse_buffer = ""
        self.cpu_stream_tail = ""
        self.stop_reader.clear()
        self.reader_thread = threading.Thread(
            target=self._serial_reader,
            args=(connection, generation),
            name="TeensySerialReader",
            daemon=True,
        )
        self.reader_thread.start()
        self._set_connected_state(True)
        self.connection_var.set(f"Connected to {port} at {BAUD_RATE} baud")
        self._append_log("system", f"Connected to {port} at {BAUD_RATE} baud\n")
        # A short delay lets boards that reset on port-open finish their reset.
        self.after(250, lambda g=generation: self._query_all_if_current(g))

    def disconnect(self, *, reason: str | None = None) -> None:
        if not self.connected and self.serial_port is None:
            return
        self.connected = False
        self.connection_generation += 1
        self.stop_reader.set()
        self.cpu_parse_buffer = ""
        self.cpu_stream_tail = ""
        self.pending_setting_group = None
        self.setting_retry_required.clear()
        connection, self.serial_port = self.serial_port, None
        if connection is not None:
            try:
                connection.close()
            except (serial.SerialException, OSError):
                pass
        self._finish_test_run()
        self._set_connected_state(False)
        self.connection_var.set("Disconnected")
        self._append_log("system", f"{reason or 'Disconnected'}\n")
        self.refresh_ports()

    def _serial_reader(self, connection, generation: int) -> None:
        """Read available bytes immediately and forward raw text through a queue."""
        while not self.stop_reader.is_set() and generation == self.connection_generation:
            try:
                # Block for one byte when idle, then drain everything already waiting.
                # This avoids readline() delaying streaming text until CR/LF arrives.
                data = connection.read(max(connection.in_waiting, 1))
                if data:
                    self.rx_queue.put(
                        (generation, "data", data.decode("utf-8", errors="replace"))
                    )
            except (serial.SerialException, OSError) as exc:
                if not self.stop_reader.is_set():
                    self.rx_queue.put((generation, "error", str(exc)))
                break

    def _process_rx_queue(self) -> None:
        try:
            while True:
                generation, kind, payload = self.rx_queue.get_nowait()
                if generation != self.connection_generation:
                    continue
                if kind == "data":
                    self._append_log("rx", f"{payload}")
                    self._buffer_cpu_text(payload)
                elif kind == "error" and self.connected:
                    self.disconnect(reason=f"Serial connection lost: {payload}")
        except queue.Empty:
            pass
        self.after(POLL_INTERVAL_MS, self._process_rx_queue)

    def _buffer_cpu_text(self, text: str) -> None:
        """Parse complete CPU lines while leaving partial lines buffered."""
        # Timed-cycle progress has no newline until many frames have completed.
        # Inspect a short rolling raw tail so the UI can report progress at once.
        self.cpu_stream_tail = (self.cpu_stream_tail + text)[-128:]
        if self.test_running and re.search(
            r"(?:^|[\r\n])\s*0:\s*[.AB]", self.cpu_stream_tail
        ):
            self._set_camera_triggers_running()

        self.cpu_parse_buffer += text
        parts = re.split(r"[\r\n]+", self.cpu_parse_buffer)
        if self.cpu_parse_buffer.endswith(("\r", "\n")):
            complete_lines = parts[:-1]
            self.cpu_parse_buffer = ""
        else:
            complete_lines = parts[:-1]
            self.cpu_parse_buffer = parts[-1]

        for line in complete_lines:
            if line:
                self._parse_cpu_text(line)

    def send_command(self, command: str) -> bool:
        command = command.strip()
        if not command:
            return False
        connection = self.serial_port
        if not self.connected or connection is None:
            messagebox.showwarning("Not connected", "Connect to the Teensy before sending commands.")
            return False
        self._append_log("tx", f"Sent command >> '{command}'")
        try:
            with self.write_lock:
                connection.write((command + "\n").encode("utf-8"))
                connection.flush()
            return True
        except (serial.SerialException, OSError) as exc:
            self.disconnect(reason=f"Serial write failed: {exc}")
            return False

    def _send_prompt(self, _event=None) -> None:
        command = self.command_var.get()
        if self.send_command(command):
            self.command_var.set("")

    def _toggle_test_run(self) -> None:
        """Start a timed run, or request an abort while one is active."""
        if self.test_running:
            if self.send_command("q"):
                # Prevent duplicate abort requests while awaiting the CPU response.
                self.start_button.configure(state="disabled")
            return

        if self.send_command("r"):
            self._begin_test_run()

    def _begin_test_run(self) -> None:
        self.test_running = True
        self.start_button.configure(text="Abort", state="normal")
        self.camera_wait_seconds = 15
        self._cancel_camera_countdown()
        self._update_camera_countdown()

    def _update_camera_countdown(self) -> None:
        if not self.test_running:
            return
        self.run_status_var.set(f"Waiting for camera ({self.camera_wait_seconds}s)")
        if self.camera_wait_seconds > 0:
            self.camera_wait_seconds -= 1
            self.camera_countdown_after_id = self.after(
                1000, self._update_camera_countdown
            )
        else:
            self.camera_countdown_after_id = None

    def _set_camera_triggers_running(self) -> None:
        if self.test_running:
            self._cancel_camera_countdown()
            self.run_status_var.set("Camera triggers running...")

    def _show_idle_status(self) -> None:
        if self.mode_var.get() == "on":
            exposure = self.confirmed_values.get("ts", "—")
            self.run_status_var.set(f"Expected µ-Mgr Exposure: {exposure}ms")
        else:
            try:
                total_cycles = int(self.confirmed_values["cycles"])
                point_count = str(total_cycles * 2)
            except (KeyError, TypeError, ValueError):
                point_count = "—"
            self.run_status_var.set(f"Expected µ-Mgr Point Count: {point_count}")

    def _finish_test_run(self) -> None:
        self._cancel_camera_countdown()
        self.test_running = False
        self._show_idle_status()
        if hasattr(self, "start_button"):
            self.start_button.configure(text="Start")
            self._refresh_start_button_state()

    def _refresh_start_button_state(self) -> None:
        if not hasattr(self, "start_button"):
            return
        enabled = self.connected and (self.test_running or self.mode_var.get() != "on")
        self.start_button.configure(state="normal" if enabled else "disabled")

    def _on_mode_changed(self) -> None:
        self._update_diagram()
        self._refresh_start_button_state()
        if not self.test_running:
            self._show_idle_status()

    def _cancel_camera_countdown(self) -> None:
        if self.camera_countdown_after_id is not None:
            try:
                self.after_cancel(self.camera_countdown_after_id)
            except tk.TclError:
                pass
            self.camera_countdown_after_id = None

    def _query_all_if_current(self, generation: int) -> None:
        if self.connected and generation == self.connection_generation:
            for index, command in enumerate(
                ("t?", "ts?", "c?", "ch?", "on?", "stm?")
            ):
                self.after(index * 80, lambda c=command: self.send_command(c))

    def _parse_cpu_text(self, text: str) -> None:
        """Extract firmware settings from individual or partial response lines."""
        if self.pending_setting_group and re.search(
            r"\b(?:Invalid|Incorrect|Missing)\b", text, re.IGNORECASE
        ):
            self.setting_retry_required.add(self.pending_setting_group)
            self.pending_setting_group = None
            self._refresh_set_button_states()

        if "Timed Exposure cycles, waiting for camera" in text and not self.test_running:
            self._begin_test_run()

        if self.test_running and re.search(r"(?:^|\n)\s*0:\s*[.AB]", text):
            self._set_camera_triggers_running()

        if self.test_running and re.search(
            r"Total cycles completed|User aborted|Test Aborted|"
            r"Time out while waiting for Camera",
            text,
            re.IGNORECASE,
        ):
            self._finish_test_run()

        for name, value in re.findall(r"\b(t[1-8])\s*:\s*(\d+)\s*ms", text, re.IGNORECASE):
            self._apply_device_value(name.lower(), value)

        ts_match = re.search(r"\bts\s*(?:=|:)\s*(\d+)\s*ms", text, re.IGNORECASE)
        if ts_match:
            self._apply_device_value("ts", ts_match.group(1))

        cycles_match = re.search(r"\bTotal Cycles?\s*(?:is set to|:)\s*(\d+)", text, re.IGNORECASE)
        if cycles_match:
            self._apply_device_value("cycles", cycles_match.group(1))

        channel_match = re.search(
            r"(?:Active Channel\s*:|Current active channel\s*:)\s*(?:ch[- ]?)?([AB])"
            r"|Channel\s+([AB])\s+set as active channel",
            text,
            re.IGNORECASE,
        )
        if channel_match:
            self.channel_var.set((channel_match.group(1) or channel_match.group(2)).upper())

        led_matches = re.findall(
            r"\bChannel\s+([AB])(?:\s*\([^)]*\))?\s*:\s*(ON|OFF)\b",
            text,
            re.IGNORECASE,
        )
        for channel, state in led_matches:
            self.led_states[channel.upper()] = state.upper()

        for channel in re.findall(
            r"\bChann(?:el|al)\s+([AB])\s+set to off\b", text, re.IGNORECASE
        ):
            self.led_states[channel.upper()] = "OFF"

        if led_matches or re.search(
            r"\bChann(?:el|al)\s+[AB]\s+set to off\b", text, re.IGNORECASE
        ):
            self._update_led_status()

        mode_match = re.search(
            r"\bStreaming mode\s*:?\s*(ON|OFF)\b", text, re.IGNORECASE
        )
        if mode_match:
            self.mode_var.set(mode_match.group(1).lower())

        # Direct acknowledgement from the short ts query/set response.
        ts_set_match = re.search(r"\bts is set to\s+(\d+)\s*ms", text, re.IGNORECASE)
        if ts_set_match:
            self._apply_device_value("ts", ts_set_match.group(1))

    def _set_mode(self) -> None:
        """Send the command represented by the selected mode radio button."""
        mode = self.mode_var.get()
        if mode in ("on", "off"):
            self.send_command(f"stm {mode}")

    def _update_led_status(self) -> None:
        self.led_status_var.set(
            f"Ch-A {self.led_states['A']} | Ch-B {self.led_states['B']}"
        )

    def _apply_device_value(self, name: str, value: str) -> None:
        """Apply a CPU-confirmed value to the editor and, for timing values, diagram."""
        acknowledgement_names = {
            "primary": "t1",
            "secondary": "t5",
            "ts": "ts",
            "cycles": "cycles",
        }
        pending_group = self.pending_setting_group
        if pending_group and acknowledgement_names[pending_group] == name:
            self.setting_retry_required.discard(pending_group)
            self.pending_setting_group = None

        value = str(value)
        self.confirmed_values[name] = value
        if name in self.timing_vars:
            self.timing_vars[name].set(value)
            self.diagram_values[name] = value
            self._update_diagram()
        elif name == "ts":
            self.ts_var.set(value)
            self._update_diagram()
            if not self.test_running:
                self._show_idle_status()
        elif name == "cycles":
            self.cycles_var.set(value)
            if not self.test_running:
                self._show_idle_status()
        self._refresh_set_button_states()

    def _on_setting_changed(self, name: str) -> None:
        """Enable only the Set button whose entry group differs from device state."""
        self._refresh_set_button_states()

    def _refresh_set_button_states(self) -> None:
        if not hasattr(self, "set_t_button"):
            return

        current = {name: variable.get().strip() for name, variable in self.timing_vars.items()}
        current["ts"] = self.ts_var.get().strip()
        current["cycles"] = self.cycles_var.get().strip()

        groups = (
            ("primary", self.set_t_button, ("t1", "t2", "t3", "t4")),
            ("secondary", self.set_p_button, ("t5", "t6", "t7", "t8")),
            ("ts", self.set_ts_button, ("ts",)),
            ("cycles", self.set_cycles_button, ("cycles",)),
        )
        for group, button, names in groups:
            changed = any(current[name] != self.confirmed_values[name] for name in names)
            needs_retry = group in self.setting_retry_required
            button.configure(
                state="normal" if self.connected and (changed or needs_retry) else "disabled"
            )

    def _validated_values(self, names: tuple[str, ...]) -> list[int] | None:
        values: list[int] = []
        for name in names:
            raw = self.timing_vars[name].get().strip()
            try:
                value = int(raw)
                if value < 0:
                    raise ValueError
            except ValueError:
                messagebox.showerror("Invalid timing", f"{name} must be a non-negative integer in ms.")
                return None
            values.append(value)
        return values

    def set_primary_timing(self) -> None:
        values = self._validated_values(("t1", "t2", "t3", "t4"))
        if values is not None:
            if self.send_command("t " + "/".join(map(str, values))):
                self.pending_setting_group = "primary"

    def set_secondary_timing(self) -> None:
        values = self._validated_values(("t5", "t6", "t7", "t8"))
        if values is not None:
            if self.send_command("p " + "/".join(map(str, values))):
                self.pending_setting_group = "secondary"

    def set_ts(self) -> None:
        try:
            value = int(self.ts_var.get().strip())
            if value < 10:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid timing", "ts must be an integer of at least 10 ms.")
            return
        if self.send_command(f"ts {value}"):
            self.pending_setting_group = "ts"

    def set_cycles(self) -> None:
        try:
            value = int(self.cycles_var.get().strip())
            if not 0 <= value <= 2_147_483_647:
                raise ValueError
        except ValueError:
            messagebox.showerror(
                "Invalid cycle count", "Total Cycle must be an integer from 0 to 2,147,483,647."
            )
            return
        if self.send_command(f"c {value}"):
            self.pending_setting_group = "cycles"

    def save_eeprom(self, slot: int) -> None:
        if messagebox.askyesno(
            "Save EEPROM preset",
            f"Overwrite EEPROM preset #{slot} with the current device settings?",
        ):
            self.send_command(f"s {slot}")

    def load_eeprom(self, slot: int) -> None:
        if self.send_command(f"l {slot}"):
            generation = self.connection_generation
            self.after(300, lambda g=generation: self._query_all_if_current(g))

    def _set_connected_state(self, connected: bool) -> None:
        state = "normal" if connected else "disabled"
        for widget in self.connected_widgets:
            widget.configure(state=state)
        if connected:
            self._update_led_status()
        else:
            self.led_status_var.set("")
        self.port_combo.configure(state="disabled" if connected else "readonly")
        self.refresh_button.configure(state="disabled" if connected else "normal")
        self.connect_button.configure(text="Disconnect" if connected else "Connect")
        self._refresh_set_button_states()
        self._refresh_start_button_state()

    def _update_diagram(self, *_args) -> None:
        if self.mode_var.get() == "on":
            # Show only the value confirmed by the Teensy. Editing the ts entry
            # must not change the diagram before the "Set ts" acknowledgement.
            ts_value = str(self.confirmed_values.get("ts", "—")).strip() or "—"
            self.diagram_var.set(
                "         Frame n             Frame (n+1)\n"
                "       +----------+          +----------+\n"
                "       |    ts    |          |    ts    |\n"
                f"       | {ts_value:>4} ms  |          | {ts_value:>4} ms  |\n"
                "   ____|          |__________|          |_______________________________\n"
                "  ----------- LED Ch-A/B LED ON  -------><-- LED Ch-A and Ch-B OFF ---\n"
                "  • Continuous Streaming mode with camera exposure controled by Micro-Manager\n"
                "  • Set ts to match Micro-Manager exposorue for proper automatic LED ON/OFF control"
            )
            return

        value = lambda name: self.diagram_values[name]
        self.diagram_var.set(
            "           Frame n                        Frame (n+1)\n"
            "       +-----------+                  +---------------+                         +---\n"
            "       |    t1     |       t2         |      t3       |          t4             |\n"
            f"       | {value('t1'):>6} ms | {value('t2'):>8} ms      | {value('t3'):>7} ms    | {value('t4'):>10} ms           |\n"
            "  _____|           |__________________|               |_________________________|\n"
            "  <--- LED  Ch-A ON  --->  ^    <----- LED Ch-B ON  ------->     ^         <-----\n"
            f"  < t5 >           < t6 >  |    < t7 >                < t8 >     |         < t5 >\n"
            "                t_wait1 ---+                          t_wait2 ---+"
        )

    def _append_log(self, tag: str, text: str) -> None:
        # RX chunks are inserted verbatim so continuous Teensy progress output
        # is not interrupted by a timestamp on every serial-read timeout.
        if tag == "rx":
            prefix = ""
        else:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            prefix = f"\n[{timestamp}] "
        display_tag = (
            "error"
            if re.search(r"\b(?:Error|Invalid)\b", text, re.IGNORECASE)
            else tag
        )
        self.log.configure(state="normal")
        self.log.insert(tk.END, prefix + text, display_tag)
        self.log.see(tk.END)
        self.log.configure(state="disabled")

    def clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", tk.END)
        self.log.configure(state="disabled")

    def _on_close(self) -> None:
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    TimeIlluminationApp().mainloop()

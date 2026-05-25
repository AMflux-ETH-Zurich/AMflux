import tkinter as tk
from tkinter import ttk

import serial


SERIAL_FIELDS = ("p", "i", "d", "pos", "switch")
SERIAL_POLL_INTERVAL_MS = 250
RAW_CHANGE_THRESHOLD = 4
POSITION_PARAM_NAME = "Target position"
PID_GAIN_RANGES = {
    "p": ("Position controller P gain", 0, 10_000_000),
    "i": ("Position controller I gain", 0, 100_000_000),
    "d": ("Position controller D gain", 0, 300_000),
}
PARAM_TO_SERIAL_FIELD = {
    param_name: field
    for field, (param_name, _min_gain, _max_gain) in PID_GAIN_RANGES.items()
}
PARAM_TO_SERIAL_FIELD[POSITION_PARAM_NAME] = "pos"


def normalize_raw_value(raw, min_raw, max_raw, min_value, max_value, invert=False):
    raw = max(min_raw, min(max_raw, raw))
    if invert:
        raw = max_raw - (raw - min_raw)
    return int((raw - min_raw) / (max_raw - min_raw) * (max_value - min_value) + min_value)


def parse_serial_pid_line(line):
    values = [part.strip() for part in line.strip("()[]").split(",")]
    if len(values) != len(SERIAL_FIELDS):
        raise ValueError(f"expected {len(SERIAL_FIELDS)} values, got {len(values)}")

    return {
        field: int(value)
        for field, value in zip(SERIAL_FIELDS, values)
    }


def read_serial_pid_samples(ser, text_buffer=None):
    try:
        waiting = ser.in_waiting
        if not waiting:
            return []

        text = ser.read(waiting).decode("utf-8", errors="ignore")
        if text_buffer is not None:
            text = text_buffer["value"] + text

        parts = text.split("\n")
        if text_buffer is not None:
            text_buffer["value"] = "" if text.endswith("\n") else parts[-1]

        complete_lines = parts if text.endswith("\n") else parts[:-1]
        complete_lines = [line.strip() for line in complete_lines if line.strip()]
        if not complete_lines:
            return []

        samples = []
        for line in complete_lines:
            try:
                samples.append(parse_serial_pid_line(line))
            except ValueError:
                continue

        if not samples:
            print(f"Serial parse error: no valid PID line in {complete_lines!r}")
        return samples
    except Exception as exc:
        print(f"Serial read error: {exc}")
        return []


def read_latest_serial_pid_values(ser):
    samples = read_serial_pid_samples(ser)
    if not samples:
        return None
    return samples[-1]


read_serial_pid_values = read_latest_serial_pid_values


def normalize_pid_values(raw_values):
    normalized = {
        param_name: normalize_raw_value(
            raw_values[field],
            min_raw=0,
            max_raw=1023,
            min_value=min_gain,
            max_value=max_gain,
            invert=True,
        )
        for field, (param_name, min_gain, max_gain) in PID_GAIN_RANGES.items()
    }
    normalized[POSITION_PARAM_NAME] = max(0, min(32768, raw_values["pos"]))
    normalized["switch"] = raw_values["switch"]
    return normalized


def build_serial_pid_panel(app, parent):
    tk.Label(parent, text="Port:").grid(row=0, column=0, padx=5)
    port_var = tk.StringVar(value="/dev/ttyUSB0")
    ttk.Entry(parent, textvariable=port_var, width=10).grid(row=0, column=1)

    tk.Label(parent, text="Baud:").grid(row=0, column=2, padx=5)
    baud_var = tk.StringVar(value="115200")
    ttk.Entry(parent, textvariable=baud_var, width=8).grid(row=0, column=3)

    connected = app.serial_connection is not None and app.serial_connection.is_open
    status_var = tk.StringVar(value="Connected" if connected else "Disconnected")
    tk.Label(parent, textvariable=status_var, fg="gray").grid(row=1, column=0, columnspan=4)

    tk.Label(parent, text="Target:").grid(row=2, column=0, padx=5)
    target_position_var = tk.StringVar(value="10000")
    ttk.Entry(parent, textvariable=target_position_var, width=10).grid(row=2, column=1)

    poll_running = {"active": False}
    serial_text_buffer = {"value": ""}
    last_raw_values = {}
    last_switch_state = {"value": None}
    next_target_sign = {"value": 1}

    def request_target_position():
        try:
            target_magnitude = abs(int(target_position_var.get(), 0))
        except ValueError:
            status_var.set(f"Invalid target: {target_position_var.get()}")
            return
        target_position = next_target_sign["value"] * target_magnitude
        app.organiser.request_update_param("Target position", target_position)
        status_var.set(f"Target position: {target_position}")
        next_target_sign["value"] *= -1

    def poll_serial():
        if not parent.winfo_exists():
            poll_running["active"] = False
            return

        poll_running["active"] = True
        print("poll_serial: start")
        try:
            if not app.serial_connection or not app.serial_connection.is_open:
                print("poll_serial: serial not connected")
                return

            print("poll_serial: reading serial")
            samples = read_serial_pid_samples(app.serial_connection, serial_text_buffer)
            print(f"poll_serial: received {len(samples)} valid sample(s)")
            if app.organiser is None or not samples:
                return

            switch_pressed = False
            for sample in samples:
                if last_switch_state["value"] == 1 and sample["switch"] == 0:
                    print("switch pressed")
                    switch_pressed = True
                last_switch_state["value"] = sample["switch"]

            if switch_pressed:
                print("poll_serial: queueing switch target")
                request_target_position()

            raw_values = samples[-1]
            values = normalize_pid_values(raw_values)
            for param_name, value in values.items():
                if param_name == "switch":
                    continue

                serial_field = PARAM_TO_SERIAL_FIELD[param_name]
                previous_raw_value = last_raw_values.get(serial_field)
                if (
                    previous_raw_value is not None
                    and abs(previous_raw_value - raw_values[serial_field]) < RAW_CHANGE_THRESHOLD
                ):
                    print(f"poll_serial: {serial_field} difference too small")
                    continue

                print(f"poll_serial: queueing {param_name} = {value}")
                app.organiser.request_update_param(param_name, value)
                last_raw_values[serial_field] = raw_values[serial_field]
        except Exception as exc:
            print(f"poll_serial: failed: {exc}")
            status_var.set(f"Serial poll error: {exc}")
        finally:
            if parent.winfo_exists():
                print("poll_serial: scheduling next callback")
                parent.after(SERIAL_POLL_INTERVAL_MS, poll_serial)
            else:
                poll_running["active"] = False

    def connect():
        try:
            app.close_serial_connection()
            app.serial_connection = serial.Serial(
                port_var.get(),
                int(baud_var.get()),
                timeout=0.05,
            )
            app.serial_connection.reset_input_buffer()
            serial_text_buffer["value"] = ""
            last_switch_state["value"] = None
            status_var.set(f"Connected: {port_var.get()}")
            if not poll_running["active"]:
                poll_serial()
        except Exception as exc:
            status_var.set(f"Error: {exc}")

    def disconnect():
        app.close_serial_connection()
        status_var.set("Disconnected")

    ttk.Button(parent, text="Connect", command=connect).grid(row=0, column=4, padx=5)
    ttk.Button(parent, text="Disconnect", command=disconnect).grid(row=0, column=5)

    if connected:
        poll_serial()


build_serial_position_panel = build_serial_pid_panel

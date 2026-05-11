import tkinter as tk
from tkinter import ttk

import serial


SERIAL_FIELDS = ("p", "i", "d", "switch")
PID_GAIN_RANGES = {
    "p": ("Position controller P gain", 0, 10_000_000),
    "i": ("Position controller I gain", 0, 100_000_000),
    "d": ("Position controller D gain", 0, 300_000),
}


def normalize_raw_value(raw, min_raw, max_raw, min_value, max_value):
    raw = max(min_raw, min(max_raw, raw))
    return int((raw - min_raw) / (max_raw - min_raw) * (max_value - min_value) + min_value)


def read_serial_pid_values(ser):
    try:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            return None

        values = [part.strip() for part in line.strip("()[]").split(",")]
        if len(values) != len(SERIAL_FIELDS):
            raise ValueError(f"expected {len(SERIAL_FIELDS)} values, got {len(values)}")

        return {
            field: int(value)
            for field, value in zip(SERIAL_FIELDS, values)
        }
    except ValueError as exc:
        print(f"Serial parse error for {line!r}: {exc}")
        return None
    except Exception as exc:
        print(f"Serial read error: {exc}")
        return None


def normalize_pid_values(raw_values):
    normalized = {
        param_name: normalize_raw_value(
            raw_values[field],
            min_raw=0,
            max_raw=1023,
            min_value=min_gain,
            max_value=max_gain,
        )
        for field, (param_name, min_gain, max_gain) in PID_GAIN_RANGES.items()
    }
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
    poll_running = {"active": False}
    last_values = {}

    def poll_serial():
        if not parent.winfo_exists():
            poll_running["active"] = False
            return

        poll_running["active"] = True
        if app.serial_connection and app.serial_connection.is_open:
            raw_values = read_serial_pid_values(app.serial_connection)
            if app.drive is not None and raw_values is not None:
                values = normalize_pid_values(raw_values)
                for param_name, value in values.items():
                    if param_name == "switch" or last_values.get(param_name) == value:
                        continue
                    app.drive.request_update_param(param_name, value)
                    last_values[param_name] = value
        parent.after(50, poll_serial)

    def connect():
        try:
            app.close_serial_connection()
            app.serial_connection = serial.Serial(
                port_var.get(),
                int(baud_var.get()),
                timeout=0.05,
            )
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

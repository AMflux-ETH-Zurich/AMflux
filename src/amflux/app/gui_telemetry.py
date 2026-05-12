from collections import deque
import math
import time
import tkinter as tk

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class MotorTelemetryPanel:
    PLOT_SIGNALS = (
        {"key": "torque", "label": "Torque", "unit": "Nm", "color": "wheat"},
        {"key": "velocity", "label": "Velocity", "unit": "RPM", "color": "lightblue"},
    )

    def __init__(self, parent, organiser, update_interval_ms=50, max_points=100, display_window=10):
        self.parent = parent
        self.organiser = organiser
        self.update_interval_ms = update_interval_ms
        self.display_window = display_window
        self.start_time = time.time()
        self.time_data = deque(maxlen=max_points)
        self.signal_data = {
            spec["key"]: deque(maxlen=max_points)
            for spec in self.PLOT_SIGNALS
        }
        self.plot_widgets = {}

        self.parent.grid_rowconfigure(0, weight=0)
        self.parent.grid_columnconfigure(0, weight=0)
        self.parent.grid_columnconfigure(1, weight=0)

        self._build_plot_panel()
        self._build_position_panel()
        self.update()

    def _build_plot_panel(self):
        self.figure = Figure(figsize=(4, 2.5), dpi=100)

        for index, spec in enumerate(self.PLOT_SIGNALS, start=1):
            axis = self.figure.add_subplot(1, len(self.PLOT_SIGNALS), index)
            line, = axis.plot([], [], "k")
            axis.set_title(f"{spec['label']} vs Time")
            axis.set_ylabel(f"{spec['label']} ({spec['unit']})")
            axis.set_xlabel("Time (s)")
            readback = axis.text(
                0.05,
                0.95,
                "",
                transform=axis.transAxes,
                fontsize=10,
                verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor=spec["color"], alpha=0.5),
            )
            self.plot_widgets[spec["key"]] = {
                "axis": axis,
                "line": line,
                "readback": readback,
                "spec": spec,
            }

        self.plot_canvas = FigureCanvasTkAgg(self.figure, master=self.parent)
        self.plot_canvas.get_tk_widget().grid(row=0, column=0)

    def _build_position_panel(self):
        self.position_canvas = tk.Canvas(self.parent, width=250, height=250, bg="white")
        self.position_canvas.grid(row=0, column=1)

        self.position_center = (125, 125)
        self.position_radius = 75
        self.position_canvas.create_oval(45, 45, 205, 205, outline="black", width=2)
        self.position_dot = self.position_canvas.create_oval(85, 25, 105, 35, fill="black")
        self.position_text = self.position_canvas.create_text(
            100,
            100,
            text="Position:\n-- rad",
            font=("Arial", 10),
            fill="black",
        )

    def read_telemetry(self):
        values = {"torque": None, "velocity": None, "position": None}
        if self.organiser is None:
            return values

        telemetry = self.organiser.get_status()
        if telemetry is None:
            return values

        for key, index in (("torque", 0), ("velocity", 1), ("position", 2)):
            if len(telemetry) > index:
                values[key] = telemetry[index]
        return values

    def _update_plot(self, elapsed_time, values):
        self.time_data.append(elapsed_time)

        for key, widgets in self.plot_widgets.items():
            value = values[key]
            self.signal_data[key].append(float("nan") if value is None else value)

            widgets["line"].set_data(self.time_data, self.signal_data[key])
            spec = widgets["spec"]
            widgets["readback"].set_text(format_readback(spec["label"], value, spec["unit"]))

            axis = widgets["axis"]
            if elapsed_time < self.display_window:
                axis.set_xlim(0, self.display_window)
            else:
                axis.set_xlim(elapsed_time - self.display_window, elapsed_time)

        self.plot_canvas.draw_idle()

    def _update_position(self, position):
        if position is None:
            self.position_canvas.itemconfig(self.position_text, text="Position:\n-- rad")
            return

        x = self.position_center[0] + self.position_radius * math.cos(position)
        y = self.position_center[1] - self.position_radius * math.sin(position)
        self.position_canvas.coords(self.position_dot, x - 5, y - 5, x + 5, y + 5)
        self.position_canvas.itemconfig(
            self.position_text,
            text=f"Position:\n{position:.2f} rad",
        )

    def update(self):
        try:
            values = self.read_telemetry()
            elapsed_time = time.time() - self.start_time
            self._update_plot(elapsed_time, values)
            self._update_position(values["position"])
        except Exception as exc:
            print(f"MotorTelemetry update failed: {exc}")
        finally:
            if self.parent.winfo_exists():
                self.parent.after(self.update_interval_ms, self.update)


def format_readback(label, value, unit):
    if value is None:
        return f"{label}:\n-- {unit}"
    return f"{label}:\n{value:.2f} {unit}"


def build_motor_telemetry_panel(app, parent):
    MotorTelemetryPanel(parent, app.organiser)

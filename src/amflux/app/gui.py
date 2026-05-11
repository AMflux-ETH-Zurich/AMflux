# ======================================================================
# Imports
# ======================================================================

import tkinter as tk
from tkinter import ttk
from collections import deque
import queue
import math
import time
import matplotlib
#tells matplotlib to use the TkAgg backend, which displays plots in Tkinter windows
matplotlib.use("TkAgg")
#takes figure and converts it into a Tkinter-compatible canvas
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
#used to create figures and plots
from matplotlib.figure import Figure
import object_dictionary_functions
from organiser import OperationModes
from drive import DriveState, drive_state_from_statusword

#serial for PID Demo
import serial

# ======================================================================
# Object dictionary and utility classes
# ======================================================================

objdict_data = object_dictionary_functions.load_drive_configuration()



operation_modes = [
    "ProfilePosition", 
    "Homing", 
    "ProfileVelocity", 
    "CyclicSynchronousPosition", 
    "CyclicSynchronousVelocity", 
    "CyclicSynchronousTorque"
]


class PageState:
    Home                        = 0
    ProfilePosition             = 1
    Homing                      = 2
    ProfileVelocity             = 3
    CyclicSynchronousPosition   = 4
    CyclicSynchronousVelocity   = 5
    CyclicSynchronousTorque     = 6
    PIDDemo                     = 7


MODE_NAME_TO_SELECTION = {
    "ProfilePosition": (PageState.ProfilePosition, OperationModes.ProfilePosition),
    "Homing": (PageState.Homing, OperationModes.Homing),
    "ProfileVelocity": (PageState.ProfileVelocity, OperationModes.ProfileVelocity),
    "CyclicSynchronousPosition": (
        PageState.CyclicSynchronousPosition,
        OperationModes.CyclicSynchronousPosition,
    ),
    "CyclicSynchronousVelocity": (
        PageState.CyclicSynchronousVelocity,
        OperationModes.CyclicSynchronousVelocity,
    ),
    "CyclicSynchronousTorque": (
        PageState.CyclicSynchronousTorque,
        OperationModes.CyclicSynchronousTorque,
    ),
}


PAGE_TO_OPERATION_MODE = {
    page_state: desired_mode
    for _mode_name, (page_state, desired_mode) in MODE_NAME_TO_SELECTION.items()
}


def drive_is_operation_enabled(drive):
    if drive is None:
        return False

    try:
        telemetry = drive.get_status()
    except Exception:
        return False

    if telemetry is None or len(telemetry) < 4 or telemetry[3] is None:
        return False

    try:
        state = drive_state_from_statusword(int(telemetry[3]))
    except Exception:
        return False

    return state == DriveState.OPERATION_ENABLED

# ======================================================================
# Home page
# ======================================================================

def HomePage(app, parent):
    """
    Home page: select operation mode and navigate to the corresponding page.
    """

    #Title Text
    landing_text_var = tk.StringVar()
    landing_text_var.set(value="EPOS4 Control Window\nPlease select an operation mode")

    label = tk.Label(
        parent,
        textvariable=landing_text_var,
        anchor=tk.CENTER,
        bg="lightblue",
        height=3,
        width=30,
        bd=3,
        font=("Helvetica", 16, "bold"),
        fg="black",
        padx=15,
        pady=15,
        justify=tk.CENTER,
        relief=tk.RAISED,
        wraplength=250
    )
    label.pack(pady=20)

    #Operation mode selection
    opt = tk.StringVar()
    opt.set("Operation Mode")
    drop = tk.OptionMenu(parent, opt, *operation_modes)
    drop.pack(pady=10)

    #Start button
    def start_button_func():
        selected = opt.get()
        selection = MODE_NAME_TO_SELECTION.get(selected)
        if selection is None:
            print("Please select a valid operation mode.")
            return

        page_state, desired_mode = selection
        app.drive.current_mode = desired_mode
        button.config(state=tk.DISABLED)

        def prepare_finished(prepared):
            def update_gui(prepared):
                button.config(state=tk.NORMAL)
                if prepared:
                    app.set_state(page_state)
                else:
                    print("Prepare operation failed. Staying on mode selection.")
            app.post_gui_event(update_gui, prepared)

        #   app.drive.stop_volt()
        print(app.drive.node.tpdo[1]["Statusword"].phys) 
        print(f"current state of the drive {app.drive.drivestate} (printed form guy.py)")
        app.drive.start_organiser()
        app.drive.request_prepare_operation(desired_mode, callback=prepare_finished)
        

    button = ttk.Button(
        parent,
        text="Start",
        command=start_button_func
    )
    button.pack(pady=10)

    def serial_demo_button_func():
        page_state = PageState.PIDDemo
        desired_mode = OperationModes.CyclicSynchronousPosition
        app.drive.current_mode = desired_mode
        button.config(state=tk.DISABLED)

        def prepare_finished(prepared):
            def update_gui(prepared):
                button.config(state=tk.NORMAL)
                if prepared:
                    app.set_state(page_state)
                else:
                    print("Prepare operation failed.")
            app.post_gui_event(update_gui, prepared)

        app.drive.start_organiser()
        app.drive.request_prepare_operation(desired_mode, callback=prepare_finished)

    ttk.Button(parent, text="Serial Position Demo",
            command=serial_demo_button_func).pack(pady=10)


# ======================================================================
# Specific mode page
# ======================================================================

def build_param_editor(parent, param_dict):
        """
        param_dict: {"param_name": value}
        """
        parent.grid_columnconfigure(1, weight=1)

        tk_vars = {}

        for row, (name, value) in enumerate(param_dict.items()):
            tk.Label(parent, text=name).grid(
                row=row, column=0, padx=5, pady=2
            )

            first_entry = next(iter(value.values()), {})
            if isinstance(first_entry, dict) and "value" in first_entry:
                current_value = first_entry["value"]
            else:
                current_value = first_entry
            var = tk.StringVar(
                value="" if current_value is None else str(current_value)
            )
            entry = ttk.Entry(parent, textvariable=var)
            entry.grid(row=row, column=1, padx=5, pady=2)

            tk_vars[name] = var

        return tk_vars

def ModePageBuilder(
    app,
    parent,
    modeint,
    modename,
    extra_panel_builder=None,
    telemetry_panel_builder=None,
): 
    
    #Grid
    parent.grid_columnconfigure(0, weight=10)
    parent.grid_columnconfigure(1, weight=5)
    parent.grid_columnconfigure(2, weight=0)
    parent.grid_columnconfigure(3, weight=0)
    parent.grid_columnconfigure(4, weight=0)
    parent.grid_columnconfigure(5, weight=5)
    parent.grid_columnconfigure(6, weight=10)
    
    
    parent.grid_rowconfigure(0, weight=10)
    parent.grid_rowconfigure(1, weight=5)
    parent.grid_rowconfigure(2, weight=0)
    parent.grid_rowconfigure(3, weight=0)
    parent.grid_rowconfigure(4, weight=0)
    parent.grid_rowconfigure(5, weight=5)
    parent.grid_rowconfigure(6, weight=10)
    


    
    #Title Text
    header = tk.Frame(parent)
    header.grid(row=0, column=3)

    landing_text_var = tk.StringVar()
    landing_text_var.set(f"{modename}\n Please enter operation variables")

    label = tk.Label(
        header, 
        textvariable=landing_text_var, 
        anchor=tk.CENTER,       
        bg="lightblue",      
        height=3,              
        width=30,              
        bd=3,                  
        font=("Helvetica", 16, "bold"),   
        fg="black",             
        padx=15,               
        pady=15,                
        justify=tk.CENTER,    
        relief=tk.RAISED,           
        wraplength=250         
    )
    label.grid(row=0, column=0, pady=20)

    #Parameter Editor
    editing = tk.Frame(parent)
    editing.grid(row=2, column=3)

    desired_mode = PAGE_TO_OPERATION_MODE[modeint]
    mode_code = OperationModes.abreviation[desired_mode]

    variables = build_param_editor(editing, objdict_data["mode"][mode_code]["comm"])

    def apply_button_func():
        if app.drive is None:
            print("Warning: Network not initialized, cannot update parameters")
            return
        for param_name, tk_var in variables.items():
            value = tk_var.get()
            app.drive.request_update_param(param_name, value, 5)

    
    apply_button = ttk.Button(editing, text="APPLY", command=lambda: apply_button_func())
    apply_button.grid(column=1)
    
    mode_options = tk.Frame(parent)
    mode_options.grid(row=3, column=3, pady=50)

    specific = tk.Frame(mode_options)
    specific.grid(row=0, column=0)

    specific_bit_specs = {
        PageState.ProfilePosition: [
            ("endless movement", 15),
            ("halt", 8),
            ("abs/rel", 6),
            ("change set immediately", 5),
            ("new setpoint", 4),
        ],
        PageState.Homing: [
            ("halt", 8),
            ("homing operation start", 4),
        ],
        PageState.ProfileVelocity: [
            ("halt", 8),
        ],
        PageState.CyclicSynchronousPosition: [],
        PageState.CyclicSynchronousVelocity: [],
        PageState.CyclicSynchronousTorque: [],
    }

    specific_bit_vars = []
    for column, (name, bit_position) in enumerate(specific_bit_specs[modeint]):
        var = tk.IntVar(value=0)
        check = ttk.Checkbutton(specific, text=name, variable=var)
        check.grid(row=0, column=column, padx=4, pady=2)
        specific_bit_vars.append((bit_position, var))

    def get_specific_bits():
        bits = 0
        for bit_position, var in specific_bit_vars:
            bits |= var.get() << bit_position
        return bits

    if extra_panel_builder is not None:
        extra_panel = tk.Frame(mode_options)
        extra_panel.grid(row=1, column=0, pady=(10, 0))
        extra_panel_builder(app, extra_panel)
    

    #Command Buttons
    commanding = tk.Frame(parent)
    commanding.grid(row=2, column=4, padx=50)

    commanding.grid_columnconfigure(0, weight=1)
    commanding.grid_columnconfigure(1, weight=0)
    commanding.grid_columnconfigure(2, weight=0)
    commanding.grid_columnconfigure(3, weight=0)
    commanding.grid_columnconfigure(4, weight=1)

    def enable_button_func():
        if app.drive is None:
            print("Warning: Network not initialized, cannot enable operation")
            return
        app.drive.request_enable_operation(get_specific_bits())
    
    def disable_button_func():
        app.drive.request_disable_voltage()

    enable_button = tk.Button(
        commanding,
        text="ENABLE",
        command=enable_button_func
    )
    enable_button.grid(row=0, column=0)
    enable_button_default_bg = enable_button.cget("background")
    enable_button_default_fg = enable_button.cget("foreground")

    def refresh_enable_button_color():
        try:
            if not enable_button.winfo_exists():
                return

            if drive_is_operation_enabled(app.drive):
                enable_button.config(
                    background="#c62828",
                    activebackground="#b71c1c",
                    foreground="white",
                    activeforeground="white",
                )
            else:
                enable_button.config(
                    background=enable_button_default_bg,
                    activebackground=enable_button_default_bg,
                    foreground=enable_button_default_fg,
                    activeforeground=enable_button_default_fg,
                )
            enable_button.after(200, refresh_enable_button_color)
        except tk.TclError:
            return

    refresh_enable_button_color()

    quick_stop_button = ttk.Button(
        commanding,
        text="QUICK-STOP",
        command= lambda: app.drive.request_quick_stop()
    )
    quick_stop_button.grid(row=1, column=0)

    disable_voltage_button = ttk.Button(
        commanding,
        text="DISABLE",
        command= disable_button_func
    )
    disable_voltage_button.grid(row=2, column=0)

    back_button = ttk.Button(
        parent, 
        text="Back", 
        command = lambda: app.set_state(PageState.Home)
    )
    back_button.grid(row=0, column=1, padx=50)
    
    def stop_record_data():
        record_button.config(text="RECORD", background = "white", command=lambda: record_data())
        app.drive.stop_recording()

    def record_data():
        record_button.config(text="RECORDING", background = "red", command=lambda: stop_record_data())
        app.drive.start_recording()

    record_button = ttk.Button(
        commanding, 
        text = "Record",
        command = lambda: record_data()
    )
    record_button.grid(row=4, column=0)

    if telemetry_panel_builder is None:
        telemetry_panel_builder = build_motor_telemetry_panel

    if telemetry_panel_builder is not None:
        telemetry_panel = tk.Frame(parent)
        telemetry_panel.grid(row=6, column=3)
        telemetry_panel_builder(app, telemetry_panel)

def read_and_normalize_serial(ser, min_raw, max_raw, min_pos, max_pos, value_index=0):
    """
    Read one value from serial and normalize it to a position range.
    min_raw/max_raw: expected range of incoming serial values
    min_pos/max_pos: target position range in drive units
    """
    
    try:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            return None

        values = [part.strip() for part in line.strip("()[]").split(",")]
        values = [part for part in values if part]
        raw = int(values[value_index])
        raw = max(min_raw, min(max_raw, raw))  # clamp to expected range
        normalized = int((raw - min_raw) / (max_raw - min_raw) * (max_pos - min_pos) + min_pos)
        return normalized
    except (ValueError, IndexError) as e:
        print(f"Serial parse error for {line!r}: {e}")
        return None
    except Exception as e:
        print(f"Serial read error: {e}")
        return None

def build_serial_position_panel(app, parent):
    tk.Label(parent, text="Port:").grid(row=0, column=0, padx=5)
    port_var = tk.StringVar(value="/dev/ttyUSB0")
    ttk.Entry(parent, textvariable=port_var, width=10).grid(row=0, column=1)

    tk.Label(parent, text="Baud:").grid(row=0, column=2, padx=5)
    baud_var = tk.StringVar(value="115200")
    ttk.Entry(parent, textvariable=baud_var, width=8).grid(row=0, column=3)

    connected = app.serial_connection is not None and app.serial_connection.is_open
    status_text = "Connected" if connected else "Disconnected"
    status_var = tk.StringVar(value=status_text)
    tk.Label(parent, textvariable=status_var, fg="gray").grid(row=1, column=0, columnspan=4)
    poll_running = {"active": False}

    def poll_serial():
        if not parent.winfo_exists():
            poll_running["active"] = False
            return

        poll_running["active"] = True
        if app.serial_connection and app.serial_connection.is_open:
            serial_value_normalized = read_and_normalize_serial(
                app.serial_connection,
                min_raw=0, max_raw=1023,    # adjust to your serial device range
                min_pos=0, max_pos=100000   # adjust to your drive position limits
            )
            if app.drive is not None and serial_value_normalized is not None:
                app.drive.request_update_param('Target position', serial_value_normalized)
        parent.after(50, poll_serial)

    def connect():
        try:
            if app.serial_connection and app.serial_connection.is_open:
                app.serial_connection.close()
            app.serial_connection = serial.Serial(port_var.get(), int(baud_var.get()), timeout=0.05)
            status_var.set(f"Connected: {port_var.get()}")
            if not poll_running["active"]:
                poll_serial()
        except Exception as e:
            status_var.set(f"Error: {e}")

    def disconnect():
        app.close_serial_connection()
        status_var.set("Disconnected")

    ttk.Button(parent, text="Connect", command=connect).grid(row=0, column=4, padx=5)
    ttk.Button(parent, text="Disconnect", command=disconnect).grid(row=0, column=5)

    if connected:
        poll_serial()

# ======================================================================
# Motor telemetry
# ======================================================================

class MotorTelemetryPanel:
    PLOT_SIGNALS = (
        {"key": "torque", "label": "Torque", "unit": "Nm", "color": "wheat"},
        {"key": "velocity", "label": "Velocity", "unit": "RPM", "color": "lightblue"},
    )

    def __init__(self, parent, drive, update_interval_ms=50, max_points=100, display_window=10):
        self.parent = parent
        self.drive = drive
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
        if self.drive is None:
            return values

        telemetry = self.drive.get_status()
        if telemetry is None:
            return values

        for key, index in (("torque", 0), ("velocity", 1), ("position", 2)):
            if len(telemetry) > index:
                values[key] = telemetry[index]
        return values

    def _format_readback(self, label, value, unit):
        if value is None:
            return f"{label}:\n-- {unit}"
        return f"{label}:\n{value:.2f} {unit}"

    def _update_plot(self, elapsed_time, values):
        self.time_data.append(elapsed_time)

        for key, widgets in self.plot_widgets.items():
            value = values[key]
            plot_value = float("nan") if value is None else value
            self.signal_data[key].append(plot_value)

            widgets["line"].set_data(self.time_data, self.signal_data[key])
            spec = widgets["spec"]
            widgets["readback"].set_text(
                self._format_readback(spec["label"], value, spec["unit"])
            )

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


def build_motor_telemetry_panel(app, parent):
    MotorTelemetryPanel(parent, app.drive)
        


# ======================================================================
# Application controller
# ======================================================================

class App(tk.Tk):
    def __init__(self, drive):
        super().__init__()
        
        self.title("EPOS4 Control Window")
        self.geometry("800x500")
        self.configure(bg="#363131") 

        #Drive
        self.drive = drive
        self.serial_connection = None
        self.gui_events = queue.Queue()
        # current state
        self.state = PageState.Home

        # container for pages
        self.container = tk.Frame(self)
        self.container.pack(fill="both", expand=True)

        self.render_page()
        self._poll_gui_events()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def post_gui_event(self, callback, *args):
        self.gui_events.put((callback, args))

    def _poll_gui_events(self):
        while True:
            try:
                callback, args = self.gui_events.get_nowait()
            except queue.Empty:
                break
            callback(*args)
        self.after(50, self._poll_gui_events)

    def set_state(self, new_state: str):
        self.state = new_state
        self.render_page()

    def close_serial_connection(self):
        if self.serial_connection is not None:
            try:
                self.serial_connection.close()
            finally:
                self.serial_connection = None

    def on_close(self):
        self.close_serial_connection()
        self.destroy()

    def clear_container(self):
        for widget in self.container.winfo_children():
            widget.destroy()

    def render_page(self):
        self.clear_container()

        if self.state == 0:
            HomePage(self, self.container)
        elif self.state == 1:
            ModePageBuilder(self, self.container, self.state, "Profile Position Mode")
        elif self.state == 2:
            ModePageBuilder(self, self.container, self.state, "Homing Mode")
        elif self.state == 3:
            ModePageBuilder(self, self.container, self.state, "Profile Velocity Mode")
        elif self.state == 4:
            ModePageBuilder(self, self.container, self.state, "Cyclic Synchronous Position Mode")
        elif self.state == 5:
            ModePageBuilder(self, self.container, self.state, "Cyclic Synchronous Velocity Mode")
        elif self.state == 6:
            ModePageBuilder(self, self.container, self.state, "Cyclic Synchronous Torque Mode")
        elif self.state == 7:
            ModePageBuilder(
                self,
                self.container,
                PageState.CyclicSynchronousPosition,
                "PID Demo",
                extra_panel_builder=build_serial_position_panel,
            )
        else:
            tk.Label(self.container, text="Unknown state").pack()


def main():
    # Start user interface
    app = App(drive=None)
    app.mainloop()

if __name__ == "__main__":
    main()

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

def ModePageBuilder(app, parent, modeint, modename): 
    
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
    
    specific = tk.Frame(parent)
    specific.grid(row=3, column=3, pady=50)

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

    #Motor Status
    MotorTelemetry(parent, app.drive)

#===============================================AI SLOP=================================================


def read_and_normalize_serial(ser, min_raw, max_raw, min_pos, max_pos):
    """
    Read a value from serial and normalize it to a position range.
    min_raw/max_raw: expected range of incoming serial values
    min_pos/max_pos: target position range in drive units
    """
    try:
        line = ser.readline().decode().strip()
        if not line:
            return None
        raw = int(line)
        raw = max(min_raw, min(max_raw, raw))  # clamp to expected range
        normalized = int((raw - min_raw) / (max_raw - min_raw) * (max_pos - min_pos) + min_pos)
        return normalized
    except Exception as e:
        print(f"Serial read error: {e}")
        return None

#===============================================AI SLOP=================================================



def pid_demo_page(app, parent):

    #define Grid
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
    landing_text_var.set("PID DEMO \n Demo Page for PID Control of Position")

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

    #===============================================AI SLOP=================================================
    # serial config UI
    serial_frame = tk.Frame(parent)
    serial_frame.grid(row=3, column=3)

    ser_ref = [None]

    tk.Label(serial_frame, text="Port:").grid(row=0, column=0, padx=5)
    port_var = tk.StringVar(value="COM3")
    ttk.Entry(serial_frame, textvariable=port_var, width=10).grid(row=0, column=1)

    tk.Label(serial_frame, text="Baud:").grid(row=0, column=2, padx=5)
    baud_var = tk.StringVar(value="115200")
    ttk.Entry(serial_frame, textvariable=baud_var, width=8).grid(row=0, column=3)

    status_var = tk.StringVar(value="Disconnected")
    tk.Label(serial_frame, textvariable=status_var, fg="gray").grid(row=1, column=0, columnspan=4)

    def poll_serial():
        if ser_ref[0] and ser_ref[0].is_open:
            serial_value_normalized = read_and_normalize_serial(
                ser_ref[0],
                min_raw=0, max_raw=1023,    # adjust to your serial device range
                min_pos=0, max_pos=100000   # adjust to your drive position limits
            )
            if serial_value_normalized is not None:
                app.drive.request_update_param('Target Position', serial_value_normalized)
        if parent.winfo_exists():
            parent.after(50, poll_serial)

    def connect():
        try:
            ser_ref[0] = serial.Serial(port_var.get(), int(baud_var.get()), timeout=0.05)
            status_var.set(f"Connected: {port_var.get()}")
            poll_serial()
        except Exception as e:
            status_var.set(f"Error: {e}")

    def disconnect():
        if ser_ref[0]:
            ser_ref[0].close()
            ser_ref[0] = None
        status_var.set("Disconnected")

    ttk.Button(serial_frame, text="Connect", command=connect).grid(row=0, column=4, padx=5)
    ttk.Button(serial_frame, text="Disconnect", command=disconnect).grid(row=0, column=5)

    #===============================================AI SLOP=================================================

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
        app.drive.request_enable_operation()
    
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

    #Motor Status
    MotorTelemetry(parent, app.drive)

# ======================================================================
# Motor telemetry
# ======================================================================

class MotorTelemetry:
    def __init__(self, parent, drive):
        """initialize the MotorTelemetry class"""

        self.drive = drive
        self.root_window = tk.Frame(parent)

        #self.root_window.grid_propagate(False)
        #self.root_window.configure(width=450, height=260)
        
        self.root_window.grid(row=6, column=3)

        #setup the grid for the two widgets side by side
        self.root_window.grid_rowconfigure(0, weight=0)
        self.root_window.grid_columnconfigure(0, weight=0)
        self.root_window.grid_columnconfigure(1, weight=0)

        #==========================================================

        #data buffers, lists that will store our values and automatically pop old ones
        #length of the deques
        self.max_points = 100
        #create the deque lines with a maximum length
        self.time_data = deque(maxlen=self.max_points)
        self.torque_data = deque(maxlen=self.max_points)
        self.velocity_data = deque(maxlen=self.max_points)

        #when the monitoring started
        self.start_time = time.time()
        # seconds to display on x-axis
        self.display_window = 10

        #==========================================================

        #create the corresponding matplotlib figures
        fig = Figure(figsize=(4, 2.5), dpi=100)

        #creates two subplots side by side (row, column, index of subplot)
        self.ax_torque = fig.add_subplot(1,2,1)
        self.ax_velocity = fig.add_subplot(1,2,2)

        #creates line objects for our plots which are empty, the comma ensures we don't return a list but the actual line data type
        self.torque_line, = self.ax_torque.plot([], [], 'k')
        self.velocity_line, = self.ax_velocity.plot([], [], 'k')

        #set titles and labels for the plots
        self.ax_torque.set_title("Torque vs Time")
        self.ax_velocity.set_title("Velocity vs Time")

        self.ax_torque.set_ylabel("Torque (Nm)")
        self.ax_velocity.set_ylabel("Velocity (RPM)")

        self.ax_torque.set_xlabel("Time (s)")
        self.ax_velocity.set_xlabel("Time (s)")

        # Create text annotations for displaying current values
        self.torque_text = self.ax_torque.text(0.05, 0.95, "", transform=self.ax_torque.transAxes,
                                                fontsize=10, verticalalignment='top',
                                                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        self.velocity_text = self.ax_velocity.text(0.05, 0.95, "", transform=self.ax_velocity.transAxes,
                                                    fontsize=10, verticalalignment='top',
                                                    bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

        #==========================================================

        #embed the matplotlib figure into the Tkinter window
        self.canvas_plot = FigureCanvasTkAgg(fig, master = self.root_window)

        #get_tk_widget() returns the Tkinter widget containing the plot and pack() adds it to the window
        #self.canvas_plot.get_tk_widget().pack(side="left")

        self.canvas_plot.get_tk_widget().grid(row=0, column=0)

        #==========================================================

        #create custom canvas to visualize motor position
        #creates the canvas widget with specified width, height, and background color
        self.canvas_widget = tk.Canvas(self.root_window, width=250, height=250, bg="white")
        #pady adds vertical padding around the canvas
        #self.canvas_widget.pack(side="left")
        self.canvas_widget.grid(row=0, column=1)
        # center of the canvas
        self.center = (125, 125)
        # radius for the motor position circle
        self.radius = 75
        #draw the motor circle on the canvas
        self.canvas_widget.create_oval(45 , 45, 205, 205, outline="black", width=2)
        #draw the position dot
        #args (x1, y1, x2, y2, fill): (x1,y1) top-left, (x2,y2) bottom-right of bounding box, fill=color
        self.dot = self.canvas_widget.create_oval(85, 25, 105, 35, fill="black")
        
        # Create text label for position value on canvas
        self.position_text = self.canvas_widget.create_text(100, 100, text="Position: 0.00 deg",
                                                                 font=("Arial", 10), fill="black")

        #==========================================================

        self.update()

    def read_motor_data(self):
        if self.drive is None:
            return None, None, None

        telemetry = self.drive.get_status()
        if telemetry is None or len(telemetry) < 3:
            return None, None, None

        torque = telemetry[0]
        velocity = telemetry[1]
        position = telemetry[2]

        return torque, velocity, position

    def _format_readback(self, label, value, unit):
        if value is None:
            return f"{label}:\n-- {unit}"
        return f"{label}:\n{value:.2f} {unit}"
    
    def update(self):
        """update the GUI with new motor data"""
        try:
            t = time.time() - self.start_time
            torque, velocity, position = self.read_motor_data()

            # Append NaN for unavailable values so plots stay alive without fake zeros.
            torque_plot = float("nan") if torque is None else torque
            velocity_plot = float("nan") if velocity is None else velocity

            self.time_data.append(t)
            self.torque_data.append(torque_plot)
            self.velocity_data.append(velocity_plot)

            #updating the plots with new data
            self.torque_line.set_data(self.time_data, self.torque_data)
            self.velocity_line.set_data(self.time_data, self.velocity_data)

            # Update text values on plots
            self.torque_text.set_text(self._format_readback("Torque", torque, "Nm"))
            self.velocity_text.set_text(self._format_readback("Velocity", velocity, "RPM"))

            #set x-axis to show moving window
            if t < self.display_window:
                self.ax_torque.set_xlim(0, self.display_window)
                self.ax_velocity.set_xlim(0, self.display_window)
            else:
                self.ax_torque.set_xlim(t - self.display_window, t)
                self.ax_velocity.set_xlim(t - self.display_window, t)

            #set y-axis limits
            #self.ax_torque.set_ylim(0, 20)
            #elf.ax_velocity.set_ylim(0, 100)

            #tells the program the figure needs to be redrawn, but do it when the GUI is idle, so it doesn't block the event loop
            self.canvas_plot.draw_idle()

            if position is not None:
                x = self.center[0] + self.radius * math.cos(position)
                y = self.center[1] - self.radius * math.sin(position)
                self.canvas_widget.coords(self.dot, x - 5, y - 5, x + 5, y + 5)
                self.canvas_widget.itemconfig(
                    self.position_text,
                    text=f"Position:\n{position:.2f} rad",
                )
            else:
                self.canvas_widget.itemconfig(self.position_text, text="Position:\n-- deg")

        except Exception as exc:
            print(f"MotorTelemetry update failed: {exc}")
        finally:
            if self.root_window.winfo_exists():
                self.root_window.after(50, self.update)
        


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
        self.gui_events = queue.Queue()
        # current state
        self.state = PageState.Home

        # container for pages
        self.container = tk.Frame(self)
        self.container.pack(fill="both", expand=True)

        self.render_page()
        self._poll_gui_events()

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
            pid_demo_page(self, self.container)
        else:
            tk.Label(self.container, text="Unknown state").pack()


def main():
    # Start user interface
    app = App(drive=None)
    app.mainloop()

if __name__ == "__main__":
    main()

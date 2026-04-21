# ======================================================================
# Imports
# ======================================================================

import tkinter as tk
from tkinter import ttk
import toml
from collections import deque
import math
import time
import matplotlib
#tells matplotlib to use the TkAgg backend, which displays plots in Tkinter windows
matplotlib.use("TkAgg")
#takes figure and converts it into a Tkinter-compatible canvas
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
#used to create figures and plots
from matplotlib.figure import Figure
from organiser import OperationModes

# ======================================================================
# Object dictionary and utility classes
# ======================================================================

# OLD: '/home/amfluxpi/AMflux/src/amflux/app/object_dictionary.toml'
# OLD: '/Users/wendelinroth/Desktop/Code/GitHub/AMflux/src/amflux/app/object_dictionary.toml'
with open("/home/amfluxpi/AMflux/src/amflux/app/object_dictionary_filled_export_for_objdict.toml") as data:
    objdict_data = toml.load(data)


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
    abreviation = {
        1:            "PPM", 
        2:            "HMM",
        3:            "PVM",
        4:            "CSP",
        5:            "CSV", 
        6:            "CST"
        }

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

        if selected == "ProfilePosition":
            app.set_state(PageState.ProfilePosition)
            app.drive.current_mode = OperationModes.ProfilePosition
        elif selected == "Homing":
            app.set_state(PageState.Homing)
            app.drive.current_mode = OperationModes.Homing
        elif selected == "ProfileVelocity":
            app.set_state(PageState.ProfileVelocity)
            app.drive.current_mode = OperationModes.ProfileVelocity
        elif selected == "CyclicSynchronousPosition":
            app.set_state(PageState.CyclicSynchronousPosition)
            app.drive.current_mode = OperationModes.CyclicSynchronousPosition
        elif selected == "CyclicSynchronousVelocity":
            app.set_state(PageState.CyclicSynchronousVelocity)
            app.drive.current_mode = OperationModes.CyclicSynchronousVelocity
        elif selected == "CyclicSynchronousTorque":
            app.set_state(PageState.CyclicSynchronousTorque)
            app.drive.current_mode = OperationModes.CyclicSynchronousTorque

        #   app.drive.stop_volt()
        print(app.drive.node.tpdo[1]["Statusword"].phys) 
        print(app.drive.drivestate)

    button = ttk.Button(
        parent,
        text="Start",
        command=start_button_func
    )
    button.pack(pady=10)


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

            var = tk.IntVar(value=next(iter(value.values())))
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

    mode_code = PageState.abreviation[modeint]

    variables = build_param_editor(editing, objdict_data["mode"][mode_code]["comm"])

    def update_params():
        if app.drive is None:
            print("Warning: Network not initialized, cannot update parameters")
            return
        for param_name, tk_var in variables.items():
            value = tk_var.get()  
            app.drive.request_update_param(param_name, value, 5)
    
    def set_params():
        if app.drive is None:
            print("Warning: Network not initialized, cannot update parameters")
            return
        print("set1")
        mode_code = PageState.abreviation[modeint]
        for name, tk_var in variables.items():
            try:
                objdict_data["mode"][mode_code]["comm"][name] = int(tk_var.get())
                print(f'{name}')
            except ValueError:
                objdict_data["mode"][mode_code]["comm"][name] = tk_var.get()
        print("finished setting")
        print(f" drive.current mode in gui: {app.drive.current_mode}")
        if app.drive.prepare_operation(app.drive.current_mode):
            print("ready to enable operation.")
            set_button.config(text="UPDATE", command=lambda: update_params())
        else:
            print("setting parameters failed")

        
    set_button = ttk.Button(editing, text="SET", command=lambda: set_params())
    set_button.grid(column=1)
    

    #Command Buttons
    commanding = tk.Frame(parent)
    commanding.grid(row=2, column=4, padx=50)

    commanding.grid_columnconfigure(0, weight=1)
    commanding.grid_columnconfigure(1, weight=0)
    commanding.grid_columnconfigure(2, weight=0)
    commanding.grid_columnconfigure(3, weight=0)
    commanding.grid_columnconfigure(4, weight=1)

    run_button = ttk.Button(
        commanding,
        text="ENABLE",
        command= lambda: app.drive.request_start()#enable_operation(10)
    )
    run_button.grid(row=0, column=0)

    pause_button = ttk.Button(
        commanding,
        text="QUICK-STOP",
        command= lambda: app.drive.request_quick_stop()
    )
    pause_button.grid(row=1, column=0)

    stop_button = ttk.Button(
        commanding,
        text="DISABLE",
        command= lambda: app.drive.request_disable_voltage()
    )
    stop_button.grid(row=2, column=0)

    back_button = ttk.Button(
        parent, 
        text="Back", 
        command = lambda: app.set_state(PageState.Home)
    )
    back_button.grid(row=0, column=1, padx=50)
    
    def stop_record_data():
        set_button.config(text="RECORD", background = "white", command=lambda: record_data())
        app.drive.stop_recording()

    def record_data():
        set_button.config(text="RECORDING", background = "red", command=lambda: stop_record_data())
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
        self.position_text = self.canvas_widget.create_text(100, 100, text="Position: 0.00 rad",
                                                                 font=("Arial", 10), fill="black")

        #==========================================================

        self.update()

    def read_motor_data(self):
        
        if self.drive is not None:
            telemetry = self.drive.get_status()
        
        torque = telemetry[0]
        velocity = telemetry[1]
        position = telemetry[2]
        
        return torque, velocity, position
    
    def update(self):
        """update the GUI with new motor data"""

        t = time.time() - self.start_time
        torque, velocity, position = self.read_motor_data()

        #append new data to the deques
        self.time_data.append(t)
        self.torque_data.append(torque)
        self.velocity_data.append(velocity)

        #updating the plots with new data
        self.torque_line.set_data(self.time_data, self.torque_data)
        self.velocity_line.set_data(self.time_data, self.velocity_data)

        # Update text values on plots
        self.torque_text.set_text(f"Torque:\n{torque:.2f} Nm")
        self.velocity_text.set_text(f"Velocity:\n{velocity:.2f} RPM")

        #set x-axis to show moving window
        if t < self.display_window:
            self.ax_torque.set_xlim(0, self.display_window)
            self.ax_velocity.set_xlim(0, self.display_window)
        else:
            self.ax_torque.set_xlim(t - self.display_window, t)
            self.ax_velocity.set_xlim(t - self.display_window, t)

        #set y-axis limits
        self.ax_torque.set_ylim(0, 20)
        self.ax_velocity.set_ylim(0, 100)

        #tells the program the figure needs to be redrawn, but do it when the GUI is idle, so it doesn't block the event loop
        self.canvas_plot.draw_idle()

        # Calculate dot position
        x = self.center[0] + self.radius * math.cos(position)
        y = self.center[1] - self.radius * math.sin(position)

        #update the position of the motor dot
        self.canvas_widget.coords(self.dot, x - 5, y - 5, x + 5, y + 5)
        
        # Update position text on canvas
        self.canvas_widget.itemconfig(self.position_text, text=f"Position:\n{position:.2f} rad")
        
        # Continue updating if not on home page
        self.root_window.after(200, self.update)
        


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
        # current state
        self.state = PageState.Home

        # container for pages
        self.container = tk.Frame(self)
        self.container.pack(fill="both", expand=True)

        self.render_page()

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
        else:
            tk.Label(self.container, text="Unknown state").pack()


def main():
   

    # Start user interface
    app = App(drive=None)
    app.mainloop()

if __name__ == "__main__":
    main()
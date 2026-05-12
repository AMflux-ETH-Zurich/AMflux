# ======================================================================
# Imports
# ======================================================================

import tkinter as tk
from tkinter import ttk
import queue
import object_dictionary_functions
from organiser import OperationModes
from drive import DriveState, drive_state_from_statusword
from gui_serial import build_serial_pid_panel
from gui_telemetry import build_motor_telemetry_panel

# ======================================================================
# Object dictionary and utility classes
# ======================================================================

objdict_data = object_dictionary_functions.load_drive_configuration()


class PageState:
    Home                        = 0
    ProfilePosition             = 1
    Homing                      = 2
    ProfileVelocity             = 3
    CyclicSynchronousPosition   = 4
    CyclicSynchronousVelocity   = 5
    CyclicSynchronousTorque     = 6
    PIDDemo                     = 7


MODE_PAGES = {
    PageState.ProfilePosition: {
        "menu_label": "ProfilePosition",
        "title": "Profile Position Mode",
        "operation_mode": OperationModes.ProfilePosition,
    },
    PageState.Homing: {
        "menu_label": "Homing",
        "title": "Homing Mode",
        "operation_mode": OperationModes.Homing,
    },
    PageState.ProfileVelocity: {
        "menu_label": "ProfileVelocity",
        "title": "Profile Velocity Mode",
        "operation_mode": OperationModes.ProfileVelocity,
    },
    PageState.CyclicSynchronousPosition: {
        "menu_label": "CyclicSynchronousPosition",
        "title": "Cyclic Synchronous Position Mode",
        "operation_mode": OperationModes.CyclicSynchronousPosition,
    },
    PageState.CyclicSynchronousVelocity: {
        "menu_label": "CyclicSynchronousVelocity",
        "title": "Cyclic Synchronous Velocity Mode",
        "operation_mode": OperationModes.CyclicSynchronousVelocity,
    },
    PageState.CyclicSynchronousTorque: {
        "menu_label": "CyclicSynchronousTorque",
        "title": "Cyclic Synchronous Torque Mode",
        "operation_mode": OperationModes.CyclicSynchronousTorque,
    },
}

DEFAULT_TELEMETRY_PANEL = object()


def mode_page_for_menu_label(menu_label):
    for page_state, config in MODE_PAGES.items():
        if config["menu_label"] == menu_label:
            return page_state
    return None


def organiser_is_operation_enabled(organiser):
    if organiser is None:
        return False

    try:
        telemetry = organiser.get_status()
    except Exception:
        return False

    if telemetry is None or len(telemetry) < 4 or telemetry[3] is None:
        return False

    try:
        state = drive_state_from_statusword(int(telemetry[3]))
    except Exception:
        return False

    return state == DriveState.OPERATION_ENABLED


def prepare_mode_page(app, page_state, desired_mode, button, failure_message):
    if app.organiser is None:
        print("Warning: Network not initialized, cannot prepare operation")
        return

    app.organiser.current_mode = desired_mode
    button.config(state=tk.DISABLED)

    def prepare_finished(prepared):
        def update_gui(prepared):
            button.config(state=tk.NORMAL)
            if prepared:
                app.set_state(page_state)
            else:
                print(failure_message)
        app.post_gui_event(update_gui, prepared)

    app.organiser.start_organiser()
    app.organiser.request_prepare_operation(desired_mode, callback=prepare_finished)


# ======================================================================
# Home page
# ======================================================================

def build_home_page(app, parent):
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
    drop = tk.OptionMenu(
        parent,
        opt,
        *[config["menu_label"] for config in MODE_PAGES.values()],
    )
    drop.pack(pady=10)

    #Start button
    def start_button_func():
        page_state = mode_page_for_menu_label(opt.get())
        if page_state is None:
            print("Please select a valid operation mode.")
            return

        desired_mode = MODE_PAGES[page_state]["operation_mode"]
        prepare_mode_page(
            app,
            page_state,
            desired_mode,
            start_button,
            "Prepare operation failed. Staying on mode selection.",
        )
        
    start_button = ttk.Button(
        parent,
        text="Start",
        command=start_button_func
    )
    start_button.pack(pady=10)

    def serial_demo_button_func():
        prepare_mode_page(
            app,
            PageState.PIDDemo,
            OperationModes.CyclicSynchronousPosition,
            serial_demo_button,
            "Prepare operation failed.",
        )

    serial_demo_button = ttk.Button(
        parent,
        text="Serial Position Demo",
        command=serial_demo_button_func,
    )
    serial_demo_button.pack(pady=10)


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
        tk.Label(parent, text=name).grid(row=row, column=0, padx=5, pady=2)

        first_entry = next(iter(value.values()), {})
        if isinstance(first_entry, dict) and "value" in first_entry:
            current_value = first_entry["value"]
        else:
            current_value = first_entry
        var = tk.StringVar(value="" if current_value is None else str(current_value))
        entry = ttk.Entry(parent, textvariable=var)
        entry.grid(row=row, column=1, padx=5, pady=2)

        tk_vars[name] = var

    return tk_vars

def build_mode_page(
    app,
    parent,
    mode_page_state,
    modename,
    extra_panel_builder=None,
    telemetry_panel_builder=DEFAULT_TELEMETRY_PANEL,
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

    desired_mode = MODE_PAGES[mode_page_state]["operation_mode"]
    mode_code = OperationModes.abreviation[desired_mode]

    variables = build_param_editor(editing, objdict_data["mode"][mode_code]["comm"])

    def apply_button_func():
        if app.organiser is None:
            print("Warning: Network not initialized, cannot update parameters")
            return
        for param_name, tk_var in variables.items():
            value = tk_var.get()
            app.organiser.request_update_param(param_name, value, 5)

    
    apply_button = ttk.Button(editing, text="APPLY", command=apply_button_func)
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
    for column, (name, bit_position) in enumerate(specific_bit_specs[mode_page_state]):
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
        if app.organiser is None:
            print("Warning: Network not initialized, cannot enable operation")
            return
        app.organiser.request_enable_operation(get_specific_bits())
    
    def disable_button_func():
        if app.organiser is None:
            print("Warning: Network not initialized, cannot disable voltage")
            return
        app.organiser.request_disable_voltage()

    def quick_stop_button_func():
        if app.organiser is None:
            print("Warning: Network not initialized, cannot quick-stop")
            return
        app.organiser.request_quick_stop()

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

            if organiser_is_operation_enabled(app.organiser):
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
        command=quick_stop_button_func,
    )
    quick_stop_button.grid(row=1, column=0)

    disable_voltage_button = ttk.Button(
        commanding,
        text="DISABLE",
        command=disable_button_func,
    )
    disable_voltage_button.grid(row=2, column=0)

    back_button = ttk.Button(
        parent, 
        text="Back", 
        command=lambda: app.set_state(PageState.Home),
    )
    back_button.grid(row=0, column=1, padx=50)
    
    def stop_record_data():
        if app.organiser is None:
            print("Warning: Network not initialized, cannot stop recording")
            return
        record_button.config(text="RECORD", background="white", command=record_data)
        app.organiser.stop_recording()

    def record_data():
        if app.organiser is None:
            print("Warning: Network not initialized, cannot start recording")
            return
        record_button.config(text="RECORDING", background="red", command=stop_record_data)
        app.organiser.start_recording()

    record_button = ttk.Button(
        commanding, 
        text="Record",
        command=record_data,
    )
    record_button.grid(row=4, column=0)

    if telemetry_panel_builder is DEFAULT_TELEMETRY_PANEL:
        telemetry_panel_builder = build_motor_telemetry_panel

    if telemetry_panel_builder is not None:
        telemetry_panel = tk.Frame(parent)
        telemetry_panel.grid(row=6, column=3)
        telemetry_panel_builder(app, telemetry_panel)


# ======================================================================
# Application controller
# ======================================================================

class App(tk.Tk):
    def __init__(self, organiser):
        super().__init__()
        
        self.title("EPOS4 Control Window")
        self.geometry("800x500")
        self.configure(bg="#363131") 

        self.organiser = organiser
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

    def set_state(self, new_state: int):
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

        if self.state == PageState.Home:
            build_home_page(self, self.container)
        elif self.state in MODE_PAGES:
            build_mode_page(self, self.container, self.state, MODE_PAGES[self.state]["title"])
        elif self.state == PageState.PIDDemo:
            build_mode_page(
                self,
                self.container,
                PageState.CyclicSynchronousPosition,
                "PID Demo",
                extra_panel_builder=build_serial_pid_panel,
            )
        else:
            tk.Label(self.container, text="Unknown state").pack()


def main():
    # Start user interface
    app = App(organiser=None)
    app.mainloop()

if __name__ == "__main__":
    main()

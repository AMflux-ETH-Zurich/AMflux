# ======================================================================
# Imports
# ======================================================================

import queue
import toml
import object_dictionary_functions
from errors import InitializationError, DriveStateDetError, DriveStatePathError, DesiredDriveStateError, DriveStateResetError, InitObjDict, DesiredMode, SanityCheck
from threading import Thread, Event
from drive import goto_state
import time
import numpy as np
from drive import DriveState, DriveCommand, Drive



# ======================================================================
# Object Dictionary
# ======================================================================

#'/home/amfluxpi/AMflux/src/amflux/app/object_dictionary.toml'
with open('/Users/wendelinroth/Desktop/Code/GitHub/AMflux/src/amflux/app/object_dictionary.toml', 'r') as data:
    objdict_data = toml.load(data)


# ======================================================================
# Operation
# ======================================================================

class OperationModes:
    ProfilePosition             = 1
    Homing                      = 3
    ProfileVelocity             = 6
    CyclicSynchronousPosition   = 8
    CyclicSynchronousVelocity   = 9
    CyclicSynchronousTorque     = 10
    abreviation = {
        ProfilePosition:            "PPM", 
        Homing:                     "HMM",
        ProfileVelocity:            "PVM",
        CyclicSynchronousPosition:  "CSP",
        CyclicSynchronousVelocity:  "CVP", 
        CyclicSynchronousTorque:    "CTP"}
    

def init_obj_dict(node, desired_mode):
    """Initializes all the object dictionary instances with helper functions. Values are read from object_dictionary.toml file.

    Args:
        node (int): EPOS4 node
        desired_mode (OperationModes): OperationMode to be reached after initialization.

    Raises:
        DesiredMode: raised if no desired mode is given
    """    
    motor_data = objdict_data["motor"]
    for func_name, params in motor_data.items():
        func = globals().get(object_dictionary_functions.func_name)
        try:
            func(node, **params)
        except InitObjDict as e:
            print(f"There was a problem Initialiting {func}, check dictionary values.")
            raise
        
    
    encoder_data = objdict_data["encoder"]
    for func_name, params in encoder_data.items():
        func = globals().get(object_dictionary_functions.func_name)
        try:
            func(node, **params)
        except InitObjDict as e:
            print(f"There was a problem Initialiting {func}, check dictionary values.")
            raise
        

    safety_data = objdict_data["safety"]
    for func_name, params in safety_data.items():
        func = globals().get(object_dictionary_functions.func_name)
        try:
            func(node, **params)
        except InitObjDict as e:
            print(f"There was a problem Initialiting {func}, check dictionary values.")
            raise
        

    if desired_mode == OperationModes.ProfilePosition:
        PPM_data = objdict_data["mode"]["PPM"]["conf"]
        completion_flag = False
        for func_name, params in PPM_data.items():
            func = globals().get(object_dictionary_functions.func_name)
            try:
                func(node, **params)
            except InitObjDict as e:
                print(f"There was a problem Initialiting {func}, check dictionary values.")
                raise
            else:
                completion_flag = True
            
    elif desired_mode == OperationModes.Homing:
        HMM_data = objdict_data["mode"]["HMM"]["conf"]
        for func_name, params in HMM_data.items():
            func = globals().get(object_dictionary_functions.func_name)
            try:
                func(node, **params)
            except InitObjDict as e:
                print(f"There was a problem Initialiting {func}, check dictionary values.")
                raise
            else:
                completion_flag = True
            
    elif desired_mode == OperationModes.ProfileVelocity:
        PVM_data = objdict_data["mode"]["PVM"]["conf"]
        for func_name, params in PVM_data.items():
            func = globals().get(object_dictionary_functions.func_name)
            try:
                func(node, **params)
            except InitObjDict as e:
                print(f"There was a problem Initialiting {func}, check dictionary values.")
                raise
            else:
                completion_flag = True
            
    elif desired_mode == OperationModes.CyclicSynchronousPosition:
        CSP_data = objdict_data["mode"]["CSP"]["conf"]
        for func_name, params in CSP_data.items():
            func = globals().get(object_dictionary_functions.func_name)
            try:
                func(node, **params)
            except InitObjDict as e:
                print(f"There was a problem Initialiting {func}, check dictionary values.")
                raise
            else:
                completion_flag = True
            
    elif desired_mode == OperationModes.CyclicSynchronousVelocity:
        CSV_data = objdict_data["mode"]["CSV"]["conf"]
        for func_name, params in CSV_data.items():
            func = globals().get(object_dictionary_functions.func_name)
            try:
                func(node, **params)
            except InitObjDict as e:
                print(f"There was a problem Initialiting {func}, check dictionary values.")
                raise
            else:
                completion_flag = True
            
    elif desired_mode == OperationModes.CyclicSynchronousTorque:
        CST_data = objdict_data["mode"]["CST"]["conf"]
        for func_name, params in CST_data.items():
            func = globals().get(object_dictionary_functions.func_name)
            try:
                func(node, **params)
            except InitObjDict as e:
                print(f"There was a problem Initialiting {func}, check dictionary values.")
                raise
            else:
                completion_flag = True
    elif completion_flag == False: #TODO: check if this works wit else/completion flag. does it get called?
        raise DesiredMode("No mode of operation selected, or selected mode is not prermittable. Please select a valid mode of operation")
    else:
        return True

class DriveOrganiser:
    """
    Manages drive operation lifecycle and telemetry monitoring.
    No control loop needed - firmware handles that.
    """
    
    def __init__(self, node, network):
        self.node = node
        self.network = network
        self.current_mode = None
        self.is_running = False
        
        # Single thread for telemetry and parameter updates
        self.monitor_thread = None
        self.stop_event = Event()
        
        # Queue for parameter updates from GUI
        self.param_update_queue = queue.Queue()

        self.recent_telemetry = None
    
    # ============================================
    # LIFECYCLE MANAGEMENT
    # ============================================
    
    def set_mode(self, desired_mode):
        self.node.sdo[0x6060] = desired_mode
        time.sleep(5)
        if self.node.sdo[0x6061] == desired_mode:
            return True
        else:
            raise Exception #TODO: define new error

    def prepare_operation(self) -> bool:
        """
        1. Read params from TOML for this mode
        2. Initialize object dictionary on controller
        3. Verify everything written correctly
        """
        
        if self.set_mode(self, self.current_mode) and init_obj_dict(self.node, self.current_mode):
            mode_code = OperationModes.abreviation[self.current_mode]
            for func_name, instance in objdict_data["mode"][mode_code]["comm"].items():
                func = globals().get(object_dictionary_functions.func_name)
                kwargs = {}
                for variable, default_val in instance.items(): 
                    user_val = None #TODO take value from GUI
                    if user_val == "":
                        write_val = default_val
                    try:
                        write_val = int(user_val)
                    except Warning:
                        try:
                            write_val = None #TODO : message on GUI: f"please enter a valid INT64 value for {variable}"
                        except Exception:
                            pass#TODO: message on GUI: f"invalid value for {variable}, using default value: {default_val}")
                    
                    kwargs[variable] = write_val
                
                func(self.node, **kwargs)
        else:
           pass  #TODO show error message on GUI: Prepare operation failed
                

    
    def start_operation(self, timeout):
        """
        1. Transition to OPERATION_ENABLED via goto_state()
        2. Start monitor thread
        3. Set is_running flag
        """
        goto_state(self.node, desired_state=DriveState.OPERATION_ENABLED, timeout=timeout)
        self.is_running = True
        self.monitor_thread = Thread(target=goto_state)


        
    def stop_operation(self):
        """
        1. Signal monitor thread to stop
        2. Shutdown drive via goto_state()
        3. Join thread
        """
        if not self.is_running:
                return
        self.is_running = False
        goto_state(self.node, desired_state=DriveState.READY_TO_SWITCH_ON, timeout=2)
        self.monitor_thread = Thread(target=goto_state)
        self.monitor_thread.join()



    
    # ============================================
    # RUNTIME UPDATES (called by GUI)
    # ============================================
    
    def update_parameter(self, param_name: str, value: int):
        """
        Queue a parameter update to be written in monitor thread.
        E.g., update_parameter('target_position', 5000)
        """
        self.param_update_queue.put((param_name, value))
    
    # ============================================
    # MONITORING THREAD
    # ============================================
    
    def _monitor_loop(self):
        """
        Runs continuously while is_running:
        
        1. Check for queued parameter updates -> write to OD
        2. Read telemetry (position, velocity, statusword)
        3. Check for faults
        4. Small sleep (e.g., 10-50ms cycle time)
        """
        while not self._stop_event.is_set():
            # Process any queued parameter updates
            self._process_param_updates()
            
            # Read telemetry
            telemetry = self._read_telemetry()
            
            # Store for get_status()
            self._latest_telemetry = telemetry
            
            time.sleep(0.02)  # 50Hz update rate
    

    def _process_param_updates(self):
        """
        Drain queue and write each param to controller OD.
        """
        while not self._param_update_queue.empty():
            param_name, value = self._param_update_queue.get()
            object_dictionary_functions.param_name = value
            
            
    def read_telemetry(self) -> dict:
        """
        Read key values from controller:
        - Position actual (0x6064)
        - Velocity actual (0x606C)
        - Statusword (0x6041)
        - Current mode display (0x6061)
        """
        #position from SSI encoder
        position_raw = self.node.sdo[0x3012][0x0D].raw
        #convert to radians
        position = (position_raw / 4096) * 2 * np.pi
        #torque from motor controller
        torque = self.node.sdo[0x6077].raw
        #velocity from motor controller
        velocity = self.node.sdo[0x606C].raw
        #statusword form motor controller
        status_word = self.node.sdo[0x6041]
        #Current mode display
        current_mode_disp = self.node.sdo[0x6061]
        
        self.recent_telemetry = [position, torque, velocity, status_word, current_mode_disp]
        
    def get_status(self) -> dict:
        """
        Return cached telemetry for GUI (non-blocking).
        """
        return self.recent_telemetry

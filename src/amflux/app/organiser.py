import queue
import toml
import object_dictionary_functions
from errors import InitializationError, DriveStateDetError, DriveStatePathError, DesiredDriveStateError, DriveStateResetError, InitObjDict, DesiredMode, SanityCheck
from threading import Thread
from main import goto_state

#'/home/amfluxpi/AMflux/src/amflux/app/object_dictionary.toml'
with open('/Users/wendelinroth/Desktop/Code/GitHub/AMflux/src/amflux/app/object_dictionary.toml', 'r') as data:
    objdict_data = toml.load(data)


class OperationModes:
    ProfilePosition             = 0
    Homing                      = 1
    ProfileVelocity             = 2
    CyclicSynchronousPosition   = 3
    CyclicSynchronousVelocity   = 4
    CyclicSynchronousTorque     = 5
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
        self.stop_event = None
        
        # Queue for parameter updates from GUI
        self._param_update_queue = queue.Queue()
    
    # ============================================
    # LIFECYCLE MANAGEMENT
    # ============================================
    
    def prepare_operation(self) -> bool:
        """
        1. Read params from TOML for this mode
        2. Initialize object dictionary on controller
        3. Verify everything written correctly
        """

        if init_obj_dict(self.node, self.current_mode):
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
                

    def start_operation(self):
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
    
    # ============================================
    # RUNTIME UPDATES (called by GUI)
    # ============================================
    
    def update_parameter(self, param_name: str, value: int):
        """
        Queue a parameter update to be written in monitor thread.
        E.g., update_parameter('target_position', 5000)
        """
        self._param_update_queue.put((param_name, value))
    
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
            # Map param_name to OD address and write
            
    def _read_telemetry(self) -> dict:
        """
        Read key values from controller:
        - Position actual (0x6064)
        - Velocity actual (0x606C)
        - Statusword (0x6041)
        - Current mode display (0x6061)
        """
        
    def get_status(self) -> dict:
        """
        Return cached telemetry for GUI (non-blocking).
        """
        return self._latest_telemetry

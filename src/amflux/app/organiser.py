# ======================================================================
# Imports
# ======================================================================

from dataclasses import dataclass
from enum import Enum, auto
import queue
import toml
import object_dictionary_functions
from errors import InitializationError, DriveStateDetError, DriveStatePathError, DesiredDriveStateError, DriveStateResetError, InitObjDict, DesiredMode, SanityCheck
from threading import Thread, Event
from drive import goto_state
import time
import numpy as np
from drive import DriveState, DriveCommand, get_DriveState
from can_functions import sword



# ======================================================================
# Object Dictionary
# ======================================================================

#'/home/amfluxpi/AMflux/src/amflux/app/object_dictionary.toml'
#'/Users/wendelinroth/Desktop/Code/GitHub/AMflux/src/amflux/app/object_dictionary.toml'
with open('/home/amfluxpi/AMflux/src/amflux/app/object_dictionary.toml', 'r') as data:
    objdict_data = toml.load(data)


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
        func = getattr(object_dictionary_functions, func_name)
        try:
            func(node, **params)
        except InitObjDict as e:
            print(f"There was a problem Initialiting {func}, check dictionary values.")
            raise
        
    
    encoder_data = objdict_data["encoder"]
    for func_name, params in encoder_data.items():
        func = getattr(object_dictionary_functions, func_name)
        try:
            func(node, **params)
        except InitObjDict as e:
            print(f"There was a problem Initialiting {func}, check dictionary values.")
            raise
        

    safety_data = objdict_data["safety"]
    for func_name, params in safety_data.items():
        func = getattr(object_dictionary_functions, func_name)
        try:
            func(node, **params)
        except InitObjDict as e:
            print(f"There was a problem Initialiting {func}, check dictionary values.")
            raise
        

    if desired_mode == OperationModes.ProfilePosition:
        PPM_data = objdict_data["mode"]["PPM"]["conf"]
        completion_flag = False
        for func_name, params in PPM_data.items():
            func = getattr(object_dictionary_functions, func_name)
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
            func = getattr(object_dictionary_functions, func_name)
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
            func = getattr(object_dictionary_functions, func_name)
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
            func = getattr(object_dictionary_functions, func_name)
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
            func = getattr(object_dictionary_functions, func_name)
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
            func = getattr(object_dictionary_functions, func_name)
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

# ======================================================================
# Drive Organiser
# ======================================================================

class CmdType(Enum):
    ENABLE_OPERATION = auto()
    QUICK_STOP = auto()
    DISABLE_VOLTAGE = auto()
    UPDATE_PARAM = auto()


@dataclass
class Command:
    type: CmdType
    data: tuple | None = None
    timeout: float | None = None


class DriveOrganiser:
    """
    Manages drive operation lifecycle and telemetry monitoring.
    No control loop needed - firmware handles that.
    """
    
    def __init__(self, node, network):
        # Network and Node
        self.node = node
        self.network = network
        
        # Organiser thread control
        self.thread = None
        self.shutdown = Event()

        # Command handling
        self.cmd_q = queue.Queue()
        self.stop_volt_requested = Event()
        self.cancel_transition = Event()
        
        # Power State
        self.power_enabled = False
        self.torque_enabled = False
        
        """
        # Queue for parameter updates from GUI
        self.param_update_queue = queue.Queue()

        self.recent_telemetry = None
        """

    def request_start(self, timeout=5.0):
        self.cmd_q.put(Command(CmdType.ENABLE_OPERATION, timeout=timeout))

    def request_disable_voltage(self):
        self.stop_volt_requested.set()
        self.cancel_transition.set()
        self.cmd_q.put(Command(CmdType.DISABLE_VOLTAGE))

    def request_quick_stop(self):
        if(get_DriveState(self.node) != DriveState.OPERATION_ENABLED):
            #TODO: print something or raise error?
            return
        
        #self.stop_volt_requested.set()
        self.cmd_q.put(Command(CmdType.QUICK_STOP))

    def request_update_param(self, name, value, timeout=5.0):
        self.cmd_q.put(Command(CmdType.UPDATE_PARAM, data=(name, value)))

    # ============================================
    # LIFECYCLE MANAGEMENT
    # ============================================
    
    def start_organiser(self):
        if self.thread and self.thread.is_alive():
            return
        self.shutdown.clear()
        #self.cmd_q.clear()
        self.thread = Thread(target=self.organiser_loop, daemon=True)
        self.thread.start()

    def stop_organiser(self):
        self.shutdown.set()
        if self.thread:
            self.thread.join(timeout=2.0)

    def organiser_loop(self):
        while not self.shutdown.is_set():
            # HIGH PRIORITY: handle request stop
            if self.stop_volt_requested.is_set():
                self.stop_volt()

                self.stop_volt_requested.clear()

                self.clear_non_stop_commands()  
            # get command
            try:
                cmd = self.cmd_q.get()
            except queue.Empty:
                cmd = None
            # execute command depending on type
            if cmd is not None:
                if cmd.type == DriveCommand.QUICK_STOP:
                    self.quick_stop()
                elif cmd.type == DriveCommand.ENABLE_OPERATION:
                    self.enable_operation()
                elif cmd.type == DriveCommand.UPDATE_PARAM:
                    name, value = cmd.data
                    self.update_parameter(name, value)
                elif cmd.type == DriveCommand.DISABLE_VOLTAGE:
                    self.stop_volt()
                continue

            if get_DriveState(self.node) == DriveState.FAULT_REACTION_ACITVE:
                var = sword(self.node)
                print("Error occured. Drive will be resetet to SWITCH ON DISABLED")
                print(f'statursword: {var}')
                goto_state(self.node, desired_state = DriveState.SWITCH_ON_DISABLED)

            #if torque enabled get telemtry
            self.read_telemetry()

            time.sleep(0.01)

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
        
        if self.set_mode(self.current_mode) and init_obj_dict(self.node, self.current_mode):
            mode_code = OperationModes.abreviation[self.current_mode]

            for func_name, instance in objdict_data["mode"][mode_code]["comm"].items():
                func = getattr(object_dictionary_functions, func_name)
                kwargs = {var_name: int(val) for var_name, val in instance.items()}
                func(self.node, **kwargs)
            return True
        else:
            return False   #TODO show error message on GUI: Prepare operation failed
                
    def enable_operation(self, timeout):
        """
        1. Transition to POWER_ENABLED and OPERATION_ENABLED via goto_state()
        2. Set power_enabled = True
        3. Set torque_enabled = True
        """
        #if self.cancel_transition.is_set():
        #    return
        #goto_state(self.node, desired_state=DriveState.POWER_ENABLED, timeout=timeout)
        #self.power_enabled = True

        if self.cancel_transition.is_set():
            return
        goto_state(self.node, desired_state=DriveState.OPERATION_ENABLED, timeout=timeout)
        var = sword(self.node)

        print(f'var: {var}')
        self.torque_enabled = True
       
    def stop_volt(self):
        """
        1. Shutdown drive via goto_state()
        2. 
        3. 
        """
        print("test a")
        if not self.power_enabled:
            return
        print("test b")
        goto_state(self.node, desired_state=DriveState.SWITCHED_ON, timeout=2)
        self.torque_enabled = False
        goto_state(self.node, desired_state=DriveState.READY_TO_SWITCH_ON, timeout=2)
        self.power_enabled = False

    def quick_stop(self, enable):
        """
        """
        if not self.torque_enabled:
            return
        
        goto_state(self.node, desired_state=DriveState.QUICK_STOP_ACTIVE, timeout=2)
        self.torque_enabled = False


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
    
    def process_param_updates(self):
        """
        Drain queue and write each param to controller OD.
        """
        #TODO: handle high priority comm params via PDO
        while not self.param_update_queue.empty():
            param_name, value = self.param_update_queue.get()
            getattr(object_dictionary_functions, param_name)(self.node, value)
            
            
    def read_telemetry(self) -> list:
        """
        Read key values from controller TPDOs (non-blocking, cached).
        Falls back to SDO if TPDO access fails.
        """
        try:
            # TPDO4: Position (0x6064), Velocity (0x606C), Torque (0x6077), Mode (0x6061)
            # TPDO3: Statusword (0x6041)
            # Non-blocking: read cached TPDO values (assume they've arrived during operation)
            
            position_raw = self.node.tpdo[4][0x6064].raw
            position = (position_raw / 4096) * 2 * np.pi
            
            velocity = self.node.tpdo[4][0x606C].raw
            torque = self.node.tpdo[4][0x6077].raw
            current_mode_disp = self.node.tpdo[4][0x6061].raw
            
            # Statusword from TPDO3 (not TPDO4)
            status_word = self.node.tpdo[3][0x6041].raw
            
        except Exception as e:
            # Fallback to SDO reads (slower but safe)
            print(f"TPDO read failed: {e}. Falling back to SDO.")
            try:
                position_raw = self.node.sdo[0x3012][0x0D].raw
                position = (position_raw / 4096) * 2 * np.pi
            except Exception:
                position = None
            try:
                velocity = self.node.sdo[0x606C].raw
            except Exception:
                velocity = None
            try:
                torque = self.node.sdo[0x6077].raw
            except Exception:
                torque = None
            try:
                status_word = self.node.sdo[0x6041].raw
            except Exception:
                status_word = None
            try:
                current_mode_disp = self.node.sdo[0x6061].raw
            except Exception:
                current_mode_disp = None
        
        self.recent_telemetry = [torque, velocity, position, status_word, current_mode_disp]
        return self.recent_telemetry
        
    def get_status(self) -> dict:
        """
        Return cached telemetry for GUI (non-blocking).
        """
        return self.read_telemetry()

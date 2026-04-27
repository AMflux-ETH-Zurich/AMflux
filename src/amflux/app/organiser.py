"""
Organizer module for managing EPOS4 drive operations and telemetry.

This module handles the lifecycle management of motor drives, including initialization,
operation modes, command processing, and telemetry monitoring. It provides both
synchronous and asynchronous interfaces for controlling drive state transitions and
reading motor parameters.

Key Components:
- OperationModes: Enumeration of supported CANopen operation modes (PPM, HMM, PVM, CSP, etc.)
- init_obj_dict(): Initializes object dictionary entries from TOML configuration
- DriveOrganiser: Main class managing drive operation with background monitoring thread

The DriveOrganiser uses a command queue for thread-safe communication between GUI and
the monitoring thread, handles state transitions, records telemetry data, and provides
parameter updates to the motor controller.

"""


# ======================================================================
# Imports
# ======================================================================

from dataclasses import dataclass
from enum import Enum, auto
import queue
import traceback
import toml
import object_dictionary_functions
from errors import DesiredMode
from threading import Thread, Event
from drive import goto_state
import time
import numpy as np
from drive import DriveState, get_DriveState
from can_functions import sword


# ======================================================================
# Object Dictionary
# ======================================================================

# load object dicitonary (OD)
# OLD: '/home/amfluxpi/AMflux/src/amflux/app/object_dictionary.toml'
# OLD: '/Users/wendelinroth/Desktop/Code/GitHub/AMflux/src/amflux/app/object_dictionary.toml'
with open("/home/amfluxpi/AMflux/src/amflux/app/object_dictionary_filled_export_for_objdict.toml") as data:
    objdict_data = toml.load(data)


class OperationModes:
    ProfilePosition             = 1
    Homing                      = 6
    ProfileVelocity             = 3
    CyclicSynchronousPosition   = 8
    CyclicSynchronousVelocity   = 9
    CyclicSynchronousTorque     = 10
    abreviation = {
        ProfilePosition:            "PPM", 
        Homing:                     "HMM",
        ProfileVelocity:            "PVM",
        CyclicSynchronousPosition:  "CSP",
        CyclicSynchronousVelocity:  "CSV", 
        CyclicSynchronousTorque:    "CST"}


MODE_BY_ABBREVIATION = {
    abbreviation: mode for mode, abbreviation in OperationModes.abreviation.items()
}

POSITION_RELEVANT_MODES = {
    OperationModes.ProfilePosition,
    OperationModes.Homing,
    OperationModes.CyclicSynchronousPosition,
}

VELOCITY_RELEVANT_MODES = {
    OperationModes.ProfileVelocity,
    OperationModes.CyclicSynchronousVelocity,
}


def normalize_operation_mode(mode):
    """Normalize an operation mode value, abbreviation, or attribute name."""
    if mode in OperationModes.abreviation:
        return mode

    if isinstance(mode, str):
        if mode in MODE_BY_ABBREVIATION:
            return MODE_BY_ABBREVIATION[mode]

        attr_value = getattr(OperationModes, mode, None)
        if attr_value in OperationModes.abreviation:
            return attr_value

    raise DesiredMode(
        f"Invalid mode of operation: {mode}. "
        "Please select a valid mode of operation."
    )


def mode_to_abbreviation(mode) -> str:
    """Return the TOML mode abbreviation for a normalized operation mode."""
    return OperationModes.abreviation[normalize_operation_mode(mode)]


def convert_toml_value(value):
    """Convert a TOML value into the runtime representation."""
    return None if value == "None" else value


def convert_toml_params(params: dict) -> dict:
    """Convert TOML parameters, converting "None" strings to Python None. OD variables can be left as "None" to envoke default values"

    Args:
        params (dict): EPOS4 network node
    """
    return {k: convert_toml_value(v) for k, v in params.items()}


def apply_od_section(node, section: dict, section_name: str) -> None:
    """Apply one TOML object-dictionary section to the controller."""
    for func_name, params in section.items():
        func = getattr(object_dictionary_functions, func_name)
        kwargs = convert_toml_params(params)
        try:
            func(node, **kwargs)
        except Exception as exc:
            print(
                f"OD apply failed for {section_name}.{func_name} "
                f"with params {kwargs}: {exc}"
            )
            raise


def init_obj_dict(node, desired_mode):
    """Initializes all the object dictionary instances with helper functions. Values are read from object_dictionary.toml file.

    Args:
        node (RemoteNode): EPOS4 node
        desired_mode (OperationModes): OperationMode to be reached after initialization.

    Raises:
        DesiredMode: raised if no desired mode is given
    """    
    desired_mode = normalize_operation_mode(desired_mode)
    mode_code = mode_to_abbreviation(desired_mode)
    completion_flag = False

    apply_od_section(node, objdict_data["motor"], "motor")
    apply_od_section(node, objdict_data["encoder"], "encoder")
    apply_od_section(node, objdict_data["safety"], "safety")

    mode_data = objdict_data.get("mode", {}).get(mode_code)
    if mode_data is None or "conf" not in mode_data:
        raise DesiredMode(
            f"No TOML configuration found for mode {mode_code} ({desired_mode})."
        )

    apply_od_section(node, mode_data["conf"], f"mode.{mode_code}.conf")
    completion_flag = True

    if completion_flag is False:
        raise DesiredMode(
            "No mode of operation selected, or selected mode is not permissible. "
            "Please select a valid mode of operation."
        )


# ======================================================================
# Drive Organiser
# ======================================================================

class CmdType(Enum):
    ENABLE_OPERATION = auto()
    QUICK_STOP = auto()
    DISABLE_VOLTAGE = auto()
    UPDATE_PARAM = auto()
    SET_PARAM = auto()

@dataclass
class Command:
    type: CmdType
    data: tuple | None = None
    timeout: float | None = None

class DriveOrganiser:
    """Manages drive operation lifecycle and telemetry monitoring.
    """
    
    def __init__(self, node, network):
        # Network and Node
        self.node = node
        self.network = network
        self.drivestate = get_DriveState(self.node)

        #loop handling
        #self.start = self.start_organiser()
        #self.stop = self.stop_organiser()

        self.current_mode = OperationModes.Homing
        
        # Organiser thread control
        self.thread = None
        self.shutdown = Event()

        # Command handling
        self.cmd_q = queue.Queue()
        self.stop_volt_requested = Event()
        self.cancel_transition = Event()
        
        # Power state
        self.power_enabled = False
        self.torque_enabled = False

        # Telemetry
        self.recent_telemetry = []

        # Recording
        self.recording = Event()
        self.data = np.array([])

    def request_enable_operation(self, timeout=5.0):
        print("requested: enable operation")
        self.cancel_transition.clear()
        self.cmd_q.put(Command(CmdType.ENABLE_OPERATION, timeout=timeout))

    def request_disable_voltage(self):
        print("requested: disable voltage")
        self.stop_volt_requested.set()
        self.cancel_transition.set()
        self.cmd_q.put(Command(CmdType.DISABLE_VOLTAGE))

    def request_quick_stop(self):
        print("requested: quick stop")
        if(get_DriveState(self.node) != DriveState.OPERATION_ENABLED):
            #TODO: print something or raise error?
            return
        
        #self.stop_volt_requested.set()
        self.cmd_q.put(Command(CmdType.QUICK_STOP))

    def request_update_param(self, name, value, timeout=5.0):
        print("requested: update params")
        self.cmd_q.put(Command(CmdType.UPDATE_PARAM, data=(name, value)))

    def request_set_param(self, mode_code, param_dict):
        self.current_mode = normalize_operation_mode(mode_code)
        print("requested: set params and preparing operation")
        self.cmd_q.put(Command(CmdType.SET_PARAM, data=(mode_code, param_dict)))

    def start_recording(self):
        print("execute: start recording")
        if self.recording.is_set():
            print("already recording")
        else:
            self.data_log.delete()
            self.recording.set()

    def stop_recording(self):
        print("execute: stop recording")
        if self.recording.is_set():
            self.recording.clear()
            time = time.time()
            np.savetxt(f"telemetry_data_{time}.csv", self.data, delimiter = ",")
        else:
            print("start recording first")
            
        
    # ============================================
    # Drive Organiser: runtime updates (called by GUI)
    # ============================================
    def _resolve_mode(self, mode):
        self.current_mode = normalize_operation_mode(mode)
        return self.current_mode

    def _current_mode_code(self) -> str:
        return mode_to_abbreviation(self.current_mode)

    def _read_sdo_value(self, entry_name: str):
        try:
            return self.node.sdo[entry_name].raw
        except Exception as exc:
            return f"<read failed: {exc}>"

    def _read_mode_display(self):
        try:
            return self.node.tpdo[1]["Modes of operation display"].phys
        except Exception:
            try:
                return self.node.sdo["Modes of operation display"].raw
            except Exception as exc:
                return f"<read failed: {exc}>"

    def _update_mode_command_config(self, mode_code: str, name: str, value):
        """Update one in-memory mode command entry and return its function kwargs."""
        comm_data = objdict_data["mode"][mode_code]["comm"]
        normalized_value = convert_toml_value(value)

        if name in comm_data:
            instance = comm_data[name]
            if isinstance(normalized_value, dict):
                updated = False
                for arg_name, arg_value in normalized_value.items():
                    if arg_name in instance:
                        instance[arg_name] = convert_toml_value(arg_value)
                        updated = True
                if not updated:
                    raise KeyError(
                        f"No matching argument names found for command '{name}'."
                    )
            else:
                arg_names = list(instance.keys())
                if not arg_names:
                    raise KeyError(f"Command '{name}' has no TOML parameters to update.")
                target_arg = arg_names[0]
                instance[target_arg] = normalized_value
                if len(arg_names) > 1:
                    print(
                        f"update command '{name}': applied scalar value to "
                        f"'{target_arg}', remaining args kept from TOML."
                    )
            return name, convert_toml_params(instance)

        for func_name, instance in comm_data.items():
            if name in instance:
                instance[name] = normalized_value
                return func_name, convert_toml_params(instance)

        raise KeyError(
            f"Unknown command parameter '{name}' for mode {mode_code}."
        )

    def _print_prepare_diagnostics(self, desired_mode):
        desired_mode = normalize_operation_mode(desired_mode)
        print("prepare_operation diagnostics:")
        print(f"  mode display: {self._read_mode_display()}")
        print(
            f"  motor pole pairs: "
            f"{self._read_sdo_value('Motor data.Number of pole pairs')}"
        )
        print(
            f"  SSI encoding type: "
            f"{self._read_sdo_value('SSI absolute encoder.SSI encoding type')}"
        )
        print(
            f"  SSI commutation offset: "
            f"{self._read_sdo_value('SSI absolute encoder.SSI commutation offset value')}"
        )
        print(
            f"  current controller gains: "
            f"P={self._read_sdo_value('Current control parameter set.Current controller P gain')}, "
            f"I={self._read_sdo_value('Current control parameter set.Current controller I gain')}"
        )

        if desired_mode in POSITION_RELEVANT_MODES:
            print(
                f"  position controller gains: "
                f"P={self._read_sdo_value('Position control parameter set.Position controller P gain')}, "
                f"I={self._read_sdo_value('Position control parameter set.Position controller I gain')}, "
                f"D={self._read_sdo_value('Position control parameter set.Position controller D gain')}"
            )

        if desired_mode in VELOCITY_RELEVANT_MODES:
            print(
                f"  velocity controller gains: "
                f"P={self._read_sdo_value('Velocity control parameter set.Velocity controller P gain')}, "
                f"I={self._read_sdo_value('Velocity control parameter set.Velocity controller I gain')}, "
                f"FFV={self._read_sdo_value('Velocity control parameter set.Velocity controller FF velocity gain')}, "
                f"FFA={self._read_sdo_value('Velocity control parameter set.Velocity controller FF acceleration gain')}"
            )

    def clear_non_stop_commands(self):
        """Remove stale non-stop commands after a stop-voltage request."""
        cleared_types = []
        retained_commands = []

        while True:
            try:
                cmd = self.cmd_q.get_nowait()
            except queue.Empty:
                break

            if cmd.type == CmdType.DISABLE_VOLTAGE:
                retained_commands.append(cmd)
            else:
                cleared_types.append(cmd.type.name)

        for cmd in retained_commands:
            self.cmd_q.put(cmd)

        if cleared_types:
            print(f"cleared stale commands after stop request: {cleared_types}")

    def set_parameter(self, mode_code, param_dict):
        desired_mode = self._resolve_mode(mode_code)
        normalized_mode_code = mode_to_abbreviation(desired_mode)

        for name, value in param_dict.items():
            self._update_mode_command_config(normalized_mode_code, name, value)

        print("finished setting commanding parameters, preparing operation")
        print(f"selected operation mode in gui: {normalized_mode_code}")
        if self.prepare_operation(desired_mode):
            print("ready to enable operation.")
        else:
            print("setting parameters failed")

    def update_parameter(self, param_name: str, value: int):
        """Apply a queued parameter update from the organiser thread.

        Args:
            param_name (str):   OD code of the parameter to be updated
            value (int):        new value for the parameter       

        """
        mode_code = self._current_mode_code()
        func_name, kwargs = self._update_mode_command_config(mode_code, param_name, value)
        print(f"applying update parameter: {func_name} -> {kwargs}")
        getattr(object_dictionary_functions, func_name)(self.node, **kwargs)
    
    def process_param_updates(self, timeout):
        """Deprecated queue path; updates are processed directly from cmd_q.

        Args:
            timeout (int): timeout limit for this operation
        """
        return

    # ============================================
    # Drive Organiser: telemetry
    # ============================================
    
    def read_telemetry(self) -> list:
        """
        Read key values from controller TPDOs (non-blocking, cached).
        Falls back to SDO if TPDO access fails.
        """
        try:
            # TPDO1: Statusword (0x6041)
            # TPDO2: Position (0x6064), Velocity (0x606C)
            # TPDO3: Current (0x30D1.0x01)
            # Non-blocking: read cached TPDO values (assume they've arrived during operation)
            
            status_word = self.node.tpdo[1]["Statusword"].raw
            
            position_raw = self.node.tpdo[2]["Position actual value"].phys
            position = (position_raw / 4096) * 2 * np.pi
            
            velocity = self.node.tpdo[2]["Velocity actual value"].phys

            torque = self.node.tpdo[3]["Current actual values.Current actual value"].phys
                    
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
            #try:
            #    current_mode_disp = self.node.sdo[0x6061].raw
            #except Exception:
            #    current_mode_disp = None
        
        self.recent_telemetry = [torque, velocity, position, status_word]
        return self.recent_telemetry
        
    def get_status(self) -> dict:
        """
        Return cached telemetry for GUI (non-blocking).
        """
        return self.read_telemetry()

    def log_telemetry(self):
        """
        Log telemetry data with a timestamp.
        Reads the current telemetry information, captures the current time, and appends both to the data collection.
        """
        
        tel = self.read_telemetry()
        timestamp = time.time()

        self.data.append(tel, timestamp)


    # ============================================
    # Drive Organiser: lifecycle management
    # ============================================
    
    def start_organiser(self):
        if self.thread and self.thread.is_alive():
            return
        self.shutdown.clear()
        with self.cmd_q.mutex:
            self.cmd_q.queue.clear()
        self.thread = Thread(target=self.organiser_loop, daemon=True)
        self.thread.start()
        print("organiser loop started")

    def stop_organiser(self):
        self.shutdown.set()
        if self.thread:
            self.thread.join(timeout=2.0)
        print("organiser loop stopped")

    def organiser_loop(self):
        """Main event loop for the organizer that manages drive operations and command processing.
        
        This loop continuously runs until shutdown is signaled, handling:
        - Drive fault detection and recovery (resets to SWITCH_ON_DISABLED state)
        - High priority stop voltage requests
        - Command queue processing (QUICK_STOP, ENABLE_OPERATION, UPDATE_PARAM, DISABLE_VOLTAGE)
        - Parameter update processing
        - Telemetry logging/reading based on recording state

        The loop runs at approximately 100 Hz (0.01s sleep interval) and processes commandsin priority order: 
        fault state handling > stop requests > command queue > telemetry.
        """
        
        while not self.shutdown.is_set():
            try:
                if get_DriveState(self.node) == DriveState.FAULT_REACTION_ACTIVE:
                    var = sword(self.node)
                    print("Error occurred. Drive will be reset to SWITCH ON DISABLED")
                    print(f"statusword: {var}")
                    goto_state(self.node, desired_state=DriveState.SWITCH_ON_DISABLED, timeout=5)

                # HIGH PRIORITY: handle request stop
                if self.stop_volt_requested.is_set():
                    self.stop_volt()
                    self.stop_volt_requested.clear()
                    self.clear_non_stop_commands()
                    self.cancel_transition.clear()

                try:
                    cmd = self.cmd_q.get(timeout=0.01)
                except queue.Empty:
                    cmd = None

                if cmd is not None:
                    if cmd.type == CmdType.QUICK_STOP:
                        self.quick_stop()
                    elif cmd.type == CmdType.ENABLE_OPERATION:
                        self.enable_operation(cmd.timeout or 5.0)
                    elif cmd.type == CmdType.UPDATE_PARAM:
                        name, value = cmd.data
                        self.update_parameter(name, value)
                    elif cmd.type == CmdType.DISABLE_VOLTAGE:
                        self.stop_volt()
                    elif cmd.type == CmdType.SET_PARAM:
                        mode_code, param_dict = cmd.data
                        self.set_parameter(mode_code, param_dict)

                if self.recording.is_set():
                    self.log_telemetry()
                else:
                    self.read_telemetry()

            except Exception as exc:
                print(f"organiser loop error: {exc}")
                traceback.print_exc()
                time.sleep(0.05)

    def set_mode(self, desired_mode):
        """Sets the modes of operation entry in OD to the desired mode
        
        Args: 
            desired_mode(OperationMode) = desired mode of operation
        """
        desired_mode = self._resolve_mode(desired_mode)
        self.node.rpdo[1]["Modes of operation"].phys = desired_mode
        self.node.rpdo[1].transmit()

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            mode_display = self._read_mode_display()
            if mode_display == desired_mode:
                print(f"set mode via rpdo: mode of operation display: {mode_display}")
                return True
            time.sleep(0.05)

        mode_display = self._read_mode_display()
        print(f"set mode via rpdo: mode of operation display: {mode_display}")
        return False

    def prepare_operation(self, desired_mode) -> bool:
        """
        1. Normalize selected mode
        2. Initialize object dictionary on controller
        3. Set and verify mode of operation
        4. Write mode command parameters
        5. Print targeted readback diagnostics
        """
        desired_mode = self._resolve_mode(desired_mode)
        mode_code = mode_to_abbreviation(desired_mode)

        init_obj_dict(self.node, desired_mode)

        if not self.set_mode(desired_mode):
            print(f"prepare_operation failed: mode display did not switch to {mode_code}")
            return False

        apply_od_section(self.node, objdict_data["mode"][mode_code]["comm"], f"mode.{mode_code}.comm")
        self._print_prepare_diagnostics(desired_mode)
        return True
                
    def enable_operation(self, timeout):
        """
        1. Transition to POWER_ENABLED and OPERATION_ENABLED via goto_state()
        2. Set power_enabled = True
        3. Set torque_enabled = True
        """
        print("execute: enable operation")
        if self.cancel_transition.is_set():
            return
        goto_state(self.node, desired_state=DriveState.OPERATION_ENABLED, timeout=timeout)
        var = sword(self.node)

        print(f'var: {var}')
        self.power_enabled = True
        self.torque_enabled = True
       
    def stop_volt(self):
        """
        1. Shutdown drive via goto_state()
        2. 
        3. 
        """
        print("execute: disable voltage")
        print("test a")
        if not self.power_enabled:
            return
        print("test b")
        goto_state(self.node, desired_state=DriveState.SWITCHED_ON, timeout=2)
        self.torque_enabled = False
        goto_state(self.node, desired_state=DriveState.READY_TO_SWITCH_ON, timeout=2)
        self.power_enabled = False

    def quick_stop(self):
        """
        """
        print("execute: quick stop")
        print("test a")
        if not self.torque_enabled:
            return
        print("test b")
        goto_state(self.node, desired_state=DriveState.QUICK_STOP_ACTIVE, timeout=2)
        self.torque_enabled = False

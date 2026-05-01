"""
Organizer module for managing EPOS4 drive operations and telemetry.

This module handles the lifecycle management of motor drives, including initialization,
operation modes, command processing, and telemetry monitoring. It provides both
synchronous and asynchronous interfaces for controlling drive state transitions and
reading motor parameters.

Key Components:
- OperationModes: Enumeration of supported CANopen operation modes (PPM, HMM, PVM, CSP, etc.)
- init_obj_dict(): Initializes object dictionary entries from DCF configuration
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

# Shared DCF-backed drive configuration.
objdict_data = object_dictionary_functions.load_drive_configuration()


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

POSITION_RELEVANT_MODES = {
    OperationModes.ProfilePosition,
    OperationModes.Homing,
    OperationModes.CyclicSynchronousPosition,
}

VELOCITY_RELEVANT_MODES = {
    OperationModes.ProfileVelocity,
    OperationModes.CyclicSynchronousVelocity,
}

DCF_TPDO_STATUS = 1
DCF_TPDO_MODE_DISPLAY = 2
DCF_TPDO_POSITION = 3
DCF_TPDO_VELOCITY = 4


def mode_to_abbreviation(mode) -> str:
    """Return the command-section abbreviation for a canonical operation mode."""
    try:
        return OperationModes.abreviation[mode]
    except KeyError as exc:
        raise DesiredMode(
            f"Invalid mode of operation: {mode}. "
            "Please select a valid mode of operation."
        ) from exc


def parse_command_value(value):
    """Convert GUI/config values into raw OD values for SDO writes."""
    if value is None:
        return None
    if isinstance(value, str):
        stripped = value.strip()
        if stripped == "" or stripped.lower() == "none":
            return None
        try:
            return int(stripped, 0)
        except ValueError:
            return stripped
    return value


def init_obj_dict(node, desired_mode):
    """DCF configuration is loaded once at startup via node.load_configuration().

    Args:
        node (RemoteNode): EPOS4 node
        desired_mode (OperationModes): OperationMode to be reached after initialization.

    Raises:
        DesiredMode: raised if no desired mode is given
    """    
    mode_code = mode_to_abbreviation(desired_mode)
    print(f"using startup-loaded DCF configuration for mode {mode_code}")


# ======================================================================
# Drive Organiser
# ======================================================================

class CmdType(Enum):
    ENABLE_OPERATION = auto()
    QUICK_STOP = auto()
    DISABLE_VOLTAGE = auto()
    UPDATE_PARAM = auto()
    PREPARE_OPERATION = auto()

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
        self.recent_telemetry = [None, None, None, None]

        # Recording
        self.recording = Event()
        self.data = np.array([])

    def request_enable_operation(self, specific_bits, timeout=5.0):
        print("requested: enable operation")
        self.cancel_transition.clear()
        self.cmd_q.put(Command(CmdType.ENABLE_OPERATION, timeout=timeout, data=specific_bits))

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

    def request_prepare_operation(self, desired_mode, callback=None):
        print("requested: prepare operation")
        self.cmd_q.put(Command(CmdType.PREPARE_OPERATION, data=(desired_mode, callback)))

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
    def _read_sdo_value(self, entry_name: str):
        try:
            return self.node.sdo[entry_name].raw
        except Exception as exc:
            return f"<read failed: {exc}>"

    def _read_mode_display(self):
        try:
            return self.node.sdo["Modes of operation display"].raw
        except Exception:
            pass

        try:
            return self.node.tpdo[DCF_TPDO_MODE_DISPLAY][
                "Modes of operation display"
            ].phys
        except Exception as exc:
            return f"<read failed: {exc}>"

    def _find_mapped_rpdo(self, index: int, subindex: int):
        for rpdo_number, rpdo_map in self.node.rpdo.map.items():
            if not rpdo_map.enabled:
                continue
            for variable in rpdo_map:
                if variable.index == index and variable.subindex == subindex:
                    return rpdo_number, rpdo_map, variable
        return None, None, None

    def _write_command_entry_sdo(self, entry: dict, value):
        index = entry["index"]
        subindex = entry["subindex"]
        if subindex == 0:
            self.node.sdo[index].raw = value
        else:
            self.node.sdo[index][subindex].raw = value
        print(f"wrote SDO 0x{index:04X}:{subindex:02X} ({entry['name']}) = {value}")

    def _write_command_entry_rpdo(self, rpdo_number: int, rpdo_map, variable, value):
        try:
            rpdo_map["Controlword"].raw = self.node.sdo["Controlword"].raw
        except KeyError:
            pass

        variable.raw = value
        rpdo_map.transmit()
        print(
            f"wrote RPDO{rpdo_number} 0x{variable.index:04X}:{variable.subindex:02X} "
            f"({variable.name}) = {value}"
        )

    def _write_command_entry(self, entry: dict):
        value = parse_command_value(entry.get("value"))
        if value is None:
            return

        index = entry["index"]
        subindex = entry["subindex"]
        rpdo_number, rpdo_map, variable = self._find_mapped_rpdo(index, subindex)
        if rpdo_map is not None:
            self._write_command_entry_rpdo(rpdo_number, rpdo_map, variable, value)
            return

        self._write_command_entry_sdo(entry, value)

    def _write_mapped_rpdo_command_entries(self, mode_code: str):
        written_count = 0
        for entry in self._iter_mode_command_entries(mode_code):
            value = parse_command_value(entry.get("value"))
            if value is None:
                continue

            rpdo_number, rpdo_map, variable = self._find_mapped_rpdo(
                entry["index"],
                entry["subindex"],
            )
            if rpdo_map is None:
                continue

            self._write_command_entry_rpdo(rpdo_number, rpdo_map, variable, value)
            written_count += 1
        return written_count

    def _iter_mode_command_entries(self, mode_code: str):
        for command_entries in objdict_data["mode"][mode_code]["comm"].values():
            for entry in command_entries.values():
                yield entry

    def _update_mode_command_config(self, mode_code: str, name: str, value):
        """Update one in-memory command value and return OD entries to write."""
        comm_data = objdict_data["mode"][mode_code]["comm"]
        normalized_value = parse_command_value(value)

        if name in comm_data:
            instance = comm_data[name]
            if isinstance(normalized_value, dict):
                updated = False
                for arg_name, arg_value in normalized_value.items():
                    if arg_name in instance:
                        instance[arg_name]["value"] = parse_command_value(arg_value)
                        updated = True
                if not updated:
                    raise KeyError(
                        f"No matching argument names found for command '{name}'."
                    )
            else:
                arg_names = list(instance.keys())
                if not arg_names:
                    raise KeyError(
                        f"Command '{name}' has no config parameters to update."
                    )
                target_arg = arg_names[0]
                instance[target_arg]["value"] = normalized_value
                if len(arg_names) > 1:
                    print(
                        f"update command '{name}': applied scalar value to "
                        f"'{target_arg}', remaining args kept from DCF defaults."
                    )
            return list(instance.values())

        for func_name, instance in comm_data.items():
            if name in instance:
                instance[name]["value"] = normalized_value
                return [instance[name]]

        raise KeyError(
            f"Unknown command parameter '{name}' for mode {mode_code}."
        )

    def _print_prepare_diagnostics(self, desired_mode):
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
            print(f"  target position: {self._read_sdo_value('Target position')}")

        if desired_mode in VELOCITY_RELEVANT_MODES:
            target_velocity = self._read_sdo_value("Target velocity")
            print(
                f"  velocity controller gains: "
                f"P={self._read_sdo_value('Velocity control parameter set.Velocity controller P gain')}, "
                f"I={self._read_sdo_value('Velocity control parameter set.Velocity controller I gain')}, "
                f"FFV={self._read_sdo_value('Velocity control parameter set.Velocity controller FF velocity gain')}, "
                f"FFA={self._read_sdo_value('Velocity control parameter set.Velocity controller FF acceleration gain')}"
            )
            print(f"  target velocity: {target_velocity}")

        if desired_mode == OperationModes.CyclicSynchronousTorque:
            print(f"  target torque: {self._read_sdo_value('Target torque')}")

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

    def update_parameter(self, param_name: str, value: int):
        """Apply a queued parameter update from the organiser thread.

        Args:
            param_name (str):   OD code of the parameter to be updated
            value (int):        new value for the parameter       

        """
        mode_code = mode_to_abbreviation(self.current_mode)
        entries = self._update_mode_command_config(mode_code, param_name, value)
        for entry in entries:
            self._write_command_entry(entry)
    
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
            # DCF TPDO1: Statusword (0x6041)
            # DCF TPDO3: Position actual value (0x6064)
            # DCF TPDO4: Velocity actual value (0x606C)
            # Current/torque is not mapped in the current DCF.
            # Non-blocking: read cached TPDO values (assume they've arrived during operation)
            
            status_word = self.node.tpdo[DCF_TPDO_STATUS]["Statusword"].raw
            
            position_raw = self.node.tpdo[DCF_TPDO_POSITION]["Position actual value"].phys
            position = (position_raw / 4096) * 2 * np.pi
            
            velocity = self.node.tpdo[DCF_TPDO_VELOCITY]["Velocity actual value"].phys

            torque = self.node.sdo["Torque actual values"]["Torque actual value averaged"].raw
                    
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
        
    def get_status(self) -> list:
        """
        Return cached telemetry for GUI without triggering controller reads.
        """
        return list(self.recent_telemetry)

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
                    print(f"in organiser loop shutdown is set, statusword: {var}")
                    goto_state(self.node, desired_state=DriveState.SWITCH_ON_DISABLED, timeout=5, specific_bits=0b0000000000000000)

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
                        specific_bits = cmd.data
                        self.enable_operation(cmd.timeout or 5.0, specific_bits=specific_bits)
                    elif cmd.type == CmdType.UPDATE_PARAM:
                        name, value = cmd.data
                        self.update_parameter(name, value)
                    elif cmd.type == CmdType.DISABLE_VOLTAGE:
                        self.stop_volt()
                    elif cmd.type == CmdType.PREPARE_OPERATION:
                        desired_mode, callback = cmd.data
                        try:
                            prepared = self.prepare_operation(desired_mode)
                        except Exception as exc:
                            prepared = False
                            print(f"prepare operation failed: {exc}")
                            traceback.print_exc()
                        if prepared:
                            print("ready to enable operation.")
                        else:
                            print("setting parameters failed")
                        if callback is not None:
                            callback(prepared)

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
        mode_to_abbreviation(desired_mode)
        self.current_mode = desired_mode
        self.node.rpdo[2]["Controlword"].raw = self.node.sdo["Controlword"].raw
        self.node.rpdo[2]["Modes of operation"].raw = desired_mode
        self.node.rpdo[2].transmit()

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
        mode_to_abbreviation(desired_mode)
        self.current_mode = desired_mode
        mode_code = mode_to_abbreviation(desired_mode)

        init_obj_dict(self.node, desired_mode)

        if not self.set_mode(desired_mode):
            print(f"prepare_operation failed: mode display did not switch to {mode_code}")
            return False

        for entry in self._iter_mode_command_entries(mode_code):
            self._write_command_entry(entry)
        self._print_prepare_diagnostics(desired_mode)
        return True
                
    def enable_operation(self, timeout, specific_bits):
        """
        1. Transition to POWER_ENABLED and OPERATION_ENABLED via goto_state()
        2. Set power_enabled = True
        3. Set torque_enabled = True
        """
        print("execute: enable operation")
        if self.cancel_transition.is_set():
            return
        goto_state(self.node, desired_state=DriveState.OPERATION_ENABLED, timeout=timeout, specific_bits=specific_bits)
        
        var = sword(self.node)
        print(f'func: enable_operation, statusword:{var}')

        self.power_enabled = True
        self.torque_enabled = True
        mode_code = mode_to_abbreviation(self.current_mode)
        written_count = self._write_mapped_rpdo_command_entries(mode_code)
        print(f"resent {written_count} mapped RPDO command value(s) after enable")
       
    def stop_volt(self, specific_bits):
        """
        1. Shutdown drive via goto_state()
        2. 
        3. 
        """
        print("execute: disable voltage")
        #print("test a")
        if not self.power_enabled:
            return
        print("test b, disable voltage direct failed")
        goto_state(self.node, desired_state=DriveState.SWITCHED_ON, timeout=2, specific_bits=0b0000000000000000)
        self.torque_enabled = False
        goto_state(self.node, desired_state=DriveState.READY_TO_SWITCH_ON, timeout=2, specific_bits=0b0000000000000000)
        self.power_enabled = False

    def quick_stop(self, specific_bits):
        """
        """
        print("execute: quick stop")
        print("test a")
        if not self.torque_enabled:
            return
        print("test b")
        goto_state(self.node, desired_state=DriveState.QUICK_STOP_ACTIVE, timeout=2, specific_bits=0b0000000000000000)
        self.torque_enabled = False

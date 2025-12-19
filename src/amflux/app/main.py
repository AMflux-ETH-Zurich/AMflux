"""
CANopen EPOS4 Configuration Utilities.

This module provides a structured, high-level interface for initializing and
configuring Maxon EPOS4 motor controllers via CANopen (CiA 301).

It includes:
    - CAN network setup and shutdown utilities
    - Node scanning and rate-limited SDO access
    - Initialization routines for motor, encoder, homing, control loops, etc.
    - A validation system (`check_init`) to prevent uninitialized parameters
    - Numerous EPOS4 configuration helpers following the Communication Guide

Structure:
    1. Auxiliary Helpers
    2. CAN Network Management
    3. Device Initialization Functions
    4. State Machine Control

This module intentionally mirrors the structure of the EPOS4 documentation
(Communication Guide, Firmware Specification), providing clear mapping from
code to documentation section numbers (e.g., "6.2.58").
"""


# ======================================================================
# Imports
# ======================================================================

import time
import canopen
import keyboard
import can
import toml
import utils
import can_functions
import object_dictionary_functions

import warnings
from typing import Mapping, Hashable, Dict, Any


# ======================================================================
# Constants
# ======================================================================

# Global CANopen network instance
net = None

# “FATAL default value” used by your system
FATAL_DEFAULT_VALUE: int = 67

# Safe transmission limit for SocketCAN
RATE_LIMIT_MSGS_PER_SEC: int = 750

# ======================================================================
# Exceptions
# ======================================================================

class InitializationError(Exception):
    """Raised when a configuration parameter has not been initialized correctly."""
    pass


class DriveStateDetError(Exception):
    """Raised when Drive State cannot be determined."""
    pass


class DriveStatePathError(Exception):
    """Raised if no valid Path is found/used"""
    pass


class DesiredDriveStateError(Exception):
    """Raised if desired DriveState is not reached"""
    pass


class DriveStateResetError(Exception):
    """Raised if DriveState cannot be reset from FAULT"""
    pass

class InitObjDict(Exception):
    """Raised if object dictionary is not initializable"""
    pass


class DesiredMode(Exception):
    """Raised if object dictionary is not initializable"""
    pass

class SanityCheck(Exception):
    """Raised if Sanity Check fails"""
    pass

# ======================================================================
# Region: AUXILIARY FUNCTIONS
# ======================================================================
'''

def removekey(d: Mapping[str, Any], key: Hashable) -> Dict[str, Any]:
    """Return a shallow copy of mapping `d` with `key` removed."""
    r = dict(d)
    # Use pop to avoid KeyError if `key` does not exist in the mapping
    r.pop(key, None)
    return r


def confirm(prompt: str = "Continue? [y/N]: ") -> bool:
    """Prompt the user for a yes/no answer and return True for yes.

    Accepts 'y', 'yes', 'j', 'ja' (case-insensitive) as affirmative answers.
    Any other response is treated as negative.
    """
    ans = input(prompt).strip().lower()
    yes_answers = {"y", "yes", "j", "ja"}
    return ans in yes_answers


def check_init(objects):
    """Checks if all objects of function to be checked have been initialized correctly.

    Args:
        objects (dict): dictionary of objects to be checked. Name, Value pairs.

    Raises:
        InitializationError: After Warning, user must confirm Default value or enter new value. InitializationError is raised if value is still not valid.
        InitializationError: Raised if no default value available. FATAL ERROR 67.

    Returns True when validation is complete and execution may continue.
    """
    objects_clean = removekey(objects, 'node')
    for name, value in objects_clean.items():
        # warn user about default value
        if value is None:
            warnings.warn(f"{name} is set to default value.", UserWarning)
            if not confirm("Continue with default value? [y/N]: "):
                new_value = input(f"Enter a valid value for {name}: ")
                try:
                    objects[name] = int(new_value)
                except ValueError:
                    raise InitializationError(f"{name} must be int.")
            else:
                continue
        # fatal, user must init value correctly
        if value is not None and (value == "67" or value == 67):
            raise InitializationError(f"{name} is not initialized correctly. Please enter a valid value for {name}.")
    return True 
'''

# ======================================================================
# Region: CAN NETWORK MANAGEMENT
# ======================================================================
'''
def network_setup(node_id: int, node_eds: str, node_channel: str):
    """Sets up CANopen network and adds node.

    Args:
        node_id (int): CANopen Node ID of the device.
        node_eds (str): Path to the EDS file for the device.
        node_channel (str): CAN interface channel (e.g., 'can0' for Linux SocketCAN).

    Returns:
        CANopen Network and Node that was added
    """
    global net
    if net == None:
        # Create a CANopen network
        net = canopen.Network()
        # Connect to a CAN interface (e.g., 'can0' for Linux SocketCAN)
        # Write 'virtual' when debugging on mac 
        net.connect(channel=node_channel, bustype='socketcan')
    # Add a node to the network with a specific EDS file
    node = net.add_node(node_id, node_eds) 
    # Return network and the node 
    return net, node


def network_shutdown():
    """Shuts down the CANopen network after use"""
    global net
    if net != None:
        # Disconnect from the network
        net.disconnect()
        net = None


def send_can_message(net, arbitration_id, data: list):
    """Sends and arbitrary CAN message on the CAN bus.
    
    Args:
        arbitration_id (int): CAN ID for the message.
        data (list): Data payload as a list of integers (0-255).
    """
    msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
    net.bus.send(msg)

   
def sanity_check(net):
    """Checks if CAN communication is running by sending a test message and checking for the expected response."""
    start = time.time()
    send_can_message(net, 0x601, [0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
    msg_soll = can.Message(
        arbitration_id=0x581, 
        data = [0x43, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00], 
        is_extended_id=False
    )
    while True:
        msg_ist = net.bus.recv(1)
        current = time.time()
        if (current - start) < 3:
            if msg_ist is not None:
                if (msg_ist.arbitration_id == 0x581 and msg_ist.data[:3] == bytes([0x43, 0x64, 0x60])):
                    print("INFO:Sanity Check passed.")
                    return True
                else: 
                    raise SanityCheck(f"Sanity Check failed. Message : {msg_ist}")
        else:
            print("no answer received after 3 seconds")
            return False
'''

# ======================================================================
# Region: EPOS4 INITIALIZATION FUNCTIONS
# ======================================================================
'''
"""
Initialization and configuration functions for EPOS4 motor controllers.
Each initialization or configuration function corresponds to a section in the EPOS4 Communication Guide. 
If no arguments are given, default values are chosen.

Initialization & Configuration Functions
    1. axis_configuration_init
    2. motor_init
    3. ssi_abs_encoder_init
    4. electrical_system_init
    5. current_control_parameter_init
    6. position_control_parameter_init
    7. velocity_control_parameter_init
    8. velocity_observer_parameter_init
    9. dual_loop_position_control_parameter_init
    10. home_position_init
    11. home_offset_distance_init
    12. Current_threshold_homing_init
    13. standstill_window_init
    14. standstill_window_time_init
    15. standstill_window_timeout_init
    16. abort_connection_option_init
    17. mode_of_operation_init
    18. following_error_window_init
    19. following_error_timeout_init
    20. position_window_init
    21. position_window_time_init
    22. shutdown_option_code
    23. disable_operation_option_code
    24. halt_option_code
    25. fault_reaction_option_code
    26. control_word
    27. target_torque
    28. motor_rated_torque
    29. target_position
    30. software_position_limit
    31. max_profile_velocity
    32. max_motor_speed
    33. profile_velocity
    34. profile_acceleration
    35. profile_deceleration
    36. quick_stop_deceleration
    37. homing_method_init
    38. homing_speeds
    39. homing_acceleration
    40. si_unit_position
    41. si_unit_velocity
    42. si_unit_acceleration
    43. position_offset
    44. velocity_offset
    45. torque_offset
    46. interpolation_time_period
    47. max_acceleration
    48. digital_outputs
    49. target_velocity
    50. motor_type

Touch Probe (6.2.134–6.2.137, 6.2.140–6.2.142) not implemented.
"""


def axis_configuration_init(node, sens_res: int=None, sys_speed: int=None):
    #6.2.52
    #Axis configuration for absolute SSI encoder
    node.sdo[0x3000][1].raw = 0x00000300
    #Axis control structure
    node.sdo[0x3000][2].raw = 0b00000000000000100000000100100001
    #Commutaton Sensors
    node.sdo[0x3000][3].raw = 0x00000020
    #Miscellaneous Axis Configuration
    node.sdo[0x3000][4].raw = 0x00000000
    #Main Sensor Resolution (given in [increments/revolution])
    node.sdo[0x3000][5].raw = sens_res #REFER TO RENISHAW ENCODER SHEET
    #Max System Speed in rpm
    node.sdo[0x3000][6].raw = sys_speed #REFER TO RENISHAW ENCODER SHEET


def motor_init(node, motor_type: int=None, nominal_current: int=None, current_lim: int=None, 
                        pole_pairs: int=None, therm_const: int=None, tor_const: int=None):
    
    
    check_init(locals())
    #6.2.53
    #motor type (Sinusoidal PM BL motor = 10 or in hex 0x0A)
    node.sdo[0x6402][0x0].raw = motor_type if motor_type is not None else 0x000A
    #nominal current flowing through motor windings in mA
    node.sdo[0x3001][0x1].raw = nominal_current if nominal_current is not None else 15000
    #output current limit in mA
    node.sdo[0x3001][0x2].raw = current_lim if current_lim is not None else 30000
    #number of pole pairs
    node.sdo[0x3001][0x3].raw = pole_pairs if pole_pairs is not None else 1
    #thermal time constant of windings
    node.sdo[0x3001][0x4].raw = therm_const if therm_const is not None else 40
    #torque constant of motor
    node.sdo[0x3001][0x5].raw = tor_const if tor_const is not None else 0


def ssi_abs_encoder_init(node, data_rate: int=None, data_bits: int=None, encoding_type: int=None,
                                timeout_time: int=None, power_up_time: int=None, commutation_offset_value: int=None,
                                        position_bits: int=None, communication_additional_delay: int=None):
    #6.2.58
    check_init(locals())
    #SSI data rate
    node.sdo[0x3012][0x1].raw = data_rate if data_rate is not None else 2000 #REFER TO RENISHAW ENCODER SHEET
    #SSI number of data bits
    node.sdo[0x3012][0x2].raw = data_bits if data_bits is not None else 0x00000C00 #REFER TO RENISHAW ENCODER SHEET
    #SSI encoding type
    node.sdo[0x3012][0x3].raw = encoding_type if encoding_type is not None else 0x001 #REFER TO RENISHAW ENCODER SHEET
    #SSI timeout time
    node.sdo[0x3012][0x5].raw = timeout_time if timeout_time is not None else 30 #REFER TO RENISHAW ENCODER SHEET
    #Special bits trailing data
    #READ ONLY node.sdo[0x3012][0x6].raw = sbits_trailing_data if sbits_trailing_data is not None else 0 #REFER TO RENISHAW ENCODER SHEET
    #SSI refresh frequency
    #READ ONLY node.sdo[0x3012][0x7].raw = refresh_frequency #REFER TO RENISHAW ENCODER SHEET
    #SSI Power up time
    node.sdo[0x3012][0x8].raw = power_up_time if power_up_time is not None else 200 #REFER TO RENISHAW ENCODER SHEET
    #SSI postition raw value, lower 32 bits
    #READ ONLY node.sdo[0x3012][0x9].raw = position_raw_value
    #SSI commutaiton offset value
    node.sdo[0x3012][0xA].raw = commutation_offset_value if commutation_offset_value is not None else 0
    #SSI position bits
    node.sdo[0x3012][0xB].raw = position_bits if position_bits is not None else  0x0000000C #REFER TO RENISHAW ENCODER SHEET, MUST BE REDUCED IF >32
    #SSI special bits leading data
    #READ ONLY node.sdo[0x3012][0xC].raw = sbits_leading_data if sbits_leading_data is not None else 0 #REFER TO RENISHAW ENCODER SHEET
    #SSI position raw value complemete
    #READ ONLY node.sdo[0x3012][0xD].raw = position_raw_value_complete
    #SSI communication additional delay
    node.sdo[0x3012][0xE].raw = communication_additional_delay if communication_additional_delay is not None else -1 #REFER TO RENISHAW EN


def electrical_system_init(node, electrical_resistance: int=None, electrical_inductance: int=None):
    #6.2.54
    check_init(locals())
    #electrical resistance of motor windings in mOhm
    node.sdo[0x3002][0x1].raw = electrical_resistance if electrical_resistance is not None else 0
    #electrical inductance of motor windings in uH
    node.sdo[0x3002][0x2].raw = electrical_inductance if electrical_inductance is not None else 0


def current_control_parameter_init(node, p_gain: int = None, i_gain: int = None):
    #6.2.61
    check_init(locals())
    #Current controller P-gain
    node.sdo[0x30A0][1].raw = p_gain if p_gain is not None else 1171880
    #Current controller I-gain
    node.sdo[0x30A0][2].raw = i_gain if i_gain is not None else 3906250


def position_control_parameter_init(node, p_gain:int = None, i_gain: int = None, d_gain:int = None,
                               ffv_gain: int = None, ffa_gain: int = None, i_gain_si_unit: int = None):
    #6.2.62
    check_init(locals())
    #Position controller P-gain
    node.sdo[0x30A1][1].raw = p_gain if p_gain is not None else 1500000
    #Position controller I-gain
    node.sdo[0x30A1][2].raw = i_gain if i_gain is not None else 780000
    #Position controller D-gain
    node.sdo[0x30A1][3].raw = d_gain if d_gain is not None else 16000
    #Position controller Feed Forward velocity gain
    node.sdo[0x30A1][4].raw = ffv_gain if ffv_gain is not None else 0
    #Position controller Feed Forward acceleration gain
    node.sdo[0x30A1][5].raw = ffa_gain if ffa_gain is not None else 0
    #Position controller I-gain SI units
    node.sdo[0x30A1][9].raw = i_gain_si_unit if i_gain_si_unit is not None else 0xFA040300


def velocity_control_parameter_init(node, p_gain:int = None, i_gain: int = None, 
                               ffv_gain: int = None, ffa_gain: int = None, f_cutoff: int = None):
    #6.2.63
    check_init(locals())
    #Velocity controller P-gain
    node.sdo[0x30A2][1].raw = p_gain if p_gain is not None else 20000
    #Velocity controller I-gain
    node.sdo[0x30A2][2].raw = i_gain if i_gain is not None else 500000
    #Velocity controller Feed Forward velocity gain
    node.sdo[0x30A2][3].raw = ffv_gain if ffv_gain is not None else 0
    #Velocity controller Feed Forward acceleration gain
    node.sdo[0x30A2][4].raw = ffa_gain if ffa_gain is not None else 0
    #Velocity controller filter cut-off frequency in HZ
    node.sdo[0x30A2][5].raw = f_cutoff if f_cutoff is not None else 600
    

def velocity_observer_parameter_init(node, pos_corr_gain: int = None, vel_corr_gain: int = None,
                                load_corr_gain: int = None, friction: int = None, inertia: int = None
                                ):
    #6.2.64
    check_init(locals())
    #Velocity observer position correction gain given in promille
    node.sdo[0x30A3][1].raw = pos_corr_gain if pos_corr_gain is not None else 400
    #Velocity observer velocity correction gain given in mHz
    node.sdo[0x30A3][2].raw = vel_corr_gain if vel_corr_gain is not None else 100000
    #Velocity observer load correction gain
    node.sdo[0x30A3][3].raw = load_corr_gain if load_corr_gain is not None else 33
    #Velocity observer friction
    node.sdo[0x30A3][4].raw = friction if friction is not None else 10
    #Velocity observer inertia
    node.sdo[0x30A3][5].raw = inertia if inertia is not None else 1000


def dual_loop_position_control_parameter_init(node, main_loop_p_gain_low:int = None, main_loop_p_gain_high:int = None, 
                                         main_loop_gain_scheduler:int = None, main_loop_filter_coeff_a:int = None,
                                         main_loop_filter_coeff_b:int = None, main_loop_filter_coeff_c:int = None,
                                         main_loop_filter_coeff_d:int = None, main_loop_filter_coeff_e:int = None,
                                         auxiliary_loop_p_gain:int = None, auxiliary_loop_i_gain:int = None,
                                         auxiliary_loop_ff_velocity_gain:int = None, auxiliary_loop_ff_acceleration_gain:int = None,
                                         auxiliary_loop_observer_pos_corr_gain:int = None, auxiliary_loop_observer_vel_corr_gain:int = None,
                                         auxiliary_loop_observer_load_corr_gain:int = None, auxiliary_loop_observer_friction:int = None,
                                         auxiliary_loop_observer_inertia:int = None, dual_loop_miscellaneous:int = None):
    #6.2.65
    check_init(locals())
    #Represents the main loop low bandwidth proportional factor
    node.sdo[0x30AE][1].raw = main_loop_p_gain_low if main_loop_p_gain_low is not None else 10000
    #Represents the main loop high bandwidth proportional factor
    node.sdo[0x30AE][2].raw = main_loop_p_gain_high if main_loop_p_gain_high is not None else 100000
    #Represents the main loop gain scheduler
    node.sdo[0x30AE][3].raw = main_loop_gain_scheduler if main_loop_gain_scheduler is not None else 12500
    #Main loop filter coefficient a
    node.sdo[0x30AE][0x10].raw = main_loop_filter_coeff_a if main_loop_filter_coeff_a is not None else 1000
    #Main loop filter coefficient b
    node.sdo[0x30AE][0x11].raw = main_loop_filter_coeff_b if main_loop_filter_coeff_b is not None else 1000
    #Main loop filter coefficient c
    node.sdo[0x30AE][0x12].raw = main_loop_filter_coeff_c if main_loop_filter_coeff_c is not None else 1000
    #Main loop filter coefficient d
    node.sdo[0x30AE][0x13].raw = main_loop_filter_coeff_d if main_loop_filter_coeff_d is not None else 1000
    #Main loop filter coefficient e
    node.sdo[0x30AE][0x14].raw = main_loop_filter_coeff_e if main_loop_filter_coeff_e is not None else 1000
    #Auxiliary loop P-gain
    node.sdo[0x30AE][0x20].raw = auxiliary_loop_p_gain if auxiliary_loop_p_gain is not None else 20000
    #Auxiliary loop I-gain
    node.sdo[0x30AE][0x21].raw = auxiliary_loop_i_gain if auxiliary_loop_i_gain is not None else 500000
    #Auxiliary loop FF velocity gain
    node.sdo[0x30AE][0x22].raw = auxiliary_loop_ff_velocity_gain if auxiliary_loop_ff_velocity_gain is not None else 0
    #Auxiliary loop FF acceleration gain
    node.sdo[0x30AE][0x23].raw = auxiliary_loop_ff_acceleration_gain if auxiliary_loop_ff_acceleration_gain is not None else 0
    #Auxiliay loop observer position correction gain
    node.sdo[0x30AE][0x30].raw = auxiliary_loop_observer_pos_corr_gain if auxiliary_loop_observer_pos_corr_gain is not None else 400
    #Auxiliay loop observer velocity correction gain
    node.sdo[0x30AE][0x31].raw = auxiliary_loop_observer_vel_corr_gain if auxiliary_loop_observer_vel_corr_gain is not None else 100000
    #Auxiliay loop observer load correction gain
    node.sdo[0x30AE][0x32].raw = auxiliary_loop_observer_load_corr_gain if auxiliary_loop_observer_load_corr_gain is not None else 33
    #Auxiliay loop observer friction
    node.sdo[0x30AE][0x33].raw = auxiliary_loop_observer_friction if auxiliary_loop_observer_friction is not None else 10
    #Auxiliay loop observer inertia
    node.sdo[0x30AE][0x34].raw = auxiliary_loop_observer_inertia if auxiliary_loop_observer_inertia is not None else 1000
    #Dual loop control miscellaneous configuration
    node.sdo[0x30AE][0x40].raw = dual_loop_miscellaneous if dual_loop_miscellaneous is not None else 0x0000


def home_position_init(node, homeposition: int = None):
    #6.2.66
    check_init(locals())
    #defines the position that will be set as zero position of the absolute position counter.
    node.sdo[0x30B0][0x00].raw = homeposition if homeposition is not None else 0


def home_offset_distance_init(node, home_offset_distance: int = None):
    #6.2.67
    check_init(locals())
    #Represents a moving distance in a homing procedure.
    node.sdo[0x30B1][0x00].raw = home_offset_distance if home_offset_distance is not None else 0


def Current_threshold_homing_init(node, current_threshold_homing: int = None):
    #6.2.68
    check_init(locals())
    #Used for homing methods «−1», «−2», «−3», and «−4».
    node.sdo[0x30B2][0x00].raw = current_threshold_homing if current_threshold_homing is not None else 1000


def standstill_window_init(node, standstill_window: int = None):
    #6.2.73.1
    check_init(locals())
    #Defines a symmetric range of accepted velocity values relatively to zero.
    node.sdo[0x30E0][0x01].raw = standstill_window if standstill_window is not None else 30


def standstill_window_time_init(node, standstill_window_time: int = None):
    #6.2.73.2
    check_init(locals())
    #Defines the time duration for which the velocity must remain within the standstill window for Standstill to be reached. [ms]
    node.sdo[0x30E0][0x02].raw = standstill_window_time if standstill_window_time is not None else 2


def standstill_window_timeout_init(node, standstill_window_timeout: int = None):
    #6.2.73.3
    check_init(locals())
    #Defines the point of time standstill is supposed to be reached, even if the standstill conditions are not yet fulfilled.
    node.sdo[0x30E0][0x03].raw = standstill_window_timeout if standstill_window_timeout is not None else 1000


def abort_connection_option_init(node, abort_option: int = None):
    #6.2.92
    check_init(locals())
    #Specifies the action that will be performed when one of the errors labeled “a” is detected
    # 2 -> «Disable voltage» command
    # 3 -> Decelerate with quick stop ramp; disabling of the drive function
    node.sdo[0x605B][0x00].raw = abort_option if abort_option is not None else 3


def mode_of_operation_init(node, op_mode: int):
    #6.2.101
    check_init(locals())
    #Set mode of operation
    #default: 1 -> Profile position mode
    node.sdo[0x6060][0x00].raw = op_mode if op_mode is not None else 1 


def following_error_window_init(node, following_error_window: int = None):
    #6.2.105
    check_init(locals())
    #Defines the maximum allowed deviation between target and actual position.
    #min=0, max=2.147.483.647
    node.sdo[0x6065][0x00].raw = following_error_window if following_error_window is not None else 2000


def following_error_timeout_init(node, following_error_timeout: int = None):
    #6.2.106
    check_init(locals())
    # Indicates the configured time for a following error condition. If exceeded, a following error will occur.
    # The value is given in milliseconds [ms].
    node.sdo[0x6066][0x00].raw = following_error_timeout if following_error_timeout is not None else 0


def position_window_init(node, position_window: int = None):
    #6.2.107
    check_init(locals())
    #Defines a symmetric range of accepted position values relatively to target position.
    #min=0, max=2147483647, disable=4294967295
    node.sdo[0x6067][0x00].raw = position_window if position_window is not None else 4294967295


def position_window_time_init(node, position_window_time: int = None):
    #6.2.108
    check_init(locals())
    # Indicates the configured position window time for the target reached condition. If the actual position is within the Position
    # window during the set time, the corresponding bit 10 (target reached) in the Statusword will be set to “1”.
    # The value is given in milliseconds [ms].
    node.sdo[0x6068][0x00].raw = position_window_time if position_window_time is not None else 0


def shutdown_option_code(node, option_code: int = None):
    #6.2.97
    check_init(locals())
    #What action is performed when state transtitions from Operation enabled to ready to switch on
    #1 is decelerate with slowdown ramp and disabling of drive function, 0 is disable drive function
    node.sdo[0x605B].raw = option_code if option_code is not None else 0


def disable_operation_option_code(node, option_code: int = None):
    #6.2.98
    check_init(locals())
    #Action performed when state transitions from Operation Enabled to Switched on
    #1 is decelerate with slowdown ramp and disabling of drive function, 0 is disable drive function
    node.sdo[0x605C].raw = option_code if option_code is not None else 1


def halt_option_code(node, option_code: int = None):
    #6.2.99
    check_init(locals())
    #Action performed when halt function is activated
    #1 is decelerate with slowdown ramp and stay in operation enabled, 2 is decelerate with quick stop ramp and stay in operation enabled
    node.sdo[0x605D].raw = option_code if option_code is not None else 1


def fault_reaction_option_code(node, option_code: int = None):
    #6.2.100
    check_init(locals())
    #Action to be performed if one of the errors labeled in "f" will be detected, "f" contains most errors except communication errors
    #2 is decelerate with quickstop ramp and disabling of drive function, 1 is decelerate with slowdown ramp and disabling of drive function, 0 is disable drive function
    node.sdo[0x605E].raw = option_code if option_code is not None else 2


def target_torque(node, torque: int = None):
    #6.2.111
    check_init(locals())
    #indicates the configured target torque value for the controller in CST mode, value is given in per thousand of motor rated torque
    node.sdo[0x6071].raw = torque if torque is not None else 0


def motor_rated_torque(node, rated_torque: int = None):
    #6.2.112
    check_init(locals())
    #holds value to which all torque objects are related to, value is defined as nominal current * torque constant, value is in mNm
    node.sdo[0x6076].raw = rated_torque if rated_torque is not None else 0


def target_position(node, position: int = None):
    #6.2.114
    check_init(locals())
    #represents the position that the drive is supposed to move to using the motion control parameters
    node.sdo[0x607A].raw = position if position is not None else 0


def software_position_limit(node, min_pos_limit: int = None, max_pos_limit: int = None):
    #6.2.117
    check_init(locals())
    #defines the min and max allowed position values for the position controller, if exceeded a position error is generated
    #min position limit
    node.sdo[0x607D][1].raw = min_pos_limit if min_pos_limit is not None else 0
    #max position limit
    node.sdo[0x607D][2].raw = max_pos_limit if max_pos_limit is not None else 0


def max_profile_velocity(node, max_velocity: int = None):
    #6.2.118
    check_init(locals())
    #used as a velocity limit in a ppm or pvm move, value is given in rpm
    node.sdo[0x607F].raw = max_velocity if max_velocity is not None else 50000


def max_motor_speed(node, max_velocity: int = None):
    #6.2.119
    check_init(locals())
    #indicates maximum allowed motor speed, value is given in rpm
    node.sdo[0x6080].raw = max_velocity if max_velocity is not None else 50000


def profile_velocity(node, prof_velocity: int = None):
    #6.2.120
    check_init(locals())
    #represents the velocity normally attained at the end of the accleeratiion ramp during a profiled move (ppm, pvm), value is given in rpm
    node.sdo[0x6081].raw = prof_velocity if prof_velocity is not None else 1000


def profile_acceleration(node, prof_acc: int = None):
    #6.2.121
    check_init(locals())
    #defines the acceleration used during a profiled move (ppm, pvm), value is given in rpm/s
    node.sdo[0x6083].raw = prof_acc if prof_acc is not None else 10000


def profile_deceleration(node, prof_dec: int = None):
    #6.2.122
    check_init(locals())
    #defines the deceleration used during a profiled move (ppm, pvm), value is given in rpm/s
    node.sdo[0x6084].raw = prof_dec if prof_dec is not None else 10000


def quick_stop_deceleration(node, prof_dec: int = None):
    #6.2.123
    check_init(locals())
    #defines the deceleration used during a profiled move (ppm, pvm), value is given in rpm/s
    node.sdo[0x6085].raw = prof_dec if prof_dec is not None else 10000

def motion_profile_type(node, profile: int = None):
    #6.2.124
    check_init(locals())
    #defines the deceleration used during a profiled move (ppm, pvm), value is given in rpm/s
    node.sdo[0x6086].raw = profile if profile is not None else 0


def homing_method_init(node, homing_method: int = None):
    #6.2.125
    check_init(locals())
    #Used to select homing method (absolute SSI encoder = 37)
    node.sdo[0x6098].raw = homing_method if homing_method is not None else 37


def homing_speeds(node, speed_sw_srch: int = None, speed_zero_srch: int = None):
    #6.2.126
    check_init(locals())
    #Speed for switch search given in rpm
    node.sdo[0x6099][1].raw = speed_sw_srch if speed_sw_srch is not None else 100
    #Speed for zero search given in rpm, used to search the index in a homing sequence
    node.sdo[0x6099][2].raw = speed_zero_srch if speed_zero_srch is not None else 10


def homing_acceleration(node, homing_acc: int = None):
    #6.2.127
    check_init(locals())
    #Acceleration used during homing procedure, value is given in rpm/s
    node.sdo[0x609A].raw = homing_acc if homing_acc is not None else 1000


def si_unit_position(node, si_unit_pos: int = None):
    #6.2.128
    check_init(locals())
    #Defines the position units
    node.sdo[0x60A8].raw = si_unit_pos if si_unit_pos is not None else 0x00B50000


def si_unit_velocity(node, si_unit_vel: int = None):
    #6.2.129
    check_init(locals())
    #Defines the velocity units
    node.sdo[0x60A9].raw = si_unit_vel if si_unit_vel is not None else 0x00B44700


def si_unit_acceleration(node, si_unit_acc: int = None):
    #6.2.130
    check_init(locals())
    #Defines the acceleration units
    node.sdo[0x60AA].raw = si_unit_acc if si_unit_acc is not None else 0x00C00300


def position_offset(node, pos_offset: int = None):
    #6.2.131
    check_init(locals())
    #Defines an offset that is added to the actual position value
    node.sdo[0x60B0].raw = pos_offset if pos_offset is not None else 0


def velocity_offset(node, vel_offset: int = None):
    #6.2.132
    check_init(locals())
    #Defines an offset that is added to the actual velocity value
    node.sdo[0x60B1].raw = vel_offset if vel_offset is not None else 0


def torque_offset(node, tor_offset: int = None):
    #6.2.133
    check_init(locals())
    #Defines an offset that is added to the actual torque value
    node.sdo[0x60B2].raw = tor_offset if tor_offset is not None else 0


#touch probe 6.2.134 - 6.2.137 and 6.2.140 - 6.2.142 not implemented


def interpolation_time_period(node, time_period_val: int = None, time_index: int = None):
    #6.2.138
    check_init(locals())
    #interpolation time period value indicates the time between two PDOs, values > 0 enable the demand value interpolation in CSP and CSV
    #is is of importance that the setpoint is written cyclically with the inerpolation time period.
    #the value is given in s*10^time_index, value of 0 disables the demand value interpolation
    node.sdo[0x60C2][1].raw = time_period_val if time_period_val is not None else 0
    #time_index
    node.sdo[0x60C2][2].raw = time_index if time_index is not None else -3


def max_acceleration(node, max_acc: int = None):
    #6.2.139
    check_init(locals())
    #used to limit the max allowed acceleration to prevent mechanical damage, represents limit of all acceleration objects of the axis
    #IN CYCLIC MODES THIS VALUE IS NOT TAKEN INTO ACCOUNT
    node.sdo[0x60C5].raw = max_acc if max_acc is not None else 4294967295


def digital_outputs(node, phys_outputs: int = None):
    #6.2.148
    check_init(locals())
    #configures the state of the digital output functionalities, if a bit is set to “1“ and the polarity is set to “0“, the signal at the corresponding pin is high
    node.sdo[0x60FE][1].raw = phys_outputs if phys_outputs is not None else 0


def target_velocity(node, vel: int = None):
    #6.2.149
    check_init(locals())
    #in PVM the object indicates the configured target velocity and is used as input for the trajectory generation, value is given in rpm
    node.sdo[0x60FF].raw = vel if vel is not None else 0


def motor_type(node, motor_type: int = None):
    #6.2.150
    check_init(locals())
    #motor type (DC = 1, Sinusoidal PM BL motor = 10, Trapezoidal PM BL motor = 11)
    node.sdo[0x6402].raw = motor_type if motor_type is not None else 10
'''

# ======================================================================
# ======================================================================
# Region: DEVICE CONTROL FUNCTIONS
# ======================================================================
# ======================================================================


def cword_write(node, value: int):
    """
    Writes a value to Controlword.
    
    :param node: CANopen Network Node with Controlword instance
    :param value: Value of Controlword [binary]
    :type value: int
    """
    node.sdo[0x6040].raw = value


def cword_read(node) -> int:
    """
    Reads Controlword.
    
    :param node: CANopen Network Node with Controlword instance
    :return: returns Controlword. [binary]
    :rtype: int
    """
    return node.sdo[0x6040].raw


def sword(node) -> int:
    """
    Reads Statusword.
    
    :param node: CANopen Network Node with Statusword instance
    :return: returns Statusword. [binary]
    :rtype: int
    """
    return node.sdo[0x6041].raw


# ======================================================================
# Sub-Region: DRIVE STATE FUNCTIONS
# ======================================================================


class DriveState:
    """
    Represents the operational state of a drive system.

    This class defines a finite state machine with 8 distinct states that a drive
    can transition through, ranging from disabled/not ready to fully operational or fault
    conditions. Each state represents a specific operational mode or condition of the drive.

    Attributes:
        NOT_READY_TO_SWITCH_ON (int): Drive is not ready to be switched on (state 0).
        SWITCH_ON_DISABLED (int): Switching on is disabled (state 1).
        READY_TO_SWITCH_ON (int): Drive is ready to be switched on (state 2).
        SWITCHED_ON (int): Drive is switched on but not operating (state 3).
        OPERATION_ENABLED (int): Drive is actively operating (state 4).
        QUICK_STOP_ACTIVE (int): Quick stop is active, drive is stopping (state 5).
        FAULT_REACTION_ACTIVE (int): Drive is reacting to a fault condition (state 6).
        FAULT (int): Drive has encountered a fault (state 7).
        state_flags (dict): Mapping of state indices to boolean flags for state tracking.
    """ 
    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED     = 1
    READY_TO_SWITCH_ON     = 2
    SWITCHED_ON            = 3
    OPERATION_ENABLED      = 4
    QUICK_STOP_ACTIVE      = 5
    FAULT_REACTION_ACTIVE  = 6
    FAULT                  = 7
    

def get_DriveState(node) -> DriveState:
    """
    Determine the drive state based on the statusword bits.
    This function extracts specific bits from a statusword obtained from a node
    and maps the bit pattern to a corresponding DriveState enum value. It evaluates
    the state machine bits (Ready to switch on, Switched on, Operation enabled, Fault,
    Quick stop, and Switch on disabled) to determine the current drive operational state.


    Args:
        node (int): The node object from which to retrieve the statusword.

    Raises:
        DriveStateDetError: If the statusword bit pattern does not match any known 
        drive state, indicating an invalid or unexpected state combination.
    
    Returns:
        DriveState:  An enum value representing the current drive state, which can be:
                        - NOT_READY_TO_SWITCH_ON
                        - SWITCH_ON_DISABLED
                        - READY_TO_SWITCH_ON
                        - SWITCHED_ON
                        - OPERATION_ENABLED
                        - QUICK_STOP_ACTIVE
                        - FAULT_REACTION_ACTIVE
                        - FAULT
    """      
    statusword = sword(node)

    b0 = (statusword >> 0) & 1  # Ready to switch on (compares right most bit)
    b1 = (statusword >> 1) & 1  # Switched on (right most bit falls off, now compare second bit)
    b2 = (statusword >> 2) & 1  # Operation enabled
    b3 = (statusword >> 3) & 1  # Fault
    b5 = (statusword >> 5) & 1  # Quick stop
    b6 = (statusword >> 6) & 1  # Switch on disabled

    # Not ready to switch on
    if (b6 == 0 and b5 == 0 and b3 == 0 and b2 == 0 and b1 == 0 and b0 == 0):
        return DriveState.NOT_READY_TO_SWITCH_ON

    # Switch on disabled
    if (b6 == 1 and b5 == 0 and b3 == 0 and b2 == 0 and b1 == 0 and b0 == 0):
        return DriveState.SWITCH_ON_DISABLED

    # Ready to switch on
    if (b6 == 0 and b5 == 1 and b3 == 0 and b2 == 0 and b1 == 0 and b0 == 1):
        return DriveState.READY_TO_SWITCH_ON

    # Switched on
    if (b6 == 0 and b5 == 1 and b3 == 0 and b2 == 0 and b1 == 1 and b0 == 1):
        return DriveState.SWITCHED_ON

    # Operation enabled
    if (b6 == 0 and b5 == 1 and b3 == 0 and b2 == 1 and b1 == 1 and b0 == 1):
        return DriveState.OPERATION_ENABLED

    # Quick stop active
    if (b6 == 0 and b5 == 0 and b3 == 0 and b2 == 1 and b1 == 1 and b0 == 1):
        return DriveState.QUICK_STOP_ACTIVE

    # Fault reaction active
    if (b6 == 0 and b5 == 1 and b3 == 1 and b2 == 1 and b1 == 1 and b0 == 1):
        return DriveState.FAULT_REACTION_ACTIVE

    # Fault
    if (b6 == 0 and b5 == 1 and b3 == 1 and b2 == 0 and b1 == 0 and b0 == 0):
        return DriveState.FAULT
    
    raise DriveStateDetError(f"Unknown drive state with statusword: {statusword:#04x}")
    

# ======================================================================
# Sub-Region: DRIVE COMMAND FUNCTIONS
# ======================================================================

class DriveCommand: 
    """
    Enumeration of CiA 402 drive commands for motor control operations.

    This class defines standard control commands used to manage the operational state
    of a drive, including startup, shutdown, and fault handling procedures following
    the CANopen DS402 device profile specification.
    """
    
    SHUTDOWN            = 0x006 #maybe normal ints
    SWITCH_ON           = 0x007
    ENABLE_OPERATION    = 0x00F
    DISABLE_VOLTAGE     = 0x000
    QUICK_STOP          = 0x002
    DISABLE_OPERATION   = 0x007           
    FAULT_RESET         = 0x100


DriveStateMap = {DriveState.NOT_READY_TO_SWITCH_ON : [(DriveCommand.SWITCH_ON, DriveState.OPERATION_ENABLED)], 
                  DriveState.SWITCH_ON_DISABLED     : [(DriveCommand.SHUTDOWN, DriveState.READY_TO_SWITCH_ON)], 
                  DriveState.READY_TO_SWITCH_ON     : [(DriveCommand.DISABLE_VOLTAGE, DriveState.SWITCH_ON_DISABLED), 
                                                       (DriveCommand.SWITCH_ON, DriveState.SWITCHED_ON), 
                                                       (DriveCommand.ENABLE_OPERATION, DriveState.OPERATION_ENABLED)],
                  DriveState.SWITCHED_ON            : [(DriveCommand.SHUTDOWN, DriveState.READY_TO_SWITCH_ON),
                                                       (DriveCommand.DISABLE_OPERATION, DriveState.SWITCH_ON_DISABLED), 
                                                       (DriveCommand.ENABLE_OPERATION, DriveState.OPERATION_ENABLED)],
                  DriveState.OPERATION_ENABLED      : [(DriveCommand.DISABLE_OPERATION, DriveState.SWITCHED_ON), 
                                                       (DriveCommand.SHUTDOWN, DriveState.READY_TO_SWITCH_ON), 
                                                       (DriveCommand.DISABLE_VOLTAGE, DriveState.SWITCH_ON_DISABLED), 
                                                       (DriveCommand.QUICK_STOP, DriveState.QUICK_STOP_ACTIVE)],
                  DriveState.QUICK_STOP_ACTIVE      : [(DriveCommand.ENABLE_OPERATION, DriveState.OPERATION_ENABLED), 
                                                       (DriveCommand.DISABLE_VOLTAGE, DriveState.SWITCH_ON_DISABLED)],
                  DriveState.FAULT                  : [(DriveCommand.FAULT_RESET, DriveState.SWITCH_ON_DISABLED)]}


def fault_reset(node, reset_tries: int = 1, timeout: float = 5.0) -> None:
    """Resets the drive from a FAULT state to SWITCH_ON_DISABLED state.
    
    Args:
        node: The CANopen node representing the drive.
        reset_tries (int): Number of attempts to reset the fault.
        timeout (float): Maximum time to wait for the reset to complete.
        
    Raises:
        DriveStateResetError: If the drive does not reach SWITCH_ON_DISABLED state after the specified number of attempts.
    """
    for _ in range(reset_tries):
        current_cword = cword_read(node)
        new_cword = current_cword | 0x0080
        cword_write(node, new_cword)
        time.sleep(0.1)
        if get_DriveState(node) ==  DriveState.SWITCH_ON_DISABLED:
            return
    raise DriveStateResetError(f"Failed to reach Fault_RESET in {reset_tries} attempts")


def wait_for_state(node, desired_state: int, timeout: float = 5.0):
    """Gives the drive some time to reach the desired state.
    
    Args:
        node: The CANopen node representing the drive.
        desired_state (int): The target drive state to wait for.
        timeout (float): Maximum time to wait for the desired state.
        
    Raises:
        TimeoutError: If the drive does not reach the desired state within the timeout period.
    """
    
    start_time = time.time()

    while True:                                                                    
        current_state = get_DriveState(node) 
        if current_state == DriveState.FAULT: 
            print("ATTENTION: Fault Reaction Active. Reset Drive? (3 tries)")
            if confirm():
                fault_reset(node, reset_tries=3, timeout=timeout)
                time.sleep(0.05)
        if current_state == desired_state:
            return True
        if time.time() - start_time > timeout:
            raise TimeoutError(f"Timeout waiting for state {desired_state}, current state is {current_state}")
        time.sleep(0.01)  # Avoid busy waiting


def do_DriveCommand(node, command: int, target: DriveState, timeout: int) -> True:
    """
    Sets the DS402 Controlword for the given command on node and waits until the drive reaches the target state within timeout. 
    Special-cases FAULT_RESET by calling fault_reset() instead of writing the normal command bits.
    
    Args:
        node (int): EPOS4 Node
        command (DriveCommand): desired DriveCommand
        target (DriveState): expected DriveState after completing function
        timeout (int)

    Raises:
        TimeoutError: raises if State cannot be switched within timeout

    Returns:
        True: returns True if successfull, else Exception error is raised.
    """    
    current_cword = cword_read(node)

    # Special case: Fault reset (set bit 7, keep all other bits)
    if command == DriveCommand.FAULT_RESET:
        fault_reset(node, reset_tries=3, timeout=timeout)
        return True

    low  = command & 0x00FF #Otto sagt aufpassen!
    high = current_cword & 0xFF00
    new_cword = high | low

    cword_write(node, new_cword)

    if(wait_for_state(node, target, timeout)):
        return #True

    raise TimeoutError(f"Failed to reach state {target} in time")


def shutdown_drive(node):
    current_state = get_DriveState()

    if current_state == DriveState.OPERATION_ENABLED or current_state == DriveState.QUICK_STOP_ACTIVE:
        do_DriveCommand(DriveCommand.DISABLE_VOLTAGE)
    elif current_state == DriveState.SWITCHED_ON:
        do_DriveCommand(DriveCommand.SHUTDOWN)
    

def Drive_State_BFS(start_state, target_state, Drive_State_map):
    """Naive breadth-firs-search algorithm to traverse DriveState map.  
       ATTENTION: THIS FUNCTION IS 100% WRITTEN BY AI

    Args:
        start_state (DriveState): Current drive state.
        target_state (DriveState): Desired drive state.
        drive_state_map (dict): Mapping {state: [(command, next_state), ...]}.

    Returns:
        list[tuple[DriveCommand, DriveState]] | None:
            A route as (command, reached_state) steps, or None if no path exists.
    """    
    
    if start_state == target_state:
        return []
    
    queue = [start_state]
    visited = {start_state}
    parent = {}

    while queue:
        current = queue.pop(0)

        if current == target_state:
            break
        
        for command, next_state in Drive_State_map.get(current, []):
            if next_state not in visited:
                visited.add(next_state)
                parent[next_state] = (current, command)
                queue.append(next_state)

    
    if target_state not in visited:
        return None
    
    route = []
    state = target_state

    while state != start_state:
        prev_state, command = parent[state]
        route.append((command, state))
        state = prev_state

    route.reverse()
    return route


def goto_state(node, desired_state, timeout):
    """Traverses state-machine to get to desired DriveState

    Args:
        node (int): EPOS4 node
        desired_state (DriveState): State to be reached.
        timeout (int): timeout for do_DriveCommand()

    Raises:
        DriveStatePathError: raised if route cannot be found/is not valid
        DesiredDriveStateError: raised if goto_state() fails generally
    """   
    current_state = get_DriveState(node)
    
    route = Drive_State_BFS(current_state, desired_state, DriveStateMap)

    if route is None:
        raise DriveStatePathError(f"No valid path to {desired_state} was found")
    
    for step in route:
        next_command = step[0]
        next_state = step[1]
        try:
            do_DriveCommand(node, next_command, next_state, timeout)
        except DriveStateDetError as e:
            # Specific drive-state errors you expect
            shutdown_drive()
            print(f"Drive state determination error: {e}")
            raise
        except TimeoutError as e:
            # Timeout errors from nested calls
            shutdown_drive()
            print(f"Timeout during state transition: {e}")
            raise
        except Exception as e:
            # Unexpected errors—log and fail fast
            shutdown_drive()
            print(f"Unexpected error: {e} current State: {get_DriveState(node)}")
            raise
        finally:
           # Cleanup that always runs: e.g., disable watchdog, log state, etc.
            print(f"Attempted transition to {next_state}")

        
    final_state = get_DriveState(node)
    
    if final_state == desired_state:
        return
    else:
        raise DesiredDriveStateError(f"{desired_state} state could not be reached. Current state: {final_state}")


# ======================================================================
# Region: OPERATING MODES
# ======================================================================

with open('/home/amfluxpi/AMflux/src/amflux/app/object_dictionary.toml', 'r') as data:
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


def drive_run(node, desired_op_mode, timeout, operation_time):
    """Checks if OD is initialized for the desired op mode and runs it if so. Otherwise states OD not initialized
    
    Args:
        node: The CANopen node representing the drive.
        desired_op_mode: The desired operation mode to run.
        timeout: Maximum time to wait for state transitions.
        operation_time: Time to maintain the desired operation state.
    """
    #TODO: power check? arduino ok check? Mode of operation write and display. adjust TOML file for COMM and CONF

    if init_obj_dict(node, desired_op_mode):
        mode_code = OperationModes.abreviation[desired_op_mode]
        for func_name, instance in objdict_data["mode"][mode_code]["comm"].items():
            func = globals().get(object_dictionary_functions.func_name)
            kwargs = {}
            for variable, default_val in instance.items(): 
                user_val = input(f"Please enter value for {func_name} -> {variable}.\n Press Enter, if {default_val} is ok.").strip()
                if user_val == "":
                    write_val = default_val
                else:
                    try:
                        write_val = int(user_val)
                    except Warning:
                        try:
                            write_val = int(input(f"please enter a valid INT64 value for {variable}").strip())
                        except Exception:
                            print(f"invalid value for {variable}, using default value: {default_val}")
                    
                kwargs[variable] = write_val
            
            func(node, **kwargs)

        goto_state(node, desired_state=4, timeout=timeout)
        start_time = time.time()
        while (current_time - start_time) < operation_time:
            current_time = time.time()
            #TODO: continous Control word and commanding params handling. Fault mode watching
            time.sleep()
          



        shutdown_drive(node)
        return
    else:
        print("Object Dicitonary not initialized")
        return





# ======================================================================
# Region: Main
# ======================================================================

def main():
    global net
    # Setup our CANopen network
    network, mc1 = can_functions.network_setup(1, '/home/amfluxpi/AMflux/src/amflux/app/Epos4_70_15.eds', 'can0')
    
    can_functions.sanity_check(network)

    drive_run(mc1, OperationModes.CyclicSynchronousPosition, timeout=5, operation_time=5)

    print("Drive completed run. Network will be shutdown")

    can_functions.network_shutdown()

    '''
    # CST mode is under int=10 (Firmware-Specification, p219)
    motor_setup(mc1, 10)
    # Run the motor in cst mode using a given amount of rpm
    motor_run_cst(mc1, 1000, 2)
    # Shutdown the CANopen network after use
    network_shutdown()
    '''
    

if __name__ == "__main__":
    #main()
    #network_scan('can0')
    main()

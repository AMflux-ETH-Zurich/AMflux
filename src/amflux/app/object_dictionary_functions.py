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

Touch Probe (6.2.134-6.2.137, 6.2.140-6.2.142) not implemented.
"""


# ======================================================================
# Imports
# ======================================================================

import time
from utils import check_init


# ======================================================================
# PDO mapping
# ======================================================================

def pdo_mapping_init(node, network):
    """initialization of PDO's for CAN network.

    Args:
        node (RemoteNode): EPOS4 network node
        network (Network): CAN network
    """    
    print(f'ATTENTION: node NMT state: {node.nmt.state}')    
    # Reset network
    node.nmt.state = 'RESET COMMUNICATION'
    node.nmt.wait_for_bootup(2)
    print(f'node NMT state: {node.nmt.state}')

    # Read current PDO configuration (must be done, otherwise library only has standard mapping, despite setting new mapping and saved it...?)
    node.tpdo.read(from_od = True)    # Read PDOs from the online object dictionary
    node.rpdo.read(from_od = True)

    # clear all PDOs, we will configure them from scratch:
    for no in range(1, 5):
        node.tpdo[no].clear()
        node.rpdo[no].clear()

    # initialize Tpdo's
    node.tpdo[1].add_variable("Statusword") # Statusword
    node.tpdo[1].add_variable("Controlword")
    node.tpdo[1].add_variable("Modes of operation display")
    node.tpdo[1].trans_type = 255 #   1=SYNC; 255=asynchronous --> with every change of the 402 state machine (in 0x2400.04, bit mask 0x00000002 must be set (which is the default))
    node.tpdo[1].enabled = True

    node.tpdo[2].add_variable("Velocity actual value") # Velocity actual value
    node.tpdo[2].add_variable("Position actual value") # Position actual value
    node.tpdo[2].trans_type = 1 #   1=SYNC; 255=asynchronous
    node.tpdo[2].enabled = True

    node.tpdo[3].add_variable("Current actual values.Current actual value") # Current actual value
    node.tpdo[3].trans_type = 1 #   1=SYNC; 255=asynchronous
    node.tpdo[3].enabled = True

    # initialize Rpdo's    
    node.rpdo[1].add_variable("Controlword")  # Controlword
    node.rpdo[1].add_variable("Modes of operation")
    node.rpdo[1].trans_type = 255 #   1=SYNC; 255=asynchronous
    node.rpdo[1].enabled = True
    
    node.rpdo[2].add_variable("Target velocity")  # Target velocity
    node.rpdo[2].add_variable("Target position")  # Target position
    node.rpdo[2].trans_type = 255 #   1=SYNC; 255=asynchronous
    node.rpdo[2].enabled = True
    
    # enter pre-operational network state and save mappings
    node.nmt.state = 'PRE-OPERATIONAL'
    node.tpdo.save()
    node.rpdo.save()  
    time.sleep(1)
    # enter operational state
    node.nmt.state = 'OPERATIONAL' 
    #time.sleep(5)
    print(f'node NMT state: {node.nmt.state}')     


# ======================================================================
# Object dictionary initializaion via SDO's
# ======================================================================

def axis_configuration_init(node, sens_res: int=None , sys_speed: int=None): 
    #6.2.52
    #Axis configuration for absolute SSI encoder
    node.sdo["Axis configuration.Sensors configuration"].raw = 0x00000300
    #Axis control structure
    node.sdo["Axis configuration.Control structure"].raw = 0x00020111
    #Commutaton Sensors
    node.sdo["Axis configuration.Commutation sensors"].raw = 0x00000020
    #Miscellaneous Axis Configuration
    node.sdo["Axis configuration.Axis configuration miscellaneous"].raw = 0x00000000


def motor_init(node, motor_type: int=None, nominal_current: int=None, current_lim: int=None, 
                        pole_pairs: int=None, therm_const: int=None, tor_const: int=None):
    
    
    check_init(locals())
    #6.2.53
    #motor type (Sinusoidal PM BL motor = 10 or in hex 0x0A)
    node.sdo["Motor type"].raw = motor_type if motor_type is not None else 0x000A
    #nominal current flowing through motor windings in mA
    node.sdo["Motor data.Nominal current"].raw = nominal_current if nominal_current is not None else 15000
    #output current limit in mA
    node.sdo["Motor data.Output current limit"].raw = current_lim if current_lim is not None else 30000
    #number of pole pairs
    node.sdo["Motor data.Number of pole pairs"].raw = pole_pairs if pole_pairs is not None else 10
    #thermal time constant of windings
    node.sdo["Motor data.Thermal time constant winding"].raw = therm_const if therm_const is not None else 40
    #torque constant of motor
    node.sdo["Motor data.Torque constant"].raw = tor_const if tor_const is not None else 0


def ssi_abs_encoder_init(node, data_rate: int=None, data_bits: int=None, encoding_type: int=None,
                                timeout_time: int=None, power_up_time: int=None, commutation_offset_value: int=None,
                                        position_bits: int=None, communication_additional_delay: int=None):
    #6.2.58
    check_init(locals())
    #SSI data rate
    node.sdo["SSI absolute encoder.SSI data rate"].raw = data_rate if data_rate is not None else 1000 #REFER TO RENISHAW ENCODER SHEET
    #SSI number of data bits
    node.sdo["SSI absolute encoder.SSI number of data bits"].raw = data_bits if data_bits is not None else 0x00000C00 #REFER TO RENISHAW ENCODER SHEET
    #SSI encoding type
    node.sdo["SSI absolute encoder.SSI encoding type"].raw = encoding_type if encoding_type is not None else 0x001 #REFER TO RENISHAW ENCODER SHEET
    #SSI timeout time
    node.sdo["SSI absolute encoder.SSI timeout time"].raw = timeout_time if timeout_time is not None else 30 #REFER TO RENISHAW ENCODER SHEET
    #Special bits trailing data
    #READ ONLY node.sdo[0x3012][0x6].raw = sbits_trailing_data if sbits_trailing_data is not None else 0 #REFER TO RENISHAW ENCODER SHEET
    #SSI refresh frequency
    #READ ONLY node.sdo[0x3012][0x7].raw = refresh_frequency #REFER TO RENISHAW ENCODER SHEET
    #SSI Power up time
    node.sdo["SSI absolute encoder.SSI power up time"].raw = power_up_time if power_up_time is not None else 200 #REFER TO RENISHAW ENCODER SHEET
    #SSI postition raw value, lower 32 bits
    #READ ONLY node.sdo[0x3012][0x9].raw = position_raw_value
    #SSI commutaiton offset value
    node.sdo["SSI absolute encoder.SSI commutation offset value"].raw = commutation_offset_value if commutation_offset_value is not None else 0
    #SSI position bits
    node.sdo["SSI absolute encoder.SSI position bits"].raw = position_bits if position_bits is not None else  0x0000000C #REFER TO RENISHAW ENCODER SHEET, MUST BE REDUCED IF >32
    #SSI special bits leading data
    #READ ONLY node.sdo[0x3012][0xC].raw = sbits_leading_data if sbits_leading_data is not None else 0 #REFER TO RENISHAW ENCODER SHEET
    #SSI position raw value complemete
    #READ ONLY node.sdo[0x3012][0xD].raw = position_raw_value_complete
    #SSI communication additional delay
    #node.sdo["SSI absolute encoder.ssi communication additional delay"].raw = communication_additional_delay if communication_additional_delay is not None else -1 #REFER TO RENISHAW EN


def electrical_system_init(node, electrical_resistance: int=None, electrical_inductance: int=None):
    #6.2.54
    check_init(locals())
    #electrical resistance of motor windings in mOhm
    node.sdo["Electrical system parameters.Electrical resistance"].raw = electrical_resistance if electrical_resistance is not None else 0
    #electrical inductance of motor windings in uH
    node.sdo["Electrical system parameters.Electrical inductance"].raw = electrical_inductance if electrical_inductance is not None else 0


def current_control_parameter_init(node, p_gain: int = None, i_gain: int = None):
    #6.2.61
    check_init(locals())
    #Current controller P-gain
    node.sdo["Current control parameter set.Current controller P gain"].raw = p_gain if p_gain is not None else 1171880
    #Current controller I-gain
    node.sdo["Current control parameter set.Current controller I gain"].raw = i_gain if i_gain is not None else 3906250


def position_control_parameter_init(node, p_gain:int = None, i_gain: int = None, d_gain:int = None,
                               ffv_gain: int = None, ffa_gain: int = None, i_gain_si_unit: int = None):
    #6.2.62
    check_init(locals())
    #Position controller P-gain
    node.sdo["Position control parameter set.Position controller P gain"].raw = p_gain if p_gain is not None else 1500000
    #Position controller I-gain
    node.sdo["Position control parameter set.Position controller I gain"].raw = i_gain if i_gain is not None else 780000
    #Position controller D-gain
    node.sdo["Position control parameter set.Position controller D gain"].raw = d_gain if d_gain is not None else 16000
    #Position controller Feed Forward velocity gain
    node.sdo["Position control parameter set.Position controller FF velocity gain"].raw = ffv_gain if ffv_gain is not None else 0
    #Position controller Feed Forward acceleration gain
    node.sdo["Position control parameter set.Position controller FF acceleration gain"].raw = ffa_gain if ffa_gain is not None else 0
    #Position controller I-gain SI units
    node.sdo["Position control parameter set.SI unit position controller I gain"].raw = i_gain_si_unit if i_gain_si_unit is not None else 0xFA040300


def velocity_control_parameter_init(node, p_gain:int = None, i_gain: int = None, 
                               ffv_gain: int = None, ffa_gain: int = None, f_cutoff: int = None):
    #6.2.63
    check_init(locals())
    #Velocity controller P-gain
    node.sdo["Velocity control parameter set.Velocity controller P gain"].raw = p_gain if p_gain is not None else 20000
    #Velocity controller I-gain
    node.sdo["Velocity control parameter set.Velocity controller I gain"].raw = i_gain if i_gain is not None else 500000
    #Velocity controller Feed Forward velocity gain
    node.sdo["Velocity control parameter set.Velocity controller FF velocity gain"].raw = ffv_gain if ffv_gain is not None else 0
    #Velocity controller Feed Forward acceleration gain
    node.sdo["Velocity control parameter set.Velocity controller FF acceleration gain"].raw = ffa_gain if ffa_gain is not None else 0
    #Velocity controller filter cut-off frequency in HZ
    node.sdo["Velocity control parameter set.Velocity controller filter cut-off frequency"].raw = f_cutoff if f_cutoff is not None else 600
    

def velocity_observer_parameter_init(node, pos_corr_gain: int = None, vel_corr_gain: int = None,
                                load_corr_gain: int = None, friction: int = None, inertia: int = None
                                ):
    #6.2.64
    check_init(locals())
    #Velocity observer position correction gain given in promille
    node.sdo["Velocity observer parameter set.Velocity observer position correction gain"].raw = pos_corr_gain if pos_corr_gain is not None else 400
    #Velocity observer velocity correction gain given in mHz
    node.sdo["Velocity observer parameter set.Velocity observer velocity correction gain"].raw = vel_corr_gain if vel_corr_gain is not None else 100000
    #Velocity observer load correction gain
    node.sdo["Velocity observer parameter set.Velocity observer load correction gain"].raw = load_corr_gain if load_corr_gain is not None else 33
    #Velocity observer friction
    node.sdo["Velocity observer parameter set.Velocity observer friction"].raw = friction if friction is not None else 10
    #Velocity observer inertia
    node.sdo["Velocity observer parameter set.Velocity observer inertia"].raw = inertia if inertia is not None else 1000


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
    node.sdo["Dual loop position control parameter set.Main loop P gain low bandwidth"].raw = main_loop_p_gain_low if main_loop_p_gain_low is not None else 10000
    #Represents the main loop high bandwidth proportional factor
    node.sdo["Dual loop position control parameter set.Main loop P gain high bandwidth"].raw = main_loop_p_gain_high if main_loop_p_gain_high is not None else 100000
    #Represents the main loop gain scheduler
    node.sdo["Dual loop position control parameter set.Main loop gain scheduling weight"].raw = main_loop_gain_scheduler if main_loop_gain_scheduler is not None else 12500
    #Main loop filter coefficient a
    node.sdo["Dual loop position control parameter set.Main loop filter coefficient a"].raw = main_loop_filter_coeff_a if main_loop_filter_coeff_a is not None else 1000
    #Main loop filter coefficient b
    node.sdo["Dual loop position control parameter set.Main loop filter coefficient b"].raw = main_loop_filter_coeff_b if main_loop_filter_coeff_b is not None else 1000
    #Main loop filter coefficient c
    node.sdo["Dual loop position control parameter set.Main loop filter coefficient c"].raw = main_loop_filter_coeff_c if main_loop_filter_coeff_c is not None else 1000
    #Main loop filter coefficient d
    node.sdo["Dual loop position control parameter set.Main loop filter coefficient d"].raw = main_loop_filter_coeff_d if main_loop_filter_coeff_d is not None else 1000
    #Main loop filter coefficient e
    node.sdo["Dual loop position control parameter set.Main loop filter coefficient e"].raw = main_loop_filter_coeff_e if main_loop_filter_coeff_e is not None else 1000
    #Auxiliary loop P-gain
    node.sdo["Dual loop position control parameter set.Auxiliary loop P gain"].raw = auxiliary_loop_p_gain if auxiliary_loop_p_gain is not None else 20000
    #Auxiliary loop I-gain
    node.sdo["Dual loop position control parameter set.Auxiliary loop I gain"].raw = auxiliary_loop_i_gain if auxiliary_loop_i_gain is not None else 500000
    #Auxiliary loop FF velocity gain
    node.sdo["Dual loop position control parameter set.Auxiliary loop FF velocity gain"].raw = auxiliary_loop_ff_velocity_gain if auxiliary_loop_ff_velocity_gain is not None else 0
    #Auxiliary loop FF acceleration gain
    node.sdo["Dual loop position control parameter set.Auxiliary loop FF acceleration gain"].raw = auxiliary_loop_ff_acceleration_gain if auxiliary_loop_ff_acceleration_gain is not None else 0
    #Auxiliay loop observer position correction gain
    node.sdo["Dual loop position control parameter set.Auxiliary loop observer position correction gain"].raw = auxiliary_loop_observer_pos_corr_gain if auxiliary_loop_observer_pos_corr_gain is not None else 400
    #Auxiliay loop observer velocity correction gain
    node.sdo["Dual loop position control parameter set.Auxiliary loop observer velocity correction gain"].raw = auxiliary_loop_observer_vel_corr_gain if auxiliary_loop_observer_vel_corr_gain is not None else 100000
    #Auxiliay loop observer load correction gain
    node.sdo["Dual loop position control parameter set.Auxiliary loop observer load correction gain"].raw = auxiliary_loop_observer_load_corr_gain if auxiliary_loop_observer_load_corr_gain is not None else 33
    #Auxiliay loop observer friction
    node.sdo["Dual loop position control parameter set.Auxiliary loop observer friction"].raw = auxiliary_loop_observer_friction if auxiliary_loop_observer_friction is not None else 10
    #Auxiliay loop observer inertia
    node.sdo["Dual loop position control parameter set.Auxiliary loop observer inertia"].raw = auxiliary_loop_observer_inertia if auxiliary_loop_observer_inertia is not None else 1000
    #Dual loop control miscellaneous configuration
    node.sdo["Dual loop position control parameter set.Dual loop configuration miscellaneous"].raw = dual_loop_miscellaneous if dual_loop_miscellaneous is not None else 0x0000


def home_position_init(node, homeposition: int = None):
    #6.2.66
    check_init(locals())
    #defines the position that will be set as zero position of the absolute position counter.
    node.sdo["Home position"].raw = homeposition if homeposition is not None else 0


def home_offset_distance_init(node, home_offset_distance: int = None):
    #6.2.67
    check_init(locals())
    #Represents a moving distance in a homing procedure.
    node.sdo["Home offset move distance"].raw = home_offset_distance if home_offset_distance is not None else 0


def current_threshold_homing_init(node, current_threshold_homing: int = None):
    #6.2.68
    check_init(locals())
    #Used for homing methods «−1», «−2», «−3», and «−4».
    node.sdo["Current threshold for homing mode"].raw = current_threshold_homing if current_threshold_homing is not None else 1000


def standstill_window_init(node, standstill_window: int = None):
    #6.2.73.1
    check_init(locals())
    #Defines a symmetric range of accepted velocity values relatively to zero.
    node.sdo["Standstill window configuration.Standstill window"].raw = standstill_window if standstill_window is not None else 30


def standstill_window_time_init(node, standstill_window_time: int = None):
    #6.2.73.2
    check_init(locals())
    #Defines the time duration for which the velocity must remain within the standstill window for Standstill to be reached. [ms]
    node.sdo["Standstill window configuration.Standstill window time"].raw = standstill_window_time if standstill_window_time is not None else 2


def standstill_window_timeout_init(node, standstill_window_timeout: int = None):
    #6.2.73.3
    check_init(locals())
    #Defines the point of time standstill is supposed to be reached, even if the standstill conditions are not yet fulfilled.
    node.sdo["Standstill window configuration.Standstill window timeout"].raw = standstill_window_timeout if standstill_window_timeout is not None else 1000


def abort_connection_option_init(node, abort_option: int = None):
    #6.2.92
    check_init(locals())
    #Specifies the action that will be performed when one of the errors labeled “a” is detected
    # 2 -> «Disable voltage» command
    # 3 -> Decelerate with quick stop ramp; disabling of the drive function
    node.sdo["Abort connection option code"].raw = abort_option if abort_option is not None else 1


def mode_of_operation_init(node, op_mode: int):
    #6.2.101
    check_init(locals())
    #Set mode of operation
    #default: 1 -> Profile position mode
    node.sdo["Modes of operation"].raw = op_mode if op_mode is not None else 1 


def following_error_window_init(node, following_error_window: int = None):
    #6.2.105
    check_init(locals())
    #Defines the maximum allowed deviation between target and actual position.
    #min=0, max=2.147.483.647
    node.sdo["Following error window"].raw = following_error_window if following_error_window is not None else 2000


def following_error_timeout_init(node, following_error_timeout: int = None):
    #6.2.106
    check_init(locals())
    # Indicates the configured time for a following error condition. If exceeded, a following error will occur.
    # The value is given in milliseconds [ms].
    node.sdo["Following error time out"].raw = following_error_timeout if following_error_timeout is not None else 0


def position_window_init(node, position_window: int = None):
    #6.2.107
    check_init(locals())
    #Defines a symmetric range of accepted position values relatively to target position.
    #min=0, max=2147483647, disable=4294967295
    node.sdo["Position window"].raw = position_window if position_window is not None else 4294967295


def position_window_time_init(node, position_window_time: int = None):
    #6.2.108
    check_init(locals())
    # Indicates the configured position window time for the target reached condition. If the actual position is within the Position
    # window during the set time, the corresponding bit 10 (target reached) in the Statusword will be set to “1”.
    # The value is given in milliseconds [ms].
    node.sdo["Position window time"].raw = position_window_time if position_window_time is not None else 0


def shutdown_option_code(node, option_code: int = None):
    #6.2.97
    check_init(locals())
    #What action is performed when state transtitions from Operation enabled to ready to switch on
    #1 is decelerate with slowdown ramp and disabling of drive function, 0 is disable drive function
    node.sdo["Shutdown option code"].raw = option_code if option_code is not None else 0


def disable_operation_option_code(node, option_code: int = None):
    #6.2.98
    check_init(locals())
    #Action performed when state transitions from Operation Enabled to Switched on
    #1 is decelerate with slowdown ramp and disabling of drive function, 0 is disable drive function
    node.sdo["Disable operation option code"].raw = option_code if option_code is not None else 1

"""
def halt_option_code(node, option_code: int = None):
    #6.2.99
    check_init(locals())
    #Action performed when halt function is activated
    #1 is decelerate with slowdown ramp and stay in operation enabled, 2 is decelerate with quick stop ramp and stay in operation enabled
    node.sdo[0x605D].raw = option_code if option_code is not None else 1
"""

def fault_reaction_option_code(node, option_code: int = None):
    #6.2.100
    check_init(locals())
    #Action to be performed if one of the errors labeled in "f" will be detected, "f" contains most errors except communication errors
    #2 is decelerate with quickstop ramp and disabling of drive function, 1 is decelerate with slowdown ramp and disabling of drive function, 0 is disable drive function
    node.sdo["Fault reaction option code"].raw = option_code if option_code is not None else 2


def target_torque(node, torque: int = None):
    #6.2.111
    check_init(locals())
    #indicates the configured target torque value for the controller in CST mode, value is given in per thousand of motor rated torque
    node.sdo["Target torque"].raw = torque if torque is not None else 0


def motor_rated_torque(node, rated_torque: int = None):
    #6.2.112
   check_init(locals())
   #holds value to which all torque objects are related to, value is defined as nominal current * torque constant, value is in mNm
   node.sdo[0x6076].raw = rated_torque if rated_torque is not None else 10


def target_position(node, position: int = None):
    #6.2.114
    check_init(locals())
    #represents the position that the drive is supposed to move to using the motion control parameters
    node.sdo["Target position"].raw = position if position is not None else 0


def software_position_limit(node, min_pos_limit: int = None, max_pos_limit: int = None):
    #6.2.117
    check_init(locals())
    #defines the min and max allowed position values for the position controller, if exceeded a position error is generated
    #min position limit
    node.sdo["Software position limit.Min position limit"].raw = min_pos_limit if min_pos_limit is not None else 0
    #max position limit
    node.sdo["Software position limit.Max position limit"].raw = max_pos_limit if max_pos_limit is not None else 0


def max_profile_velocity(node, max_velocity: int = None):
    #6.2.118
    check_init(locals())
    #used as a velocity limit in a ppm or pvm move, value is given in rpm
    node.sdo["Max profile velocity"].raw = max_velocity if max_velocity is not None else 250


def max_motor_speed(node, max_velocity: int = None):
    #6.2.119
    check_init(locals())
    #indicates maximum allowed motor speed, value is given in rpm
    node.sdo["Max motor speed"].raw = max_velocity if max_velocity is not None else 250


def profile_velocity(node, prof_velocity: int = None):
    #6.2.120
    check_init(locals())
    #represents the velocity normally attained at the end of the accleeratiion ramp during a profiled move (ppm, pvm), value is given in rpm
    node.sdo["Profile velocity"].raw = prof_velocity if prof_velocity is not None else 200


def profile_acceleration(node, prof_acc: int = None):
    #6.2.121
    check_init(locals())
    #defines the acceleration used during a profiled move (ppm, pvm), value is given in rpm/s
    node.sdo["Profile acceleration"].raw = prof_acc if prof_acc is not None else 10000


def profile_deceleration(node, prof_dec: int = None):
    #6.2.122
    check_init(locals())
    #defines the deceleration used during a profiled move (ppm, pvm), value is given in rpm/s
    node.sdo["Profile deceleration"].raw = prof_dec if prof_dec is not None else 10000


def quick_stop_deceleration(node, prof_dec: int = None):
    #6.2.123
    check_init(locals())
    #defines the deceleration used during a profiled move (ppm, pvm), value is given in rpm/s
    node.sdo["Quick stop deceleration"].raw = prof_dec if prof_dec is not None else 10000

def motion_profile_type(node, profile: int = None):
    #6.2.124
    check_init(locals())
    #defines the deceleration used during a profiled move (ppm, pvm), value is given in rpm/s
    node.sdo["Motion profile type"].raw = profile if profile is not None else 0


def homing_method_init(node, homing_method: int = None):
    #6.2.125
    check_init(locals())
    #Used to select homing method (absolute SSI encoder = 37)
    node.sdo["Homing method"].raw = homing_method if homing_method is not None else 37


def homing_speeds(node, speed_sw_srch: int = None, speed_zero_srch: int = None):
    #6.2.126
    check_init(locals())
    #Speed for switch search given in rpm
    node.sdo["Homing speeds.Speed for switch search"].raw = speed_sw_srch if speed_sw_srch is not None else 100
    #Speed for zero search given in rpm, used to search the index in a homing sequence
    node.sdo["Homing speeds.Speed for zero search"].raw = speed_zero_srch if speed_zero_srch is not None else 10


def homing_acceleration(node, homing_acc: int = None):
    #6.2.127
    check_init(locals())
    #Acceleration used during homing procedure, value is given in rpm/s
    node.sdo["Homing acceleration"].raw = homing_acc if homing_acc is not None else 1000


def si_unit_position(node, si_unit_pos: int = None):
    #6.2.128
    check_init(locals())
    #Defines the position units
    node.sdo["SI unit position"].raw = si_unit_pos if si_unit_pos is not None else 0x00B50000


def si_unit_velocity(node, si_unit_vel: int = None):
    #6.2.129
    check_init(locals())
    #Defines the velocity units
    node.sdo["SI unit velocity"].raw = si_unit_vel if si_unit_vel is not None else 0x00B44700


def si_unit_acceleration(node, si_unit_acc: int = None):
    #6.2.130
    check_init(locals())
    #Defines the acceleration units
    node.sdo["SI unit acceleration"].raw = si_unit_acc if si_unit_acc is not None else 0x00C00300


def position_offset(node, pos_offset: int = None):
    #6.2.131
    check_init(locals())
    #Defines an offset that is added to the actual position value
    node.sdo["Position offset"].raw = pos_offset if pos_offset is not None else 0


def velocity_offset(node, vel_offset: int = None):
    #6.2.132
    check_init(locals())
    #Defines an offset that is added to the actual velocity value
    node.sdo["Velocity offset"].raw = vel_offset if vel_offset is not None else 0


def torque_offset(node, tor_offset: int = None):
    #6.2.133
    check_init(locals())
    #Defines an offset that is added to the actual torque value
    node.sdo["Torque offset"].raw = tor_offset if tor_offset is not None else 0


#touch probe 6.2.134 - 6.2.137 and 6.2.140 - 6.2.142 not implemented


def interpolation_time_period(node, time_period_val: int = None, time_index: int = None):
    #6.2.138
    check_init(locals())
    #interpolation time period value indicates the time between two PDOs, values > 0 enable the demand value interpolation in CSP and CSV
    #is is of importance that the setpoint is written cyclically with the inerpolation time period.
    #the value is given in s*10^time_index, value of 0 disables the demand value interpolation
    node.sdo["Interpolation time period.Interpolation time period value"].raw = time_period_val if time_period_val is not None else 0
    #time_index
    node.sdo["Interpolation time period.Interpolation time index"].raw = time_index if time_index is not None else -3


def max_acceleration(node, max_acc: int = None):
    #6.2.139
    check_init(locals())
    #used to limit the max allowed acceleration to prevent mechanical damage, represents limit of all acceleration objects of the axis
    #IN CYCLIC MODES THIS VALUE IS NOT TAKEN INTO ACCOUNT
    node.sdo["Max acceleration"].raw = max_acc if max_acc is not None else 4294967295


def digital_outputs(node, phys_outputs: int = None):
    #6.2.148
    check_init(locals())
    #configures the state of the digital output functionalities, if a bit is set to “1“ and the polarity is set to “0“, the signal at the corresponding pin is high
    node.sdo["Digital outputs.Physical outputs"].raw = phys_outputs if phys_outputs is not None else 0


def target_velocity(node, vel: int = None):
    #6.2.149
    check_init(locals())
    #in PVM the object indicates the configured target velocity and is used as input for the trajectory generation, value is given in rpm
    node.sdo["Target velocity"].raw = vel if vel is not None else 50


def motor_type(node, motor_type: int = None):
    #6.2.150
    check_init(locals())
    #motor type (DC = 1, Sinusoidal PM BL motor = 10, Trapezoidal PM BL motor = 11)
    node.sdo["Motor type"].raw = motor_type if motor_type is not None else 10


#####HOMING MODE ADDITIONAL FUNCTIONS#####
def digital_input_properties_init(node, polarity: int = None):
    #6.2.74
    check_init(locals())
    #index 1 is RO
    #if a bit is set to "1" the corresponding digital input is active high
    node.sdo["Digital input properties.Digital inputs polarity"].raw = polarity if polarity is not None else 0x0000


def config_digital_inputs_init(node, input_1_config: int = None, input_2_config: int = None,
                              input_3_config: int = None, input_4_config: int = None,
                              input_5_config: int = None, input_6_config: int = None,
                              input_7_config: int = None, input_8_config: int = None):
    #6.2.75
    check_init(locals())
    #maps functions to digital inputs, each function can only be assigned once, if sensor 2 is configured then digital inputs from 1 to 4 will bes disabled
    #index corresponds to relevant digital input
    #input 1 (DgIn1)
    node.sdo["Configuration of digital inputs.Digital input 1 configuration"].raw = input_1_config if input_1_config is not None else 0
    #input 2 (DgIn2)
    node.sdo["Configuration of digital inputs.Digital input 2 configuration"].raw = input_2_config if input_2_config is not None else 1
    #input 3 (DgIn3)
    node.sdo["Configuration of digital inputs.Digital input 3 configuration"].raw = input_3_config if input_3_config is not None else 2
    #input 4 (DgIn4)
    node.sdo["Configuration of digital inputs.Digital input 4 configuration"].raw = input_4_config if input_4_config is not None else 19
    #input 5 (HsDgIn1)
    node.sdo["Configuration of digital inputs.High-speed digital input 1 configuration"].raw = input_5_config if input_5_config is not None else 255
    #input 6 (HsDgIn2)
    node.sdo["Configuration of digital inputs.High-speed digital input 2 configuration"].raw = input_6_config if input_6_config is not None else 255
    #input 7 (HsDgIn3)
    node.sdo["Configuration of digital inputs.High-speed digital input 3 configuration"].raw = input_7_config if input_7_config is not None else 255
    #input 8 (HsDgIn4)
    node.sdo["Configuration of digital inputs.High-speed digital input 4 configuration"].raw = input_8_config if input_8_config is not None else 255




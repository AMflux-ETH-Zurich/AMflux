import time
import canopen
import keyboard
import can
import toml
from utils import check_init


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

"""def rxpdo_mapping_init(node):

    #RXPDO1
    node.sdo[0x1600][0x00].raw = 0 #disable rxpdo1
    node.sdo[0x1600][0x01].raw = 0x60400010 #control word, 0x6040(index), 0x00(subindex), 0x10(size)
    node.sdo[0x1600][0x00].raw = 1 #number of mapped objects

    node.sdo[0x1400][0x01].raw = 0x00000200 + node.id  # COB-ID for RXPDO1
    node.sdo[0x1400][0x02].raw = 255 #transmission type for RXPDO1 (255 = asynchronous)

    node.rpdo[1].clear()
    node.rpdo[1].add_variable(0x6040, 0x00)
    node.rpdo[1].cob_id = 0x00000200 + node.id
    node.rpdo[1].trans_type = 255


    #RXPDO2
    node.sdo[0x1601][0x00].raw = 0 #disable rxpdo2
    node.sdo[0x1601][0x01].raw = 0x607A0020 #target position, 0x607A(index), 0x00(subindex), 0x20(size)
    node.sdo[0x1601][0x02].raw = 0x60FF0020 #target velocity, 0x60FF(index), 0x00(subindex), 0x20(size)
    node.sdo[0x1601][0x00].raw = 2 #number of mapped objects

    node.sdo[0x1401][0x01].raw = 0x00000300 + node.id  # COB-ID for RXPDO1
    node.sdo[0x1401][0x02].raw = 255 #transmission type for RXPDO1 (255 = asynchronous)

    node.rpdo[2].clear()
    node.rpdo[2].add_variable(0x607A, 0x00)
    node.rpdo[2].add_variable(0x60FF, 0x00)
    node.rpdo[2].cob_id = 0x00000300 + node.id
    node.rpdo[2].trans_type = 255"""
   

def pdo_mapping_init(node, network):
    
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

    # TxPDO1: asynchronous/event-driven, contains statusword (is sent if something changes in the 402 state machine)
    node.tpdo[1].add_variable(0x6041, 0x00) # Statusword
    node.sdo[0x1A00:1].raw = 0x60410010  # PDO1 mapping: Statusword
    node.tpdo[1].trans_type = 255 #   1=SYNC; 255=asynchronous --> with every change of the 402 state machine (in 0x2400.04, bit mask 0x00000002 must be set (which is the default))
    node.tpdo[1].enabled = True

    # TxPDO2: synchronous, contains velocity actual value and position actual value
    node.tpdo[2].add_variable(0x606C, 0x00) # Velocity actual value
    node.sdo[0x1A01:1].raw = 0x606C0010  # PDO2 mapping: Velocity actual value
    node.tpdo[2].add_variable(0x6064, 0x00) # Position actual value
    node.sdo[0x1A01:2].raw = 0x60640020  # PDO2 mapping: Position actual value
    #node.tpdo[2].add_variable(0x6077, 0x00) # Torque actual value
    #node.tpdo[2].add_variable(0x6061, 0x00) # Modes of operation display
    node.tpdo[2].trans_type = 1 #   1=SYNC; 255=asynchronous
    node.tpdo[2].enabled = True

    # RxPDO1: event-driven, contains controlword
    node.rpdo[1].add_variable(0x6040, 0x00)  # Controlword
    node.sdo[0x1600:1].raw = 0x60400010  # RxPDO1 mapping: Controlword
    node.rpdo[1].trans_type = 255 #   1=SYNC; 255=asynchronous
    node.rpdo[1].enabled = True
    
    # RxPDO2: event-driven, contains target velocity and target position
    #node.rpdo[2].add_variable(0x6060, 0x00)  # Modes of operation
    node.rpdo[2].add_variable(0x60FF, 0x00)  # Target velocity
    node.sdo[0x1601:1].raw = 0x60FF0010  # RxPDO2 mapping: Target velocity
    node.rpdo[2].add_variable(0x607A, 0x00)  # Target position
    node.sdo[0x1601:2].raw = 0x607A0020  # RxPDO2 mapping: Target position
    node.rpdo[2].trans_type = 255 #   1=SYNC; 255=asynchronous
    node.rpdo[2].enabled = True
    
    node.nmt.state = 'PRE-OPERATIONAL'
    node.tpdo.save()
    node.rpdo.save()  

    #node.rpdo[4]['0x606C.0x00'].phys = 1000
    network.nmt.state = 'OPERATIONAL'      


def axis_configuration_init(node, sens_res: int=None, sys_speed: int=None):
    #6.2.52
    #Axis configuration for absolute SSI encoder
    node.sdo[0x3000:1].raw = 0x00000300
    #Axis control structure
    node.sdo[0x3000:2].raw = 0b00000000000000100000000100100001
    #Commutaton Sensors
    node.sdo[0x3000:3].raw = 0x00000020
    #Miscellaneous Axis Configuration
    node.sdo[0x3000:4].raw = 0x00000000


def motor_init(node, motor_type: int=None, nominal_current: int=None, current_lim: int=None, 
                        pole_pairs: int=None, therm_const: int=None, tor_const: int=None):
    
    
    check_init(locals())
    #6.2.53
    #motor type (Sinusoidal PM BL motor = 10 or in hex 0x0A)
    node.sdo[0x6402:0].raw = motor_type if motor_type is not None else 0x000A
    #nominal current flowing through motor windings in mA
    node.sdo[0x3001:1].raw = nominal_current if nominal_current is not None else 15000
    #output current limit in mA
    node.sdo[0x3001:2].raw = current_lim if current_lim is not None else 30000
    #number of pole pairs
    node.sdo[0x3001:3].raw = pole_pairs if pole_pairs is not None else 1
    #thermal time constant of windings
    node.sdo[0x3001:4].raw = therm_const if therm_const is not None else 40
    #torque constant of motor
    node.sdo[0x3001:5].raw = tor_const if tor_const is not None else 0


def ssi_abs_encoder_init(node, data_rate: int=None, data_bits: int=None, encoding_type: int=None,
                                timeout_time: int=None, power_up_time: int=None, commutation_offset_value: int=None,
                                        position_bits: int=None, communication_additional_delay: int=None):
    #6.2.58
    check_init(locals())
    #SSI data rate
    node.sdo[0x3012:1].raw = data_rate if data_rate is not None else 2000 #REFER TO RENISHAW ENCODER SHEET
    #SSI number of data bits
    node.sdo[0x3012:2].raw = data_bits if data_bits is not None else 0x00000C00 #REFER TO RENISHAW ENCODER SHEET
    #SSI encoding type
    node.sdo[0x3012:3].raw = encoding_type if encoding_type is not None else 0x001 #REFER TO RENISHAW ENCODER SHEET
    #SSI timeout time
    node.sdo[0x3012:5].raw = timeout_time if timeout_time is not None else 30 #REFER TO RENISHAW ENCODER SHEET
    #Special bits trailing data
    #READ ONLY node.sdo[0x3012][0x6].raw = sbits_trailing_data if sbits_trailing_data is not None else 0 #REFER TO RENISHAW ENCODER SHEET
    #SSI refresh frequency
    #READ ONLY node.sdo[0x3012][0x7].raw = refresh_frequency #REFER TO RENISHAW ENCODER SHEET
    #SSI Power up time
    node.sdo[0x3012:8].raw = power_up_time if power_up_time is not None else 200 #REFER TO RENISHAW ENCODER SHEET
    #SSI postition raw value, lower 32 bits
    #READ ONLY node.sdo[0x3012][0x9].raw = position_raw_value
    #SSI commutaiton offset value
    node.sdo[0x3012:10].raw = commutation_offset_value if commutation_offset_value is not None else 0
    #SSI position bits
    node.sdo[0x3012:11].raw = position_bits if position_bits is not None else  0x0000000C #REFER TO RENISHAW ENCODER SHEET, MUST BE REDUCED IF >32
    #SSI special bits leading data
    #READ ONLY node.sdo[0x3012][0xC].raw = sbits_leading_data if sbits_leading_data is not None else 0 #REFER TO RENISHAW ENCODER SHEET
    #SSI position raw value complemete
    #READ ONLY node.sdo[0x3012][0xD].raw = position_raw_value_complete
    #SSI communication additional delay
    node.sdo[0x3012:12].raw = communication_additional_delay if communication_additional_delay is not None else -1 #REFER TO RENISHAW EN


def electrical_system_init(node, electrical_resistance: int=None, electrical_inductance: int=None):
    #6.2.54
    check_init(locals())
    #electrical resistance of motor windings in mOhm
    node.sdo[0x3002:1].raw = electrical_resistance if electrical_resistance is not None else 0
    #electrical inductance of motor windings in uH
    node.sdo[0x3002:2].raw = electrical_inductance if electrical_inductance is not None else 0


def current_control_parameter_init(node, p_gain: int = None, i_gain: int = None):
    #6.2.61
    check_init(locals())
    #Current controller P-gain
    node.sdo[0x30A0:1].raw = p_gain if p_gain is not None else 1171880
    #Current controller I-gain
    node.sdo[0x30A0:2].raw = i_gain if i_gain is not None else 3906250


def position_control_parameter_init(node, p_gain:int = None, i_gain: int = None, d_gain:int = None,
                               ffv_gain: int = None, ffa_gain: int = None, i_gain_si_unit: int = None):
    #6.2.62
    check_init(locals())
    #Position controller P-gain
    node.sdo[0x30A1:1].raw = p_gain if p_gain is not None else 1500000
    #Position controller I-gain
    node.sdo[0x30A1:2].raw = i_gain if i_gain is not None else 780000
    #Position controller D-gain
    node.sdo[0x30A1:3].raw = d_gain if d_gain is not None else 16000
    #Position controller Feed Forward velocity gain
    node.sdo[0x30A1:4].raw = ffv_gain if ffv_gain is not None else 0
    #Position controller Feed Forward acceleration gain
    node.sdo[0x30A1:5].raw = ffa_gain if ffa_gain is not None else 0
    #Position controller I-gain SI units
    node.sdo[0x30A1:9].raw = i_gain_si_unit if i_gain_si_unit is not None else 0xFA040300


def velocity_control_parameter_init(node, p_gain:int = None, i_gain: int = None, 
                               ffv_gain: int = None, ffa_gain: int = None, f_cutoff: int = None):
    #6.2.63
    check_init(locals())
    #Velocity controller P-gain
    node.sdo[0x30A2:1].raw = p_gain if p_gain is not None else 20000
    #Velocity controller I-gain
    node.sdo[0x30A2:2].raw = i_gain if i_gain is not None else 500000
    #Velocity controller Feed Forward velocity gain
    node.sdo[0x30A2:3].raw = ffv_gain if ffv_gain is not None else 0
    #Velocity controller Feed Forward acceleration gain
    node.sdo[0x30A2:4].raw = ffa_gain if ffa_gain is not None else 0
    #Velocity controller filter cut-off frequency in HZ
    node.sdo[0x30A2:5].raw = f_cutoff if f_cutoff is not None else 600
    

def velocity_observer_parameter_init(node, pos_corr_gain: int = None, vel_corr_gain: int = None,
                                load_corr_gain: int = None, friction: int = None, inertia: int = None
                                ):
    #6.2.64
    check_init(locals())
    #Velocity observer position correction gain given in promille
    node.sdo[0x30A3:1].raw = pos_corr_gain if pos_corr_gain is not None else 400
    #Velocity observer velocity correction gain given in mHz
    node.sdo[0x30A3:2].raw = vel_corr_gain if vel_corr_gain is not None else 100000
    #Velocity observer load correction gain
    node.sdo[0x30A3:3].raw = load_corr_gain if load_corr_gain is not None else 33
    #Velocity observer friction
    node.sdo[0x30A3:4].raw = friction if friction is not None else 10
    #Velocity observer inertia
    node.sdo[0x30A3:5].raw = inertia if inertia is not None else 1000


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
    node.sdo[0x30AE:1].raw = main_loop_p_gain_low if main_loop_p_gain_low is not None else 10000
    #Represents the main loop high bandwidth proportional factor
    node.sdo[0x30AE:2].raw = main_loop_p_gain_high if main_loop_p_gain_high is not None else 100000
    #Represents the main loop gain scheduler
    node.sdo[0x30AE:3].raw = main_loop_gain_scheduler if main_loop_gain_scheduler is not None else 12500
    #Main loop filter coefficient a
    node.sdo[0x30AE:16].raw = main_loop_filter_coeff_a if main_loop_filter_coeff_a is not None else 1000
    #Main loop filter coefficient b
    node.sdo[0x30AE:17].raw = main_loop_filter_coeff_b if main_loop_filter_coeff_b is not None else 1000
    #Main loop filter coefficient c
    node.sdo[0x30AE:18].raw = main_loop_filter_coeff_c if main_loop_filter_coeff_c is not None else 1000
    #Main loop filter coefficient d
    node.sdo[0x30AE:19].raw = main_loop_filter_coeff_d if main_loop_filter_coeff_d is not None else 1000
    #Main loop filter coefficient e
    node.sdo[0x30AE:20].raw = main_loop_filter_coeff_e if main_loop_filter_coeff_e is not None else 1000
    #Auxiliary loop P-gain
    node.sdo[0x30AE:32].raw = auxiliary_loop_p_gain if auxiliary_loop_p_gain is not None else 20000
    #Auxiliary loop I-gain
    node.sdo[0x30AE:33].raw = auxiliary_loop_i_gain if auxiliary_loop_i_gain is not None else 500000
    #Auxiliary loop FF velocity gain
    node.sdo[0x30AE:34].raw = auxiliary_loop_ff_velocity_gain if auxiliary_loop_ff_velocity_gain is not None else 0
    #Auxiliary loop FF acceleration gain
    node.sdo[0x30AE:35].raw = auxiliary_loop_ff_acceleration_gain if auxiliary_loop_ff_acceleration_gain is not None else 0
    #Auxiliay loop observer position correction gain
    node.sdo[0x30AE:48].raw = auxiliary_loop_observer_pos_corr_gain if auxiliary_loop_observer_pos_corr_gain is not None else 400
    #Auxiliay loop observer velocity correction gain
    node.sdo[0x30AE:49].raw = auxiliary_loop_observer_vel_corr_gain if auxiliary_loop_observer_vel_corr_gain is not None else 100000
    #Auxiliay loop observer load correction gain
    node.sdo[0x30AE:50].raw = auxiliary_loop_observer_load_corr_gain if auxiliary_loop_observer_load_corr_gain is not None else 33
    #Auxiliay loop observer friction
    node.sdo[0x30AE:51].raw = auxiliary_loop_observer_friction if auxiliary_loop_observer_friction is not None else 10
    #Auxiliay loop observer inertia
    node.sdo[0x30AE:52].raw = auxiliary_loop_observer_inertia if auxiliary_loop_observer_inertia is not None else 1000
    #Dual loop control miscellaneous configuration
    node.sdo[0x30AE:64].raw = dual_loop_miscellaneous if dual_loop_miscellaneous is not None else 0x0000


def home_position_init(node, homeposition: int = None):
    #6.2.66
    check_init(locals())
    #defines the position that will be set as zero position of the absolute position counter.
    node.sdo[0x30B0:0].raw = homeposition if homeposition is not None else 0


def home_offset_distance_init(node, home_offset_distance: int = None):
    #6.2.67
    check_init(locals())
    #Represents a moving distance in a homing procedure.
    node.sdo[0x30B1:0].raw = home_offset_distance if home_offset_distance is not None else 0


def current_threshold_homing_init(node, current_threshold_homing: int = None):
    #6.2.68
    check_init(locals())
    #Used for homing methods «−1», «−2», «−3», and «−4».
    node.sdo[0x30B2:0].raw = current_threshold_homing if current_threshold_homing is not None else 1000


def standstill_window_init(node, standstill_window: int = None):
    #6.2.73.1
    check_init(locals())
    #Defines a symmetric range of accepted velocity values relatively to zero.
    node.sdo[0x30E0:1].raw = standstill_window if standstill_window is not None else 30


def standstill_window_time_init(node, standstill_window_time: int = None):
    #6.2.73.2
    check_init(locals())
    #Defines the time duration for which the velocity must remain within the standstill window for Standstill to be reached. [ms]
    node.sdo[0x30E0:2].raw = standstill_window_time if standstill_window_time is not None else 2


def standstill_window_timeout_init(node, standstill_window_timeout: int = None):
    #6.2.73.3
    check_init(locals())
    #Defines the point of time standstill is supposed to be reached, even if the standstill conditions are not yet fulfilled.
    node.sdo[0x30E0:3].raw = standstill_window_timeout if standstill_window_timeout is not None else 1000


def abort_connection_option_init(node, abort_option: int = None):
    #6.2.92
    check_init(locals())
    #Specifies the action that will be performed when one of the errors labeled “a” is detected
    # 2 -> «Disable voltage» command
    # 3 -> Decelerate with quick stop ramp; disabling of the drive function
    node.sdo[0x605B:0].raw = abort_option if abort_option is not None else 3


def mode_of_operation_init(node, op_mode: int):
    #6.2.101
    check_init(locals())
    #Set mode of operation
    #default: 1 -> Profile position mode
    node.sdo[0x6060].raw = op_mode if op_mode is not None else 1 


def following_error_window_init(node, following_error_window: int = None):
    #6.2.105
    check_init(locals())
    #Defines the maximum allowed deviation between target and actual position.
    #min=0, max=2.147.483.647
    node.sdo[0x6065:0].raw = following_error_window if following_error_window is not None else 2000


def following_error_timeout_init(node, following_error_timeout: int = None):
    #6.2.106
    check_init(locals())
    # Indicates the configured time for a following error condition. If exceeded, a following error will occur.
    # The value is given in milliseconds [ms].
    node.sdo[0x6066:0].raw = following_error_timeout if following_error_timeout is not None else 0


def position_window_init(node, position_window: int = None):
    #6.2.107
    check_init(locals())
    #Defines a symmetric range of accepted position values relatively to target position.
    #min=0, max=2147483647, disable=4294967295
    node.sdo[0x6067:0].raw = position_window if position_window is not None else 4294967295


def position_window_time_init(node, position_window_time: int = None):
    #6.2.108
    check_init(locals())
    # Indicates the configured position window time for the target reached condition. If the actual position is within the Position
    # window during the set time, the corresponding bit 10 (target reached) in the Statusword will be set to “1”.
    # The value is given in milliseconds [ms].
    node.sdo[0x6068:0].raw = position_window_time if position_window_time is not None else 0


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


#def motor_rated_torque(node, rated_torque: int = None):
#    #6.2.112
 #   check_init(locals())
 #   #holds value to which all torque objects are related to, value is defined as nominal current * torque constant, value is in mNm
 #   node.sdo[0x6076].raw = rated_torque if rated_torque is not None else 0


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
    node.sdo[0x607D:1].raw = min_pos_limit if min_pos_limit is not None else 0
    #max position limit
    node.sdo[0x607D:2].raw = max_pos_limit if max_pos_limit is not None else 0


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
    node.sdo[0x6099:1].raw = speed_sw_srch if speed_sw_srch is not None else 100
    #Speed for zero search given in rpm, used to search the index in a homing sequence
    node.sdo[0x6099:2].raw = speed_zero_srch if speed_zero_srch is not None else 10


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
    node.sdo[0x60C2:1].raw = time_period_val if time_period_val is not None else 0
    #time_index
    node.sdo[0x60C2:2].raw = time_index if time_index is not None else -3


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
    node.sdo[0x60FE:1].raw = phys_outputs if phys_outputs is not None else 0


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


#####HOMING MODE ADDITIONAL FUNCTIONS#####
def digital_input_properties_init(node, polarity: int = None):
    #6.2.74
    check_init(locals())
    #index 1 is RO
    #if a bit is set to "1" the corresponding digital input is active high
    node.sdo[0x3141:2].raw = polarity if polarity is not None else 0x0000


def config_digital_inputs_init(node, input_1_config: int = None, input_2_config: int = None,
                              input_3_config: int = None, input_4_config: int = None,
                              input_5_config: int = None, input_6_config: int = None,
                              input_7_config: int = None, input_8_config: int = None):
    #6.2.75
    check_init(locals())
    #maps functions to digital inputs, each function can only be assigned once, if sensor 2 is configured then digital inputs from 1 to 4 will bes disabled
    #index corresponds to relevant digital input
    #input 1 (DgIn1)
    node.sdo[0x3142:1].raw = input_1_config if input_1_config is not None else 0
    #input 2 (DgIn2)
    node.sdo[0x3142:2].raw = input_2_config if input_2_config is not None else 1
    #input 3 (DgIn3)
    node.sdo[0x3142:3].raw = input_3_config if input_3_config is not None else 2
    #input 4 (DgIn4)
    node.sdo[0x3142:4].raw = input_4_config if input_4_config is not None else 19
    #input 5 (HsDgIn1)
    node.sdo[0x3142:5].raw = input_5_config if input_5_config is not None else 255
    #input 6 (HsDgIn2)
    node.sdo[0x3142:5].raw = input_6_config if input_6_config is not None else 255
    #input 7 (HsDgIn3)
    node.sdo[0x3142:5].raw = input_7_config if input_7_config is not None else 255
    #input 8 (HsDgIn4)
    node.sdo[0x3142:5].raw = input_8_config if input_8_config is not None else 255



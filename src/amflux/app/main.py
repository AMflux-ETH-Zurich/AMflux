import time
import canopen
import keyboard
import can
from can.io import LimitedSend

net = None
"""
def network_scan(node_channel: str):
    global net
    if net == None:
        # Create a CANopen network  
        net = canopen.Network()
        # Connect to a CAN interface (e.g., 'can0' for Linux SocketCAN)
        net.connect(channel= node_channel, bustype='socketcan')

    # Scan for nodes on the network
    net.scanner.search()
    # Print found nodes
    for nid in net.scanner.nodes:
        print(f"Found node: {nid}")
"""


# Define a safe message rate to avoid buffer overflow
RATE_LIMIT_MSGS_PER_SEC = 750 

def network_scan(node_channel: str):
    global net
    if net == None:
        # 1. Create the base SocketCAN bus object
        # This is the object that interacts with the Linux kernel
        bus = can.interface.Bus(
            channel=node_channel, 
            bustype='socketcan'
        )

        # 2. Wrap the bus object to enforce a transmission rate limit
        # This fixes the "Transmit buffer full" error during rapid scanning
        rate_limited_bus = LimitedSend(bus, max_message_per_second=RATE_LIMIT_MSGS_PER_SEC) 
        
        # 3. Create a CANopen network and assign the rate-limited bus
        # This allows CANopen to use the rate-limited transmission logic
        net = canopen.Network()
        net.bus = rate_limited_bus 

    # Scan for nodes on the network. This call now respects the rate limit.
    net.scanner.search()
    
    # Print found nodes
    for nid in net.scanner.nodes:
        print(f"Found node: {nid}")

def network_setup(node_id: int, node_eds: str, node_channel: str):
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



def axis_configuration_setup(node, sens_res: int=None, sys_speed: int=None):
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


def motor_init_setup(node, motor_type: int=None, nominal_current: int=None, current_lim: int=None, 
                        pole_pairs: int=None, therm_const: int=None, tor_const: int=None):
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


def ssi_abs_encoder_init_setup(node, data_rate: int=None, data_bits: int=None, encoding_type: int=None,
                                timeout_time: int=None, power_up_time: int=None, commutation_offset_value: int=None,
                                        position_bits: int=None, communication_additional_delay: int=None):
    #6.2.58
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


def electrical_system_init_setup(node, electrical_resistance: int=None, electrical_inductance: int=None):
    #6.2.54
    #electrical resistance of motor windings in mOhm
    node.sdo[0x3002][0x1].raw = electrical_resistance if electrical_resistance is not None else 0
    #electrical inductance of motor windings in uH
    node.sdo[0x3002][0x2].raw = electrical_inductance if electrical_inductance is not None else 0


def current_control_parameter(node, p_gain: int = None, i_gain: int = None):
    #6.2.61
    #Current controller P-gain
    node.sdo[0x30A0][1].raw = p_gain if p_gain is not None else 1171880
    #Current controller I-gain
    node.sdo[0x30A0][2].raw = i_gain if i_gain is not None else 3906250


def position_control_parameter(node, p_gain:int = None, i_gain: int = None, d_gain:int = None,
                               ffv_gain: int = None, ffa_gain: int = None, i_gain_si_unit: int = None):
    #6.2.62
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


def velocity_control_parameter(node, p_gain:int = None, i_gain: int = None, 
                               ffv_gain: int = None, ffa_gain: int = None, f_cutoff: int = None):
    #6.2.63
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
    

def velocity_observer_parameter(node, pos_corr_gain: int = None, vel_corr_gain: int = None,
                                load_corr_gain: int = None, friction: int = None, inertia: int = None
                                ):
    #6.2.64
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


def dual_loop_position_control_parameter(node, main_loop_p_gain_low:int = None, main_loop_p_gain_high:int = None, 
                                         main_loop_gain_scheduler:int = None, main_loop_filter_coeff_a:int = None,
                                         main_loop_filter_coeff_b:int = None, main_loop_filter_coeff_c:int = None,
                                         main_loop_filter_coeff_d:int = None, main_loop_filter_coeff_e:int = None,
                                         auxiliary_loop_p_gain:int = None, auxiliary_loop_i_gain:int = None,
                                         auxiliary_loop_ff_velocity_gain:int = None, auxiliary_loop_ff_acceleration_gain:int = None,
                                         auxiliary_loop_observer_pos_corr_gain:int = None, auxiliary_loop_observer_vel_corr_gain:int = None,
                                         auxiliary_loop_observer_load_corr_gain:int = None, auxiliary_loop_observer_friction:int = None,
                                         auxiliary_loop_observer_inertia:int = None, dual_loop_miscellaneous:int = None):
    #6.2.65
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
    #defines the position that will be set as zero position of the absolute position counter.
    node.sdo[0x30B0][0x00].raw = homeposition if homeposition is not None else 0


def home_offset_distance_init(node, home_offset_distance: int = None):
    #6.2.67
    #Represents a moving distance in a homing procedure.
    node.sdo[0x30B1][0x00].raw = home_offset_distance if home_offset_distance is not None else 0


def Current_threshold_homing_init(node, current_threshold_homing: int = None):
    #6.2.68
    #Used for homing methods «−1», «−2», «−3», and «−4».
    node.sdo[0x30B2][0x00].raw = current_threshold_homing if current_threshold_homing is not None else 1000


def homing_method_init(node, homing_method: int = None):
    #6.2.125
    #Used to select homing method (absolute SSI encoder = 37)
    node.sdo[0x6098].raw = homing_method if homing_method is not None else 37


def standstill_window_init(node, standstill_window: int = None):
    #6.2.73.1
    #Defines a symmetric range of accepted velocity values relatively to zero.
    node.sdo[0x30E0][0x01].raw = standstill_window if standstill_window is not None else 30


def standstill_window_time_init(node, standstill_window_time: int = None):
    #6.2.73.2
    #Defines the time duration for which the velocity must remain within the standstill window for Standstill to be reached. [ms]
    node.sdo[0x30E0][0x02].raw = standstill_window_time if standstill_window_time is not None else 2


def standstill_window_timeout_init(node, standstill_window_timeout: int = None):
    #6.2.73.3
    #Defines the point of time standstill is supposed to be reached, even if the standstill conditions are not yet fulfilled.
    node.sdo[0x30E0][0x03].raw = standstill_window_timeout if standstill_window_timeout is not None else 1000


def motor_setup(node, operating_mode: int):
    # Shutdown
    node.sdo[0x6040].raw = 0x0006
    # Switch on
    node.sdo[0x6040].raw = 0x0007
    # Mode
    node.sdo[0x6060].raw = operating_mode
    # Enable operation
    node.sdo[0x6040].raw = 0x000F

    
def quick_stop(node):
	node.sdo[0x6040].raw = 0x0002
     

def motor_run_cst(node, torque: int, duration: float):
    # Give system time for setup
    time.sleep(0.1)
    # Torque Command, Torque in mNm (Firmware-Specification, p223)
    node.sdo[0x60FF].raw = torque
    # Records start time
    start = time.time()    
    # Loop that runs for a given duration
    while time.time() - start < duration:
        # If key 's' is pressed 
        if keyboard.is_pressed('s'):
            quick_stop(node)
            # Finishing the loop
            break
        # So we're not running at maximum cpu capacity
        time.sleep(0.01)
    # Give system time for use of Quick Stop
    time.sleep(0.1)    
    # Shutdown
    node.sdo[0x6040].raw = 0x0006
    # Give system time for Shutdown
    time.sleep(0.1)
    # Disable drive
    node.sdo[0x6040].raw = 0x0000

    
    
def network_shutdown():
    global net
    if net != None:
        # Disconnect from the network
        net.disconnect()
        net = None


def main():
    global net
    # Setup our CANopen network
    network, mc1 = network_setup(1, '/home/amflux/AMflux/src/amflux/app/Epos4_70_15.eds', 'can0')
    # Setup our Motor Controller
    # CST mode is under int=10 (Firmware-Specification, p219)
    motor_setup(mc1, 10)
    # Run the motor in cst mode using a given amount of rpm
    motor_run_cst(mc1, 1000, 2)
    # Shutdown the CANopen network after use
    network_shutdown()




    

if __name__ == "__main__":
    #main()
    network_scan('can0')
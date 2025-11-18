import time
import canopen
import keyboard

net = None

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


def motor_init_setup(node, motor_type: int=None, nominal_current: int=None, current_lim: int=None, 
                        pole_pairs: int=None, therm_const: int=None, tor_const: int=None):
def axis_configuration_setup(node, sens_res: int, sys_speed: int):
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


def motor_init_setup(node, nominal_current: int, current_lim: int, pole_pairs: int, therm_const: int, 
                     tor_const: int, ):
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
                                timeout_time: int=None, sbits_trailing_data: int = None, refresh_frequency: int=None,
                                    power_up_time: int=None, position_raw_value: int=None, commutation_offset_value: int=None,
                                        position_bits: int=None, sbits_leading_data: int=None, position_raw_value_complete: int=None, 
                                            communication_additional_delay: int=None):
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
    node.sdo[0x3012][0x6].raw = sbits_trailing_data if sbits_trailing_data is not None else 0 #REFER TO RENISHAW ENCODER SHEET
    #SSI refresh frequency
    node.sdo[0x3012][0x7].raw = refresh_frequency #REFER TO RENISHAW ENCODER SHEET
    #SSI Power up time
    node.sdo[0x3012][0x8].raw = power_up_time if power_up_time is not None else 200 #REFER TO RENISHAW ENCODER SHEET
    #SSI postition raw value, lower 32 bits
    node.sdo[0x3012][0x9].raw = position_raw_value
    #SSI commutaiton offset value
    node.sdo[0x3012][0xA].raw = commutation_offset_value if commutation_offset_value is not None else 0
    #SSI position bits
    node.sdo[0x3012][0xB].raw = position_bits if position_bits is not None else  0x0000000C #REFER TO RENISHAW ENCODER SHEET, MUST BE REDUCED IF >32
    #SSI special bits leading data
    node.sdo[0x3012][0xC].raw = sbits_leading_data if sbits_leading_data is not None else 0 #REFER TO RENISHAW ENCODER SHEET
    #SSI position raw value complemete
    node.sdo[0x3012][0xD].raw = position_raw_value_complete
    #SSI communication additional delay
    node.sdo[0x3012][0xE].raw = communication_additional_delay if communication_additional_delay is not None else -1 #REFER TO RENISHAW EN




def electrical_system_init_setup(node, electrical_resistance: int=None, electrical_inductance: int=None):
    #6.2.54
    #electrical resistance of motor windings in mOhm
    node.sdo[0x3002][0x1].raw = electrical_resistance if electrical_resistance is not None else 0
    #electrical inductance of motor windings in uH
    node.sdo[0x3002][0x2].raw = electrical_inductance if electrical_resistance is not None else 0




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
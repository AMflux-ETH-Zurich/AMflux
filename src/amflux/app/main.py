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


def controller_init_setup():

def motor_init_setup(nominal_current: int, current_lim: int, pole_pairs: int, th_t_const: int, tor_const: int, ):
    #motor type (Sinusoidal PM BL motor = 10 or in hex 0x0A)
    node.sdo[0x6402.0x00].raw = 0x000A
    #nominal current flowing through motor windings in mA
    node.sdo[0x3001.0x01].raw = nominal_current
    #output current limit in mA
    node.sdo[0x3001.0x02].raw = current_lim
    #number of pole pairs
    node.sdo[0x3001.0x03].raw = pole_pairs
    #thermal time constant of windings
    node.sdo[0x3001.0x04].raw = th_t_const
    #torque constant of motor
    node.sdo[0x3001.0x05].raw = tor_const
    
    
    
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
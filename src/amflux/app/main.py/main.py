import time
import canopen
import keyboard


def network_setup(node_id: int, node_eds: str, node_channel: str):
    # Create a CANopen network
    network = canopen.Network()
    # Connect to a CAN interface (e.g., 'can0' for Linux SocketCAN)
    # Write 'virtual' when debugging on mac 
    network.connect(channel=node_channel, bustype='socketcan')
    # Add a node to the network with a specific EDS file
    node = network.add_node(node_id, node_eds) 
    # Return network and the node 
    return network, node
    
    
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

    
    
def network_shutdown(network):
    # Disconnect from the network
    network.disconnect()


def main():
    # Setup our CANopen network
    network, mc1 = network_setup(1, 'src/amflux/app/main.py/Epos4_70_15.eds', 'can0')
    # Setup our Motor Controller
    # CST mode is under int=10 (Firmware-Specification, p219)
    motor_setup(mc1, 10)
    # Run the motor in cst mode using a given amount of rpm
    motor_run_cst(mc1, 1000, 2)
    # Shutdown the CANopen network after use
    network_shutdown(network)




    

if __name__ == "__main__":

    main()
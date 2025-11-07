import canopen
import time

def network_setup():
    # Create a CANopen network
    network = canopen.Network()

    # Connect to a CAN interface (e.g., 'can0' for Linux SocketCAN)
    network.connect(channel='can0', bustype='socketcan') #write 'virtal' when debugging on mac 

    # Add a node to the network with a specific EDS file
    node = network.add_node(1, 'src/amflux/app/main.py/Epos4_70_15.eds') 

    # disconnect from the network
    network.disconnect()




def run_motor():
    node.sdo[0x6040].raw = 0x0006      # Shutdown
    node.sdo[0x6040].raw = 0x0007      # Switch on

    node.sdo[0x6060].raw = 10          # Open-loop mode (254 decimal) (10 should work)

    node.sdo[0x6040].raw = 0x000F      # Enable operation

    node.sdo[0x60FF].raw = 300         # Command (depends on mode scaling)

    time.sleep(2)

    node.sdo[0x6040].raw = 0x0000      # Disable drive (stop)



def main():
    network_setup()





    

if __name__ == "__main__":

    main()
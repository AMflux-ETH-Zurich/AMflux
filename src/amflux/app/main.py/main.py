import canopen


def network_setup():
    # Create a CANopen network
    network = canopen.Network()

    # Connect to a CAN interface (e.g., 'can0' for Linux SocketCAN)
    network.connect(channel='can0', bustype='socketcan')

    # Add a node to the network with a specific EDS file
    node = network.add_node(1, 'path/to/your/eds/file.eds') 

    # disconnect from the network
    network.disconnect()





def main():
    network_setup()

    #blablabal
    

if __name__ == "__main__":

    main()
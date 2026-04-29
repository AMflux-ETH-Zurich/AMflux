"""
CAN bus communication functions for CANopen network management and device control.

This module provides utilities for setting up CANopen networks, managing node connections,
sending CAN messages, and controlling CANopen devices through SDO/PDO communication.

Functions:
    network_setup: Initializes CANopen network and adds a node
    network_shutdown: Disconnects and shuts down the CANopen network
    send_can_message: Sends raw CAN messages on the bus
    sanity_check: Verifies CAN communication with a test message exchange
    cword_write: Writes a value to the device Controlword (0x6040)
    cword_read: Reads the current Controlword value (0x6040)
    sword: Reads the device Statusword (0x6041)

"""


# ======================================================================
# Imports
# ======================================================================

import time
import canopen
import can
from errors import SanityCheck


# ======================================================================
# Network control functions
# ======================================================================

def network_setup(node_id: int, node_od: str, node_channel: str, net):
    """Sets up CANopen network and adds node.

    Args:
        node_id (int): CANopen Node ID of the device.
        node_od (str): Path to the EDS/DCF file for the device.
        node_channel (str): CAN interface channel (e.g., 'can0' for Linux SocketCAN).

    Returns:
        CANopen Network and Node that was added
    """
    if net == None:
        # Create a CANopen network
        net = canopen.Network()
        # Connect to a CAN interface (e.g., 'can0' for Linux SocketCAN)
        # Write 'virtual' when debugging on mac 
        net.connect(channel=node_channel, bustype='socketcan')
    # Add a node to the network with a specific EDS/DCF file.
    node = net.add_node(node_id, str(node_od)) 
    # Return network and the node 
    return net, node


def network_shutdown(net):
    """Shuts down the CANopen network after use"""
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
        

# ======================================================================
# Device control functions
# ======================================================================

def cword_write(node, value: int):
    """
    Writes a value to Controlword.
    
    :param node: CANopen Network Node with Controlword instance
    :param value: Value of Controlword [binary]
    :type value: int
    """

    # write controlword via RPDO
    node.rpdo[1]["Controlword"].raw = value
    node.rpdo[1].transmit()

    # write controlword via SDO
    #node.sdo[0x6040].raw = value


def cword_read(node) -> int:
    """
    Reads Controlword.
    
    :param node: CANopen Network Node with Controlword instance
    :return: returns Controlword. [binary]
    :rtype: int
    """
    
    # Controlword is not mapped in the DCF TPDO layout.
    return node.sdo[0x6040].raw


def sword(node) -> int:
    """
    Reads Statusword.
    
    :param node: CANopen Network Node with Statusword instance
    :return: returns Statusword. [binary]
    :rtype: int
    """
    # read statusword via TPDO
    return node.tpdo[1]["Statusword"].raw
    

    # read statusword via SDO
    #return node.sdo[0x6041].raw

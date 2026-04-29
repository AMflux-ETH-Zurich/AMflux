"""
CANopen EPOS4 Configuration Utilities.

This module provides a structured, high-level interface for initializing and
configuring Maxon EPOS4 motor controllers via CANopen (CiA 405).

It includes:
    - CAN network setup and shutdown utilities
    - Node scanning and rate-limited SDO access
    - Initialization routines for motor, encoder, homing, control loops, etc.
    - A validation system (`check_init`) to prevent uninitialized parameters
    - Numerous EPOS4 configuration helpers following the Communication Guide
    - State Control of the EPOS4

This module intentionally mirrors the structure of the EPOS4 documentation
(Communication Guide, Firmware Specification), providing clear mapping from
code to documentation section numbers (e.g., "6.2.58").
"""


# ======================================================================
# Imports
# ======================================================================

import can_functions
import object_dictionary_functions
import organiser
import gui


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
# Main
# ======================================================================

def main():
    global net
    # Setup our CANopen network
    network, mc1 = can_functions.network_setup(
        1, 
        object_dictionary_functions.resolve_dcf_path(),
        'can0', 
        net
    )

    # Sanity check for Network communication
    can_functions.sanity_check(network)

    # Apply the EPOS Studio DCF, including PDO mapping and ParameterValue entries.
    mc1.nmt.state = 'PRE-OPERATIONAL'
    object_dictionary_functions.disable_command_parameter_values(mc1)
    mc1.load_configuration()
    mc1.nmt.state = 'OPERATIONAL'

    # Initialize Drive Organiser
    EPOS4 = organiser.DriveOrganiser(mc1, network=network)

    # Start user interface
    app = gui.App(drive=EPOS4)
    app.mainloop()
    
    # Shutdown network after user interface
    #can_functions.network_shutdown(net)



if __name__ == "__main__":
    main()

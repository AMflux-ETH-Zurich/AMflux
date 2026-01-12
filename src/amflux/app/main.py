"""
CANopen EPOS4 Configuration Utilities.

This module provides a structured, high-level interface for initializing and
configuring Maxon EPOS4 motor controllers via CANopen (CiA 301).

It includes:
    - CAN network setup and shutdown utilities
    - Node scanning and rate-limited SDO access
    - Initialization routines for motor, encoder, homing, control loops, etc.
    - A validation system (`check_init`) to prevent uninitialized parameters
    - Numerous EPOS4 configuration helpers following the Communication Guide

Structure:
    1. Auxiliary Helpers
    2. CAN Network Management
    3. Device Initialization Functions
    4. State Machine Control

This module intentionally mirrors the structure of the EPOS4 documentation
(Communication Guide, Firmware Specification), providing clear mapping from
code to documentation section numbers (e.g., "6.2.58").
"""


# ======================================================================
# Imports
# ======================================================================

import time
import canopen
import keyboard
import can
import toml
import utils
import can_functions
import object_dictionary_functions


import warnings
from typing import Mapping, Hashable, Dict, Any
from errors import InitializationError, DriveStateDetError, DriveStatePathError, DesiredDriveStateError, DriveStateResetError, InitObjDict, DesiredMode, SanityCheck

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
# Region: Main
# ======================================================================

def main():
    global net
    # Setup our CANopen network
    network, mc1 = can_functions.network_setup(1, '/home/amfluxpi/AMflux/src/amflux/app/Epos4_70_15.eds', 'can0', net)
    
    can_functions.sanity_check(network)



    object_dictionary_functions.rxpdo_mapping_init(mc1)
    object_dictionary_functions.txpdo_mapping_init(mc1)

    object_dictionary_functions.axis_configuration_init(mc1)





    ###drive_run(mc1, OperationModes.CyclicSynchronousPosition, timeout=5, operation_time=5)

    ###print("Drive completed run. Network will be shutdown")

    #OUTPUT CHECK SSI ENCODER
    utils.SSI_encoder_output_check(mc1, interval = 0.01, duration = 20)

    can_functions.network_shutdown(net)

    '''
    # CST mode is under int=10 (Firmware-Specification, p219)
    motor_setup(mc1, 10)
    # Run the motor in cst mode using a given amount of rpm
    motor_run_cst(mc1, 1000, 2)
    # Shutdown the CANopen network after use
    network_shutdown()
    '''

    

if __name__ == "__main__":
    #main()
    #network_scan('can0')
    main()

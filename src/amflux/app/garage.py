# ======================================================================
# Region: GARAGE
# ======================================================================


def OLD_quick_statusword_test(channel: str = "can", node_id: int=1):
    # Adjust EDS path if needed
    eds_path = "/home/amfluxpi/AMflux/src/amflux/app/Epos4_70_15.eds"
    

    net, node = network_setup(node_id, eds_path, channel)

    try:
        sw = sword(node)
        # Optional: quick decode of basic state bits (0–3)
        ready_to_switch_on = bool(sw & (1 << 0))
        switched_on        = bool(sw & (1 << 1))
        operation_enabled  = bool(sw & (1 << 2))
        fault              = bool(sw & (1 << 3))

        print(f"Statusword: {sw:#06x}")

        print("Decoded state bits:")
        print(f"  Ready to switch on : {ready_to_switch_on}")
        print(f"  Switched on        : {switched_on}")
        print(f"  Operation enabled  : {operation_enabled}")
        print(f"  Fault              : {fault}")
    finally:
        network_shutdown()


def OLD_motor_run_cst(node, torque: int, duration: float):

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
            #quick_stop(node)
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


def OLD_network_scan(node_channel: str):
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
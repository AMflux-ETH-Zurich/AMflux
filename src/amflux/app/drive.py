import queue
import toml
import object_dictionary_functions
from errors import InitializationError, DriveStateDetError, DriveStatePathError, DesiredDriveStateError, DriveStateResetError, InitObjDict, DesiredMode, SanityCheck
from threading import Thread, Event
#from main import goto_state
import time
from can_functions import cword_read
from can_functions import cword_write
from can_functions import sword





# ======================================================================
# Sub-Region: DRIVE STATE FUNCTIONS
# ======================================================================


class DriveState:
    """
    Represents the operational state of a drive system.

    This class defines a finite state machine with 8 distinct states that a drive
    can transition through, ranging from disabled/not ready to fully operational or fault
    conditions. Each state represents a specific operational mode or condition of the drive.

    Attributes:
        NOT_READY_TO_SWITCH_ON (int): Drive is not ready to be switched on (state 0).
        SWITCH_ON_DISABLED (int): Switching on is disabled (state 1).
        READY_TO_SWITCH_ON (int): Drive is ready to be switched on (state 2).
        SWITCHED_ON (int): Drive is switched on but not operating (state 3).
        OPERATION_ENABLED (int): Drive is actively operating (state 4).
        QUICK_STOP_ACTIVE (int): Quick stop is active, drive is stopping (state 5).
        FAULT_REACTION_ACTIVE (int): Drive is reacting to a fault condition (state 6).
        FAULT (int): Drive has encountered a fault (state 7).
        state_flags (dict): Mapping of state indices to boolean flags for state tracking.
    """ 
    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED     = 1
    READY_TO_SWITCH_ON     = 2
    SWITCHED_ON            = 3
    OPERATION_ENABLED      = 4
    QUICK_STOP_ACTIVE      = 5
    FAULT_REACTION_ACTIVE  = 6
    FAULT                  = 7
    

def get_DriveState(node) -> DriveState:
    """
    Determine the drive state based on the statusword bits.
    This function extracts specific bits from a statusword obtained from a node
    and maps the bit pattern to a corresponding DriveState enum value. It evaluates
    the state machine bits (Ready to switch on, Switched on, Operation enabled, Fault,
    Quick stop, and Switch on disabled) to determine the current drive operational state.


    Args:
        node (int): The node object from which to retrieve the statusword.

    Raises:
        DriveStateDetError: If the statusword bit pattern does not match any known 
        drive state, indicating an invalid or unexpected state combination.
    
    Returns:
        DriveState:  An enum value representing the current drive state, which can be:
                        - NOT_READY_TO_SWITCH_ON
                        - SWITCH_ON_DISABLED
                        - READY_TO_SWITCH_ON
                        - SWITCHED_ON
                        - OPERATION_ENABLED
                        - QUICK_STOP_ACTIVE
                        - FAULT_REACTION_ACTIVE
                        - FAULT
    """      
    statusword = sword(node)

    b0 = (statusword >> 0) & 1  # Ready to switch on (compares right most bit)
    b1 = (statusword >> 1) & 1  # Switched on (right most bit falls off, now compare second bit)
    b2 = (statusword >> 2) & 1  # Operation enabled
    b3 = (statusword >> 3) & 1  # Fault
    b5 = (statusword >> 5) & 1  # Quick stop
    b6 = (statusword >> 6) & 1  # Switch on disabled

    # Not ready to switch on
    if (b6 == 0 and b5 == 0 and b3 == 0 and b2 == 0 and b1 == 0 and b0 == 0):
        return DriveState.NOT_READY_TO_SWITCH_ON

    # Switch on disabled
    if (b6 == 1 and b5 == 0 and b3 == 0 and b2 == 0 and b1 == 0 and b0 == 0):
        return DriveState.SWITCH_ON_DISABLED

    # Ready to switch on
    if (b6 == 0 and b5 == 1 and b3 == 0 and b2 == 0 and b1 == 0 and b0 == 1):
        return DriveState.READY_TO_SWITCH_ON

    # Switched on
    if (b6 == 0 and b5 == 1 and b3 == 0 and b2 == 0 and b1 == 1 and b0 == 1):
        return DriveState.SWITCHED_ON

    # Operation enabled
    if (b6 == 0 and b5 == 1 and b3 == 0 and b2 == 1 and b1 == 1 and b0 == 1):
        return DriveState.OPERATION_ENABLED

    # Quick stop active
    if (b6 == 0 and b5 == 0 and b3 == 0 and b2 == 1 and b1 == 1 and b0 == 1):
        return DriveState.QUICK_STOP_ACTIVE

    # Fault reaction active
    if (b6 == 0 and b5 == 1 and b3 == 1 and b2 == 1 and b1 == 1 and b0 == 1):
        return DriveState.FAULT_REACTION_ACTIVE

    # Fault
    if (b6 == 0 and b5 == 1 and b3 == 1 and b2 == 0 and b1 == 0 and b0 == 0):
        return DriveState.FAULT
    
    raise DriveStateDetError(f"Unknown drive state with statusword: {statusword:#04x}")
    

# ======================================================================
# Sub-Region: DRIVE COMMAND FUNCTIONS
# ======================================================================

class DriveCommand: 
    """
    Enumeration of CiA 402 drive commands for motor control operations.

    This class defines standard control commands used to manage the operational state
    of a drive, including startup, shutdown, and fault handling procedures following
    the CANopen DS402 device profile specification.
    """
    
    SHUTDOWN            = 0b00000110 #maybe normal ints
    SWITCH_ON           = 0b00000111
    ENABLE_OPERATION    = 0b00001111
    DISABLE_VOLTAGE     = 0b00000000
    QUICK_STOP          = 0b00000010
    DISABLE_OPERATION   = 0b00000111           
    FAULT_RESET         = 0b10000000


DriveStateMap = {DriveState.NOT_READY_TO_SWITCH_ON : [(DriveCommand.SWITCH_ON, DriveState.OPERATION_ENABLED)], 
                  DriveState.SWITCH_ON_DISABLED     : [(DriveCommand.SHUTDOWN, DriveState.READY_TO_SWITCH_ON)], 
                  DriveState.READY_TO_SWITCH_ON     : [(DriveCommand.DISABLE_VOLTAGE, DriveState.SWITCH_ON_DISABLED), 
                                                       (DriveCommand.SWITCH_ON, DriveState.SWITCHED_ON), 
                                                       (DriveCommand.ENABLE_OPERATION, DriveState.OPERATION_ENABLED)],
                  DriveState.SWITCHED_ON            : [(DriveCommand.SHUTDOWN, DriveState.READY_TO_SWITCH_ON),
                                                       (DriveCommand.DISABLE_OPERATION, DriveState.SWITCH_ON_DISABLED), 
                                                       (DriveCommand.ENABLE_OPERATION, DriveState.OPERATION_ENABLED)],
                  DriveState.OPERATION_ENABLED      : [(DriveCommand.DISABLE_OPERATION, DriveState.SWITCHED_ON), 
                                                       (DriveCommand.SHUTDOWN, DriveState.READY_TO_SWITCH_ON), 
                                                       (DriveCommand.DISABLE_VOLTAGE, DriveState.SWITCH_ON_DISABLED), 
                                                       (DriveCommand.QUICK_STOP, DriveState.QUICK_STOP_ACTIVE)],
                  DriveState.QUICK_STOP_ACTIVE      : [(DriveCommand.ENABLE_OPERATION, DriveState.OPERATION_ENABLED), 
                                                       (DriveCommand.DISABLE_VOLTAGE, DriveState.SWITCH_ON_DISABLED)],
                  DriveState.FAULT                  : [(DriveCommand.FAULT_RESET, DriveState.SWITCH_ON_DISABLED)]}


def fault_reset(node, reset_tries: int = 1, timeout: float = 5.0) -> None:
    """Resets the drive from a FAULT state to SWITCH_ON_DISABLED state.
    
    Args:
        node: The CANopen node representing the drive.
        reset_tries (int): Number of attempts to reset the fault.
        timeout (float): Maximum time to wait for the reset to complete.
        
    Raises:
        DriveStateResetError: If the drive does not reach SWITCH_ON_DISABLED state after the specified number of attempts.
    """
    for _ in range(reset_tries):
        current_cword = cword_read(node)
        new_cword = current_cword | 0x0080
        cword_write(node, new_cword)
        time.sleep(0.1)
        if get_DriveState(node) ==  DriveState.SWITCH_ON_DISABLED:
            return
    raise DriveStateResetError(f"Failed to reach Fault_RESET in {reset_tries} attempts")


def wait_for_state(node, desired_state: int, timeout: float = 5.0):
    """Gives the drive some time to reach the desired state.
    
    Args:
        node: The CANopen node representing the drive.
        desired_state (int): The target drive state to wait for.
        timeout (float): Maximum time to wait for the desired state.
        
    Raises:
        TimeoutError: If the drive does not reach the desired state within the timeout period.
    """
    
    start_time = time.time()

    while True:                                                                    
        current_state = get_DriveState(node) 
        if current_state == DriveState.FAULT: 
            print("ATTENTION: Fault Reaction Active. Reset Drive? (3 tries)")
            if utils.confirm():
                fault_reset(node, reset_tries=3, timeout=timeout)
                time.sleep(0.05)
        if current_state == desired_state:
            return True
        if time.time() - start_time > timeout:
            raise TimeoutError(f"Timeout waiting for state {desired_state}, current state is {current_state}")
        time.sleep(0.01)  # Avoid busy waiting


def do_DriveCommand(node, command: int, target: DriveState, timeout: int) -> True:
    """
    Sets the DS402 Controlword for the given command on node and waits until the drive reaches the target state within timeout. 
    Special-cases FAULT_RESET by calling fault_reset() instead of writing the normal command bits.
    
    Args:
        node (int): EPOS4 Node
        command (DriveCommand): desired DriveCommand
        target (DriveState): expected DriveState after completing function
        timeout (int)

    Raises:
        TimeoutError: raises if State cannot be switched within timeout

    Returns:
        True: returns True if successfull, else Exception error is raised.
    """    
    current_cword = cword_read(node)

    # Special case: Fault reset (set bit 7, keep all other bits)
    if command == DriveCommand.FAULT_RESET:
        fault_reset(node, reset_tries=3, timeout=timeout)
        return True

    low  = command & 0x00FF #Otto sagt aufpassen!
    high = current_cword & 0xFF00
    new_cword = high | low

    cword_write(node, new_cword)

    if(wait_for_state(node, target, timeout)):
        return #True

    raise TimeoutError(f"Failed to reach state {target} in time")


def shutdown_drive(node):
    current_state = get_DriveState(node)

    if current_state == DriveState.OPERATION_ENABLED or current_state == DriveState.QUICK_STOP_ACTIVE:
        do_DriveCommand(DriveCommand.DISABLE_VOLTAGE)
    elif current_state == DriveState.SWITCHED_ON:
        do_DriveCommand(DriveCommand.SHUTDOWN)
    

def Drive_State_BFS(start_state, target_state, Drive_State_map):
    """Naive breadth-firs-search algorithm to traverse DriveState map.  
       ATTENTION: THIS FUNCTION IS 100% WRITTEN BY AI

    Args:
        start_state (DriveState): Current drive state.
        target_state (DriveState): Desired drive state.
        drive_state_map (dict): Mapping {state: [(command, next_state), ...]}.

    Returns:
        list[tuple[DriveCommand, DriveState]] | None:
            A route as (command, reached_state) steps, or None if no path exists.
    """    
    
    if start_state == target_state:
        return []
    
    queue = [start_state]
    visited = {start_state}
    parent = {}

    while queue:
        current = queue.pop(0)

        if current == target_state:
            break
        
        for command, next_state in Drive_State_map.get(current, []):
            if next_state not in visited:
                visited.add(next_state)
                parent[next_state] = (current, command)
                queue.append(next_state)

    
    if target_state not in visited:
        return None
    
    route = []
    state = target_state

    while state != start_state:
        prev_state, command = parent[state]
        route.append((command, state))
        state = prev_state

    route.reverse()
    return route


def goto_state(node, desired_state, timeout):
    """Traverses state-machine to get to desired DriveState

    Args:
        node (int): EPOS4 node
        desired_state (DriveState): State to be reached.
        timeout (int): timeout for do_DriveCommand()

    Raises:
        DriveStatePathError: raised if route cannot be found/is not valid
        DesiredDriveStateError: raised if goto_state() fails generally
    """   
    current_state = get_DriveState(node)
    
    route = Drive_State_BFS(current_state, desired_state, DriveStateMap)

    if route is None:
        raise DriveStatePathError(f"No valid path to {desired_state} was found")
    
    for step in route:
        next_command = step[0]
        next_state = step[1]
        try:
            do_DriveCommand(node, next_command, next_state, timeout)
        except DriveStateDetError as e:
            # Specific drive-state errors you expect
            shutdown_drive(node)
            print(f"Drive state determination error: {e}")
            raise
        except TimeoutError as e:
            # Timeout errors from nested calls
            shutdown_drive(node)
            print(f"Timeout during state transition: {e}")
            raise
        except Exception as e:
            # Unexpected errors—log and fail fast
            shutdown_drive(node)
            print(f"Unexpected error: {e} current State: {get_DriveState(node)}")
            raise
        finally:
           # Cleanup that always runs: e.g., disable watchdog, log state, etc.
            print(f"Attempted transition to {next_state}")

        
    final_state = get_DriveState(node)
    
    if final_state == desired_state:
        return
    else:
        raise DesiredDriveStateError(f"{desired_state} state could not be reached. Current state: {final_state}")
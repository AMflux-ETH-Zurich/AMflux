"""
Drive State Management Module

This module manages the operational state of CANopen-based drive systems following the
DS402 device profile specification. It provides functionality for:

- Drive state detection and monitoring (8-state finite state machine)
- Drive command execution (shutdown, switch on, enable operation, etc.)
- State transitions using breadth-first search pathfinding
- Fault detection and recovery procedures
- Timeout-protected state waiting and validation

Key Components:
    DriveState: Enumeration of 8 drive operational states
    DriveCommand: Standard CiA 402 control commands
    DriveStateMap: State transition graph mapping valid state transitions
    
Functions:
    get_DriveState(): Determine current drive state from statusword
    execute_transition(): Execute one legal state transition
    goto_state(): Traverse state machine to desired state using BFS
    fault_reset(): Reset drive from FAULT state
    wait_for_state(): Poll drive state with timeout protection

Dependencies:
    can_functions: CAN communication primitives
    errors: Custom exception definitions
"""


# ======================================================================
# Imports
# ======================================================================

from collections import deque
import time
from errors import DriveStateDetError, DriveStatePathError, DesiredDriveStateError, DriveStateResetError
from can_functions import cword_write
from can_functions import sword


# ======================================================================
# Drive State Functions
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

STATE_NAMES = {
    DriveState.NOT_READY_TO_SWITCH_ON: "NOT_READY_TO_SWITCH_ON",
    DriveState.SWITCH_ON_DISABLED: "SWITCH_ON_DISABLED",
    DriveState.READY_TO_SWITCH_ON: "READY_TO_SWITCH_ON",
    DriveState.SWITCHED_ON: "SWITCHED_ON",
    DriveState.OPERATION_ENABLED: "OPERATION_ENABLED",
    DriveState.QUICK_STOP_ACTIVE: "QUICK_STOP_ACTIVE",
    DriveState.FAULT_REACTION_ACTIVE: "FAULT_REACTION_ACTIVE",
    DriveState.FAULT: "FAULT",
}


STATE_MASKS = {
    DriveState.NOT_READY_TO_SWITCH_ON: (0x4F, 0x00),
    DriveState.SWITCH_ON_DISABLED: (0x4F, 0x40),
    DriveState.READY_TO_SWITCH_ON: (0x6F, 0x21),
    DriveState.SWITCHED_ON: (0x6F, 0x23),
    DriveState.OPERATION_ENABLED: (0x6F, 0x27),
    DriveState.QUICK_STOP_ACTIVE: (0x6F, 0x07),
    DriveState.FAULT_REACTION_ACTIVE: (0x4F, 0x0F),
    DriveState.FAULT: (0x4F, 0x08),
}


def format_drive_state(state: int) -> str:
    """Return a readable state label for logs and exceptions."""
    return f"{STATE_NAMES.get(state, 'UNKNOWN_STATE')} ({state})"


def get_DriveState(node) -> DriveState:
    """Determine the drive state from the DS402 statusword."""
    #statusword = sword(node)
    print(f"get_DriveState {statusword}")
    for state, (bitmask, bits) in STATE_MASKS.items():
        if statusword & bitmask == bits:
            return state

    raise DriveStateDetError(f"Unknown drive state with statusword: {statusword:#06x}")


# ======================================================================
# Drive command functions
# ======================================================================

class DriveCommand: 
    """
    Enumeration of CiA 402 drive commands for motor control operations.

    This class defines standard control commands used to manage the operational state
    of a drive, including startup, shutdown, and fault handling procedures following
    the CANopen DS402 device profile specification.
    """
    
    SHUTDOWN            = 0b00000110 
    SWITCH_ON           = 0b00000111
    ENABLE_OPERATION    = 0b00001111
    DISABLE_VOLTAGE     = 0b00000000
    QUICK_STOP          = 0b00000010
    DISABLE_OPERATION   = 0b00000111
    FAULT_RESET         = 0b10000000


TRANSITION_COMMANDS = {
    (DriveState.NOT_READY_TO_SWITCH_ON, DriveState.SWITCH_ON_DISABLED): None,
    (DriveState.FAULT_REACTION_ACTIVE, DriveState.FAULT): None,
    (DriveState.FAULT, DriveState.SWITCH_ON_DISABLED): DriveCommand.FAULT_RESET,
    (DriveState.SWITCH_ON_DISABLED, DriveState.READY_TO_SWITCH_ON): DriveCommand.SHUTDOWN,
    (DriveState.READY_TO_SWITCH_ON, DriveState.SWITCHED_ON): DriveCommand.SWITCH_ON,
    (DriveState.SWITCHED_ON, DriveState.OPERATION_ENABLED): DriveCommand.ENABLE_OPERATION,
    (DriveState.OPERATION_ENABLED, DriveState.SWITCHED_ON): DriveCommand.DISABLE_OPERATION,
    (DriveState.SWITCHED_ON, DriveState.READY_TO_SWITCH_ON): DriveCommand.SHUTDOWN,
    (DriveState.READY_TO_SWITCH_ON, DriveState.SWITCH_ON_DISABLED): DriveCommand.DISABLE_VOLTAGE,
    (DriveState.OPERATION_ENABLED, DriveState.READY_TO_SWITCH_ON): DriveCommand.SHUTDOWN,
    (DriveState.OPERATION_ENABLED, DriveState.SWITCH_ON_DISABLED): DriveCommand.DISABLE_VOLTAGE,
    (DriveState.OPERATION_ENABLED, DriveState.QUICK_STOP_ACTIVE): DriveCommand.QUICK_STOP,
    (DriveState.QUICK_STOP_ACTIVE, DriveState.OPERATION_ENABLED): DriveCommand.ENABLE_OPERATION,
    (DriveState.QUICK_STOP_ACTIVE, DriveState.SWITCH_ON_DISABLED): DriveCommand.DISABLE_VOLTAGE,
}


DriveStateMap = {state: [] for state in STATE_NAMES}
for (from_state, to_state), _controlword in TRANSITION_COMMANDS.items():
    DriveStateMap[from_state].append(to_state)


def fault_reset(node, reset_tries: int = 1, timeout: float = 5.0) -> None:
    """Reset the drive from FAULT to SWITCH_ON_DISABLED."""
    deadline = time.monotonic() + timeout

    for _ in range(reset_tries):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break

        # Generate a clean rising edge on bit 7 for fault reset.
        cword_write(node, DriveCommand.DISABLE_VOLTAGE)
        time.sleep(0.05)
        cword_write(node, DriveCommand.FAULT_RESET)
        try:
            wait_for_state(node, DriveState.SWITCH_ON_DISABLED, remaining)
            return
        except TimeoutError:
            time.sleep(0.05)

    raise DriveStateResetError(
        f"Failed to reset drive from {format_drive_state(DriveState.FAULT)} "
        f"to {format_drive_state(DriveState.SWITCH_ON_DISABLED)}"
    )


def wait_for_state(node, desired_state: int, timeout: float = 5.0):
    """Poll until the drive reaches the desired state or the timeout expires."""
    start_time = time.monotonic()

    while True:                                                                    
        current_state = get_DriveState(node)
        if current_state == desired_state:
            return True
        if time.monotonic() - start_time > timeout:
            raise TimeoutError(
                f"Timeout waiting for state {format_drive_state(desired_state)}, "
                f"current state is {format_drive_state(current_state)}"
            )
        time.sleep(0.01)  # Avoid busy waiting


def execute_transition(node, from_state: int, to_state: int, timeout: float) -> None:
    """Execute one legal DS402 state transition."""
    try:
        controlword = TRANSITION_COMMANDS[(from_state, to_state)]
    except KeyError as exc:
        raise DriveStatePathError(
            f"Illegal state transition from {format_drive_state(from_state)} "
            f"to {format_drive_state(to_state)}"
        ) from exc

    if controlword is None:
        wait_for_state(node, to_state, timeout)
        return

    if controlword == DriveCommand.FAULT_RESET:
        fault_reset(node, reset_tries=3, timeout=timeout)
        return

    cword_write(node, controlword)
    wait_for_state(node, to_state, timeout)


def shutdown_drive(node, timeout: float = 5.0):
    """Drive the node into SWITCH_ON_DISABLED."""
    goto_state(node, DriveState.SWITCH_ON_DISABLED, timeout)
    

def DriveState_BFS(start_state, target_state, Drive_State_map):
    """Return the shortest state path between start_state and target_state."""
    print(f"BFS: {format_drive_state(start_state)} -> {format_drive_state(target_state)}")
    if start_state == target_state:
        return [start_state]
    
    queue = deque([start_state])
    parent = {start_state: None}

    while queue:
        current = queue.popleft()
        
        for next_state in Drive_State_map.get(current, []):
            if next_state not in parent:
                parent[next_state] = current
                if next_state == target_state:
                    queue.clear()
                    break
                queue.append(next_state)

    if target_state not in parent:
        return None
    
    route = []
    state = target_state

    while state is not None:
        route.append(state)
        state = parent[state]

    route.reverse()
    return route


def goto_state(node, desired_state, timeout):
    """Traverse the DS402 state machine to reach the desired DriveState.

    Args:
        node (int): EPOS4 node
        desired_state (DriveState): State to be reached.
        timeout (int): Overall timeout for reaching the desired state.

    Raises:
        DriveStatePathError: raised if route cannot be found/is not valid
        DesiredDriveStateError: raised if goto_state() times out generally
    """
    deadline = time.monotonic() + timeout

    while True:
        current_state = get_DriveState(node)
        if current_state == desired_state:
            print(f"transition to {format_drive_state(desired_state)} successfull")
            return

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise DesiredDriveStateError(
                f"{format_drive_state(desired_state)} state could not be reached. "
                f"Current state: {format_drive_state(current_state)}"
            )

        route = DriveState_BFS(current_state, desired_state, DriveStateMap)
        if route is None or len(route) < 2:
            raise DriveStatePathError(
                f"No valid path from {format_drive_state(current_state)} "
                f"to {format_drive_state(desired_state)} was found"
            )

        next_state = route[1]
        execute_transition(node, current_state, next_state, remaining)
        print(f"Attempted transition to {format_drive_state(next_state)}")


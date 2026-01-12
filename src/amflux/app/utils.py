import time
import canopen
import keyboard
import can
import toml
import warnings

from typing import Mapping, Hashable, Dict, Any
from errors import InitializationError, DriveStateDetError, DriveStatePathError, DesiredDriveStateError, DriveStateResetError, InitObjDict, DesiredMode, SanityCheck

def removekey(d: Mapping[str, Any], key: Hashable) -> Dict[str, Any]:
    """Return a shallow copy of mapping `d` with `key` removed."""
    r = dict(d)
    # Use pop to avoid KeyError if `key` does not exist in the mapping
    r.pop(key, None)
    return r


def confirm(prompt: str = "Continue? [y/N]: ") -> bool:
    """Prompt the user for a yes/no answer and return True for yes.

    Accepts 'y', 'yes', 'j', 'ja' (case-insensitive) as affirmative answers.
    Any other response is treated as negative.
    """
    ans = input(prompt).strip().lower()
    yes_answers = {"y", "yes", "j", "ja"}
    return ans in yes_answers


def check_init(objects):
    """Checks if all objects of function to be checked have been initialized correctly.

    Args:
        objects (dict): dictionary of objects to be checked. Name, Value pairs.

    Raises:
        InitializationError: After Warning, user must confirm Default value or enter new value. InitializationError is raised if value is still not valid.
        InitializationError: Raised if no default value available. FATAL ERROR 67.

    Returns True when validation is complete and execution may continue.
    """
    objects_clean = removekey(objects, 'node')
    for name, value in objects_clean.items():
        # warn user about default value
        if value is None:
            warnings.warn(f"{name} is set to default value.", UserWarning)
            if not confirm("Continue with default value? [y/N]: "):
                new_value = input(f"Enter a valid value for {name}: ")
                try:
                    objects[name] = int(new_value)
                except ValueError:
                    raise InitializationError(f"{name} must be int.")
            else:
                continue
        # fatal, user must init value correctly
        if value is not None and (value == "67" or value == 67):
            raise InitializationError(f"{name} is not initialized correctly. Please enter a valid value for {name}.")
    return True 

def SSI_encoder_output_check(node, interval:float = 0.25, duration: int = 60):
    """Checks if the SSI encoder is outputting data.

    Args:
        node: canopen Node object representing the SSI encoder.
    
    Returns:
        Prints SSI absolute position every [Interval] seconds until [Duration] is up.
    """

    """start_time = time.time()
    while (time.time()-start_time) < duration:
        pos_val = node.sdo[0x3012][0x0D].raw
        b = node.sdo[0x3012][0x0D].raw

        if abs(pos_val - b) > 1000:
            print("Torn read detected", pos_val, b)
        print(f"Current SSI absolute position: {pos_val}")
        time.sleep(interval)"""
    
    start_time = time.time()
    while (time.time()-start_time) < duration:
        pos_val = node.tpdo[2]['Position Actual Value'].raw
        
        print(f"Current SSI absolute position: {pos_val}")
        time.sleep(interval)
    




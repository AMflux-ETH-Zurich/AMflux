"""DCF helpers for EPOS4 controller configuration.

The drive configuration itself is applied through python-canopen's
``RemoteNode.load_configuration()``. This module only resolves the DCF path and
extracts GUI command defaults from DCF ``ParameterValue`` entries.
"""

import os
from functools import lru_cache
from pathlib import Path


_MODE_COMMAND_DCF_MAP = {
    "PPM": {
        "target_position": {"position": (0x607A, 0)},
        "profile_velocity": {"prof_velocity": (0x6081, 0)},
        "profile_acceleration": {"prof_acc": (0x6083, 0)},
        "profile_deceleration": {"prof_dec": (0x6084, 0)},
    },
    "HMM": {
        "homing_method_init": {"homing_method": (0x6098, 0)},
        "homing_speeds": {
            "speed_sw_srch": (0x6099, 1),
            "speed_zero_srch": (0x6099, 2),
        },
        "homing_acceleration": {"homing_acc": (0x609A, 0)},
        "home_offset_distance_init": {"home_offset_distance": (0x30B1, 0)},
        "home_position_init": {"homeposition": (0x30B0, 0)},
        "current_threshold_homing_init": {
            "current_threshold_homing": (0x30B2, 0),
        },
    },
    "PVM": {
        "target_velocity": {"vel": (0x60FF, 0)},
        "profile_acceleration": {"prof_acc": (0x6083, 0)},
        "profile_deceleration": {"prof_dec": (0x6084, 0)},
        "motion_profile_type": {"profile": (0x6086, 0)},
    },
    "CSP": {
        "target_position": {"position": (0x607A, 0)},
        "position_offset": {"pos_offset": (0x60B0, 0)},
        "torque_offset": {"tor_offset": (0x60B2, 0)},
    },
    "CSV": {
        "target_velocity": {"vel": (0x60FF, 0)},
        "velocity_offset": {"vel_offset": (0x60B1, 0)},
        "torque_offset": {"tor_offset": (0x60B2, 0)},
    },
    "CST": {
        "target_torque": {"torque": (0x6071, 0)},
        "torque_offset": {"tor_offset": (0x60B2, 0)},
    },
}

_DCF_LOAD_SKIP_INDEXES = {
    0x1010,  # Store parameters
    0x1011,  # Restore default parameters
    0x1F50,  # Program data
    0x1F51,  # Program control
    0x1F56,  # Program software identification
    0x1F57,  # Flash status identification
    0x6040,  # Controlword
}


def resolve_dcf_path(dcf_path=None) -> Path:
    if dcf_path is not None:
        path = Path(dcf_path).expanduser()
        if path.exists():
            return path
        raise FileNotFoundError(f"DCF file not found: {path}")

    env_path = os.getenv("AMFLUX_DCF_PATH")
    candidates = []
    if env_path:
        candidates.append(Path(env_path).expanduser())

    module_dir = Path(__file__).resolve().parent
    candidates.extend(
        [
            module_dir / "test.dcf",
            module_dir / "export_for_objdict.dcf",
            Path("/home/amfluxpi/AMflux/src/amflux/app/test.dcf"),
            Path("/home/amfluxpi/AMflux/src/amflux/app/export_for_objdict.dcf"),
        ]
    )

    for candidate in candidates:
        if candidate.exists():
            return candidate

    searched = "\n".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(
        "No DCF file found. Set AMFLUX_DCF_PATH or place a DCF at one of:\n"
        f"{searched}"
    )


def _import_dcf(dcf_path):
    import canopen

    return canopen.import_od(str(resolve_dcf_path(dcf_path)))


def _od_variable(od, index, subindex=0):
    obj = od[index]
    if subindex == 0 and hasattr(obj, "value"):
        return obj
    return obj[subindex]


def _iter_od_variables(obj):
    if hasattr(obj, "value"):
        yield obj
    else:
        for subobj in obj.values():
            if hasattr(subobj, "value"):
                yield subobj


def disable_command_parameter_values(node) -> int:
    """Prevent load_configuration() from writing command-like DCF entries."""
    disabled_count = 0
    for index in _DCF_LOAD_SKIP_INDEXES:
        if index not in node.object_dictionary:
            continue
        for variable in _iter_od_variables(node.object_dictionary[index]):
            if variable.value is not None:
                variable.value = None
                disabled_count += 1

    if disabled_count:
        print(f"disabled {disabled_count} command ParameterValue entries before DCF load")
    return disabled_count


def _od_command_entry(od, index, subindex=0):
    try:
        variable = _od_variable(od, index, subindex)
        name = variable.name
        value = variable.value
    except Exception:
        name = f"0x{index:04X}:{subindex:02X}"
        value = None

    return {
        "index": index,
        "subindex": subindex,
        "name": name,
        "value": value,
    }


def _mode_command_defaults(od):
    mode_config = {}
    for mode_code, command_map in _MODE_COMMAND_DCF_MAP.items():
        comm = {}
        for command_name, entry_map in command_map.items():
            comm[command_name] = {
                arg_name: _od_command_entry(od, index, subindex)
                for arg_name, (index, subindex) in entry_map.items()
            }
        mode_config[mode_code] = {"comm": comm}
    return mode_config


@lru_cache(maxsize=4)
def load_drive_configuration(dcf_path=None) -> dict:
    config_path = resolve_dcf_path(dcf_path)
    od = _import_dcf(config_path)
    return {
        "dcf_path": str(config_path),
        "mode": _mode_command_defaults(od),
    }

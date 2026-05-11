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
        "Target position": {"Target position": (0x607A, 0)},
        "Profile velocity": {"Profile velocity": (0x6081, 0)},
        "Profile acceleration": {"Profile acceleration": (0x6083, 0)},
        "Profile deceleration": {"Profile deceleration": (0x6084, 0)},
    },
    "HMM": {
        "Homing method": {"Homing method": (0x6098, 0)},
        "Homing speeds": {
            "Speed for switch search": (0x6099, 1),
            "Speed for zero search": (0x6099, 2),
        },
        "Homing acceleration": {"Homing acceleration": (0x609A, 0)},
        "Home offset move distance": {"Home offset move distance": (0x30B1, 0)},
        "Home position": {"Home position": (0x30B0, 0)},
        "Current threshold for homing mode": {
            "Current threshold for homing mode": (0x30B2, 0),
        },
    },
    "PVM": {
        "Target velocity": {"Target velocity": (0x60FF, 0)},
        "Profile acceleration": {"Profile acceleration": (0x6083, 0)},
        "Profile deceleration": {"Profile deceleration": (0x6084, 0)},
        "Motion profile type": {"Motion profile type": (0x6086, 0)},
    },
    "CSP": {
        "Target position": {"Target position": (0x607A, 0)},
        "Position controller P gain": {"Position controller P gain": (0x30A1, 1)},
        "Position controller I gain": {"Position controller I gain": (0x30A1, 2)},
        "Position controller D gain": {"Position controller D gain": (0x30A1, 3)},
        "Position offset": {"Position offset": (0x60B0, 0)},
        "Torque offset": {"Torque offset": (0x60B2, 0)},
    },
    "CSV": {
        "Target velocity": {"Target velocity": (0x60FF, 0)},
        "Velocity offset": {"Velocity offset": (0x60B1, 0)},
        "Torque offset": {"Torque offset": (0x60B2, 0)},
    },
    "CST": {
        "Target torque": {"Target torque": (0x6071, 0)},
        "Torque offset": {"Torque offset": (0x60B2, 0)},
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

_DCF_LOAD_SKIP_OBJECTS = {
    od_ref
    for command_map in _MODE_COMMAND_DCF_MAP.values()
    for entry_map in command_map.values()
    for od_ref in entry_map.values()
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
    """Prevent load_configuration() from writing command/runtime DCF entries."""
    disabled_count = 0
    for index in _DCF_LOAD_SKIP_INDEXES:
        if index not in node.object_dictionary:
            continue
        for variable in _iter_od_variables(node.object_dictionary[index]):
            if variable.value is not None:
                variable.value = None
                disabled_count += 1

    for index, subindex in _DCF_LOAD_SKIP_OBJECTS:
        if index not in node.object_dictionary:
            continue
        try:
            variable = _od_variable(node.object_dictionary, index, subindex)
        except Exception:
            continue
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

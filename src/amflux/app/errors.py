class InitializationError(Exception):
    """Raised when a configuration parameter has not been initialized correctly."""
    pass


class DriveStateDetError(Exception):
    """Raised when Drive State cannot be determined."""
    pass


class DriveStatePathError(Exception):
    """Raised if no valid Path is found/used"""
    pass


class DesiredDriveStateError(Exception):
    """Raised if desired DriveState is not reached"""
    pass


class DriveStateResetError(Exception):
    """Raised if DriveState cannot be reset from FAULT"""
    pass

class InitObjDict(Exception):
    """Raised if object dictionary is not initializable"""
    pass


class DesiredMode(Exception):
    """Raised if object dictionary is not initializable"""
    pass

class SanityCheck(Exception):
    """Raised if Sanity Check fails"""
    pass
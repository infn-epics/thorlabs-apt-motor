# ============================================================================
# APT DC Servo Motor Controller (e.g. KDC101, TDC001)
# ============================================================================
# Load motor database
dbLoadTemplate("AptDC.substitutions")

# AptControllerConfig(portName, devicePath, motorType, numAxes, movingPoll, idlePoll)
#
#   portName    - EPICS asyn port name (must match PORT in substitutions)
#   devicePath  - Linux serial device, e.g. /dev/ttyUSB0
#                 Or ser2net address, e.g. 192.168.1.100:4001
#                 Use udev rules for persistent naming:
#                   SUBSYSTEM=="tty", ATTRS{serial}=="27xxxxxx", SYMLINK+="thorlabs_dc0"
#   motorType   - "dc" for DC servo, "stepper" for stepper motors
#   numAxes     - Number of motor axes:
#                   0 = auto-detect from controller HW_GET_INFO
#                   1 = single-channel USB (TDC001, KDC101, ...)
#                   N = multi-channel bay controller with N channels
#   movingPoll  - polling period in seconds while moving
#   idlePoll    - polling period in seconds while idle
#
AptControllerConfig("APT-DC1", "/dev/ttyUSB0", "dc", 1, 0.2, 1.0)

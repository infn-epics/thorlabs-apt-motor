# ============================================================================
# APT Stepper Motor Controller (e.g. KST101, TST001, BSC10x)
# ============================================================================
# Load motor database
dbLoadTemplate("AptStepper.substitutions")

# AptControllerConfig(portName, devicePath, motorType, numAxes, movingPoll, idlePoll)
#
#   portName    - EPICS asyn port name (must match PORT in substitutions)
#   devicePath  - Linux serial device, e.g. /dev/ttyUSB1
#                 Use udev rules for persistent naming:
#                   SUBSYSTEM=="tty", ATTRS{serial}=="26xxxxxx", SYMLINK+="thorlabs_step0"
#   motorType   - "stepper" for stepper motors
#   numAxes     - Number of motor axes:
#                   0 = auto-detect from controller HW_GET_INFO
#                   1 = single-channel USB (TST001, KST101, ...)
#                   N = multi-channel bay controller with N channels
#   movingPoll  - polling period in seconds while moving
#   idlePoll    - polling period in seconds while idle
#
AptControllerConfig("APT-ST1", "/dev/ttyUSB1", "stepper", 1, 0.2, 1.0)

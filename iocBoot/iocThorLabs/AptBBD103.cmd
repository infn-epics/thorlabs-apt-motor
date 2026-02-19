# ============================================================================
# APT BBD103 — 3-Channel Brushless DC Servo Bay Controller
# ============================================================================
#
# The BBD103 is a 3-bay card-slot brushless DC controller.
# Each bay hosts an independent DC servo channel.
# The driver uses APT_RACK (0x11) as the destination and ChanIdent 1-3.
#
# USB serial prefix: 73xxxxxx (BBD103), 103xxxxx (BBD30x)
#
# ============================================================================

# Load motor database — one motor record per axis (ADDR=0,1,2)
dbLoadTemplate("AptBBD103.substitutions")

# AptControllerConfig(portName, devicePath, motorType, numAxes, movingPoll, idlePoll)
#
#   numAxes = 3   → creates 3 axes (ADDR 0,1,2) with APT channels 1,2,3
#   numAxes = 0   → auto-detect from HW_GET_INFO (should detect 3)
#
AptControllerConfig("APT-BBD103", "/dev/ttyUSB0", "dc", 3, 0.2, 1.0)

# Optional: set velocity parameters per axis (in APT internal units)
# AptSetVelParams(portName, axisNum, minVelocity, acceleration, maxVelocity)
# AptSetVelParams("APT-BBD103", 0, 0, 13422, 134218)
# AptSetVelParams("APT-BBD103", 1, 0, 13422, 134218)
# AptSetVelParams("APT-BBD103", 2, 0, 13422, 134218)

# ============================================================================
# APT BSC103 — 3-Channel Stepper Motor Bay Controller
# ============================================================================
#
# The BSC103 is a 3-bay card-slot stepper controller. Each bay hosts an
# independent stepper channel.  The driver uses APT_RACK (0x11) as the
# destination byte and addresses channels 1-3 via ChanIdent in each
# APT data message.
#
# USB serial prefix: 70xxxxxx
#
# ============================================================================

# Load motor database — one motor record per axis (ADDR=0,1,2)
dbLoadTemplate("AptBSC103.substitutions")

# AptControllerConfig(portName, devicePath, motorType, numAxes, movingPoll, idlePoll)
#
#   numAxes = 3   → creates 3 axes (ADDR 0,1,2) with APT channels 1,2,3
#   numAxes = 0   → auto-detect from HW_GET_INFO (should detect 3)
#
AptControllerConfig("APT-BSC103", "/dev/ttyUSB0", "stepper", 3, 0.2, 1.0)

# Optional: set velocity parameters per axis (in APT internal units)
# AptSetVelParams(portName, axisNum, minVelocity, acceleration, maxVelocity)
# AptSetVelParams("APT-BSC103", 0, 0, 263844, 772981)
# AptSetVelParams("APT-BSC103", 1, 0, 263844, 772981)
# AptSetVelParams("APT-BSC103", 2, 0, 263844, 772981)

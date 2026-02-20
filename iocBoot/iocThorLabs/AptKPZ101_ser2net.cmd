# ============================================================================
# APT KIM101 — 4-Channel Piezo Inertial Motor Controller via ser2net
# ============================================================================
#
# The KIM101 is a multi-channel K-Cube Piezo Inertial Motor controller.
# It drives stick-slip piezo actuators (PIA, PIAK series) and uses
# the PZMOT_* APT messages (0x08xx range), which are separate from
# both MOT_* (stepper/DC) and PZ_* (direct piezo) message families.
#
# All 4 channels share one USB serial link, addressed via channel
# bitmask (1,2,4,8) in each PZMOT data message.  The driver auto-
# detects USB vs rack destination (KIM101 → dest=0x50, USB mode).
#
# ser2net v4 config (on the remote host):
#
#   connection: &kim101
#     accepter: tcp,4001
#     connector: serialdev,/dev/thorlabs_kim,115200n81,local
#     options:
#       kickolduser: true
#
# ============================================================================

# Load motor database — one motor record per channel (ADDR=0,1,2,3)
dbLoadTemplate("AptKPZ101_ser2net.substitutions")

# --------------------------------------------------------------------------
# AptControllerConfig(portName, devicePath, motorType, numAxes, movingPoll, idlePoll)
#
#   portName   = "APT-PZ"     — shared EPICS asyn port for all 4 channels
#   devicePath = host:port     — single ser2net TCP endpoint
#   motorType  = "kim"         — PZMOT_* APT messages for KIM101/TIM101
#                                 piezo inertial motor controllers
#   numAxes    = 4             — creates 4 axes (ADDR 0,1,2,3)
#                                with APT channels 1,2,3,4
#   numAxes    = 0             — auto-detect from HW_GET_INFO
#   movingPoll = 0.2 s
#   idlePoll   = 1.0 s
# --------------------------------------------------------------------------

AptControllerConfig("APT-PZ", "pldanteco101.lnf.infn.it:4001", "kim", 4, 0.2, 1.0)

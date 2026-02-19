# ============================================================================
# APT DC Servo Motor Controller via ser2net (remote)
# ============================================================================
# Load motor database
dbLoadTemplate("AptDC.substitutions")

# AptControllerConfig(portName, devicePath, motorType, numAxes, movingPoll, idlePoll)
#
#   devicePath  - "host:port" address of the ser2net daemon.
#                 The ser2net instance must be configured with:
#                   115200 baud, 8N1, RTS/CTS, raw mode
#
#                 Example ser2net v4 config (on the remote host):
#                   connection: &thorlabs-dc
#                     accepter: tcp,4001
#                     connector: serialdev,/dev/ttyUSB0,115200n81,local,rtscts
#                     options:
#                       kickolduser: true
#
AptControllerConfig("APT-DC1", "pldanteco101.lnf.infn.it:4001", "dc", 1, 0.2, 1.0)

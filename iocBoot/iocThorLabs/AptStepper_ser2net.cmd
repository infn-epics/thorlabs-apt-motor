# ============================================================================
# APT Stepper Motor Controller via ser2net (remote)
# ============================================================================
# Load motor database
dbLoadTemplate("AptStepper.substitutions")

# AptControllerConfig(portName, devicePath, motorType, numAxes, movingPoll, idlePoll)
#
#   devicePath  - "host:port" address of the ser2net daemon.
#                 The ser2net instance must be configured with:
#                   115200 baud, 8N1, RTS/CTS, raw mode
#
#                 Example ser2net v4 config (on the remote host):
#                   connection: &thorlabs-stepper
#                     accepter: tcp,4002
#                     connector: serialdev,/dev/ttyUSB1,115200n81,local,rtscts
#                     options:
#                       kickolduser: true
#
AptControllerConfig("APT-ST1", "192.168.1.100:4002", "stepper", 1, 0.2, 1.0)

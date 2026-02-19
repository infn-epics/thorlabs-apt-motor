#!../../bin/linux-x86_64/thorlabs

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/thorlabs.dbd"
thorlabs_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=Motor:")

## ============================================================================
## Load APT motor configurations
## Uncomment the controller(s) you need:
## ============================================================================

## DC servo motor (KDC101, TDC001, etc.)
##   Local USB:  AptDC.cmd          (device at /dev/ttyUSBx)
##   ser2net:    AptDC_ser2net.cmd  (device at host:port)
#< AptDC.cmd
< AptDC_ser2net.cmd

## Stepper motor (KST101, TST001, BSC10x, etc.)
##   Local USB:  AptStepper.cmd          (device at /dev/ttyUSBx)
##   ser2net:    AptStepper_ser2net.cmd  (device at host:port)
#< AptStepper.cmd
#< AptStepper_ser2net.cmd

## ============================================================================

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("Motor:")

# Boot complete
# var drvAptDebug 2
# dbpf Motor:DC1.VAL 4
# dbpf Motor:DC1.VAL 2
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Build the Thorlabs APT support library
LIBRARY_IOC += thorlabsSupport

# DBD file for the support library
DBD += devThorLabsApt.dbd

#=============================
# Source files for the support library

thorlabsSupport_SRCS += aptSerial.cpp
thorlabsSupport_SRCS += drvApt.cpp
thorlabsSupport_SRCS += drvAptDC.cpp
thorlabsSupport_SRCS += drvAptStepper.cpp

#=============================
# Libraries needed by the support library
thorlabsSupport_LIBS += motor
thorlabsSupport_LIBS += asyn
thorlabsSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

# Install header files for downstream IOCs
INC += drvApt.h
INC += aptProtocol.h
INC += aptSerial.h

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


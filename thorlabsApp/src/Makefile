TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
# USR_CFLAGS   += -g -Og
# USR_CXXFLAGS += -g -Og

#=============================
# Build the IOC application

PROD_IOC = thorlabs
# thorlabs.dbd will be created and installed
DBD += thorlabs.dbd

# thorlabs.dbd will be made up from these files:
thorlabs_DBD += base.dbd

# Include dbd files from all support applications:
thorlabs_DBD += asyn.dbd
thorlabs_DBD += drvAsynSerialPort.dbd
thorlabs_DBD += motorSupport.dbd

# APT support library DBD (from thorlabsSup)
thorlabs_DBD += devThorLabsApt.dbd

#=============================
# Link the Thorlabs APT support library (built by thorlabsSup)
thorlabs_LIBS += thorlabsSupport

# Other support libraries
thorlabs_LIBS += motor
thorlabs_LIBS += asyn

ifdef SNCSEQ
thorlabs_LIBS += seq pv
endif

# thorlabs_registerRecordDeviceDriver.cpp derives from thorlabs.dbd
thorlabs_SRCS += thorlabs_registerRecordDeviceDriver.cpp

# Build the main IOC entry point
thorlabs_SRCS_DEFAULT += thorlabsMain.cpp
thorlabs_SRCS_vxWorks += -nil-

# Finally link to the EPICS Base libraries
thorlabs_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

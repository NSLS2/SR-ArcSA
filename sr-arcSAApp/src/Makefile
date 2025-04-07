TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = sr-arcSA
# sr-arcSA.dbd will be created and installed
DBD += sr-arcSA.dbd

# sr-arcSA.dbd will be made up from these files:
sr-arcSA_DBD += base.dbd

# Include dbd files from all support applications:
sr-arcSA_DBD += menuScan.dbd
sr-arcSA_DBD += system.dbd
sr-arcSA_DBD += asyn.dbd
sr-arcSA_DBD += calcSupport.dbd
sr-arcSA_DBD += okSupport.dbd
sr-arcSA_DBD += asSupport.dbd
sr-arcSA_DBD += devIocStats.dbd
sr-arcSA_DBD += caPutLog.dbd
sr-arcSA_DBD += reccaster.dbd

USR_CPPFLAGS += -I$(OK)  -D_X86_64_ -DUNIX -DLINUX

# Add all the support libraries needed by this IOC
sr-arcSA_LIBS += arcSAApp asyn autosave devIocStats calc sscan caPutLog seq pv reccaster pvDatabase qsrv pvaClient pvAccessIOC pvAccess pvAccessCA nt pvData

sr-arcSA_SYS_LIBS += okFrontPanel

USR_LDFLAGS += -L$(OK)

# sr-arcSA_registerRecordDeviceDriver.cpp derives from sr-arcSA.dbd
sr-arcSA_SRCS += sr-arcSA_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
sr-arcSA_SRCS_DEFAULT += sr-arcSAMain.cpp
sr-arcSA_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#sr-arcSA_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

LIBRARY_IOC += arcSAApp

arcSAApp_SRCS += okiface.cpp
arcSAApp_SRCS += arcSPort.cpp

# Link QSRV (pvAccess Server) if available
ifdef EPICS_QSRV_MAJOR_VERSION
    sr-arcSA_LIBS += qsrv
    sr-arcSA_LIBS += $(EPICS_BASE_PVA_CORE_LIBS)
    sr-arcSA_DBD += PVAServerRegister.dbd
    sr-arcSA_DBD += qsrv.dbd
endif

# Finally link to the EPICS Base libraries
sr-arcSA_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


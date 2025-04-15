#!../../bin/linux-x86_64/sr-arcSA

#- You may have to change sr-arcSA to something else
#- everywhere it appears in this file

< envPaths

cd "${TOP}"

epicsEnvSet("LOCATION","RF BLDG/CAGE")
epicsEnvSet("EPICS_CA_AUTO_ADDR_LIST","NO")
epicsEnvSet("EPICS_CA_ADDR_LIST","10.0.153.255")
epicsEnvSet("EPICS_CAS_AUTO_BEACON_ADDR_LIST","NO")
epicsEnvSet("EPICS_CAS_BEACON_ADDR_LIST","10.0.153.255")
epicsEnvSet("EPICS_TZ", "EST5EDT,M3.2.0/2,M11.1.0/2")

## Register all support components
dbLoadDatabase "dbd/sr-arcSA.dbd"
sr_arcSA_registerRecordDeviceDriver(pdbbase) 

pwd

#system("export LD_LIBRARY_PATH=$(EPICS_BASE)/lib/linux-x86_64:$(TOP)/lib/linux-x86_64:$LD_LIBRARY_PATH;$(TOP)/bin/linux-x86_64/okLoad_bitfile 1650000GSP ARC5_XEM.bit")
system("export LD_LIBRARY_PATH=$(EPICS_BASE)/lib/linux-x86_64:$(TOP)/lib/linux-x86_64:$LD_LIBRARY_PATH;$(TOP)/bin/linux-x86_64/okLoad_bitfile 1749000KK5 ARC5_XEM.bit")
okls()

createOKPort("arcport","1749000KK5", 0)

## Load record instances
dbLoadRecords("db/arcS.db", "PORT=arcport, S1=SR-RF, D1=ArcS:A")
dbLoadRecords("db/okWireIn.db", "PORT=arcport, S1=SR-RF, D1=ArcS:A")
dbLoadRecords("db/okWireOut.db", "PORT=arcport, S1=SR-RF, D1=ArcS:A")
dbLoadRecords("db/iocAdminSoft.db", "IOC=RF-CT{$(IOC)}")
dbLoadRecords("db/asynRecord.db", "P=SR-RF{ArcS:A},R=port, PORT=arcport, ADDR=0, OMAX=40, IMAX=40")
dbLoadRecords("db/save_restoreStatus.db", "P=RF-CT{$(IOC)}")
dbLoadRecords("db/reccaster.db", "P=SR-RF{ArcS:A-RC}")

# Auto save/restore
save_restoreSet_status_prefix("RF-CT{$(IOC)}")

# ensure directories exist
system("install -d ${TOP}/as")
system("install -d ${TOP}/as/req")
system("install -d ${TOP}/as/save")

set_savefile_path("${TOP}/as","/save")
set_requestfile_path("${TOP}/as","/req")

set_pass0_restoreFile("ioc_settings.sav")
set_pass0_restoreFile("ioc_values.sav")
set_pass1_restoreFile("ioc_values.sav")
set_pass1_restoreFile("ioc_waveforms.sav")

##asynSetTraceMask("arcport", -1, 0xff)
##asynSetTraceIOMask("arcport", -1, 0xff)

asSetFilename("/cf-update/acf/default.acf")
#asSetFilename("${TOP}/DEFAULT.acf")

cd "${TOP}/iocBoot/${IOC}"
callbackSetQueueSize(20000)
iocInit()

dbl > records.dbl

system "cp records.dbl /cf-update/$HOSTNAME.$IOCNAME.dbl"

makeAutosaveFileFromDbInfo("${TOP}/as/req/ioc_settings.req", "autosaveFields_pass0")
makeAutosaveFileFromDbInfo("${TOP}/as/req/ioc_values.req", "autosaveFields")
makeAutosaveFileFromDbInfo("${TOP}/as/req/ioc_waveforms.req", "autosaveFields_pass1")

caPutLogInit("10.0.152.133:7004",1)

create_monitor_set("ioc_settings.req", 5 , "")
create_monitor_set("ioc_values.req", 5 , "")
create_monitor_set("ioc_waveforms.req", 5 , "")

date


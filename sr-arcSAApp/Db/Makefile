TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
DB += arcS.db
DB += okWireIn.db
DB += okWireOut.db

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

DB_INSTALLS += ${ASYN}/db/asynRecord.db
DB_INSTALLS += ${AUTOSAVE}/asApp/Db/save_restoreStatus.db
DB_INSTALLS += ${DEVIOCSTATS}/db/iocAdminSoft.db
DB_INSTALLS += ${RECSYNC}/db/reccaster.db

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


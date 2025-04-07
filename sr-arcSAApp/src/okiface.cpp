#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>

#include <memory>
#include <map>

#include <cstring>

#include <alarm.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <devSup.h>
#include <link.h>
#include <callback.h>

#include <boRecord.h>
#include <biRecord.h>
#include <longoutRecord.h>
#include <longinRecord.h>
#include <stringinRecord.h>
#include <waveformRecord.h>

#include "linkoptions.h"

#include <okFrontPanelDLL.h>

#define TRYREC(PREC) \
{ if(!(PREC)->dpvt) return oops(PREC); \
okUser *priv=(okUser*)(PREC)->dpvt; try

#define CATCHREC(PREC) \
catch(std::exception& e) { \
    std::cout<<(PREC)->name<<": "<<e.what()<<"\n"; \
    return S_db_errArg; \
} }

static
const char * ModelNames_[] =
{
    "Unknown",
    "XEM3001v1",
    "XEM3001v2",
    "XEM3010",
    "XEM3005",
    "XEM3001CL",
    "XEM3020",
    "XEM3050",
    "XEM9002",
};

static
const char* ModelName(ok_BoardModel m)
{
  if      (m == 41 ) return "XEM7010 A50";
  else if (m == 42 ) return "XEM7010 A200";
  else if (m < 0 ||
          (size_t)m >= NELEMENTS(ModelNames_)) return "Unknown model";
  else                                         return ModelNames_[m];
}

extern "C"
void okls() {
    char serial[64];
    int i, cnt;
    ok_BoardModel m;

    okFrontPanel_HANDLE H = okFrontPanel_Construct();
    if(!H) {
        std::cerr<<"OK not OK\n";
        return;
    }

    cnt = okFrontPanel_GetDeviceCount(H);
    std::cout<<"Found " << cnt << " OK devices\n";

    for(i = 0; i < cnt;i++) {
        m = okFrontPanel_GetDeviceListModel(H, i);
        okFrontPanel_GetDeviceListSerial(H, i, serial);
        printf("Model: %s S/N:%s\n", ModelName(m), serial);
    }
    okFrontPanel_Destruct(H);
}


#include <epicsExport.h>
#include <iocsh.h>

/* okls */
static const iocshArg * const oklsArgs[] = {};
static const iocshFuncDef oklsFuncDef = {"okls",0,oklsArgs};
static void oklsCallFunc(const iocshArgBuf *args)
{
    okls();
}

static
void okifaceReg()
{
    iocshRegister(&oklsFuncDef,oklsCallFunc);
}

epicsExportRegistrar(okifaceReg);

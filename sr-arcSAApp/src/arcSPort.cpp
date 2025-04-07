#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>

#include <byteswap.h>
#include <epicsMath.h>
#include <string.h>
#include <time.h>
#include <dbDefs.h>
#include <epicsExit.h>
#include <epicsTime.h>
#include <epicsEndian.h>

using namespace std;

#include "arcSPort.h"

static const char * ModelNames_[] = {
    "Unknown",
    "XEM3001v1",
    "XEM3001v2",
    "XEM3010",
    "XEM3005",
    "XEM3001CL",
    "XEM3020",
    "XEM3050",
};

static const char* ModelName(ok_BoardModel m) {
  if      (m == 41 ) return "XEM7010 A50";
  else if (m == 42 ) return "XEM7010 A200";
  else if (m < 0 ||
          (size_t)m >= NELEMENTS(ModelNames_)) return "Unknown model";
  else                                         return ModelNames_[m];
}

char *okStrErr(int e) {
  int i = abs(e);
  char *okStrErr[21] = { (char *)"ok_NoError",
                       (char *)"ok_Failed",
                       (char *)"ok_Timeout",
                       (char *)"ok_DoneNotHigh",
                       (char *)"ok_TransferError",
                       (char *)"ok_CommunicationError",
                       (char *)"ok_InvalidBitstream",
                       (char *)"ok_FileError",
                       (char *)"ok_DeviceNotOpen",
                       (char *)"ok_InvalidEndpoint",
                       (char *)"ok_InvalidBlockSize",
                       (char *)"ok_I2CRestrictedAddress",
                       (char *)"ok_I2CBitError",
                       (char *)"ok_I2CNack",
                       (char *)"ok_I2CUnknownStatus",
                       (char *)"ok_UnsupportedFeature",
                       (char *)"ok_FIFOUnderflow",
                       (char *)"ok_FIFOOverflow",
                       (char *)"ok_DataAlignmentError",
                       (char *)"ok_InvalidResetProfile",
                       (char *)"ok_InvalidParameter" };

  if( i >= 0 && i < 21 ) return okStrErr[i];
  else return (char *)"Unknown error";
}

static const char* ErrorName(ok_ErrorCode m) {
  return (const char *)okStrErr(m);
}

#define HWERR(ERR) hwError(ERR, __FILE__, __LINE__)

#define HWERR2(USR, ERR) hwError(ERR, __FILE__, __LINE__, USR)

arcSPort::arcSPort(const char *portName, const char *serial, int debug)
      : asynPortDriver(portName,
      40,
      asynOctetMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynDrvUserMask,
      0, /*Interrupt mask*/
      ASYN_CANBLOCK, 
      1, // Autoconnect
      0, // Default priority
      0) // Default stack size
    , H(NULL)
    , nextdac(false)
    , zombie(false)
{
try {
    if(debug) {
        // early debugging
        pasynTrace->setTraceMask(pasynUserSelf, 0x09);
        pasynTrace->setTraceIOMask(pasynUserSelf, 0x04);
    }

    strcpy(SerialNo, serial);

    std::cout << "Created OpalKelly port: Name=" << portName << " ";
    std::cout << "Serial:" << serial << " Debug=" << debug << endl;

    AERR(createParam("Model",         asynParamOctet, &P_model));
    AERR(createParam("ID",            asynParamOctet, &P_id));
    AERR(createParam("Major",         asynParamInt32, &P_major));
    AERR(createParam("Minor",         asynParamInt32, &P_minor));
    AERR(createParam("Serial",        asynParamOctet, &P_serial));
    AERR(createParam("Load",          asynParamInt32, &P_load));
    AERR(createParam("Reset",         asynParamInt32, &P_reset));
    AERR(createParam("Update",        asynParamInt32, &P_update));

    AERR(createParam("Bit Directory", asynParamOctet, &P_bit_dir));
    AERR(createParam("Bit File",      asynParamOctet, &P_bit_file));

//----------------------------------------------------------------------------
//	March 20, 2012

    AERR(createParam("WireOut EP20",  asynParamInt32, &P_WO_EP20));
    AERR(createParam("WireOut EP21",  asynParamInt32, &P_WO_EP21));
    AERR(createParam("WireOut EP22",  asynParamInt32, &P_WO_EP22));
    AERR(createParam("WireOut EP23",  asynParamInt32, &P_WO_EP23));
    AERR(createParam("WireOut EP24",  asynParamInt32, &P_WO_EP24));
    AERR(createParam("WireOut EP25",  asynParamInt32, &P_WO_EP25));
    AERR(createParam("WireOut EP26",  asynParamInt32, &P_WO_EP26));
    AERR(createParam("WireOut EP27",  asynParamInt32, &P_WO_EP27));
    AERR(createParam("WireOut EP28",  asynParamInt32, &P_WO_EP28));
    AERR(createParam("WireOut EP29",  asynParamInt32, &P_WO_EP29));
    AERR(createParam("WireOut EP2a",  asynParamInt32, &P_WO_EP2a));
    AERR(createParam("WireOut EP2b",  asynParamInt32, &P_WO_EP2b));
    AERR(createParam("WireOut EP2c",  asynParamInt32, &P_WO_EP2c));
    AERR(createParam("WireOut EP2d",  asynParamInt32, &P_WO_EP2d));

//----------------------------------------------------------------------------

    AERR(createParam("WireIn EP00", asynParamInt32, &P_WI_EP00));
    AERR(createParam("WireIn EP01", asynParamInt32, &P_WI_EP01));
    AERR(createParam("WireIn EP02", asynParamInt32, &P_WI_EP02));
    AERR(createParam("WireIn EP03", asynParamInt32, &P_WI_EP03));
    AERR(createParam("WireIn EP04", asynParamInt32, &P_WI_EP04));
    AERR(createParam("WireIn EP05", asynParamInt32, &P_WI_EP05));
    AERR(createParam("WireIn EP06", asynParamInt32, &P_WI_EP06));
    AERR(createParam("WireIn EP07", asynParamInt32, &P_WI_EP07));
/********************************************************/
/********************************************************/
/* Feb. 27, 2020					*/
        AERR(createParam("okSNo",    asynParamOctet, &P_ok_SN));
/*							*/
/********************************************************/
/*------------------------------------------------------*/
    AERR(setStringParam(P_serial, SerialNo));

    epicsAtExit(&shutdown, (void*)this);
  } catch(std::exception& e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: Failed to initialize port\nError: %s\n",
              portName, e.what());
    stop();
    }
}

arcSPort::~arcSPort()
{
    stop();
}

void arcSPort::shutdown(void *raw)
{
    arcSPort *port=(arcSPort*)raw;
    try{
        port->stop();
    }catch(std::exception& e) {
        std::cerr<<port->portName<<": Shutdown error: "<<e.what()<<"\n";
    }
}

void arcSPort::stop()
{
    lock();
    if(zombie) {
        unlock();
        return;
    }
    zombie=true;

    okFrontPanel_Destruct(H);

    unlock();
}

asynStatus arcSPort::connect(asynUser *pasynUser)
{
  try {
    FLOW(pasynUser);
    int addr;

    AERR(getAddress(pasynUser, &addr));

    if(zombie) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: Can't connect to zombie...\n", portName);
        return asynError;
    }
    if(connected[addr]) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: Already open\n", portName);
        return asynError;
    }

    // if device already connected then done
    if(connected[0]) {
        connected[addr] = true;
        return asynPortDriver::connect(pasynUser);
    }

    H = okFrontPanel_Construct();
    if(!H)
        throw std::bad_alloc();

    error_t err = okFrontPanel_OpenBySerial(H, SerialNo );
    if(err!=ok_NoError) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: connect error: %s\n",
                  portName, ErrorName(err));
        return asynError;
    }

    AERR(setStringParam(P_model, ModelName(okFrontPanel_GetBoardModel(H))));
    char tmp[40]; // Hope this is enough...
    okFrontPanel_GetDeviceID(H, tmp);
    tmp[39]='\0';
    setStringParam(P_id, tmp);

    AERR(setIntegerParam(P_major, okFrontPanel_GetDeviceMajorVersion(H)));
    AERR(setIntegerParam(P_minor, okFrontPanel_GetDeviceMinorVersion(H)));

    init(pasynUser);

    connected[0] = true;
    connected[addr] = true;
    if(addr!=0)
        asynPortDriver::connect(pasynUserSelf); // self is connected to addr 0

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: connected\n", portName);
    return asynPortDriver::connect(pasynUser);
  }CATCHALL(pasynUser)
}

asynStatus arcSPort::disconnect(asynUser *pasynUser) {
  try {
    int addr;
    FLOW(pasynUser);

    AERR(getAddress(pasynUser, &addr));

    if(!connected[addr]) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: Already closed\n", portName);
        return asynError;
    }
    connected[addr] = false;

    if(addr==0) {
        // zero disconnects everyone
        for(int i=1; i<NUM_CHANNELS; i++) {
            AsynUser usr(portName, i);
            connected[i]=false;
            asynPortDriver::disconnect(usr);
        }
    }

    okFrontPanel_Destruct(H);
    H=NULL;

    return asynPortDriver::disconnect(pasynUser);
  }CATCHALL(pasynUser)
}

void arcSPort::hwError(error_t err, const char* file, int line, asynUser *usr) {
    if(!usr) usr = pasynUserSelf;
    if(err == ok_NoError) return;
    switch(err) {
    case ok_FileError:
        // occurs when loading a new bit file
        break;
    default:
        disconnect(usr);
    }

    std::ostringstream msg;
    msg << portName << ": " << ErrorName(err) << ": on line " << line;
    throw std::runtime_error(msg.str());
}

void arcSPort::init(asynUser *pasynUser) {
    // DAC length
    HWERR2(pasynUser,
           okFrontPanel_SetWireInValue(H, EP_LENGTH,
                                          2048,
                                          0xffff));

    // ??? matlab script does this
    HWERR2(pasynUser,
           okFrontPanel_SetWireInValue(H, EP_CONTROL,
                                          4<<CONTROL_DEBUG_shft,
                                          CONTROL_DEBUG_mask));

    // Setup 1-wire
    HWERR2(pasynUser,
           okFrontPanel_SetWireInValue(H, EP_1WIRE_CMD,
                                          51,
                                          0xffff));

    okFrontPanel_UpdateWireIns(H);

    HWERR2(pasynUser, okFrontPanel_ActivateTriggerIn(H, EP_ACTIONS, ACTIONS_UPDATE_DS1825) );

    okFrontPanel_UpdateWireOuts(H);

    epicsInt16 wire1[4];
    for(int i = 0; i < 4; i++) {
        wire1[i] = okFrontPanel_GetWireOutValue(H, EP_1WIRE_RES_0 + i);
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s: 1wire %02x %02x %02x %02x\n",
              portName, wire1[0], wire1[1], wire1[2], wire1[3]);

    nextdac=false;
    HWERR2(pasynUser,
           okFrontPanel_SetWireInValue(H,EP_CONTROL,
                                          CTL_DAC_RD,
                                          CTL_DAC_RD));

    // move write pointer to A
    HWERR2(pasynUser,
           okFrontPanel_SetWireInValue(H,EP_CONTROL,
                                          0 << CTL_DAC_WR_shft,
                                          CTL_DAC_WR_mask));

    okFrontPanel_UpdateWireIns(H);
}

asynStatus arcSPort::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual) {
  try {
    FLOW(pasynUser);
    int addr;
    int function = pasynUser->reason;

    AERR(getAddress(pasynUser, &addr));

    if( function == P_bit_dir )
	{
	strncpy( bitdirname, value, sizeof(bitdirname) );
	//cout << "writeOctet: bitdir:" << value << endl;
	}
    else
    if( function == P_bit_file )
	{
	strncpy( bitfilename, value, sizeof(bitfilename) );
        //cout << "writeOctet: bitfile:" << value << endl;
	}

    AERR(setStringParam(addr, function, value));

    *nActual = strlen(value);

//  AERR(callParamCallbacks());

    return asynSuccess;

   } CATCHALL(pasynUser)
}

asynStatus arcSPort::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason) {
  try {
    FLOW(pasynUser);
    int addr;
    int function = pasynUser->reason;
    ok_ErrorCode  okErr;
    okTDeviceInfo okDevInfo;

    AERR(getAddress(pasynUser, &addr));

    if( function == P_bit_dir )
	{
	// strncpy( bitdirname, value, sizeof(bitdirname) );
	// cout << "writeOctet: bitdir:" << value << endl;
	}
    else
    if( function == P_bit_file )
	{
	//strncpy( bitfilename, value, sizeof(bitfilename) );
        // cout << "writeOctet: bitfile:" << value << endl;
	}
    else
    if( function == P_ok_SN )
	{
        okErr = okFrontPanel_GetDeviceInfo(H, &okDevInfo);
        if( !okErr ) strncpy( value, okDevInfo.serialNumber, 40);
        else         strcpy( value, "0000000000");
	}
    else
    if( function == P_model )
	{
        okErr = okFrontPanel_GetDeviceInfo(H, &okDevInfo);
        if( !okErr ) strncpy( value, okDevInfo.productName, 40);
	}
    else
    if( function == P_id )
	{
        okErr = okFrontPanel_GetDeviceInfo(H, &okDevInfo);
        if( !okErr ) strncpy( value, okDevInfo.deviceID, 40);
	}

    AERR(setStringParam(addr, function, value));

    *nActual = strlen(value);

    //AERR(callParamCallbacks());

    return asynSuccess;

   } CATCHALL(pasynUser)
}

asynStatus arcSPort::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int           val, msk;
  ok_ErrorCode  okErr;

  try {
    FLOW(pasynUser);
    int addr, function = pasynUser->reason;

    AERR(getAddress(pasynUser, &addr));

    if(function==P_load) {
        char bitfile[80];

        AERR(getStringParam(P_bit_dir, NELEMENTS(bitfile), bitfile));
        strcat(bitfile,"/");
        size_t blen=strlen(bitfile);
        AERR(getStringParam(P_bit_file, NELEMENTS(bitfile)-blen, bitfile+blen));

        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: Load bitfile: '%s'\n",
                  portName, bitfile);

        lock();

	cout << "P_load: 1. Loading bitfile:" << bitfile << endl;

        // HWERR2(pasynUser, okFrontPanel_LoadDefaultPLLConfiguration(H) );
        // HWERR2(pasynUser, okFrontPanel_ConfigureFPGA(H, bitfile) );

        okErr = okFrontPanel_ConfigureFPGA(H, bitfile);

	cout << "P_load: 2. Loaded bitfile:" << bitfile << " ";
        cout << "okErr = " << okErr << " ";
        if( !okErr ) cout << "SUCCESS!" << endl;
        else         cout << "Failed."  << endl;

        ts[0].tv_sec = 		0;
        ts[0].tv_nsec=	500000000; // Undocumented delay: pause(0.5)
        ts[1].tv_sec = 		0;
        ts[1].tv_nsec=		0;
        nanosleep( &ts[0], &ts[1]);

        cout << "P_load: 3. Loaded bitfile:" << bitfile << endl;

        HWERR2(pasynUser, okFrontPanel_SetWireInValue(H, 0x10, 0x43, 0xffff) );	// Maxim DS1825
        okFrontPanel_UpdateWireIns(H);

        cout << "P_load: 4. Loaded bitfile:" << bitfile << endl;

        HWERR2(pasynUser, okFrontPanel_ActivateTriggerIn(H, EP_ACTIONS, ACTIONS_RESET) );

        okFrontPanel_UpdateTriggerOuts(H);
        ts[0].tv_sec = 		0;
        ts[0].tv_nsec=	500000000; // Undocumented delay: pause(0.5)
        ts[1].tv_sec = 		0;
        ts[1].tv_nsec=		0;
        nanosleep( &ts[0], &ts[1]);
        unlock();

        cout << "P_load: 5. Loaded bitfile:" << bitfile << endl;

        init(pasynUser);

    } else if(function==P_reset) {

        lock();
        HWERR2(pasynUser, okFrontPanel_ActivateTriggerIn(H, EP_ACTIONS, ACTIONS_RESET) );
        okFrontPanel_UpdateTriggerOuts(H);
        ts[0].tv_sec = 		0;
        ts[0].tv_nsec=	500000000; // Undocumented delay: pause(0.5)
        ts[1].tv_sec = 		0;
        ts[1].tv_nsec=		0;
        nanosleep( &ts[0], &ts[1]);

        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: Global RESET:\n",
                  portName);

        // cout << "P_reset:" << endl;
        unlock();

    } else if ( function == P_update) {
        okFrontPanel_UpdateWireIns(H);
        okFrontPanel_UpdateWireOuts(H);
        update(pasynUser);

    } else if ( function == P_WI_EP00) {
        val = value;
        msk = 0xffff;
        okErr = okFrontPanel_SetWireInValue(H, 0x00, val, msk);
        okFrontPanel_UpdateWireIns(H);
        if( okErr ) cerr << "SetWireIn 1 0x00: " << okStrErr(okErr) << endl;
        //cout << "okFrontPanel_SetWireInValue(H, 0x00, 0x" << hex << setfill('0') << setw(8) << val << " ";
        //cout << hex << setfill('0') << setw(8) << msk << endl;
    } else if ( function == P_WI_EP01) {
        val = value;
        msk = 0xffff;
        okErr = okFrontPanel_SetWireInValue(H, 0x01, val, msk);
        okFrontPanel_UpdateWireIns(H);
        if( okErr ) cerr << "SetWireIn 1 0x01: " << okStrErr(okErr) << endl;
        //cout << "okFrontPanel_SetWireInValue(H, 0x01, 0x" << hex << setfill('0') << setw(8) << val << " ";
        //cout << hex << setfill('0') << setw(8) << msk << endl;
    } else if ( function == P_WI_EP02) {
        val = value;
        msk = 0xffff;
        okErr = okFrontPanel_SetWireInValue(H, 0x02, val, msk);
        okFrontPanel_UpdateWireIns(H);
        if( okErr ) cerr << "SetWireIn 1 0x02: " << okStrErr(okErr) << endl;
        //cout << "okFrontPanel_SetWireInValue(H, 0x02, 0x" << hex << setfill('0') << setw(8) << val << " ";
        //cout << hex << setfill('0') << setw(8) << msk << endl;
    } else if ( function == P_WI_EP03) {
        val = value;
        msk = 0xffff;
        okErr = okFrontPanel_SetWireInValue(H, 0x03, val, msk);
        okFrontPanel_UpdateWireIns(H);
        if( okErr ) cerr << "SetWireIn 1 0x03: " << okStrErr(okErr) << endl;
        //cout << "okFrontPanel_SetWireInValue(H, 0x03, 0x" << hex << setfill('0') << setw(8) << val << " ";
        //cout << hex << setfill('0') << setw(8) << msk << endl;
    } else if ( function == P_WI_EP04) {
        val = value;
        msk = 0xffff;
        okErr = okFrontPanel_SetWireInValue(H, 0x04, val, msk);
        okFrontPanel_UpdateWireIns(H);
        if( okErr ) cerr << "SetWireIn 1 0x04: " << okStrErr(okErr) << endl;
        //cout << "okFrontPanel_SetWireInValue(H, 0x04, 0x" << hex << setfill('0') << setw(8) << val << " ";
        //cout << hex << setfill('0') << setw(8) << msk << endl;
    } else if ( function == P_WI_EP05) {
        val = value;
        msk = 0xffff;
        okErr = okFrontPanel_SetWireInValue(H, 0x05, val, msk);
        okFrontPanel_UpdateWireIns(H);
        if( okErr ) cerr << "SetWireIn 1 0x05: " << okStrErr(okErr) << endl;
        //cout << "okFrontPanel_SetWireInValue(H, 0x05, 0x" << hex << setfill('0') << setw(8) << val << " ";
        //cout << hex << setfill('0') << setw(8) << msk << endl;
    } else if ( function == P_WI_EP06) {
        val = value;
        msk = 0xffff;
        okErr = okFrontPanel_SetWireInValue(H, 0x06, val, msk);
        okFrontPanel_UpdateWireIns(H);
        if( okErr ) cerr << "SetWireIn 1 0x06: " << okStrErr(okErr) << endl;
        //cout << "okFrontPanel_SetWireInValue(H, 0x06, 0x" << hex << setfill('0') << setw(8) << val << " ";
        //cout << hex << setfill('0') << setw(8) << msk << endl;
    } else if ( function == P_WI_EP07) {
        val = value;
        msk = 0xffff;
        okErr = okFrontPanel_SetWireInValue(H, 0x07, val, msk);
        okFrontPanel_UpdateWireIns(H);
        if( okErr ) cerr << "SetWireIn 1 0x07: " << okStrErr(okErr) << endl;
        //cout << "okFrontPanel_SetWireInValue(H, 0x06, 0x" << hex << setfill('0') << setw(8) << val << " ";
        //cout << hex << setfill('0') << setw(8) << msk << endl;
    } else {
      printf( "writeInt32::function=%d - no match\n", function);
      }

    // AERR(setIntegerParam(addr, function, value));
    // AERR(callParamCallbacks());

    return asynSuccess;
  } CATCHALL(pasynUser)
}

//===================================================================================
asynStatus arcSPort::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    ok_ErrorCode  okErr;
    okTDeviceInfo okDevInfo;
    int           oval;

/*************************************************************************
  cout << "P_model   =" << P_model    << endl;
  cout << "P_id      =" << P_id       << endl;
  cout << "P_major   =" << P_major    << endl;
  cout << "P_minor   =" << P_minor    << endl;
  cout << "P_serial  =" << P_serial   << endl;
  cout << "P_load    =" << P_load     << endl;
  cout << "P_reset   =" << P_reset    << endl;
  cout << "P_update  =" << P_update   << endl;
  cout << "P_bit_dir =" << P_bit_dir  << endl;
  cout << "P_bit_file=" << P_bit_file << endl;
  cout << "P_WO_EP20 =" << P_WO_EP20  << endl;
  cout << "P_WO_EP21 =" << P_WO_EP21  << endl;
  cout << "P_WO_EP22 =" << P_WO_EP22  << endl;
  cout << "P_WO_EP23 =" << P_WO_EP23  << endl;
  cout << "P_WO_EP24 =" << P_WO_EP24  << endl;
  cout << "P_WO_EP25 =" << P_WO_EP25  << endl;
  cout << "P_WO_EP26 =" << P_WO_EP26  << endl; 
  cout << "P_WO_EP27 =" << P_WO_EP27  << endl;
  cout << "P_WO_EP28 =" << P_WO_EP28  << endl;
  cout << "P_WO_EP29 =" << P_WO_EP29  << endl;
  cout << "P_WO_EP2a =" << P_WO_EP2a  << endl;
  cout << "P_WO_EP2b =" << P_WO_EP2b  << endl;
  cout << "P_WO_EP2c =" << P_WO_EP2c  << endl;
  cout << "P_WO_EP2d =" << P_WO_EP2d  << endl;
  cout << "P_ok_SN   =" << P_ok_SN    << endl;
  cout << "P_WI_EP00 =" << P_WI_EP00  << endl;
  cout << "P_WI_EP01 =" << P_WI_EP01  << endl;
  cout << "P_WI_EP02 =" << P_WI_EP02  << endl;
  cout << "P_WI_EP03 =" << P_WI_EP03  << endl;
  cout << "P_WI_EP04 =" << P_WI_EP04  << endl;
  cout << "P_WI_EP05 =" << P_WI_EP05  << endl;
  cout << "P_WI_EP06 =" << P_WI_EP06  << endl;
*************************************************************************/

  try {
    FLOW(pasynUser);
    int addr, function = pasynUser->reason;

    AERR(getAddress(pasynUser, &addr));


    okFrontPanel_UpdateWireOuts(H);

/*************************************************************************
    cout << "******FUNCTION=" << function << endl;
    if ( function == P_WO_EP20 ) cout << "=========P_WO_EP20" << endl;
    else if ( function == P_WO_EP26 ) cout << ":::::::::P_WO_EP26" << endl;
    else if ( function == P_WO_EP27 ) cout << ">>>>>>>>>P_WO_EP27" << endl;
    else                              cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
*************************************************************************/

    if(function==P_update) {
      update(pasynUser);
      }
    else if(function==P_major) {
      okErr = okFrontPanel_GetDeviceInfo(H, &okDevInfo);
      if( !okErr ) { *value = okDevInfo.deviceMajorVersion;
        // std::cout << "readInt32: P_major = " << dec << *value << endl;
        AERR( setIntegerParam( P_major, *value ) );
        }
      }
    else if(function==P_minor) {
      okErr = okFrontPanel_GetDeviceInfo(H, &okDevInfo);
      if( !okErr ) { *value = okDevInfo.deviceMinorVersion;
        // std::cout << "readInt32: P_minor = " << dec << *value << endl;
        AERR( setIntegerParam( P_minor, *value ) );
        }
      }
    else if(function==P_load) { }
    else if(function==P_reset){ }
    else if(function==P_WO_EP20) {
      oval = okFrontPanel_GetWireOutValue(H, 0x20 );
      *value=oval;
      //cout << "*P_WO_EP20: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP20, (int)(short)oval ) );
      }
    else if(function==P_WO_EP21) {
      oval = okFrontPanel_GetWireOutValue(H, 0x21 );
      *value=oval;
      //cout << "*P_WO_EP21: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP21, (int)(short)oval ) );
      }
    else if(function==P_WO_EP22) {
      oval = okFrontPanel_GetWireOutValue(H, 0x22 );
      *value=oval;
      //cout << "*P_WO_EP22: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP22, (int)(short)oval ) );
      }
    else if(function==P_WO_EP23) {
      oval = okFrontPanel_GetWireOutValue(H, 0x23 );
      *value=oval;
      //cout << "*P_WO_EP23: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP23, (int)(short)oval ) );
      }
    else if(function==P_WO_EP24) {
      oval = okFrontPanel_GetWireOutValue(H, 0x24 );
      *value=oval;
      //cout << "*P_WO_EP24: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP24, (int)(short)oval ) );
      }
    else if(function==P_WO_EP25) {
      oval = okFrontPanel_GetWireOutValue(H, 0x25 );
      *value=oval;
      //cout << "*P_WO_EP25: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP25, (int)(short)oval ) );
      }
    else if(function==P_WO_EP26) {
      oval = okFrontPanel_GetWireOutValue(H, 0x26 );
      *value=oval;
      //cout << "*P_WO_EP26: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP26, (int)(short)oval ) );
      }
    else
    if(function==P_WO_EP27) {
      oval = okFrontPanel_GetWireOutValue(H, 0x27 );
      *value=oval;
      //cout << "*P_WO_EP27: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP27, (int)(short)oval ) );
      }
    else if(function==P_WO_EP28) {
      oval = okFrontPanel_GetWireOutValue(H, 0x28 );
      *value=oval;
      //cout << "*P_WO_EP28: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP28, (int)(short)oval ) );
      }
    else
    if(function==P_WO_EP29) {
      oval = okFrontPanel_GetWireOutValue(H, 0x29 );
      *value=oval;
      //cout << "*P_WO_EP29: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP29, (int)(short)oval ) );
      }
    else
    if(function==P_WO_EP2a) {
      oval = okFrontPanel_GetWireOutValue(H, 0x2A );
      *value=oval;
      //cout << "*P_WO_EP2A: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP2a, (int)(short)oval ) );
      }
    else
    if(function==P_WO_EP2b) {
      oval = okFrontPanel_GetWireOutValue(H, 0x2B );
      *value=oval;
      //cout << "*P_WO_EP2B: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP2b, (int)(short)oval ) );
      }
    else
    if(function==P_WO_EP2c) {
      oval = okFrontPanel_GetWireOutValue(H, 0x2C );
      *value=oval;
      //cout << "*P_WO_EP2C: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP2c, (int)(short)oval ) );
      }
    else
    if(function==P_WO_EP2d) {
      oval = okFrontPanel_GetWireOutValue(H, 0x2D );
      *value=oval;
      //cout << "*P_WO_EP2D: 0x" << hex << setfill('0') << setw(8) << oval << endl;
      AERR( setIntegerParam( P_WO_EP2d, (int)(short)oval ) );
      }
    else
    if( function==P_WI_EP00 || function==P_WI_EP01 ||
        function==P_WI_EP02 || function==P_WI_EP03 ||
        function==P_WI_EP04 || function==P_WI_EP05 ||
        function==P_WI_EP06 || function==P_WI_EP07) { }
    else printf("readInt32:function=%d - not found\n", function);

    //AERR(callParamCallbacks());

    return asynSuccess;
  } CATCHALL(pasynUser)
}

//***********************************************************************************

asynStatus arcSPort::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
  try {
    FLOW(pasynUser);

    int    addr, function = pasynUser->reason;

    AERR(getAddress(pasynUser, &addr));
    
    AERR(setDoubleParam(function, value));
    AERR(callParamCallbacks());

    return asynSuccess;

 } CATCHALL(pasynUser)
}

asynStatus arcSPort::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements)
{
  try {
    FLOW(pasynUser);
    return asynSuccess;
  }CATCHALL(pasynUser)
}

asynStatus arcSPort::readInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements, size_t *nIn) {
  try {
    FLOW(pasynUser);
    return asynSuccess;
    }CATCHALL(pasynUser)
}

asynStatus arcSPort::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn) {
  try {
    FLOW(pasynUser);
    return asynSuccess;
    }CATCHALL(pasynUser)
}

void arcSPort::update(asynUser* pasynUser) {
register
  int i;
  int nWOVals = N_WOVALS;
  int oval[N_WOVALS];

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ok_ErrorCode  okErr;
  okTDeviceInfo okDevInfo;

  okErr = okFrontPanel_GetDeviceInfo(H, &okDevInfo);
  // printf("update: SN:%s\n", okDevInfo.serialNumber);
  if( !okErr ) AERR(setStringParam(P_ok_SN, okDevInfo.serialNumber));

  okFrontPanel_UpdateWireOuts(H);
  i = 0;
  do {
    oval[i] = okFrontPanel_GetWireOutValue(H, (0x20 + i) );
    i++;
    }
  while( i < nWOVals );

  //cout << "P_WO_EP20: 0x" << hex << setfill('0') << setw(8) << oval[0] << endl;
  //cout << "P_WO_EP27: 0x" << hex << setfill('0') << setw(8) << oval[7] << endl;

  AERR( setIntegerParam( P_WO_EP20, oval[ 0] ) );
  AERR( setIntegerParam( P_WO_EP21, oval[ 1] ) );
  AERR( setIntegerParam( P_WO_EP22, (int)(short)oval[ 2] ) );
  AERR( setIntegerParam( P_WO_EP23, (int)(short)oval[ 3] ) );
  AERR( setIntegerParam( P_WO_EP24, (int)(short)oval[ 4] ) );
  AERR( setIntegerParam( P_WO_EP25, (int)(short)oval[ 5] ) );
  AERR( setIntegerParam( P_WO_EP26, (int)(short)oval[ 6] ) );
  AERR( setIntegerParam( P_WO_EP27, (int)(short)oval[ 7] ) );
  AERR( setIntegerParam( P_WO_EP28, oval[ 8] ) );
  AERR( setIntegerParam( P_WO_EP29, (int)(short)oval[ 9] ) );
  AERR( setIntegerParam( P_WO_EP2a, oval[10] ) );
  AERR( setIntegerParam( P_WO_EP2b, oval[11] ) );
  AERR( setIntegerParam( P_WO_EP2c, oval[12] ) );
  AERR( setIntegerParam( P_WO_EP2d, oval[13] ) );

  //AERR(callParamCallbacks());
}

void arcSPort::report(FILE *fp, int details)
{
    asynPortDriver::report(fp, details);

    fprintf(fp, "Serial#: %s\n", SerialNo );
}

AsynUser::AsynUser(const char *portName, int addr)
{
    usr = pasynManager->createAsynUser(&AsynUser::procCB, NULL);
    if(!usr)
        throw std::bad_alloc();
try{
    AERR(pasynManager->connectDevice(usr, portName, addr));

    usr->userPvt = (void*)this;
}catch(...){
    pasynManager->freeAsynUser(usr);
    throw;
}
}

AsynUser::AsynUser(asynUser *other)
    :usr(other)
{}

AsynUser::~AsynUser()
{
    if(!usr)
        return;
    pasynManager->freeAsynUser(usr);
}

const char* AsynUser::name() throw()
{
    const char *ret=0;
    if(pasynManager->getPortName(usr, &ret)!=asynSuccess)
        return "<not connected>";
    return ret;
}

asynUser*
AsynUser::release()
{
    asynUser *t=usr;
    usr=0;
    return t;
}

void
AsynUser::process()
{
    asynPrint(usr, ASYN_TRACE_FLOW, "%s: %s\n", name(), BOOST_CURRENT_FUNCTION);
}

void AsynUser::procCB(asynUser *usr) {
  AsynUser *inst = (AsynUser*)usr->userPvt;
try {
    inst->process();

    } catch(std::exception& e) {
      asynPrint(usr, ASYN_TRACE_ERROR, "%s: %s unhandled exception '%s'\n",
              inst->name(), BOOST_CURRENT_FUNCTION, e.what());
  }
}

extern "C"
void createOKPort(const char *port, const char *serial, int debug) {
    // can't delete it anyway...
    new arcSPort(port, serial, debug);
}

#include <epicsExport.h>
#include <iocsh.h>

/* createOKPort */
static const iocshArg createOKPortArg0 = { "portname",iocshArgString};
static const iocshArg createOKPortArg1 = { "serial#",iocshArgString};
static const iocshArg createOKPortArg2 = { "debug",iocshArgInt};
static const iocshArg * const createOKPortArgs[] = {&createOKPortArg0,&createOKPortArg1,&createOKPortArg2};
static const iocshFuncDef createOKPortFuncDef = {"createOKPort",3,createOKPortArgs};
static void createOKPortCallFunc(const iocshArgBuf *args)
{
    createOKPort(args[0].sval,args[1].sval, args[2].ival);
}

static
void arcSPortReg()
{
    iocshRegister(&createOKPortFuncDef,createOKPortCallFunc);
}

epicsExportRegistrar(arcSPortReg);

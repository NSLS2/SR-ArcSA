#ifndef AWGPORT_H
#define AWGPORT_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <stdexcept>

#include <epicsTypes.h>
#include <epicsThread.h>
#include <epicsGuard.h>
#include <epicsTime.h>

#include <asynPortDriver.h>

#include "registers.h"

#include <okFrontPanelDLL.h>

class arcSPort : public asynPortDriver
{
public:
    arcSPort(const char *portName, const char *serial, int debug);
    virtual ~arcSPort();

    virtual void report(FILE *fp, int details);
    virtual asynStatus connect(asynUser *pasynUser);
    virtual asynStatus disconnect(asynUser *pasynUser);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
    virtual asynStatus readInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements, size_t *nIn);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    virtual asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn);

protected:

    // static info
    int P_model;
    int P_id;
    int P_major, P_minor;
    int P_serial;

    // commands
    int P_load;
    int P_reset;
    int P_update;

    // settings
    int P_bit_dir;
    int P_bit_file;

/********************************************************/
/*							*/
/* March 19, 2012					*/
    int	P_WO_EP20;
    int	P_WO_EP21;
    int	P_WO_EP22;
    int	P_WO_EP23;
    int	P_WO_EP24;
    int	P_WO_EP25;
    int	P_WO_EP26;
    int	P_WO_EP27;
    int P_WO_EP28;
    int	P_WO_EP29;
    int	P_WO_EP2a;
    int	P_WO_EP2b;
    int	P_WO_EP2c;
    int	P_WO_EP2d;
    int	P_WI_EP00;
    int	P_WI_EP01;
    int	P_WI_EP02;
    int	P_WI_EP03;
    int	P_WI_EP04;
    int	P_WI_EP05;
    int	P_WI_EP06;
    int	P_WI_EP07;
    int P_ok_SN;

    const char* retParamName(int idx=0) {return retParamName(0,idx);}
    const char* retParamName(int list, int idx) {
        const char *ret=0;
        if(getParamName(list,idx,&ret)!=asynSuccess)
            return "<invalid>";
        return ret;
    }

private:
    static void shutdown(void* raw);
    void stop();

    void update(asynUser*);

    char SerialNo[64];

    okFrontPanel_HANDLE H;
    typedef ok_ErrorCode error_t;

    void hwError(error_t err, const char* file, int line,
                 asynUser *usr=0);

    void init(asynUser *pasynUser);

    bool connected[NUM_CHANNELS];
    bool open;
    bool nextdac;

    epicsTime lastTrig;

    bool zombie; // brains...
struct
	timespec	ts[3];
	char		bitfilename[256];
	char		bitdirname[256];

};

#include <boost/current_function.hpp>

#define FLOW(USR) asynPrint(USR, ASYN_TRACE_FLOW, "%s: %s\n", portName, BOOST_CURRENT_FUNCTION)

/* translate c++ exceptions into asyn errors */

#define CATCHALL(USR) catch(std::exception& e) { \
asynPrint(USR, ASYN_TRACE_ERROR, "%s: %s: %s\n", portName, BOOST_CURRENT_FUNCTION, e.what()); \
return asynError; }

#define CATCHALL_CONTINUE(USR) catch(std::exception& e) { \
asynPrint(USR, ASYN_TRACE_ERROR, "%s: %s: %s\n", portName, BOOST_CURRENT_FUNCTION, e.what()); \
return; }

/* and vis versa */

#define AERR(RET) do { if((RET)!=asynSuccess) { \
std::ostringstream msg; msg<<portName<<": "<<BOOST_CURRENT_FUNCTION<<": on line "<<__LINE__; \
        throw std::runtime_error(msg.str()); } } while(0)

//! Container for asynUser
class AsynUser {
    asynUser *usr;

    AsynUser(const AsynUser&);
    AsynUser& operator=(const AsynUser&);
public:
    AsynUser(const char* port, int addr=-1);
    AsynUser(asynUser* other);

    virtual ~AsynUser();

    asynUser* release();

    const char *name() throw();

    operator asynUser*(){return usr;}

    asynUser* operator->(){return usr;}
    asynUser& operator*(){return *usr;}

    virtual void process();

private:
    static void procCB(asynUser *);
};

#endif // AWGPORT_H

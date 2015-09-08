/*  -*- mode: c++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-

 *  LabJack U6 Plugin for MWorks
 *
 *  100421: Mark Histed created
 *    (based on Nidaq plugin code; Hendry, Maunsell)
 *  120708 histed - revised for two levers
 *  
 *  08-26-2015 Ziye - revised for a universal setting
 *             this version removes Lever2 and Lever2Solenoid
 *             moves Lever1, Lever1Solenoid and Strobed_word to CIO/MIO
 *             fix ljU6ReadPorts output type
 *             fix I/O state bit for CIO
 *             change 12 bit strobed word to 8 bit (turn off CIO)
 *             turn on both timer(2) and counter(2)
 *             change Lever1 state to ciostate in ReadLeverDI
 */


#include <boost/bind.hpp>
#include "u6.h"
#include "LabJackU6Device.h"

#define kBufferLength	2048
#define kDIDeadtimeUS	5000	
#define kDIReportTimeUS	5000


#define LJU6_EMPIRICAL_DO_LATENCY_MS 1   // average when plugged into a highspeed hub.  About 8ms otherwise

static const unsigned char ljPortDir[3] = {  // 0 input, 1 output, perform bit manipulation
    (  (0x01 << LJU6_REWARD_FIO)                       // FIO
     | (0x01 << LJU6_LASERTRIGGER_FIO)),
        0xff,                                          // EIO
    (  (0x00 << (LJU6_LEVER1_CIO - LJU6_CIO_OFFSET))   // CIO
     | (0x01 << (LJU6_LEVER1SOLENOID_CIO - LJU6_CIO_OFFSET))
     | (0x01 << (LJU6_STROBE_CIO - LJU6_CIO_OFFSET)))  
};

// Copied from libusb.h
typedef struct libusb_device_handle libusb_device_handle;
extern "C" int libusb_reset_device(libusb_device_handle *dev);


BEGIN_NAMESPACE_MW


/* helper function declarations */
// update lever state
void debounce_bit(unsigned int *thisState, unsigned int *lastState, MWTime *lastTransitionTimeUS, shared_ptr <Clock> clock);


const std::string LabJackU6Device::PULSE_DURATION("pulse_duration");
const std::string LabJackU6Device::PULSE_ON("pulse_on");
const std::string LabJackU6Device::LEVER1("lever1");
const std::string LabJackU6Device::LEVER1_SOLENOID("lever1_solenoid");
const std::string LabJackU6Device::LASER_TRIGGER("laser_trigger");
const std::string LabJackU6Device::STROBED_DIGITAL_WORD("strobed_digital_word");
const std::string LabJackU6Device::COUNTER("counter");
const std::string LabJackU6Device::COUNTER2("counter2");
const std::string LabJackU6Device::QUADRATURE("quadrature");


/* Notes to self MH 100422
 This is how we do setup and cleanup
    * Constructor [called at plugin load time]
        Sets instant variables
    * core calls attachPhysicalDevice()
        -> variableSetup()
    * startup()  [called by core; once, I think]
    * startDeviceIO()  [called by core; every trial]
    * stopDeviceIO()   [called by core; every trial]
    * shutdown() [called by core; once, I think]
    * Destructor
        -> detachPhysicalDevice
 
 What we do:
    Constructor [sets up instance variables]
    
*/

/* Object functions **********************/


void LabJackU6Device::describeComponent(ComponentInfo &info) {
    IODevice::describeComponent(info);
    
    info.setSignature("iodevice/labjacku6");
    
    info.addParameter(PULSE_DURATION);
    info.addParameter(PULSE_ON, "false");
    info.addParameter(LEVER1, "false");
    info.addParameter(LEVER1_SOLENOID, "false");
    info.addParameter(LASER_TRIGGER, "false");
    info.addParameter(STROBED_DIGITAL_WORD, "0");
    info.addParameter(COUNTER, "0");
    info.addParameter(COUNTER2, "0");
    info.addParameter(QUADRATURE, "0");
}


// Constructor for LabJackU6Device
LabJackU6Device::LabJackU6Device(const ParameterValueMap &parameters) :
    IODevice(parameters),
    scheduler(Scheduler::instance()),
    pulseDuration(parameters[PULSE_DURATION]),
    pulseOn(parameters[PULSE_ON]),
    lever1(parameters[LEVER1]),
    lever1Solenoid(parameters[LEVER1_SOLENOID]),
    laserTrigger(parameters[LASER_TRIGGER]),
    strobedDigitalWord(parameters[STROBED_DIGITAL_WORD]),
    counter(parameters[COUNTER]),
    counter2(parameters[COUNTER2]),
    quadrature(parameters[QUADRATURE]),
    deviceIOrunning(false),
    ljHandle(NULL),
    lastLever1Value(-1),  // -1 means always report first value
    lastLever1TransitionTimeUS(0)
{
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf(M_IODEVICE_MESSAGE_DOMAIN, "LabJackU6Device: constructor");
	}
}


// Destructor
LabJackU6Device::~LabJackU6Device(){ 
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: destructor");
	}
	if (pulseScheduleNode != NULL) {
		boost::mutex::scoped_lock locker(pulseScheduleNodeLock); 
        pulseScheduleNode->cancel();
		pulseScheduleNode->kill();
    }
    detachPhysicalDevice();
}

// Schedule function, never scheduled if LabJack is not initialized

void *endPulse(const shared_ptr<LabJackU6Device> &gp) {
	
	shared_ptr <Clock> clock = Clock::instance();			
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: endPulse callback at %lld us", clock->getCurrentTimeUS());
    }
    gp->pulseDOLow();
    return(NULL);
}


void LabJackU6Device::pulseDOHigh(int pulseLengthUS) {
	shared_ptr <Clock> clock = Clock::instance();
    // Takes and releases pulseScheduleNodeLock
    // Takes and releases driver lock

	// Set the DO high first
    boost::mutex::scoped_lock lock(ljU6DriverLock);  //printf("lock DOhigh\n"); fflush(stdout);
	if (ljHandle == NULL) {
		return;
	}
    if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: setting pulse high %d ms (%lld)", pulseLengthUS / 1000, clock->getCurrentTimeUS());
	}
	MWTime t1 = clock->getCurrentTimeUS();  // to check elapsed time below
    if (ljU6WriteDO(ljHandle, LJU6_REWARD_FIO, 1) == false) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing digital output high; device likely to not work from here on");
        return;
    }
    lock.unlock();      //printf("unlock DOhigh\n"); fflush(stdout);

    if (clock->getCurrentTimeUS() - t1 > 4000) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, 
               "LJU6: Writing the DO took longer than 4ms.  Is the device connected to a high-speed hub?  Pulse length is wrong.");
    }
    
	// Schedule endPulse call
    if (pulseLengthUS <= LJU6_EMPIRICAL_DO_LATENCY_MS+1) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "LJU6: requested pulse length %dms too short (<%dms), not doing digital IO", 
               pulseLengthUS, LJU6_EMPIRICAL_DO_LATENCY_MS+1);
    } else {
        // long enough, do it
        boost::mutex::scoped_lock pLock(pulseScheduleNodeLock);
        shared_ptr<LabJackU6Device> this_one = shared_from_this();
        pulseScheduleNode = scheduler->scheduleMS(std::string(FILELINE ": ") + getTag(),
											  (pulseLengthUS / 1000.0) - LJU6_EMPIRICAL_DO_LATENCY_MS, 
											  0, 
											  1, 
											  boost::bind(endPulse, this_one),
											  M_DEFAULT_IODEVICE_PRIORITY,
											  M_DEFAULT_IODEVICE_WARN_SLOP_US,
											  M_DEFAULT_IODEVICE_FAIL_SLOP_US,
											  M_MISSED_EXECUTION_DROP);
	
        MWTime current = clock->getCurrentTimeUS();
        if (VERBOSE_IO_DEVICE >= 2) {
            mprintf("LabJackU6Device:  schedule endPulse callback at %lld us (%lld)", current, clock->getCurrentTimeUS());
        }
        highTimeUS = current;
    }
    
}

// set the DO low

void LabJackU6Device::pulseDOLow() {

	shared_ptr <Clock> clock = Clock::instance();    
    MWTime current = clock->getCurrentTimeUS();

    boost::mutex::scoped_lock lock(ljU6DriverLock);
	if (ljHandle == NULL) {
		return;
	}
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: pulseDOLow at %lld us (pulse %lld us long)", current, current - highTimeUS);
    }
    if (ljU6WriteDO(ljHandle, LJU6_REWARD_FIO, 0) == false) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing digital output low; device likely to not work from here on");
    }
	// set juice variable low
	pulseDuration->setValue(Datum((long)0));
	
}
    
void LabJackU6Device::leverSolenoidDO(bool state) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);

    if (ljU6WriteDO(ljHandle, LJU6_LEVER1SOLENOID_CIO, state) != true) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing lever 1 solenoid state; device likely to be broken (state %d)", state);
    }
}

void LabJackU6Device::laserDO(bool state) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
	
    if (ljU6WriteDO(ljHandle, LJU6_LASERTRIGGER_FIO, state) != true) {  
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing laser trigger state; device likely to be broken (state %d)", state);
    }
	
}

void LabJackU6Device::strobedDigitalWordDO(unsigned int digWord) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
	
    LabJackU6Device::ljU6WriteStrobedWord(ljHandle, digWord); // error checking done inside here; will call merror
	
}


bool LabJackU6Device::readLeverDI(bool *outLever1)
// Takes the driver lock and releases it
{
	shared_ptr <Clock> clock = Clock::instance();
    
	unsigned int lever1State = 0L;
	
	unsigned int fioState = 0L;
	unsigned int eioState = 0L;
	unsigned int cioState = 0L;
	
	static unsigned int lastLever1State = 0xff;
	static long unsigned slowCount = 0;
	static long unsigned allCount = 0;
	
	double pct;
	
	boost::mutex::scoped_lock lock(ljU6DriverLock);
	
	if (ljHandle == NULL || !this->getActive()) {
		return false;
	}
    
	
	MWTime st = clock->getCurrentTimeUS();
	if (ljU6ReadPorts(ljHandle, &fioState, &eioState, &cioState) != 0 ) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Error reading DI, stopping IO and returning FALSE");
        stopDeviceIO();  // We are seeing USB errors causing this, and the U6 doesn't work anyway, so might as well stop the threads
        return false;
    }
    MWTime elT = clock->getCurrentTimeUS()-st;
    allCount = allCount+1;
	

	if (elT > kDIReportTimeUS) {
		++slowCount;
		if ((slowCount < 20) || (slowCount % 10 == 0)) {
			pct = 100.0*((double)slowCount+1)/((double)allCount);
			mwarning(M_IODEVICE_MESSAGE_DOMAIN, "read port elapsed: this %.3fms, >%.0f ms %ld/%ld times (%4.3f%%)",
					 elT / 1000.0, 
					 kDIReportTimeUS / 1000.0,
					 slowCount, 
					 allCount, 
					 pct);
		}
	}
    
    //mprintf("*******Lever1State: 0x%x", fioState);
	lever1State = (cioState >> (LJU6_LEVER1_CIO - LJU6_CIO_OFFSET)) & 0x01;
    
    // software debouncing
	debounce_bit(&lever1State, &lastLever1State, &lastLever1TransitionTimeUS, clock);
	
    *outLever1 = lever1State;
	
	return(1);
}

/*******************************************************************/

void debounce_bit(unsigned int *thisState, unsigned int *lastState, MWTime *lastTransitionTimeUS, shared_ptr <Clock> clock) {
	// software debouncing
	if (*thisState != *lastState) {
		if (clock->getCurrentTimeUS() - *lastTransitionTimeUS < kDIDeadtimeUS) {
			*thisState = *lastState;				// discard changes during deadtime
			mwarning(M_IODEVICE_MESSAGE_DOMAIN, 
                     "LabJackU6Device: readLeverDI, debounce rejecting new read (last %lld now %lld, diff %lld)", 
                     *lastTransitionTimeUS, 
                     clock->getCurrentTimeUS(),
                     clock->getCurrentTimeUS() - *lastTransitionTimeUS);
		}
		*lastState = *thisState;					// record and report the transition
		*lastTransitionTimeUS = clock->getCurrentTimeUS();
	}
}	
	

// External function for scheduling

void *update_lever(const weak_ptr<LabJackU6Device> &gp){
	shared_ptr <Clock> clock = Clock::instance();
	shared_ptr <LabJackU6Device> sp = gp.lock();
	sp->pollAllDI();
	sp.reset();
    return NULL;
}

bool LabJackU6Device::pollAllDI() {	
	
    bool lever1Value;
    bool res;
    
    res = readLeverDI(&lever1Value);
    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "levers: %d %d", lever1Value, lever2Value);
    
    if (!res) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "LJU6: error in readLeverDI()");
    }

    // Change MW variable value only if switch state is unchanged, or this is the first time through
    if ( (lastLever1Value == -1) // -1 means first time through
        || (lever1Value != lastLever1Value) ) {
        
        lever1->setValue(Datum(lever1Value));
        lastLever1Value = lever1Value;
    }    
    
	return true;
}



/* IODevice virtual calls (made by MWorks) ***********************/


// Attempt to find the LabJack hardware and initialize it.

bool LabJackU6Device::initialize() {
	
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: initialize");
	}

    boost::mutex::scoped_lock lock(ljU6DriverLock);
    assert(ljHandle == NULL);  // should not try to configure if already open.  Perhaps can relax this in the future
	ljHandle = openUSBConnection(-1);						    // Open first available U6 on USB
    ljU6DriverLock.unlock();

	if (ljHandle == NULL) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Error opening LabJack U6.  Is it connected to USB?");
        return false;														// no cleanup needed
    }
    setupU6PortsAndRestartIfDead();
    if (VERBOSE_IO_DEVICE >= 0) {
        mprintf("LabJackU6Device::initialize: found LabJackU6");
    }

	// This configures digital output on the ports so do it after we setup the hardware lines.
	this->variableSetup();

	
    // set output ports to desired state here
    
    if (!ljU6WriteDO(ljHandle, LJU6_LEVER1SOLENOID_CIO, 0) == 1)
        return false; // merror is done in ljU6WriteDO
    this->lever1Solenoid->setValue(Datum(M_BOOLEAN, 0));
    
    if (!ljU6WriteDO(ljHandle, LJU6_REWARD_FIO, 0) == 1) 
        return false; // merror is done in ljU6WriteDO
    this->pulseOn->setValue(Datum(M_INTEGER, 0));
    this->pulseDuration->setValue(Datum(M_INTEGER, 0));
    
    if (!ljU6WriteDO(ljHandle, LJU6_LASERTRIGGER_FIO, 0) == 1) 
        return false; // merror is done in ljU6WriteDO    
    this->laserTrigger->setValue(Datum(M_BOOLEAN, 0));
    
    
    if (!ljU6WriteDO(ljHandle, LJU6_STROBE_CIO, 0) == 1)
        return false; // merror is done in ljU6WriteDO   
    this->strobedDigitalWord->setValue(Datum(M_INTEGER, 0));

    
    return true;
}

bool LabJackU6Device::setupU6PortsAndRestartIfDead() {
    // This is not a pleasant solution, but it works for now
    // takes and releases lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);  
    assert(ljHandle != NULL);  // you must have opened before calling this

    // Do physical port setup
    if (!ljU6ConfigPorts(ljHandle)) {
        // assume dead
        
        // Force a USB re-enumerate, and reconnect
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "LJU6 found dead, restarting.  (bug if not on MWServer restart)");

        libusb_reset_device((libusb_device_handle *)ljHandle); // patched usb library uses ReEnumerate
        closeUSBConnection(ljHandle);

        sleep(1.2); // histed: MaunsellMouse1 - 0.1s not enough, 0.2 works, add padding
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "Sleeping for 1.2 s after restarting LJU6");
    
        if( (ljHandle = openUSBConnection(-1)) == NULL) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Error: could not reopen USB U6 device after reset; U6 will not work now.");
            return false;  // no cleanup needed
        }
    
        // Redo port setup
        if (!ljU6ConfigPorts(ljHandle)) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Error: configuring U6 after restart, U6 will not work now.  Check for patched version of libusb with reenumerate call.\n");
            return false;  // no cleanup needed
        }
    }

    return true;
}

bool LabJackU6Device::startup() {
	// Do nothing right now
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: startup");
	}
	return true;
}


bool LabJackU6Device::shutdown(){
	// Do nothing right now
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: shutdown");
	}
	return true;
}


bool LabJackU6Device::startDeviceIO(){
    // Start the scheduled IO on the LabJackU6.  This starts a thread that reads the input ports
	
	if (VERBOSE_IO_DEVICE >= 1) {
		mprintf("LabJackU6Device: startDeviceIO");
	}
	if (deviceIOrunning) {
		merror(M_IODEVICE_MESSAGE_DOMAIN, 
               "LabJackU6Device startDeviceIO:  startDeviceIO request was made without first stopping IO, aborting");
        return false;
	}
    
    // check hardware and restart if necessary
    setupU6PortsAndRestartIfDead();

//	schedule_nodes_lock.lock();			// Seems to be no longer supported in MWorks
	
	setActive(true);
	deviceIOrunning = true;

	shared_ptr<LabJackU6Device> this_one = shared_from_this();
	pollScheduleNode = scheduler->scheduleUS(std::string(FILELINE ": ") + getTag(),
                                             (MWTime)0, 
                                             LJU6_DITASK_UPDATE_PERIOD_US, 
                                             M_REPEAT_INDEFINITELY, 
                                             boost::bind(update_lever, weak_ptr<LabJackU6Device>(this_one)),
                                             M_DEFAULT_IODEVICE_PRIORITY,
                                             LJU6_DITASK_WARN_SLOP_US,
                                             LJU6_DITASK_FAIL_SLOP_US,                                             
                                             M_MISSED_EXECUTION_DROP);
	
	//schedule_nodes.push_back(pollScheduleNode);       
//	schedule_nodes_lock.unlock();		// Seems to be no longer supported in MWorks

	return true;
}

bool LabJackU6Device::stopDeviceIO(){

    // Stop the LabJackU6 collecting data.  This is typically called at the end of each trial.
    
	if (VERBOSE_IO_DEVICE >= 1) {
		mprintf("LabJackU6Device: stopDeviceIO");
	}
	if (!deviceIOrunning) {
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "stopDeviceIO: already stopped on entry; using this chance to turn off lever solenoids");
        
        // force off solenoid
        this->lever1Solenoid->setValue(false);
        leverSolenoidDO(false);

		return false;
	}
	
	// stop all the scheduled DI checking (i.e. stop calls to "updateChannel")
	//stopAllScheduleNodes();								// IO device base class method -- this is thread safe
	if (pollScheduleNode != NULL) {
        //merror(M_IODEVICE_MESSAGE_DOMAIN, "Error: pulseDOL
		boost::mutex::scoped_lock(pollScheduleNodeLock);
        pollScheduleNode->cancel();
		//pollScheduleNode->kill();  // MH This is not allowed!  This can make both the USB bus unhappy and also leave the lock
                                     //    in a locked state.  
                                     //    If you insist on killing a thread that may be talking to the LabJack you should reset the USB bus.
    }

	//setActive(false);   // MH - by leaving active == true, we can use the Reward window to schedule pulses when trials are not running
	deviceIOrunning = false;
	return true;
}


void LabJackU6Device::variableSetup() {

    //	weak_ptr<LabJackU6Device> weak_self_ref(getSelfPtr<LabJackU6Device>());
    	weak_ptr<LabJackU6Device> weak_self_ref(component_shared_from_this<LabJackU6Device>());

	shared_ptr<Variable> doReward = this->pulseOn;
	shared_ptr<VariableNotification> notif(new LabJackU6DeviceOutputNotification(weak_self_ref));
	doReward->addNotification(notif);
    
	// lever1Solenoid
	shared_ptr<Variable> doL1S = this->lever1Solenoid;
	shared_ptr<VariableNotification> notif2(new LabJackU6DeviceL1SNotification(weak_self_ref));
	doL1S->addNotification(notif2);	
    
	// laserTrigger
	shared_ptr<Variable> doLT = this->laserTrigger;
	shared_ptr<VariableNotification> notif3(new LabJackU6DeviceLTNotification(weak_self_ref));
	doLT->addNotification(notif3);
	
	// strobedDigitalWord
	shared_ptr<Variable> doSDW = this->strobedDigitalWord;
	shared_ptr<VariableNotification> notif4(new LabJackU6DeviceSDWNotification(weak_self_ref));
	doSDW->addNotification(notif4);
	
	connected = true;	
}

void LabJackU6Device::detachPhysicalDevice() {
	if (VERBOSE_IO_DEVICE >= 1) {
		mprintf("LabJackU6Device: detachPhysicalDevice");
	}
    assert(connected == true); // "Was not connected on entry to detachPhysicalDevice");
		
    boost::mutex::scoped_lock lock(ljU6DriverLock);  
	//printf("lock in detachP\n");
    assert(ljHandle != NULL); // "Device handle is NULL before attempt to disconnect");
    
    closeUSBConnection(ljHandle);
    ljHandle = NULL;

}

/* Hardware functions *********************************/
// None of these have any locking, callers must lock

bool LabJackU6Device::ljU6ConfigPorts(HANDLE Handle) {
    /// set up IO ports
    uint8 sendDataBuff[7]; // recDataBuff[1];
    uint8 Errorcode, ErrorFrame;
    

    // Setup FIO as constants specify.  
    //       EIO always output
    //       CIO mask is hardcoded
    
    sendDataBuff[0] = 29;       // PortDirWrite
    sendDataBuff[1] = 0xff;     // update mask for FIO: update all
    sendDataBuff[2] = 0xff;     // update mask for EIO
    sendDataBuff[3] = 0x0f;     // update mask for CIO (only 4 bits)
    
    sendDataBuff[4] = ljPortDir[0];
    sendDataBuff[5] = ljPortDir[1];
    sendDataBuff[6] = ljPortDir[2];         
    
    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "*****************Output %02x %02x %02x %02x %02x %02x %02x\n",
    //       sendDataBuff[0], sendDataBuff[1], sendDataBuff[2], sendDataBuff[3],
    //       sendDataBuff[4], sendDataBuff[5], sendDataBuff[6]);
    
    if(ehFeedback(Handle, sendDataBuff, sizeof(sendDataBuff), &Errorcode, &ErrorFrame, NULL, 0) < 0) {
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehFeedback error, see stdout");  // note we will get a more informative error on stdout
        return false;  
    }
    if(Errorcode) {
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "ehFeedback: error with command, errorcode was %d", Errorcode);
        return false;
    } 

    // Configure timer and counter

    long aEnableTimers[] = { 1, 1, 0, 0 };  // Use Timer0 and Timer1
    long aEnableCounters[] = { 1, 1 };      // Use Counter0 and Counter1
    long aTimerModes[] = { 8, 8, 0, 0 };  // Use quadrature input mode for both timers
    double aTimerValues[] = { 0.0, 0.0, 0.0, 0.0 };
    
   
    if (long error = eTCConfig(Handle,
                               aEnableTimers,
                               aEnableCounters,
                               LJU6_TCPIN_OFFSET,           // TCPinOffset
                               LJ_tc48MHZ,                  // TimerClockBaseIndex
                               0,                           // TimerClockDivisor
                               aTimerModes,
                               aTimerValues,
                               0,                           // Reserved1
                               0))                          // Reserved2
    {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "eTCConfig failed, error code was %ld", error);
        return false;
    }
    
    return true;

    // cleanup now done externally to this function
}



long LabJackU6Device::ljU6ReadPorts(HANDLE Handle,
									unsigned int *fioState, unsigned int *eioState, unsigned int *cioState)
{
    uint8 sendDataBuff[9];      // Hard-coded
    uint8 recDataBuff[15];      // Hard-coded
	
    sendDataBuff[0] = 26;       //IOType is PortStateRead

    sendDataBuff[1] = 42;       //IOType is Timer0
    sendDataBuff[2] = 0;        //  - Don't reset timer
    sendDataBuff[3] = 0;        //  - Value LSB (ignored)
    sendDataBuff[4] = 0;        //  - Value MSB (ignored)

    sendDataBuff[5] = 54;       //IOType is Counter0
    sendDataBuff[6] = 0;        //  - Don't reset counter
    sendDataBuff[7] = 55;       //IOType is Counter1
    sendDataBuff[8] = 0;        //  - Don't reset counter
    
    uint8 Errorcode, ErrorFrame;
	
    if(ehFeedback(Handle, sendDataBuff, sizeof(sendDataBuff), &Errorcode, &ErrorFrame, recDataBuff, sizeof(recDataBuff)) < 0)
        return -1;
    if(Errorcode)
        return (long)Errorcode;
	
	*fioState = recDataBuff[0];
	*eioState = recDataBuff[1];
	*cioState = recDataBuff[2];

    // debug
    //mprintf("FIO 0x%x EIO 0x%x CIO 0x%x", *fioState, *eioState, *cioState);
    // debug timer and counter
    //mprintf("*****************Output q1:%02x q2:%02x q3:%02x q4:%02x c1:%02x c2:%02x c3:%02x c4:%02x c1:%02x c2:%02x c3:%02x c4:%02x\n",
    //       recDataBuff[3], recDataBuff[4], recDataBuff[5], recDataBuff[6],
    //       recDataBuff[7], recDataBuff[8], recDataBuff[9], recDataBuff[10], recDataBuff[11],
    //      recDataBuff[12], recDataBuff[13], recDataBuff[14]);

    // Unpack timer value (i.e. quadrature)
    std::int32_t quadratureValue;
    for (size_t i = 0; i < 4; i++) {
    	// timer output has 4 bit and two channels have the same output
        ((uint8 *)(&quadratureValue))[i] = recDataBuff[3 + i];
    }
    quadratureValue = CFSwapInt32LittleToHost(quadratureValue);  // Convert to host byte order
    //mprintf("*****Quadrature = %d *******", quadratureValue);
    
    // Update quadrature variable (only if quadrature value has changed)
    if (quadrature->getValue().getInteger() != quadratureValue) {
        quadrature->setValue(long(quadratureValue));
    }
    
    // Unpack counter value
    std::int32_t counterValue[2];
    for (size_t j = 0; j < 2; j++) {
        for (size_t i = 0; i < 4; i++) {
            // each counter output has 4 bit
            ((uint8 *)(counterValue + j))[i] = recDataBuff[3 + 4*(j+1) + i];
        }
        counterValue[j] = CFSwapInt32LittleToHost(counterValue[j]);  // Convert to host byte order
    }
    //mprintf("*****Counter = %d *******", counterValue[0]);
    
    // Update counter variables (only if counter value has changed)
    if (counter->getValue().getInteger() != counterValue[0]) {
        counter->setValue(long(counterValue[0]));
    }
    if (counter2->getValue().getInteger() != counterValue[1]) {
        counter2->setValue(long(counterValue[1]));
    }
    
    return 0;
    
}

bool LabJackU6Device::ljU6WriteDO(HANDLE Handle, long Channel, long State) {

    uint8 sendDataBuff[2]; // recDataBuff[1];
    uint8 Errorcode, ErrorFrame;

    sendDataBuff[0] = 11;             //IOType is BitStateWrite
    sendDataBuff[1] = Channel + 128*((State > 0) ? 1 : 0);  //IONumber(bits 0-4) + State (bit 7)

    if(ehFeedback(Handle, sendDataBuff, sizeof(sendDataBuff), &Errorcode, &ErrorFrame, NULL, 0) < 0) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehFeedback error, see stdout");  // note we will get a more informative error on stdout
        return false;
    }
    if(Errorcode) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "ehFeedback: error with command, errorcode was %d", Errorcode);
        return false;
    }
    return true;
}

bool LabJackU6Device::ljU6WriteStrobedWord(HANDLE Handle, unsigned int inWord) {
	
	uint8 outEioBits = inWord & 0xff;      // Turn on specific EIO ports
	
    uint8 sendDataBuff[27]; 
    uint8 Errorcode, ErrorFrame;

	if (inWord > 0xff) {
		merror(M_IODEVICE_MESSAGE_DOMAIN, "error writing strobed word; value is larger than 8 bits (nothing written)");
		return false;
	}
		
	
	
    sendDataBuff[0] = 27;			// PortStateWrite, 7 bytes total
	sendDataBuff[1] = 0x00;			// FIO: don't update
	sendDataBuff[2] = 0xff;			// EIO: update
	sendDataBuff[3] = 0x00;			// CIO: don't update
	sendDataBuff[4] = 0x00;			// FIO: data
	sendDataBuff[5] = outEioBits;	        // EIO: data
	sendDataBuff[6] = 0x00;	                // CIO: data
	
	sendDataBuff[7] = 5;			// WaitShort
	sendDataBuff[8] = 1;			// Time(*128us)
	
	sendDataBuff[9]  = 11;			// BitStateWrite, update CIO2
	sendDataBuff[10] = 18 | 0x80;	// bits (0-4): port # (CIO2); last bit(bit 7), state
	
	sendDataBuff[11] = 5;			// WaitShort
	sendDataBuff[12] = 1;			// Time(*128us)
	
        sendDataBuff[13] = 27;			// PortStateWrite, 7 bytes total
	sendDataBuff[14] = 0x00;		// FIO: don't update
	sendDataBuff[15] = 0xff;		// EIO: update
	sendDataBuff[16] = 0x04;		// CIO: update CIO2
	sendDataBuff[17] = 0x00;		// FIO: data
	sendDataBuff[18] = 0x00;		// EIO: data
	sendDataBuff[19] = 0x00;		// CIO: data
	
	sendDataBuff[20] = 29;			// PortDirWrite - for some reason the above seems to reset the FIO input/output state
	sendDataBuff[21] = 0xff;		//  FIO: update
	sendDataBuff[22] = 0xff;		//  EIO: update
	sendDataBuff[23] = 0xff;		//  CIO: update
	sendDataBuff[24] = ljPortDir[0];        //  FIO hardcoded above
	sendDataBuff[25] = ljPortDir[1];        //  EIO hardcoded above
	sendDataBuff[26] = ljPortDir[2];        //  CIO hardcoded above

    // ehFeedback (                    size of sendDataBuff                 )
    if(ehFeedback(Handle, sendDataBuff, sizeof(sendDataBuff), &Errorcode, &ErrorFrame, NULL, 0) < 0) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehFeedback error, see stdout");  // note we will get a more informative error on stdout
        return false;
    }
    if(Errorcode) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "ehFeedback: error with command, errorcode was %d", Errorcode);
        return false;
    }
	
    return true;
}


END_NAMESPACE_MW

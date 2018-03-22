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
 *
 * 09-10-2015 Ziye - make changes on LaserTrigger so that the voltage is directly
 *            outputed from DAC0 instead of FIO3
 *
 * 12-10-2015 Ziye - add control for 2 led syncing with camera
 */


#include <boost/bind.hpp>
#include "LabJackU6Device.h"

using std::vector;

#define kBufferLength	2048
#define kDIDeadtimeUS	5000
#define kDIReportTimeUS	5000


#define LJU6_EMPIRICAL_DO_LATENCY_MS 1   // average when plugged into a highspeed hub.  About 8ms otherwise

static const unsigned char ljPortDir[3] = {  // 0 input, 1 output, perform bit manipulation
    (  (0x01 << LJU6_REWARD_FIO)                                // FIO
     | (0x00 << LJU6_LEVER1_FIO)
     //| (0x01 << LJU6_LED1_FIO)
     | (0x01 << LJU6_LED2_FIO)),
        0xff,
    (  (0x01 << (LJU6_LASERTRIGGER_CIO - LJU6_CIO_OFFSET))
     | (0x01 << (LJU6_LEVER1SOLENOID_CIO - LJU6_CIO_OFFSET))
     | (0x01 << (LJU6_STROBE_CIO - LJU6_CIO_OFFSET))
     | (0x00 << (LJU6_QTRIGGER_CIO - LJU6_CIO_OFFSET)) )
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
const std::string LabJackU6Device::TRIAL_LASER_POWERMW("trial_laser_powerMw");
const std::string LabJackU6Device::LASER_TRIGGER("laser_trigger");
const std::string LabJackU6Device::LASER_DURATION("laser_duration");
const std::string LabJackU6Device::STROBED_DIGITAL_WORD("strobed_digital_word");
const std::string LabJackU6Device::COUNTER("counter");
const std::string LabJackU6Device::COUNTER2("counter2");
//const std::string LabJackU6Device::COUNTER3("counter3");
//const std::string LabJackU6Device::COUNTER4("counter4");
const std::string LabJackU6Device::QUADRATURE("quadrature");
const std::string LabJackU6Device::OPTIC_DEVICE("optic_device");
const std::string LabJackU6Device::LED_SEQ("led_seq");
const std::string LabJackU6Device::DO2LED("do2led");
const std::string LabJackU6Device::LED1_STATUS("led1_status");
const std::string LabJackU6Device::LED2_STATUS("led2_status");
const std::string LabJackU6Device::LED_DURATION("LED_duration");
const std::string LabJackU6Device::QBIN_SIZE("Qbin_size");
const std::string LabJackU6Device::QBIN_TIMEUS("Qbin_timeUS");
const std::string LabJackU6Device::DOCB("doCB");
const std::string LabJackU6Device::START_CB_STILL("start_CB_still");
const std::string LabJackU6Device::STILL_DURATION("still_duration");
const std::string LabJackU6Device::START_CB_RUNNING("start_CB_running");
const std::string LabJackU6Device::RUNNING_CRITERIA("running_criteria");
const std::string LabJackU6Device::QPULSE_CRITERIA("Qpulse_criteria");
const std::string LabJackU6Device::CHECKRUN("checkrun");
const std::string LabJackU6Device::DO_WHEELSPEED("do_wheelspeed");
const std::string LabJackU6Device::WS_DURATIONUS("ws_durationUS");
const std::string LabJackU6Device::WHEEL_SPEED("wheel_speed");
const std::string LabJackU6Device::PUFF_DURATION("puff_duration");

/* Notes to self MH 100422
 Notes 09252015
 
 This is how we do setup and cleanup
 * Constructor [called at plugin load time]
 Sets instant variables
 * core calls attachPhysicalDevice()
 -> variableSetup()
 * startup()        [called by core; once]
 * initialize()     [caleed once]
 * startDeviceIO()  [called by core; every time start button is pressed]
 * stopDeviceIO()   [called by core; every time stop button is pressed]
 * shutdown() [Not called]
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
    info.addParameter(TRIAL_LASER_POWERMW, "0");
    info.addParameter(LASER_TRIGGER, "false");
    info.addParameter(LASER_DURATION, "0");
    info.addParameter(STROBED_DIGITAL_WORD, "0");
    info.addParameter(COUNTER, "0");
    info.addParameter(COUNTER2, "0");
    //info.addParameter(COUNTER3, "0");
    //info.addParameter(COUNTER4, "0");
    info.addParameter(QUADRATURE, "0");
    info.addParameter(OPTIC_DEVICE, "led");
    info.addParameter(DO2LED,"false");
    info.addParameter(LED_SEQ,"1");
    info.addParameter(LED1_STATUS,"false");
    info.addParameter(LED2_STATUS,"false");
    info.addParameter(LED_DURATION,"0");
    info.addParameter(QBIN_SIZE,"0");
    info.addParameter(QBIN_TIMEUS,"0");
    info.addParameter(DOCB,"false");
    info.addParameter(START_CB_STILL,"false");
    info.addParameter(STILL_DURATION,"0");
    info.addParameter(START_CB_RUNNING,"false");
    info.addParameter(RUNNING_CRITERIA,"0");
    info.addParameter(QPULSE_CRITERIA,"0");
    info.addParameter(CHECKRUN,"false");
    info.addParameter(DO_WHEELSPEED,"false");
    info.addParameter(WS_DURATIONUS,"1");
    info.addParameter(WHEEL_SPEED,"0");
    info.addParameter(PUFF_DURATION,"0");

}


// Constructor for LabJackU6Device
LabJackU6Device::LabJackU6Device(const ParameterValueMap &parameters) :
IODevice(parameters),
lastLever1TransitionTimeUS(0),
QTimeUS(0),
QTime2US(0),
lastLEDonTimeUS(0),
lastLever1Value(-1),  // -1 means always report first value
lastCameraState(0),
ledCount(0),
lastBinQuadratureValue(0),
lastQuadratureValue(0),
trial(0),
scheduler(Scheduler::instance()),
ljHandle(NULL),
pulseDuration(parameters[PULSE_DURATION]),
pulseOn(parameters[PULSE_ON]),
lever1(parameters[LEVER1]),
lever1Solenoid(parameters[LEVER1_SOLENOID]),
tTrialLaserPowerMw(parameters[TRIAL_LASER_POWERMW]),
laserTrigger(parameters[LASER_TRIGGER]),
laserDuration(parameters[LASER_DURATION]),
strobedDigitalWord(parameters[STROBED_DIGITAL_WORD]),
counter(parameters[COUNTER]),
counter2(parameters[COUNTER2]),
//counter3(parameters[COUNTER3]),
//counter4(parameters[COUNTER4]),
quadrature(parameters[QUADRATURE]),
optic_device(parameters[OPTIC_DEVICE]),
do2led(parameters[DO2LED]),
led_seq(parameters[LED_SEQ]),
led1_status(parameters[LED1_STATUS]),
led2_status(parameters[LED2_STATUS]),
LED_duration(parameters[LED_DURATION]),
Qbin_size(parameters[QBIN_SIZE]),
Qbin_timeUS(parameters[QBIN_TIMEUS]),
doCB(parameters[DOCB]),
start_CB_still(parameters[START_CB_STILL]),
still_duration(parameters[STILL_DURATION]),
start_CB_running(parameters[START_CB_RUNNING]),
running_criteria(parameters[RUNNING_CRITERIA]),
Qpulse_criteria(parameters[QPULSE_CRITERIA]),
checkrun(parameters[CHECKRUN]),
do_wheelspeed(parameters[DO_WHEELSPEED]),
ws_durationUS(parameters[WS_DURATIONUS]),
wheel_speed(parameters[WHEEL_SPEED]),
puffDuration(parameters[PUFF_DURATION]),
deviceIOrunning(false)
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
    }
    detachPhysicalDevice();
}

// Schedule function, never scheduled if LabJack is not initialized
// Reward end
void *endPulse(const shared_ptr<LabJackU6Device> &gp) {
    
    shared_ptr <Clock> clock = Clock::instance();
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: endPulse callback at %lld us", clock->getCurrentTimeUS());
    }
    gp->pulseDOLow();
    return(NULL);
}

// Air puff end
void *endPuff(const shared_ptr<LabJackU6Device> &gp) {
    
    shared_ptr <Clock> clock = Clock::instance();
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: endPulse callback at %lld us", clock->getCurrentTimeUS());
    }
    gp->puffDOLow();
    return(NULL);
}

// Led end
void *endLaser(const shared_ptr<LabJackU6Device> &gp) {
    
    shared_ptr <Clock> clock = Clock::instance();
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: endLaser callback at %lld us", clock->getCurrentTimeUS());
    }
    gp->laserDOLow();
    return(NULL);
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
    bool cameraState;
    bool cameraState2;
    
    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "levers: %d %d", lever1Value, lever2Value);
    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Camera State: %d:", cameraState);
        
    if (readLeverDI(&lever1Value, &cameraState, &cameraState2) != true) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "LJU6: error in readLeverDI()");
    }
    
    // Change MW variable value only if switch state is unchanged, or this is the first time through
    if ( (lastLever1Value == -1) // -1 means first time through
        || (lever1Value != lastLever1Value) ) {
        
        lever1->setValue(Datum(lever1Value));
        lastLever1Value = lever1Value;
        
    }
    
    // control 2 led if neeeded
    if(do2led->getValue().getBool() == true ) {       // could have created a separated schedule node
        ledDo2(cameraState, cameraState2);            // but since we update all ports in pollAllDI, we might just
    }                               // call do2led here
    
    return true;
}

// set Reward high
void LabJackU6Device::pulseDOHigh(int pulseLengthUS) {
    shared_ptr <Clock> clock = Clock::instance();
    // Takes and releases pulseScheduleNodeLock
    // Takes and releases driver lock
    
    // Set the DO high first
    boost::mutex::scoped_lock lock(ljU6DriverLock);
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
                                                  M_MISSED_EXECUTION_CATCH_UP);
        
        MWTime current = clock->getCurrentTimeUS();
        if (VERBOSE_IO_DEVICE >= 2) {
            mprintf("LabJackU6Device:  schedule endPulse callback at %lld us (%lld)", current, clock->getCurrentTimeUS());
        }
        highTimeUS = current;
    }
    
}

// set the DO low for Reward

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

// set Puff high
void LabJackU6Device::puffDOHigh(int puffLengthMS) {
    shared_ptr <Clock> clock = Clock::instance();
    // Takes and releases pulseScheduleNodeLock
    // Takes and releases driver lock
    
    // Set the DO high first
    boost::mutex::scoped_lock lock(ljU6DriverLock);
    if (ljHandle == NULL) {
        return;
    }
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: setting pulse high %d ms (%lld)", puffLengthMS / 1000, clock->getCurrentTimeUS());
    }
    MWTime t1 = clock->getCurrentTimeUS();  // to check elapsed time below
    if (ljU6WriteDO(ljHandle, LJU6_PUFF_FIO, 1) == false) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing digital output high; device likely to not work from here on");
        return;
    }
    lock.unlock();      //printf("unlock DOhigh\n"); fflush(stdout);
    
    if (clock->getCurrentTimeUS() - t1 > 4000) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "LJU6: Writing the DO took longer than 4ms.  Is the device connected to a high-speed hub?  Pulse length is wrong.");
    }
    
    // Schedule endPulse call
    if (puffLengthMS <= LJU6_EMPIRICAL_DO_LATENCY_MS+1) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "LJU6: requested pulse length %dms too short (<%dms), not doing digital IO",
               puffLengthMS, LJU6_EMPIRICAL_DO_LATENCY_MS+1);
    } else {
        // long enough, do it
        boost::mutex::scoped_lock pLock(pulseScheduleNodeLock);
        shared_ptr<LabJackU6Device> this_one = shared_from_this();
        pulseScheduleNode = scheduler->scheduleMS(std::string(FILELINE ": ") + getTag(),
                                                  (puffLengthMS) - LJU6_EMPIRICAL_DO_LATENCY_MS,
                                                  0,
                                                  1,
                                                  boost::bind(endPuff, this_one),
                                                  M_DEFAULT_IODEVICE_PRIORITY,
                                                  M_DEFAULT_IODEVICE_WARN_SLOP_US,
                                                  M_DEFAULT_IODEVICE_FAIL_SLOP_US,
                                                  M_MISSED_EXECUTION_CATCH_UP);
        
        MWTime current = clock->getCurrentTimeUS();
        if (VERBOSE_IO_DEVICE >= 2) {
            mprintf("LabJackU6Device:  schedule endPuff callback at %lld us (%lld)", current, clock->getCurrentTimeUS());
        }
        highTimeUS = current;
    }
    
}

// set the DO low for Air puff

void LabJackU6Device::puffDOLow() {
    
    shared_ptr <Clock> clock = Clock::instance();
    MWTime current = clock->getCurrentTimeUS();
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
    if (ljHandle == NULL) {
        return;
    }
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: puffDoLow at %lld us (puff %lld us long)", current, current - highTimeUS);
    }
    if (ljU6WriteDO(ljHandle, LJU6_PUFF_FIO, 0) == false) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing digital output low; device likely to not work from here on");
    }
    // set juice variable low
    puffDuration->setValue(Datum((long)0));
}

// set Laser high
void LabJackU6Device::laserDOHigh(int laserLengthMS) {
    shared_ptr <Clock> clock = Clock::instance();
    // Takes and releases pulseScheduleNodeLock
    // Takes and releases driver lock
    
    // Set the DO high first
    boost::mutex::scoped_lock lock(ljU6DriverLock);
    if (ljHandle == NULL) {
        return;
    }
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: setting laser high for %d ms (%lld)", laserLengthMS, clock->getCurrentTimeUS());
    }
    MWTime t1 = clock->getCurrentTimeUS();  // to check elapsed time below
    
    //double power = tTrialLaserPowerMw->getValue().getFloat();
    
    //if (ljU6WriteLaser(ljHandle, power) == false) {
    //    merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing digital output high; device likely to not work from here on");
    //    return;
    //}
    lock.unlock();      //printf("unlock DOhigh\n"); fflush(stdout);
    
    if (clock->getCurrentTimeUS() - t1 > 4000) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "LJU6: Writing the DO took longer than 4ms.  Is the device connected to a high-speed hub?  Pulse length is wrong.");
    }
    
    // Schedule endPulse call
    if (laserLengthMS <= LJU6_EMPIRICAL_DO_LATENCY_MS+1) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "LJU6: requested pulse length %dms too short (<%dms), not doing digital IO",
               laserLengthMS, LJU6_EMPIRICAL_DO_LATENCY_MS+1);
    } else {
        // long enough, do it
        boost::mutex::scoped_lock pLock(laserScheduleNodeLock);
        shared_ptr<LabJackU6Device> this_one = shared_from_this();
        laserScheduleNode = scheduler->scheduleMS(std::string(FILELINE ": ") + getTag(),
                                                  (laserLengthMS) - LJU6_EMPIRICAL_DO_LATENCY_MS,
                                                  0,
                                                  1,
                                                  boost::bind(endLaser, this_one),
                                                  M_DEFAULT_IODEVICE_PRIORITY,
                                                  M_DEFAULT_IODEVICE_WARN_SLOP_US,
                                                  M_DEFAULT_IODEVICE_FAIL_SLOP_US,
                                                  M_MISSED_EXECUTION_CATCH_UP);
        
        MWTime current = clock->getCurrentTimeUS();
        if (VERBOSE_IO_DEVICE >= 2) {
            mprintf("LabJackU6Device:  schedule endPulse callback at %lld us (%lld)", current, clock->getCurrentTimeUS());
        }
        highTimeUS = current;
    }
    
}

// set the DO low for Laser

void LabJackU6Device::laserDOLow() {
    
    shared_ptr <Clock> clock = Clock::instance();
    MWTime current = clock->getCurrentTimeUS();
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
    if (ljHandle == NULL) {
        return;
    }
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: pulseDOLow at %lld us (pulse %lld us long)", current, current - highTimeUS);
    }
    if (!ljU6WriteDO(ljHandle, LJU6_LASERTRIGGER_CIO, 0) == 1) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing digital output low; device likely to not work from here on");
    }
    if (optic_device->getValue().getString() ==  "led") {
        if (ljU6WriteLaser(ljHandle, 0) == false) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing digital output low; device likely to not work from here on");
        }
    }
    // set laserDuration variable low
    laserDuration->setValue(Datum((long)0));
    
}

void LabJackU6Device::leverSolenoidDO(bool state) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
    
    if (ljU6WriteDO(ljHandle, LJU6_LEVER1SOLENOID_CIO, state) != true) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing lever 1 solenoid state; device likely to be broken (state %d)", state);
    }
}

void LabJackU6Device::laserDO(double Power) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
    
    LabJackU6Device::ljU6WriteLaser(ljHandle, Power);
    
}

void LabJackU6Device::laserDO2(bool state) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
    
    if (ljU6WriteDO(ljHandle, LJU6_LASERTRIGGER_CIO, state) != true) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing laser trigger state; device likely to be broken (state %d)", state);
    }
    
}


bool LabJackU6Device::ledDo2(bool &cameraState, bool &cameraState2){
    
    shared_ptr <Clock> clock = Clock::instance();
    
    // Takes and releases driver lock
    boost::mutex::scoped_lock lock(ljU6DriverLock);
    
    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "counter value: %lld", counter->getValue().getInteger());
    
    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "camera state MIO2: %d and last camera state: %d", cameraState, lastCameraState);
    
    // calculate led index by counter value and led sequence size
    //int led_index = (counter->getValue().getInteger()) % (led_seq->getValue().getNElements());
    // Cannot rely on counter signal from Camera, so set up our own frame counter
    
    
    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "LED Port is  %d", led_port);
    
    if ( (cameraState != lastCameraState && cameraState > 0 && ledCount==0) || (cameraState != lastCameraState && cameraState > 0 && cameraState2 > 0 ) ) {
        
        int led_index = (ledCount) % (led_seq->getValue().getNElements());
        
        int led_port = int(led_seq->getValue().getElement(led_index));  // get led # to match labjack port
        
        lastCameraState = 1;
        
        if (led_port != 0) {
            
            if (led_port == 1) {
                if (ljU6WriteDO(ljHandle, LJU6_LED1_FIO, 1) != true) {
                    merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state high; device likely to be broken", led_port);
                }
                led1_status-> setValue(true);
            }
            if (led_port == 2) {
                if (ljU6WriteDO(ljHandle, LJU6_LED2_FIO, 1) != true) {
                    merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state high; device likely to be broken", led_port);
                }
                led2_status-> setValue(true);
            }
        }
        // to get excution time in ms and delay could be <=1ms
        lastLEDonTimeUS = clock->getCurrentTimeUS();
        
        //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "time for led on: %ld", test_time);
        
                    //if (ljU6WriteDO(ljHandle, LJU6_LED2_FIO, 0) != true) {
            //    merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state high; device likely to be broken", led_port);
            //}
        
                    //if (ljU6WriteDO(ljHandle, LJU6_LED1_FIO, 0) != true) {
            //    merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state high; device likely to be broken", led_port);
            //}
        
        
        //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "LED status: 1--%d, 2--%d", led1_status->getValue().getBool(), led2_status->getValue().getBool());
        
    }
    if ( ((clock->getCurrentTimeUS() - lastLEDonTimeUS + 600) >= (LED_duration->getValue().getFloat())*1000) && lastCameraState == 1) {
        
        lastCameraState = 0;
        
        int led_index = (ledCount) % (led_seq->getValue().getNElements());
        
        int led_port = int(led_seq->getValue().getElement(led_index));  // get led # to match labjack port

        ledCount++;
        
        //if (ljU6WriteDO(ljHandle, led_port+1, 0) != true) {
        //    merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state low; device likely to be broken", led_port);
        //}
        
        //long test_time = getTickCount();
        //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "time for led off: %ld", test_time);
        
        //if (ljU6WriteDO(ljHandle, LJU6_LED1_FIO, 0) != true) {
        //    merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state high; device likely to be broken", LJU6_LED1_FIO);
        //}
        
        //if (ljU6WriteDO(ljHandle, LJU6_LED2_FIO, 0) != true) {
        //    merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state high; device likely to be broken", LJU6_LED2_FIO);
        //}
        /*
        if (led1_status->getValue().getBool() == true) {
            led1_status-> setValue(false);
            if (ljU6WriteDO(ljHandle, LJU6_LED1_FIO, 0) != true) {
                merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state high; device likely to be broken", LJU6_LED1_FIO);
            }
        }
        
        if (led2_status->getValue().getBool() == true) {
            led2_status-> setValue(false);
            if (ljU6WriteDO(ljHandle, LJU6_LED2_FIO, 0) != true) {
                merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state high; device likely to be broken", LJU6_LED2_FIO);
            }
        }
        */
        if (led_port != 0) {
            
            if (led_port == 1) {
                if (ljU6WriteDO(ljHandle, LJU6_LED1_FIO, 0) != true) {
                    merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state high; device likely to be broken", led_port);
                }
                led1_status-> setValue(false);
            }
            if (led_port == 2) {
                if (ljU6WriteDO(ljHandle, LJU6_LED2_FIO, 0) != true) {
                    merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing led %d state high; device likely to be broken", led_port);
                }
                led2_status-> setValue(false);
            }
        }
        //led1_status-> setValue(false);
        
        //led2_status-> setValue(false);
        
        
        //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "LED status: 1--%d, 2--%d", led1_status->getValue().getBool(), led2_status->getValue().getBool());
    }
    
    return true;
    
}


void LabJackU6Device::strobedDigitalWordDO(unsigned int digWord) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
    
    LabJackU6Device::ljU6WriteStrobedWord(ljHandle, digWord); // error checking done inside here; will call merror
    
}


bool LabJackU6Device::readLeverDI(bool *outLever1, bool *cameraState, bool *cameraState2)
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
    //lever1State = (cioState >> (LJU6_LEVER1_CIO - LJU6_CIO_OFFSET)) & 0x01;
    lever1State = (fioState >> LJU6_LEVER1_FIO) & 0x01;
    
    // software debouncing
    debounce_bit(&lever1State, &lastLever1State, &lastLever1TransitionTimeUS, clock);
    
    *outLever1 = lever1State;
    
    // if camera is on
    *cameraState = (cioState >> (LJU6_QTRIGGER_CIO - LJU6_CIO_OFFSET) ) & 0x01;
    
    *cameraState2 = (fioState >> 6) & 0x01;
    
    // to reset "trial" counts (how many times functions are called)
    if (trial > 10000) {
        trial = 1;
    } else {
        trial++;
    }
    
    return(1);
}

/*******************************************************************/
// Ziye - I think debounce here would not update state if time elapsed is short than required read time
// but actually labjack read time is ~ 1ms short enough.
void debounce_bit(unsigned int *thisState, unsigned int *lastState, MWTime *lastTransitionTimeUS, shared_ptr <Clock> clock) {
    // software debouncing
    if (*thisState != *lastState) {
        if (clock->getCurrentTimeUS() - *lastTransitionTimeUS < kDIDeadtimeUS) {
            *thisState = *lastState;				// discard changes during deadtime
            //mwarning(M_IODEVICE_MESSAGE_DOMAIN,
            //         "LabJackU6Device: readLeverDI, debounce rejecting new read (last %lld now %lld, diff %lld)",
            //         *lastTransitionTimeUS,
            //         clock->getCurrentTimeUS(),
            //         clock->getCurrentTimeUS() - *lastTransitionTimeUS);
        }
        *lastState = *thisState;					// record and report the transition
        *lastTransitionTimeUS = clock->getCurrentTimeUS();
    }
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
    
    if (!ljU6WriteLaser(ljHandle, 0) == 1)
        return false; // merror is done in ljU6WriteDO
    this->tTrialLaserPowerMw->setValue(Datum(M_FLOAT, 0));
    
    if (!ljU6WriteDO(ljHandle, LJU6_LASERTRIGGER_CIO, 0) == 1)
        return false; // merror is done in ljU6WriteDO
    this->laserTrigger->setValue(Datum(M_BOOLEAN, 0));
    
    if (!ljU6WriteDO(ljHandle, LJU6_STROBE_CIO, 0) == 1)
        return false; // merror is done in ljU6WriteDO
    this->strobedDigitalWord->setValue(Datum(M_INTEGER, 0));
    
    if (!ljU6WriteDO(ljHandle, LJU6_LED1_FIO, 0) == 1)
        return false; // merror is done in ljU6WriteDO
    this->led1_status->setValue(Datum(M_BOOLEAN,0));
    this->puffDuration->setValue(Datum(M_INTEGER, 0));
    
    if (!ljU6WriteDO(ljHandle, LJU6_LED2_FIO, 0) == 1)
        return false; // merror is done in ljU6WriteDO
    this->led2_status->setValue(Datum(M_BOOLEAN,0));
    
    //mprintf("Initialize()\n");
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
        //mprintf("setupU6PortsAndRestartIfDead():reset ports\n");
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
        
        // Redo port setup x times until it is ready
        //while (!ljU6ConfigPorts(ljHandle)) {
        //    mprintf("Reconfigure Labjack......");
        //}
        
        // Redo port setup
        if (!ljU6ConfigPorts(ljHandle)) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Error: configuring U6 after restart, U6 will not work now.  Check for patched version of libusb with reenumerate call.\n");
            return false;  // no cleanup needed
        }
    }
    //mprintf("setupU6PortsAndRestartIfDead()\n");
    return true;
}

bool LabJackU6Device::startup() {
    
    // Do nothing right now
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: startup");
    }
    //mprintf("startup()\n");
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
    
    trial=0;
    
    // Test if spelling error in optic_device
    //if (optic_device->getValue().getString() != "led" && optic_device->getValue().getString() != "laserblue")       {
    //    merror(M_IODEVICE_MESSAGE_DOMAIN, "Optic Device Name Error and Exit now.");
    //    return false;
    //}
    
    
        voltage.clear();                       // clear voltage vector
        pmw.clear();                           // clear pmw vector
    
        if (loadLEDTable(voltage, pmw) != 0) { // Load calibration table into array
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Error loading Calibration table");
            return false;
        }
    
    QBinValue.clear();
    lastBinQuadratureValue=0;
    
    //debug read in vector
    //mprintf("voltage[0] is: %g", voltage[0]);
    
    shared_ptr<LabJackU6Device> this_one = shared_from_this();
    pollScheduleNode = scheduler->scheduleUS(std::string(FILELINE ": ") + getTag(),
                                             (MWTime)0,
                                             LJU6_DITASK_UPDATE_PERIOD_US,
                                             M_REPEAT_INDEFINITELY,
                                             boost::bind(update_lever, weak_ptr<LabJackU6Device>(this_one)),
                                             M_DEFAULT_IODEVICE_PRIORITY,
                                             LJU6_DITASK_WARN_SLOP_US,
                                             LJU6_DITASK_FAIL_SLOP_US,
                                             M_MISSED_EXECUTION_CATCH_UP);
    
    //ledDo2();
    
    //schedule_nodes.push_back(pollScheduleNode);
    //	schedule_nodes_lock.unlock();		// Seems to be no longer supported in MWorks
    //mprintf("startDeviceIO()");
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
    
    // turn off laser/led and optic switch
    laserDO(0);
    laserDO2(false);
    
    if (!ljU6WriteDO(ljHandle, LJU6_LED1_FIO, 0) == 1)
        return false; // merror is done in ljU6WriteDO
    
    if (!ljU6WriteDO(ljHandle, LJU6_LED2_FIO, 0) == 1)
        return false; // merror is done in ljU6WriteDO

    this->led1_status->setValue(false);
    this->led2_status->setValue(false);

    return true;
}


void LabJackU6Device::variableSetup() {
    
    //	weak_ptr<LabJackU6Device> weak_self_ref(getSelfPtr<LabJackU6Device>());
    weak_ptr<LabJackU6Device> weak_self_ref(component_shared_from_this<LabJackU6Device>());
    
    // Reward
    shared_ptr<Variable> doReward = this->pulseOn;
    shared_ptr<VariableNotification> notif(new LabJackU6DeviceOutputNotification(weak_self_ref));
    doReward->addNotification(notif);
    
    // lever1Solenoid
    shared_ptr<Variable> doL1S = this->lever1Solenoid;
    shared_ptr<VariableNotification> notif2(new LabJackU6DeviceL1SNotification(weak_self_ref));
    doL1S->addNotification(notif2);
    
    // laserTrigger for LED
    shared_ptr<Variable> doLT = this->tTrialLaserPowerMw;
    shared_ptr<VariableNotification> notif3(new LabJackU6DeviceLTNotification(weak_self_ref));
    doLT->addNotification(notif3);
    
    // laserTrigger for Laser
    shared_ptr<Variable> doLT2 = this->laserTrigger;
    shared_ptr<VariableNotification> notif3a(new LabJackU6DeviceLT2Notification(weak_self_ref));
    doLT2->addNotification(notif3a);
    
    // laserTrigger for do2LED
    //shared_ptr<Variable> doLED2 = this->do2led;
    //shared_ptr<VariableNotification> notif3b(new LabJackU6DeviceLED2Notification(weak_self_ref));
    //doLED2->addNotification(notif3b);

    // strobedDigitalWord
    shared_ptr<Variable> doSDW = this->strobedDigitalWord;
    shared_ptr<VariableNotification> notif4(new LabJackU6DeviceSDWNotification(weak_self_ref));
    doSDW->addNotification(notif4);
    
    //laser duration
    shared_ptr<Variable> doLaserDuration = this->laserDuration;
    shared_ptr<VariableNotification> notif5(new LabJackU6DeviceLaserDurNotification(weak_self_ref));
    doLaserDuration->addNotification(notif5);
    
    // Puff
    shared_ptr<Variable> doPuff = this->puffDuration;
    shared_ptr<VariableNotification> notif6(new LabJackU6DevicePuffNotification(weak_self_ref));
    doPuff->addNotification(notif6);

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
    
    long aEnableTimers[] = { 1, 1, 0, 0 };  // Use Timer0-3
    long aEnableCounters[] = { 1, 1 };      // Use Counter0 and Counter1
    long aTimerModes[] = { 8, 8, 0, 0 };  // Use quadrature input mode for both timers and firmware counter
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
    
    if (trial == 0) {
        sendDataBuff[1] = 42;       //IOType is Timer0
        sendDataBuff[2] = 1;        //  - Don't reset timer
        sendDataBuff[3] = 0;        //  - Value LSB (ignored)
        sendDataBuff[4] = 0;        //  - Value MSB (ignored)
        
        sendDataBuff[5] = 54;       //IOType is Counter0
        sendDataBuff[6] = 1;        //  - Reset counter
        sendDataBuff[7] = 55;       //IOType is Counter1
        sendDataBuff[8] = 1;        //  - Reset counter
        /*
        sendDataBuff[5] = 46;       //IOType is Timer2
        sendDataBuff[6] = 1;        //  - Don't reset timer
        sendDataBuff[7] = 0;        //  - Value LSB (ignored)
        sendDataBuff[8] = 0;        //  - Value MSB (ignored)
        
        sendDataBuff[9] = 48;       //IOType is Timer3
        sendDataBuff[10] = 1;        //  - Don't reset timer
        sendDataBuff[11] = 0;        //  - Value LSB (ignored)
        sendDataBuff[12] = 0;        //  - Value MSB (ignored)
        
        sendDataBuff[13] = 54;       //IOType is Counter0
        sendDataBuff[14] = 1;        //  - Reset counter
        sendDataBuff[15] = 55;       //IOType is Counter1
        sendDataBuff[16] = 1;        //  - Reset counter
         */
        
    }
    else {
        sendDataBuff[1] = 42;       //IOType is Timer0
        sendDataBuff[2] = 0;        //  - Don't reset timer
        sendDataBuff[3] = 0;        //  - Value LSB (ignored)
        sendDataBuff[4] = 0;        //  - Value MSB (ignored)
        
        sendDataBuff[5] = 54;       //IOType is Counter0
        sendDataBuff[6] = 0;        //  - Reset counter
        sendDataBuff[7] = 55;       //IOType is Counter1
        sendDataBuff[8] = 0;        //  - Reset counter

        /*
        sendDataBuff[5] = 46;       //IOType is Timer2
        sendDataBuff[6] = 0;        //  - Don't reset timer
        sendDataBuff[7] = 0;        //  - Value LSB (ignored)
        sendDataBuff[8] = 0;        //  - Value MSB (ignored)
        
        sendDataBuff[9] = 48;       //IOType is Timer3
        sendDataBuff[10] = 0;        //  - Don't reset timer
        sendDataBuff[11] = 0;        //  - Value LSB (ignored)
        sendDataBuff[12] = 0;        //  - Value MSB (ignored)
        
        sendDataBuff[13] = 54;       //IOType is Counter0
        sendDataBuff[14] = 0;        //  - Don't reset counter
        sendDataBuff[15] = 55;       //IOType is Counter1
        sendDataBuff[16] = 0;        //  - Don't reset counter
        */
    }
    uint8 Errorcode, ErrorFrame;
    
    if(ehFeedback(Handle, sendDataBuff, sizeof(sendDataBuff), &Errorcode, &ErrorFrame, recDataBuff, sizeof(recDataBuff)) < 0)
        return -1;
    if(Errorcode)
        return (long)Errorcode;
    
    *fioState = recDataBuff[0];
    *eioState = recDataBuff[1];
    *cioState = recDataBuff[2];
    
    shared_ptr <Clock> clock = Clock::instance();
    
    // debug
    //mprintf("FIO 0x%x EIO 0x%x CIO 0x%x", *fioState, *eioState, *cioState);
    // debug timer and counter
    //mprintf("*****************Output q1:%02x q2:%02x q3:%02x q4:%02x c1:%02x c2:%02x c3:%02x c4:%02x c1:%02x c2:%02x c3:%02x c4:%02x\n",
    //       recDataBuff[3], recDataBuff[4], recDataBuff[5], recDataBuff[6],
    //       recDataBuff[7], recDataBuff[8], recDataBuff[9], recDataBuff[10], recDataBuff[11],
    //      recDataBuff[12], recDataBuff[13], recDataBuff[14]);
    
    // Unpack timer value (i.e. quadrature)
    std::int32_t quadratureValue;
    
    /* another way to extract
     //quadratureValue = recDataBuff[3] + recDataBuff[4]*256 + recDataBuff[5]*65536 + recDataBuff[6]*16777216;
     */
    
    for (size_t i = 0; i < 4; i++) {
        // timer output has 4 bit and two channels have the same output
        ((uint8 *)(&quadratureValue))[i] = recDataBuff[3 + i];
    }
    quadratureValue = CFSwapInt32LittleToHost(quadratureValue);  // Convert to host byte order
    
    //mprintf("*****Quadrature = %d *******", quadratureValue);
    
    //quadratureValue = CFSwapInt32LittleToHost(quadratureValue);  // Convert to host byte order
    //mprintf("*****Quadrature = %d *******", quadratureValue);
    // Update quadrature variable (only if quadrature value has changed)
    //if (quadrature->getValue().getInteger() != quadratureValue) {
    
    //}
    
    if (doCB->getValue().getBool()==true && do_wheelspeed->getValue().getBool() == false) {
        quadrature->setValue(long(quadratureValue));   // to update quadrature reading more frequent
        if (checkrun->getValue().getBool())
            runningCriteria(checkrun->getValue().getBool());
        else {
            QTimeUS = clock->getCurrentTimeUS();
            QBinValue.clear();
            lastBinQuadratureValue = quadrature->getValue().getInteger();
        }
    } else if(doCB->getValue().getBool()==false && do_wheelspeed->getValue().getBool() == true) {
        if (quadrature->getValue().getInteger() != quadratureValue)
            quadrature->setValue(long(quadratureValue));
        if (trial == 0) {
            QTime2US = clock->getCurrentTimeUS();
            lastQuadratureValue = quadrature->getValue().getInteger();
        }
        calculateWheelSpeed();
    } else if (doCB->getValue().getBool()==true && do_wheelspeed->getValue().getBool() == true) {
        quadrature->setValue(long(quadratureValue));   // to update quadrature reading more frequent
        /*if (checkrun->getValue().getBool())
            runningCriteria(checkrun->getValue().getBool());
        else if (!checkrun->getValue().getBool() || start_CB_running->getValue().getBool() ){
            QTimeUS = clock->getCurrentTimeUS();
            QBinValue.clear();
            lastBinQuadratureValue = quadrature->getValue().getInteger();
        }
         */
        if (trial == 0) {
            QTime2US = clock->getCurrentTimeUS();
            lastQuadratureValue = quadrature->getValue().getInteger();
        }
        calculateWheelSpeed();
    } else {
        if (quadrature->getValue().getInteger() != quadratureValue)
            quadrature->setValue(long(quadratureValue));
    }
    
    // Unpack counter value
    uint32 counterValue[2];
    
    /* another way to do it
     //counterValue[0] = recDataBuff[7] + recDataBuff[8]*256 + recDataBuff[9]*65536 + recDataBuff[10]*16777216;
     //counterValue[1] = recDataBuff[11] + recDataBuff[12]*256 + recDataBuff[13]*65536 + recDataBuff[14]*16777216;
     */
    
    //mprintf("*****Counter0 = %d *******", counterValue[0]);
    //mprintf("*****Counter1 = %d *******", counterValue[1]);
    
    for (size_t j = 0; j < 2; j++) {
        for (size_t i = 0; i < 4; i++) {
            // each counter output has 4 bit
            // Hard-coded: counter value starts after quadrature
            ((uint8 *)(counterValue + j))[i] = recDataBuff[3 + 4*(j+1) + i];
        }
        //mprintf("*****Counter0 = %d *******", counterValue[0]);
        //mprintf("*****Counter1 = %d *******", counterValue[1]);
        counterValue[j] = CFSwapInt32LittleToHost(counterValue[j]);  // Convert to host byte order
    }
    
    //counterValue[0] = CFSwapInt32LittleToHost(counterValue[0]);
    //counterValue[1] = CFSwapInt32LittleToHost(counterValue[1]);
    //counterValue[2] = CFSwapInt32LittleToHost(counterValue[2]);
    //counterValue[3] = CFSwapInt32LittleToHost(counterValue[3]);
    
    // Update counter variables (only if counter value has changed)
    if (counter->getValue().getInteger() != counterValue[0]) {
        counter->setValue(long(counterValue[0]));
    }
    
    if (counter2->getValue().getInteger() != counterValue[1]) {
        counter2->setValue(long(counterValue[1]));
    }
    
    /*
    if (counter3->getValue().getInteger() != counterValue[2]) {
        counter3->setValue(long(counterValue[2]));
    }
    
    if (counter4->getValue().getInteger() != counterValue[3]) {
        counter4->setValue(long(counterValue[3]));
    }
    */
    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "counter value: %lld", counter->getValue().getInteger());
    
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

bool LabJackU6Device::ljU6WriteLaser(HANDLE Handle, double laserPower) {
    
    //int x_tam = TABLE_SIZE;
    //double xx[] = {laserPower};
    std::vector<double> xx;
    xx.push_back(laserPower);
    
    //int xx_tam = 1;          // Hard-coded for only one laserPower
    //double laserVol[]= {0};  // Hard-coded size
    //std::vector<double> laserVol;
    //laserVol.push_back(0);
    
    u6CalibrationInfo CalibInfo;          // get calibration struct
    
    if( getCalibrationInfo(ljHandle, &CalibInfo) < 0 ) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Error Calibrating LabJack U6.");
        return false;
    }
    
    if (laserPower == 0) {
        if( eDAC(Handle, &CalibInfo, LJU6_LASERPOWER_DAC, laserPower, 0, 0, 0) !=0 ) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: eDAC error");
            return false;
        } else {
            mprintf("*** Laser is turned off ***\n");
        }
    } else {
        std::vector<double> laserVol=interp1(pmw, voltage, xx);   //Get interpolated voltage
        
        //mprintf("Laser Voltage: %lg\n", laserVol[0]);
        
        if( eDAC(Handle, &CalibInfo, LJU6_LASERPOWER_DAC, laserVol[0], 0, 0, 0) !=0 ) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: eDAC error");
            return false;
        } else {
            mprintf("*** %gmW is received and %gV is sent ***\n", xx[0], laserVol[0]);
        }
    }
    
    return true;
}


int LabJackU6Device::findNearestNeighbourIndex( double value, const std::vector< double > &x )
{
    double dist = FLT_MAX;
    int idx = -1;
    for ( int i = 0; i < x.size(); ++i ) {
        double newDist = value - x[i];
        if ( newDist > 0 && newDist < dist ) {
            dist = newDist;
            idx = i;
        }
    }
    
    return idx;
}

vector<double> LabJackU6Device::interp1( const std::vector< double > &x, const std::vector< double > &y, const std::vector< double > &x_new )
{
    std::vector< double > y_new;
    y_new.reserve( x_new.size() );
    
    std::vector< double > dx, dy, slope, intercept;
    dx.reserve( x.size() );
    dy.reserve( x.size() );
    slope.reserve( x.size() );
    intercept.reserve( x.size() );
    for( int i = 0; i < x.size(); ++i ){
        if( i < x.size()-1 )
        {
            dx.push_back( x[i+1] - x[i] );
            dy.push_back( y[i+1] - y[i] );
            slope.push_back( dy[i] / dx[i] );
            intercept.push_back( y[i] - x[i] * slope[i] );
        }
        else
        {
            dx.push_back( dx[i-1] );
            dy.push_back( dy[i-1] );
            slope.push_back( slope[i-1] );
            intercept.push_back( intercept[i-1] );
        }
    }
    
    for ( int i = 0; i < x_new.size(); ++i )
    {
        int idx = findNearestNeighbourIndex( x_new[i], x );
        y_new.push_back( slope[idx] * x_new[i] + intercept[idx] );
        
    }
    return y_new;
}

int LabJackU6Device::loadLEDTable(std::vector<double> &voltage, std::vector<double> &pmw) {
    char const *inname;
    //FILE *infile;
    //char hostname[1024];
    
    //gethostname(hostname, 1024);
    //if (strcmp(optic_device->getValue().getString(), "led")==0) {

    if (optic_device->getValue().getString() == "led") {
        inname = "/Users/hullglick/Documents/Calibration_Table/led.txt";
    //} else if (strcmp(optic_device->getValue().getString(), "laserblue")==0){
    } else if (optic_device->getValue().getString() == "laserblue"){
        inname = "/Users/hullglick/Documents/Calibration_Table/laserblue.txt";
    //} else if (strcmp(optic_device->getValue().getString(), "lasergreen")==0){
    } else if (optic_device->getValue().getString() == "lasergreen"){
        inname = "/Users/hullglick/Documents/Calibration_Table/lasergreen.txt";
    }
    mprintf("Calibration file name is: %s\n",inname);
    
    std::ifstream ifs(inname);
    
    std::string line;

    while(std::getline(ifs, line)) // read one line from ifs
    {
        std::istringstream iss(line); // access line as a stream
        
        // we only need the first two columns
        double column1;
        double column2;
        
        iss >> column1 >> column2; // read in column 1 & 2
        
        voltage.push_back(column1);
        pmw.push_back(column2);
        
    }
    
    return 0;
}

void LabJackU6Device::runningCriteria(bool checkRunning) {
    // get the clock
    shared_ptr <Clock> clock = Clock::instance();
    int Qbin_sum = 0;
    int Qindex = 0;
    // if current time is longer than required time
    if (clock->getCurrentTimeUS() -  QTimeUS >= Qbin_timeUS->getValue().getInteger()) {
        
        QTimeUS = clock->getCurrentTimeUS();
        
        //mprintf("The bin time is %lld.", Qbin_timeUS->getValue().getInteger());
        
        // if the quadrature reading has exceeded the running criteria
        if ( (quadrature->getValue().getInteger() - lastBinQuadratureValue) >= Qpulse_criteria->getValue().getInteger()) {
            lastBinQuadratureValue = quadrature->getValue().getInteger();
            QBinValue.push_back(1);
            //mprintf("exceeding running criteria");
        } else {
            QBinValue.push_back(0);
            //mprintf("Not exceeding running criteria");
        }
        
        if (QBinValue.size() > Qbin_size->getValue().getInteger()) {
            QBinValue.erase(QBinValue.begin());
            //mprintf("test=====");
            for (int n:QBinValue) {
                Qbin_sum += n;
                ++Qindex;
                mprintf("The running status at bin %d is %d", Qindex, n);
            }
        } else {
            for (int n:QBinValue) {
                Qbin_sum += n;
                ++Qindex;
                mprintf("The running status at bin %d is %d", Qindex, n);
            }
        }
        //mprintf("Qbin_sum is %d and the QbinSize is %lu", Qbin_sum, QBinValue.size());
        //mprintf("The running criteria is %lld.", Qpulse_criteria->getValue().getInteger());
        if (Qbin_sum >= running_criteria->getValue().getInteger()) {
            start_CB_running->setValue(true);
            start_CB_still->setValue(false);
            QBinValue.erase(QBinValue.begin(),QBinValue.begin()+QBinValue.size());
            //mprintf("Qbin_sum is %d and the QbinSize is %lu", Qbin_sum, QBinValue.size());
            //mprintf("The stimulus should start now and the current time is %lld.", clock->getCurrentTimeUS());
        } else if (Qbin_sum == 0 && QBinValue.size() >= (still_duration->getValue().getInteger()*1000/Qbin_timeUS->getValue().getInteger())){
            // if still longer than required also make sure Qbin_timeUs is same as wheelspeed interval
            start_CB_still->setValue(true);
            start_CB_running->setValue(false);
        } else
            start_CB_still->setValue(false);
            start_CB_running->setValue(false);

    }
    
}

void LabJackU6Device::calculateWheelSpeed() {
    
    double speed;
    long binSize;
    int speed_sum = 0;
    
    shared_ptr <Clock> clock = Clock::instance();
    
    if (clock->getCurrentTimeUS() -  QTime2US >= ws_durationUS->getValue().getInteger()) {
        
        QTime2US = clock->getCurrentTimeUS();
        speed = (quadrature->getValue().getInteger() - lastQuadratureValue)*1000000/ws_durationUS->getValue().getInteger();
        
        wheel_speed->setValue(speed);
        if (doCB->getValue().getBool()==true) {
            WheelSpeedArray.push_back(speed);
            binSize = (still_duration->getValue().getInteger()*1000/ws_durationUS->getValue().getInteger());
            if (speed >= running_criteria->getValue().getInteger()) {
                start_CB_running->setValue(true);
            }
            
            if (speed < running_criteria->getValue().getInteger() && WheelSpeedArray.size() >= binSize) {
                std::vector<int> temp_speed(WheelSpeedArray.end() - binSize + 1, WheelSpeedArray.end());
                for (int n:temp_speed) {
                    speed_sum += n;
                }
                if (speed_sum == 0) {
                    start_CB_still->setValue(true);
                }
                WheelSpeedArray.clear();
            }
        }
        //mprintf("*****Quadrature = %d *******", quadrature->getValue().getInteger());
        //mprintf("*****LastQuadrature = %d *******", lastBinQuadratureValue);
        //mprintf("*****wheelSpeed = %f *******", speed);
        lastQuadratureValue = quadrature->getValue().getInteger();
    }
    
    
}

END_NAMESPACE_MW

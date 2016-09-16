/*
 *  LabJack U6 Plugin for MWorks
 *
 *  Created by Mark Histed on 4/21/2010
 *    (based on Nidaq plugin code by Jon Hendry and John Maunsell)
 *
 *  08-26-2015 Ziye -- changed LJU6 Ports Macros
 *                     removed Lever2 and Lever2Solenoid
 */

#ifndef	_LJU6_DEVICE_H_
#define _LJU6_DEVICE_H_


#include <boost/noncopyable.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <cfloat>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <libusb-1.0/libusb.h>
#include "u6.h"
#include "labjackusb.h"

#undef VERBOSE_IO_DEVICE
#define VERBOSE_IO_DEVICE 0  // verbosity level is 0-2, 2 is maximum

#define LJU6_DITASK_UPDATE_PERIOD_US 1000    // "sampling time" in MWorks, not going faster than this?
#define LJU6_DITASK_WARN_SLOP_US     500000000
#define LJU6_DITASK_FAIL_SLOP_US     500000000

// Strobed_word output: Use a 8-bit word; EIO0-7, all encoded below
#define LJU6_REWARD_FIO         0
#define LJU6_LEVER1_FIO         1
#define LJU6_LED1_FIO           2
#define LJU6_PUFF_FIO           2
#define LJU6_LED2_FIO           3
#define LJU6_TCPIN_OFFSET       4 // Timer offset pin

#define LJU6_CIO_OFFSET         16

#define LJU6_LEVER1SOLENOID_CIO 16
#define LJU6_LASERTRIGGER_CIO   17
#define LJU6_QTRIGGER_CIO       18
#define LJU6_STROBE_CIO         19 // not using it now

#define LJU6_LASERPOWER_DAC     0 // DAC0 as output

BEGIN_NAMESPACE_MW


class LabJackU6DeviceOutputNotification;

class LabJackU6Device : public IODevice, boost::noncopyable {
    
protected:
	
	bool						connected;
    
	MWTime						lastLever1TransitionTimeUS;
    MWTime                      QTimeUS;
    MWTime                      QTime2US;
    MWTime                      lastLEDonTimeUS;
	int lastLever1Value;
    int lastCameraState;
    int ledCount;
    long lastBinQuadratureValue;
    long lastQuadratureValue;
    
    
    unsigned int trial;
    
	boost::shared_ptr <Scheduler> scheduler;
	shared_ptr<ScheduleTask>	pulseScheduleNode;
	shared_ptr<ScheduleTask>	pollScheduleNode;
    shared_ptr<ScheduleTask>	laserScheduleNode;
	boost::mutex				pulseScheduleNodeLock;
	boost::mutex				pollScheduleNodeLock;
    boost::mutex				laserScheduleNodeLock;
	boost::mutex				ljU6DriverLock;
	MWTime						highTimeUS;  // Used to compute length of scheduled high/low pulses
	
	HANDLE                      ljHandle;   // LabJack device handle
    
	boost::shared_ptr <Variable> pulseDuration;
	boost::shared_ptr <Variable> pulseOn;
	boost::shared_ptr <Variable> lever1;
    boost::shared_ptr <Variable> lever1Solenoid;
	boost::shared_ptr <Variable> tTrialLaserPowerMw;
    boost::shared_ptr <Variable> laserTrigger;
    boost::shared_ptr <Variable> laserDuration;
	boost::shared_ptr <Variable> strobedDigitalWord;
	boost::shared_ptr <Variable> counter;
	boost::shared_ptr <Variable> counter2;
    //boost::shared_ptr <Variable> counter3;
	//boost::shared_ptr <Variable> counter4;
	boost::shared_ptr <Variable> quadrature;
    boost::shared_ptr <Variable> optic_device;
    boost::shared_ptr <Variable> do2led;
    boost::shared_ptr <Variable> led_seq;
    boost::shared_ptr <Variable> led1_status;
    boost::shared_ptr <Variable> led2_status;
    boost::shared_ptr <Variable> LED_duration;
    boost::shared_ptr <Variable> Qbin_size;
    boost::shared_ptr <Variable> Qbin_timeUS;
    boost::shared_ptr <Variable> doCB;
    boost::shared_ptr <Variable> start_CB;
    boost::shared_ptr <Variable> start_CB_running;
    boost::shared_ptr <Variable> running_criteria;
    boost::shared_ptr <Variable> Qpulse_criteria;
    boost::shared_ptr <Variable> checkrun;
    boost::shared_ptr <Variable> do_wheelspeed;
    boost::shared_ptr <Variable> ws_durationUS;
    boost::shared_ptr <Variable> wheel_speed;
    boost::shared_ptr <Variable> puffDuration;
    
	//MWTime update_period;  MH this is now hardcoded, users should not change this
	
	bool active;
	boost::mutex active_mutex;
	bool deviceIOrunning;
	
    std::vector<double> voltage;
    std::vector<double> pmw;
    
    std::vector<int> QBinValue;
    
	// raw hardware functions
	bool ljU6ConfigPorts(HANDLE Handle);
	bool ljU6WriteDO(HANDLE Handle, long Channel, long State);
    bool ljU6WriteLaser(HANDLE Handle, double laserPower);
	bool ljU6WriteStrobedWord(HANDLE Handle, unsigned int inWord);
	long ljU6ReadPorts(HANDLE Handle, unsigned int *fioState, unsigned int *eioState, unsigned int *cioState);
    
public:
    static const std::string PULSE_DURATION;
    static const std::string PULSE_ON;
    static const std::string LEVER1;
    static const std::string LEVER1_SOLENOID;
    static const std::string TRIAL_LASER_POWERMW;
    static const std::string LASER_TRIGGER;
    static const std::string LASER_DURATION;
    static const std::string STROBED_DIGITAL_WORD;
    static const std::string COUNTER;
    static const std::string COUNTER2;
    //static const std::string COUNTER3;
    //static const std::string COUNTER4;
    static const std::string QUADRATURE;
    static const std::string OPTIC_DEVICE;
    static const std::string DO2LED;
    static const std::string LED_SEQ;
    static const std::string LED1_STATUS;
    static const std::string LED2_STATUS;
    static const std::string LED_DURATION;
    static const std::string QBIN_SIZE;
    static const std::string QBIN_TIMEUS;
    static const std::string DOCB;
    static const std::string START_CB;
    static const std::string START_CB_RUNNING;
    static const std::string RUNNING_CRITERIA;
    static const std::string QPULSE_CRITERIA;
    static const std::string CHECKRUN;
    static const std::string DO_WHEELSPEED;
    static const std::string WS_DURATIONUS;
    static const std::string WHEEL_SPEED;
    static const std::string PUFF_DURATION;
    
    static void describeComponent(ComponentInfo &info);
	
    explicit LabJackU6Device(const ParameterValueMap &parameters);
	~LabJackU6Device();
	
	virtual bool startup();
	virtual bool shutdown();
	//       virtual bool attachPhysicalDevice();		DEPRECATED IN 0.4.4
	virtual bool initialize();
	virtual bool startDeviceIO();
	virtual bool stopDeviceIO();
	
	virtual bool pollAllDI();
	void detachPhysicalDevice();
	void variableSetup();
	bool setupU6PortsAndRestartIfDead();
    
	bool readLeverDI(bool *outLever1, bool *cameraState, bool *cameraState2);
	void pulseDOHigh(int pulseLengthUS);
	void pulseDOLow();
    void puffDOHigh(int puffLengthMS);
    void puffDOLow();
    void laserDOHigh(int laserLengthMS);
    void laserDOLow();
	void leverSolenoidDO(bool state);
	void laserDO(double laserPower);  // LED modulation
    void laserDO2(bool state);   //optic switch
    bool ledDo2(bool &cameraState, bool &cameraState2);     //control two leds for wide field
	void strobedDigitalWordDO(unsigned int digWord);
    
    
    // two functions to do linear interpolation on LED power
    int  loadLEDTable(std::vector<double> &voltage, std::vector<double> &pmw );
    int findNearestNeighbourIndex( double value, const std::vector< double > &x );
    std::vector<double> interp1( const std::vector< double > &x, const std::vector< double > &y, const std::vector< double > &x_new );
    
    void runningCriteria(bool checkRunning);
    
    void calculateWheelSpeed();
    
	virtual void dispense(Datum data){
		if(getActive()){
			bool doReward = (bool)data;
			
			// Bring DO high for pulseDuration
			if (doReward) {
				this->pulseDOHigh(pulseDuration->getValue());
			}
		}
	}
    
    virtual void dispenseAir(Datum data){
        if(getActive()){
            bool doPuff = (bool)data;
            
            // Bring DO high for pulseDuration
            if (doPuff) {
                this->puffDOHigh(puffDuration->getValue());
            }
        }
    }
    
    virtual void dispenseLaser(Datum data){
        if(getActive()){
            bool doLaserDuration = (bool)data;
            
            // Bring DO high for pulseDuration
            if (doLaserDuration) {
                this->laserDOHigh(laserDuration->getValue());
            }
        }
    }
    
	virtual void setLever1Solenoid(Datum data) {
        //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "set 1");
		if (getActive()) {
			bool lever1SolenoidState = (bool)data;
			this->leverSolenoidDO(lever1SolenoidState);
		}
	}
    
	virtual void setLaserTrigger(Datum data) {
		if (getActive()) {
			double Power = (double)data;
			this->laserDO(Power);
		}
	}
    
    virtual void setLaserTrigger2(Datum data) {
        if (getActive()) {
            bool laserState = (bool)data;
            this->laserDO2(laserState);
        }
    }
    
            
	virtual void setStrobedDigitalWord(Datum data) {
		if (getActive()) {
			unsigned int digWord = (int)data;
			this->strobedDigitalWordDO(digWord);
		} else {
			// silent: we need to doublecheck the active/deviceIORunning states and make sure they're doing the right thing.
			// here, we set the value of this variable to zero on init; active/deviceIORunning is true only
			//   during a trial.  I think the right thing to do is silently drop the output function.
            //merror(M_IODEVICE_MESSAGE_DOMAIN, "LJU6: not running; not writing to strobed port (data was 0x%02x)", (int)data);
        }
    }
	
	virtual void setActive(bool _active){
		boost::mutex::scoped_lock active_lock(active_mutex);
		active = _active;
	}
	
	virtual bool getActive(){
		boost::mutex::scoped_lock active_lock(active_mutex);
		bool is_active = active;
		return is_active;
	}
	
	shared_ptr<LabJackU6Device> shared_from_this() { return boost::static_pointer_cast<LabJackU6Device>(IODevice::shared_from_this()); }
	
};


class LabJackU6DeviceOutputNotification : public VariableNotification {
    /* reward variable */
protected:
    weak_ptr<LabJackU6Device> daq;
	
public:
    LabJackU6DeviceOutputNotification(weak_ptr<LabJackU6Device> _daq){
        daq = _daq;
    }
	
    virtual void notify(const Datum& data, MWTime timeUS){
        shared_ptr<LabJackU6Device> shared_daq(daq);
        shared_daq->dispense(data);
    }
};

class LabJackU6DevicePuffNotification : public VariableNotification {
    /* reward variable */
protected:
    weak_ptr<LabJackU6Device> daq;
    
public:
    LabJackU6DevicePuffNotification(weak_ptr<LabJackU6Device> _daq){
        daq = _daq;
    }
    
    virtual void notify(const Datum& data, MWTime timeUS){
        shared_ptr<LabJackU6Device> shared_daq(daq);
        shared_daq->dispenseAir(data);
    }
};

class LabJackU6DeviceLaserDurNotification : public VariableNotification {
    /* laser variable */
protected:
    weak_ptr<LabJackU6Device> daq;
    
public:
    LabJackU6DeviceLaserDurNotification(weak_ptr<LabJackU6Device> _daq){
        daq = _daq;
    }
    
    virtual void notify(const Datum& data, MWTime timeUS){
        shared_ptr<LabJackU6Device> shared_daq(daq);
        shared_daq->dispenseLaser(data);
    }
};

class LabJackU6DeviceL1SNotification : public VariableNotification {
    
protected:
	weak_ptr<LabJackU6Device> daq;
public:
	LabJackU6DeviceL1SNotification(weak_ptr<LabJackU6Device> _daq){
		daq = _daq;
	}
	virtual void notify(const Datum& data, MWTime timeUS){
		shared_ptr<LabJackU6Device> shared_daq(daq);
		shared_daq->setLever1Solenoid(data);
	}
};



class LabJackU6DeviceLTNotification : public VariableNotification {
    
protected:
    weak_ptr<LabJackU6Device> daq;
public:
    LabJackU6DeviceLTNotification(weak_ptr<LabJackU6Device> _daq){
        daq = _daq;
    }
    virtual void notify(const Datum& data, MWTime timeUS){
        shared_ptr<LabJackU6Device> shared_daq(daq);
        shared_daq->setLaserTrigger(data);
    }
};

class LabJackU6DeviceLT2Notification : public VariableNotification {
    
protected:
    weak_ptr<LabJackU6Device> daq;
public:
    LabJackU6DeviceLT2Notification(weak_ptr<LabJackU6Device> _daq){
        daq = _daq;
    }
    virtual void notify(const Datum& data, MWTime timeUS){
        shared_ptr<LabJackU6Device> shared_daq(daq);
        shared_daq->setLaserTrigger2(data);
    }
};


class LabJackU6DeviceSDWNotification : public VariableNotification {
    
protected:
    weak_ptr<LabJackU6Device> daq;
public:
    LabJackU6DeviceSDWNotification(weak_ptr<LabJackU6Device> _daq){
        daq = _daq;
    }
    virtual void notify(const Datum& data, MWTime timeUS){
        shared_ptr<LabJackU6Device> shared_daq(daq);
        shared_daq->setStrobedDigitalWord(data);
    }
};



END_NAMESPACE_MW


#endif







#! /usr/bin/python
import u6
import pylab as pl
from pylab import plot, show
import numpy as np
import threading
from threading import Thread
import time
import os
import sys
#sys.path.append("/usr/share/adafruit/webide/repositories/Adafruit-Raspberry-Pi-Python-Code/Adafruit_MCP4725")
#from Adafruit_MCP4725 import MCP4725
#sys.path.append("/usr/share/adafruit/webide/repositories/Adafruit-Raspberry-Pi-Python-Code/Adafruit_ADS1x15")
#from Adafruit_ADS1x15 import ADS1x15    

startV = 0
stopV = 5
nVPts = 51
rangeValue = 28 # max range of power meter (mW)

d = u6.U6() # open LabJack U6

d.getCalibrationData()

# Configuration MCP4725 output channel
#dac = MCP4725(0x62)
#voltageSource = 5.0
#maxVal = 4096

# Configure the ADS1115 to receive parameters from AO
#ADS1115 = 0x01
#adc = ADS1x15(ic=ADS1115)


# setup plotting
#pl.ion()
#pH, = pl.plot(0,0, 'b.-')
#pl.xlabel('Command Voltage (V)')
#pl.ylabel('Calculated LED Output (mW)')

# loop and collect data
vLevels = np.linspace(startV, stopV, nVPts)
respV = vLevels*np.nan
for (tVNum,tV) in enumerate(vLevels):
    
    # set ao value
    volts = (tV*np.ones(1))
    print(volts)
    #voltsInBits = [int((volts/voltageSource)*maxVal)]
    for val in volts:
        #dac.setVoltage(val) # np.ones(2) to np.ones(100)
        DAC0_value = d.voltagetoDACBits(val, dacNumber = 0, is16Bits = False)
        d.getFeedback(u6.DAC0_8(DAC0_value))
    #time.sleep(2)
    
    # read resulting analog
    #tVolts = adc.readADCSingleEnded(0,4096, 250)/1000
    ainValue = d.getAIN(0)
    tDataCol = (ainValue)#*rangeValue)/2
    #tDataCol = tVolts
    print tDataCol
    
    responseData = tDataCol
    #responseData = np.array([x for x in tDataCol])
    #load into a vector - discard pts at start for stability
    respV[tVNum] = np.mean(responseData)
    #respV = responseData

    noNanIx = np.logical_not(np.isnan(respV))

    #pH.set_xdata(vLevels[noNanIx])
    #pH.set_ydata(np.array(respV[noNanIx]))
    #pl.gca().relim()
    #pl.gca().autoscale_view(1,1,1)
    #pl.draw()
    #pl.close

    #pl.pause(1.0)

print 'Press enter to write LUT and exit'
nodata=raw_input()
pl.close
outName = os.path.expanduser('/home/pi/Desktop/testing.txt')
print outName

fd = open(outName, 'w')
for (a,b) in zip(vLevels, respV):
    fd.write('%7.4f %7.6f\n' % (a,b))
fd.close()
os.system("sshpass -p hullglick rsync ~/Desktop/testing.txt hullglick@test-rig-2.dhe.duke.edu:~/Documents/LED_Table" )
#dac.setVoltage(-1)
#! /usr/bin/python
import u6
import pylab as pl
from pylab import plot, show
import numpy as np
import time, datetime, os

today = datetime.date.today()

todaystr = today.isoformat()

startV = 0
stopV = 5
nVPts = 51

d = u6.U6() # open LabJack U6

d.getCalibrationData() # get calibration info for U6

#power_2V = 0.64
# setup plotting
#pl.ion()
#pH, = pl.plot(0,0, 'b.-')
#pl.xlabel('Command Voltage (V)')
#pl.ylabel('Calculated LED Output (mW)')

# Set DAC0 to 2V and get analog reading from AIN0 to calculate scale
DAC0_value = d.voltageToDACBits(2, dacNumber = 0, is16Bits = False)
d.getFeedback(u6.DAC0_8(DAC0_value))

power_2V=raw_input('Press enter Power Reading on Photometer: ')

scale = power_2V/d.getAIN(0) 

vLevels = np.linspace(startV, stopV, nVPts)
respV = vLevels*np.nan # initialize led power array

for (tVNum,tV) in enumerate(vLevels):
    
    # set ao value
    #volts = (tV*np.ones(1))
    print(tV)
    
    # loop through each voltage value and set DAC0
    
    DAC0_value = d.voltageToDACBits(tV, dacNumber = 0, is16Bits = False)
    d.getFeedback(u6.DAC0_8(DAC0_value))
    time.sleep(3)  # wait for 3s
    
    # read resulting analog AIN0
    ainValue = d.getAIN(0)
    tDataCol = (ainValue*scale) #scale the input voltage to power
    print tDataCol
    
    #responseData = tDataCol
    #responseData = np.array([x for x in tDataCol])
    #load into a vector - discard pts at start for stability
    #respV[tVNum] = np.mean(tDataCol)
    respV[tVNum] = tDataCol
    #respV = responseData

    #noNanIx = np.logical_not(np.isnan(respV))

    #pH.set_xdata(vLevels[noNanIx])
    #pH.set_ydata(np.array(respV[noNanIx]))
    #pl.gca().relim()
    #pl.gca().autoscale_view(1,1,1)
    #pl.draw()
    #pl.close

    #pl.pause(1.0)
DAC0_value = d.voltageToDACBits(0, dacNumber = 0, is16Bits = False)
d.getFeedback(u6.DAC0_8(DAC0_value))

nodata=raw_input('Press enter to write LUT and exit')
pl.close

dirname = "~/Documents/LED_Table/" + todaystr;
os.mkdir(dirname)
os.chdir(dirname)
#outName = os.path.expanduser('~/Documents/LED_Table/LED.txt')
outName = "LED.txt"
print outName

fd = open(outName, 'w')
for (a,b) in zip(vLevels, respV):
    fd.write('%7.4f %7.6f\n' % (a,b))
fd.close()

#! /usr/bin/python
import u6
import pylab as pl
from pylab import plot, show
import numpy as np
import time, datetime, os

today = datetime.date.today()

todaystr = today.isoformat()

d = u6.U6() # open LabJack U6

d.getCalibrationData() # get calibration info for U6

#power_2V = 0.64
# setup plotting
#pl.ion()
#pH, = pl.plot(0,0, 'b.-')
#pl.xlabel('Command Voltage (V)')
#pl.ylabel('Calculated LED Output (mW)')
device_type = raw_input('Are you calibrating led or laser? ')
# Set DAC0 to 2V and get analog reading from AIN0 to calculate scale
if device_type == "led":
    startV = 0
    stopV  = 5
    nVPts  = 51
    DAC0_value = d.voltageToDACBits(2, dacNumber = 0, is16Bits = False)
elif device_type == "laser":
    startV = 1.43
    stopV  = 2.1
    nVPts  = 101
    DAC0_value = d.voltageToDACBits(1.8, dacNumber = 0, is16Bits = False)

d.getFeedback(u6.DAC0_8(DAC0_value))

power_2V=float(raw_input('Press enter Power Reading on Photometer: '))

scale = power_2V/d.getAIN(0) 

vLevels = np.linspace(startV, stopV, nVPts)
print vLevels
respV = vLevels*np.nan # initialize led power array

for (tVNum,tV) in enumerate(vLevels):
    
    #volts = (tV*np.ones(1))
    print(tV)
    
    #set DAC0
    DAC0_value = d.voltageToDACBits(tV, dacNumber = 0, is16Bits = False)
    d.getFeedback(u6.DAC0_8(DAC0_value))
    if device_type == "led":
        time.sleep(3)  # wait for 3s
    elif device_type == "laser":
        time.sleep(10)
    # read resulting analog AIN0
    ainValue = d.getAIN(0)
   
    tDataCol = (ainValue*scale) #scale the input voltage to power
    print tDataCol
    
    respV[tVNum] = tDataCol

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

pl.close

dirname = "/Users/hullglick/Documents/Calibration_Table/" + todaystr;
if not os.path.exists(dirname):
    os.makedirs(dirname)

os.chdir(dirname)

outName = device_type + ".txt"
print dirname + "/" + outName +" is written."

fd = open(outName, 'w')
for (a,b) in zip(vLevels, respV):
    fd.write('%7.4f %7.6f\n' % (a,b))
fd.close()

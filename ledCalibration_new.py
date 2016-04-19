#! /usr/bin/python
import u6
import pylab
import numpy as np
import time, datetime, os

today       = datetime.date.today() # get time string for folder name
todaystr    = today.isoformat()

d           = u6.U6() # open LabJack U6

d.getCalibrationData() # get calibration info for U6

# turn off device first
DAC0_value = d.voltageToDACBits(0, dacNumber = 0, is16Bits = False)
d.getFeedback(u6.DAC0_8(DAC0_value))

device_type = raw_input('Are you calibrating led or lasergreen or laserblue? ')
fiber_type  = raw_input('What is your fiber diameter(nm)? ')

# Set DAC0 to 2V and get analog reading from AIN0 to calculate scale
if device_type == "led":
    startV = 0.057
    stopV  = 2
    nVPts  = 81
    DAC0_value = d.voltageToDACBits(1, dacNumber = 0, is16Bits = False)
elif device_type == "lasergreen":
    startV = 1.6
    stopV  = 2.15
    nVPts  = 21
    DAC0_value = d.voltageToDACBits(1.9, dacNumber = 0, is16Bits = False)
elif device_type == "laserblue":
    startV = 1.1
    stopV  = 3
    nVPts  = 21
    DAC0_value = d.voltageToDACBits(1.9, dacNumber = 0, is16Bits = False)

d.getFeedback(u6.DAC0_8(DAC0_value))

power_2V=float(raw_input('Press enter Power Reading on Photometer: '))

scale = power_2V/d.getAIN(0) # get scale for photometer

vLevels = np.linspace(startV, stopV, nVPts)
print vLevels
respV = vLevels*np.nan # initialize led power array

for (tVNum,tV) in enumerate(vLevels):

    print [ tV ]
    
    #set DAC0
    DAC0_value = d.voltageToDACBits(tV, dacNumber = 0, is16Bits = False)
    d.getFeedback(u6.DAC0_8(DAC0_value))
    if device_type == "led":
        time.sleep(4)  # wait for 3s
    elif device_type == "laser":
        time.sleep(10)

    # read resulting analog AIN0
    ainValue = d.getAIN(0)
   
    tDataCol = (ainValue*scale) #scale the input voltage to power
    print tDataCol
    
    respV[tVNum] = tDataCol

    #noNanIx = np.logical_not(np.isnan(respV))


pylab.plot(vLevels, respV)
pylab.xlabel('Voltage')
pylab.ylabel('Power(mW)')

DAC0_value = d.voltageToDACBits(0, dacNumber = 0, is16Bits = False)
d.getFeedback(u6.DAC0_8(DAC0_value))

dirname = "/Users/hullglick/Documents/Calibration_Table/" + todaystr + "fiber" + fiber_type;
if not os.path.exists(dirname):
    os.makedirs(dirname)
    outName = device_type + ".txt"
elif os.path.exists(dirname):
    print "writing path alread exists and file to be written is renamed as 2.txt"
    outName = device_type + "2.txt"
os.chdir(dirname)


print dirname + "/" + outName +" is written."

fd = open(outName, 'w')
for (a,b) in zip(vLevels, respV):
    fd.write('%7.4f %7.6f\n' % (a,b))
fd.close()

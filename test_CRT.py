#! /usr/bin/python
import u6
import time
import numpy

d           = u6.U6() # open LabJack U6

d.getCalibrationData() # get calibration info for U6
runtime=[]
for i in range(0,1000):
	t1 = time.time()

	d.getFeedback(u6.BitStateWrite(0,1))

	ain = d.getAIN(0)

	if ain > 1:
		runtime.append(time.time()-t1)
	
print numpy.mean(runtime)

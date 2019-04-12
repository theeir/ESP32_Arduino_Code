# A test program.
This program is a quick hack to test samples from the accelerometer. 
It outputs raw data from the mpu6050 accelerometer. The data is an average of a set number of samples. 
the samples are set by a variable.

there are 3 bluetooth commands. 
- sample. (hex code 0x01)
reads the samples for the set time.  
-  change sample time (hex code 0x02 0x[data])
change the number of secs it samples for. 
- change sample number (hex code 0x3 0x[data])
change the number of samples. it is a multiple of 10, so if data is 0x02, the sample number will be set to 20. 

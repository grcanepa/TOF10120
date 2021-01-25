# TOF10120
How to use the sensor commands interface
I was surprised at the lack of reference code to communicate with the Tof10120 over I2C

There is basic code to get the distance, but when you dig into the data sheet and translate the Chinese you can find a lot more, such as:
- Read "filtered" distance
- Set/read a distance offset (calibrate)
- Enable/disable I2C/uart communication
- Set/read the I2C address

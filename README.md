# TOF10120
How to use the sensor commands interface
I was surprised at the lack of reference code to communicate with the Tof10120 over I2C

There is basic code to get the distance, but when you dig into the data sheet and translate the Chinese you can find a lot more, such as:
- Read "filtered" distance
- Set/read a distance offset (calibrate)
- Enable/disable I2C/uart communication
- Set/read the I2C address

This is my first time publishing code on GitHub, so doing both to share with the world, be able to in future find for myself, as well as practice how to use GitHub

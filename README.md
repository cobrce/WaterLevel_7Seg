# WaterLevel
Measure and display water level in a tank using vl53l0x sensor

Started as ATtiny26 project with an ultrasonic sensor, due to the shape of the tank I was working on the sensor starts giving wrong measurments when it reaches some level, so I had to switch to the VL53L0X time of flight sensor, this one uses I2C and a has to be initialized wite some data making the code too big for the tiny26 (or maybe I was too lazy to opitimize), thus the switch to ATmega328 (the one I had with enough memory)


# THIS IS A CUT VERSION THAT USES ONLY 3 SEVEN SEGMENTS AND BARGRAPH, NO MAX7219 AND NO ERROR DISPLAY

# Credits:
Thanks to yetifrisstlama for [vl53l0x library](https://github.com/yetifrisstlama/vl53l0x-non-arduino)

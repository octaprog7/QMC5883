# MicroPython module for control QMC5883L Geomagnetic Sensor.
## [На русском](README_RU.md)

# I2C bus
Just connect (VCC, GND, SDA, SCL) from your QMC5883L board to Arduino, ESP or any other board with MicroPython firmware.

# Supply
Supply voltage QMC5883L 3.3 Volts only!

# Upload
Upload micropython firmware to the NANO(ESP, etc) board, and then files: main.py, qmc5883mod.py and sensor_pack folder. 
Then open main.py in your IDE and run it.

# Pictures
## QMC5883L board view
![alt text](https://github.com/octaprog7/QMC5883/blob/master/pics/board_5883.jpg)
The inscription on the board (HMC5883L) is incorrect! All (almost) such boards have QMC5883L installed!
## IDE
![alt text](https://github.com/octaprog7/QMC5883/blob/master/pics/ide_5883.png)

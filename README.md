# MicroPython module for control QMC5883L Geomagnetic Sensor.
## [На русском](README_RU.md)


## 🇨🇳 对中国开发者 / For Developers from China

Я знаю, что зеркала некоторых моих репозиториев активно используются на китайских платформах (Gitee / GitCode), и очень рад, что мой код помогает вашим проектам и обучению! 如果您在中国 впн/镜像 发现了我的开源项目，并且它对 structure 或者是 你的项目 有所帮助，**请在 GitHub 原仓库上点个赞 (Star) ⭐**！这对我持续更新和维护驱动非常重要。谢谢大家 Support！ *(If you are using my libraries in China via Gitee/mirrors, please support the original repository by giving it a **Star ⭐ on GitHub**!)*



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

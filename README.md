# TemperatureSensor
Code base contains the firmware, and post processing code, for reading, processing, and displaying read back temperature information from a 10K thermistor. Python was used for reading sent values over COM port, interpreting them, and updating a realtime plot of temperature. All calculations and temperature handling is done on device by firmware.\
\
ATMega328P MCU, coupled with Arduino Nano, was used for handling the analog signals from thermistor and sending data to computer for visualization. 

## Contents
- [readSerial.py](https://github.com/Sharketec2016/TemperatureSensor/blob/master/Python/readserial.py) -> File on computer for reading serial data from nano.
- [main.cpp](https://github.com/Sharketec2016/TemperatureSensor/blob/master/src/main.cpp) -> Firmware file for ATMega328P.

## Dependencies
Note: All dependencies used are the latest version for each package.
### Package
- [PlatformIO](https://platformio.org/) -> Used for creating the packages firmware, flashing, and deploying onto ATMega328P MCU.

### Main Firmware
- [Avrdudes/AVR](https://github.com/avrdudes/avr-libc)
- [Arduino C Library](https://docs.arduino.cc/libraries/)
### Temperature Displayment
- [Matplotlib](https://matplotlib.org/) -> Visualizing continous time varying temperature
- [Numpy](https://numpy.org/) -> array and matrix handling 
- [Pyserial](https://pyserial.readthedocs.io/en/latest/index.html) -> read incoming data from COM port
  

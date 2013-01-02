# Hygrometer

Basic Arduino sketch to measure relative humidity and temperature using an EFS-10 and an LM135.

## Installing/building
- Build the cicuit below
- Checkout this project
- Build and upload to your circuit
- Monitor the serial port for values

_Need a quick and easy way to collect data from the serial port?_
See [LogSerial](https://github.com/aweijnitz/LogSerial) :-)


## Notes
This currently uses an uncalibrated temperature senor setup, but it is very easy to add one. Basically you just have to add a 10K potentionmeter to the third pin of the sensor. I got decent readings without calibration and just added in some "software calibration" to compensate on the fly. 

See the application notes in the LM135 datasheet for more info on calibration of the sensor (figure 5, Calibrated Sensor). [LM135 datasheet](http://www.ti.com/lit/ds/symlink/lm335.pdf) 


## The circuit
[![CircuitLab Schematic 6h6z7z](https://www.circuitlab.com/circuit/6h6z7z/screenshot/540x405/)](https://www.circuitlab.com/circuit/6h6z7z/arduino-hygrometer/)





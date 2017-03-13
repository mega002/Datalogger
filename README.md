# Datalogger

Datalogger is a power-failure resistant firmware that allows continuous logging of data from external source, which then could be downloaded at the touch of a button. It is implemented on the TI CC2650 LaunchPad, where the data is stored on an external flash, and being downloaded over BLE. 

Here, we demonstrate the Datalogger firmware with another chip, which measures temperature at intervals.

Check out our webpage for more details:

[TBD](https://www.google.com)



## Installation
This project is based on two TI example projects:
* Datalogger is based on the simple peripheral BLE example project - Here, you should update the files 'main.c', 'simple_peripheral.c', 'simple_peripheral.h', and add to the 'Application' folder the files 'data_service.c', 'data_service.h'
* External data source is based on the pin interrupt example project - Here, you should replace the file 'pinInterrupt.c' with the file 'data_sender.c'

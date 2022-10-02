<p align="center"> <img src="https://drive.google.com/uc?id=17raboZZPt7QHst0Pi5oCvFXSffUdkcQB"/></p>

> MSPFCI: Multiwii Serial Protocol Flight Controller Interface

A multiwii serial protocol interface for your flight controller running Cleanflight/Betaflight/Inav

## Prerequisites
 
- [Serial](https://github.com/wjwwood/serial)

## License

This software is made available to the public to use (source-available), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the [LICENSE](LICENSE) file. 

## Project Status

 - [x] Periodic callbacks with custom frequency based on RAII
 - [x] Separate threads for each periodic callbacks
 - [x] SFINAE based MSP message decoding
 - [x] Multi level logger
 - [ ] Implementation of MSP messgaes for all sensor
 - [ ] Implementation of Arming commands
 - [ ] Implementation of control commands
 - [ ] ROS1 wrapper
 - [ ] ROS2 wrapper

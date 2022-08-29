# Python program for FLIR Tau2 IR camera

## Credits

https://github.com/jveitchmichaelis with the flirpy package https://github.com/LJMUAstroecology/flirpy

## Basic usage

- *tau2.py* contains `TeaxGrabber` class, which allows to communicate with the camera through **serial** protocol and adjust settings (i.e. gain mode, FFC mode...) as well as grabbing RAW images from the camera and save them to **FITS** format through the **FTDI** protocol. Note that these communications exclude each other. One can only communicate through serial to send instructions and modify parameters of the camera and cannot receive images at the same time. Only necessary methods have been implemented into the class. See the official manufacturer documentation to implement additional methods you would like to use based on the paradigm imposed in the class.
- *tau2_instructions.py* contains all instructions to send to the camera and *default_settings* dictionary which applies default parameters after camera initialization in **serial** mode.

## Example

![Sample image of FLIR Tau2 camera in flux RAW 14bits non-radiometric mode](flir_tau2_image_flux.png)

## References
- FLIR Tau2/Quark2 software IDD : 102-PS242-43 version 133
- FLIR Tau2 product specification : 102-PS242-40 version 141
- FLIR Advanced Radiometry Application Note : 102-PS242-100-14 version 120

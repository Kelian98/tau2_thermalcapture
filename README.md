# Python program for FLIR Tau2 IR camera

### Credits

https://github.com/jveitchmichaelis with the flirpy package https://github.com/LJMUAstroecology/flirpy

## Quick start

```python
# Import and create object from this class to adjust parameters of the camera
from tau2_class import *
Tau2 = FLIR_Tau2()
gain_mode, _ = Tau2.get_gain_mode()
gain_mode
>>> GAIN MODE : 0X0002 = High Gain Only
Tau2.__exit__() # need to exit before starting acquisition for protocol purposes

# Import and create object from this class to start image acquisition
from tau2_grab import *
camera = TeaxGrabber()
imlist = camera.grab_image(duration=1.0, display=True)
camera.__exit__() # need to exit before starting acquisition for protocol purposes
```

## Basic usage

- *tau2_class.py* contains `FLIR_Tau2` class, which allows to communicate with the camera through **serial** protocol and adjust settings (i.e. gain mode, FFC mode...)
- *tau2_grab.py* contains `TeaxGrabber` class, which allows to get RAW images from the camera and save them to **FITS** format through the **FTDI** protocol.
- *tau2_instructions.py* contains all instructions to send to the camera and *default_settings* dictionnary which applies default parameters after camera initialization in **serial** mode.

## References
- FLIR Tau2/Quark2 software IDD : 102-PS242-43 version 133
- FLIR Tau2 product specification : 102-PS242-40 version 141
- FLIR Advanced Radiometry Application Note : 102-PS242-100-14 version 120

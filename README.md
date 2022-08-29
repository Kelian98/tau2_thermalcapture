# Python program for FLIR Tau2 IR camera

## Credits

https://github.com/jveitchmichaelis with the flirpy package https://github.com/LJMUAstroecology/flirpy

## Basic usage

- *tau2.py* contains `FLIR_Tau2` class, which allows to communicate with the camera through **serial** protocol and adjust settings (i.e. gain mode, FFC mode...) as well as grabbing RAW images from the camera and save them to **FITS** format through the **FTDI** protocol. Note that these communications exclude each other. One can only communicate through serial to send instructions and modify parameters of the camera and cannot receive images at the same time. Only necessary methods have been implemented into the class. See the official manufacturer documentation to implement additional methods you would like to use based on the paradigm imposed in the class.
- *tau2_instructions.py* contains all instructions to send to the camera and *default_settings* dictionary which applies default parameters after camera initialization in **serial** mode.

## Example

```python
In [1]: from tau2 import *

In [2]: Tau2 = FLIR_Tau2()
========== ACQ : Connected to the FLIR Tau2 camera ==========
CMD : Sending b'n\x00\x00\n\x00\x0288\x00\x02 B'
CMD : Received: array('B', [110, 0, 0, 10, 0, 2, 56, 56, 0, 2, 32, 66])
CMD : Response OK
CMD : Sending b'n\x00\x00\n\x00\x00\x18z\x00\x00\x00'
CMD : Received: array('B', [110, 0, 0, 10, 0, 2, 56, 56, 0, 2, 32, 66])
CMD : Response OK
GAIN MODE : 0X0002 = High Gain Only
CMD : GAIN MODE IS CONFIGURED PROPERLY
CMD : Sending b'n\x00\x00M\x00\x04\xc0\xc3\x00\x00\x00\x00\x00\x00'
CMD : Received: array('B', [110, 0, 0, 77, 0, 0, 128, 71, 0, 0])
CMD : Response OK
CMD : Sending b'n\x00\x00M\x00\x04\xc0\xc3\x00\x01\x00\x0070'
CMD : Received: array('B', [110, 0, 0, 77, 0, 2, 160, 5, 0, 0, 0, 0])
CMD : Response OK
SHUTTER TEMP MODE : 0X0000 = User, User specified shutter temperature
CMD : SHUTTER TEMPERATURE MODE IS CONFIGURED PROPERLY
CMD : Sending b'n\x00\x00\x0b\x00\x02\x0f\x08\x00\x00\x00\x00'
CMD : Received: array('B', [110, 0, 0, 11, 0, 2, 15, 8, 0, 0, 0, 0])
CMD : Response OK
CMD : Sending b'n\x00\x00\x0b\x00\x00/J\x00\x00\x00'
CMD : Received: array('B', [110, 0, 0, 11, 0, 2, 15, 8, 0, 0, 0, 0])
CMD : Response OK
FFC MODE : 0X0000 = Manual
CMD : FFC MODE IS CONFIGURED PROPERLY
CMD : Sending b'n\x00\x00\x0b\x00\x04o\xce\x00\x02\x00\x02N"'
CMD : Received: array('B', [110, 0, 0, 11, 0, 0, 47, 74, 0, 0])
CMD : Response OK
CMD : Sending b'n\x00\x00\x0b\x00\x04o\xce\x00\x03\x00\x00YP'
CMD : Received: array('B', [110, 0, 0, 11, 0, 2, 15, 8, 0, 2, 32, 66])
CMD : Response OK
FFC NFRAMES : 0X0002 =  16 frames
CMD : FFC FRAMES IS CONFIGURED PROPERLY
CMD : Sending b'n\x00\x00\x12\x00\x02\xd2\xfa\x03\x02u\x11'
CMD : Received: array('B', [110, 0, 0, 18, 0, 2, 210, 250, 3, 2, 117, 17])
CMD : Response OK
CMD : Sending b'n\x00\x00\x12\x00\x02\xd2\xfa\x02\x00fb'
CMD : Received: array('B', [110, 0, 0, 18, 0, 2, 210, 250, 0, 2, 32, 66])
CMD : Response OK
XP MODE : 0x0002 = CMOS 14-bit w/ 1 discrete
CMD : XP MODE IS CONFIGURED PROPERLY
CMD : Sending b'n\x00\x00\x12\x00\x02\xd2\xfa\x06\x00\xaa\xa6'
CMD : Received: array('B', [110, 0, 0, 18, 0, 2, 210, 250, 6, 0, 170, 166])
CMD : Response OK
CMD : Sending b'n\x00\x00\x12\x00\x02\xd2\xfa\x08\x00\x89\xa9'
CMD : Received: array('B', [110, 0, 0, 18, 0, 2, 210, 250, 0, 0, 0, 0])
CMD : Response OK
CMOS BIT DEPTH : 0x0000 = 14bit
CMD : CMOS BITDEPTH IS CONFIGURED PROPERLY
CMD : Sending b'n\x00\x00\x8e\x00\x04\xbfd\x00@\x00\x00\x1d\xad'
CMD : Received: array('B', [110, 0, 0, 142, 0, 0, 255, 224, 0, 0])
CMD : Response OK
CMD : Sending b'n\x00\x00\x8e\x00\x02\xdf\xa2\x00@H\xc4'
CMD : Received: array('B', [110, 0, 0, 142, 0, 2, 223, 162, 0, 0, 0, 0])
CMD : Response OK
TLIN MODE DISABLED : 0x0000 = disabled
CMD : TLINEAR MODE IS CONFIGURED PROPERLY
CMD : ALL PARAMETERS SET, CAN START ACQUISITION
========== FFC IN PROGRESS ==========
CMD : Sending b'n\x00\x00\x0c\x00\x00\xaa\xda\x00\x00\x00'
CMD : Received: array('B', [110, 0, 0, 12, 0, 0, 170, 218, 0, 0])
CMD : Response OK
========== FFC DONE ==========

In [3]: Tau2.get_mode()
Out[3]: <BitMode.RESET: 0>

# Need to be in 'syncff' mode to grab images
In [4]: Tau2.set_mode("syncff")
Current mode is : syncff

In [5]: Tau2.grab_image(display=True)
Read buffer for 1.0 seconds
Image sequence sliced
Out[5]: 
[array([[1389, 1387, 1386, ..., 1350, 1352, 1353],
        [1389, 1389, 1385, ..., 1350, 1351, 1353],
        [1386, 1390, 1388, ..., 1355, 1354, 1352],
        ...,
        [1414, 1420, 1415, ..., 1362, 1361, 1359],
        [1414, 1412, 1412, ..., 1362, 1365, 1362],
        [1411, 1412, 1413, ..., 1362, 1360, 1363]], dtype=int16),
 array([[1388, 1387, 1386, ..., 1350, 1353, 1353],
        [1389, 1389, 1385, ..., 1352, 1352, 1353],
        [1388, 1389, 1388, ..., 1354, 1355, 1353],
        ...,
        [1415, 1420, 1415, ..., 1366, 1360, 1360],
        [1424, 1414, 1412, ..., 1365, 1365, 1362],
        [1417, 1416, 1414, ..., 1362, 1360, 1363]], dtype=int16),
 array([[1388, 1387, 1386, ..., 1351, 1354, 1354],
        [1389, 1388, 1385, ..., 1353, 1354, 1354],
        [1389, 1392, 1390, ..., 1354, 1354, 1353],
        ...,
        [1414, 1420, 1415, ..., 1366, 1361, 1360],
        [1423, 1413, 1412, ..., 1365, 1365, 1362],
        [1417, 1416, 1414, ..., 1362, 1360, 1363]], dtype=int16),
 array([[1399, 1403, 1400, ..., 1369, 1368, 1367],
        [1402, 1401, 1399, ..., 1369, 1365, 1367],
        [1401, 1401, 1400, ..., 1368, 1366, 1366],
        ...,
        [1422, 1426, 1422, ..., 1372, 1372, 1369],
        [1421, 1416, 1418, ..., 1373, 1370, 1370],
        [1420, 1420, 1418, ..., 1371, 1366, 1367]], dtype=int16),
 array([[1400, 1403, 1398, ..., 1367, 1368, 1367],
        [1400, 1401, 1392, ..., 1368, 1367, 1367],
        [1400, 1398, 1398, ..., 1366, 1366, 1366],
        ...,
        [1426, 1424, 1422, ..., 1372, 1370, 1369],
        [1421, 1417, 1420, ..., 1371, 1371, 1369],
        [1420, 1420, 1418, ..., 1371, 1367, 1368]], dtype=int16)]

In [6]: Tau2.__exit__()
========== ACQ : Disconnecting from camera ==========

```

![Sample image of FLIR Tau2 camera in flux RAW 14bits non-radiometric mode](flir_tau2_image_flux.png)

## References
- FLIR Tau2/Quark2 software IDD : 102-PS242-43 version 133
- FLIR Tau2 product specification : 102-PS242-40 version 141
- FLIR Advanced Radiometry Application Note : 102-PS242-100-14 version 120

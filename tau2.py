##################################################
## Python class for the FLIR TAU2 camera in FTDI mode
##################################################
## Authors: Kelian SOMMER, Bertrand PLEZ, Johann COHEN-TANUGI
## Credits: flirpy
## Version: 1.1
## Date : 2022-05-02
##################################################

import serial
import binascii
import struct
import time
from datetime import datetime
import logging
import numpy as np
from astropy.io import fits
import tqdm
import math
import os
import usb.core
import usb.util
from pyftdi.ftdi import Ftdi
import pyftdi.serialext
import matplotlib.pyplot as plt
plt.style.use('seaborn-white')

# Create new folder to store images
time_sync = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
curr_path = os.getcwd()
new_dir = curr_path+'/'+time_sync
os.mkdir(new_dir)

# Setting up log file
log_cam = logging.getLogger()
log_cam.setLevel(logging.INFO)
log_file = time_sync + '_Tau2.log'
handler = logging.FileHandler(new_dir+'/'+log_file, 'w', 'utf-8')
handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
log_cam.addHandler(handler)

from tau2_instructions import *

class TeaxGrabber(object):
    """
    Data acquisition class for the Teax ThermalCapture Grabber USB w/ FLIR Tau2 camera
    """
    
    def __init__(self, vid=0x0403, pid=0x6010, width=640, height=512):
        """Initialize FTDI connection to the FLIR Tau2 camera for acquisition

        Parameters
        ----------
        vid : bytes
            Vendor id
        pid : bytes
            Product id
        width : int
           Width of images (default = 640 px)
        height : int
           Height of images (default = 512 px)

        """
        
        self.dev = usb.core.find(idVendor=vid, idProduct=pid)
        self._ftdi = None
        self.frame_size = 2*height*width+10+4*height # 10 byte header, 4 bytes pad per row
        self.magic_ftdi = b'TEAX'
        self.magic_uart = b'UART'
        
        if self.dev is not None:
            self.connect(mode='serial')
            self._sync(allow_timeout=True)
            
    def connect(self, mode):
        """Connect to the FLIR Tau2 camera using FTDI interface
        Parameters
        ----------
        mode : str
            Serial mode or acquisition mode
        """
        
        if self.dev.is_kernel_driver_active(0):
            self.dev.detach_kernel_driver(0)
        
        self._claim_dev()
        
        self._ftdi = Ftdi()
        self._ftdi.open_from_device(self.dev)
        
        if self._ftdi.is_connected == True:
            print("========== ACQ : Connected to the FLIR Tau2 camera ==========")
            log_cam.info("========== ACQ : Connected to the FLIR Tau2 camera ==========")
            if mode == 'serial':
                self._ftdi.set_bitmode(0xFF, Ftdi.BitMode.RESET)
                self.current_mode = 'serial'

                self.settings_state = self.check_settings()
                if self.settings_state == True:
                    print("CMD : ALL PARAMETERS SET, CAN START ACQUISITION")
                    log_cam.info("CMD : ALL PARAMETERS SET, CAN START ACQUISITION")
                    self.do_ffc_short()
                else:
                    print("CMD : AN ERROR OCCURED WHILE SETTING UP PARAMETERS")
                    log_cam.error("CMD : AN ERROR OCCURED WHILE SETTING UP PARAMETERS")
                    self.__exit__()            
            else: 
                self._ftdi.set_bitmode(0xFF, Ftdi.BitMode.SYNCFF)
                self.current_mode = 'syncff'
            # self._ftdi.set_baudrate(baudrate=12e6) # 12MBaud maximum of FT2232H
            self._ftdi.purge_buffers()
        else:
            print("========== ACQ : An error occurred while trying to establish connection with FLIR TAU2 camera with FTDI protocol ==========")
            log_cam.error("========== ACQ : An error occurred while trying to establish connection with FLIR TAU2 camera with FTDI protocol ==========")
        
    def get_mode(self, display = False):
        """Get current operating mode
        Parameters
        ----------
        display : bool
            Print current mode
        Returns
        -------
        self._ftdi.bitmode : <enum 'BitMode'>
            Current mode
        """
        
        if display == True:
            print('Current mode is : {}'.format(self.current_mode))
        log_cam.info('Current mode is : {}'.format(self.current_mode))
        return self._ftdi._bitmode
        
    def set_mode(self, mode):
        """Set current operating mode
        Parameters
        ----------
        mode : str
            Can be "serial" for soft control or "syncff" for image acquisition
        """
        if mode == 'serial':
            self._ftdi.set_bitmode(0xFF, Ftdi.BitMode.RESET)
            self.current_mode = 'serial'
        else:
            self._ftdi.set_bitmode(0xFF, Ftdi.BitMode.SYNCFF)
            self.current_mode = 'syncff'
        self._ftdi.purge_buffers()
        print('Current mode is : {}'.format(self.current_mode))
        log_cam.info('Current mode is : {}'.format(self.current_mode))
        
    def _claim_dev(self):
        """Claim USB interface
        """
        
        self.dev.reset()
        self._release()
        
        self.dev.set_configuration(1)

        usb.util.claim_interface(self.dev, 0)
        usb.util.claim_interface(self.dev, 1)
        
    def _release(self):
        for cfg in self.dev:
            for intf in cfg:
                if self.dev.is_kernel_driver_active(intf.bInterfaceNumber):
                    try:
                        self.dev.detach_kernel_driver(intf.bInterfaceNumber)
                    except usb.core.USBError as e:
                        print("ACQ : Could not detatch kernel driver from interface({0}): {1}".format(intf.bInterfaceNumber, str(e)))
                        log_cam.error("ACQ : Could not detatch kernel driver from interface({0}): {1}".format(intf.bInterfaceNumber, str(e)))
    
    def _read(self, n_bytes=0, packets_per_transfer=8, num_transfers=256):
        """Read bytes from device

        Parameters
        ----------
        n_bytes : int
            Number of bytes to read from the device
        packets_per_transfer : int
            Number of packets per transfer (standard value for this FTDI FT2232 chip)
        num_transfers : int
            Number of total transfers (standard value for this FTDI FT2232 chip)

        Returns
        -------
        self._ftdi.read_data(n_bytes) : bytes
            Raw response from the camera in bytes

        """
        
        FTDI_PACKET_SIZE = 512
        
        if n_bytes == 0:
            n_bytes = packets_per_transfer * FTDI_PACKET_SIZE
  
        return self._ftdi.read_data(n_bytes)
                
    def _sync(self, allow_timeout=False):
        """Sync output buffer with TEAX magic keyword

        Parameters
        ----------
        allow_timeout : bool
            Enable timeout for response

        Returns
        -------
        data[data.find(magic):] : bytesarray
            Raw response from the camera in bytes

        """
        data = self._read()
        
        t = time.time()
        while data.find(self.magic_ftdi) == -1:
            data = self._read()
            
            if not allow_timeout and time.time()-t > 0.2:
                log_cam.warn("Timeout in frame sync")
                break
            elif time.time() -t > 0.2:
                break
            
        return data[data.find(self.magic_ftdi):]
    
    def _sync_uart(self, allow_timeout=False):
        """Sync output buffer with UART magic keyword

        Parameters
        ----------
        allow_timeout : bool
            Enable timeout for response

        Returns
        -------
        data[data.find(magic):] : bytesarray
            Raw response from the camera in bytes

        """
        data = self._read()
        
        t = time.time()
        while data.find(self.magic_uart) == -1:
            data = self._read()
            
            if not allow_timeout and time.time()-t > 0.2:
                log_cam.warn("Timeout in command sync")
                break
            elif time.time() -t > 0.2:
                break
        
        return data[data.find(self.magic_uart):]
        
    def _send_data(self, data):
        """Send byte data to the camera

        Parameters
        ----------
        data : bytesarray
            Write bytesarray to the serial device

        """
        self.dev.write(0x2, data)

    def _receive_data(self, nbytes):
        """Read bytes data from the camera
        
        Parameters
        ----------
        nbytes : int
            Number of bytes to read from the device

        """
        time.sleep(0.1)
        res = self.dev.read(0x81, nbytes)
        return res

    def _send_packet(self, command, argument=None):
        """Send packet with command and argument to the camera

        Parameters
        ----------
        command : object tau2_instructions.code
            Command to execute on the device
        argument : bytes
            Additional argument to provide [i.e struct.pack(">h", 0x0000)]

        """
        if argument is None: 
            argument = [] 

        # Refer to Tau 2 Software IDD 
        # Packet Protocol (Table 3.2) 
        packet_size = len(argument) 
        assert(packet_size == command.cmd_bytes)   

        process_code = int(0x6E).to_bytes(1, 'big') 
        status = int(0x00).to_bytes(1, 'big') 
        function = command.code.to_bytes(1, 'big') 

        # First CRC is the first 6 bytes of the packet 
        # 1 - Process code 
        # 2 - Status code 
        # 3 - Reserved 
        # 4 - Function 
        # 5 - N Bytes MSB 
        # 6 - N Bytes LSB 

        packet = [process_code, status, function] 
        packet.append( ((packet_size & 0xFF00) >> 8).to_bytes(1, 'big') ) 
        packet.append( (packet_size & 0x00FF).to_bytes(1, 'big') ) 
        crc_1 = binascii.crc_hqx(struct.pack("ccxccc", *packet), 0) 

        packet.append( ((crc_1 & 0xFF00) >> 8).to_bytes(1, 'big') ) 
        packet.append( (crc_1 & 0x00FF).to_bytes(1, 'big') ) 

        if packet_size > 0: 

            # Second CRC is the CRC of the data (if any) 
            crc_2 = binascii.crc_hqx(argument, 0) 
            packet.append(argument)
            packet.append( ((crc_2 & 0xFF00) >> 8).to_bytes(1, 'big') ) 
            packet.append( (crc_2 & 0x00FF).to_bytes(1, 'big') ) 

            fmt = ">cxcccccc{}scc".format(packet_size) 

        else: 
            fmt = ">cxccccccxxx" 

        data = struct.pack(fmt, *packet) 
        log_cam.info("CMD : Sending {}".format(data))

        self._send_data(data)
    
    def _read_packet(self, function, post_delay=0.1):
        """Read packet from the camera

        Parameters
        ----------
        function : object tau2_instructions.code
            Function command to execute on the device
        post_delay : float
            Waiting delay for the camera to respond

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        
        argument_length = function.reply_bytes
        data = self._receive_data(10+2+argument_length)
        data = data[2:] # remove 2 first bad bytes
        
        log_cam.info("CMD : Received: {}".format(data))

        if self._check_header(data[:6]) and len(data) > 0:
            if argument_length == 0:
                res = struct.unpack(">ccxcccccxx", data)
            else:
                res = struct.unpack(">ccxccccc{}scc".format(argument_length), data)
                #check_data_crc(res[7])
        else:
            res = None
            print("CMD : Error reply from camera. Try re-sending command, or check parameters.")
            log_cam.warning("CMD : Error reply from camera. Try re-sending command, or check parameters.")

        if post_delay > 0:
            time.sleep(post_delay)

        return res

    def _check_header(self, data):
        """Check if camera response is formatted successfully
        
        Parameters
        ----------
        data : bytesarray
            Bytesarray coming from the serial device

        """
        res = struct.unpack(">BBxBBB", data)

        if res[0] != 0x6E:
            print("CMD : Initial packet byte incorrect. Byte was: {}".format(res[0]))
            log_cam.warning("CMD : Initial packet byte incorrect. Byte was: {}".format(res[0]))
            return False

        if not self._check_status(res[1]):
            return False

        return True

    def _check_status(self, code):
        """Check camera status

        Parameters
        ----------
        code : object tau2_instructions.code
            Camera response status
        """

        if code == CAM_OK:
            print("CMD : Response OK")
            log_cam.debug("CMD :Response OK")
            return True
        elif code == CAM_BYTE_COUNT_ERROR:
            print("CMD : Byte count error.")
            log_cam.warning("CMD : Byte count error.")
        elif code == CAM_FEATURE_NOT_ENABLED:
            print("CMD : Feature not enabled.")
            log_cam.warning("CMD : Feature not enabled.")
        elif code == CAM_NOT_READY:
            print("CMD : Camera not ready.")
            log_cam.warning("CMD : Camera not ready.")
        elif code == CAM_RANGE_ERROR:
            print("CMD : Camera range error.")
            log_cam.warning("CMD : Camera range error.")
        elif code == CAM_TIMEOUT_ERROR:
            print("CMD : Camera timeout error.")
            log_cam.warning("CMD : Camera timeout error.")
        elif code == CAM_UNDEFINED_ERROR:
            print("CMD : Camera returned an undefined error.")
            log_cam.warning("CMD : Camera returned an undefined error.")
        elif code == CAM_UNDEFINED_FUNCTION_ERROR:
            print("CMD : Camera function undefined. Check the function code.")
            log_cam.warning("CMD : Camera function undefined. Check the function code.")
        elif code == CAM_UNDEFINED_PROCESS_ERROR:
            print("CMD : Camera process undefined.")
            log_cam.warning("CMD : Camera process undefined.")

        return False

    def _check_mode(argument):
        def decorator(func):
            def wrapper(self, *args, **kwargs):
                """Check if current mode of the camera is compatible with called method

                Parameters
                ----------
                func : method
                Function/method to decorate

                """
                if argument == 'serial':
                    if self.get_mode(display=False) == Ftdi.BitMode.RESET:
                        result = func(self, *args, **kwargs)
                    else:
                        print("ERROR : current mode doesn't support serial communication")
                        log_cam.error("ERROR : current mode doesn't support serial communication")
                        self.set_mode("serial")
                        result = func(self, *args, **kwargs)
                else:
                    if self.get_mode(display=False) == Ftdi.BitMode.SYNCFF:
                        result = func(self, *args, **kwargs)
                    else:
                        print("ERROR : current mode doesn't support syncff communication")
                        log_cam.error("ERROR : current mode doesn't support syncff communication")
                        self.set_mode("syncff")
                        result = func(self, *args, **kwargs)
                return result
            return wrapper
        return decorator

    def _flush_in_out(func):
        """Decorator to flush input and output buffer before and after each instruction sent to the camera
        
        Parameters
        ----------
        func : method
            Function/method to decorate

        """
        def decorated_func(self, *args, **kwargs):

            self._ftdi.purge_buffers()
            result = func(self, *args, **kwargs)
            self._ftdi.purge_buffers()
            return result

        return decorated_func
    
    def close(self):
        """Close the FTDI communication.
        """
        if self._ftdi is not None:
            self._ftdi.close()

    def __exit__(self):
        """Exit the connection
        """
        self.close()
        print("========== ACQ : Disconnecting from camera ==========")
        log_cam.info("========== ACQ : Disconnecting from camera ==========")

    @_flush_in_out
    @_check_mode('serial')
    def ping(self):
        """Ping the camera to see if it's alive

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes
        """
        
        function = NO_OP
        self._send_packet(function)
        res = self._read_packet(function)
        return res

    @_flush_in_out
    @_check_mode('serial')
    def get_serial_number(self):
        """Get serial number

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes
        """

        function = GET_SERIAL_NUMBER
        self._send_packet(function)
        res = self._read_packet(function)

        log_cam.info("Camera serial: {}".format(int.from_bytes(res[7][:4], byteorder='big', signed=False)))
        log_cam.info("Sensor serial: {}".format(int.from_bytes(res[7][4:], byteorder='big', signed=False)))

        print("Camera serial: {}".format(int.from_bytes(res[7][:4], byteorder='big', signed=False)))
        print("Sensor serial: {}".format(int.from_bytes(res[7][4:], byteorder='big', signed=False)))

        return res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_baud_rate(self):
        """Gets the baud rate of serial communication channel

        Returns
        -------
        baud_rate : int
            Baudrate speed for communication with the device (default = 921600)
        res : tuple
            Raw response from the camera in bytes

        """

        function = GET_BAUD_RATE
        self._send_packet(function)
        res = self._read_packet(function)

        baud_rate = struct.unpack(">h", res[7])[0]
        
        if baud_rate == 0:
            print("BAUD RATE : 0X0000 = Auto baud")
            log_cam.info("BAUD RATE : 0X0000 = Auto baud")
        elif baud_rate == 1:
            print("BAUD RATE : 0X0001 = 9600 baud")
            log_cam.info("BAUD RATE : 0X0001 = 9600 baud")
        elif baud_rate == 2:
            print("BAUD RATE : 0X0002 = 19200 baud")
            log_cam.info("BAUD RATE : 0X0002 = 19200 baud")
        elif baud_rate == 4:
            print("BAUD RATE : 0X0004 = 57600 baud")
            log_cam.info("BAUD RATE : 0X0004 = 57600 baud")
        elif baud_rate == 5:
            print("BAUD RATE : 0X0005 = 115200 baud")
            log_cam.info("BAUD RATE : 0X0005 = 115200 baud")
        elif baud_rate == 6:
            print("BAUD RATE : 0X0006 = 460800 baud")
            log_cam.info("BAUD RATE : 0X0006 = 460800 baud")
        elif baud_rate == 7:
            print("BAUD RATE : 0X0007 = 921600 baud")
            log_cam.info("BAUD RATE : 0X0007 = 921600 baud")

        return baud_rate, res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_gain_mode(self):
        """Gets gain mode of the camera

        Returns
        -------
        gain_mode : int
            Gain mode of the camera (0 = automatic, 1 = low gain, 2 = high gain, 3 = manual)
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = GET_GAIN_MODE
        self._send_packet(function)
        res = self._read_packet(function)

        gain_mode = res[7]
        
        if gain_mode == b'\x00\x00':
            print("GAIN MODE : 0X0000 = Automatic")
            log_cam.info("GAIN MODE : 0X0000 = Automatic")
        elif gain_mode == b'\x00\x01':
            print("GAIN MODE : 0X0001 = Low Gain Only")
            log_cam.info("GAIN MODE : 0X0001 = Low Gain Only")
        elif gain_mode == b'\x00\x02':
            print("GAIN MODE : 0X0002 = High Gain Only")
            log_cam.info("GAIN MODE : 0X0002 = High Gain Only")
        elif gain_mode == b'\x00\x03':
            print("GAIN MODE : 0X0003 = Manual")
            log_cam.info("GAIN MODE : 0X0003 = Manual")

        return gain_mode, res
    
    @_flush_in_out
    @_check_mode('serial')
    def set_gain_mode(self, gain_mode):
        """Sets gain mode of the camera

        Parameters
        ----------
        gain_mode : bytes
            Gain mode of the camera ('automatic', 'low', 'high', 'manual')

        Returns
        -------
        gain_mode : int
            Gain mode of the camera (0 = automatic, 1 = low gain, 2 = high gain, 3 = manual)
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = SET_GAIN_MODE
        argument = gain_mode
        self._send_packet(function, argument)
        res = self._read_packet(function)

        return res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_ace_correct(self):
        """Gets the Active Contrast Enhancement (ACE) Correction for AGC

        Returns
        -------
        ace_correct : int
            Active Contrast Enhancement disabled = 0
        res : tuple
            Raw response from the camera in bytes

        """

        function = GET_AGC_ACE_CORRECT
        argument = None
        self._send_packet(function, argument)
        res = self._read_packet(function)

        ace_correct = struct.unpack(">h", res[7])[0]
        
        if ace_correct == 0:
            print("Active Contrast Enhancement : 0 = disabled")
            log_cam.info("Active Contrast Enhancement : 0 = disabled")
        else:
            print("Active Contrast Enhancement : {} = enabled".format(ace_correct))
            log_cam.info("Active Contrast Enhancement : {} = enabled".format(ace_correct))

        return ace_correct, res

    @_flush_in_out
    @_check_mode('serial')
    def disable_ace_correct(self):
        """Disable the Active Contrast Enhancement (ACE) Correction for AGC

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """

        function = SET_AGC_ACE_CORRECT
        argument = struct.pack(">h", 0x0000)
        self._send_packet(function, argument)
        res = self._read_packet(function)

        return res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_lens_number(self):
        """Get the lens number (which affects which correction terms are applied)

        Returns
        -------
        lens_number : int
            Lens parameters configuration used (0 or 1)
        res : tuple
            Raw response from the camera in bytes

        """

        function = GET_LENS_NUMBER
        argument = None
        self._send_packet(function, argument)
        res = self._read_packet(function)

        lens_number = res[7]
        
        print("Lens number: {}".format(struct.unpack(">h", res[7])[0]))
        log_cam.info("Lens number: {}".format(struct.unpack(">h", res[7])[0]))

        return lens_number, res
    
    @_flush_in_out
    @_check_mode('serial')
    def set_lens_number(self, lens_number):
        """Set the lens number (which affects which correction terms are applied)

        Returns
        -------
        lens_number : int
            Lens parameters configuration used (0 or 1)
        res : tuple
            Raw response from the camera in bytes

        """

        function = SET_LENS_NUMBER
        argument = lens_number
        self._send_packet(function, argument)
        res = self._read_packet(function)

        return res

    @_flush_in_out
    @_check_mode('serial')
    def get_fpa_temperature(self):
        """Get focal plane array (FPA) temperature in degree Celsius

        Returns
        -------
        fpa_temperature : float
            Temperature of the FPA in C
        res : tuple
            Raw response from the camera in bytes

        """

        function = GET_FPA_TEMPERATURE
        argument = struct.pack(">h", 0x00)
        self._send_packet(function, argument)
        res = self._read_packet(function)

        fpa_temperature = struct.unpack(">h", res[7])[0]
        fpa_temperature /= 10.0
        
        print("FPA temp: {}C".format(fpa_temperature))
        log_cam.info("FPA temp: {}C".format(fpa_temperature))
        
        return fpa_temperature, res

    @_flush_in_out
    @_check_mode('serial')
    def get_housing_temperature(self):
        """Get the housing temperature of the camera in degree Celsius

        Returns
        -------
        housing_temperature : float
            Temperature of the FPA in C
        res : tuple
            Raw response from the camera in bytes

        """

        function = GET_HOUSING_TEMPERATURE
        argument = struct.pack(">h", 0x0A)
        self._send_packet(function, argument)
        res = self._read_packet(function)
        
        housing_temperature = struct.unpack(">h", res[7])[0]
        housing_temperature /= 100.0

        print("Housing temp: {}C".format(housing_temperature))
        log_cam.info("Housing temp: {}C".format(housing_temperature))

        return housing_temperature, res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_shutter_temperature(self):
        """Gets the temperature of the shutter (both internal & external) as used for radiometry.

        Returns
        -------
        shutter_temperature : float
            Temperature of the shutter in C
        res : tuple
            Raw response from the camera in bytes

        """

        function = GET_SHUTTER_TEMP
        argument = None
        self._send_packet(function, argument)
        res = self._read_packet(function)

        shutter_temperature = struct.unpack(">h", res[7])[0]
        shutter_temperature /= 100.0
        
        print("SHUTTER temperature = {}C".format(shutter_temperature))
        log_cam.info("SHUTTER temp = {}C".format(shutter_temperature))
        
        return shutter_temperature, res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_shutter_temperature_mode(self):
        """Gets the mode of shutter temperature usage.

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes
        """
        
        function = GET_SHUTTER_TEMP_MODE
        arg1=b'\x00\x01'
        arg2=b'\x00\x00'
        argument=arg1+arg2
        self._send_packet(function, argument)
        res = self._read_packet(function)
        shutter_temperature_mode = res[7]
        
        if shutter_temperature_mode == b'\x00\x00':
            print("SHUTTER TEMP MODE : 0X0000 = User, User specified shutter temperature")
            log_cam.info("SHUTTER TEMP MODE : 0X0000 = User, User specified shutter temperature")
        elif shutter_temperature_mode == b'\x00\x01':
            print("SHUTTER TEMP MODE : 0X0001 = Automatic, calibrated temperatures")
            log_cam.info("SHUTTER TEMP MODE : 0X0001 = Automatic, calibrated temperatures")
        elif shutter_temperature_mode == b'\x00\x02':
            print("SHUTTER TEMP MODE : 0x0002 = Static, shutter-less operation")
            log_cam.info("SHUTTER TEMP MODE : 0x0002 = Static, shutter-less operation")

        return shutter_temperature_mode, res
    
    @_flush_in_out
    @_check_mode('serial')
    def set_shutter_temperature_mode(self, shutter_temperature_mode):
        """Sets the mode of shutter temperature usage.

        Parameters
        ----------
        shutter_temperature_mode : float
            Shutter temperature mode for FFC calibration

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """

        function = SET_SHUTTER_TEMP_MODE
        arg1 = b'\x00\x00'
        arg2 = shutter_temperature_mode
        argument = arg1 + arg2
        self._send_packet(function, argument)
        res = self._read_packet(function)
        
        return res
        
    @_flush_in_out
    @_check_mode('serial')
    def set_shutter_temperature(self, shutter_temperature):
        """Sets the temperature of the shutter (both internal & external) as used for radiometry.

        Parameters
        ----------
        shutter_temperature : float
            Shutter temperature for FFC calibration

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """

        function = SET_SHUTTER_TEMP
        argument = struct.pack(">h", int(shutter_temperature*100))
        self._send_packet(function, argument)
        res = self._read_packet(function)
        
        return res
    
    @_flush_in_out
    @_check_mode('serial')
    def do_ffc_short(self):
        """Execute short flat field correction (FFC)

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        
        print("========== FFC IN PROGRESS ==========")
        log_cam.info("========== FFC IN PROGRESS ==========")
        
        function = DO_FFC_SHORT
        self._send_packet(function)
        res = self._read_packet(function)
        
        print("========== FFC DONE ==========")
        log_cam.info("========== FFC DONE ==========")
        
        return res
        
    @_flush_in_out
    @_check_mode('serial')
    def do_ffc_long(self):
        """Execute long flat field correction (FFC)

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        function = DO_FFC_LONG
        argument = struct.pack(">h", 0x0001)
        self._send_packet(function, argument)
        res = self._read_packet(function)

        ffc_state = struct.unpack(">h", res[7])[0]
        
        if ffc_state == -1:
            print("LONG FFC DONE : 0XFFFF = executed")
            log_cam.info("LONG FFC DONE : 0XFFFF = executed")
            
        return res

    @_flush_in_out
    @_check_mode('serial')
    def get_planck_coefficients(self):

        """Get the Planck coefficients (RBFO) used to convert flux to temperature

        Returns
        -------
        RBFO : tuple
            Planck coefficients
        res : tuple
            Raw response from the camera in bytes

        """

        function = GET_PLANCK_COEFFICIENTS
        argument = struct.pack(">h", 0x0200)
        self._send_packet(function, argument)
        res = self._read_packet(function)
        
        res = res[7]
        
        R = struct.unpack(">I", res[0:4])[0]
        B = struct.unpack(">I", res[4:8])[0]
        F = struct.unpack(">I", res[8:12])[0]
        O = struct.unpack(">i", res[12:16])[0]

        print('R:', R)
        print('B:', B)
        print('F:', F)
        print('O:', O)

        RBFO = (R, B, F, O)

        return RBFO, res

    @_flush_in_out
    @_check_mode('serial')
    def get_ffc_mode(self):
        """Get the FFC mode (manual, automatic or external)

        Returns
        -------
        ffc_mode : int
            FFC mode FFC mode (manual=0, auto=1 or external=2)
        res : tuple
            Raw response from the camera in bytes

        """

        function = GET_FFC_MODE
        argument = None
        self._send_packet(function, argument)
        res = self._read_packet(function)

        ffc_mode = res[7]
        
        if ffc_mode == b'\x00\x00':
            print("FFC MODE : 0X0000 = Manual")
            log_cam.info("FFC MODE : 0X0000 = Manual")
        elif ffc_mode == b'\x00\x01':
            print("FFC MODE : 0X0001 = Automatic")
            log_cam.info("FFC MODE : 0X0001 = Automatic")
        elif ffc_mode == b'\x00\x02':
            print("FFC MODE : 0X0002 =  External")
            log_cam.info("FFC MODE : 0X0002 =  External")

        return ffc_mode, res

    @_flush_in_out
    @_check_mode('serial')
    def set_ffc_mode(self, ffc_mode):
        """Set the FFC mode (manual, automatic or external)

        Parameters
        ----------
        ffc_mode : bytes
            FFC mode (manual, auto or external)

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = SET_FFC_MODE
        argument = ffc_mode
        self._send_packet(function, argument)
        res = self._read_packet(function)

        return res
    
    
    @_flush_in_out
    @_check_mode('serial')
    def get_ffc_frames(self):
        """Get the number of integrated frames during FFC.

        Returns
        -------
        ffc_frames : int
            Number of frames (4 frames=0, 8 frames=1 or 16 frames =2)
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = GET_FFC_NFRAMES
        arg1=b'\x00\x03'
        arg2=b'\x00\x00'
        argument=arg1+arg2
        self._send_packet(function, argument)
        res = self._read_packet(function)
        ffc_frames = res[7]
        
        if ffc_frames == b'\x00\x00':
            print("FFC NFRAMES : 0X0000 = 4 frames")
            log_cam.info("FFC NFRAMES : 0X0000 = 4 frames")
        elif ffc_frames == b'\x00\x01':
            print("FFC NFRAMES : 0X0001 = 8 frames")
            log_cam.info("FFC NFRAMES : 0X0001 = 8 frames")
        elif ffc_frames == b'\x00\x02':
            print("FFC NFRAMES : 0X0002 =  16 frames")
            log_cam.info("FFC NFRAMES : 0X0002 =  16 frames")

        return ffc_frames, res
    
    @_flush_in_out
    @_check_mode('serial')
    def set_ffc_frames(self, ffc_frames):
        """Set the number of integrated frames during FFC.

        Parameters
        -------
        ffc_frames : bytes
            Number of integrated frames during FFC
            
        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = SET_FFC_NFRAMES
        arg1 = b'\x00\x02'
        arg2 = ffc_frames
        argument = arg1+arg2
        self._send_packet(function, argument)
        res = self._read_packet(function)

        return res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_xp_mode(self):
        """Gets the XP Mode

        Returns
        -------
        xp_mode : bytes
            XP mode
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = GET_XP_MODE
        argument = struct.pack(">h", 0x0200)
        self._send_packet(function, argument)
        res = self._read_packet(function)

        xp_mode = res[7]
        
        if xp_mode == b'\x00\x00':
            print("XP MODE : 0x0000 = DISABLED")
            log_cam.info("XP MODE : 0x0000 = DISABLED")
        elif xp_mode == b'\x00\x01':
            print("XP MODE : 0x0001 = BT656")
            log_cam.info("XP MODE : 0x0001 = BT656")
        elif xp_mode == b'\x00\x02':
            print("XP MODE : 0x0002 = CMOS 14-bit w/ 1 discrete")
            log_cam.info("XP MODE : 0x0002 = CMOS 14-bit w/ 1 discrete")
        elif xp_mode == b'\x00\x03':
            print("XP MODE : 0x0003 = CMOS 8-bit w/ 8 discretes")
            log_cam.info("XP MODE : 0x0003 = CMOS 8-bit w/ 8 discretes")
        elif xp_mode == b'\x00\x04':
            print("XP MODE : 0x0004 = CMOS 16-bit")
            log_cam.info("XP MODE : 0x0004 = CMOS 16-bit")

        return xp_mode, res
    
    @_flush_in_out
    @_check_mode('serial')
    def set_xp_mode(self, xp_mode):
        """Sets the XP Mode
        
        Parameters
        -------
        xp_mode : bytes
            XP mode setting value

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = SET_XP_MODE
        arg1 = b'\x03'
        arg2 = xp_mode
        argument = arg1+arg2
        self._send_packet(function, argument)
        res = self._read_packet(function)

        return res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_cmos_bit_depth(self):
        """Gets the CMOS mode Bit Depth (8 or 14bit)

        Returns
        -------
        cmos_bit_depth : bytes
            CMOS bit depth
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = GET_CMOS_BIT_DEPTH
        argument = struct.pack(">h", 0x0800)
        self._send_packet(function, argument)
        res = self._read_packet(function)
        
        cmos_bit_depth = res[7]
        
        if cmos_bit_depth == b'\x00\x00':
            print("CMOS BIT DEPTH : 0x0000 = 14bit")
            log_cam.info("CMOS BIT DEPTH : 0x0000 = 14bit")
        elif cmos_bit_depth == b'\x00\x01':
            print("CMOS BIT DEPTH : 0x0001 = 8bit post-AGC/pre-colorize")
            log_cam.info("CMOS BIT DEPTH : 0x0001 = 8bit post-AGC/pre-colorize")
        elif cmos_bit_depth == b'\x00\x02':
            print("CMOS BIT DEPTH : 0x0002 = 8bit Bayer encoded")
            log_cam.info("CMOS BIT DEPTH : 0x0002 = 8bit Bayer encoded")
        elif cmos_bit_depth == b'\x00\x03':
            print("CMOS BIT DEPTH : 0x0003 = 16bit YCbCr")
            log_cam.info("CMOS BIT DEPTH : 0x0003 = 16bit YCbCr")
        elif cmos_bit_depth == b'\x00\x04':
            print("CMOS BIT DEPTH : 0x0004 = 8bit 2x Clock YCbCr")
            log_cam.info("CMOS BIT DEPTH : 0x0004 = 8bit 2x Clock YCbCr")

        return cmos_bit_depth, res
    

    @_flush_in_out
    @_check_mode('serial')
    def set_cmos_bit_depth(self, cmos_bit_depth):
        """Sets the CMOS mode Bit Depth (8 or 14bit)

        Parameters
        -------
        cmos_bit_depth : bytes
            CMOS byte depth for raw mode
        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = SET_CMOS_BIT_DEPTH
        arg1 = b'\x06'
        arg2 = cmos_bit_depth
        argument = arg1+arg2
        self._send_packet(function, argument)
        res = self._read_packet(function)

        return res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_tlinear_resolution(self):
        """Get the resolution of the TLinear
digital video.

        Returns
        -------
        tlinear_resolution : bytes
            Resolution in TLinear mode
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = GET_TLINEAR_MODE
        argument = struct.pack(">h", 0x0010)
        self._send_packet(function, argument)
        res = self._read_packet(function)

        tlinear_resolution = res[7]
        
        if tlinear_resolution == b'\x00\x00':
            print("TLIN OUTPUT MODE : 0x0000 = Low resolution mode")
            log_cam.info("TLIN OUTPUT MODE : 0x0000 = Low resolution mode")
        elif tlinear_resolution == b'\x00\x01':
            print("TLIN OUTPUT MODE : 0x0001 = High resolution mode")
            log_cam.info("TLIN OUTPUT MODE : 0x0001 = High resolution mode")

        return tlinear_resolution, res
    
    @_flush_in_out
    @_check_mode('serial')
    def set_tlinear_resolution(self, tlinear_resolution):
        """Set the resolution of the TLinear
digital video.

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes
            
        Output Mode:
        0x0000 = Low resolution mode
        (0.4Kelvin/count in 14-bit digital)
        0x0001 = High resolution mode
        (0.04 Kelvin/count in 14-bit digital)
        
        """
        
        function = SET_TLINEAR_MODE
        arg1 = b'\x00\x10'
        arg2 = tlinear_resolution
        argument = arg1+arg2
        self._send_packet(function, argument)
        res = self._read_packet(function)

        return res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_tlinear_mode(self):
        """Get Tlinear status

        Returns
        -------
        tlinear_mode : bytes
            Tlinear mode status (0x0000 = disabled, 0x0001 = enabled)
        res : tuple
            Raw response from the camera in bytes

        In normal mode with TLinear disabled, the Tau camera outputs digital data linear in radiometric flux. In TLinear mode, the Tau camera outputs digital data linear in scene temperature. The TLinear feature applies to the 14-bit CMOS and LVDS channels and is user selectable. Two resolution modes are available and user selectable: high resolution (0.04 Kelvin/count) and low resolution (0.4 Kelvin/count).

        """
        
        function = GET_TLINEAR_MODE
        argument = struct.pack(">h", 0x0040)
        self._send_packet(function, argument)
        res = self._read_packet(function)

        tlinear_mode = res[7]
        
        if tlinear_mode == b'\x00\x00':
            print("TLIN MODE DISABLED : 0x0000 = disabled")
            log_cam.info("TLIN MODE DISABLED : 0x0000 = disabled")
        elif tlinear_mode == b'\x00\x01':
            print("TLIN MODE ENABLED : 0x0001 = enabled")
            log_cam.info("TLIN MODE ENABLED : 0x0001 = enabled")

        return tlinear_mode, res
    
    @_flush_in_out
    @_check_mode('serial')
    def set_tlinear_mode(self, tlinear_mode):
        """Enables/disables TLinear output
        
        Parameters
        -------
        tlinear_mode : bytes
            Enable/disable TLinear mode (0x0000 = disabled, 0x0001 = enabled)

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = SET_TLINEAR_MODE
        arg1 = b'\x00\x40'
        arg2 = tlinear_mode
        argument = arg1+arg2
        self._send_packet(function, argument)
        res = self._read_packet(function)

        return res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_video_standard(self):
        """Gets or sets the video standard (affects frame rate)
        
        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = GET_VIDEO_STANDARD
        argument = None
        self._send_packet(function, argument)
        res = self._read_packet(function)

        video_standard = struct.unpack(">h", res[7])[0]
        
        if video_standard == 0:
            print("VIDEO STANDARD : 0x0000 = NTSC, 30Hz")
            log_cam.info("VIDEO STANDARD : 0x0000 = NTSC, 30Hz")
        elif video_standard == 1:
            print("VIDEO STANDARD : 0x0001 = PAL, 25Hz")
            log_cam.info("VIDEO STANDARD : 0x0001 = PAL, 25Hz")
        elif video_standard == 2:
            print("VIDEO STANDARD : 0x0002 = reserved")
            log_cam.info("VIDEO STANDARD : 0x0002 = reserved")
        elif video_standard == 3:
            print("VIDEO STANDARD : 0x0003 = reserved")
            log_cam.info("VIDEO STANDARD : 0x0003 = reserved")
        elif video_standard == 4:
            print("VIDEO STANDARD : 0x0004 = NTSC, 60Hz")
            log_cam.info("VIDEO STANDARD : 0x0004 = NTSC, 60Hz")
        elif video_standard == 5:
            print("VIDEO STANDARD : 0x0005 = PAL, 50Hz")
            log_cam.info("VIDEO STANDARD : 0x0005 = PAL, 50Hz")

        return res
    
    @_flush_in_out
    @_check_mode('serial')
    def get_lens_parameters(self):
        """Gets lens parameters for the calculated responsivity

        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = GET_LENS_RESPONSE_PARAMS
        argument = struct.pack(">h", 0x0000)
        self._send_packet(function, argument)
        res = self._read_packet(function)
        
        res = b''.join(res)
        focal_ratio = struct.unpack(">h", res[7:9])[0]
        transmission = struct.unpack(">h", res[9:11])[0]
        
        print("Focal ratio: {}".format(focal_ratio))
        print("Transmission: {}".format(transmission))
        log_cam.info("Focal ratio: {}".format(focal_ratio))
        log_cam.info("Transmission: {}".format(transmission))

        return res
        
    @_flush_in_out
    @_check_mode('serial')
    def get_scene_parameters(self):
        """Gets scene parameters for radiometric calculations
        
        Returns
        -------
        res : tuple
            Raw response from the camera in bytes

        """
        
        function = GET_SCENE_PARAMS
        
        d1 = {"RAD_EMISSIVITY": None,
             "RAD_TBKG_X100": None,
             "RAD_TRANSMISSION_WIN": None,
             "RAD_TWIN_X100": None,
             "RAD_TAU_ATM": None,
             "RAD_TATM_X100": None,
             "RAD_REFL_WIN": None,
             "RAD_TREFL_X100": None
             }
        
        d2 = {"RAD_EMISSIVITY": 0x0100,
             "RAD_TBKG_X100": 0x0101,
             "RAD_TRANSMISSION_WIN": 0x0102,
             "RAD_TWIN_X100": 0x0103,
             "RAD_TAU_ATM": 0x0104,
             "RAD_TATM_X100": 0x0105,
             "RAD_REFL_WIN": 0x0106,
             "RAD_TREFL_X100": 0x0107
             }
        
        for key, value in d2.items():
        
            argument = struct.pack(">h", value)
            self._send_packet(function, argument)
            res = self._read_packet(function)
            d1[key] = struct.unpack(">h", res[7])[0]
            print("{} : {}".format(key, d1[key]))

    
    def check_settings(self):
        """Check and set custom settings
        Returns
        -------
        settings_state : bool
            If True, no error
        """
        
        settings_state = True

        # 1. Enable high gain mode only
        _ = self.set_gain_mode(gain_mode=default_settings['gain_mode']['bytes'])
        q_gain_mode, _ = self.get_gain_mode()

        if q_gain_mode != default_settings['gain_mode']['bytes']:
            print("CMD : GAIN MODE IS NOT CONFIGURED PROPERLY")
            log_cam.error("CMD : GAIN MODE IS NOT CONFIGURED PROPERLY")
            settings_state = False
        else:
            print("CMD : GAIN MODE IS CONFIGURED PROPERLY")
            log_cam.info("CMD : GAIN MODE IS CONFIGURED PROPERLY")

        # 2. Check lens number configuration
        """
        _ = self.set_lens_number(lens_number=default_settings['lens_number']['bytes'])
        q_lens_number, _ = self.get_lens_number()

        if q_lens_number != default_settings['lens_number']['bytes']:
            print("CMD : LENS NUMBER IS NOT CONFIGURED PROPERLY")
            log_cam.error("CMD : LENS NUMBER IS NOT CONFIGURED PROPERLY")
            settings_state = False
        else:
            print("CMD : LENS NUMBER IS CONFIGURED PROPERLY")
            log_cam.info("CMD : LENS NUMBER IS CONFIGURED PROPERLY")
        """
            
        # 3. Check shutter temperature mode
        _ = self.set_shutter_temperature_mode(shutter_temperature_mode=default_settings['shutter_temperature_mode']['bytes'])
        q_shutter_temperature_mode, _ = self.get_shutter_temperature_mode()

        if q_shutter_temperature_mode != default_settings['shutter_temperature_mode']['bytes']:
            print("CMD : SHUTTER TEMPERATURE MODE IS NOT CONFIGURED PROPERLY")
            log_cam.error("CMD : SHUTTER TEMPERATURE MODE IS NOT CONFIGURED PROPERLY")
            settings_state = False
        else:
            print("CMD : SHUTTER TEMPERATURE MODE IS CONFIGURED PROPERLY")
            log_cam.info("CMD : SHUTTER TEMPERATURE MODE IS CONFIGURED PROPERLY")

        # 4. Check FFC mode
        _ = self.set_ffc_mode(ffc_mode=default_settings['ffc_mode']['bytes'])
        q_ffc_mode, _ = self.get_ffc_mode()

        if q_ffc_mode != default_settings['ffc_mode']['bytes']:
            print("CMD : FFC MODE IS NOT CONFIGURED PROPERLY")
            log_cam.error("CMD : FFC MODE IS NOT CONFIGURED PROPERLY")
            settings_state = False
        else:
            print("CMD : FFC MODE IS CONFIGURED PROPERLY")
            log_cam.info("CMD : FFC MODE IS CONFIGURED PROPERLY")

        # 5. Check FFC frames
        _ = self.set_ffc_frames(ffc_frames=default_settings['ffc_frames']['bytes'])
        q_ffc_frames, _ = self.get_ffc_frames()

        if q_ffc_frames != default_settings['ffc_frames']['bytes']:
            print("CMD : FFC FRAMES IS NOT CONFIGURED PROPERLY")
            log_cam.error("CMD : FFC FRAMES IS NOT CONFIGURED PROPERLY")
            settings_state = False
        else:
            print("CMD : FFC FRAMES IS CONFIGURED PROPERLY")
            log_cam.info("CMD : FFC FRAMES IS CONFIGURED PROPERLY")

        # 6. Check bit depth
        _ = self.set_xp_mode(xp_mode=default_settings['xp_mode']['bytes'])
        q_xp_mode, _ = self.get_xp_mode()

        if q_xp_mode != default_settings['xp_mode']['return']:
            print("CMD : XP MODE IS NOT CONFIGURED PROPERLY")
            log_cam.error("CMD : XP MODE IS NOT CONFIGURED PROPERLY")
            settings_state = False
        else:
            print("CMD : XP MODE IS CONFIGURED PROPERLY")
            log_cam.info("CMD : XP MODE IS CONFIGURED PROPERLY")

        # 7. Check CMOS bit depth
        _ = self.set_cmos_bit_depth(cmos_bit_depth=default_settings['cmos_bit_depth']['bytes'])
        q_cmos_bit_depth, _ = self.get_cmos_bit_depth()

        if q_cmos_bit_depth != default_settings['cmos_bit_depth']['return']:
            print("CMD : CMOS BITDEPTH IS NOT CONFIGURED PROPERLY")
            log_cam.error("CMD : CMOS BITDEPTH IS NOT CONFIGURED PROPERLY")
            settings_state = False
        else:
            print("CMD : CMOS BITDEPTH IS CONFIGURED PROPERLY")
            log_cam.info("CMD : CMOS BITDEPTH IS CONFIGURED PROPERLY")

        # 8. Check Tlinear Tlinear state
        _ = self.set_tlinear_mode(tlinear_mode=default_settings['tlinear_mode']['bytes'])
        q_tlinear_mode, _ = self.get_tlinear_mode()

        if q_tlinear_mode != default_settings['tlinear_mode']['bytes']:
            print("CMD : TLINEAR MODE IS NOT CONFIGURED PROPERLY")
            log_cam.error("CMD : TLINEAR MODE IS NOT CONFIGURED PROPERLY")
            settings_state = False
        else:
            print("CMD : TLINEAR MODE IS CONFIGURED PROPERLY")
            log_cam.info("CMD : TLINEAR MODE IS CONFIGURED PROPERLY")
            
        return settings_state
    

    @_check_mode('syncff')
    def grab_image(self, fpa_temperature, housing_temperature, blackbody_temperature, ambient_temperature, shutter_temperature, ffc_before, ffc_time,
                   duration=1.0, sequence=1, average=False, display=False):
        """Grab sequence of images for a specific duration period

        Parameters
        ----------
        fpa_temperature : float
            Temperature of Focal Plane Array for FITS headers
        housing_temperature : float
            Temperature of housing electronic of the camera for FITS headers
        shutter_temperature : float
            Temperature of FFC target for FITS headers
        ambient_temperature : float
            Ambient temperature close to the instrument
        ffc_before : bool
            Write if FFC flag in FITS header
        duration : float
            Duration for which the buffer is read
        sequence : int
            Sequence number
        average : bool
            Set to True in order to average images per sequence
        display : bool
            Plot images

        """
        
        for s in range(0, sequence):
        
            data = b''
            
            self._ftdi.purge_buffers()

            log_cam.info("Read buffer for {} seconds".format(duration))
            t = time.time()
            t_format = time.strftime('%Y-%m-%dT%H:%M:%S')
            while time.time()-t < duration: 
                data += self._read()
                
            if len(data)>self.frame_size:
                list_img = self.create_images(data)
                list_img = [i for i in list_img if i is not None]
                self.write_images(list_img, t_format, s, average, fpa_temperature, housing_temperature, blackbody_temperature, ambient_temperature, shutter_temperature, ffc_before, ffc_time)
                if display == True:
                    self.plot_images(list_img)
                # return list_img
            
    def create_images(self, data):
        """Create list of images with raw bytesarray data

        Parameters
        ----------
        data : bytesarray
            Raw data from camera's buffer

        Returns
        -------
        list_img : list
            List of images with the right properties

        """
        
        pos = []
        pos.append(data.find(self.magic_ftdi))
        i=0
        while data.find(self.magic_ftdi, pos[i]+1) != -1:
            pos.append(data.find(self.magic_ftdi, pos[i]+1))
            i=i+1
            
        list_img = []
        for i in range(1, len(pos)):
            if pos[i]-pos[i-1] == self.frame_size:
                array = np.frombuffer(data[pos[i-1]+10:pos[i]], dtype='uint16') # to 16 bits
                array_14bits = array & 0x3FFF # rescale to 14 bits
                array_14bits = array_14bits.reshape(512, 642) # reshape
                array_14bits = np.delete(array_14bits, [0, -1], 1) # remove first and last columns containing zeros
                array_14bits = array_14bits.view('int16') # transform to signed int16
                if 255 in array_14bits:
                    list_img.append(None)
                else:
                    list_img.append(array_14bits)
        
        log_cam.info("Image sequence sliced")
        return list_img

    def plot_images(self, list_img):
        """Plot images

        Parameters
        ----------
        list_img : list
            List of images with the right properties

        """

        for img in list_img:
            if img is not None:
                plt.figure()
                plt.imshow(img)
                plt.colorbar()
            
        plt.show()

    def write_images(self, list_img, date_obs, s, average, fpa_temperature, housing_temperature, blackbody_temperature, ambient_temperature, shutter_temperature, ffc_before, ffc_time):
        """Write images with headers to FITS file format

        Parameters
        ----------
        list_img : list
            List of images with the right properties
        date_obs : str
            Date of observation for FITS headers
        s : int
            Sequence number
        average : bool
            Set to True in order to average images per sequence
        fpa_temperature : float
            Temperature of Focal Plane Array for FITS headers
        housing_temperature : float
            Temperature of housing electronic of the camera for FITS headers
        shutter_temperature : float
            Temperature of FFC target for FITS headers
        ambient_temperature : float
            Ambient temperature close to the instrument
        ffc_before : bool
            Write if FFC flag in FITS header

        """
        if average == False:
            i=0
            for img in list_img:
                hdu = fits.PrimaryHDU(img)
                hdu.scale('int16')
                hdu.header['CAMERA'] = CAMERA
                hdu.header['FILTER'] = FILTER
                hdu.header['FOCAL'] = FOCAL
                hdu.header['APERTURE'] = APERTURE
                hdu.header['PXSIZE'] = PXSIZE
                hdu.header['IMTYPE'] = IMTYPE
                hdu.header['OBSERVER'] = OBSERVER
                hdu.header['TARGET'] = TARGET
                hdu.header['LATOBS'] = LATOBS
                hdu.header['LONGOBS'] = LONGOBS
                hdu.header['DATE-OBS'] = date_obs
                hdu.header['INDEX'] = i
                hdu.header['GAIN'] = default_settings['gain_mode']['value']
                hdu.header['LENS'] = default_settings['lens_number']['value']
                hdu.header['SHU-TEMP'] = shutter_temperature
                hdu.header['SHU-MODE'] = default_settings['shutter_temperature_mode']['value']
                hdu.header['FPA-TEMP'] = fpa_temperature
                hdu.header['IN-TEMP'] = housing_temperature
                hdu.header['BB-TEMP'] = blackbody_temperature
                hdu.header['AMB-TEMP'] = ambient_temperature
                hdu.header['FFC-BEF'] = ffc_before
                hdu.header['FFC-TIME'] = ffc_time
                hdu.header['FFCMODE'] = default_settings['ffc_mode']['value']
                hdu.header['FFCFRAME'] = default_settings['ffc_frames']['value']
                hdu.header['XPMODE'] = default_settings['xp_mode']['value']
                hdu.header['CMOSBD'] = default_settings['cmos_bit_depth']['value']
                hdu.header['TLINEAR'] = default_settings['tlinear_mode']['value']
                hdu.writeto(new_dir+'/'+datetime.now().strftime('%Y_%m_%d_%H_%M_%S_flux_SEQ_{}_N_{}.fits'.format(s, i)), overwrite=True)
                i=i+1
                
        else:
            try:
                average_image = np.sum(list_img, axis=0)/len(list_img)
                average_image = np.round(average_image, 0)
                hdu = fits.PrimaryHDU(average_image)
                hdu.scale('int16')
                hdu.header['CAMERA'] = CAMERA
                hdu.header['FILTER'] = FILTER
                hdu.header['FOCAL'] = FOCAL
                hdu.header['APERTURE'] = APERTURE
                hdu.header['PXSIZE'] = PXSIZE
                hdu.header['IMTYPE'] = IMTYPE
                hdu.header['OBSERVER'] = OBSERVER
                hdu.header['TARGET'] = TARGET
                hdu.header['LATOBS'] = LATOBS
                hdu.header['LONGOBS'] = LONGOBS
                hdu.header['DATE-OBS'] = date_obs
                hdu.header['NAVERAGE'] = len(list_img)
                hdu.header['GAIN'] = default_settings['gain_mode']['value']
                hdu.header['LENS'] = default_settings['lens_number']['value']
                hdu.header['SHU-TEMP'] = shutter_temperature
                hdu.header['SHU-MODE'] = default_settings['shutter_temperature_mode']['value']
                hdu.header['FPA-TEMP'] = fpa_temperature
                hdu.header['IN-TEMP'] = housing_temperature
                hdu.header['BB-TEMP'] = blackbody_temperature
                hdu.header['AMB-TEMP'] = ambient_temperature
                hdu.header['FFC-BEF'] = ffc_before
                hdu.header['FFC-TIME'] = ffc_time
                hdu.header['FFCMODE'] = default_settings['ffc_mode']['value']
                hdu.header['FFCFRAME'] = default_settings['ffc_frames']['value']
                hdu.header['XPMODE'] = default_settings['xp_mode']['value']
                hdu.header['CMOSBD'] = default_settings['cmos_bit_depth']['value']
                hdu.header['TLINEAR'] = default_settings['tlinear_mode']['value']
                hdu.writeto(new_dir+'/'+datetime.now().strftime('%Y_%m_%d_%H_%M_%S_flux.fits'), overwrite=True)
            except:
               log_cam.error("ERROR : Image sequence #{} NOT written to FITS files".format(s))
               print("{} : ERROR : Image sequence #{} NOT written to FITS files".format(date_obs, s))
 
        log_cam.info("Image sequence #{} written to FITS files".format(s))
        print("{} : Image sequence #{} written to FITS files".format(date_obs, s))

    def write_image(self, data, date_obs, fpa_temperature, housing_temperature, blackbody_temperature, ambient_temperature, shutter_temperature, ffc_before, ffc_time):
        
        array = np.frombuffer(data[10:], dtype='uint16') # to 16 bits
        array_14bits = array & 0x3FFF # rescale to 14 bits
        array_14bits = array_14bits.reshape(512, 642) # reshape
        array_14bits = np.delete(array_14bits, [0, -1], 1) # remove first and last columns containing zeros
        array_14bits = array_14bits.view('int16') # transform to signed int16
        
        # if 255 in array_14bits or 0 in array_14bits:
            # log_cam.error("ERROR : Image NOT written to FITS files")
            # print("ERROR : Image NOT written to FITS files")
            # return 0
        
        hdu = fits.PrimaryHDU(array_14bits)
        hdu.scale('int16')
        hdu.header['CAMERA'] = CAMERA
        hdu.header['FILTER'] = FILTER
        hdu.header['FOCAL'] = FOCAL
        hdu.header['APERTURE'] = APERTURE
        hdu.header['PXSIZE'] = PXSIZE
        hdu.header['IMTYPE'] = IMTYPE
        hdu.header['OBSERVER'] = OBSERVER
        hdu.header['TARGET'] = TARGET
        hdu.header['LATOBS'] = LATOBS
        hdu.header['LONGOBS'] = LONGOBS
        hdu.header['DATE-OBS'] = date_obs
        hdu.header['GAIN'] = default_settings['gain_mode']['value']
        hdu.header['LENS'] = default_settings['lens_number']['value']
        hdu.header['SHU-TEMP'] = shutter_temperature
        hdu.header['SHU-MODE'] = default_settings['shutter_temperature_mode']['value']
        hdu.header['FPA-TEMP'] = fpa_temperature
        hdu.header['IN-TEMP'] = housing_temperature
        hdu.header['BB-TEMP'] = blackbody_temperature
        hdu.header['AMB-TEMP'] = ambient_temperature
        hdu.header['FFC-BEF'] = ffc_before
        hdu.header['FFC-TIME'] = ffc_time
        hdu.header['FFCMODE'] = default_settings['ffc_mode']['value']
        hdu.header['FFCFRAME'] = default_settings['ffc_frames']['value']
        hdu.header['XPMODE'] = default_settings['xp_mode']['value']
        hdu.header['CMOSBD'] = default_settings['cmos_bit_depth']['value']
        hdu.header['TLINEAR'] = default_settings['tlinear_mode']['value']
        hdu.writeto(new_dir+'/'+datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f_flux.fits'), overwrite=True)

    @_check_mode('syncff')
    def get_image(self, attempts = 5):
        """Get sequence of images for a specific duration period

        Parameters
        ----------
        fpa_temperature : float
            Temperature of Focal Plane Array for FITS headers
        housing_temperature : float
            Temperature of housing electronic of the camera for FITS headers
        shutter_temperature : float
            Temperature of FFC target for FITS headers
        ambient_temperature : float
            Ambient temperature close to the instrument
        ffc_before : bool
            Write if FFC flag in FITS header
        duration : float
            Duration for which the buffer is read
        sequence : int
            Sequence number
        average : bool
            Set to True in order to average images per sequence
        display : bool
            Plot images

        """
        
        data = self._ftdi.read_data_bytes(self.frame_size, attempts)
        return data
    
    def process_images(self, l_data, l_time, n, fpa_temperature, housing_temperature, blackbody_temperature, ambient_temperature, shutter_temperature, ffc_time):
        """Process and write images with headers to FITS file format

        Parameters
        ----------
        l_data : list
            List of raw data containg bytesarray
        l_time : list
            List of datetime.datetime objects for each image
        n : int
            Number of images to stack
        fpa_temperature : float
            Temperature of Focal Plane Array for FITS headers
        housing_temperature : float
            Temperature of housing electronic of the camera for FITS headers
        shutter_temperature : float
            Temperature of FFC target for FITS headers
        ambient_temperature : float
            Ambient temperature close to the instrument

        """
        
        # Concatenate list of raw data to a single bytearray
        data = b''.join(l_data)
        
        # Find positions in the array where b'TEAX' is located
        pos = []
        pos.append(data.find(self.magic_ftdi))
        i=0
        while data.find(self.magic_ftdi, pos[i]+1) != -1:
            pos.append(data.find(self.magic_ftdi, pos[i]+1))
            i=i+1
            
        # Format images and remove unconsistent data
        list_img = []
        list_timestamp = []
        list_dateobs = []
        for i in range(1, len(pos)):
            if pos[i]-pos[i-1] == self.frame_size:
                
                # Image data
                array = np.frombuffer(data[pos[i-1]+10:pos[i]], dtype='uint16') # to 16 bits
                array_14bits = array & 0x3FFF # rescale to 14 bits
                array_14bits = array_14bits.reshape(512, 642) # reshape
                array_14bits = np.delete(array_14bits, [0, -1], 1) # remove first and last columns containing zeros
                array_14bits = array_14bits.view('int16') # transform to signed int16
                
                # Timestamp
                timestamp = np.frombuffer(data[pos[i-1]:pos[i-1]+10], dtype='uint16') # to 16 bits
                timestamp_14bits = timestamp & 0x3FFF # rescale to 14 bits
                
                # Append to lists
                if 255 not in array_14bits:
                    list_timestamp.append(timestamp_14bits[-1])
                    list_img.append(array_14bits)
                    list_dateobs.append(l_time[i])
                    print(l_time[i].strftime('%Y-%m-%dT%H:%M:%S.%f'))
                    
        limg_chunks = list(zip(*[iter(list_img)]*n))
        ltimestamp_chunks = list(zip(*[iter(list_timestamp)]*n))
        ldateobs_chunks = list(zip(*[iter(list_dateobs)]*n))
        
        for i in range(0, len(limg_chunks)):
            image_concat = [limg_chunks[i][k] for k in range(0, len(limg_chunks[i]))]
            final_image = np.sum(image_concat, axis=0)/n
            mean_time = ldateobs_chunks[i][0]+((ldateobs_chunks[i][-1]-ldateobs_chunks[i][0])/2)
            
            hdu = fits.PrimaryHDU(final_image)
            hdu.scale('int16')
            hdu.header['CAMERA'] = CAMERA
            hdu.header['FILTER'] = FILTER
            hdu.header['FOCAL'] = FOCAL
            hdu.header['APERTURE'] = APERTURE
            hdu.header['PXSIZE'] = PXSIZE
            hdu.header['IMTYPE'] = IMTYPE
            hdu.header['OBSERVER'] = OBSERVER
            hdu.header['TARGET'] = TARGET
            hdu.header['LATOBS'] = LATOBS
            hdu.header['LONGOBS'] = LONGOBS
            hdu.header['DATE-OBS'] = mean_time.strftime('%Y-%m-%dT%H:%M:%S')
            hdu.header['TS'] = np.mean(ltimestamp_chunks[i])
            hdu.header['SHU-TEMP'] = shutter_temperature
            hdu.header['FPA-TEMP'] = fpa_temperature
            hdu.header['IN-TEMP'] = housing_temperature
            hdu.header['BB-TEMP'] = blackbody_temperature
            hdu.header['AMB-TEMP'] = ambient_temperature
            hdu.header['FFC-TIME'] = ffc_time
            hdu.header['GAIN'] = default_settings['gain_mode']['value']
            hdu.header['LENS'] = default_settings['lens_number']['value']
            hdu.header['SHU-MODE'] = default_settings['shutter_temperature_mode']['value']
            hdu.header['FFCMODE'] = default_settings['ffc_mode']['value']
            hdu.header['FFCFRAME'] = default_settings['ffc_frames']['value']
            hdu.header['XPMODE'] = default_settings['xp_mode']['value']
            hdu.header['CMOSBD'] = default_settings['cmos_bit_depth']['value']
            hdu.header['TLINEAR'] = default_settings['tlinear_mode']['value']
            hdu.writeto(new_dir+'/'+mean_time.strftime('%Y_%m_%d_%H_%M_%S_%f_flux.fits'), overwrite=True)
            print('IMAGE {} | {}/{} written to FITS file'.format(mean_time.strftime('%Y_%m_%d_%H_%M_%S_%f_flux.fits'), i+1, len(limg_chunks)))
            

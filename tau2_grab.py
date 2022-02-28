##################################################
## Python class for the FLIR TAU2 camera in FTDI mode
##################################################
## Authors: Kelian SOMMER, Bertrand PLEZ, Johann COHEN-TANUGI
## Credits: flirpy
## Version: 1.0
## Date : 2022-02-23
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

time_sync = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
curr_path = os.getcwd()
new_dir = curr_path+'/'+time_sync
os.mkdir(new_dir)

# Setting up log file
log_acq = logging.getLogger()
log_acq.setLevel(logging.DEBUG)
log_file = time_sync + '_Tau2_ACQ.log'
handler = logging.FileHandler(new_dir+'/'+klog_file, 'w', 'utf-8')
handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
log_acq.addHandler(handler)

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
            self.connect()
            self._sync(allow_timeout=True)
            
    def connect(self):
        """Connect to the FLIR Tau2 camera using FTDI interface
        """
        
        if self.dev.is_kernel_driver_active(0):
            self.dev.detach_kernel_driver(0)
        
        self._claim_dev()
        
        self._ftdi = Ftdi()
        self._ftdi.open_from_device(self.dev)
        
        if self._ftdi.is_connected == True:
            print("========== ACQ : Connected to the FLIR Tau2 camera ==========")
            log_acq.info("========== ACQ : Connected to the FLIR Tau2 camera ==========")
            self._ftdi.set_bitmode(0xFF, Ftdi.BitMode.RESET)
            self._ftdi.set_bitmode(0xFF, Ftdi.BitMode.SYNCFF)
            # self._ftdi.set_baudrate(baudrate=12e6) # 12MBaud maximum of FT2232H
            self._ftdi.purge_buffers()
        else:
            print("========== ACQ : An error occurred while trying to establish connection with FLIR TAU2 camera with FTDI protocol ==========")
            log_acq.error("========== ACQ : An error occurred while trying to establish connection with FLIR TAU2 camera with FTDI protocol ==========")
        
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
                        log_acq.error("ACQ : Could not detatch kernel driver from interface({0}): {1}".format(intf.bInterfaceNumber, str(e)))
    
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
                log_acq.warn("Timeout in frame sync")
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
                log_acq.warn("Timeout in command sync")
                break
            elif time.time() -t > 0.2:
                break
        
        return data[data.find(self.magic_uart):]
        
    def _send_data(self, data):
        """Send data to the device

        Parameters
        ----------
        data : bytesarray
            Input data to write to the device

        """
                
        # header
        buffer = b"UART"
        buffer += int(len(data)).to_bytes(1, byteorder='big') # doesn't matter
        buffer += data
        self._ftdi.write_data(buffer)
        
    def _receive_data(self, length):
        """Sync output buffer with UART magic keyword

        Parameters
        ----------
        length : int
            Length of data to receive

        Returns
        -------
        data[:length][5::6] : bytesarray
            Raw response from the camera in bytes

        """
        
        length += length*5
        
        data = self._sync_uart()
        
        if len(data) < length:
            data += self._read(n_bytes=length-len(data))
            
            
        return data[:length][5::6]
    
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
        log_acq.info("========== ACQ : Disconnecting from camera ==========")
        
    def grab_image(self, duration=1.0, display=False):
        """Grab sequence of images for a specific duration period

        Parameters
        ----------
        duration : float
            Duration for which the buffer is read
        display : bool
            Plot images

        Returns
        -------
        list_img : list
            List of images with the right properties

        """
        
        data = b''
        
        self._ftdi.purge_buffers()
        # self._ftdi.purge_rx_buffer()
        # self._ftdi.purge_tx_buffer()
        
        t = time.time()
        t_format = time.strftime('%Y-%m-%dT%H:%M:%S')
        while time.time()-t < duration: 
            data += self._read()
            
        if len(data)>self.frame_size:
            list_img = self.create_images(data)
            self.write_images(list_img, t_format)
            if display == True:
                self.plot_images(list_img)
            return list_img
            
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

    def write_images(self, list_img, date_obs):
        """Write images with headers to FITS file format

        Parameters
        ----------
        list_img : list
            List of images with the right properties

        """

        i=0
        for img in list_img:
            if img is not None:
                hdu = fits.PrimaryHDU(img)
                hdu.scale('int16')
                hdu.header['DATE-OBS'] = date_obs
                hdu.header['INDEX'] = i
                hdu.header['GAIN'] = default_settings['gain_mode']['value']
                hdu.header['LENS'] = default_settings['lens_number']['value']
                hdu.header['SHU-TEMP'] = default_settings['shutter_temperature']['value']
                hdu.header['FFCMODE'] = default_settings['ffc_mode']['value']
                hdu.header['FFCFRAME'] = default_settings['ffc_frames']['value']
                hdu.header['XPMODE'] = default_settings['xp_mode']['value']
                hdu.header['CMOSBD'] = default_settings['cmos_bit_depth']['value']
                hdu.header['TLINEAR'] = default_settings['tlinear_mode']['value']
                hdu.writeto(datetime.now().strftime('%Y_%m_%d_%H_%M_%S_flux_{}.fits'.format(i)), overwrite=True)
                i=i+1

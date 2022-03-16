import smbus
import time
from datetime import datetime
 
# Get I2C bus
bus = smbus.SMBus(1)
# Sensor addresses
address_1 = 0x4A
address_2 = 0x4B

class TemperatureSensors(object):
    """
    Class for temperature reading with two STS35 sensors
    """
    
    def __init__(self, address_1=0x4A, address_2=0x4B):
        """Initialize addresses

        Parameters
        ----------
        address_1 : bytes
            Address of sensor #1
        address_2 : bytes
            Address of sensor #2

        """
        
        self.address_1 = address_1
        self.address_2 = address_2
        
    def read_temperature(self, display=True):
        """Read temperatures from both sensors
        
        Parameters
        ----------
        display : bool
            Choose to display temperature in cmd
            
        Returns
        ------
        cTemp1 : float
            Temperature from sensor #1 in Celsius degrees
        cTemp2 : float
            Temperature from sensor #2 in Celsius degrees
        """
        
        bus.write_i2c_block_data(self.address_1, 0x2C, [0x06])
        time.sleep(0.02)
        # Read data back from 0x00(00), 6 bytes
        data1 = bus.read_i2c_block_data(self.address_1, 0x00, 6)
        # Convert the data
        temp1 = data1[0] * 256 + data1[1]
        cTemp1 = -45 + (175 * temp1 / 65535.0)
        
        # STS35 address, 0x4B
        bus.write_i2c_block_data(self.address_2, 0x2C, [0x06])
        time.sleep(0.02)
        # Read data back from 0x00(00), 6 bytes
        data2 = bus.read_i2c_block_data(self.address_2, 0x00, 6)
        # Convert the data
        temp2 = data2[0] * 256 + data2[1]
        cTemp2 = -45 + (175 * temp2 / 65535.0)
        
        if display == True:
            print("Temperature 1 in Celsius is : %.2f C" %cTemp1)
            print("Temperature 2 in Celsius is : %.2f C" %cTemp2)
        
        return round(cTemp1, 2), round(cTemp2, 2)

 

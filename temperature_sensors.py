import smbus
import time
from datetime import datetime
 
# Get I2C bus
bus = smbus.SMBus(1)

filename = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

with open("/home/pi/Desktop/{}.csv".format(filename), "a") as log:
    
    while True:
        
        # STS35 address, 0x4A
        bus.write_i2c_block_data(0x4A, 0x2C, [0x06])
        time.sleep(0.02)
        # Read data back from 0x00(00), 6 bytes
        data1 = bus.read_i2c_block_data(0x4A, 0x00, 6)
         
        # Convert the data
        temp1 = data1[0] * 256 + data1[1]
        cTemp1 = -45 + (175 * temp1 / 65535.0)
         
        # Output data to screen
        print("Temperature 1 in Celsius is : %.2f C" %cTemp1)
        # print(type(cTemp1))
        
        # STS35 address, 0x4B
        bus.write_i2c_block_data(0x4B, 0x2C, [0x06])
        time.sleep(0.02)
        # Read data back from 0x00(00), 6 bytes
        data2 = bus.read_i2c_block_data(0x4B, 0x00, 6)
         
        # Convert the data
        temp2 = data2[0] * 256 + data2[1]
        cTemp2 = -45 + (175 * temp2 / 65535.0)
         
        # Output data to screen
        print("Temperature 2 in Celsius is : %.2f C" %cTemp2)
        # print(type(cTemp2))
        
        log.write("{0},{1},{2}\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S"),float(cTemp1), float(cTemp2)))
        
        time.sleep(1)

from tau2_instructions import *
from tau2_class import *
from tau2_grab import *
import sys

try:
    NSEQ = int(sys.argv[1])
except:
    print("Error parsing NSEQ")
# NSEQ = 5000

# Loop
# Instantiate control class in serial mode
Tau2 = FLIR_Tau2()
settings_state = Tau2.check_settings()
fpa_temperature, _ = Tau2.get_fpa_temperature()
housing_temperature, _ = Tau2.get_housing_temperature()
Tau2.ser.close()

if settings_state == True:
    print("CMD : ALL PARAMETERS SET, CAN START ACQUISITION")
    log_cmd.info("CMD : ALL PARAMETERS SET, CAN START ACQUISITION")

    if Tau2.ser.is_open == False:
        
        camera = TeaxGrabber()
        
        if camera._ftdi.is_connected == True:

            for i in range(0, NSEQ):
                
                print("SEQUENCE #{} of {}".format(i+1, NSEQ))
                camera.grab_image(fpa_temperature, housing_temperature, duration=1.0, sequence=1, average=True, display=False)
                
                if i>0 and i%10==0:
                    
                    camera.close()
                    time.sleep(0.1)
                    
                    if camera._ftdi.is_connected == False:
                        try:
                            Tau2.ser.port = '/dev/ttyUSB0'
                            Tau2.ser.open()
                        except:
                            Tau2.ser.port = '/dev/ttyUSB1'
                            Tau2.ser.open()
                            
                        fpa_temperature, _ = Tau2.get_fpa_temperature()
                        housing_temperature, _ = Tau2.get_housing_temperature()
                        Tau2.ser.close()
                        time.sleep(0.1)
                        camera.connect()
                        
                if i>0 and i%100==0:
                    
                    camera.close()
                    time.sleep(0.1)
                    
                    if camera._ftdi.is_connected == False:
                        try:
                            Tau2.ser.port = '/dev/ttyUSB0'
                            Tau2.ser.open()
                        except:
                            Tau2.ser.port = '/dev/ttyUSB1'
                            Tau2.ser.open()
                        
                        Tau2.do_ffc_long()
                        print("FFC IN PROGRES...")
                        time.sleep(0.5)
                        Tau2.ser.close()
                        time.sleep(0.1)
                        camera.connect()
                    

            camera.close()
            
else:
    print("CMD : AN ERROR OCCURED WHILE SETTING UP PARAMETERS")
    log_cmd.error("CMD : AN ERROR OCCURED WHILE SETTING UP PARAMETERS")

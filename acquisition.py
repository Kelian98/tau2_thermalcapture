from tau2_instructions import *
from tau2_class import *
from tau2_grab import *
from sensors_class import *
import sys
from influxdb import InfluxDBClient

client = InfluxDBClient('localhost', 8086)
client.switch_database('stardiceir')

try:
    NSEQ = int(sys.argv[1])
    FFC_interval = int(sys.argv[2])
except:
    print("Error parsing args")

# Instantiate sensors_class
STS35 = TemperatureSensors()
blackbody_temperature, ambient_temperature = STS35.read_temperature()

# Instantiate control class in serial mode
Tau2 = FLIR_Tau2()
settings_state = Tau2.check_settings()
fpa_temperature, _ = Tau2.get_fpa_temperature()
housing_temperature, _ = Tau2.get_housing_temperature()
q_tshutter, _ = Tau2.get_shutter_temperature()

k=0
# Set shutter temperature to blackbody_temperature
while float(q_tshutter) != blackbody_temperature:
    print("SHUTTER TEMPERATURE HAS NOT BEEN SET, RETRY #{}".format(k+1))
    Tau2.set_shutter_temperature(blackbody_temperature)
    q_tshutter, _ = Tau2.get_shutter_temperature()
    print("SHUTTER SENSOR TEMPERATURE = {}".format(blackbody_temperature))
    k=k+1
    if k>5:
        break

# Do initial FFC
Tau2.do_ffc_short()
Tau2.ser.close()

if settings_state == True:
    print("CMD : ALL PARAMETERS SET, CAN START ACQUISITION")
    log_cmd.info("CMD : ALL PARAMETERS SET, CAN START ACQUISITION")

    if Tau2.ser.is_open == False:
        
        camera = TeaxGrabber()
        
        if camera._ftdi.is_connected == True:
            
            ffc_before = True

            for i in range(0, NSEQ):
                
                print("SEQUENCE #{} of {}".format(i+1, NSEQ))
                camera.grab_image(fpa_temperature, housing_temperature, blackbody_temperature, ambient_temperature,
                                  ffc_before, duration=1.0, sequence=1, average=True, display=False)
                ffc_before = False
                
                if i>0 and i%10==0:
                    
                    blackbody_temperature, ambient_temperature = STS35.read_temperature(display=True)
                    time.sleep(0.1)
                    
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
                        
                        if i%FFC_interval==0:
                            Tau2.do_ffc_short()
                            time.sleep(1.0)
                            ffc_before = True
                            
                        Tau2.ser.close()
                        time.sleep(0.1)
                        camera.connect()
                        
                    curr_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    curr_time = curr_time+'Z'
                    json_body = [
                            {
                                "measurement": "blackbody_temperature",
                                "fields": {
                                    "value": blackbody_temperature
                                }
                            },
                            {
                                "measurement": "ambient_temperature",
                                "fields": {
                                    "value": ambient_temperature
                                }
                            },
                            {
                                "measurement": "fpa_temperature",
                                "fields": {
                                    "value": fpa_temperature
                                }
                            },
                            {
                                "measurement": "housing_temperature",
                                "fields": {
                                    "value": housing_temperature
                                }
                            }
                        ]
                    client.write_points(json_body)

            camera.close()
            
else:
    print("CMD : AN ERROR OCCURED WHILE SETTING UP PARAMETERS")
    log_cmd.error("CMD : AN ERROR OCCURED WHILE SETTING UP PARAMETERS")

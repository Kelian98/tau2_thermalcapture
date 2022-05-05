from tau2_instructions import *
from tau2 import *
from sensors_class import *
import sys
from multiprocessing import Process

try:
    NSEQ = int(sys.argv[1])
    FFC_interval = int(sys.argv[2])
except:
    print("Error parsing args")

# Instantiate sensors_class
sts35 = TemperatureSensors()
camera = TeaxGrabber()

camera.set_mode('syncff')
blackbody_temperature, ambient_temperature = sts35.read_temperature(display=False)

i=0

def func1(l_data, l_time, n, fpa_temperature, housing_temperature, blackbody_temperature, ambient_temperature, shutter_temperature, ffc_time, pid):
    print('start writing {}'.format(pid))
    camera.process_images(l_data, l_time, n, fpa_temperature, housing_temperature, blackbody_temperature, ambient_temperature, shutter_temperature, ffc_time)
    print('finished writing {}'.format(pid))
    
while True:
    print('Start run {}'.format(i))
    t_ini = datetime.now()
    l_data = []
    l_time = []
    while (datetime.now()-t_ini).total_seconds() < 30:
        t2 = datetime.now()
        d = camera._ftdi.read_data(camera.frame_size)
        l_data.append(d)
        l_time.append(t2)
        time.sleep(0.02)
        # print(t2.strftime('%Y-%m-%dT%H:%M:%S.%f'))
    print('End run {}'.format(i))
    camera.set_mode('serial')
    fpa_temperature, _ = camera.get_fpa_temperature()
    housing_temperature, _ = camera.get_housing_temperature()
    ffc_time = datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
    blackbody_temperature, ambient_temperature = sts35.read_temperature(display=False)
    k=0
    q_tshutter, _ = camera.get_shutter_temperature()
    while abs(float(q_tshutter) - blackbody_temperature) > 0.02:
        print("SHUTTER TEMPERATURE HAS NOT BEEN SET, RETRY #{}".format(k+1))
        camera.set_shutter_temperature(blackbody_temperature)
        time.sleep(0.1)
        q_tshutter, _ = camera.get_shutter_temperature()
        print("READ SHUTTER TEMPERATURE FROM SENSOR = {}".format(blackbody_temperature))
        print("QUERIED SHUTTER TEMPERATURE FROM CAMERA = {}".format(q_tshutter))
        k=k+1
    shutter_temperature = q_tshutter
    camera.do_ffc_short()
    time.sleep(0.1)
    res = camera.dev.read(0x81, 512)
    camera.set_mode('syncff')
    d = camera.get_image()
    
    p1 = Process(target=func1, args=(l_data, l_time, 5, fpa_temperature, housing_temperature, blackbody_temperature, ambient_temperature, shutter_temperature, ffc_time, i,))
    p1.start()
    i=i+1

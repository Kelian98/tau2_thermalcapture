from tau2_instructions import *
from tau2_class import *
from tau2_grab import *

# Instantiate control class in serial mode
Tau2 = FLIR_Tau2()
settings_state = Tau2.check_settings()

if settings_state == True:
    print("CMD : ALL PARAMETERS SET, CAN START ACQUISITION")
    log_cmd.info("CMD : ALL PARAMETERS SET, CAN START ACQUISITION")
    fpa_temperature, _ = Tau2.get_fpa_temperature()
    housing_temperature, _ = Tau2.get_housing_temperature()
    Tau2.__exit__()
    # Instantiate acquisition class
    camera = TeaxGrabber()
    camera.grab_image(fpa_temperature, housing_temperature, duration=1.0, sequence=5, average=True, display=False)
    camera.__exit__()
else:
    print("CMD : AN ERROR OCCURED WHILE SETTING UP PARAMETERS")
    log_cmd.error("CMD : AN ERROR OCCURED WHILE SETTING UP PARAMETERS")

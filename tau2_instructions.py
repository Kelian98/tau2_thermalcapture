##################################################
## Python class for the FLIR TAU2 camera w/ ThermalCapture TeaX interface
## 2 modes of operation : serial for command/control and ftdi for image acquisition
##################################################
## Authors: Kelian SOMMER, Bertrand PLEZ, Johann COHEN-TANUGI
## Credits: flirpy
## Version: 1.2
## Date : 2022-07-28
##################################################

class code(object):
    """Class for cmd requests to the FLIR Tau2 Camera"""
    def __init__(self, code = 0, cmd_bytes = 0, reply_bytes = 0):
        self.code = code # function code
        self.cmd_bytes = cmd_bytes # byte count get/set cmd
        self.reply_bytes = reply_bytes # byte count reply

# ========== Tau2 commands from official software IDD ========== #

# Tau Status codes
CAM_OK = 0x00
CAM_NOT_READY = 0x02
CAM_RANGE_ERROR = 0x03
CAM_UNDEFINED_ERROR = 0x04
CAM_UNDEFINED_PROCESS_ERROR = 0x05
CAM_UNDEFINED_FUNCTION_ERROR = 0x06
CAM_TIMEOUT_ERROR = 0x07
CAM_BYTE_COUNT_ERROR = 0x09
CAM_FEATURE_NOT_ENABLED= 0x0A

# General Commands
NO_OP = code(0x00, 0, 0)
SET_DEFAULTS = code(0x01, 0, 0)
CAMERA_RESET = code(0x02, 0, 0)
RESTORE_FACTORY_DEFAULTS = code(0x03, 0, 0)
GET_SERIAL_NUMBER = code(0x04, 0, 8)
GET_REVISION = code(0x05, 0, 8)
GET_BAUD_RATE = code(0x07, 0, 2)
SET_BAUD_RATE = code(0x07, 2, 2)

# Gain Commands
GET_GAIN_MODE = code(0x0A, 0, 2)
SET_GAIN_MODE = code(0x0A, 2, 2)

# FFC Commands
GET_FFC_MODE = code(0x0B, 0, 2)
SET_FFC_MODE = code(0x0B, 2, 2)
GET_FFC_NFRAMES = code(0x0B, 4, 2)
SET_FFC_NFRAMES = code(0x0B, 4, 0)
DO_FFC_SHORT = code(0x0C, 0, 0)
DO_FFC_LONG = code(0x0C, 2, 2)
GET_FFC_PERIOD = code(0x0D, 0, 4)
SET_FFC_PERIOD_LOW_GAIN = code(0x0D, 2, 2)
SET_FFC_PERIOD_HIGH_GAIN = code(0x0D, 2, 2)
SET_FFC_PERIOD = code(0x0D, 4, 4)
GET_FFC_TEMP_DELTA = code(0x0E, 0, 4)
SET_FFC_TEMP_DELTA_LOW_GAIN = code(0x0E, 2, 2)
SET_FFC_TEMP_DELTA_HIGH_GAIN = code(0x0E, 2, 2)
SET_FFC_TEMP_DELTA = code(0x0E, 4, 4)
GET_FFC_WARN_TIME = code(0x3C, 0, 2)
SET_FFC_WARN_TIME = code(0x3C, 2, 2)
WRITE_NVFFC_TABLE = code(0xC6, 0, 0)

# Image processing
GET_DIGITAL_OUTPUT_MODE = code(0x12, 0, 2)
SET_DIGITAL_OUTPUT_MODE = code(0x12, 2, 2)
GET_XP_MODE = code(0x12, 2, 2)
SET_XP_MODE = code(0x12, 2, 2)
GET_CMOS_BIT_DEPTH = code(0x12, 2, 2)
SET_CMOS_BIT_DEPTH = code(0x12, 2, 2)
SET_AGC_ACE_CORRECT = code(0x1C, 2, 0)
GET_AGC_ACE_CORRECT = code(0x1C, 0, 2)

# Lens
SET_LENS_NUMBER = code(0x1E, 2, 2)
GET_LENS_NUMBER = code(0x1E, 0, 2)

# Onboard sensors
GET_FPA_TEMPERATURE = code(0x20, 2, 2)
GET_HOUSING_TEMPERATURE = code(0x20, 2, 2)

# Shutter
GET_SHUTTER_TEMP = code(0x4D, 0, 2)
SET_SHUTTER_TEMP = code(0x4D, 2, 0)
GET_SHUTTER_TEMP_MODE = code(0x4D, 4, 2)
SET_SHUTTER_TEMP_MODE = code(0x4D, 4, 0)

# TLinear
GET_TLINEAR_MODE = code(0x8E, 2, 2)
SET_TLINEAR_MODE = code(0x8E, 4, 0)

# Lens response parameters
GET_LENS_RESPONSE_PARAMS = code(0xE5, 2, 4)
SET_LENS_RESPONSE_PARAMS = code(0xE5, 6, 0)
GET_SCENE_PARAMS = code(0xE5, 2, 2)
SET_SCENE_PARAMS = code(0xE5, 4, 0)

# Radiometry
GET_PLANCK_COEFFICIENTS = code(0xB9, 2, 16)
SET_PLANCK_COEFFICIENTS = code(0xB9, 18, 18)


# Default configuration settings
default_settings = {"gain_mode" :
                        {"value" : 'high',
                         "bytes" : b'\x00\x02',
                         "comment" : 'High Gain Only'
                        },
                    "lens_number" :
                        {"value" : 0,
                         "bytes" : b'\x00\x00',
                         "comment" : 'Lens 0 configuration'
                        },
                    "shutter_temperature_mode" :
                        {"value" : 'user',
                         "bytes" : b'\x00\x00',
                         "comment" : '0x0000 = User, User specified shutter temperature'
                        },
                    "ffc_mode" :
                        {"value" : 'manual',
                         "bytes" : b'\x00\x00',
                         "comment" : 'FFC in manual mode'
                        },
                    "ffc_frames" :
                        {"value" : 16,
                         "bytes" : b'\x00\x02',
                         "comment" : '16 frames averaged for FFC'
                        },
                    "xp_mode" :
                        {"value" : 'CMOS 14 bits',
                         "bytes" : b'\x02',
                         "return" : b'\x00\x02',
                         "comment" : 'CMOS 14-bit'
                        },
                    "cmos_bit_depth" :
                        {"value" : 'CMOS 14 bits',
                         "bytes" : b'\x00',
                         "return" : b'\x00\x00',
                         "comment" : 'CMOS 14-bit'
                        },
                    "tlinear_mode":
                        {"value" : 'disabled',
                         "bytes" : b'\x00\x00',
                         "comment" : 'Tlinear mode disabled'
                        }
                    }


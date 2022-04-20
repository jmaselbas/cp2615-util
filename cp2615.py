# SPDX-License-Identifier: BSD-2-Clause
from enum import Enum

def encode_static(cfg, dft):
    return dft

def encode_config(cfg, arg):
    key = arg[0]
    dft = arg[1]
    if key in cfg:
        return cfg[key]
    else:
        return dft

def encode_io_options(cfg, _):
    """Encode the bits for the IO options byte"""
    # CP2615_A01 requires ENABLE_EAPROTO_IO flag always set to work around
    # an interface mapping issue
    # CP2615_A02 adds new flag ENABLE_BULK_ITF to force the IO interface on 
    # without requiring serial or io protocols
    if is_rev_a01(cfg):
        mask = 0x10
    else:
        mask = 0x02
    if cfg['serial_protocol_enabled']:
        mask |= 0x0c
    if cfg['io_protocol_enabled']:
        mask |= 0x10
    return mask

def bitmask(bit):
    mask = 0
    i = 0
    for b in bit:
        mask |= 1 << i if b else 0
        i += 1
    return mask

def encode_pin_mask(cfg, arg):
    gpio = cfg['gpios']
    mask = 0
    if arg == 'output':
        # 0: input;  1: output
        mask = bitmask(gpio[i][0] != GPIO_Mode.INPUT for i in range(len(gpio)))
    elif arg == 'pushpull':
        # 0: open-drain;  1: push-pull
        mask = bitmask(gpio[i][0] == GPIO_Mode.PUSH_PULL for i in range(len(gpio)))
    elif arg == 'gpio':
        # 0: alt function;  1: gpio
        mask = bitmask(gpio[i][1] == GPIO_Function.GPIO for i in range(len(gpio)))
    elif arg == 'reset':
        mask = bitmask(gpio[i][2] for i in range(len(gpio)))
    return mask

#def encode_string(d, arg):
#    l=arg[0]
#    k=arg[1]
#    if k in d and d[k] != None:

############################################################
# pack

def pack_u8(buffer, offset, value):
    return pack_into('B', buffer, offset, value)

def pack_be16(buffer, offset, value):
    return pack_into('>H', buffer, offset, value)

def pack_be32(buffer, offset, value):
    return pack_into('>L', buffer, offset, value)

def pack_array(buffer, offset, value):
    return pack_into('s', buffer, offset, value)

def pack_var_array(buffer, offset, value):
    array_size = len(value)
    array_offset = len(buffer) - offset;
    pack_into('>H>H', buffer, offset, array_offset, array_size)
    pack_into('s', buffer, len(buffer), value)

u8=pack_u8
be16=pack_be16
be32=pack_be32
array=pack_array
varray=pack_var_array

cp2615_fields = [
    (0x00,  4, array, 'cookie',            encode_static, b'2614'),
    (0x04,  1,    u8, 'version',           encode_static, 1),
1    (0x05,  1,    u8, 'configLockKey',     encode_config, ('cfg_is_lock', 0xff)),
    (0x06, 34, array, 'serialStringUtf8Usb', encode_string, (34, 'usb_custom_sn')),
    (0x28,  2,  be16, 'checksum',          encode_static, 0),
    (0x2a,  2,  be16, 'length',            encode_static, 0),
    (0x2c,  2,  be16, 'optionID',          encode_config, ('option_id', 0)),
    (0x2e,  1,    u8, 'debugMode',         encode_static, 0),
    (0x2f,  1,    u8, 'defaultSampleRate', encode_config, ('default_sample_rate', 44)),
    (0x30,  1,    u8, 'clockingConfig',    encode_clocking, None),
    (0x31,  1,    u8, 'audioOpts',         encode_audio_opts, None),
    (0x32,  2,  be16, 'volumeAndMask',     encode_config, ('vol_and_mask', 0)),
    (0x34,  2,  be16, 'volumeOrMask',      encode_config, ('vol_or_mask', 0)),
    (0x36,  1,    u8, 'volumeBitStartPos', encode_config, ('vol_bit_start', 0)),
    (0x37,  1,    u8, 'usbPlaybackFeatureUnitVolumeMasterDefaultDb',
                                           encode_config, ('vol_default_master', 0)),
    (0x38,  1,    u8, 'usbPlaybackFeatureUnitVolumeLeftDefaultDb',
                                           encode_config, ('vol_default_left', 0)),
    (0x39,  1,    u8, 'usbPlaybackFeatureUnitVolumeRightDefaultDb',
                                           encode_config, ('vol_default_right', 0)),
    (0x3a,  1,    u8, 'usbPlaybackFeatureUnitVolumeMinDb',
                                           encode_config, ('vol_min', 0)),
    (0x3b,  1,    u8, 'usbPlaybackFeatureUnitVolumeMinCounts',
                                           encode_config, ('vol_min_counts', 0)),
    (0x3c,  1,    u8, 'usbPlaybackFeatureUnitVolumeMaxDb',
                                           encode_config, ('vol_max', 0)),
    (0x3d,  1,    u8, 'usbPlaybackFeatureUnitVolumeMaxCounts',
                                           encode_config, ('vol_max_counts', 0)),
    (0x3e,  2,  be16, 'usbPlaybackFeatureUnitVolumeResDbX256',
                                           encode_config, ('vol_resolution', 0)),
    (0x40,  1,    u8, 'muteConfig',        encode_mute_config, None),
    (0x41,  2,  be16, 'muteMask',          encode_config, ('mute_mask', 0)),
    (0x43,  2,  be16, 'volumeSettingForMute', encode_static, 0),
    (0x45,  1,    u8, 'ioOptions',         encode_io_options, None),
    (0x46,  1,    u8, 'unused-1',          encode_static, 0),
    (0x47,  4,varray, 'unused-2',          encode_static, b'N/A\x00'),
    (0x4b,  2,  be16, 'pinGpioMask',       encode_pin_mask, 'gpio'),
    (0x4d,  2,  be16, 'pinDirMask',        encode_pin_mask, 'output'),
    (0x4f,  2,  be16, 'pinModeMask',       encode_pin_mask, 'pushpull'),
    (0x51,  2,  be16, 'pinInitValue',      encode_pin_mask, 'reset'),
    (0x53,  8, array, 'gpioAltFunctions',    encode_alt_functions, b'\xff' * 8),
    (0x5b, 16, array, 'buttonsConfiguration', encode_buttons, b'\x00' * 16),
    (0x6b,  4, be32, 'serialDataRate', encode_serial_rate, None),
    (0x6f,  1,    u8, 'clkoutDivider', encode_config, ('clkout_divider', 1)),
    (0x70,  1,    u8, 'powerConfig',   encode_power_config, None),
    (0x71,  2,  be16, 'unused-3', encode_static, 0),
    (0x73,  2,  be16, 'unused-4', encode_static, 0),
    (0x75,  1,    u8, 'unused-5', encode_static, 0),
    (0x76,  1,    u8, 'unused-6', encode_static, 0),
    (0x77,  4, varray, 'usbDeviceDescriptor',        encode_usb_descriptor, 'device'),
    (0x7b,  4, varray, 'usbConfigurationDescriptor', encode_usb_descriptor, 'config'),
    (0x7f,  2,   be16, 'usbLanguageCode',            encode_config, ('usb_lang', 0x0409)),
    (0x81,  4, varray, 'manufacturerStringUtf8Usb',  encode_utf8usb, 'mfr'),
    (0x85,  4, varray, 'productStringUtf8Usb',       encode_utf8usb, 'prod'),
    (0x89,  4, varray, 'serProtocolStringUtf8Usb',   encode_utf8usb, 'proto_name'),
    (0x8d,  4, varray, 'unused-7', encode_static, b'\x00\x05\x00\x00\x00'),
    (0x91,  4, varray, 'unused-8', encode_static, b'\xff\xff'),
    (0x95,  4, varray, 'unused-9', encode_static, b'\xff\xff'),
    (0x99,  1,    u8, 'gestureDownTicks', encode_static, 40),
    (0x9a,  1,    u8, 'gestureUpTicks', encode_static, 9),
    (0x9b,  4, array, 'gestureButtons', '\x00' * 4),
    (0x9f, 10, array, 'spareConfigElements', '\xff' * 10),
    (0xa9,  1,    u8, 'delayFromStandbyDeassertToCodecInitMs', encode_config, ('i2c_delay', 2)),
    (0xaa,  4, varray, 'i2cCmdStrCodecInit',            encode_i2c_cmdstr, 'codec_init'),
    (0xae,  4, varray, 'i2cCmdStrCodecHighToLow',       encode_i2c_cmdstr, 'codec_high_to_low'),
    (0xb2,  4, varray, 'i2cCmdStrCodecLowToHigh', b'\x01\x00'),
    (0xb6,  4, varray, 'i2cCmdStrCodecStart',           encode_i2c_cmdstr, 'codec_start'),
    (0xba,  4, varray, 'i2cCmdStrCodecStop',            encode_i2c_cmdstr, 'codec_stop'),
    (0xbe,  4, varray, 'i2cCmdStrSetVolumeLeftPrefix', None),
    (0xc2,  4, varray, 'i2cCmdStrSetVolumeLeftSuffix', None),
    (0xc6,  4, varray, 'i2cCmdStrSetVolumeRightPrefix', None),
    (0xca,  4, varray, 'i2cCmdStrSetVolumeRightSuffix', None),
    (0xce,  4, varray, 'i2cCmdStrGetMutePrefix', None),
    (0xd2,  4, varray, 'i2cCmdStrSetMutePrefix', None),
    (0xd6,  4, varray, 'i2cCmdStrSetMuteSuffix', None),
    (0xda,  4, varray, 'i2cCmdStrSetSampleRate44', None),
    (0xde,  4, varray, 'i2cCmdStrSetSampleRate48', None),
    (0xe2,  4, varray, 'i2cCmdStrProfile0', None),
    (0xe6,  4, varray, 'i2cCmdStrProfile1', None),
    (0xea,  4, varray, 'i2cCmdStrProfile2', ),
    (0xee,  20, array, 'spareI2cCmdStr', '\x00' * 20),
    (0x102,  4, varray, 'endVarMarker', 'VEND'),
    (0x106,  4, array, 'endConfigMarker', 'STOP'),
]

class GPIO_Function(Enum):
    GPIO = 0x00
    ALTERNATE = 0x01
    SUSPEND = 0x01
    SUSPEND_N = 0x02
    LOWPWR = 0x03
    LOWPWR_N = 0x04
    RMUTE = 0x05
    RMUTE_N = 0x06
    VBUS_IS_5V = 0x07
    PBMUTE = 0x08
    PBMUTE_N = 0x09
    # **NOTE:** gap in values
    EXT_SUPPLY = 0x0f
    # **NOTE:** all buttons have bit7 set
    PLAY_PAUSE = 0x80
    FFWD = 0x81
    REW = 0x82
    MUTE = 0x83
    VOL_P = 0x84
    VOL_N = 0x85
    PLAY = 0x86
    STOP = 0x87
    # **NOTE:** gap in values
    # **NOTE:** non-HID buttons have bit6 set
    GESTURE = 0xc0
    RECMUTE = 0xcd
    PROFILE_SEL = 0xcf

class GPIO_Mode(Enum):
    INPUT = 0
    PUSH_PULL = 1
    OPEN_DRAIN = 2

cp2615_config = {
        # set to yes to lock configuration (can no longer be updated)
        'cfg_is_locked': False,

        # General configuration items
        'usb_vid': 0x10C4,
        'usb_pid': 0xEAC1,
        'usb_lang': 0x0409,

        # Custom serial number can be enabled (yes | no)
        # SN is a UTF-8 string max 31 bytes.  Note that an UTF-8 character
        # may be more than 1 byte for non-ascii characters
        'usb_custom_sn': None,

        'usb_mfr': "Silicon Labs",
        'usb_prod': "CP2615 Digital Audio Bridge",

        # user-specified 16-bit option ID (optional)
        'option_id': 0,

        # not used by end user, leave as 0
        'debug_mode': 0,

        ################
        # Audio configuration items

        # Audio interface selection
        # 0-none, 1-16in, 2-24/16in, 3-16out, 4-24/16out, 5-16in/out,
        # 6-24in, 7-24out
        'audio_if': 5,

        # set to yes to enable asynchronous mode
        'use_async': False,

        # Supported sample rates: 0-48, 1=44, 2-44/48
        'sample_rate': [44100, 48000],

        # eable or disable feature unit (default should be yes)
        'enable_fu': True,

        # input terminal type: 0-line input, 1-microphone
        'input_terminal': 0,

        # output terminal type: 0-headphones, 1-speaker, 2-line output
        'output_terminal': 0,

        # I2S output clocks can be active when not streaming (yes|no)
        'mclk_active': True,
        'lrck_active': True,

        # volume settings are in dB
        'vol_default_master': 0,
        'vol_default_left': 0,
        'vol_default_right': 0,
        'vol_min': -127,
        'vol_max': 0,
        'vol_min_counts': 0,
        'vol_max_counts': 127,

        # dB per count, floating point number
        'vol_resolution': 1,

        ################
        # Codec configuration items

        # volume register size in bytes, 1 or 2
        'vol_reg_size': 1,

        # volume register can be signed (True) or unsigned (False)
        'vol_is_signed': False,

        # starting bit position of volume field
        'vol_bit_start': 0,

        # volume register mask values
        'vol_and_mask': 0x7f,
        'vol_or_mask': 0x00,

        # mute controls
        # playback mute uses codec register
        'mute_by_reg': True,

        # mute polarity is 0-low, 1-high
        'mute_polarity': 0,

        # mute bits in register
        'mute_mask': 0x80,

        # I2C startup delay - milliseconds
        'i2c_delay': 5,

        # Codec I2C strings

        'codec_init':
            "C" # assert codec reset
            "D\x01\x00" # delay 1ms
            "c" # deassert codec reset
            "W\x02\x20\x03\x20P" # Write reg 0x3 (Soft)
            "W\x02\x20\x06\x10P" # Write reg 0x6 (ADC uses I2S)
            "W\x02\x20\x07\x06P" # Write reg 0x7 (AMUTE = BMUTE, CPE)
            "\x00",
        'codec_high_to_low':
            "\x00",
        'codec_low_to_high':
            "\x00",
        'codec_start':
            "\x00",
        'codec_stop':
            "\x00",
        'codec_left_vol_prefix':
            "W\x03\x20\x04", # Write reg 0x4
        'codec_left_vol_suffix':
            "P\x00",
        'codec_right_vol_prefix':
            "W\x03\x20\x05", # Write reg 0x5
        'codec_right_vol_suffix':
            "P\x00",
        'codec_get_mute_prefix':
            "W\x01\x20\x04"
            "R\x01\x21P\x00",
        'codec_set_mute_prefix':
            "W\x02\x20\x04",
        'codec_set_mute_suffix':
            "P\x00",
        'codec_set_rate_44': "\x00",
        'codec_set_rate_48': "\x00",
        'codec_profile_0': "\x00",
        'codec_profile_1': "\x00",
        'codec_profile_2': "\x00",

        ################
        # I/O Options
        'serial_protocol_enabled': "yes",
        'io_protocol_enabled': "no",
        # ignore proto_name - it is not required in the input dictionary
        'proto_name': "N/A",

        ################
        # Power Options
        'self_powered': "no",
        'max_current_ma': "100",

        ################
        # GPIO options
        #
        # settings for GPIO 00-07
        #
        # each gpio can be set for direction, drive mode, function, and
        # reset value
        #
        # gpio_nn_mode = [0-2]
        #   0 - input
        #   1 - output push pull
        #   2 - output open-drain
        #
        # gpio_nn_function (values in hex)
        #   00 - GPIO
        #   01 - SUSPEND (O)
        #   02 - SUSPEND_N (O)
        #   03 - LOWPWR (O)
        #   04 - LOWPWR_N (O)
        #   05 - RMUTE (O)
        #   06 - RMUTE_N (O)
        #   07 - VBUS_IS_5V (O)
        #   08 - PBMUTE (O)
        #   09 - PBMUTE_N (O)
        # **NOTE:** gap in values
        #   0F - EXT_SUPPLY (I)
        # **NOTE:** all buttons have bit7 set
        #   80 - PLAY_PAUSE (I)
        #   81 - FFWD (I)
        #   82 - REW (I)
        #   83 - MUTE (I)
        #   84 - VOL_P (I)
        #   85 - VOL_N (I)
        #   86 - PLAY (I)
        #   87 - STOP (I)
        # **NOTE:** gap in values
        # **NOTE:** non-HID buttons have bit6 set
        #   C0 - GESTURE (I)
        #   CD - RECMUTE (I)
        #   CF - PROFILE_SEL (I)
        #
        # gpio_nn_reset = 0 | 1, reset value of the pin (should default to 1)
        'gpios': [
            # gpio mode,           gpio function,          gpio reset value
            (GPIO_Mode.INPUT,      GPIO_Function.GPIO,     1),
            (GPIO_Mode.INPUT,      GPIO_Function.GPIO,     1),
            (GPIO_Mode.PUSH_PULL,  GPIO_Function.LOWPWR,   1),
            (GPIO_Mode.INPUT,      GPIO_Function.VOL_P,    1),
            (GPIO_Mode.INPUT,      GPIO_Function.VOL_N,    1),
            (GPIO_Mode.INPUT,      GPIO_Function.FFWD,     1),
            (GPIO_Mode.INPUT,      GPIO_Function.REW,      1),
            (GPIO_Mode.INPUT,      GPIO_Function.PLAY_PAUSE, 1),
        # GPIO 08-15 are similar to 00-07 except that the function only has
        # two choices:
        #
        # gpio_nn_function (values in hex)
        #   00 - GPIO
        #   01 - fixed alternate function
            (GPIO_Mode.INPUT,      GPIO_Function.GPIO, 1),
            (GPIO_Mode.INPUT,      GPIO_Function.GPIO, 1),
            (GPIO_Mode.INPUT,      GPIO_Function.GPIO, 1),
            (GPIO_Mode.INPUT,      GPIO_Function.GPIO, 1),
            (GPIO_Mode.INPUT,      GPIO_Function.GPIO, 1),
            (GPIO_Mode.INPUT,      GPIO_Function.GPIO, 1),
            (GPIO_Mode.INPUT,      GPIO_Function.GPIO, 1),
            (GPIO_Mode.INPUT,      GPIO_Function.GPIO, 1),
        ],

        ################
        # Analog Pin Options (and Misc)

        'clkout_divider': 1,
        'serial_rate': 115200,

        'button_00': 0,
        'button_01': 0,
        'button_02': 0,
        'button_03': 0,
        'button_04': 0,
        'button_05': 0,
        'button_06': 0,
        'button_07': 0,
        'button_08': 0,
        'button_09': 0,
        'button_10': 0,
        'button_11': 0,
        'button_12': 0,
        'button_13': 0,
        'button_14': 0,

        ################
        # Gesture Button
        #   00 - Long Press
        #   01 - Single Click
        #   02 - Double Click
        #   03 - Triple Click
        'gesture_00': 0,
        'gesture_01': 0,
        'gesture_02': 0,
        'gesture_03': 0,
}

from struct import *

def pack(fields, values):
    bufsize = 0
    for off, size, fmt, name, val in fields:
        assert bufsize == off
        bufsize += size
    buf = bytearray(bufsize)
    for off, size, pack, name, val in fields:
        print ("pack('{}', buf, {}, {})".format(fmt, hex(off), val))
        pack(fmt, buf, off, val)

def genconf(fields, values):
    bufsize = 0
    for name, encode, size, defval in field_map:
        bufsize += size
    buf = bytearray(bufsize)
    off = 0
    for name, encode, size, defval in field_map:
        if name in field_val:
            val = field_val[name]
        else:
            val = defval
        bufsize += encode(buf, off, val);

#cfg = ConfigStruct(cp2615_fields)
cfg = pack(cp2615_fields, {})


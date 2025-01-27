/*
 * Copyright (c) 2014 The DragonFly Project.  All rights reserved.
 *
 * This code is derived from software contributed to The DragonFly Project
 * by Matthew Dillon <dillon@backplane.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of The DragonFly Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific, prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *------------------------------------------------------------------------------
 * Copyright 2011 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ''AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ATMEL OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *-----------------------------------------------------------------------------
 */
/*
 * This file contains a mix of code taken from the atmel-maxtouch/obp-utils
 * github codebase and my own for testing the atmel touchscreen chipset in
 * the Acer C720P chromebook.
 *
 * I have rewritten the linux mess for the most part.
 */

#include <stdint.h>

#ifndef _OBP_UTILS_H_
#define _OBP_UTILS_H_

#define MXT_MAX_FINGERS 10

/*
 * CRC structure hangs off the end of the object configuration array and
 * is calculated against { mxt_id_info, mxt_object[num_objects] }.
 *
 * { mxt_id_info, mxt_object[num_objects], mxt_raw_crc }
 */
typedef struct __attribute__((__packed__)){
    uint16_t CRC;        /* low 16 bits */
    uint8_t CRC_hi;        /* high 8 bits */
} mxt_raw_crc;

/*
 * register 0 - Base information structure
 */
typedef struct __attribute__((__packed__)){
    uint8_t family;        /*!< Device family */
    uint8_t variant;    /*!< Device variant */
    
    uint8_t version;    /*!< Firmware version (Major/minor nibbles) */
    uint8_t build;        /*!< Firmware build number */
    
    uint8_t matrix_x_size;    /*!< Matrix X Size */
    uint8_t matrix_y_size;    /*!< Matrix Y Size */
    
    /*! Number of elements in the object table. The actual number of
     *  objects can be different if any object has more than one
     *  instance.
     */
    uint8_t num_objects;
} mxt_id_info;

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37    37
#define MXT_GEN_MESSAGE_T5        5
#define MXT_GEN_COMMAND_T6        6
#define MXT_GEN_POWER_T7        7
#define MXT_GEN_ACQUIRE_T8        8
#define MXT_GEN_DATASOURCE_T53        53
#define MXT_TOUCH_MULTI_T9        9
#define MXT_TOUCH_KEYARRAY_T15        15
#define MXT_TOUCH_PROXIMITY_T23        23
#define MXT_TOUCH_PROXKEY_T52        52
#define MXT_PROCI_GRIPFACE_T20        20
#define MXT_PROCG_NOISE_T22        22
#define MXT_PROCI_ONETOUCH_T24        24
#define MXT_PROCI_TWOTOUCH_T27        27
#define MXT_PROCI_GRIP_T40        40
#define MXT_PROCI_PALM_T41        41
#define MXT_PROCI_TOUCHSUPPRESSION_T42    42
#define MXT_PROCI_STYLUS_T47        47
#define MXT_PROCG_NOISESUPPRESSION_T48    48
#define MXT_SPT_COMMSCONFIG_T18        18
#define MXT_SPT_GPIOPWM_T19        19
#define MXT_SPT_SELFTEST_T25        25
#define MXT_SPT_CTECONFIG_T28        28
#define MXT_SPT_USERDATA_T38        38
#define MXT_SPT_DIGITIZER_T43        43
#define MXT_SPT_MESSAGECOUNT_T44    44
#define MXT_SPT_CTECONFIG_T46        46
#define MXT_TOUCH_MULTITOUCHSCREEN_T100 100

#define MXT_OBJECT_START    0x07

/*
 * register 0 - Object configuration element(s) (occurs after mxt_id_info)
 */
typedef struct __attribute__((__packed__)){
    uint8_t type;            /* object type */
    uint16_t start_address;        /* start address of config structure */
    //uint8_t start_pos_lsb;        /* start address of config structure */
    //uint8_t start_pos_msb;        /* start address of config structure */
    uint8_t size_minus_one;        /* size of config - 1 */
    uint8_t instances_minus_one;    /* #of instances of object - 1 */
    uint8_t num_report_ids;        /* max# of touches, sliders, buts, etc*/
} mxt_object;

/*
 * message structures (MXT_GEN_MESSAGEPROCESSOR)
 */
typedef struct __attribute__((__packed__)){
    uint8_t    reportid;
    uint8_t    data[7];
} mxt_message_any;

/* MXT_GEN_POWER_T7 field */
typedef struct __attribute__((__packed__)){
    uint8_t idle;
    uint8_t active;
} t7_config;

#define MXT_POWER_CFG_RUN        0
#define MXT_POWER_CFG_DEEPSLEEP        1

typedef struct __attribute__((__packed__)){
    uint8_t reportid;
    uint8_t flags;        /* msg[0] */
    uint8_t pos[3];        /* xxxxxxxx(hi) yyyyyyyy(hi) xxxxyyyy(lo) */
    uint8_t area;
    uint8_t amplitude;
} mxt_message_touch_t9;

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET    (1 << 7)
#define MXT_T6_STATUS_OFL    (1 << 6)
#define MXT_T6_STATUS_SIGERR    (1 << 5)
#define MXT_T6_STATUS_CAL    (1 << 4)
#define MXT_T6_STATUS_CFGERR    (1 << 3)
#define MXT_T6_STATUS_COMSERR    (1 << 2)

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_CTRL        0
#define MXT_T9_ORIENT        9
#define MXT_T9_RANGE        18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP        (1 << 0)
#define MXT_T9_SUPPRESS        (1 << 1)
#define MXT_T9_AMP        (1 << 2)
#define MXT_T9_VECTOR        (1 << 3)
#define MXT_T9_MOVE        (1 << 4)
#define MXT_T9_RELEASE        (1 << 5)
#define MXT_T9_PRESS        (1 << 6)
#define MXT_T9_DETECT        (1 << 7)

typedef struct __attribute__((__packed__)){
    uint16_t x;        /* x */
    uint16_t y;        /* y */
} t9_range;
#define MXT_T9_ORIENT_SWITCH    (1 << 0)

typedef union mxt_message mxt_message_t;

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL        0
#define MXT_T100_CFG1        1
#define MXT_T100_SCRAUX      2
#define MXT_T100_TCHAUX      3
#define MXT_T100_XSIZE       9
#define MXT_T100_YSIZE       20
#define MXT_T100_XRANGE      13
#define MXT_T100_YRANGE      24

#define MXT_T100_CFG_SWITCHXY    BIT(5)

#define MXT_T100_TCHAUX_VECT    BIT(0)
#define MXT_T100_TCHAUX_AMPL    BIT(1)
#define MXT_T100_TCHAUX_AREA    BIT(2)

#define MXT_T100_DETECT        BIT(7)
#define MXT_T100_TYPE_MASK    0x70

enum t100_type {
    MXT_T100_TYPE_FINGER = 1,
    MXT_T100_TYPE_PASSIVE_STYLUS = 2,
    MXT_T100_TYPE_HOVERING_FINGER = 4,
    MXT_T100_TYPE_GLOVE = 5,
    MXT_T100_TYPE_LARGE_TOUCH = 6,
};

#define MXT_DISTANCE_ACTIVE_TOUCH    0
#define MXT_DISTANCE_HOVERING        1

#define MXT_TOUCH_MAJOR_DEFAULT        1
#define MXT_PRESSURE_DEFAULT        1

typedef struct __attribute__((__packed__)){
    uint8_t reportid;
    uint8_t flags;        /* msg[0] */
    uint16_t x;
    uint16_t y;
    uint8_t area;
    uint8_t amplitude;
} mxt_message_touch_t100;

union __attribute__((__packed__)) mxt_message {
    mxt_message_any any;
    mxt_message_touch_t9 touch_t9;
    mxt_message_touch_t100 touch_t100;
};

/*
 * Object types
 */
#define MXT_RESERVED_T0                0
#define MXT_RESERVED_T1                1
#define MXT_DEBUG_DELTAS            2
#define MXT_DEBUG_REFERENCES            3
#define MXT_SIGNALS                4
#define MXT_GEN_MESSAGEPROCESSOR        5
#define MXT_GEN_COMMANDPROCESSOR        6
#define MXT_GEN_POWERCONFIG            7
#define MXT_ACQUISITIONCONFIG            8
#define MXT_TOUCH_MULTITOUCHSCREEN9        9
#define MXT_TOUCH_SINGLETOUCHSCREEN        10
#define MXT_TOUCH_XSLIDER            11
#define MXT_TOUCH_YSLIDER            12
#define MXT_TOUCH_XWHEEL            13
#define MXT_TOUCH_YWHEEL            14
#define MXT_TOUCH_KEYARRAY            15
#define MXT_PROCG_SIGNALFILTER            16
#define MXT_PROCI_LINEARIZATIONTABLE        17
#define MXT_SPT_COMMSCONFIG            18
#define MXT_SPT_GPIOPWM                19
#define MXT_PROCI_GRIPFACESUPPRESSION        20
#define MXT_RESERVED21                21
#define MXT_PROCG_NOISESUPPRESSION22        22
#define MXT_TOUCH_PROXIMITY            23
#define MXT_PROCI_ONETOUCHGESTUREPROCESSOR    24
#define MXT_SPT_SELFTEST            25
#define MXT_DEBUG_CTERANGE            26
#define MXT_PROCI_TWOTOUCHGESTUREPROCESSOR    27
#define MXT_SPT_CTECONFIG28            28
#define MXT_SPT_GPI                29
#define MXT_SPT_GATE                30
#define MXT_TOUCH_KEYSET            31
#define MXT_TOUCH_XSLIDERSET            32
#define MXT_RESERVED33                33
#define MXT_GEN_MESSAGEBLOCK            34
#define MXT_SPT_PROTOTYPE            35
#define MXT_RESERVED36                36
#define MXT_DEBUG_DIAGNOSTIC            37
#define MXT_SPT_USERDATA            38
#define MXT_SPARE39                39
#define MXT_PROCI_GRIPSUPPRESSION        40
#define MXT_PROCI_PALMSUPPRESSION        41
#define MXT_PROCI_TOUCHSUPPRESSION        42
#define MXT_SPT_DIGITIZER            43
#define MXT_SPT_MESSAGECOUNT            44
#define MXT_PROCI_VIRTUALKEY            45
#define MXT_SPT_CTECONFIG46            46
#define MXT_PROCI_STYLUS            47
#define MXT_PROCG_NOISESUPPRESSION48        48
#define MXT_GEN_DUALPULSE            49
#define MXT_SPARE50                50
#define MXT_SPT_SONY_CUSTOM            51
#define MXT_TOUCH_PROXKEY            52
#define MXT_GEN_DATASOURCE            53
#define MXT_PROCG_NOISESUPPRESSION54        54
#define MXT_PROCI_ADAPTIVETHRESHOLD        55
#define MXT_PROCI_SHIELDLESS            56
#define MXT_PROCI_EXTRATOUCHSCREENDATA        57
#define MXT_SPT_EXTRANOISESUPCTRLS        58
#define MXT_SPT_FASTDRIFT            59
#define MXT_SPT_TIMER                61
#define MXT_PROCG_NOISESUPPRESSION62        62
#define MXT_PROCI_ACTIVESTYLUS63        63
#define MXT_SPT_REFERENCERELOAD            64
#define MXT_PROCI_LENSBENDING            65
#define MXT_SPT_GOLDENREFERENCES        66
#define MXT_PROCI_CUSTOMGESTUREPROCESSOR    67
#define MXT_SERIAL_DATA_COMMAND            68
#define MXT_PROCI_PALMGESTUREPROCESSOR        69
#define MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER    70
#define MXT_SPT_DYNAMICCONFIGURATIONCONTAINER    71
#define MXT_PROCG_NOISESUPPRESSION72        72
#define MXT_PROCI_ZONEINDICATION        73
#define MXT_PROCG_SIMPLEGESTUREPROCESSOR    74
#define MXT_MOTION_SENSING_OBJECT        75
#define MXT_PROCI_MOTION_GESTURES        76
#define MXT_SPT_CTESCANCONFIG            77
#define MXT_PROCI_GLOVEDETECTION        78
#define MXT_SPT_TOUCHEVENTTRIGGER        79
#define MXT_PROCI_RETRANSMISSIONCOMPENSATION    80
#define MXT_PROCI_UNLOCKGESTURE            81
#define MXT_SPT_NOISESUPEXTENSION        82
#define MXT_ENVIRO_LIGHTSENSING            83
#define MXT_PROCI_GESTUREPROCESSOR        84
#define MXT_PEN_ACTIVESTYLUSPOWER        85
#define MXT_PROCG_NOISESUPACTIVESTYLUS        86
#define MXT_PEN_ACTIVESTYLUSDATA        87
#define MXT_PEN_ACTIVESTYLUSRECEIVE        88
#define MXT_PEN_ACTIVESTYLUSTRANSMIT        89
#define MXT_PEN_ACTIVESTYLUSWINDOW        90
#define MXT_DEBUG_CUSTOMDATACONFIG        91
#define MXT_PROCI_SYMBOLGESTUREPROCESSOR    92
#define MXT_PROCI_TOUCHSEQUENCELOGGER        93
#define MXT_TOUCH_MULTITOUCHSCREEN        100
#define MXT_SPT_TOUCHSCREENHOVER        101
#define MXT_SPT_SELFCAPHOVERCTECONFIG        102
#define MXT_PROCI_SCHNOISESUPPRESSION        103
#define MXT_SPT_AUXTOUCHCONFIG            104
#define MXT_SPT_DRIVENPLATEHOVERCONFIG        105
#define MXT_SPT_ACTIVESTYLUSMMBCONFIG        106
#define MXT_PROCI_ACTIVESTYLUS107        107
#define MXT_PROCG_NOISESUPSELFCAP        108
#define MXT_SPT_SELFCAPGLOBALCONFIG        109
#define MXT_SPT_SELFCAPTUNINGPARAMS        110
#define MXT_SPT_SELFCAPCONFIG            111
#define MXT_SPT_PROXMEASURECONFIG        113
#define MXT_GEN_INFOBLOCK16BIT            254
#define MXT_RESERVED255                255

#define MXTSTR(label)    #label

#define MXT_INIT_STRINGS    \
{ MXTSTR(MXT_RESERVED_T0),            \
MXT_RESERVED_T0        },        \
{ MXTSTR(MXT_RESERVED_T1),            \
MXT_RESERVED_T1        },        \
{ MXTSTR(MXT_DEBUG_DELTAS),            \
MXT_DEBUG_DELTAS        },        \
{ MXTSTR(MXT_DEBUG_REFERENCES),            \
MXT_DEBUG_REFERENCES        },        \
{ MXTSTR(MXT_SIGNALS),                \
MXT_SIGNALS            },        \
{ MXTSTR(MXT_GEN_MESSAGEPROCESSOR),        \
MXT_GEN_MESSAGEPROCESSOR    },        \
{ MXTSTR(MXT_GEN_COMMANDPROCESSOR),        \
MXT_GEN_COMMANDPROCESSOR    },        \
{ MXTSTR(MXT_GEN_POWERCONFIG),            \
MXT_GEN_POWERCONFIG        },        \
{ MXTSTR(MXT_ACQUISITIONCONFIG),        \
MXT_ACQUISITIONCONFIG    },        \
{ MXTSTR(MXT_TOUCH_MULTITOUCHSCREEN),        \
MXT_TOUCH_MULTITOUCHSCREEN    },        \
{ MXTSTR(MXT_TOUCH_SINGLETOUCHSCREEN),        \
MXT_TOUCH_SINGLETOUCHSCREEN    },        \
{ MXTSTR(MXT_TOUCH_XSLIDER),            \
MXT_TOUCH_XSLIDER        },        \
{ MXTSTR(MXT_TOUCH_YSLIDER),            \
MXT_TOUCH_YSLIDER        },        \
{ MXTSTR(MXT_TOUCH_XWHEEL),            \
MXT_TOUCH_XWHEEL        },        \
{ MXTSTR(MXT_TOUCH_YWHEEL),            \
MXT_TOUCH_YWHEEL        },        \
{ MXTSTR(MXT_TOUCH_KEYARRAY),            \
MXT_TOUCH_KEYARRAY        },        \
{ MXTSTR(MXT_PROCG_SIGNALFILTER),        \
MXT_PROCG_SIGNALFILTER    },        \
{ MXTSTR(MXT_PROCI_LINEARIZATIONTABLE),        \
MXT_PROCI_LINEARIZATIONTABLE    },        \
{ MXTSTR(MXT_SPT_COMMSCONFIG),            \
MXT_SPT_COMMSCONFIG        },        \
{ MXTSTR(MXT_SPT_GPIOPWM),            \
MXT_SPT_GPIOPWM        },        \
{ MXTSTR(MXT_PROCI_GRIPFACESUPPRESSION),    \
MXT_PROCI_GRIPFACESUPPRESSION    },    \
{ MXTSTR(MXT_RESERVED21),            \
MXT_RESERVED21            },    \
{ MXTSTR(MXT_PROCG_NOISESUPPRESSION22),        \
MXT_PROCG_NOISESUPPRESSION22        },    \
{ MXTSTR(MXT_PROCG_NOISESUPPRESSION48),        \
MXT_PROCG_NOISESUPPRESSION48        },    \
{ MXTSTR(MXT_PROCG_NOISESUPPRESSION54),        \
MXT_PROCG_NOISESUPPRESSION54        },    \
{ MXTSTR(MXT_PROCG_NOISESUPPRESSION62),        \
MXT_PROCG_NOISESUPPRESSION62        },    \
{ MXTSTR(MXT_PROCG_NOISESUPPRESSION72),        \
MXT_PROCG_NOISESUPPRESSION72        },    \
{ MXTSTR(MXT_TOUCH_PROXIMITY),            \
MXT_TOUCH_PROXIMITY            },    \
{ MXTSTR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR),    \
MXT_PROCI_ONETOUCHGESTUREPROCESSOR    },    \
{ MXTSTR(MXT_SPT_SELFTEST),            \
MXT_SPT_SELFTEST            },    \
{ MXTSTR(MXT_DEBUG_CTERANGE),            \
MXT_DEBUG_CTERANGE            },    \
{ MXTSTR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR),    \
MXT_PROCI_TWOTOUCHGESTUREPROCESSOR    },    \
{ MXTSTR(MXT_SPT_CTECONFIG28),            \
MXT_SPT_CTECONFIG28            },    \
{ MXTSTR(MXT_SPT_CTECONFIG46),            \
MXT_SPT_CTECONFIG46            },    \
{ MXTSTR(MXT_SPT_GPI),                \
MXT_SPT_GPI            },        \
{ MXTSTR(MXT_SPT_GATE),                \
MXT_SPT_GATE            },        \
{ MXTSTR(MXT_TOUCH_KEYSET),            \
MXT_TOUCH_KEYSET        },        \
{ MXTSTR(MXT_TOUCH_XSLIDERSET),            \
MXT_TOUCH_XSLIDERSET        },        \
{ MXTSTR(MXT_RESERVED33),            \
MXT_RESERVED33        },        \
{ MXTSTR(MXT_GEN_MESSAGEBLOCK),            \
MXT_GEN_MESSAGEBLOCK        },        \
{ MXTSTR(MXT_SPT_PROTOTYPE),            \
MXT_SPT_PROTOTYPE        },        \
{ MXTSTR(MXT_RESERVED36),            \
MXT_RESERVED36        },        \
{ MXTSTR(MXT_DEBUG_DIAGNOSTIC),            \
MXT_DEBUG_DIAGNOSTIC        },        \
{ MXTSTR(MXT_SPT_USERDATA),            \
MXT_SPT_USERDATA        },        \
{ MXTSTR(MXT_SPARE39),                \
MXT_SPARE39            },        \
{ MXTSTR(MXT_PROCI_GRIPSUPPRESSION),        \
MXT_PROCI_GRIPSUPPRESSION    },        \
{ MXTSTR(MXT_PROCI_PALMSUPPRESSION),        \
MXT_PROCI_PALMSUPPRESSION    },        \
{ MXTSTR(MXT_PROCI_TOUCHSUPPRESSION),        \
MXT_PROCI_TOUCHSUPPRESSION    },        \
{ MXTSTR(MXT_SPT_DIGITIZER),            \
MXT_SPT_DIGITIZER        },        \
{ MXTSTR(MXT_SPT_MESSAGECOUNT),            \
MXT_SPT_MESSAGECOUNT        },        \
{ MXTSTR(MXT_PROCI_VIRTUALKEY),            \
MXT_PROCI_VIRTUALKEY        },        \
{ MXTSTR(MXT_PROCI_STYLUS),            \
MXT_PROCI_STYLUS        },        \
{ MXTSTR(MXT_GEN_DUALPULSE),            \
MXT_GEN_DUALPULSE        },        \
{ MXTSTR(MXT_SPARE50),                \
MXT_SPARE50            },        \
{ MXTSTR(MXT_SPT_SONY_CUSTOM),            \
MXT_SPT_SONY_CUSTOM        },        \
{ MXTSTR(MXT_TOUCH_PROXKEY),            \
MXT_TOUCH_PROXKEY        },        \
{ MXTSTR(MXT_GEN_DATASOURCE),            \
MXT_GEN_DATASOURCE        },        \
{ MXTSTR(MXT_PROCI_ADAPTIVETHRESHOLD),        \
MXT_PROCI_ADAPTIVETHRESHOLD    },        \
{ MXTSTR(MXT_PROCI_SHIELDLESS),            \
MXT_PROCI_SHIELDLESS        },        \
{ MXTSTR(MXT_PROCI_EXTRATOUCHSCREENDATA),    \
MXT_PROCI_EXTRATOUCHSCREENDATA    },    \
{ MXTSTR(MXT_SPT_EXTRANOISESUPCTRLS),        \
MXT_SPT_EXTRANOISESUPCTRLS        },    \
{ MXTSTR(MXT_SPT_FASTDRIFT),            \
MXT_SPT_FASTDRIFT            },    \
{ MXTSTR(MXT_SPT_TIMER),            \
MXT_SPT_TIMER            },    \
{ MXTSTR(MXT_PROCI_ACTIVESTYLUS63),        \
MXT_PROCI_ACTIVESTYLUS63        },    \
{ MXTSTR(MXT_PROCI_ACTIVESTYLUS107),        \
MXT_PROCI_ACTIVESTYLUS107        },    \
{ MXTSTR(MXT_SPT_REFERENCERELOAD),        \
MXT_SPT_REFERENCERELOAD        },    \
{ MXTSTR(MXT_PROCI_LENSBENDING),        \
MXT_PROCI_LENSBENDING        },    \
{ MXTSTR(MXT_SPT_GOLDENREFERENCES),        \
MXT_SPT_GOLDENREFERENCES        },    \
{ MXTSTR(MXT_PROCI_CUSTOMGESTUREPROCESSOR),    \
MXT_PROCI_CUSTOMGESTUREPROCESSOR    },    \
{ MXTSTR(MXT_SERIAL_DATA_COMMAND),        \
MXT_SERIAL_DATA_COMMAND        },    \
{ MXTSTR(MXT_PROCI_PALMGESTUREPROCESSOR),    \
MXT_PROCI_PALMGESTUREPROCESSOR    },    \
{ MXTSTR(MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER),    \
MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER    },    \
{ MXTSTR(MXT_SPT_DYNAMICCONFIGURATIONCONTAINER),    \
MXT_SPT_DYNAMICCONFIGURATIONCONTAINER    },    \
{ MXTSTR(MXT_PROCI_ZONEINDICATION),        \
MXT_PROCI_ZONEINDICATION        },    \
{ MXTSTR(MXT_PROCG_SIMPLEGESTUREPROCESSOR),    \
MXT_PROCG_SIMPLEGESTUREPROCESSOR    },    \
{ MXTSTR(MXT_MOTION_SENSING_OBJECT),        \
MXT_MOTION_SENSING_OBJECT        },    \
{ MXTSTR(MXT_PROCI_MOTION_GESTURES),        \
MXT_PROCI_MOTION_GESTURES        },    \
{ MXTSTR(MXT_SPT_CTESCANCONFIG),        \
MXT_SPT_CTESCANCONFIG        },    \
{ MXTSTR(MXT_PROCI_GLOVEDETECTION),        \
MXT_PROCI_GLOVEDETECTION        },    \
{ MXTSTR(MXT_SPT_TOUCHEVENTTRIGGER),        \
MXT_SPT_TOUCHEVENTTRIGGER        },    \
{ MXTSTR(MXT_PROCI_RETRANSMISSIONCOMPENSATION),    \
MXT_PROCI_RETRANSMISSIONCOMPENSATION    },    \
{ MXTSTR(MXT_PROCI_UNLOCKGESTURE),        \
MXT_PROCI_UNLOCKGESTURE        },    \
{ MXTSTR(MXT_SPT_NOISESUPEXTENSION),        \
MXT_SPT_NOISESUPEXTENSION        },    \
{ MXTSTR(MXT_ENVIRO_LIGHTSENSING),        \
MXT_ENVIRO_LIGHTSENSING        },    \
{ MXTSTR(MXT_PROCI_GESTUREPROCESSOR),        \
MXT_PROCI_GESTUREPROCESSOR        },    \
{ MXTSTR(MXT_PEN_ACTIVESTYLUSPOWER),        \
MXT_PEN_ACTIVESTYLUSPOWER        },    \
{ MXTSTR(MXT_PROCG_NOISESUPACTIVESTYLUS),    \
MXT_PROCG_NOISESUPACTIVESTYLUS    },    \
{ MXTSTR(MXT_PEN_ACTIVESTYLUSDATA),        \
MXT_PEN_ACTIVESTYLUSDATA        },    \
{ MXTSTR(MXT_PEN_ACTIVESTYLUSRECEIVE),        \
MXT_PEN_ACTIVESTYLUSRECEIVE        },    \
{ MXTSTR(MXT_PEN_ACTIVESTYLUSTRANSMIT),        \
MXT_PEN_ACTIVESTYLUSTRANSMIT        },    \
{ MXTSTR(MXT_PEN_ACTIVESTYLUSWINDOW),        \
MXT_PEN_ACTIVESTYLUSWINDOW        },    \
{ MXTSTR(MXT_DEBUG_CUSTOMDATACONFIG),        \
MXT_DEBUG_CUSTOMDATACONFIG        },    \
{ MXTSTR(MXT_PROCI_SYMBOLGESTUREPROCESSOR),    \
MXT_PROCI_SYMBOLGESTUREPROCESSOR    },    \
{ MXTSTR(MXT_PROCI_TOUCHSEQUENCELOGGER),    \
MXT_PROCI_TOUCHSEQUENCELOGGER    },    \
{ MXTSTR(MXT_TOUCH_MULTITOUCHSCREEN),        \
MXT_TOUCH_MULTITOUCHSCREEN        },    \
{ MXTSTR(MXT_SPT_TOUCHSCREENHOVER),        \
MXT_SPT_TOUCHSCREENHOVER        },    \
{ MXTSTR(MXT_SPT_SELFCAPHOVERCTECONFIG),    \
MXT_SPT_SELFCAPHOVERCTECONFIG    },    \
{ MXTSTR(MXT_PROCI_SCHNOISESUPPRESSION),    \
MXT_PROCI_SCHNOISESUPPRESSION    },    \
{ MXTSTR(MXT_SPT_AUXTOUCHCONFIG),        \
MXT_SPT_AUXTOUCHCONFIG        },    \
{ MXTSTR(MXT_SPT_DRIVENPLATEHOVERCONFIG),    \
MXT_SPT_DRIVENPLATEHOVERCONFIG    },    \
{ MXTSTR(MXT_SPT_ACTIVESTYLUSMMBCONFIG),    \
MXT_SPT_ACTIVESTYLUSMMBCONFIG    },    \
{ MXTSTR(MXT_PROCG_NOISESUPSELFCAP),    \
MXT_PROCG_NOISESUPSELFCAP    },    \
{ MXTSTR(MXT_SPT_SELFCAPGLOBALCONFIG),    \
MXT_SPT_SELFCAPGLOBALCONFIG    },    \
{ MXTSTR(MXT_SPT_SELFCAPTUNINGPARAMS),    \
MXT_SPT_SELFCAPTUNINGPARAMS    },    \
{ MXTSTR(MXT_SPT_SELFCAPCONFIG),    \
MXT_SPT_SELFCAPCONFIG    },    \
{ MXTSTR(MXT_SPT_PROXMEASURECONFIG),    \
MXT_SPT_PROXMEASURECONFIG    },    \
{ MXTSTR(MXT_GEN_INFOBLOCK16BIT),    \
MXT_GEN_INFOBLOCK16BIT    },    \
{ MXTSTR(MXT_RESERVED255),        \
MXT_RESERVED255        }

/*
 * MXT_GEN_COMMANDPROCESSOR object offsets.
 */
#define MXT_CMDPROC_RESET_OFF        0x00
#define MXT_CMDPROC_BACKUPNV_OFF    0x01
#define MXT_CMDPROC_CALIBRATE_OFF    0x02
#define MXT_CMDPROC_REPORTALL_OFF    0x03
#define MXT_CMDPROC_RESERVED04_OFF    0x04
#define MXT_CMDPROC_DIAGNOSTIC_OFF    0x05

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE        0xa5
#define MXT_RESET_VALUE        0x01
#define MXT_BACKUP_VALUE    0x55

/*
 * device driver helper structures
 */
struct mxt_strinfo {
    const char *id;
    int    type;
};

struct mxt_rollup {
    mxt_id_info    info;
    mxt_object    *objs;
    int            nobjs;
    uint8_t            *buf;
};

struct atmel_softc {
    int flags[20];
    int x[20];
    int y[20];
    int p[20];
    int area[20];
    int ampl[20];
};

/*
 * Kernel AND user mode support
 */
uint32_t obp_convert_crc(mxt_raw_crc *crc);
uint32_t obp_crc24(uint8_t *buf, size_t bytes);

#endif

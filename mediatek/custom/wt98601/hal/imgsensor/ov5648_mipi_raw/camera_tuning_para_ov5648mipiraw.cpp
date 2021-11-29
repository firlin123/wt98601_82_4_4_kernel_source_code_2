#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_ov5648mipiraw.h"
#include "camera_info_ov5648mipiraw.h"
#include "camera_custom_AEPlinetable.h"
#include "camera_custom_tsf_tbl.h"
const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =
{{
    //Version
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,
    //SensorId
    SensorId: SENSOR_ID,
    ISPComm:{
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    },
    ISPPca:{
        #include INCLUDE_FILENAME_ISP_PCA_PARAM
    },
    ISPRegs:{
        #include INCLUDE_FILENAME_ISP_REGS_PARAM
        },
    ISPMfbMixer:{{
        {//00: MFB mixer for ISO 100
            0x00000000, 0x00000000
        },
        {//01: MFB mixer for ISO 200
            0x00000000, 0x00000000
        },
        {//02: MFB mixer for ISO 400
            0x00000000, 0x00000000
        },
        {//03: MFB mixer for ISO 800
            0x00000000, 0x00000000
        },
        {//04: MFB mixer for ISO 1600
            0x00000000, 0x00000000
        },
        {//05: MFB mixer for ISO 2400
            0x00000000, 0x00000000
        },
        {//06: MFB mixer for ISO 3200
            0x00000000, 0x00000000
        }
    }},
    ISPCcmPoly22:{
        59400,    // i4R_AVG
        13477,    // i4R_STD
        88275,    // i4B_AVG
        23748,    // i4B_STD
        {  // i4P00[9]
            4012500, -1127500, -332500, -762500, 3250000, 75000, 35000, -2025000, 4552500
        },
        {  // i4P10[9]
            1017996, -1115024, 95397, 1652, -204270, 198221, 59279, 522706, -575888
        },
        {  // i4P01[9]
            46543, -197637, 145331, -81753, -246715, 328737, -46762, -248939, 300772
        },
        {  // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P02[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    }
}};

const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =
{
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version
    SENSOR_ID, // SensorId

    // AE NVRAM
    {
        // rDevicesInfo
        {
            1136,    // u4MinGain, 1024 base = 1x
            10240,    // u4MaxGain, 16x
            51,    // u4MiniISOGain, ISOxx  
            64,    // u4GainStepUnit, 1x/8 
            35,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            35,    // u4VideoExpUnit  
            30,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            35,    // u4CapExpUnit 
            15,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            24,    // u4LensFno, Fno = 2.8
            350    // u4FocusLength_100x
        },
        // rHistConfig
        {
            2,    // u4HistHighThres
            40,    // u4HistLowThres
            2,    // u4MostBrightRatio
            1,    // u4MostDarkRatio
            160,    // u4CentralHighBound
            20,    // u4CentralLowBound
            {240, 230, 220, 210, 200},    // u4OverExpThres[AE_CCT_STRENGTH_NUM] 
            {86, 108, 128, 148, 170},    // u4HistStretchThres[AE_CCT_STRENGTH_NUM] 
            {18, 22, 26, 30, 34}    // u4BlackLightThres[AE_CCT_STRENGTH_NUM] 
        },
        // rCCTConfig
        {
            TRUE,    // bEnableBlackLight
            TRUE,    // bEnableHistStretch
            FALSE,    // bEnableAntiOverExposure
            TRUE,    // bEnableTimeLPF
            TRUE,    // bEnableCaptureThres
            TRUE,    // bEnableVideoThres
            TRUE,    // bEnableStrobeThres
            47,    // u4AETarget
            0,    // u4StrobeAETarget
            50,    // u4InitIndex
            10,    // u4BackLightWeight
            32,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            4,    // u4BlackLightStrengthIndex
            4,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            -10,    // i4BVOffset delta BV = value/10 
            32,    // u4PreviewFlareOffset
            32,    // u4CaptureFlareOffset
            5,    // u4CaptureFlareThres
            32,    // u4VideoFlareOffset
            5,    // u4VideoFlareThres
            32,    // u4StrobeFlareOffset
            2,    // u4StrobeFlareThres
            160,    // u4PrvMaxFlareThres
            0,    // u4PrvMinFlareThres
            160,    // u4VideoMaxFlareThres
            0,    // u4VideoMinFlareThres
            18,    // u4FlatnessThres    // 10 base for flatness condition.
            75    // u4FlatnessStrength
        }
    },
    // AWB NVRAM
    {
        // AWB calibration data
        {
            // rUnitGain (unit gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rGoldenGain (golden sample gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rD65Gain (D65 WB gain: 1.0 = 512)
            {
                692,    // i4R
                512,    // i4G
                561    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                0,    // i4X
                0    // i4Y
            },
            // Horizon
            {
                -502,    // i4X
                -204    // i4Y
            },
            // A
            {
                -371,    // i4X
                -206    // i4Y
            },
            // TL84
            {
                -167,    // i4X
                -258    // i4Y
            },
            // CWF
            {
                -110,    // i4X
                -354    // i4Y
            },
            // DNP
            {
                -64,    // i4X
                -248    // i4Y
            },
            // D65
            {
                77,    // i4X
                -145    // i4Y
            },
            // DF
            {
                28,    // i4X
                -304    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                0,    // i4X
                0    // i4Y
            },
            // Horizon
            {
                -528,    // i4X
                -123    // i4Y
            },
            // A
            {
                -399,    // i4X
                -146    // i4Y
            },
            // TL84
            {
                -205,    // i4X
                -229    // i4Y
            },
            // CWF
            {
                -164,    // i4X
                -333    // i4Y
            },
            // DNP
            {
                -102,    // i4X
                -235    // i4Y
            },
            // D65
            {
                53,    // i4X
                -155    // i4Y
            },
            // DF
            {
                -20,    // i4X
                -305    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                512,    // i4R
                512,    // i4G
                512    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                767,    // i4G
                1995    // i4B
            },
            // A 
            {
                512,    // i4R
                640,    // i4G
                1399    // i4B
            },
            // TL84 
            {
                579,    // i4R
                512,    // i4G
                911    // i4B
            },
            // CWF 
            {
                712,    // i4R
                512,    // i4G
                959    // i4B
            },
            // DNP 
            {
                656,    // i4R
                512,    // i4G
                782    // i4B
            },
            // D65 
            {
                692,    // i4R
                512,    // i4G
                561    // i4B
            },
            // DF 
            {
                804,    // i4R
                512,    // i4G
                744    // i4B
            }
        },
        // Rotation matrix parameter
        {
            9,    // i4RotationAngle
            253,    // i4Cos
            40    // i4Sin
        },
        // Daylight locus parameter
        {
            -168,    // i4SlopeNumerator
            128    // i4SlopeDenominator
        },
        // AWB light area
        {
            // Strobe:FIXME
            {
            0,    // i4RightBound
            0,    // i4LeftBound
            0,    // i4UpperBound
            0    // i4LowerBound
            },
            // Tungsten
            {
            -255,    // i4RightBound
            -1100,    // i4LeftBound
            -44,    // i4UpperBound
            -184    // i4LowerBound
            },
            // Warm fluorescent
            {
            -255,    // i4RightBound
            -1100,    // i4LeftBound
            -184,    // i4UpperBound
            -304    // i4LowerBound
            },
            // Fluorescent
            {
            -152,    // i4RightBound
            -255,    // i4LeftBound
            -79,    // i4UpperBound
            -281    // i4LowerBound
            },
            // CWF
            {
            -152,    // i4RightBound
            -255,    // i4LeftBound
            -281,    // i4UpperBound
            -510    // i4LowerBound
            },
            // Daylight
            {
            88,    // i4RightBound
            -152,    // i4LeftBound
            -75,    // i4UpperBound
            -380    // i4LowerBound
            },
            // Shade
            {
            438,    // i4RightBound
            88,    // i4LeftBound
            -75,    // i4UpperBound
            -260    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            0,    // i4RightBound
            0,    // i4LeftBound
            0,    // i4UpperBound
            0    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            438,    // i4RightBound
            -1100,    // i4LeftBound
            0,    // i4UpperBound
            -510    // i4LowerBound
            },
            // Daylight
            {
            113,    // i4RightBound
            -152,    // i4LeftBound
            -75,    // i4UpperBound
            -380    // i4LowerBound
            },
            // Cloudy daylight
            {
            213,    // i4RightBound
            38,    // i4LeftBound
            -75,    // i4UpperBound
            -380    // i4LowerBound
            },
            // Shade
            {
            313,    // i4RightBound
            38,    // i4LeftBound
            -75,    // i4UpperBound
            -380    // i4LowerBound
            },
            // Twilight
            {
            -152,    // i4RightBound
            -312,    // i4LeftBound
            -75,    // i4UpperBound
            -380    // i4LowerBound
            },
            // Fluorescent
            {
            103,    // i4RightBound
            -305,    // i4LeftBound
            -105,    // i4UpperBound
            -383    // i4LowerBound
            },
            // Warm fluorescent
            {
            -299,    // i4RightBound
            -499,    // i4LeftBound
            -105,    // i4UpperBound
            -383    // i4LowerBound
            },
            // Incandescent
            {
            -299,    // i4RightBound
            -499,    // i4LeftBound
            -75,    // i4UpperBound
            -380    // i4LowerBound
            },
            // Gray World
            {
            5000,    // i4RightBound
            -5000,    // i4LeftBound
            5000,    // i4UpperBound
            -5000    // i4LowerBound
            }
        },
        // PWB default gain	
        {
            // Daylight
            {
            712,    // i4R
            512,    // i4G
            682    // i4B
            },
            // Cloudy daylight
            {
            839,    // i4R
            512,    // i4G
            545    // i4B
            },
            // Shade
            {
            887,    // i4R
            512,    // i4G
            504    // i4B
            },
            // Twilight
            {
            561,    // i4R
            512,    // i4G
            947    // i4B
            },
            // Fluorescent
            {
            667,    // i4R
            512,    // i4G
            788    // i4B
            },
            // Warm fluorescent
            {
            477,    // i4R
            512,    // i4G
            1249    // i4B
            },
            // Incandescent
            {
            465,    // i4R
            512,    // i4G
            1226    // i4B
            },
            // Gray World
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        // AWB preference color	
        {
            // Tungsten
            {
            0,    // i4SliderValue
            7548    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            5243    // i4OffsetThr
            },
            // Shade
            {
            0,    // i4SliderValue
            1482    // i4OffsetThr
            },
            // Daylight WB gain
            {
            580,    // i4R
            512,    // i4G
            714    // i4B
            },
            // Preference gain: strobe
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            493,    // i4R
            512,    // i4G
            529    // i4B
            },
            // Preference gain: warm fluorescent
            {
            493,    // i4R
            512,    // i4G
            529    // i4B
            },
            // Preference gain: fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: CWF
            {
            524,    // i4R
            512,    // i4G
            533    // i4B
            },
            // Preference gain: daylight
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: shade
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        {// CCT estimation
            {// CCT
                2300,    // i4CCT[0]
                2850,    // i4CCT[1]
                4100,    // i4CCT[2]
                5100,    // i4CCT[3]
                6500    // i4CCT[4]
            },
            {// Rotated X coordinate
                -581,    // i4RotatedXCoordinate[0]
                -452,    // i4RotatedXCoordinate[1]
                -258,    // i4RotatedXCoordinate[2]
                -155,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
            }
        }
    },
    {0}
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace

const CAMERA_TSF_TBL_STRUCT CAMERA_TSF_DEFAULT_VALUE =
{
    #include INCLUDE_FILENAME_TSF_PARA
    #include INCLUDE_FILENAME_TSF_DATA
};


typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


namespace NSFeature {
template <>
UINT32
SensorInfoSingleton_T::
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
{
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),
                                             sizeof(NVRAM_CAMERA_3A_STRUCT),
                                             sizeof(NVRAM_CAMERA_SHADING_STRUCT),
                                             sizeof(NVRAM_LENS_PARA_STRUCT),
                                             sizeof(AE_PLINETABLE_T),
                                             0,
                                             sizeof(CAMERA_TSF_TBL_STRUCT)};

    if (CameraDataType > CAMERA_DATA_TSF_TABLE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
    {
        return 1;
    }

    switch(CameraDataType)
    {
        case CAMERA_NVRAM_DATA_ISP:
            memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_3A:
            memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_SHADING:
            memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));
            break;
        case CAMERA_DATA_AE_PLINETABLE:
            memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));
            break;
        case CAMERA_DATA_TSF_TABLE:
            memcpy(pDataBuf,&CAMERA_TSF_DEFAULT_VALUE,sizeof(CAMERA_TSF_TBL_STRUCT));
            break;
        default:
            break;
    }
    return 0;
}}; // NSFeature



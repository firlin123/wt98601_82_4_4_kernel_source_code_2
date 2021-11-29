#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h" 
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_imx134raw.h"
#include "camera_info_imx134raw.h"
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
        74375,    // i4R_AVG
        17582,    // i4R_STD
        101575,    // i4B_AVG
        24119,    // i4B_STD
        {  // i4P00[9]
            4990000, -2460000, 22500, -775000, 3897500, -557500, 10000, -2430000, 4962500
        },
        {  // i4P10[9]
            2010939, -2126252, 153903, -156661, 32280, 106378, -5672, 79935, -22472
        },
        {  // i4P01[9]
            1617449, -1693994, 123067, -341720, -38321, 366335, -86537, -605850, 750333
        },
        { // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        { // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        { // i4P02[9]
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
            10928,    // u4MaxGain, 16x
            100,    // u4MiniISOGain, ISOxx  
            128,    // u4GainStepUnit, 1x/8 
            18,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            17,    // u4VideoExpUnit  
            31,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            17,    // u4CapExpUnit 
            25,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            20,    // u4LensFno, Fno = 2.8
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
            4,    // u4BackLightWeight
            32,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            0,    // u4BlackLightStrengthIndex
            0,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            -19,    // i4BVOffset delta BV = value/10 
            64,    // u4PreviewFlareOffset
            64,    // u4CaptureFlareOffset
            5,    // u4CaptureFlareThres
            64,    // u4VideoFlareOffset
            5,    // u4VideoFlareThres
            64,    // u4StrobeFlareOffset
            5,    // u4StrobeFlareThres
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
                981,    // i4R
                512,    // i4G
                682    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                34,    // i4X
                -356    // i4Y
            },
            // Horizon
            {
                -433,    // i4X
                -353    // i4Y
            },
            // A
            {
                -298,    // i4X
                -373    // i4Y
            },
            // TL84
            {
                -163,    // i4X
                -367    // i4Y
            },
            // CWF
            {
                -113,    // i4X
                -415    // i4Y
            },
            // DNP
            {
                -9,    // i4X
                -404    // i4Y
            },
            // D65
            {
                134,    // i4X
                -346    // i4Y
            },
            // DF
            {
                84,    // i4X
                -410    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                21,    // i4X
                -357    // i4Y
            },
            // Horizon
            {
                -445,    // i4X
                -338    // i4Y
            },
            // A
            {
                -311,    // i4X
                -363    // i4Y
            },
            // TL84
            {
                -176,    // i4X
                -361    // i4Y
            },
            // CWF
            {
                -128,    // i4X
                -411    // i4Y
            },
            // DNP
            {
                -23,    // i4X
                -404    // i4Y
            },
            // D65
            {
                122,    // i4X
                -351    // i4Y
            },
            // DF
            {
                70,    // i4X
                -413    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                869,    // i4R
                512,    // i4G
                792    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                570,    // i4G
                1652    // i4B
            },
            // A 
            {
                566,    // i4R
                512,    // i4G
                1271    // i4B
            },
            // TL84 
            {
                675,    // i4R
                512,    // i4G
                1049    // i4B
            },
            // CWF 
            {
                770,    // i4R
                512,    // i4G
                1046    // i4B
            },
            // DNP 
            {
                874,    // i4R
                512,    // i4G
                895    // i4B
            },
            // D65 
            {
                981,    // i4R
                512,    // i4G
                682    // i4B
            },
            // DF 
            {
                999,    // i4R
                512,    // i4G
                796    // i4B
            }
        },
        // Rotation matrix parameter
        {
            2,    // i4RotationAngle
            256,    // i4Cos
            9    // i4Sin
        },
        // Daylight locus parameter
        {
            -138,    // i4SlopeNumerator
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
            -226,    // i4RightBound
            -876,    // i4LeftBound
            -300,    // i4UpperBound
            -400    // i4LowerBound
            },
            // Warm fluorescent
            {
            -226,    // i4RightBound
            -876,    // i4LeftBound
            -400,    // i4UpperBound
            -520    // i4LowerBound
            },
            // Fluorescent
            {
            -73,    // i4RightBound
            -226,    // i4LeftBound
            -285,    // i4UpperBound
            -386    // i4LowerBound
            },
            // CWF
            {
            -73,    // i4RightBound
            -226,    // i4LeftBound
            -386,    // i4UpperBound
            -461    // i4LowerBound
            },
            // Daylight
            {
            147,    // i4RightBound
            -73,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Shade
            {
            507,    // i4RightBound
            147,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            150,    // i4RightBound
            -73,    // i4LeftBound
            -431,    // i4UpperBound
            -550    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            507,    // i4RightBound
            -876,    // i4LeftBound
            0,    // i4UpperBound
            -550    // i4LowerBound
            },
            // Daylight
            {
            172,    // i4RightBound
            -73,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Cloudy daylight
            {
            272,    // i4RightBound
            97,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Shade
            {
            372,    // i4RightBound
            97,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Twilight
            {
            -73,    // i4RightBound
            -233,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
            },
            // Fluorescent
            {
            172,    // i4RightBound
            -276,    // i4LeftBound
            -301,    // i4UpperBound
            -461    // i4LowerBound
            },
            // Warm fluorescent
            {
            -211,    // i4RightBound
            -411,    // i4LeftBound
            -301,    // i4UpperBound
            -461    // i4LowerBound
            },
            // Incandescent
            {
            -211,    // i4RightBound
            -411,    // i4LeftBound
            -271,    // i4UpperBound
            -431    // i4LowerBound
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
            893,    // i4R
            512,    // i4G
            755    // i4B
            },
            // Cloudy daylight
            {
            1065,    // i4R
            512,    // i4G
            625    // i4B
            },
            // Shade
            {
            1136,    // i4R
            512,    // i4G
            583    // i4B
            },
            // Twilight
            {
            685,    // i4R
            512,    // i4G
            1003    // i4B
            },
            // Fluorescent
            {
            815,    // i4R
            512,    // i4G
            905    // i4B
            },
            // Warm fluorescent
            {
            582,    // i4R
            512,    // i4G
            1301    // i4B
            },
            // Incandescent
            {
            558,    // i4R
            512,    // i4G
            1251    // i4B
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
            61,    // i4SliderValue
            4319    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            5744    // i4OffsetThr
            },
            // Shade
            {
            0,    // i4SliderValue
            1341    // i4OffsetThr
            },
            // Daylight WB gain
            {
            812,    // i4R
            512,    // i4G
            836    // i4B
            },
            // Preference gain: strobe
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: warm fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: CWF
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
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
            511,    // i4G
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
                -567,    // i4RotatedXCoordinate[0]
                -433,    // i4RotatedXCoordinate[1]
                -298,    // i4RotatedXCoordinate[2]
                -145,    // i4RotatedXCoordinate[3]
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



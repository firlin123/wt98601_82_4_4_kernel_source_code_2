#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_a5142mipiraw.h"
#include "camera_info_a5142mipiraw.h"
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
        69325,    // i4R_AVG
        13022,    // i4R_STD
        104850,    // i4B_AVG
        30928,    // i4B_STD
        {  // i4P00[9]
            5187500, -2172500, -452500, -925000, 3727500, -247500, -167500, -1862500, 4590000
        },
        {  // i4P10[9]
            1191990, -1255284, 58358, -216397, 159355, 61621, -86007, 553751, -467744
        },
        {  // i4P01[9]
            754960, -833427, 77645, -309302, 32816, 285013, -154118, -158820, 312938
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
            2304,    // u4MinGain, 1024 base = 1x
            10240,    // u4MaxGain, 16x
            50,    // u4MiniISOGain, ISOxx  
            128,    // u4GainStepUnit, 1x/8 
            31,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            31,    // u4VideoExpUnit  
            30,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            34,    // u4CapExpUnit 
            14,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            28,    // u4LensFno, Fno = 2.8
            350    // u4FocusLength_100x
        },
        // rHistConfig
        {
            4,    // u4HistHighThres
            40,    // u4HistLowThres
            2,    // u4MostBrightRatio
            1,    // u4MostDarkRatio
            160,    // u4CentralHighBound
            20,    // u4CentralLowBound
            {240, 230, 220, 210, 200},    // u4OverExpThres[AE_CCT_STRENGTH_NUM] 
            {62, 70, 82, 108, 141},    // u4HistStretchThres[AE_CCT_STRENGTH_NUM] 
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
            65,    // u4InitIndex
            4,    // u4BackLightWeight
            32,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            2,    // u4BlackLightStrengthIndex
            4,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            -8,    // i4BVOffset delta BV = value/10 
            64,    // u4PreviewFlareOffset
            64,    // u4CaptureFlareOffset
            3,    // u4CaptureFlareThres
            64,    // u4VideoFlareOffset
            3,    // u4VideoFlareThres
            64,    // u4StrobeFlareOffset
            3,    // u4StrobeFlareThres
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
                873,    // i4R
                512,    // i4G
                615    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                15,    // i4X
                -347    // i4Y
            },
            // Horizon
            {
                -464,    // i4X
                -378    // i4Y
            },
            // A
            {
                -333,    // i4X
                -372    // i4Y
            },
            // TL84
            {
                -148,    // i4X
                -373    // i4Y
            },
            // CWF
            {
                -135,    // i4X
                -447    // i4Y
            },
            // DNP
            {
                -46,    // i4X
                -304    // i4Y
            },
            // D65
            {
                129,    // i4X
                -265    // i4Y
            },
            // DF
            {
                0,    // i4X
                0    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                -64,    // i4X
                -341    // i4Y
            },
            // Horizon
            {
                -537,    // i4X
                -263    // i4Y
            },
            // A
            {
                -408,    // i4X
                -286    // i4Y
            },
            // TL84
            {
                -228,    // i4X
                -329    // i4Y
            },
            // CWF
            {
                -233,    // i4X
                -404    // i4Y
            },
            // DNP
            {
                -114,    // i4X
                -285    // i4Y
            },
            // D65
            {
                65,    // i4X
                -287    // i4Y
            },
            // DF
            {
                0,    // i4X
                0    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                836,    // i4R
                512,    // i4G
                802    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                576,    // i4G
                1799    // i4B
            },
            // A 
            {
                540,    // i4R
                512,    // i4G
                1329    // i4B
            },
            // TL84 
            {
                695,    // i4R
                512,    // i4G
                1036    // i4B
            },
            // CWF 
            {
                781,    // i4R
                512,    // i4G
                1126    // i4B
            },
            // DNP 
            {
                725,    // i4R
                512,    // i4G
                822    // i4B
            },
            // D65 
            {
                873,    // i4R
                512,    // i4G
                615    // i4B
            },
            // DF 
            {
                512,    // i4R
                512,    // i4G
                512    // i4B
            }
        },
        // Rotation matrix parameter
        {
            13,    // i4RotationAngle
            249,    // i4Cos
            58    // i4Sin
        },
        // Daylight locus parameter
        {
            -201,    // i4SlopeNumerator
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
            -278,    // i4RightBound
            -928,    // i4LeftBound
            -224,    // i4UpperBound
            -324    // i4LowerBound
            },
            // Warm fluorescent
            {
            -278,    // i4RightBound
            -928,    // i4LeftBound
            -324,    // i4UpperBound
            -444    // i4LowerBound
            },
            // Fluorescent
            {
            -164,    // i4RightBound
            -278,    // i4LeftBound
            -215,    // i4UpperBound
            -366    // i4LowerBound
            },
            // CWF
            {
            -164,    // i4RightBound
            -278,    // i4LeftBound
            -366,    // i4UpperBound
            -454    // i4LowerBound
            },
            // Daylight
            {
            90,    // i4RightBound
            -164,    // i4LeftBound
            -207,    // i4UpperBound
            -367    // i4LowerBound
            },
            // Shade
            {
            450,    // i4RightBound
            90,    // i4LeftBound
            -207,    // i4UpperBound
            -367    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            90,    // i4RightBound
            -164,    // i4LeftBound
            -367,    // i4UpperBound
            -450    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            450,    // i4RightBound
            -928,    // i4LeftBound
            0,    // i4UpperBound
            -454    // i4LowerBound
            },
            // Daylight
            {
            115,    // i4RightBound
            -164,    // i4LeftBound
            -207,    // i4UpperBound
            -367    // i4LowerBound
            },
            // Cloudy daylight
            {
            215,    // i4RightBound
            40,    // i4LeftBound
            -207,    // i4UpperBound
            -367    // i4LowerBound
            },
            // Shade
            {
            315,    // i4RightBound
            40,    // i4LeftBound
            -207,    // i4UpperBound
            -367    // i4LowerBound
            },
            // Twilight
            {
            -164,    // i4RightBound
            -324,    // i4LeftBound
            -207,    // i4UpperBound
            -367    // i4LowerBound
            },
            // Fluorescent
            {
            115,    // i4RightBound
            -333,    // i4LeftBound
            -237,    // i4UpperBound
            -454    // i4LowerBound
            },
            // Warm fluorescent
            {
            -308,    // i4RightBound
            -508,    // i4LeftBound
            -237,    // i4UpperBound
            -454    // i4LowerBound
            },
            // Incandescent
            {
            -308,    // i4RightBound
            -508,    // i4LeftBound
            -207,    // i4UpperBound
            -367    // i4LowerBound
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
            797,    // i4R
            512,    // i4G
            713    // i4B
            },
            // Cloudy daylight
            {
            929,    // i4R
            512,    // i4G
            556    // i4B
            },
            // Shade
            {
            978,    // i4R
            512,    // i4G
            513    // i4B
            },
            // Twilight
            {
            638,    // i4R
            512,    // i4G
            1019    // i4B
            },
            // Fluorescent
            {
            805,    // i4R
            512,    // i4G
            868    // i4B
            },
            // Warm fluorescent
            {
            594,    // i4R
            512,    // i4G
            1411    // i4B
            },
            // Incandescent
            {
            540,    // i4R
            512,    // i4G
            1330    // i4B
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
            7820    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            10,    // i4SliderValue
            5593    // i4OffsetThr
            },
            // Shade
            {
            20,    // i4SliderValue
            949    // i4OffsetThr
            },
            // Daylight WB gain
            {
            728,    // i4R
            512,    // i4G
            824    // i4B
            },
            // Preference gain: strobe
            {
            520,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            512,    // i4R
            512,    // i4G
            600    // i4B
            },
            // Preference gain: warm fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: fluorescent
            {
            520,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: CWF
            {
            520,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight
            {
            520,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: shade
            {
            520,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight fluorescent
            {
            520,    // i4R
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
                -602,    // i4RotatedXCoordinate[0]
                -473,    // i4RotatedXCoordinate[1]
                -293,    // i4RotatedXCoordinate[2]
                -179,    // i4RotatedXCoordinate[3]
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



#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_hi545raw.h"
#include "camera_info_hi545raw.h"
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
        68900,    // i4R_AVG
        13968,    // i4R_STD
        91462,    // i4B_AVG
        17983,    // i4B_STD
        {  // i4P00[9]
            4376598, -1592078, -238988, -669427, 3350496, -116817, -343429, -2453705, 5362247
        },
        {  // i4P10[9]
            -908551, 845691, 23088, -132483, -1151652, 1276709, -335080, 141261, 214156
        },
        {  // i4P01[9]
            -1011824, 1011796, -29702, -169898, -1193324, 1357441, -347085, -1020401, 1382527
        },
        {  // i4P20[9]
            3351462, -3722024, 405070, -111816, 1708284, -1626304, 625976, 1554615, -2195932
        },
        {  // i4P11[9]
            8489394, -9343578, 975906, -230433, 4360682, -4156925, 1503820, 2093233, -3657887
        },
        {  // i4P02[9]
            5204336, -5856890, 744584, -43076, 2800656, -2756612, 942754, 120074, -1109503
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
            1152,    // u4MinGain, 1024 base = 1x
            10240,    // u4MaxGain, 16x
            49,    // u4MiniISOGain, ISOxx  
            128,    // u4GainStepUnit, 1x/8 
            16,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            16,    // u4VideoExpUnit  
            30,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            16,    // u4CapExpUnit 
            30,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            24,    // u4LensFno, Fno = 2.8
            0    // u4FocusLength_100x
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
            {82, 108, 128, 148, 170},    // u4HistStretchThres[AE_CCT_STRENGTH_NUM] 
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
            57,    // u4AETarget
            0,    // u4StrobeAETarget
            50,    // u4InitIndex
            4,    // u4BackLightWeight
            32,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            2,    // u4BlackLightStrengthIndex
            0,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            -3,    // i4BVOffset delta BV = value/10 
            64,    // u4PreviewFlareOffset
            64,    // u4CaptureFlareOffset
            1,    // u4CaptureFlareThres
            64,    // u4VideoFlareOffset
            1,    // u4VideoFlareThres
            32,    // u4StrobeFlareOffset
            1,    // u4StrobeFlareThres
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
                893,    // i4R
                512,    // i4G
                664    // i4B
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
                -368,    // i4X
                -265    // i4Y
            },
            // A
            {
                -280,    // i4X
                -298    // i4Y
            },
            // TL84
            {
                -156,    // i4X
                -312    // i4Y
            },
            // CWF
            {
                -134,    // i4X
                -391    // i4Y
            },
            // DNP
            {
                -19,    // i4X
                -338    // i4Y
            },
            // D65
            {
                110,    // i4X
                -301    // i4Y
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
                0,    // i4X
                0    // i4Y
            },
            // Horizon
            {
                -368,    // i4X
                -265    // i4Y
            },
            // A
            {
                -280,    // i4X
                -298    // i4Y
            },
            // TL84
            {
                -156,    // i4X
                -312    // i4Y
            },
            // CWF
            {
                -134,    // i4X
                -391    // i4Y
            },
            // DNP
            {
                -19,    // i4X
                -338    // i4Y
            },
            // D65
            {
                110,    // i4X
                -301    // i4Y
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
                512,    // i4R
                512,    // i4G
                512    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                589,    // i4G
                1385    // i4B
            },
            // A 
            {
                525,    // i4R
                512,    // i4G
                1119    // i4B
            },
            // TL84 
            {
                633,    // i4R
                512,    // i4G
                965    // i4B
            },
            // CWF 
            {
                725,    // i4R
                512,    // i4G
                1043    // i4B
            },
            // DNP 
            {
                788,    // i4R
                512,    // i4G
                830    // i4B
            },
            // D65 
            {
                893,    // i4R
                512,    // i4G
                664    // i4B
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
            0,    // i4RotationAngle
            256,    // i4Cos
            0    // i4Sin
        },
        // Daylight locus parameter
        {
            -119,    // i4SlopeNumerator
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
            -206,    // i4RightBound
            -856,    // i4LeftBound
            -231,    // i4UpperBound
            -331    // i4LowerBound
            },
            // Warm fluorescent
            {
            -206,    // i4RightBound
            -856,    // i4LeftBound
            -331,    // i4UpperBound
            -451    // i4LowerBound
            },
            // Fluorescent
            {
            -69,    // i4RightBound
            -206,    // i4LeftBound
            -226,    // i4UpperBound
            -351    // i4LowerBound
            },
            // CWF
            {
            -69,    // i4RightBound
            -206,    // i4LeftBound
            -351,    // i4UpperBound
            -441    // i4LowerBound
            },
            // Daylight
            {
            135,    // i4RightBound
            -69,    // i4LeftBound
            -221,    // i4UpperBound
            -381    // i4LowerBound
            },
            // Shade
            {
            495,    // i4RightBound
            135,    // i4LeftBound
            -221,    // i4UpperBound
            -381    // i4LowerBound
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
            495,    // i4RightBound
            -856,    // i4LeftBound
            -196,    // i4UpperBound
            -451    // i4LowerBound
            },
            // Daylight
            {
            160,    // i4RightBound
            -69,    // i4LeftBound
            -221,    // i4UpperBound
            -381    // i4LowerBound
            },
            // Cloudy daylight
            {
            260,    // i4RightBound
            85,    // i4LeftBound
            -221,    // i4UpperBound
            -381    // i4LowerBound
            },
            // Shade
            {
            360,    // i4RightBound
            85,    // i4LeftBound
            -221,    // i4UpperBound
            -381    // i4LowerBound
            },
            // Twilight
            {
            -69,    // i4RightBound
            -229,    // i4LeftBound
            -221,    // i4UpperBound
            -381    // i4LowerBound
            },
            // Fluorescent
            {
            160,    // i4RightBound
            -256,    // i4LeftBound
            -251,    // i4UpperBound
            -441    // i4LowerBound
            },
            // Warm fluorescent
            {
            -180,    // i4RightBound
            -380,    // i4LeftBound
            -251,    // i4UpperBound
            -441    // i4LowerBound
            },
            // Incandescent
            {
            -180,    // i4RightBound
            -380,    // i4LeftBound
            -221,    // i4UpperBound
            -381    // i4LowerBound
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
            818,    // i4R
            512,    // i4G
            724    // i4B
            },
            // Cloudy daylight
            {
            972,    // i4R
            512,    // i4G
            609    // i4B
            },
            // Shade
            {
            1040,    // i4R
            512,    // i4G
            569    // i4B
            },
            // Twilight
            {
            629,    // i4R
            512,    // i4G
            942    // i4B
            },
            // Fluorescent
            {
            766,    // i4R
            512,    // i4G
            873    // i4B
            },
            // Warm fluorescent
            {
            560,    // i4R
            512,    // i4G
            1195    // i4B
            },
            // Incandescent
            {
            527,    // i4R
            512,    // i4G
            1124    // i4B
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
            5989    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            50,    // i4SliderValue
            4307    // i4OffsetThr
            },
            // Shade
            {
            50,    // i4SliderValue
            341    // i4OffsetThr
            },
            // Daylight WB gain
            {
            750,    // i4R
            512,    // i4G
            790    // i4B
            },
            // Preference gain: strobe
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            480,    // i4R
            512,    // i4G
            540    // i4B
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
                -478,    // i4RotatedXCoordinate[0]
                -390,    // i4RotatedXCoordinate[1]
                -266,    // i4RotatedXCoordinate[2]
                -129,    // i4RotatedXCoordinate[3]
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



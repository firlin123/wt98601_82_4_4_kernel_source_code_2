//s_add new sensor driver here
//export funtions

UINT32 GC0328_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 SP2529_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 NT99252_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 A5142_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 SP0718_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 HI545_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
 
//! Add Sensor Init function here
//! Note:
//! 1. Add by the resolution from ""large to small"", due to large sensor
//!    will be possible to be main sensor.
//!    This can avoid I2C error during searching sensor.
//! 2. This file should be the same as mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp
ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT kdSensorList[MAX_NUM_OF_SUPPORT_SENSOR+1] =
{
//----------------------------5m-----------------------
#if defined(GC0328_YUV)
    {GC0328_SENSOR_ID, SENSOR_DRVNAME_GC0328_YUV, GC0328_YUV_SensorInit},
#endif

#if defined(HI545_MIPI_RAW)
  {HI545MIPI_SENSOR_ID, SENSOR_DRVNAME_HI545_MIPI_RAW,HI545_MIPI_RAW_SensorInit},
#endif
//----------------------------2m------------------------
#if defined(SP2529_YUV)
    {SP2529_SENSOR_ID, SENSOR_DRVNAME_SP2529_YUV, SP2529_YUV_SensorInit}, 
#endif
#if defined(SP0718_YUV)
    {SP0718_SENSOR_ID, SENSOR_DRVNAME_SP0718_YUV, SP0718_YUV_SensorInit}, 
#endif
#if defined(NT99252_YUV)
    {NT99252_SENSOR_ID, SENSOR_DRVNAME_NT99252_YUV, NT99252_YUV_SensorInit}, 
#endif
#if defined(A5142_MIPI_RAW)
  {A5142MIPI_SENSOR_ID, SENSOR_DRVNAME_A5142_MIPI_RAW,A5142_MIPI_RAW_SensorInit},
#endif

/*  ADD sensor driver before this line */
    {0,{0},NULL}, //end of list
};
//e_add new sensor driver here


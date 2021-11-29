#include <lcm_drv.h>

extern LCM_DRIVER hx8394_hd720_dsi_vdo_lcm_drv;
extern LCM_DRIVER ili9806_dsi_vdo_lcm_drv;
extern LCM_DRIVER otm1280a_hd720_dsi_cmd_drv;
extern LCM_DRIVER otm1282a_hd720_dsi_cmd_drv;
extern LCM_DRIVER nt35521_hd720_dsi_vdo_lcm_drv;
extern LCM_DRIVER otm1283a_hd720_dsi_vdo_lcm_drv;

LCM_DRIVER* lcm_driver_list[] = 
{ 
#if defined(OTM1283A_HD720_DSI_VDO)
	&otm1283a_hd720_dsi_vdo_lcm_drv, 
#endif
#if defined(NT35521_HD720_DSI_VDO)	
	&nt35521_hd720_dsi_vdo_lcm_drv, 
#endif

#if defined(HX8394_HD720_DSI_VDO)
    &hx8394_hd720_dsi_vdo_lcm_drv,
#endif

#if defined(ILI9806C_DSI_VDO)
&ili9806_dsi_vdo_lcm_drv,
#endif

#if defined(OTM1280A_HD720_DSI_CMD)	
	&otm1280a_hd720_dsi_cmd_drv, 
#endif

#if defined(OTM1282A_HD720_DSI_CMD)	
	&otm1282a_hd720_dsi_cmd_drv, 
#endif
};

#define LCM_COMPILE_ASSERT(condition) LCM_COMPILE_ASSERT_X(condition, __LINE__)
#define LCM_COMPILE_ASSERT_X(condition, line) LCM_COMPILE_ASSERT_XX(condition, line)
#define LCM_COMPILE_ASSERT_XX(condition, line) char assertion_failed_at_line_##line[(condition)?1:-1]

unsigned int lcm_count = sizeof(lcm_driver_list)/sizeof(LCM_DRIVER*);
//LCM_COMPILE_ASSERT(0 != sizeof(lcm_driver_list)/sizeof(LCM_DRIVER*));

#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw stk3x1x_cust_alsps_hw = {
    .i2c_num = 2,
	.polling_mode_ps = 0,
	.polling_mode_als = 1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x46, 0x0, 0x0, 0x00},
	.als_level	= {1,  100,  300, 400, 500,  660, 1000, 1360,  2000, 2500, 3000, 3500, 4000,  5000,  10000},
	.als_value	= {10,	40,   40,  40,  40,   40,  320,  320,   640,  640, 1280, 1280, 2600,  2600,  10240, 10240},
	.ps_threshold_high = 126, 
	.ps_threshold_low = 77,  
};
struct alsps_hw *stk3x1x_get_cust_alsps_hw(void) {
    return &stk3x1x_cust_alsps_hw;
}




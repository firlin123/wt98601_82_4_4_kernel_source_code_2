#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw ltr558_cust_alsps_hw = {
    .i2c_num    = 2,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x46, 0x0, 0x0, 0x00},
	.als_level	= {1,	100,  380, 550, 800,  1200, 2000, 3000,  4500, 6000, 9000, 12000, 18000,  25000,  30000},
	.als_value	= {10,	 40,   40,  40,  40,   40,   320,  320,   640,  640, 1280,  1280,  2600,   2600,  10240, 10240},
	.ps_threshold_high = 200, 
	.ps_threshold_low = 150,  
};
struct alsps_hw *ltr558_get_cust_alsps_hw(void) {
    return &ltr558_cust_alsps_hw;
}




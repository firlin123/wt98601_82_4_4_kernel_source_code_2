#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
  .i2c_num    = 2,
  .polling_mode_ps =0,
  .polling_mode_als =1,
  .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
  .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
  //.i2c_addr   = {0x3C, 0x38, 0x3A, 0x00}, //
  .als_level  = { 1,100,500,800,1200,1650,2000,3000,4000,6000,8800,12000,18000,25000,35000},
  .als_value  = { 0, 90, 90, 90,  90,  90, 320, 320, 320, 320, 320,10240,10240,10240,10240,10240},
  .ps_threshold_high = 103,
  .ps_threshold_low = 78,
};

struct alsps_hw *get_cust_alsps_hw(void) {
	return &cust_alsps_hw;
}

 


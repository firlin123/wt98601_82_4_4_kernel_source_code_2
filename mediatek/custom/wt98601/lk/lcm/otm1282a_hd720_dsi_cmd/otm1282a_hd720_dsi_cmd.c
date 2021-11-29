#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									0
#if (LCM_DSI_CMD_MODE)
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)
#else
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)
#endif

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
//#define LCM_DSI_CMD_MODE									1

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
#if 1
	{0xff,3,{0x12,0x82,0x01}},	
	{0x00,1,{0x80}},	            //Orise mode enable
	{0xff,2,{0x12,0x82}},
	    
	{0x00,1,{0x00}},
	{0x35,1,{0x01}},	//FTE enable
		
	{0x00,1,{0x91}},
	{0xB3,2,{0x04,0x10}},	// 2 lanes

	{0x00,1,{0x84}},
	{0xff,2,{0x10,0x02}},	// 2 lanes
		
	{0x00,1,{0x00}},
	{0x1C,1,{0x00}},	//32
		
	{0x00,1,{0xA1}},
	{0xC1,1,{0xee}},		    	//Video Mode Sync Signal Control

	{0x00,1,{0x80}},
	{0xC4,4,{0x08,0x0C,0x00,0x00}},
	        
	{0x00,1,{0x90}},
	{0xC5,6,{0x12,0xD6,0xDF,0xDF,0x95,0xBA}},  //VGH & VGL


	{0x00,1,{0x00}},
	{0xD8,2,{0x29,0x29}},  //GVDDP & GVDDN

	{0x00,1,{0x00}},
	{0xD9,1,{0x87}},   //0x92-->0x87 tianma 0531 LSM  //VCOM

	{0x00,1,{0x80}},	            //TCON Setting 1
	{0xA5,3,{0x0F,0x00,0x01}},

	{0x00,1,{0x80}},				// TCON Setting 2
	{0xC0,14,{0x00,0x85,0x00,0x06,0x06,0x00,0x85,0x06,0x06,0x00,0x85,0x00,0x06,0x06}},

	{0x00,1,{0xa0}},				// SWR/G/B timing setting
	{0xC0,12,{0x00,0x00,0x00,0x09,0x00,0x00,0x1E,0x00,0x00,0x00,0x00,0x00}},
//	{0xC0,12,{0x00,0x00,0x00,0x13,0x00,0x1B,0x03,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0x80}},				// STV Setting
	{0xC2,8,{0x84,0x02,0x79,0x14,0x84,0x00,0x4D,0xA2}},

	{0x00,1,{0x90}},				// LTPS CKV Setting 1
	{0xC2,15,{0x83,0x00,0x01,0x0B,0x00,0x82,0x00,0x01,0x0B,0x00,0x81,0x01,0x01,0x0B,0x00}},
	{0x00,1,{0xa0}},				// LTPS CKV Setting 2
	{0xc2,15,{0x84,0x00,0x01,0x0B,0x00,0x83,0x00,0x01,0x0B,0x00,0x82,0x00,0x81,0x0B,0x00}},	//  XCLK timing setting
	{0x00,1,{0xB0}},				// LTPS CKV Setting 3
	{0xc2,10,{0x81,0x01,0x01,0x0B,0x00,0x84,0x00,0x01,0x0B,0x00}},

	{0x00,1,{0xF8}},				// LTPS SELR/G/B Setting 2
	{0xC2,1,{0x02}},	
		
	{0x00,1,{0xFA}},	
	{0xC2,3,{0x00,0x0c,0x01}},	
		
	{0x00,1,{0x90}},						// Power On Sequence Timing Setting 1 
	{0xCb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xA0}},						// Power On Sequence Timing Setting 2 
	{0xCb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xb0}},						// Power On Sequence Timing Setting 3 
	{0xCb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0x80}},				    	// Power On Sequence Timing Setting 4
	{0xf5,2,{0x21,0x18}},

	{0x00,1,{0xD2}},		
	{0xf5,1,{0x20}},

	{0x00,1,{0x91}},		
	{0xB0,1,{0x9E}},

	{0x00,1,{0x90}},		
	{0xA5,1,{0x00}},

	{0x00,1,{0xB3}},		
	{0xC0,1,{0x00}}, //0x00->0x11	                    //Panel Driving Mode: Dot Inversion,Pixel Inversion,Column Inversion...
		
	{0x00,1,{0xB4}},		
	{0xC0,1,{0x40}},	 	               //Õý·´’ß

	{0x00,1,{0xC0}},						// Power Off Sequence Timing Setting 1
	{0xCb,15,{0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x28,0x28}},

	{0x00,1,{0xD0}},						// Power Off Sequence Timing Setting 2
	{0xCb,15,{0x28,0x28,0x14,0x04,0x18,0x3C,0x04,0xF7,0x04,0x14,0x14,0x14,0x14,0x04,0x04}},
		
	{0x00,1,{0xE0}},						// Power Off Sequence Timing Setting 3
	{0xCb,15,{0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x15,0x15,0x07,0x14,0x14,0x00,0x00}},
		
	{0x00,1,{0xF0}},						// Abnormal Power Off Sequence Timing Setting
	{0xCb,12,{0xFF,0xFF,0xF3,0x03,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0x80}},						// U2D signal setting
	{0xCC,10,{0x01,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x02}},

	{0x00,1,{0xb0}},						// U2U signal setting
	{0xCC,10,{0x01,0x0A,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02}},


	{0x00,1,{0xA0}},						// CGOUT Signal Setting 1
	{0xCD,15,{0x02,0x2D,0x04,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D}},
	{0x00,1,{0xB0}},						// CGOUT Signal Setting 2
	{0xCD,15,{0x2D,0x2D,0x2D,0x14,0x01,0x0A,0x0C,0x0B,0x13,0x2D,0x2D,0x2D,0x2d,0x2d,0x2d}},
	{0x00,1,{0xC0}},
	{0xCD,10,{0x2d,0x2d,0x2d,0x27,0x28,0x29,0x2a,0x2b,0x1d,0x2d}},
	{0x00,1,{0xD0}},
	{0xCD,15,{0x2D,0x03,0x2D,0x05,0x2D,0x2D,0x2D,0x2d,0x2d,0x2d,0x2d,0x2d,0x2D,0x2D,0x2D}},
	 {0x00,1,{0xE0}},						// CGOUT Signal Setting 5
	{0xCD,15,{0x2D,0x2D,0x2D,0x14,0x01,0x0A,0x0C,0x0B,0x13,0x2D,0x2D,0x2D,0x2d,0x2d,0x2d}},
	{0x00,1,{0xF0}},
	{0xCD,10,{0x2d,0x2d,0x2d,0x27,0x28,0x29,0x2a,0x2b,0x1d,0x2d}},

	//-------------------------------------Video Mode Setting -------------------------------------------------//

	{0x00,1,{0xD0}},						//	LTPS SELR/G/B Setting (Video Mode)
	{0xC0,12,{0x00,0x00,0x00,0x13,0x00,0x1B,0x03,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0x80}},						//	LTPS STV Setting (Video Mode)
	{0xC3,8,{0x84,0x02,0x79,0x14,0x84,0x00,0x4D,0xA2}},

	{0x00,1,{0x90}},						//	LTPS CKV Setting (Video Mode)
	{0xC3,15,{0x83,0x00,0x01,0x0B,0x00,0x82,0x00,0x01,0x0B,0x00,0x81,0x01,0x01,0x0B,0x00}},

	{0x00,1,{0xA0}},						// LTPS CKV Setting 2 (Video Mode)
	{0xc3,15,{0x84,0x00,0x01,0x0B,0x00,0x83,0x00,0x01,0x0B,0x00,0x82,0x00,0x81,0x0B,0x00}},	
	{0x00,1,{0xB0}},						// LTPS CKV Setting 3 (Video Mode)
	{0xc3,10,{0x81,0x01,0x01,0x0B,0x00,0x84,0x00,0x01,0x0B,0x00}},
	{0x00,1,{0xEA}},						// LTPS CKV Setting 4 (Video Mode)
	{0xc3,3,{0x00,0x01,0x11}},
		
	//////////////     gamma  setting         ///////////////
	{0x00,1,{0x00}},   //R+          		
	{0xE1,24,{0x16,0x2C,0x31,0x3E,0x45,0x4C,0x55,0x64,0x6C,0x7A,0x84,0x8C,0x6E,0x68,0x62,0x55,0x45,0x23,0x1E,0x1B,0x19,0x17,0x16,0x15}},
		
	{0x00,1,{0x00}},   //R-          		
	{0xE2,24,{0x16,0x2C,0x31,0x3E,0x45,0x4C,0x55,0x64,0x6C,0x7A,0x84,0x8C,0x6E,0x68,0x62,0x55,0x45,0x23,0x1E,0x1B,0x19,0x17,0x16,0x15}},
		
	{0x00,1,{0x00}},   //G+          		
	{0xE3,24,{0x16,0x2C,0x31,0x3E,0x45,0x4C,0x55,0x64,0x6C,0x7A,0x84,0x8C,0x6E,0x68,0x62,0x55,0x45,0x23,0x1E,0x1B,0x19,0x17,0x16,0x15}},
		
	{0x00,1,{0x00}},    //G-         		
	{0xE4,24,{0x16,0x2C,0x31,0x3E,0x45,0x4C,0x55,0x64,0x6C,0x7A,0x84,0x8C,0x6E,0x68,0x62,0x55,0x45,0x23,0x1E,0x1B,0x19,0x17,0x16,0x15}},
		
	{0x00,1,{0x00}},    //B+         		
	{0xE5,24,{0x16,0x2C,0x31,0x3E,0x45,0x4C,0x55,0x64,0x6C,0x7A,0x84,0x8C,0x6E,0x68,0x62,0x55,0x45,0x23,0x1E,0x1B,0x19,0x17,0x16,0x15}},
		
	{0x00,1,{0x00}},    //B-         		
	{0xE6,24,{0x16,0x2C,0x31,0x3E,0x45,0x4C,0x55,0x64,0x6C,0x7A,0x84,0x8C,0x6E,0x68,0x62,0x55,0x45,0x23,0x1E,0x1B,0x19,0x17,0x16,0x15}},

		
#endif
//------------ sleep out ------------------------//
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	// Display ON
	{0x29, 1, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}},
	       
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    {0x13, 0, {}},
    {REGFLAG_DELAY, 20, {}},
    // Sleep Out
	{0x11, 0, {}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0x22, 0, {}},
    {REGFLAG_DELAY, 50, {}},
	// Display off sequence
	{0x28, 0, {}},
	{REGFLAG_DELAY, 20, {}},
    // Sleep Mode On
	{0x10, 0, {}},
	{REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	//{0xB9,	3,	{0xFF, 0x83, 0x69}},
	{REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
	//{0xC3, 1, {0xFF}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		unsigned int div2_real=0;
		unsigned int cycle_time = 0;
		unsigned int ui = 0;
		unsigned int hs_trail_m, hs_trail_n;
		#define NS_TO_CYCLE(n, c)	((n) / c + (( (n) % c) ? 1 : 0))

		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;


#if (LCM_DSI_CMD_MODE)
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_FALLING;
#endif

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_THREE_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=128;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 15;
		params->dsi.vertical_frontporch					= 15;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;
		params->dsi.horizontal_backporch				= 45;
		params->dsi.horizontal_frontporch				= 45;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		//1 Every lane speed		
		//params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps		
		//params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4			
		//params->dsi.fbk_div =0x12;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		params->dsi.PLL_CLOCK = 235;

}



static void lcm_init_register(void)
{
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);                              
}                                       

static void lcm_init(void)
{
    SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(100);    
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	lcm_init();
	
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_UBOOT
    char  buffer[3];
	int   array[4];
	
    if(lcm_esd_test)
    {
        lcm_esd_test = FALSE;
        return TRUE;
    }

    array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);
	
	//printk("otm1282a %s, buffer[0] = 0x%08x\n", __func__, buffer[0]);

	if(buffer[0]==0x9c)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
#endif
}

static unsigned int lcm_esd_recover(void)
{
    unsigned char para = 0;

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(120);
	  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
	  push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
    dsi_set_cmdq_V2(0x35, 1, &para, 1);     ///enable TE
    MDELAY(10);

    return TRUE;
}

static unsigned int lcm_compare_id(void)
{	
	return 1;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER otm1282a_hd720_dsi_cmd_drv = 
{
    .name			= "hd720",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
//	.set_backlight	= lcm_setbacklight,
//	.set_pwm        = lcm_setpwm,
//	.get_pwm        = lcm_getpwm,
//	.esd_check   = lcm_esd_check,
//      .esd_recover   = lcm_esd_recover,
	.compare_id    = lcm_compare_id,

};

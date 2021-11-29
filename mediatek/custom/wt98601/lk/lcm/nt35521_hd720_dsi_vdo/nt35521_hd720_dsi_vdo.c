/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
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

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0xFFFE
#define REGFLAG_END_OF_TABLE      							0xFFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID_NT35521                                      0x5521

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) 


static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[128];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
	
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
	{0xB1,2,{0x60,0x21}},
	{0xBC,2,{0x00,0x00}},
	{0xBD,5,{0x01,0xA3,0x20,0x10,0x01}},
	{0x6F,1,{0x02}},
	{0xB8,1,{0x08}},
	{0xBB,2,{0x11,0x11}},
	{0xB6,1,{0x01}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
	{0xB0,2,{0x02,0x02}},
	{0xB1,2,{0x02,0x02}},
	{0xB3,2,{0x38,0x38}},
	{0xB4,2,{0x11,0x11}},
	{0xB9,2,{0x44,0x44}},
	{0xBA,2,{0x24,0x24}},
	{0xBC,2,{0xA0,0x00}},
	{0xBD,2,{0xA0,0x00}},
	{0xBE,1,{0x50}},
	{0xC0,1,{0x0C}},
	{0xC4,2,{0x11,0x11}},
	{0xCA,1,{0x01}},
	{0xCE,1,{0x66}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},
	{0xEE,1,{0x02}},
	{0x6F,1,{0x00}},
	{0xB0,6,{0x00,0x9C,0x00,0xB5,0x00,0xDE}},
	{0x6F,1,{0x06}},
	{0xB0,6,{0x00,0xFB,0x01,0x12,0x01,0x36}},
	{0x6F,1,{0x0C}},
	{0xB0,4,{0x01,0x52,0x01,0x7C}},
	{0x6F,1,{0x00}},
	{0xB1,6,{0x01,0x9E,0x01,0xD0,0x01,0xF8}},
	{0x6F,1,{0x06}},
	{0xB1,6,{0x02,0x32,0x02,0x61,0x02,0x62}},
	{0x6F,1,{0x0C}},
	{0xB1,4,{0x02,0x8D,0x02,0xBF}},
	{0x6F,1,{0x00}},
	{0xB2,6,{0x02,0xDA,0x03,0x03,0x03,0x21}},
	{0x6F,1,{0x06}},
	{0xB2,6,{0x03,0x47,0x03,0x61,0x03,0x80}},
	{0x6F,1,{0x0C}},
	{0xB2,4,{0x03,0x92,0x03,0xA7}},
	{0xB3,4,{0x03,0xBB,0x03,0xC2}},
	{0x6F,1,{0x00}},
	{0xBC,6,{0x00,0x4B,0x00,0x65,0x00,0x90}},
	{0x6F,1,{0x06}},
	{0xBC,6,{0x00,0xAE,0x00,0xC7,0x00,0xEE}},
	{0x6F,1,{0x0C}},
	{0xBC,4,{0x01,0x0D,0x01,0x3E}},
	{0x6F,1,{0x00}},
	{0xBD,6,{0x01,0x65,0x01,0xA3,0x01,0xD4}},
	{0x6F,1,{0x06}},
	{0xBD,6,{0x02,0x1E,0x02,0x58,0x02,0x5A}},
	{0x6F,1,{0x0C}},
	{0xBD,4,{0x02,0x8D,0x02,0xBF}},
	{0x6F,1,{0x00}},
	{0xBE,6,{0x02,0xE1,0x03,0x0D,0x03,0x2A}},
	{0x6F,1,{0x06}},
	{0xBE,6,{0x03,0x4F,0x03,0x67,0x03,0x84}},
	{0x6F,1,{0x0C}},
	{0xBE,4,{0x03,0x95,0x03,0xA9}},
	{0x6F,1,{0x00}},
	{0xBF,4,{0x03,0xBC,0x03,0xC2}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},
	{0xB0,2,{0x00,0x10}},
	{0xB1,2,{0x12,0x14}},
	{0xB2,2,{0x16,0x18}},
	{0xB3,2,{0x1A,0x2A}},
	{0xB4,2,{0x29,0x02}},
	{0xB5,2,{0x34,0x31}},
	{0xB6,2,{0x34,0x31}},
	{0xB7,2,{0x31,0x34}},
	{0xB8,2,{0x34,0x04}},
	{0xB9,2,{0x34,0x34}},
	{0xBA,2,{0x34,0x34}},
	{0xBB,2,{0x05,0x34}},
	{0xBC,2,{0x31,0x31}},
	{0xBD,2,{0x31,0x34}},
	{0xBE,2,{0x31,0x34}},
	{0xBF,2,{0x03,0x29}},
	{0xC0,2,{0x2A,0x1B}},
	{0xC1,2,{0x19,0x17}},
	{0xC2,2,{0x15,0x13}},
	{0xC3,2,{0x11,0x01}},
	{0xE5,2,{0x31,0x31}},
	{0xC4,2,{0x03,0x1B}},
	{0xC5,2,{0x19,0x17}},     
	{0xC6,2,{0x15,0x13}},
	{0xC7,2,{0x11,0x2A}},
	{0xC8,2,{0x29,0x01}},
	{0xC9,2,{0x34,0x31}},
	{0xCA,2,{0x34,0x31}},
	{0xCB,2,{0x31,0x34}},
	{0xCC,2,{0x34,0x05}},
	{0xCD,2,{0x34,0x34}},
	{0xCE,2,{0x34,0x34}},
	{0xCF,2,{0x04,0x34}},
	{0xD0,2,{0x31,0x31}},
	{0xD1,2,{0x31,0x34}},
	{0xD2,2,{0x31,0x34}},
	{0xD3,2,{0x00,0x29}},
	{0xD4,2,{0x2A,0x10}},
	{0xD5,2,{0x12,0x14}},
	{0xD6,2,{0x16,0x18}},
	{0xD7,2,{0x1A,0x02}},
	{0xE5,2,{0x31,0x31}},
	{0xE6,2,{0x31,0x31}},
	{0xD8,5,{0x00,0x00,0x00,0x14,0x00}},
	{0xD9,5,{0x00,0x15,0x00,0x00,0x00}},
	{0xE7,1,{0x00}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},
	{0xB1,2,{0x00,0x00}},
	{0xB0,2,{0x00,0x00}},
	{0xB2,5,{0x05,0x00,0x00,0x00,0x00}},
	{0xB3,5,{0x05,0x00,0x00,0x00,0x00}},
	{0xB4,5,{0x05,0x00,0x00,0x00,0x00}},
	{0xB3,5,{0x05,0x00,0x17,0x00,0x00}},
	{0xB6,5,{0x12,0x00,0x19,0x00,0x00}},
	{0xB7,5,{0x12,0x00,0x19,0x00,0x00}},
	{0xB8,5,{0x12,0x00,0x19,0x00,0x00}},
	{0xB9,5,{0x12,0x00,0x19,0x00,0x00}},
	{0xBA,5,{0x57,0x00,0x00,0x00,0x00}},
	{0xBB,5,{0x57,0x00,0x00,0x00,0x00}},
	{0xBC,5,{0x75,0x00,0x1A,0x00,0x00}},
	{0xBD,5,{0x53,0x00,0x1A,0x00,0x00}},
	{0xC0,4,{0x00,0x34,0x00,0x00}},
	{0xC1,4,{0x00,0x34,0x00,0x00}},
	{0xC2,4,{0x00,0x34,0x00,0x00}},
	{0xC3,4,{0x00,0x34,0x00,0x00}},
	{0xC4,1,{0x20}},
	{0xC5,1,{0x80}},
	{0xC6,1,{0x00}},
	{0xC7,1,{0x00}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},
	{0xED,1,{0x30}},
	{0xE5,1,{0x00}},
	{0xB0,2,{0x17,0x06}},
	{0xB8,1,{0x08}},
	{0xBD,5,{0x03,0x07,0x00,0x03,0x00}},
	{0xB1,2,{0x17,0x06}},
	{0xB9,2,{0x00,0x07}},
	{0xB2,2,{0x00,0x00}},
	{0xBA,2,{0x00,0x00}},
	{0xB3,2,{0x17,0x06}},
	{0xBB,2,{0x0A,0x03}},
	{0xB4,2,{0x17,0x06}},
	{0xB5,2,{0x17,0x06}},
	{0xB6,2,{0x14,0x03}},
	{0xB7,2,{0x00,0x00}},
	{0xBC,2,{0x02,0x00}},
	{0xE5,1,{0x06}},
	{0xE6,1,{0x06}},
	{0xE7,1,{0x00}},
	{0xE8,1,{0x06}},
	{0xE9,1,{0x06}},
	{0xEA,1,{0x06}},
	{0xEB,1,{0x00}},
	{0xEC,1,{0x00}},
	{0xC0,1,{0x07}},
	{0xC1,1,{0x80}},
	{0xC2,1,{0xA4}},
	{0xC3,1,{0x05}},
	{0xC4,1,{0x00}},
	{0xC5,1,{0x02}},
	{0xC6,1,{0x22}},
	{0xC7,1,{0x03}},
	{0xC8,2,{0x05,0x30}},
	{0xC9,2,{0x01,0x31}},
	{0xCA,2,{0x03,0x21}},
	{0xCB,2,{0x01,0x20}},
	{0xCC,3,{0x00,0x00,0x3E}},
	{0xCD,3,{0x00,0x00,0x3E}}, 
	{0xCE,3,{0x00,0x00,0x02}},  
	{0xCF,3,{0x00,0x00,0x02}}, 
	{0xD0,7,{0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xD1,5,{0x00,0x04,0xFE,0x07,0x10}},
	{0xD2,5,{0x10,0x05,0x04,0x03,0x10}},
	{0xD3,5,{0x20,0x00,0x48,0x07,0x10}},
	{0xD4,5,{0x30,0x00,0x43,0x07,0x10}},
	{0xD5,7,{0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x6F,1,{0x07}},
	{0xD5,4,{0x00,0x00,0x00,0x00}},
	{0xD6,7,{0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x6F,1,{0x07}},
	{0xD6,4,{0x00,0x00,0x00,0x00}},
	{0xD7,7,{0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x6F,1,{0x07}},
	{0xD7,4,{0x00,0x00,0x00,0x00}},
	{0xD8,5,{0x00,0x00,0x00,0x00,0x00}},
	{0x6F,1,{0x02}},
	{0xF7,1,{0x47}},
	{0x6F,1,{0x17}},
	{0xF4,1,{0x60}},
	{0x6F,1,{0x11}},
	{0xF3,1,{0x01}},
	{0x11,0,{}},     
	{REGFLAG_DELAY, 120, {}},
	{0x29,0,{}},
	{REGFLAG_DELAY, 20, {}},	
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
  
  
	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x0, {}}	
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


static void init_lcm_registers(void)
{
   unsigned int data_array[16];

    data_array[0] = 0x00043902;                          
    data_array[1] = 0x9483FFB9;                 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00113902;                          
    data_array[1] = 0x008211BA; //send one param for test
    data_array[2] = 0x1000c516;
    data_array[3] = 0x03240fff;
    data_array[4] = 0x20252421;
    data_array[5] = 0x00000008;
    dsi_set_cmdq(&data_array, 6, 1);
    MDELAY(1);


//0x12,0x82,0x00,0x16,0xC5,0x00,0x10,0xFF,0x0F,0x24,0x03,0x21,0x24,0x25,0x20,0x08



    data_array[0] = 0x00113902;                          
    data_array[1] = 0x040001B1; 
    data_array[2] = 0x11110187;
    data_array[3] = 0x3f3f372f;
    data_array[4] = 0xE6011247;
    data_array[5] = 0x000000E2;
    dsi_set_cmdq(&data_array, 6, 1); 
    MDELAY(1);
    
   // 0x01,0x00,0x04,0x87,0x01,0x11,0x11,0x2F,0x37,0x3F,0x3F,0x47,0x12,0x01,0xE6,0xE2
    
    data_array[0] = 0x00073902;                          
    data_array[1] = 0x08c800B2; 
    data_array[2] = 0x00220004;
    dsi_set_cmdq(&data_array, 3, 1); 
    MDELAY(1);
    
    //0x00,0xC8,0x08,0x04,0x00,0x22

    data_array[0] = 0x00173902;                          
    data_array[1] = 0x320680B4; 
    data_array[2] = 0x15320310; 
    data_array[3] = 0x08103208;
    data_array[4] = 0x05430433;
    data_array[5] = 0x06430437;
    data_array[6] = 0x00066161;
    dsi_set_cmdq(&data_array, 7, 1);
    MDELAY(1);

//0x80,0x06,0x32,0x10,0x03,0x32,0x15,0x08,0x32,0x10,0x08,0x33,0x04,0x43,0x05,0x37,0x04,0x43,0x06,0x61,0x61,0x06
    data_array[0] = 0x00053902;                          
    data_array[1] = 0x100006Bf; 
    data_array[2] = 0x00000004;
    dsi_set_cmdq(&data_array, 3, 1); 
    MDELAY(1);

	//{0xBF,4,{0x06,0x00,0x10,0x04}},
	
	  data_array[0] = 0x00033902;                          
    data_array[1] = 0x00170CC0;
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000bB6; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x000032D4; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00213902;                          
    data_array[1] = 0x000000D5; 
    data_array[2] = 0x01000A00;
    data_array[3] = 0x0000cc00;
    data_array[4] = 0x88888800;
    data_array[5] = 0x88888888;
    data_array[6] = 0x01888888;
    data_array[7] = 0x01234567;
    data_array[8] = 0x88888823;
    data_array[9] = 0x00000088;

    dsi_set_cmdq(&data_array, 10, 1); 
    MDELAY(1);

//0x00,0x00,0x00,0x00,0x0A,0x00,0x01,0x00,0xCC,0x00,0x00,0x00,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x01,0x67,0x45,0x23,0x01,0x23,0x88,0x88,0x88,0x88}},
	
    data_array[0] = 0x00023902;                          
    data_array[1] = 0x000009CC; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00053902;                          
    data_array[1] = 0x001000C7;
    data_array[2] = 0x00000010;  
    dsi_set_cmdq(&data_array, 3, 1);
    MDELAY(1);

   
//#if using cabc,please open it
/*  data_array[0] = 0x00023902;                          
    data_array[1] = 0x0000FF51; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(5);

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x00002453; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(5);

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000155; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(5);*/

//#if use TE function,please open it
/*
    data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000035; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);*/


	
    data_array[0] = 0x002B3902;                          
    data_array[1] = 0x060400E0; 
    data_array[2] = 0x133F332b;                          
    data_array[3] = 0x0D0e0a34;
    data_array[4] = 0x13111311;                          
    data_array[5] = 0x04001710;
    data_array[6] = 0x3F332b06;                          
    data_array[7] = 0x0e0a3413;
    data_array[8] = 0x1113110d;                          
    data_array[9] = 0x0b171013;
    data_array[10] = 0x0b110717;
    data_array[11] = 0x00110717;
    dsi_set_cmdq(&data_array, 12, 1);
    MDELAY(5);

//0x00,0x04,0x06,0x2B,0x33,0x3F,0x13,0x34,0x0A,0x0E,0x0D,0x11,0x13,0x11,0x13,0x10,0x17,0x00,0x04,0x06,0x2B,0x33,0x3F,0x13,0x34,0x0A,0x0E,0x0D,0x11,0x13,0x11,0x13,0x10,0x17,0x0B,0x17,0x07,0x11,0x0B,0x17,0x07,0x11}

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x000032d4; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);
    
    data_array[0] = 0x00110500; // Sleep Out
    dsi_set_cmdq(&data_array, 1, 1); 
    MDELAY(400);

    data_array[0] = 0x00290500; // Display On
    dsi_set_cmdq(&data_array, 1, 1); 
    MDELAY(10);
    
    data_array[0] = 0x00023902;                          
    data_array[1] = 0x000141e4; 
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);
    
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
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    //params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity			= LCM_POLARITY_RISING;

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
    params->dsi.packet_size = 256;

    // Video mode setting		
    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;


    params->dsi.word_count = 720*3;	
/*
	params->dsi.vertical_sync_active				= 4; // 1
	params->dsi.vertical_backporch					= 16; // 16
	params->dsi.vertical_frontporch 				= 16; // 15
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 90; // 60
	params->dsi.horizontal_backporch				= 90; // 72
	params->dsi.horizontal_frontporch				= 129; // 108
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;


    // Bit rate calculation
    params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
    params->dsi.fbk_div =24;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	*/
/////QQQQQ
params->dsi.vertical_sync_active	=2;//
	params->dsi.vertical_backporch		= 8;//
	params->dsi.vertical_frontporch		= 6;//
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 30;//
	params->dsi.horizontal_backporch	= 80;//
	params->dsi.horizontal_frontporch	= 86;//
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;
        //params->dsi.HS_PRPR = 4;
    //    params->dsi.CLK_HS_PRPR = 6;
	// Bit rate calculation
	
	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4
	params->dsi.fbk_div =17;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);

    //init_lcm_registers();
  
  	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];

#if 1
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);
#endif

#if 0
    // Display Off
	data_array[0]=0x00280500;
	dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(120);

    // Sleep In
    data_array[0] = 0x00100500;
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(50);
#endif
}


static void lcm_resume(void)
{
    lcm_init();
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
    dsi_set_cmdq(&data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(&data_array, 3, 1);

    data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    dsi_set_cmdq(&data_array, 1, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(&data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{	
#if 1	
    unsigned int id=0;
	unsigned char buffer[3];
	unsigned int array[16];  
	unsigned int data_array[16];

	SET_RESET_PIN(1);
    MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);	
	SET_RESET_PIN(1);
	MDELAY(120); 

    data_array[0] = 0x00063902;
    data_array[1] = 0x52AA55F0;  
    data_array[2] = 0x00000108;                
    dsi_set_cmdq(&data_array, 3, 1); 

	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xC5, buffer, 3);
	id = buffer[1]; //we only need ID
    #ifdef BUILD_LK
		printf("%s, LK nt35590 debug: nt35590 id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
    #else
		printk("%s, LK nt35590 debug: nt35590 id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
    #endif

   // if(id == LCM_ID_NT35521)
    if(buffer[0]==0x55 && buffer[1]==0x21)
    	return 1;
    else
        return 0;     
#else
    return 1;
#endif
}


LCM_DRIVER nt35521_hd720_dsi_vdo_lcm_drv = 
{
	.name		    = "nt35521_hd720_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};



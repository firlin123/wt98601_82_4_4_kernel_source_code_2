/*
 * This file is part of the AP3216C sensor driver for MTK platform.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: YC Hou <yc.hou@liteonsemi.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3216c_mtk.c
 *
 * Summary:
 *	AP3216C sensor dirver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 05/11/12 YC		 Original Creation (Test version:1.0)
 * 05/30/12 YC		 Modify AP3216C_check_and_clear_intr return value and exchange
 *                   AP3216C_get_ps_value return value to meet our spec.
 * 05/30/12 YC		 Correct shift number in AP3216C_read_ps.
 * 05/30/12 YC		 Correct ps data formula.
 * 05/31/12 YC		 1. Change the reg in clear int function from low byte to high byte 
 *                      and modify the return value.
 *                   2. Modify the eint_work function to filter als int.
 * 06/04/12 YC		 Add PS high/low threshold instead of using the same value.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsen_helper.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <cust_gpio_usage.h>
#include "ap3216c_mtk.h"
#include "cust_ap3216c.h"

#include <linux/wakelock.h>
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define AP3216C_DEV_NAME     "AP3216C"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/
/*for interrup work mode support --add by liaoxl.lenovo 12.08.2011*/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
	
/*----------------------------------------------------------------------------*/
static struct i2c_client *AP3216C_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id AP3216C_i2c_id[] = {{AP3216C_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_ap3216c={ I2C_BOARD_INFO(AP3216C_DEV_NAME, 0x3C>>1)};

//static unsigned short AP3216C_force[] = {0x02, 0x3C, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const AP3216C_forces[] = { AP3216C_force, NULL };
//static struct i2c_client_address_data AP3216C_addr_data = { .forces = AP3216C_forces,};
/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int AP3216C_i2c_remove(struct i2c_client *client);
static int AP3216C_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int AP3216C_i2c_resume(struct i2c_client *client);

static struct AP3216C_priv *g_AP3216C_ptr = NULL;

#if defined(WT_DO_ALS_SMOOTH)
static unsigned int als_count = 0;
static int als_first_report = 1;
static int current_als_value = -1;
static int last_als_data = 0;
#endif
static int als_sleep_flag = 0; 

static struct wake_lock  ps_lock;

#define ALS_DO_AVR    //als do average
#ifdef ALS_DO_AVR
static int als_times = 0;
static int als_temp[ALS_AVR_COUNT] = {0};
static int first_cycle = 1;
#endif

#ifdef WT_ALS_SWITCH_AUTO
static int first_switch = 1;
#endif

#ifdef WT_ALS_ENABLE_SKIP
static int als_lux_pre = -1;
static int als_skip_count = CUST_ALS_SKIP_COUNT;
#endif

static bool ps_interrupt_flag = false;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;

/*----------------------------------------------------------------------------*/
struct AP3216C_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct AP3216C_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct AP3216C_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
#ifdef WT_ALS_SWITCH_AUTO
	u32 		als_level_ing[C_CUST_ALS_LEVEL-1];
#endif
    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_h;   /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_l;   /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver AP3216C_i2c_driver = {	
	.probe      = AP3216C_i2c_probe,
	.remove     = AP3216C_i2c_remove,
	.detect     = AP3216C_i2c_detect,
	.suspend    = AP3216C_i2c_suspend,
	.resume     = AP3216C_i2c_resume,
	.id_table   = AP3216C_i2c_id,
//	.address_data = &AP3216C_addr_data,
	.driver = {
//		.owner          = THIS_MODULE,
		.name           = AP3216C_DEV_NAME,
	},
};

static struct AP3216C_priv *AP3216C_obj = NULL;
static struct platform_driver AP3216C_alsps_driver;
/*----------------------------------------------------------------------------*/
int AP3216C_get_addr(struct alsps_hw *hw, struct AP3216C_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void AP3216C_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "AP3216C")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "AP3216C")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
#if 0
	else
	{
		if(on)
		{
			mt_set_gpio_dir(GPIO_LIGHT_LDO_EN_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_LIGHT_LDO_EN_PIN,GPIO_OUT_ONE);
		}
		else
		{
			mt_set_gpio_dir(GPIO_LIGHT_LDO_EN_PIN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_LIGHT_LDO_EN_PIN,GPIO_OUT_ZERO);
		}
	}	
#endif
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_enable_als(struct i2c_client *client, int enable)
{
		struct AP3216C_priv *obj = i2c_get_clientdata(client);
		u8 databuf[2];	  
		int res = 0;
		u8 buffer[1];
		u8 reg_value[1];

#ifdef ALS_DO_AVR
	int i;
	als_times = 0;
	for(i=0;i<ALS_AVR_COUNT;i++){
		als_temp[i] = 0;
	}
	first_cycle = 1;
#endif

#ifdef WT_ALS_SWITCH_AUTO
	first_switch = 1;
#endif
		
#ifdef WT_ALS_ENABLE_SKIP
	als_skip_count = CUST_ALS_SKIP_COUNT;
#endif
		
		if(client == NULL)
		{
			APS_DBG("CLIENT CANN'T EQUAL NULL\n");
			return -1;
		}
	
		buffer[0]=AP3216C_LSC_ENABLE;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		
		if(enable)
		{
			databuf[0] = AP3216C_LSC_ENABLE;	
			//databuf[1] = reg_value[0] |0x01; //@hfs
			databuf[1]= test_bit(CMC_BIT_PS, &obj->enable) ? 3:1;//By Kathy	
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			APS_DBG("AP3216C ALS enable\n");
		}
		else
		{
			databuf[0] = AP3216C_LSC_ENABLE;	
			//databuf[1] = reg_value[0] &0xFE; //@hfs
			databuf[1]= test_bit(CMC_BIT_PS, &obj->enable) ? 2:0;//By Kathy
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 0);
			APS_DBG("AP3216C ALS disable\n");
		}
		return 0;
		
	EXIT_ERR:
		APS_ERR("AP3216C_enable_als fail\n");
		return res;
}

/*----------------------------------------------------------------------------*/
static int AP3216C_enable_ps(struct i2c_client *client, int enable)
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

	buffer[0]=AP3216C_LSC_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	if(enable)
	{
		databuf[0] = AP3216C_LSC_ENABLE;    
		//databuf[1] = reg_value[0] |0x02;
        databuf[1]= test_bit(CMC_BIT_ALS, &obj->enable) ? 3:2;//By Kathy	
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));

		mt_eint_unmask(CUST_EINT_ALS_NUM);
		APS_DBG("AP3216C PS enable\n");
	}
	else
	{
		databuf[0] = AP3216C_LSC_ENABLE;    
		//databuf[1] = reg_value[0] &0xfd;
        databuf[1]= test_bit(CMC_BIT_ALS, &obj->enable) ? 1:0;//By Kathy
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("AP3216C PS disable\n");

		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			mt_eint_mask(CUST_EINT_ALS_NUM);
		}
	}
	return 0;
	
EXIT_ERR:
	APS_ERR("AP3216C_enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_check_and_clear_intr(struct i2c_client *client) 
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[1], ints[1];

	/* Get Int status */
	buffer[0] = AP3216C_LSC_INT_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ints, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	/* Clear ALS int flag */
	buffer[0] = AP3216C_LSC_ADATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	/* Clear PS int flag */
	buffer[0] = AP3216C_LSC_PDATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	return ints[0];

EXIT_ERR:
	APS_ERR("AP3216C_check_and_clear_intr fail\n");
	return -1;
}
/*----------------------------------------------------------------------------*/
static bool  is_ps_in = false;
void AP3216C_eint_func(void)
{
	struct AP3216C_priv *obj = g_AP3216C_ptr;
	if(!obj)
	{
		return;
	}
	
	is_ps_in=true;
	schedule_work(&obj->eint_work);
}

/*----------------------------------------------------------------------------*/
// This function depends the real hw setting, customers should modify it. 2012/5/10 YC. 
int AP3216C_setup_eint(struct i2c_client *client)
{
    struct AP3216C_priv *obj = i2c_get_clientdata(client);        

    g_AP3216C_ptr = obj;

    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

    mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, AP3216C_eint_func, 0);

    mt_eint_unmask(CUST_EINT_ALS_NUM);  
    return 0;
}


int AP3216C_ldo_init(void)
{
//	mt_set_gpio_dir(GPIO_LIGHT_LDO_EN_PIN, GPIO_DIR_OUT);
//	mt_set_gpio_mode(GPIO_LIGHT_LDO_EN_PIN, GPIO_LIGHT_LDO_EN_PIN_M_GPIO);
	
	mdelay(1);
}


/*----------------------------------------------------------------------------*/
static int AP3216C_init_client(struct i2c_client *client)
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = AP3216C_LSC_ENABLE;    
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_LOW_THD_LOW;    
	databuf[1] = atomic_read(&obj->ps_thd_val_l) & 0x03;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_HIGH_THD_LOW;    
	databuf[1] = atomic_read(&obj->ps_thd_val_h) & 0x03;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_LOW_THD_HIGH;    
	databuf[1] = (atomic_read(&obj->ps_thd_val_l) & 0x3FC) >> 2;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_HIGH_THD_HIGH;    
	databuf[1] = (atomic_read(&obj->ps_thd_val_h) & 0x3FC) >> 2;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}
	
	//add for light calibration
	databuf[0] = AP3216C_LIGHT_CALIBRATION;    
	databuf[1] = CALIBRATION_VALUE_AP3216C;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}
	
	databuf[0] = AP3216C_ALS_CONFIGERATION; //ALS dynamic range
	databuf[1] = CUST_AP3216C_ALS_CONFIGERATION; 
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_PS_CONFIGERATION;    
	databuf[1] = CUST_AP3216C_PS_CONFIGERATION;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_PS_LEDCTRL;    
	databuf[1] = CUST_AP3216C_PS_LEDCTRL;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_PS_MEAN_TIME;    
	databuf[1] = CUST_AP3216C_PS_MEAN_TIME;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	if(res = AP3216C_setup_eint(client))
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	if((res = AP3216C_check_and_clear_intr(client)) < 0)
	{
		APS_ERR("check/clear intr: %d\n", res);
	}
	
	return AP3216C_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int AP3216C_read_als(struct i2c_client *client, u16 *data)
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);	 
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	int res = 0;
	int luxdata_int;
	if(is_ps_in)
	{
		return -1;
	}
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

	// get ALS adc count
	buffer[0]=AP3216C_LSC_ADATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	buffer[0]=AP3216C_LSC_ADATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	luxdata_int = (als_value_low[0] | (als_value_high[0]<<8));
	//printk("als_data is %d \n ",*data);

	if (luxdata_int < 0)
	{
		*data = 0;
		APS_DBG("als_value is invalid!!\n");
		return -1;
	}
	//------------------------------------
#ifdef ALS_DO_AVR
	int sum = 0;
	int i;
	als_temp[als_times]=luxdata_int;
     als_times++;

	if(first_cycle){
		for(i=0; i<als_times; i++){
			sum+=als_temp[i];
		}
		luxdata_int = sum / als_times;
	}else{
		for(i=0; i<ALS_AVR_COUNT; i++){
			sum+=als_temp[i];
		}
		luxdata_int = sum / ALS_AVR_COUNT;
	}

	if(als_times >= ALS_AVR_COUNT){
		als_times = 0;
		first_cycle = 0;
	}
#endif		
	//-------------------------------------
	*data = luxdata_int;

	return 0;

EXIT_ERR:
	APS_ERR("AP3216C_read_als fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_get_als_value(struct AP3216C_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;

#if defined(WT_DO_ALS_SMOOTH)
	unsigned int raw_data;
#endif

#ifdef WT_ALS_SWITCH_AUTO
	int idx_high = -1, idx_low = -1, i = 0;
	static int last_idx_high = -1, last_idx_low = -1;
	
	if(first_switch)
	{
		for(i = 0; i < obj->als_level_num; i++)
		{
			obj->als_level_ing[i] = obj->hw->als_level[i];
		}
		last_idx_high = -1;
		last_idx_low = -1;
		first_switch = 0;
	}
#endif

#ifdef WT_ALS_SWITCH_AUTO
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->als_level_ing[idx])
		{
			break;
		}
	}
#else
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}

		if(obj->hw->als_level[idx] == 0xFFFF)//great then the top level
		{
			break;
		}
	}
#endif
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}

#ifdef WT_ALS_SWITCH_AUTO
	if(obj->hw->als_level_high[idx] != obj->hw->als_level[idx])
	{
		obj->als_level_ing[idx] = obj->hw->als_level_high[idx];
		idx_high = idx;
	}
	if((idx > 0)&&(obj->hw->als_level_low[idx-1] != obj->hw->als_level[idx-1]))
	{
		obj->als_level_ing[idx-1] = obj->hw->als_level_low[idx-1];
		idx_low = idx - 1;
	}
	if((idx_low!=last_idx_low)||(idx_high!=last_idx_high))
	{
		for(i=0; i<obj->als_level_num; i++)
		{
			if((i == idx_low)||(i == idx_high))
			{
				continue;
			}
			obj->als_level_ing[i] = obj->hw->als_level[i];
		}
		last_idx_low = idx_low;
		last_idx_high = idx_high;
	}
#endif

	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
#if defined(WT_DO_ALS_SMOOTH)	
		if(als_sleep_flag == 1) 
			return current_als_value; // in sleep 

		raw_data = obj->hw->als_value[idx];

		if(raw_data == last_als_data)
		{
			als_count++;
		}
		else
		{
			als_count = 0;
			last_als_data = raw_data;
		}
		//normal
		if(als_count >= ALS_COUNT_FOR_SMOOTH)
		{
			als_count = 0;
			current_als_value = raw_data;
		}
		//after resume
		if(als_sleep_flag == 2 && als_count >= ALS_COUNT_FOR_SMOOTH/4)
		{
			als_count = 0;
			current_als_value = raw_data;
			als_sleep_flag = 0;
		}

		//the first time
		if(current_als_value == -1)
		{
			als_count = 0;
			last_als_data = raw_data;
			current_als_value = raw_data;
		}

		//APS_DBG("ALS: %05d => %05d, value:%d, count:%d, flag:%d\n", 
		//	als, obj->hw->als_value[idx], current_als_value, als_count, als_sleep_flag);
		//printk("als_value :%d => %d => %d\n",als,obj->hw->als_value[idx],current_als_value);
		return current_als_value;
#else

#ifdef WT_ALS_ENABLE_SKIP
		if(als_lux_pre == -1) //the first time
		{
			als_lux_pre = obj->hw->als_value[idx];
			als_skip_count = 0;
		}
		if(als_sleep_flag == 1) //in sleep
		{
			return als_lux_pre;
		}
		if(als_skip_count)	//skip CUST_ALS_SKIP_COUNT times
		{
			als_skip_count--;
		}
		else	
		{
			als_lux_pre = obj->hw->als_value[idx];	 // report real value
		}
		if(als_skip_count < 0)
		{
			als_skip_count = 0;
		}
		//printk("ALS :%d => %d, real = %d\n",als,als_lux_pre,obj->hw->als_value[idx]);
		return als_lux_pre;
#else
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		//printk("lidechun als_value is %d ",obj->hw->als_value[idx]);
		return obj->hw->als_value[idx];
#endif
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
int AP3216C_read_ps(struct i2c_client *client, u16 *data)
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);       
	u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	int res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

	buffer[0]=AP3216C_LSC_PDATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	buffer[0]=AP3216C_LSC_PDATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	*data = (ps_value_low[0] & 0x0f) | ((ps_value_high[0] & 0x3f) << 4);
	//printk("ps_data is %d \n ",*data);
	return 0;    

EXIT_ERR:
	APS_ERR("AP3216C_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
/* 
   for AP3216C_get_ps_value:
	return 1 = object close,
	return 0 = object far away. 2012/5/10 YC   // exchange 0 and 1 2012/5/30 YC 
*/
static int AP3216C_get_ps_value(struct AP3216C_priv *obj, u16 ps)
{
	int val;
	int invalid = 0;
	
#if 0
	static int val_temp = 1;

    if(ps_cali.valid == 1)
	{
	    
            if(ps > ps_cali.close)
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			else if(ps < ps_cali.far_away)
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
				val = val_temp;
	}
	else
	{
			if((ps > atomic_read(&obj->ps_thd_val_h)))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			//else if((ps < atomic_read(&obj->ps_thd_val_low))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val_low)))
			else if((ps < atomic_read(&obj->ps_thd_val_l)))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
				val = val_temp;	
	}
#endif

#if 1
	u8 buffer[2];

	buffer[0]=AP3216C_LSC_PDATA_L;
	i2c_master_send(obj->client, buffer, 0x1);
	i2c_master_recv(obj->client, buffer+1, 0x1);
	if (buffer[1] & 0x80){
		val = 0;
	}
	else{
		val = 1;
	}
#endif

	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		//printk("ps_value is %d ,val = %d \n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}


/*----------------------------------------------------------------------------*/
static void AP3216C_eint_work(struct work_struct *work)
{
	struct AP3216C_priv *obj = (struct AP3216C_priv *)container_of(work, struct AP3216C_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
	u8 databuf[2];
	int res = 0; 

	if((err = AP3216C_check_and_clear_intr(obj->client)) < 0)
	{
		APS_ERR("AP3216C_eint_work check intrs: %d\n", err);
	}

		AP3216C_read_ps(obj->client, &obj->ps);
		sensor_data.values[0] = AP3216C_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		APS_DBG("%s: rawdata ps=%d, ps_value=%d\n",__func__,obj->ps,sensor_data.values[0]);

		//@hfs---start 20130712 for ps up system cannot wake
         if(1 == sensor_data.values[0]) 
         {
             if( false == wake_lock_active(&ps_lock))
            {
            	wake_lock_timeout(&ps_lock, 5*HZ); //5s
            }
        }
        else   //wake unlock when close 
       	{
            wake_unlock(&ps_lock);
        }
		//@hfs---end 20130712 for ps up system cannot wake
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	ps_interrupt_flag = true;
	mt_eint_unmask(CUST_EINT_ALS_NUM);      
	is_ps_in=false;
  	return;
}

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
/*----------------------------------------------------------------------------*/
static ssize_t AP3216C_show_als(struct device_driver *ddri, char *buf)
{
	int als_data = 0;
	int als_enable, als_value;

	if(!AP3216C_obj)
	{
		APS_ERR("AP3216C_obj is null!!\n");
		return 0;
	}
	als_enable = test_bit(CMC_BIT_ALS, &AP3216C_obj->enable)?(1):(0);
	if(als_enable){
		AP3216C_read_als(AP3216C_obj->client, &als_data);
	}else{
		return snprintf(buf, PAGE_SIZE, "ALS is disable\n");
	}
	if(als_data < 0){
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", als_data);
	}else{
		als_value = AP3216C_get_als_value(AP3216C_obj, als_data);
	}
	return snprintf(buf, PAGE_SIZE, "%d => %d lux\n",als_data,als_value);
	
}
/*----------------------------------------------------------------------------*/
static ssize_t AP3216C_show_ps(struct device_driver *ddri, char *buf)
{
	int	ps_data = 0;
	int ps_value, ps_enable;
	 
	if(!AP3216C_obj)
	{
		APS_ERR("AP3216C_obj is null!!\n");
		return 0;
	}
	ps_enable = test_bit(CMC_BIT_PS, &AP3216C_obj->enable)?(1):(0);
	if(ps_enable){
		AP3216C_read_ps(AP3216C_obj->client, &ps_data);
	}else{
		return snprintf(buf, PAGE_SIZE, "PS is disable\n");
	}		
	if(ps_data < 0){
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", ps_data);
	}else{
		ps_value = AP3216C_get_ps_value(AP3216C_obj, ps_data);
	}
	return snprintf(buf, PAGE_SIZE, "%d => 0x%04X\n",ps_data,ps_value);
}

/*----------------------------------------------------------------------------*/
static ssize_t AP3216C_show_ps_thres(struct device_driver *ddri, char *buf)
{
	int ps_thres_close, ps_thres_far, valid;

	struct alsps_hw *hw = get_cust_alsps_hw();
	
		if(ps_cali.valid == 1)
		{
			ps_thres_close = ps_cali.close;
			ps_thres_far = ps_cali.far_away;
			valid = ps_cali.valid;
		}
		else if(ps_cali.valid == 0){
			ps_thres_close = hw->ps_threshold_high;
			ps_thres_far = hw->ps_threshold_low;
			valid = ps_cali.valid;
		}
	return snprintf(buf, PAGE_SIZE, "%d=>%d,%d\n",valid,ps_thres_close,ps_thres_far);
}

/*----------------------------------------------------------------------------*/
static ssize_t AP3216C_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int ps_enable, als_enable;
	if(!AP3216C_obj)
	{
		APS_ERR("AP3216C_obj is null!!\n");
		return 0;
	}
	als_enable = test_bit(CMC_BIT_ALS, &AP3216C_obj->enable)?(1):(0);
	ps_enable  = test_bit(CMC_BIT_PS,  &AP3216C_obj->enable)?(1):(0);
	
	if(AP3216C_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			AP3216C_obj->hw->i2c_num, AP3216C_obj->hw->power_id, AP3216C_obj->hw->power_vol);
	}else{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "Enable: als:%d ps:%d\n",als_enable,ps_enable);
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t AP3216C_show_reg(struct device_driver *ddri, char *buf)
{
	int i = 0;
	u8 bufdata;
	int count = 0;
	
	if(!AP3216C_obj)
	{
		APS_ERR("AP3216C_obj is null!!\n");
		return 0;
	}
	/*** 0x00, 0x01, 0x02***/
	for(i = 0;i < 3; i++)
	{
		hwmsen_read_byte(AP3216C_obj->client,0x00+i,&bufdata);
		count+= sprintf(buf+count,"[%x] = (%x)\n",0x00+i,bufdata);
	}
	/*** 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10***/
	for(i = 0;i < 7; i++)
	{
		hwmsen_read_byte(AP3216C_obj->client,0x0A+i,&bufdata);
		count+= sprintf(buf+count,"[%x] = (%x)\n",0x0A+i,bufdata);
	}
	/*** 0x19, 0x1A, 0x1B, 0x1C, 0x1D***/
	for(i = 0;i < 5; i++)
	{
		hwmsen_read_byte(AP3216C_obj->client,0x19+i,&bufdata);
		count+= sprintf(buf+count,"[%x] = (%x)\n",0x19+i,bufdata);
	}
	/*** 0x20, 0x21, 0x22, 0x23, 0x24***/
	for(i = 0;i < 5; i++)
	{
		hwmsen_read_byte(AP3216C_obj->client,0x20+i,&bufdata);
		count+= sprintf(buf+count,"[%x] = (%x)\n",0x20+i,bufdata);
	}
	/*** 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D***/
	for(i = 0;i < 6; i++)
	{
		hwmsen_read_byte(AP3216C_obj->client,0x28+i,&bufdata);
		count+= sprintf(buf+count,"[%x] = (%x)\n",0x28+i,bufdata);
	}

	return count;
}

static ssize_t AP3216C_store_reg(struct device_driver *ddri,char *buf,ssize_t count)
{
	u8 addr = 0,data = 0;
	
	if(!AP3216C_obj)
	{
		APS_ERR("AP3216C_obj is null\n");
		return 0;
	}
	else if(2 != sscanf(buf,"0x%02X 0x%02X",&addr,&data))
	{
		APS_ERR("invalid format:%s\n",buf);
		return 0;
	}
	hwmsen_write_byte(AP3216C_obj->client,addr,data);

	return count;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,	 S_IWUSR | S_IRUGO, AP3216C_show_als, NULL);
static DRIVER_ATTR(ps,	 S_IWUSR | S_IRUGO, AP3216C_show_ps,  NULL);
static DRIVER_ATTR(ps_thres,	S_IWUSR | S_IRUGO, AP3216C_show_ps_thres,  NULL);
static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, AP3216C_show_status, NULL);
static DRIVER_ATTR(reg,	 S_IWUSR | S_IRUGO, AP3216C_show_reg, AP3216C_store_reg);
/*----------------------------------------------------------------------------*/
static struct device_attribute *AP3216C_attr_list[] = {
	&driver_attr_als,
	&driver_attr_ps,
	&driver_attr_ps_thres,
	&driver_attr_status,
	&driver_attr_reg,
};
/*----------------------------------------------------------------------------*/
static int AP3216C_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(AP3216C_attr_list)/sizeof(AP3216C_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, AP3216C_attr_list[idx]))
		{
			APS_ERR("driver_create_file (%s) = %d\n", AP3216C_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
	static int AP3216C_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(AP3216C_attr_list)/sizeof(AP3216C_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, AP3216C_attr_list[idx]);
	}

	return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int AP3216C_open(struct inode *inode, struct file *file)
{
	file->private_data = AP3216C_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int AP3216C_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*begin: added by fangliang for ps nvram*/

static int AP3216C_init_client_for_cali(struct i2c_client *client)
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = AP3216C_LSC_ENABLE;    
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_LOW_THD_LOW;    
	databuf[1] = atomic_read(&obj->ps_thd_val_l) & 0x03;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_HIGH_THD_LOW;    
	databuf[1] = atomic_read(&obj->ps_thd_val_h) & 0x03;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_LOW_THD_HIGH;    
	databuf[1] = (atomic_read(&obj->ps_thd_val_l) & 0x3FC) >> 2;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_HIGH_THD_HIGH;    
	databuf[1] = (atomic_read(&obj->ps_thd_val_h) & 0x3FC) >> 2;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}
	
	//add for light calibration
	databuf[0] = AP3216C_LIGHT_CALIBRATION;    
	databuf[1] = CALIBRATION_VALUE_AP3216C;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}
	
	if(res = AP3216C_setup_eint(client))
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	if((res = AP3216C_check_and_clear_intr(client)) < 0)
	{
		APS_ERR("check/clear intr: %d\n", res);
	}
	
	return AP3216C_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

static void ap3216c_WriteCalibrationEx(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *data_cali)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	u8 databuf[2];	   
	int res;
		
	if(data_cali->valid == 1)
	{
		ps_cali.close = data_cali->close;
		ps_cali.far_away= data_cali->far_away;
		ps_cali.valid = 1;
		
#if 1
	databuf[0] = AP3216C_LSC_INT_LOW_THD_LOW;
	databuf[1] = (u8)(ps_cali.far_away & 0x03);    
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_LOW_THD_HIGH;	
	databuf[1] = (u8)((ps_cali.far_away &  0x3FC) >> 2);	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_HIGH_THD_LOW;  
	databuf[1] = (u8)(ps_cali.close & 0x03);	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		return AP3216C_ERR_I2C;
	}
	
	databuf[0] = AP3216C_LSC_INT_HIGH_THD_HIGH;  
	databuf[1] = (u8)((ps_cali.close & 0x3FC) >> 2);	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		return AP3216C_ERR_I2C;
	}
#endif 
	}
	else if(data_cali->valid == 0){
			ps_cali.close = hw->ps_threshold_high;
			ps_cali.far_away = hw->ps_threshold_low;
			ps_cali.valid = 0;
	}
}

static int ap3216c_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
     int i=0 ,err = 0,j = 0,sum=0;
	 u16 data[21] = {0},data_cali;

	 for(i = 0;i<20;i++)
	 	{
	 		mdelay(50);//mdelay(5);fangliang
			if(err ==  AP3216C_read_ps(client,&data[i]))
			{
				sum += data[i];
			}
			else
			{
				break;
			}
			mdelay(55);
	 	}

	 if(i == 20)
	 	{
			data_cali = sum/20;

			ps_data_cali->far_away = data_cali + CUST_AP3216C_PS_THRES_FAR;
			ps_data_cali->close = data_cali + CUST_AP3216C_PS_THRES_CLOSE;
			ps_data_cali->valid = 1;

			err= 0;
	 	}
	 else
	 	{
	 	ps_data_cali->valid = 0;
		APS_LOG("%s: data error \n",__func__);
	 	err=  -1;
	 	}
	 
	 return err;
}
/*end: added by fangliang for ps nvram*/
/*----------------------------------------------------------------------------*/
static int AP3216C_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct AP3216C_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	struct PS_CALI_DATA_STRUCT ps_cali_temp;// added by fangliang

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = AP3216C_enable_ps(obj->client, 1))
				{
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if(err = AP3216C_enable_ps(obj->client, 0))
				{
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if(ps_interrupt_flag){
			if(err = AP3216C_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = AP3216C_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			ps_interrupt_flag = false;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if(err = AP3216C_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = AP3216C_enable_als(obj->client, 1))
				{
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if(err = AP3216C_enable_als(obj->client, 0))
				{
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if(err = AP3216C_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = AP3216C_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if(err = AP3216C_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		case ALSPS_SET_PS_CALI:
			dat = (void __user*)arg;
			if(dat == NULL)
			{
				APS_LOG("dat == NULL\n");
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&ps_cali_temp,dat, sizeof(ps_cali_temp)))
			{
				APS_LOG("copy_from_user\n");
				err = -EFAULT;
				break;	  
			}
			ap3216c_WriteCalibrationEx(obj->client,&ps_cali_temp);
			APS_LOG(" ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid);
			
			break;
		case ALSPS_GET_PS_RAW_DATA_FOR_CALI:
			AP3216C_enable_ps(obj->client, 1);
			ap3216c_read_data_for_cali(obj->client,&ps_cali_temp);
			if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations AP3216C_fops = {
	.owner = THIS_MODULE,
	.open = AP3216C_open,
	.release = AP3216C_release,
	.unlocked_ioctl = AP3216C_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice AP3216C_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &AP3216C_fops,
};
/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	APS_FUN();    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
static void AP3216C_early_suspend(struct early_suspend *h) 
{   
	struct AP3216C_priv *obj = container_of(h, struct AP3216C_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 1);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = AP3216C_enable_als(obj->client, 0))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}

	als_sleep_flag = 1;
#if defined(WT_DO_ALS_SMOOTH)
	als_count = 0;
#endif
}
/*----------------------------------------------------------------------------*/
static void AP3216C_late_resume(struct early_suspend *h)
{   
	struct AP3216C_priv *obj = container_of(h, struct AP3216C_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = AP3216C_enable_als(obj->client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}

	als_sleep_flag = 2;
#if defined(WT_DO_ALS_SMOOTH)
	als_count = 0;
#endif
}

int AP3216C_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct AP3216C_priv *obj = (struct AP3216C_priv *)self;
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					if(err = AP3216C_enable_ps(obj->client, 1))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if(err = AP3216C_enable_ps(obj->client, 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				AP3216C_read_ps(obj->client, &obj->ps);
				
				sensor_data->values[0] = AP3216C_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int AP3216C_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct AP3216C_priv *obj = (struct AP3216C_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if(err = AP3216C_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if(err = AP3216C_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				AP3216C_read_als(obj->client, &obj->als);
								
				sensor_data->values[0] = AP3216C_get_als_value(obj, obj->als);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, AP3216C_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct AP3216C_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	AP3216C_obj = obj;

	obj->hw = get_cust_alsps_hw();
	AP3216C_get_addr(obj->hw, &obj->addr);

	INIT_WORK(&obj->eint_work, AP3216C_eint_work);
	
	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_h,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_l,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
	obj->als_modulus = (400*100*40)/(1*1500);

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
#ifdef WT_ALS_SWITCH_AUTO
	memcpy(obj->als_level_ing, obj->hw->als_level, sizeof(obj->als_level_ing));
#endif
	
	AP3216C_i2c_client = client;

	if(err = AP3216C_init_client(client))
	{
		goto exit_init_failed;
	}
	APS_LOG("AP3216C_init_client() OK!\n");
	
	if(err = misc_register(&AP3216C_device))
	{
		APS_ERR("AP3216C_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if(err = AP3216C_create_attr(&AP3216C_alsps_driver.driver))
	{
		APS_ERR("AP3216C_create_attr err = %d\n", err);
		goto exit_create_attr_failed;
	}

	obj_ps.self = AP3216C_obj;
	
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
	}

	obj_ps.sensor_operate = AP3216C_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = AP3216C_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = AP3216C_als_operate;
	if(err = hwmsen_attach(ID_LIGHT, &obj_als))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = AP3216C_early_suspend,
	obj->early_drv.resume   = AP3216C_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

    {
       #include <linux/hardware_info.h>
       hardwareinfo_set_prop(HARDWARE_ALSPS, "AP3216C");
	}

	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
		AP3216C_delete_attr(&AP3216C_alsps_driver.driver);
	exit_misc_device_register_failed:
		misc_deregister(&AP3216C_device);
	exit_init_failed:
	exit_kfree:
	kfree(obj);
	exit:
	AP3216C_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_remove(struct i2c_client *client)
{
	int err;	

	if(err = AP3216C_delete_attr(&AP3216C_alsps_driver.driver))
	{
		APS_ERR("AP3216C_delete_attr fail: %d\n", err);
	}
	if(err = misc_deregister(&AP3216C_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	AP3216C_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

//	AP3216C_ldo_init();
	AP3216C_power(hw, 1);    
//	AP3216C_force[0] = hw->i2c_num;
//	AP3216C_force[1] = hw->i2c_addr[0];
//	APS_DBG("I2C = %d, addr =0x%x\n",AP3216C_force[0],AP3216C_force[1]);
	if(i2c_add_driver(&AP3216C_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	AP3216C_power(hw, 0);    
	i2c_del_driver(&AP3216C_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver AP3216C_alsps_driver = {
	.probe      = AP3216C_probe,
	.remove     = AP3216C_remove,    
	.driver     = {
		.name  = "als_ps",
		.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/
static int __init AP3216C_init(void)
{
    APS_FUN();
    struct alsps_hw *hw = get_cust_alsps_hw();
    APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 

    i2c_register_board_info(hw->i2c_num, &i2c_ap3216c, 1);
    if(platform_driver_register(&AP3216C_alsps_driver))
    {
        APS_ERR("failed to register driver");
        return -ENODEV;
    }
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit AP3216C_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&AP3216C_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(AP3216C_init);
module_exit(AP3216C_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("YC Hou");
MODULE_DESCRIPTION("AP3216C driver");
MODULE_LICENSE("GPL");


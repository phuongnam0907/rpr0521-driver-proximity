/*
 *  rpr0521.c - Linux kernel modules for sensortek rpr301x, rpr321x and rpr331x
 *  proximity/ambient light sensor
 *
 *  Copyright (c) 2013, The Linux Foundation. All Rights Reserved.
 *  Copyright (C) 2012 Lex Hsieh / sensortek <lex_hsieh@sitronix.com.tw> or
 *   <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Linux Foundation chooses to take subject only to the GPLv2 license
 *  terms, and distributes only under these terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#define DRIVER_VERSION  "0.1.4"

/* Driver Settings */
#define CONFIG_RPR_PS_ALS_USE_CHANGE_THRESHOLD
#ifdef CONFIG_RPR_PS_ALS_USE_CHANGE_THRESHOLD
#define RPR_ALS_CHANGE_THD	20	/* The threshold to trigger ALS interrupt, unit: lux */
#endif	/* #ifdef CONFIG_RPR_PS_ALS_USE_CHANGE_THRESHOLD */
#define RPR_INT_PS_MODE			1	/* 1, 2, or 3	*/
#define RPR_POLL_PS
#define RPR_POLL_ALS		/* ALS interrupt is valid only when RPR_PS_INT_MODE = 1	or 4*/

/* Define Register Map */
#define RPR_SYS_CTL_REG			0x40
#define RPR_STATE_REG 			0x41
#define RPR_PSCTRL_REG 			0x43
#define RPR_ALSCTRL_REG 		0x42
#define RPR_INT_REG 			0x4A
#define RPR_THDH1_PS_REG 		0x4B
#define RPR_THDH2_PS_REG 		0x4C
#define RPR_THDL1_PS_REG 		0x4D
#define RPR_THDL2_PS_REG 		0x4E
#define RPR_THDH1_ALS_REG 		0x4F
#define RPR_THDH2_ALS_REG 		0x50
#define RPR_THDL1_ALS_REG 		0x51
#define RPR_THDL2_ALS_REG 		0x52
#define RPR_DATA1_PS_REG	 	0x44
#define RPR_DATA2_PS_REG 		0x45
#define RPR_DATA1_ALS_REG 		0x46
#define RPR_DATA2_ALS_REG 		0x47
#define RPR_DATA1_ALS_REG_1		0x48
#define RPR_DATA2_ALS_REG_1		0x49
#define RPR_DATA1_OFFSET_REG 	0x53
#define RPR_DATA2_OFFSET_REG 	0x54
#define RPR_PDT_ID_REG 			0x92
#define RPR_RSRVD_REG 			0x3F
#define RPR_SW_RESET_REG		0x80


/* Define state reg */
#define RPR_STATE_EN_IRS_SHIFT  	7
#define RPR_STATE_EN_AK_SHIFT  		6
#define RPR_STATE_EN_IRO_SHIFT  	4
#define RPR_STATE_EN_WAIT_SHIFT  	2
#define RPR_STATE_EN_ALS_SHIFT  	1
#define RPR_STATE_EN_PS_SHIFT  		0

// #define RPR_STATE_EN_IRS_MASK	0x80
#define RPR_STATE_EN_AK_MASK	0x40
#define RPR_STATE_EN_IRO_MASK	0x10

#define RPR_STATE_EN_ALS_MASK	BIT(7)
#define RPR_STATE_EN_PS_MASK	BIT(6)

/* Define PS ctrl reg */
#define RPR_PS_PRS_SHIFT  		6
#define RPR_PS_GAIN_SHIFT  		4
#define RPR_PS_IT_SHIFT  		0

#define RPR_PS_PRS_MASK			0xC0
#define RPR_PS_GAIN_MASK		0x30
#define RPR_PS_IT_MASK			0x0F

/* Define ALS ctrl reg */
#define RPR_ALS_PRS_SHIFT  		6
#define RPR_ALS_GAIN_SHIFT  	4
#define RPR_ALS_IT_SHIFT  		0

#define RPR_ALS_PRS_MASK		0xC0
#define RPR_ALS_GAIN_MASK		0x30
#define RPR_ALS_IT_MASK			0x0F

/* Define LED ctrl reg */
#define RPR_LED_IRDR_SHIFT  	6
#define RPR_LED_DT_SHIFT  		0

#define RPR_LED_IRDR_MASK		0xC0
#define RPR_LED_DT_MASK			0x3F

/* Define interrupt reg */
#define RPR_INT_CTRL_SHIFT  	7
#define RPR_INT_OUI_SHIFT  		4
#define RPR_INT_ALS_SHIFT  		3
#define RPR_INT_PS_SHIFT  		0

#define RPR_INT_CTRL_MASK		0x80
#define RPR_INT_OUI_MASK		0x10
#define RPR_INT_ALS_MASK		0x08
#define RPR_INT_PS_MASK			0x07

#define RPR_INT_ALS				0x08

/* Define flag reg */
#define RPR_FLG_ALSDR_SHIFT  		7
#define RPR_FLG_PSDR_SHIFT  		6
#define RPR_FLG_ALSINT_SHIFT  		5
#define RPR_FLG_PSINT_SHIFT  		4
#define RPR_FLG_OUI_SHIFT  			2
#define RPR_FLG_IR_RDY_SHIFT  		1
#define RPR_FLG_NF_SHIFT  			0

#define RPR_FLG_ALSDR_MASK		0x80
#define RPR_FLG_PSDR_MASK		0x40
#define RPR_FLG_ALSINT_MASK		0x20
#define RPR_FLG_PSINT_MASK		0x10
#define RPR_FLG_OUI_MASK		0x04
#define RPR_FLG_IR_RDY_MASK		0x02
#define RPR_FLG_NF_MASK			0x01

/* misc define */
#define MIN_ALS_POLL_DELAY_NS	110000000

#define DEVICE_NAME		"rpr_ps"
#define ALS_NAME		"rpr0521-ls"
#define PS_NAME 		"proximity"

/* POWER SUPPLY VOLTAGE RANGE */
#define RPR0521_VDD_MIN_UV	2000000
#define RPR0521_VDD_MAX_UV	3300000
#define RPR0521_VIO_MIN_UV	1750000
#define RPR0521_VIO_MAX_UV	1950000

#define RPR_FIR_LEN 16
#define MAX_FIR_LEN 32

#define RPR0521_MANUFACT_ID	0xE0
#define SIZE_ARRAY_PS_CALIBRATE 100

static struct sensors_classdev sensors_light_cdev = {
	.name = "rpr0521-light",
	.vendor = "Sensortek",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "6500",
	.resolution = "0.0625",
	.sensor_power = "0.09",
	.min_delay = (MIN_ALS_POLL_DELAY_NS / 1000),	/* us */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.flags =2,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "rpr0521-proximity",
	.vendor = "Sensortek",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "10.0",
	.resolution = "10.0",
	.sensor_power = "0.1",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.flags =3,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

struct data_filter {
	u16 raw[MAX_FIR_LEN];
	int sum;
	int number;
	int idx;
};

struct rpr0521_platform_data {
	uint8_t state_reg;
	uint8_t psctrl_reg;
	uint8_t alsctrl_reg;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	int int_pin;
	uint32_t transmittance;
	uint32_t int_flags;
	bool use_fir;
};

struct rpr0521_data {
	struct i2c_client *client;
	struct rpr0521_platform_data *pdata;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
	int		int_pin;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	struct mutex io_lock;
	struct input_dev *ps_input_dev;
	int32_t ps_distance_last;
	bool ps_enabled;
	struct wake_lock ps_wakelock;
	struct work_struct rpr_ps_work;
	struct workqueue_struct *rpr_ps_wq;
#ifdef RPR_POLL_PS
	struct wake_lock ps_nosuspend_wl;
#endif
	struct input_dev *als_input_dev;
	int32_t als_lux_last;
	uint32_t als_transmittance;
	bool als_enabled;
	struct hrtimer als_timer;
	struct hrtimer ps_timer;
	ktime_t als_poll_delay;
	ktime_t ps_poll_delay;
#ifdef RPR_POLL_ALS
    struct work_struct rpr_als_work;
	struct workqueue_struct *rpr_als_wq;
#endif
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;
	bool use_fir;
	struct data_filter      fir;
	atomic_t                firlength;
};

#if( !defined(CONFIG_RPR_PS_ALS_USE_CHANGE_THRESHOLD))
static uint32_t lux_threshold_table[] =
{
	3,
	10,
	40,
	65,
	145,
	300,
	550,
	930,
	1250,
	1700,
};

#define LUX_THD_TABLE_SIZE (sizeof(lux_threshold_table)/sizeof(uint32_t)+1)
static uint16_t code_threshold_table[LUX_THD_TABLE_SIZE+1];
#endif

static int32_t rpr0521_enable_ps(struct rpr0521_data *ps_data, uint8_t enable);
static int32_t rpr0521_enable_als(struct rpr0521_data *ps_data, uint8_t enable);
static int32_t rpr0521_set_ps_thd_l(struct rpr0521_data *ps_data, uint16_t thd_l);
static int32_t rpr0521_set_ps_thd_h(struct rpr0521_data *ps_data, uint16_t thd_h);
static int32_t rpr0521_set_als_thd_l(struct rpr0521_data *ps_data, uint16_t thd_l);
static int32_t rpr0521_set_als_thd_h(struct rpr0521_data *ps_data, uint16_t thd_h);
static int rpr0521_device_ctl(struct rpr0521_data *ps_data, bool enable);
//static int32_t rpr0521_set_ps_aoffset(struct rpr0521_data *ps_data, uint16_t offset);

inline uint32_t rpr_alscode2lux(struct rpr0521_data *ps_data, uint32_t alscode)
{
	alscode/=1000;
	alscode += ((alscode<<7)+(alscode<<3)+(alscode>>1));
    alscode<<=3;
    alscode/=ps_data->als_transmittance;
	return alscode;
}

inline uint32_t rpr_lux2alscode(struct rpr0521_data *ps_data, uint32_t lux)
{
    lux*=ps_data->als_transmittance;
    lux/=1100;
    if (unlikely(lux>=(1<<16)))
        lux = (1<<16) -1;
    return lux;
}

#ifndef CONFIG_RPR_PS_ALS_USE_CHANGE_THRESHOLD
static void rpr_init_code_threshold_table(struct rpr0521_data *ps_data)
{
    uint32_t i,j;
    uint32_t alscode;

    code_threshold_table[0] = 0;
#ifdef RPR_DEBUG_PRINTF
    printk(KERN_INFO "alscode[0]=%d\n",0);
#endif
    for (i=1,j=0;i<LUX_THD_TABLE_SIZE;i++,j++)
    {
        alscode = rpr_lux2alscode(ps_data, lux_threshold_table[j]);
		dev_dbg(&ps_data->client->dev, "alscode[%d]=%d\n", i, alscode);
        code_threshold_table[i] = (uint16_t)(alscode);
    }
    code_threshold_table[i] = 0xffff;
	dev_dbg(&ps_data->client->dev, "alscode[%d]=%d\n", i, alscode);
}

static uint32_t rpr_get_lux_interval_index(uint16_t alscode)
{
    uint32_t i;
    for (i=1;i<=LUX_THD_TABLE_SIZE;i++)
    {
        if ((alscode>=code_threshold_table[i-1])&&(alscode<code_threshold_table[i]))
        {
            return i;
        }
    }
    return LUX_THD_TABLE_SIZE;
}
#else
inline void rpr_als_set_new_thd(struct rpr0521_data *ps_data, uint16_t alscode)
{
    int32_t high_thd,low_thd;
    high_thd = alscode + rpr_lux2alscode(ps_data, RPR_ALS_CHANGE_THD);
    low_thd = alscode - rpr_lux2alscode(ps_data, RPR_ALS_CHANGE_THD);
    if (high_thd >= (1<<16))
        high_thd = (1<<16) -1;
    if (low_thd <0)
        low_thd = 0;
    rpr0521_set_als_thd_h(ps_data, (uint16_t)high_thd);
    rpr0521_set_als_thd_l(ps_data, (uint16_t)low_thd);
}
#endif // CONFIG_RPR_PS_ALS_USE_CHANGE_THRESHOLD


static int32_t rpr0521_init_all_reg(struct rpr0521_data *ps_data, struct rpr0521_platform_data *plat_data)
{
	int32_t ret;
	uint8_t w_reg;

	w_reg = plat_data->state_reg;
    ret = i2c_smbus_write_byte_data(ps_data->client, RPR_STATE_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

	ps_data->ps_thd_h = plat_data->ps_thd_h;
	ps_data->ps_thd_l = plat_data->ps_thd_l;

	w_reg = plat_data->psctrl_reg;
    ret = i2c_smbus_write_byte_data(ps_data->client, RPR_PSCTRL_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	w_reg = plat_data->alsctrl_reg;
    ret = i2c_smbus_write_byte_data(ps_data->client, RPR_ALSCTRL_REG, w_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

	rpr0521_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	rpr0521_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
	// ret = i2c_smbus_read_word_data(ps_data->client, RPR_THDL1_PS_REG);
	// w_reg = ((ret & 0xFF00) >> 8) | ((ret & 0x00FF) << 8) ;
	// rpr0521_set_ps_thd_l(ps_data, w_reg);

	w_reg = 0;
#ifndef RPR_POLL_PS
	w_reg |= RPR_INT_PS_MODE;
#else
	w_reg |= 0x01;
#endif

#if (!defined(RPR_POLL_ALS) && (RPR_INT_PS_MODE != 0x02) && (RPR_INT_PS_MODE != 0x03))
	w_reg |= RPR_INT_ALS;
#endif
    ret = i2c_smbus_write_byte_data(ps_data->client, RPR_INT_REG, w_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(ps_data->client, 0x87, 0x60);
	if (ret < 0) {
		dev_err(&ps_data->client->dev,
			"%s: write i2c error\n", __func__);
		return ret;
	}
	return 0;
}

static int32_t rpr0521_check_pid(struct rpr0521_data *ps_data)
{
	int32_t err1;

	err1 = i2c_smbus_read_byte_data(ps_data->client,RPR_PDT_ID_REG);
	if (err1 < 0)
	{
		printk(KERN_ERR "%s: read i2c error, err=%d\n", __func__, err1);
		return err1;
	}

	if(err1 == 0xE0)
		printk(KERN_INFO "%s: RPR0521_PID=%d\n", __func__, RPR0521_MANUFACT_ID);

	return 0;
}


static int32_t rpr0521_software_reset(struct rpr0521_data *ps_data)
{
    int32_t r;

    r = i2c_smbus_write_byte_data(ps_data->client,RPR_SYS_CTL_REG,0x7A);
    if (r<0)
    {
        printk(KERN_ERR "%s: software reset: read error after reset\n", __func__);
        return r;
    }
    msleep(1);
    return 0;
}


static int32_t rpr0521_set_als_thd_l(struct rpr0521_data *ps_data, uint16_t thd_l)
{
    uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_l;
    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    return i2c_smbus_write_word_data(ps_data->client,RPR_THDL1_ALS_REG,thd_l);
}
static int32_t rpr0521_set_als_thd_h(struct rpr0521_data *ps_data, uint16_t thd_h)
{
	uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_h;
    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    return i2c_smbus_write_word_data(ps_data->client,RPR_THDH1_ALS_REG,thd_h);
}

static int32_t rpr0521_set_ps_thd_l(struct rpr0521_data *ps_data, uint16_t thd_l)
{
    uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_l;

    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    ps_data->ps_thd_l = thd_l;
	return i2c_smbus_write_word_data(ps_data->client,RPR_THDL1_PS_REG,thd_l);
}

static int32_t rpr0521_set_ps_thd_h(struct rpr0521_data *ps_data, uint16_t thd_h)
{
    uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_h;

    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    ps_data->ps_thd_h = thd_h;
	return i2c_smbus_write_word_data(ps_data->client,RPR_THDH1_PS_REG,thd_h);
}

static inline uint32_t rpr0521_get_ps_reading(struct rpr0521_data *ps_data)
{
	int32_t word_data, tmp_word_data;

	tmp_word_data = i2c_smbus_read_word_data(ps_data->client,RPR_DATA1_PS_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;
	}
	// word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
	word_data = tmp_word_data;
	return word_data;
}

static int32_t rpr0521_enable_ps(struct rpr0521_data *ps_data, uint8_t enable)
{
    int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_ps_enable;
	curr_ps_enable = ps_data->ps_enabled?1:0;
	if(curr_ps_enable == enable)
		return 0;

	if (enable) {
		ret = rpr0521_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}

    ret = i2c_smbus_read_byte_data(ps_data->client, RPR_STATE_REG);
    if (ret < 0)
    {
			printk(KERN_ERR "%s: write i2c error, ret=%d\n", __func__, ret);
		return ret;
    }

	w_state_reg = ret;
	w_state_reg &= ~(RPR_STATE_EN_PS_MASK);
	if(enable)
	{
		w_state_reg |= RPR_STATE_EN_PS_MASK;
	}
    ret = i2c_smbus_write_byte_data(ps_data->client, RPR_STATE_REG, w_state_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error, ret=%d\n", __func__, ret);
		return ret;
	}

    if(enable)
	{
#ifdef RPR_POLL_PS
		hrtimer_start(&ps_data->ps_timer, ps_data->ps_poll_delay, HRTIMER_MODE_REL);
		ps_data->ps_distance_last = -1;
#endif
		ps_data->ps_enabled = true;
#ifndef RPR_POLL_PS
#ifndef RPR_POLL_ALS
		if(!(ps_data->als_enabled))
#endif	/* #ifndef RPR_POLL_ALS	*/
			enable_irq(ps_data->irq);
		msleep(1);
		ret = rpr0521_get_ps_reading(ps_data);

		printk(KERN_INFO "%s: RPR0521 uper threshold - %d", __func__, ps_data->ps_thd_h);
		printk(KERN_INFO "%s: RPR0521 lower threshold - %d", __func__, ps_data->ps_thd_l);
		printk(KERN_INFO "%s: RPR0521 distance - %d", __func__, ret);

		// near_far_state = ((ret < ps_data->ps_thd_h) && (ret > ps_data->ps_thd_l))?0:1;
		if ((ret < ps_data->ps_thd_h) && (ret > ps_data->ps_thd_l)) near_far_state = 0;
		else near_far_state = 1;
		ps_data->ps_distance_last = near_far_state;
		input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
		input_sync(ps_data->ps_input_dev);
		wake_lock_timeout(&ps_data->ps_wakelock, 3*HZ);
		reading = rpr0521_get_ps_reading(ps_data);
		dev_dbg(&ps_data->client->dev,
			"%s: ps input event=%d, ps code = %d\n",
			__func__, near_far_state, reading);
#endif	/* #ifndef RPR_POLL_PS */
	}
	else
	{
#ifdef RPR_POLL_PS
		hrtimer_cancel(&ps_data->ps_timer);
#else
#ifndef RPR_POLL_ALS
		if(!(ps_data->als_enabled))
#endif
			disable_irq(ps_data->irq);
#endif
		ps_data->ps_enabled = false;
	}
	if (!enable) {
		ret = rpr0521_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}

	return ret;
}

static int32_t rpr0521_enable_als(struct rpr0521_data *ps_data, uint8_t enable)
{
    int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_als_enable = (ps_data->als_enabled)?1:0;

	if(curr_als_enable == enable)
		return 0;

	if (enable) {
		ret = rpr0521_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}
#ifndef RPR_POLL_ALS
    if (enable)
	{
        rpr0521_set_als_thd_h(ps_data, 0x0000);
        rpr0521_set_als_thd_l(ps_data, 0xFFFF);
	}
#endif
    ret = i2c_smbus_read_byte_data(ps_data->client, RPR_STATE_REG);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
    }
	w_state_reg = (uint8_t)(ret & (~(RPR_STATE_EN_ALS_MASK)));
	if(enable)
		w_state_reg |= RPR_STATE_EN_ALS_MASK;

    ret = i2c_smbus_write_byte_data(ps_data->client, RPR_STATE_REG, w_state_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

    if (enable)
    {
		ps_data->als_enabled = true;
#ifdef RPR_POLL_ALS
		hrtimer_start(&ps_data->als_timer, ps_data->als_poll_delay, HRTIMER_MODE_REL);
#else
#ifndef RPR_POLL_PS
		if(!(ps_data->ps_enabled))
#endif
			enable_irq(ps_data->irq);
#endif
    }
	else
	{
		ps_data->als_enabled = false;
#ifdef RPR_POLL_ALS
		hrtimer_cancel(&ps_data->als_timer);
#else
#ifndef RPR_POLL_PS
		if(!(ps_data->ps_enabled))
#endif
			disable_irq(ps_data->irq);
#endif
	}
	if (!enable) {
		ret = rpr0521_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}

	printk(KERN_INFO "%s: enable rpr0521_ALS successfully\n", __func__);

    return ret;
}

static inline int32_t rpr0521_filter_reading(struct rpr0521_data *ps_data,
			int32_t word_data1, int32_t word_data2)
{
	int32_t word_data;
	word_data = 0;
	if (word_data1 <= 0) word_data = 0;
	else
	{
		int32_t data = 0;
		data = word_data2 * 1000 / word_data1;

		if (data < 595) word_data = word_data1 * 1682 - word_data2 * 1877;
		else if (data < 1015) word_data = word_data1 * 644 - word_data2 * 132;
		else if (data < 1352) word_data = word_data1 * 756 - word_data2 * 243;
		else if (data < 3053) word_data = word_data1 * 766 - word_data2 * 250;
	}
	return word_data;
}

static inline int32_t rpr0521_get_als_reading(struct rpr0521_data *ps_data)
{
    int32_t word_data1, word_data2, word_data, tmp_word_data1, tmp_word_data2;

	tmp_word_data1 = i2c_smbus_read_word_data(ps_data->client, RPR_DATA1_ALS_REG);
	if(tmp_word_data1 < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data1);
		return tmp_word_data1;
	}
	word_data1 = ((tmp_word_data1 & 0xFF00) >> 8) | ((tmp_word_data1 & 0x00FF) << 8);

	tmp_word_data2 = i2c_smbus_read_word_data(ps_data->client, RPR_DATA1_ALS_REG_1);
	if(tmp_word_data2 < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data2);
		return tmp_word_data2;
	}
	word_data2 = ((tmp_word_data2 & 0xFF00) >> 8) | ((tmp_word_data2 & 0x00FF) << 8) ;

	word_data = rpr0521_filter_reading(ps_data, word_data1, word_data2);

	return word_data;
}

static int32_t rpr0521_get_ir_reading(struct rpr0521_data *ps_data)
{
	if(ps_data->ps_enabled)
	{
		rpr0521_enable_ps(ps_data, 0);
		ps_data->ps_enabled = true;
	}

	msleep(100);

	if(ps_data->ps_enabled)
		rpr0521_enable_ps(ps_data, 1);
	return 1;
}


static ssize_t rpr_als_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
    int32_t reading;

    reading = rpr0521_get_als_reading(ps_data);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static int rpr_als_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct rpr0521_data *als_data = container_of(sensors_cdev,
						struct rpr0521_data, als_cdev);
	int err;

	mutex_lock(&als_data->io_lock);
	err = rpr0521_enable_als(als_data, enabled);
	mutex_unlock(&als_data->io_lock);

	if (err < 0)
		return err;
	return 0;
}

static ssize_t rpr_als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data = dev_get_drvdata(dev);
    int32_t enable, ret;

    mutex_lock(&ps_data->io_lock);
	enable = (ps_data->als_enabled)?1:0;
    mutex_unlock(&ps_data->io_lock);
    ret = i2c_smbus_read_byte_data(ps_data->client,RPR_STATE_REG);
    ret = (ret & RPR_STATE_EN_ALS_MASK)?1:0;

	if(enable != ret)
		printk(KERN_ERR "%s: driver and sensor mismatch! driver_enable=0x%x, sensor_enable=%x\n", __func__, enable, ret);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t rpr_als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct rpr0521_data *ps_data = dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: Enable ALS : %d\n", __func__, en);
    mutex_lock(&ps_data->io_lock);
    rpr0521_enable_als(ps_data, en);
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t rpr_als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data = dev_get_drvdata(dev);
    int32_t als_reading;
	uint32_t als_lux;
    als_reading = rpr0521_get_als_reading(ps_data);
	mutex_lock(&ps_data->io_lock);
	als_lux = rpr_alscode2lux(ps_data, als_reading);
	mutex_unlock(&ps_data->io_lock);
    return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_lux);
}

static ssize_t rpr_als_lux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 16, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
    mutex_lock(&ps_data->io_lock);
    ps_data->als_lux_last = value;
	input_report_abs(ps_data->als_input_dev, ABS_MISC, value);
	input_sync(ps_data->als_input_dev);
	mutex_unlock(&ps_data->io_lock);
	dev_dbg(dev, "%s: als input event %ld lux\n", __func__, value);

    return size;
}


static ssize_t rpr_als_transmittance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
    int32_t transmittance;
    mutex_lock(&ps_data->io_lock);
    transmittance = ps_data->als_transmittance;
    mutex_unlock(&ps_data->io_lock);
    return scnprintf(buf, PAGE_SIZE, "%d\n", transmittance);
}


static ssize_t rpr_als_transmittance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	mutex_lock(&ps_data->io_lock);
    ps_data->als_transmittance = value;
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t rpr_als_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			(u32)ktime_to_ms(ps_data->als_poll_delay));
}

static inline void rpr_als_delay_store_fir(struct rpr0521_data *ps_data)
{
	ps_data->fir.number = 0;
	ps_data->fir.idx = 0;
	ps_data->fir.sum = 0;
}

static int rpr_als_poll_delay_set(struct sensors_classdev *sensors_cdev,
						unsigned int delay_msec)
{
	struct rpr0521_data *als_data = container_of(sensors_cdev,
						struct rpr0521_data, als_cdev);
	uint64_t value = 0;

	value = delay_msec * 1000000;

	if (value < MIN_ALS_POLL_DELAY_NS)
		value = MIN_ALS_POLL_DELAY_NS;

	mutex_lock(&als_data->io_lock);
	if (value != ktime_to_ns(als_data->als_poll_delay))
		als_data->als_poll_delay = ns_to_ktime(value);

	if (als_data->use_fir)
		rpr_als_delay_store_fir(als_data);

	mutex_unlock(&als_data->io_lock);

	return 0;
}

static ssize_t rpr_als_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint64_t value = 0;
	int ret;
	struct rpr0521_data *als_data =  dev_get_drvdata(dev);
	ret = kstrtoull(buf, 10, &value);
	if(ret < 0)
	{
		dev_err(dev, "%s:kstrtoull failed, ret=0x%x\n",	__func__, ret);
		return ret;
	}
#ifdef RPR_DEBUG_PRINTF
	dev_dbg(dev, "%s: set als poll delay=%lld\n", __func__, value);
#endif
	ret = rpr_als_poll_delay_set(&als_data->als_cdev, value);
	if (ret < 0)
		return ret;
	return size;
}

static ssize_t rpr_als_ir_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
    int32_t reading;
    reading = rpr0521_get_ir_reading(ps_data);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static ssize_t rpr_als_firlen_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	int len = atomic_read(&ps_data->firlength);

	dev_dbg(dev, "%s: len = %2d, idx = %2d\n",
			__func__, len, ps_data->fir.idx);
	dev_dbg(dev, "%s: sum = %5d, ave = %5d\n",
			__func__, ps_data->fir.sum, ps_data->fir.sum/len);

	return scnprintf(buf, PAGE_SIZE, "%d\n", len);
}

static ssize_t rpr_als_firlen_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	uint64_t value = 0;
	int ret;
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	ret = kstrtoull(buf, 10, &value);
	if (ret < 0) {
		dev_err(dev, "%s:strict_strtoull failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}

	if (value > MAX_FIR_LEN) {
		dev_err(dev, "%s: firlen exceed maximum filter length\n",
			__func__);
	} else if (value < 1) {
		atomic_set(&ps_data->firlength, 1);
		memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));
	} else {
		atomic_set(&ps_data->firlength, value);
		memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));
	}
	return size;
}

static ssize_t rpr_als_fir_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->use_fir);
}

static ssize_t rpr_als_fir_enable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	uint64_t value = 0;
	int ret;
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	ret = kstrtoull(buf, 10, &value);
	if (ret < 0) {
		dev_err(dev, "%s:strict_strtoull failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}

	if (value) {
		ps_data->use_fir = true;
		memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));
	} else {
		ps_data->use_fir = false;
	}
	return size;
}
static ssize_t rpr_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data = dev_get_drvdata(dev);
    uint32_t reading;
    reading = rpr0521_get_ps_reading(ps_data);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static int rpr_ps_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct rpr0521_data *ps_data = container_of(sensors_cdev,
						struct rpr0521_data, ps_cdev);
	int err;

	mutex_lock(&ps_data->io_lock);
	err = rpr0521_enable_ps(ps_data, enabled);
	mutex_unlock(&ps_data->io_lock);

	if (err < 0)
		return err;
	return 0;
}

static unsigned int rpr_ps_calibration(struct rpr0521_data *ps_data)
{
	int array[SIZE_ARRAY_PS_CALIBRATE];
	int i = 0, max = 0,index = 0, j = 0;
	int frequency;

	printk(KERN_INFO "%s STARTING CALIBRATION ... (15 seconds)", __func__);

	for (j = 0; j < SIZE_ARRAY_PS_CALIBRATE; j++)
	{
		array[j] = rpr0521_get_ps_reading(ps_data);
		msleep(50);
	}

	while(i < SIZE_ARRAY_PS_CALIBRATE - 1)	{
 
		frequency = 1;
 
		while(array[i] == array[i+1])	{
 
			frequency++;
			i++;
		}
 
		if(max < frequency)	{
 
			max = frequency;
			index = i;
		}
 
		i++;
	}

	return array[index];
}


static ssize_t rpr_ps_calibrate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_l1_reg, ps_thd_l2_reg;
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
    mutex_lock(&ps_data->io_lock);
    ps_thd_l1_reg = i2c_smbus_read_byte_data(ps_data->client,RPR_THDL1_PS_REG);
    if(ps_thd_l1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l1_reg);
		return -EINVAL;
	}
    ps_thd_l2_reg = i2c_smbus_read_byte_data(ps_data->client,RPR_THDL2_PS_REG);
    if(ps_thd_l2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l2_reg);
		return -EINVAL;
	}
    mutex_unlock(&ps_data->io_lock);
	ps_thd_l1_reg = ps_thd_l1_reg<<8 | ps_thd_l2_reg;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_l1_reg);
}

static ssize_t rpr_ps_calibrate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	unsigned int value = 0;
	
	value = rpr_ps_calibration(ps_data);

    mutex_lock(&ps_data->io_lock);
    rpr0521_set_ps_thd_l(ps_data, value);
    mutex_unlock(&ps_data->io_lock);
	printk(KERN_INFO "%s FINISH CALIBRATION", __func__);
    return value;
}

static ssize_t rpr_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t enable, ret;
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);

    mutex_lock(&ps_data->io_lock);
	enable = (ps_data->ps_enabled)?1:0;
    mutex_unlock(&ps_data->io_lock);
    ret = i2c_smbus_read_byte_data(ps_data->client,RPR_STATE_REG);
    ret = (ret & RPR_STATE_EN_PS_MASK)?1:0;

	if(enable != ret)
		printk(KERN_ERR "%s: driver and sensor mismatch! driver_enable=0x%x, sensor_enable=%x\n", __func__, enable, ret);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t rpr_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: Enable PS : %d\n", __func__, en);
    mutex_lock(&ps_data->io_lock);
    rpr0521_enable_ps(ps_data, en);
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t rpr_ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
    int32_t word_data, tmp_word_data;

	tmp_word_data = i2c_smbus_read_word_data(ps_data->client, RPR_DATA1_OFFSET_REG);
	if(tmp_word_data < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, tmp_word_data);
		return tmp_word_data;
	}
		word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
	return scnprintf(buf, PAGE_SIZE, "%d\n", word_data);
}

static ssize_t rpr_ps_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	uint16_t offset;

	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	if(value > 65535)
	{
		printk(KERN_ERR "%s: invalid value, offset=%ld\n", __func__, value);
		return -EINVAL;
	}

	offset = (uint16_t) ((value&0x00FF) << 8) | ((value&0xFF00) >>8);
	ret = i2c_smbus_write_word_data(ps_data->client,RPR_DATA1_OFFSET_REG,offset);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	return size;
}


static ssize_t rpr_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
    int32_t dist=1, ret;

    mutex_lock(&ps_data->io_lock);
   	ret = rpr0521_get_ps_reading(ps_data);

	printk(KERN_INFO "%s: RPR0521 uper threshold - %d", __func__, ps_data->ps_thd_h);
	printk(KERN_INFO "%s: RPR0521 lower threshold - %d", __func__, ps_data->ps_thd_l);
	printk(KERN_INFO "%s: RPR0521 distance - %d", __func__, ret);

    // dist = ((ret < ps_data->ps_thd_h) && (ret > ps_data->ps_thd_l))?0:1;
	if ((ret < ps_data->ps_thd_h) && (ret > ps_data->ps_thd_l)) dist = 0;
	else dist = 1;

    ps_data->ps_distance_last = dist;
	input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, dist);
	input_sync(ps_data->ps_input_dev);
    mutex_unlock(&ps_data->io_lock);
	wake_lock_timeout(&ps_data->ps_wakelock, 3*HZ);
	dev_dbg(dev, "%s: ps input event %d cm\n", __func__, dist);
    return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}


static ssize_t rpr_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
    mutex_lock(&ps_data->io_lock);
    ps_data->ps_distance_last = value;
	input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, value);
	input_sync(ps_data->ps_input_dev);
    mutex_unlock(&ps_data->io_lock);
	wake_lock_timeout(&ps_data->ps_wakelock, 3*HZ);
	dev_dbg(dev, "%s: ps input event %ld cm\n", __func__, value);
    return size;
}


static ssize_t rpr_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_l1_reg, ps_thd_l2_reg;
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
    mutex_lock(&ps_data->io_lock);
    ps_thd_l1_reg = i2c_smbus_read_byte_data(ps_data->client,RPR_THDL1_PS_REG);
    if(ps_thd_l1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l1_reg);
		return -EINVAL;
	}
    ps_thd_l2_reg = i2c_smbus_read_byte_data(ps_data->client,RPR_THDL2_PS_REG);
    if(ps_thd_l2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l2_reg);
		return -EINVAL;
	}
    mutex_unlock(&ps_data->io_lock);
	ps_thd_l1_reg = ps_thd_l1_reg<<8 | ps_thd_l2_reg;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_l1_reg);
}


static ssize_t rpr_ps_code_thd_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
    mutex_lock(&ps_data->io_lock);
    rpr0521_set_ps_thd_l(ps_data, value);
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t rpr_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_h1_reg, ps_thd_h2_reg;
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
    mutex_lock(&ps_data->io_lock);
    ps_thd_h1_reg = i2c_smbus_read_byte_data(ps_data->client,RPR_THDH1_PS_REG);
    if(ps_thd_h1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h1_reg);
		return -EINVAL;
	}
    ps_thd_h2_reg = i2c_smbus_read_byte_data(ps_data->client,RPR_THDH2_PS_REG);
    if(ps_thd_h2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h2_reg);
		return -EINVAL;
	}
    mutex_unlock(&ps_data->io_lock);
	ps_thd_h1_reg = ps_thd_h1_reg<<8 | ps_thd_h2_reg;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_h1_reg);
}


static ssize_t rpr_ps_code_thd_h_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
    mutex_lock(&ps_data->io_lock);
    rpr0521_set_ps_thd_h(ps_data, value);
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t rpr_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_reg[22];
	uint8_t cnt;
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);
    mutex_lock(&ps_data->io_lock);
	for(cnt=0;cnt<21;cnt++)
	{
		ps_reg[cnt] = i2c_smbus_read_byte_data(ps_data->client, (cnt + 64));
		if(ps_reg[cnt] < 0)
		{
			mutex_unlock(&ps_data->io_lock);
			printk(KERN_ERR "rpr_all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);
			return -EINVAL;
		}
		else
		{
			dev_dbg(dev, "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = i2c_smbus_read_byte_data(ps_data->client, RPR_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		mutex_unlock(&ps_data->io_lock);
		printk( KERN_ERR "all_reg_show:i2c_smbus_read_byte_data fail, ret=%d", ps_reg[cnt]);
		return -EINVAL;
	}
	dev_dbg(dev, "reg[0x%x]=0x%2X\n", RPR_PDT_ID_REG, ps_reg[cnt]);

    mutex_unlock(&ps_data->io_lock);

    return scnprintf(buf, PAGE_SIZE, "%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X %2X %2X %2X,%2X %2X\n",
		ps_reg[0], ps_reg[1], ps_reg[2], ps_reg[3], ps_reg[4], ps_reg[5], ps_reg[6], ps_reg[7], ps_reg[8],
		ps_reg[9], ps_reg[10], ps_reg[11], ps_reg[12], ps_reg[13], ps_reg[14], ps_reg[15], ps_reg[16], ps_reg[17],
		ps_reg[18], ps_reg[19], ps_reg[20], ps_reg[21]);
}

static ssize_t rpr_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t rpr_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value = 0;
	int ret;
	int32_t recv_data;
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);

	ret = kstrtoul(buf, 16, &value);
	if (ret < 0) {
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	recv_data = i2c_smbus_read_byte_data(ps_data->client,value);
	printk("%s: reg 0x%x=0x%x\n", __func__, (int)value, recv_data);
	return size;
}


static ssize_t rpr_send_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t rpr_send_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr, cmd;
	u8 addr_u8, cmd_u8;
	int32_t ret, i;
	char *token[10];
	struct rpr0521_data *ps_data =  dev_get_drvdata(dev);

	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");
	ret = kstrtoul(token[0], 16, (unsigned long *)&(addr));
	if (ret < 0) {

		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	ret = kstrtoul(token[1], 16, (unsigned long *)&(cmd));
	if (ret < 0) {
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	dev_dbg(dev, "%s: write reg 0x%x=0x%x\n", __func__, addr, cmd);
	addr_u8 = (u8) addr;
	cmd_u8 = (u8) cmd;
	//mutex_lock(&ps_data->io_lock);
	ret = i2c_smbus_write_byte_data(ps_data->client,addr_u8,cmd_u8);
	//mutex_unlock(&ps_data->io_lock);
	if (0 != ret)
	{
		printk(KERN_ERR "%s: i2c_smbus_write_byte_data fail\n", __func__);
		return ret;
	}

	return size;
}

static struct device_attribute als_enable_attribute = __ATTR(enable,0664,rpr_als_enable_show,rpr_als_enable_store);	// DONE
static struct device_attribute als_lux_attribute = __ATTR(lux,0664,rpr_als_lux_show,rpr_als_lux_store);
static struct device_attribute als_code_attribute = __ATTR(code, 0444, rpr_als_code_show, NULL);
static struct device_attribute als_transmittance_attribute = __ATTR(transmittance,0664,rpr_als_transmittance_show,rpr_als_transmittance_store); // DONE
static struct device_attribute als_poll_delay_attribute = __ATTR(poll_delay, 0664, rpr_als_delay_show, rpr_als_delay_store);	// DONE
static struct device_attribute als_ir_code_attribute = __ATTR(ircode,0444,rpr_als_ir_code_show,NULL);	// DONE
static struct device_attribute als_firlen_attribute = __ATTR(firlen, 0664, rpr_als_firlen_show, rpr_als_firlen_store);	// DONE
static struct device_attribute als_fir_enable_attribute = __ATTR(fir_enable, 0664, rpr_als_fir_enable_show, rpr_als_fir_enable_store);	// DONE

static struct attribute *rpr_als_attrs [] =
{
	&als_enable_attribute.attr,
    &als_lux_attribute.attr,
    &als_code_attribute.attr,
    &als_transmittance_attribute.attr,
	&als_poll_delay_attribute.attr,
	&als_ir_code_attribute.attr,
	&als_firlen_attribute.attr,
	&als_fir_enable_attribute.attr,
    NULL
};

static struct attribute_group rpr_als_attribute_group = {
	.attrs = rpr_als_attrs,
};


static struct device_attribute ps_calibrate_attribute = __ATTR(calibrate,0664,rpr_ps_calibrate_show,rpr_ps_calibrate_store);
static struct device_attribute ps_enable_attribute = __ATTR(enable,0664,rpr_ps_enable_show,rpr_ps_enable_store);
static struct device_attribute ps_distance_attribute = __ATTR(distance,0664,rpr_ps_distance_show, rpr_ps_distance_store);
static struct device_attribute ps_offset_attribute = __ATTR(offset,0664,rpr_ps_offset_show, rpr_ps_offset_store);
static struct device_attribute ps_code_attribute = __ATTR(code, 0444, rpr_ps_code_show, NULL);
static struct device_attribute ps_code_thd_l_attribute = __ATTR(codethdl,0664,rpr_ps_code_thd_l_show,rpr_ps_code_thd_l_store);
static struct device_attribute ps_code_thd_h_attribute = __ATTR(codethdh,0664,rpr_ps_code_thd_h_show,rpr_ps_code_thd_h_store);
static struct device_attribute recv_attribute = __ATTR(recv,0664,rpr_recv_show,rpr_recv_store);
static struct device_attribute send_attribute = __ATTR(send,0664,rpr_send_show, rpr_send_store);
static struct device_attribute all_reg_attribute = __ATTR(allreg, 0444, rpr_all_reg_show, NULL);

static struct attribute *rpr_ps_attrs [] =
{
	&ps_calibrate_attribute.attr,
    &ps_enable_attribute.attr,
    &ps_distance_attribute.attr,
	&ps_offset_attribute.attr,
    &ps_code_attribute.attr,
	&ps_code_thd_l_attribute.attr,
	&ps_code_thd_h_attribute.attr,
	&recv_attribute.attr,
	&send_attribute.attr,
	&all_reg_attribute.attr,
    NULL
};

static struct attribute_group rpr_ps_attribute_group = {
	.attrs = rpr_ps_attrs,
};

#ifdef RPR_POLL_ALS
static enum hrtimer_restart rpr_als_timer_func(struct hrtimer *timer)
{
	struct rpr0521_data *ps_data = container_of(timer, struct rpr0521_data, als_timer);
	queue_work(ps_data->rpr_als_wq, &ps_data->rpr_als_work);
	hrtimer_forward_now(&ps_data->als_timer, ps_data->als_poll_delay);
	return HRTIMER_RESTART;
}

static void rpr_als_work_func(struct work_struct *work)
{
	struct rpr0521_data *ps_data = container_of(work, struct rpr0521_data, rpr_als_work);
	int32_t reading;

    mutex_lock(&ps_data->io_lock);
	reading = rpr0521_get_als_reading(ps_data);
	if(reading < 0)
		return;
	ps_data->als_lux_last = rpr_alscode2lux(ps_data, reading);
	input_report_abs(ps_data->als_input_dev, ABS_MISC, ps_data->als_lux_last);
	input_sync(ps_data->als_input_dev);
	mutex_unlock(&ps_data->io_lock);
}
#endif

static enum hrtimer_restart rpr_ps_timer_func(struct hrtimer *timer)
{
	struct rpr0521_data *ps_data = container_of(timer, struct rpr0521_data, ps_timer);
	queue_work(ps_data->rpr_ps_wq, &ps_data->rpr_ps_work);
#ifdef RPR_POLL_PS
	hrtimer_forward_now(&ps_data->ps_timer, ps_data->ps_poll_delay);
	return HRTIMER_RESTART;
#else
	hrtimer_cancel(&ps_data->ps_timer);
	return HRTIMER_NORESTART;
#endif
}

static void rpr_ps_work_func(struct work_struct *work)
{
	struct rpr0521_data *ps_data = container_of(work, struct rpr0521_data, rpr_ps_work);
	uint32_t reading;
	int32_t near_far_state;
    mutex_lock(&ps_data->io_lock);

	reading = rpr0521_get_ps_reading(ps_data);
	printk(KERN_INFO "%s: RPR0521 uper threshold - %d", __func__, ps_data->ps_thd_h);
	printk(KERN_INFO "%s: RPR0521 lower threshold - %d", __func__, ps_data->ps_thd_l);
	printk(KERN_INFO "%s: RPR0521 distance - %d", __func__, reading);

	// near_far_state = ((reading<ps_data->ps_thd_h) && (reading > ps_data->ps_thd_l))?0:1;

	if ((reading < ps_data->ps_thd_h) && (reading > ps_data->ps_thd_l)) near_far_state = 0;
	else near_far_state = 1;

	if(ps_data->ps_distance_last != near_far_state)
	{
		ps_data->ps_distance_last = near_far_state;
		input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, reading /* near_far_state */);
		input_sync(ps_data->ps_input_dev);
		wake_lock_timeout(&ps_data->ps_wakelock, 3*HZ);
#ifdef RPR_DEBUG_PRINTF
		printk(KERN_INFO "%s: ps input event %d cm, ps code = %d\n",__func__, near_far_state, reading);
#endif
	}

	mutex_unlock(&ps_data->io_lock);
	return;
}

static inline void rpr0521_init_fir(struct rpr0521_data *ps_data)
{
	memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));
	atomic_set(&ps_data->firlength, RPR_FIR_LEN);
}

static int32_t rpr0521_init_all_setting(struct i2c_client *client, struct rpr0521_platform_data *plat_data)
{
	int32_t ret;
	struct rpr0521_data *ps_data = i2c_get_clientdata(client);

	ret = rpr0521_software_reset(ps_data);
	if(ret < 0)
		return ret;

	ret = rpr0521_check_pid(ps_data);
	if(ret < 0)
		return ret;

	ret = rpr0521_init_all_reg(ps_data, plat_data);
	if(ret < 0)
		return ret;
#ifndef CONFIG_RPR_PS_ALS_USE_CHANGE_THRESHOLD
	rpr_init_code_threshold_table(ps_data);
#endif

	if (plat_data->use_fir)
		rpr0521_init_fir(ps_data);

    return 0;
}

static int rpr0521_power_ctl(struct rpr0521_data *data, bool on)
{
	int ret = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			ret = regulator_enable(data->vdd);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator vdd enable failed ret=%d\n",
					ret);
			}
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "rpr0521_power_ctl on=%d\n",
				on);
	} else if (on && !data->power_enabled) {

		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "rpr0521_power_ctl on=%d\n",
				on);
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int rpr0521_power_init(struct rpr0521_data *data, bool on)
{
	int ret;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd,
					0, RPR0521_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio,
					0, RPR0521_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			ret = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			ret = regulator_set_voltage(data->vdd,
					RPR0521_VDD_MIN_UV,
					RPR0521_VDD_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			ret = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			ret = regulator_set_voltage(data->vio,
					RPR0521_VIO_MIN_UV,
					RPR0521_VIO_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, RPR0521_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int rpr0521_device_ctl(struct rpr0521_data *ps_data, bool enable)
{
	int ret;
	struct device *dev = &ps_data->client->dev;

	if (enable && !ps_data->power_enabled) {
		ret = rpr0521_power_ctl(ps_data, true);
		if (ret) {
			dev_err(dev, "Failed to enable device power\n");
			goto err_exit;
		}
		ret = rpr0521_init_all_setting(ps_data->client, ps_data->pdata);
		if (ret < 0) {
			rpr0521_power_ctl(ps_data, false);
			dev_err(dev, "Failed to re-init device setting\n");
			goto err_exit;
		}
	} else if (!enable && ps_data->power_enabled) {
		if (!ps_data->als_enabled && !ps_data->ps_enabled) {
			ret = rpr0521_power_ctl(ps_data, false);
			if (ret) {
				dev_err(dev, "Failed to disable device power\n");
				goto err_exit;
			}
		} else {
			dev_dbg(dev, "device control: als_enabled=%d, ps_enabled=%d\n",
				ps_data->als_enabled, ps_data->ps_enabled);
		}
	} else {
		dev_dbg(dev, "device control: enable=%d, power_enabled=%d\n",
			enable, ps_data->power_enabled);
	}
	return 0;

err_exit:
	return ret;
}
#ifdef CONFIG_OF
static int rpr0521_parse_dt(struct device *dev,
			struct rpr0521_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	pdata->int_pin = of_get_named_gpio_flags(np, "rpr,irq-gpio",
				0, &pdata->int_flags);
	if (pdata->int_pin < 0) {
		dev_err(dev, "Unable to read irq-gpio\n");
		return pdata->int_pin;
	}

	rc = of_property_read_u32(np, "rpr,transmittance", &temp_val);
	if (!rc)
		pdata->transmittance = temp_val;
	else {
		dev_err(dev, "Unable to read transmittance\n");
		return rc;
	}

	rc = of_property_read_u32(np, "rpr,state-reg", &temp_val);
	if (!rc)
		pdata->state_reg = temp_val;
	else {
		dev_err(dev, "Unable to read state-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "rpr,psctrl-reg", &temp_val);
	if (!rc)
		pdata->psctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read psctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "rpr,alsctrl-reg", &temp_val);
	if (!rc)
		pdata->alsctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read alsctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "rpr,ps-thdh", &temp_val);
	if (!rc)
		pdata->ps_thd_h = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}

	rc = of_property_read_u32(np, "rpr,ps-thdl", &temp_val);
	if (!rc)
		pdata->ps_thd_l = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}

	pdata->use_fir = of_property_read_bool(np, "rpr,use-fir");

	return 0;
}
#else
static int rpr0521_parse_dt(struct device *dev,
			struct rpr0521_platform_data *pdata)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */

static int rpr0521_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    int err = -ENODEV;
    struct rpr0521_data *ps_data;
	struct rpr0521_platform_data *plat_data;
    printk(KERN_INFO "%s: driver version = %s\n", __func__, DRIVER_VERSION);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    {
        printk(KERN_ERR "%s: No Support for I2C_FUNC_SMBUS_BYTE_DATA\n", __func__);
        return -ENODEV;
    }
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
    {
        printk(KERN_ERR "%s: No Support for I2C_FUNC_SMBUS_WORD_DATA\n", __func__);
        return -ENODEV;
    }

	ps_data = kzalloc(sizeof(struct rpr0521_data),GFP_KERNEL);
	if(!ps_data)
	{
		printk(KERN_ERR "%s: failed to allocate rpr0521_data\n", __func__);
		return -ENOMEM;
	}
	ps_data->client = client;
	i2c_set_clientdata(client,ps_data);
	mutex_init(&ps_data->io_lock);
	wake_lock_init(&ps_data->ps_wakelock,WAKE_LOCK_SUSPEND, "rpr_input_wakelock");

#ifdef RPR_POLL_PS
	wake_lock_init(&ps_data->ps_nosuspend_wl,WAKE_LOCK_SUSPEND, "rpr_nosuspend_wakelock");
#endif
	if (client->dev.of_node) {
		plat_data = devm_kzalloc(&client->dev,
			sizeof(struct rpr0521_platform_data), GFP_KERNEL);
		if (!plat_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = rpr0521_parse_dt(&client->dev, plat_data);
		dev_err(&client->dev,
			"%s: rpr0521_parse_dt ret=%d\n", __func__, err);
		if (err)
			return err;
	} else
		plat_data = client->dev.platform_data;

	if (!plat_data) {
		dev_err(&client->dev,
			"%s: no rpr0521 platform data!\n", __func__);
		goto err_als_input_allocate;
	}
	ps_data->als_transmittance = plat_data->transmittance;
	ps_data->int_pin = plat_data->int_pin;
	ps_data->use_fir = plat_data->use_fir;
	ps_data->pdata = plat_data;

	if (ps_data->als_transmittance == 0) {
		dev_err(&client->dev,
			"%s: Please set als_transmittance\n", __func__);
		goto err_als_input_allocate;
	}

	ps_data->als_input_dev = devm_input_allocate_device(&client->dev);
	if (ps_data->als_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate als device\n", __func__);
		err = -ENOMEM;
		goto err_als_input_allocate;
	}
	ps_data->ps_input_dev = devm_input_allocate_device(&client->dev);
	if (ps_data->ps_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate ps device\n", __func__);
		err = -ENOMEM;
		goto err_als_input_allocate;
	}
	ps_data->als_input_dev->name = ALS_NAME;
	ps_data->ps_input_dev->name = PS_NAME;
	set_bit(EV_ABS, ps_data->als_input_dev->evbit);
	set_bit(EV_ABS, ps_data->ps_input_dev->evbit);
	input_set_abs_params(ps_data->als_input_dev, ABS_MISC, 0, rpr_alscode2lux(ps_data, (1<<16)-1), 0, 0);
	input_set_abs_params(ps_data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);
	err = input_register_device(ps_data->als_input_dev);
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register als input device\n", __func__);
		goto err_als_input_allocate;
	}
	err = input_register_device(ps_data->ps_input_dev);
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register ps input device\n", __func__);
		goto err_als_input_allocate;
	}

	err = sysfs_create_group(&ps_data->als_input_dev->dev.kobj, &rpr_als_attribute_group);
	if (err < 0)
	{
		printk(KERN_ERR "%s:could not create sysfs group for als\n", __func__);
		goto err_als_input_allocate;
	}
	err = sysfs_create_group(&ps_data->ps_input_dev->dev.kobj, &rpr_ps_attribute_group);
	if (err < 0)
	{
		printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
		goto err_ps_sysfs_create_group;
	}
	input_set_drvdata(ps_data->als_input_dev, ps_data);
	input_set_drvdata(ps_data->ps_input_dev, ps_data);

#ifdef RPR_POLL_ALS
	ps_data->rpr_als_wq = create_singlethread_workqueue("rpr_als_wq");
	INIT_WORK(&ps_data->rpr_als_work, rpr_als_work_func);
	hrtimer_init(&ps_data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	ps_data->als_timer.function = rpr_als_timer_func;
#endif

	ps_data->rpr_ps_wq = create_singlethread_workqueue("rpr_ps_wq");
	INIT_WORK(&ps_data->rpr_ps_work, rpr_ps_work_func);
	hrtimer_init(&ps_data->ps_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->ps_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	ps_data->ps_timer.function = rpr_ps_timer_func;

	err = rpr0521_power_init(ps_data, true);
	if (err)
		goto err_power_init;

	err = rpr0521_power_ctl(ps_data, true);
	if (err)
		goto err_power_on;

	ps_data->als_enabled = false;
	ps_data->ps_enabled = false;

	/* make sure everything is ok before registering the class device */
	ps_data->als_cdev = sensors_light_cdev;
	ps_data->als_cdev.sensors_enable = rpr_als_enable_set;
	ps_data->als_cdev.sensors_poll_delay = rpr_als_poll_delay_set;
	err = sensors_classdev_register(&ps_data->als_input_dev->dev,
			&ps_data->als_cdev);
	if (err)
		goto err_power_on;

	ps_data->ps_cdev = sensors_proximity_cdev;
	ps_data->ps_cdev.sensors_enable = rpr_ps_enable_set;
	err = sensors_classdev_register(&ps_data->ps_input_dev->dev,
			&ps_data->ps_cdev);
	if (err)
		goto err_class_sysfs;

	/* enable device power only when it is enabled */
	err = rpr0521_power_ctl(ps_data, false);
	if (err)
		goto err_init_all_setting;

	dev_dbg(&client->dev, "%s: probe successfully", __func__);
	printk(KERN_INFO "%s: Driver RPR0521 probe successfully\n", __func__);
	// sensors_classdev_unregister(&ps_data->als_cdev);
	return 0;

err_init_all_setting:
	rpr0521_power_ctl(ps_data, false);
	sensors_classdev_unregister(&ps_data->ps_cdev);
err_class_sysfs:
	sensors_classdev_unregister(&ps_data->als_cdev);
err_power_on:
	rpr0521_power_init(ps_data, false);
err_power_init:
#ifndef RPR_POLL_PS
	free_irq(ps_data->irq, ps_data);
	gpio_free(plat_data->int_pin);
#endif
#ifdef RPR_POLL_ALS
	hrtimer_try_to_cancel(&ps_data->als_timer);
	destroy_workqueue(ps_data->rpr_als_wq);
#endif
	destroy_workqueue(ps_data->rpr_ps_wq);
	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &rpr_ps_attribute_group);
err_ps_sysfs_create_group:
	sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &rpr_als_attribute_group);
err_als_input_allocate:
#ifdef RPR_POLL_PS
    wake_lock_destroy(&ps_data->ps_nosuspend_wl);
#endif
    wake_lock_destroy(&ps_data->ps_wakelock);
    mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);
    return err;
}


static int rpr0521_remove(struct i2c_client *client)
{
	struct rpr0521_data *ps_data = i2c_get_clientdata(client);
#ifndef RPR_POLL_PS
	free_irq(ps_data->irq, ps_data);
	gpio_free(ps_data->int_pin);
#endif
#ifdef RPR_POLL_ALS
	hrtimer_try_to_cancel(&ps_data->als_timer);
	destroy_workqueue(ps_data->rpr_als_wq);
#endif
	destroy_workqueue(ps_data->rpr_ps_wq);
	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &rpr_ps_attribute_group);
	sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &rpr_als_attribute_group);
#ifdef RPR_POLL_PS
	wake_lock_destroy(&ps_data->ps_nosuspend_wl);
#endif
	wake_lock_destroy(&ps_data->ps_wakelock);
    mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);

    return 0;
}

static const struct i2c_device_id rpr_ps_id[] =
{
    { "rpr_ps", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, rpr_ps_id);

static struct of_device_id rpr_match_table[] = {
	{ .compatible = "yellowfin3,rpr0521", },
	{ },
};

static struct i2c_driver rpr_ps_driver =
{
    .driver = {
        .name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = rpr_match_table,
    },
    .probe = rpr0521_probe,
    .remove = rpr0521_remove,
    .id_table = rpr_ps_id,
};


static int __init rpr0521_init(void)
{
	int ret;
    ret = i2c_add_driver(&rpr_ps_driver);
    if (ret)
        return ret;

    return 0;
}

static void __exit rpr0521_exit(void)
{
    i2c_del_driver(&rpr_ps_driver);
}

module_init(rpr0521_init);
module_exit(rpr0521_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sitronix.com.tw>");
MODULE_DESCRIPTION("Sensortek rpr0521 Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

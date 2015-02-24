/*
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define DEBUG
#define _SP_TX_DRV_C_

#include "slimport_custom_declare.h"
#include <linux/slimport.h>
#include <linux/msm_hdmi.h>

#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#ifndef SLIMPORT_METRICS_STR_LEN
#define SLIMPORT_METRICS_STR_LEN 128
#endif
#endif


#ifdef CEC_ENABLE
enum CEC_CTRL_STAS cec_states;
#endif

#define SLIMPORT_DRV_DEBUG

#ifndef XTAL_CLK_DEF
#define XTAL_CLK_DEF XTAL_27M
#endif

#define XTAL_CLK_M10 pXTAL_data[XTAL_CLK_DEF].xtal_clk_m10
#define XTAL_CLK pXTAL_data[XTAL_CLK_DEF].xtal_clk

#define SP_REPEATER_MODE 0x40
#define HDCP_SYNC_LOST 0x04
#define UPSTREAM_HDMI_HDCP	0x10
#define DS_VGA_SINK_COUNT 0x01
#define ANX7732_ID_H 0x77
#define ANX7732_ID_L 0x32

static unchar sp_tx_test_bw;
static bool sp_tx_test_lt;
static bool sp_tx_test_edid;
static bool ds_auth_done;
static bool auth_done;
static bool ds_hpd_lost;
static bool audio_info_process;

static unsigned char g_changed_bandwidth;
static unsigned char g_hdmi_dvi_status;

static unsigned char g_need_clean_status;

static bool hdcp_repeater_passed;

static unchar edid_reading_times;
static bool edid_header_good;
static bool edid_checksum_good;

#ifdef ENABLE_READ_EDID
unsigned char g_edid_break;
unsigned char g_edid_checksum;
static unsigned char g_read_edid_flag;
#endif

static struct Packet_AVI sp_tx_packet_avi;
static struct Packet_SPD sp_tx_packet_spd;
static struct Packet_MPEG sp_tx_packet_mpeg;
static struct AudiInfoframe sp_tx_audioinfoframe;

enum SP_TX_System_State sp_tx_system_state;
enum RX_CBL_TYPE sp_tx_rx_type;

enum CHARGING_STATUS downstream_charging_status;
enum AUDIO_OUTPUT_STATUS sp_tx_ao_state;
enum VIDEO_OUTPUT_STATUS sp_tx_vo_state;
enum SINK_CONNECTION_STATUS sp_tx_sc_state;
enum SP_TX_LT_STATUS sp_tx_LT_state;
enum SP_TX_System_State sp_tx_system_state_bak;
enum HDCP_STATUS HDCP_state;

struct completion hdmi_edid_read_done;
extern int sp_set_ds_charge_type(enum CHARGING_STATUS);

static void sp_tx_variable_init(void);
static void sp_system_reset(void);

uint code chipid_list[CO3_NUMS] = {
	0x7818,
	0x7816,
	0x7812,
	0x7810,
	0x7806,
	0x7802
};

struct COMMON_INT common_int_status;
static int counter_unstable_video;
struct HDMI_RX_INT hdmi_rx_int_status;
static bool sp_repeater_mode;

#define COMMON_INT1 (common_int_status.common_int[0])
#define COMMON_INT2 (common_int_status.common_int[1])
#define COMMON_INT3 (common_int_status.common_int[2])
#define COMMON_INT4 (common_int_status.common_int[3])
#define COMMON_INT5 (common_int_status.common_int[4])
#define COMMON_INT_CHANGED (common_int_status.change_flag)
#define HDMI_RX_INT1 (hdmi_rx_int_status.hdmi_rx_int[0])
#define HDMI_RX_INT2 (hdmi_rx_int_status.hdmi_rx_int[1])
#define HDMI_RX_INT3 (hdmi_rx_int_status.hdmi_rx_int[2])
#define HDMI_RX_INT4 (hdmi_rx_int_status.hdmi_rx_int[3])
#define HDMI_RX_INT5 (hdmi_rx_int_status.hdmi_rx_int[4])
#define HDMI_RX_INT6 (hdmi_rx_int_status.hdmi_rx_int[5])
#define HDMI_RX_INT7 (hdmi_rx_int_status.hdmi_rx_int[6])
#define HDMI_RX_INT_CHANGED (hdmi_rx_int_status.change_flag)
static unchar down_sample_en;

#define sp_tx_aux_polling_enable() sp_write_reg_or(TX_P0, TX_DEBUG1, POLLING_EN)
#define sp_tx_aux_polling_disable() sp_write_reg_and(TX_P0, TX_DEBUG1, ~POLLING_EN)

#define reg_bit_ctl(addr, offset, data, enable) \
	do { \
		unchar c; \
		sp_read_reg(addr, offset, &c); \
		if (enable) { \
			if ((c & data) != data) { \
				c |= data; \
				sp_write_reg(addr, offset, c); \
			} \
		} else { \
			if ((c & data) == data) { \
				c &= ~data; \
				sp_write_reg(addr, offset, c); \
			} \
		} \
	} while (0)

#define sp_tx_video_mute(enable) \
	reg_bit_ctl(TX_P2, VID_CTRL1, VIDEO_MUTE, enable)
#define hdmi_rx_mute_audio(enable) \
	reg_bit_ctl(RX_P0, RX_MUTE_CTRL, AUD_MUTE, enable)
#define hdmi_rx_mute_video(enable) \
	reg_bit_ctl(RX_P0, RX_MUTE_CTRL, VID_MUTE, enable)
#define sp_tx_addronly_set(enable) \
	reg_bit_ctl(TX_P0, AUX_CTRL2, ADDR_ONLY_BIT, enable)

#define sp_tx_set_link_bw(bw) \
	sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, bw);
#define sp_tx_get_link_bw() \
	__i2c_read_byte(TX_P0, SP_TX_LINK_BW_SET_REG)

#define sp_tx_get_pll_lock_status() \
	((__i2c_read_byte(TX_P0, TX_DEBUG1) & DEBUG_PLL_LOCK) != 0 ? 1 : 0)

#define gen_M_clk_with_downspeading() \
	sp_write_reg_or(TX_P0, SP_TX_M_CALCU_CTRL, M_GEN_CLK_SEL)
#define gen_M_clk_without_downspeading \
	sp_write_reg_and(TX_P0, SP_TX_M_CALCU_CTRL, (~M_GEN_CLK_SEL))

#define hdmi_rx_set_hpd(enable) do { \
	if ((bool)enable) \
		sp_write_reg_or(TX_P2, SP_TX_VID_CTRL3_REG, HPD_OUT); \
	else \
		sp_write_reg_and(TX_P2, SP_TX_VID_CTRL3_REG, ~HPD_OUT); \
	} while (0)

#define hdmi_rx_set_termination(enable) do { \
	if ((bool)enable) \
		sp_write_reg_and(RX_P0, HDMI_RX_TMDS_CTRL_REG7, ~TERM_PD); \
	else \
		sp_write_reg_or(RX_P0, HDMI_RX_TMDS_CTRL_REG7, TERM_PD); \
	} while (0)

#define sp_tx_get_rx_bw(pdata) \
	sp_tx_aux_dpcdread_bytes(0x00, 0x00, DPCD_MAX_LINK_RATE, 1, pdata)

#define sp_tx_clean_hdcp_status() do { \
	sp_write_reg(TX_P0, TX_HDCP_CTRL0, 0x03); \
	sp_write_reg_or(TX_P0, TX_HDCP_CTRL0, RE_AUTH); \
	mdelay(2); \
	pr_info("%s %s : sp_tx_clean_hdcp_status\n", LOG_TAG, __func__); \
	} while (0)
#define reg_hardware_reset() do { \
	sp_write_reg_or(TX_P2, SP_TX_RST_CTRL_REG, HW_RST); \
	sp_tx_clean_state_machine(); \
	sp_tx_set_sys_state(STATE_SP_INITIALIZED); \
	mdelay(500); \
	} while (0)

#define write_dpcd_addr(addrh, addrm, addrl) \
	do { \
		unchar temp; \
		if (__i2c_read_byte(TX_P0, AUX_ADDR_7_0) != (unchar)addrl) \
			sp_write_reg(TX_P0, AUX_ADDR_7_0, (unchar)addrl); \
				if (__i2c_read_byte(TX_P0, AUX_ADDR_15_8) != (unchar)addrm) \
					sp_write_reg(TX_P0, AUX_ADDR_15_8, (unchar)addrm); \
		sp_read_reg(TX_P0, AUX_ADDR_19_16, &temp); \
		if ((unchar)(temp & 0x0F)  != ((unchar)addrh & 0x0F)) \
			sp_write_reg(TX_P0, AUX_ADDR_19_16, (temp  & 0xF0) | ((unchar)addrh)); \
	} while (0)

#define sp_tx_set_sys_state(ss) \
	do { \
		pr_info("%s %s : set: clean_status: %x,\n ", LOG_TAG, __func__, (uint)g_need_clean_status); \
		if ((sp_tx_system_state >= STATE_LINK_TRAINING) && (ss < STATE_LINK_TRAINING)) \
			sp_write_reg_or(TX_P0, SP_TX_ANALOG_PD_REG, CH0_PD); \
		sp_tx_system_state_bak = sp_tx_system_state; \
		sp_tx_system_state = (unchar)ss; \
		g_need_clean_status = 1; \
		print_sys_state(sp_tx_system_state); \
	} while (0)

#define goto_next_system_state() \
	do { \
		pr_info("%s %s : next: clean_status: %x,\n ", LOG_TAG, __func__, (uint)g_need_clean_status); \
		sp_tx_system_state_bak = sp_tx_system_state; \
		sp_tx_system_state++;\
		print_sys_state(sp_tx_system_state); \
	} while (0)

#define redo_cur_system_state() \
	do { \
		pr_info("%s %s : redo: clean_status: %x,\n ", LOG_TAG, __func__, (uint)g_need_clean_status); \
		g_need_clean_status = 1; \
		sp_tx_system_state_bak = sp_tx_system_state; \
		print_sys_state(sp_tx_system_state); \
	} while (0)

#define system_state_change_with_case(status) \
	do { \
		if (sp_tx_system_state >= status) { \
			pr_info("%s %s : change_case: clean_status: %xm,\n ", LOG_TAG, __func__, (uint)g_need_clean_status); \
			if ((sp_tx_system_state >= STATE_LINK_TRAINING) && (status < STATE_LINK_TRAINING)) \
			sp_write_reg_or(TX_P0, SP_TX_ANALOG_PD_REG, CH0_PD); \
			g_need_clean_status = 1; \
			sp_tx_system_state_bak = sp_tx_system_state; \
			sp_tx_system_state = (unchar)status; \
			print_sys_state(sp_tx_system_state); \
		} \
	} while (0)

#define sp_write_reg_or(address, offset, mask) \
		sp_write_reg(address, offset, ((unsigned char)__i2c_read_byte(address, offset) | (mask)))
#define sp_write_reg_and(address, offset, mask) \
	sp_write_reg(address, offset, ((unsigned char)__i2c_read_byte(address, offset) & (mask)))

#define sp_write_reg_and_or(address, offset, and_mask, or_mask) \
	sp_write_reg(address, offset, (((unsigned char)__i2c_read_byte(address, offset)) & and_mask) | (or_mask))
#define sp_write_reg_or_and(address, offset, or_mask, and_mask) \
	sp_write_reg(address, offset, (((unsigned char)__i2c_read_byte(address, offset)) | or_mask) & (and_mask))


void slimport_set_edid_complete(void)
{
	complete(&hdmi_edid_read_done);
}

void hardware_power_ctl(unchar enable)
{
	if (enable == 0)
		sp_tx_hardware_powerdown();
	else
		sp_tx_hardware_poweron();

	pr_debug("%s: %s: %s\n", LOG_TAG, __func__, (enable ? "ON" : "OFF"));
}

void wait_aux_op_finish(unchar *err_flag)
{
	unchar cnt;
	unchar c;

	*err_flag = 0;
	cnt = 150;
	while (__i2c_read_byte(TX_P0, AUX_CTRL2) & AUX_OP_EN) {
				mdelay(2);
				if ((cnt--) == 0) {
					pr_info("%s %s :aux operate failed!\n", LOG_TAG, __func__);
					*err_flag = 1;
					break;
				}
			}

	sp_read_reg(TX_P0, SP_TX_AUX_STATUS, &c);
	if (c & 0x0F) {
			pr_info("%s %s : wait aux operation failed %.2x\n", LOG_TAG, __func__, (uint)c);
			*err_flag = 1;
	}
}

#define SLIMPORT_DRV_DEBUG
void print_sys_state(unchar ss)
{
	switch(ss) {
	case STATE_INIT:
		debug_puts("-STATE_INIT- \n");
		break;
	case STATE_WAITTING_CABLE_PLUG:
		debug_puts("-STATE_WAITTING_CABLE_PLUG- \n");
		break;
	case STATE_SP_INITIALIZED:
		debug_puts("-STATE_SP_INITIALIZED- \n");
		break;
	case STATE_SINK_CONNECTION:
		debug_puts("-STATE_SINK_CONNECTION- \n");
		break;
	#ifdef ENABLE_READ_EDID
	case STATE_PARSE_EDID:
		debug_puts("-STATE_PARSE_EDID- \n");
		break;
	#endif
	case STATE_LINK_TRAINING:
		debug_puts("-STATE_LINK_TRAINING- \n");
		break;
	case STATE_VIDEO_OUTPUT:
		debug_puts("-STATE_VIDEO_OUTPUT-\n");
		break;
	case STATE_HDCP_AUTH:
		debug_puts("-STATE_HDCP_AUTH-\n");
		break;
	case STATE_AUDIO_OUTPUT:
		debug_puts("-STATE_AUDIO_OUTPUT-\n");
		break;
	case STATE_PLAY_BACK:
		debug_puts("-STATE_PLAY_BACK- \n");
		break;
	default:
		debug_puts("ERROR: system state is error1 \n");
		break;
	}
}

//DPCD
void sp_tx_rst_aux(void)
{
	sp_tx_aux_polling_disable();
	sp_write_reg_or(TX_P2, RST_CTRL2, AUX_RST);
	sp_write_reg_and(TX_P2, RST_CTRL2, ~AUX_RST);
	sp_tx_aux_polling_enable();
}

unchar sp_tx_aux_dpcdread_bytes(unchar addrh, unchar addrm,
	unchar addrl, unchar cCount, unchar *pBuf)
{
	unchar c,c1, i;
	unchar bOK;
	sp_write_reg(TX_P0, BUF_DATA_COUNT, 0x80);//clear buffer
	//command and length
	c = ((cCount - 1) << 4) | 0x09;
	sp_write_reg(TX_P0, AUX_CTRL, c);
	write_dpcd_addr(addrh, addrm, addrl);
	sp_write_reg_or(TX_P0, AUX_CTRL2, AUX_OP_EN);
	delay_ms(2);
	wait_aux_op_finish(&bOK);
	if (bOK == AUX_ERR) {
		debug_puts("aux read failed\n");
		//add by span 20130217.
		sp_read_reg(TX_P2, SP_TX_INT_STATUS1, &c);
		sp_read_reg(TX_P0, TX_DEBUG1, &c1);
		//if polling is enabled, wait polling error interrupt
		if(c1&POLLING_EN){
			if(c & POLLING_ERR)
				sp_tx_rst_aux();
		}else
			sp_tx_rst_aux();
		return AUX_ERR;
	}

	for (i = 0; i < cCount; i++) {
		sp_read_reg(TX_P0, BUF_DATA_0 + i, &c);
		*(pBuf + i) = c;
		if (i >= MAX_BUF_CNT)
			break;
	}
	return AUX_OK;
}


unchar sp_tx_aux_dpcdwrite_bytes(unchar addrh, unchar addrm, unchar addrl, unchar cCount, unchar *pBuf)
{
	unchar c, i, ret;
	c =  ((cCount - 1) << 4) | 0x08;
	sp_write_reg(TX_P0, AUX_CTRL, c);
	write_dpcd_addr(addrh, addrm, addrl);
	for (i = 0; i < cCount; i++) {
		c = *pBuf;
		pBuf++;
		sp_write_reg(TX_P0, BUF_DATA_0 + i, c);

		if (i >= 15)
			break;
	}
	sp_write_reg_or(TX_P0, AUX_CTRL2, AUX_OP_EN);
	wait_aux_op_finish(&ret);
	return ret;
}

unchar sp_tx_aux_dpcdwrite_byte(unchar addrh, unchar addrm, unchar addrl, unchar data1)
{
	unchar ret;
	sp_write_reg(TX_P0, AUX_CTRL, 0x08);//one byte write.
	write_dpcd_addr(addrh, addrm, addrl);
	sp_write_reg(TX_P0, BUF_DATA_0, data1);
	sp_write_reg_or(TX_P0, AUX_CTRL2, AUX_OP_EN);
	wait_aux_op_finish(&ret);
	return ret;
}

//========initialized system
void slimport_block_power_ctrl(enum SP_TX_POWER_BLOCK sp_tx_pd_block, unchar power)
{
	if(power == SP_POWER_ON)
		sp_write_reg_and(TX_P2, SP_POWERD_CTRL_REG, (~sp_tx_pd_block));
	else
		sp_write_reg_or(TX_P2, SP_POWERD_CTRL_REG, (sp_tx_pd_block));
	 debug_printf("sp_tx_power_on: %.2x\n", (uint)sp_tx_pd_block);
}

void vbus_power_ctrl(unsigned char ON)
{
	unchar i;
	if(ON == 0) {
		sp_write_reg_and(TX_P2, TX_PLL_FILTER, ~V33_SWITCH_ON);
		sp_write_reg_or(TX_P2, TX_PLL_FILTER5,  P5V_PROTECT_PD | SHORT_PROTECT_PD);
		debug_puts("3.3V output disabled\n");
	} else {
		for (i = 0; i < 5; i++) {
			sp_write_reg_and(TX_P2, TX_PLL_FILTER5, (~P5V_PROTECT_PD & ~SHORT_PROTECT_PD));
			sp_write_reg_or(TX_P2, TX_PLL_FILTER, V33_SWITCH_ON);
			if (!((unchar)__read_reg(TX_P2, TX_PLL_FILTER5) & 0xc0)) {
				debug_puts("3.3V output enabled\n");
				break;
			}else{
				debug_puts("VBUS power can not be supplied\n");
			}
		}
	}
}

/*
 * Reinit sp state machine, power-off dongle
 * and reset ANX3618 chip.
 */
static void sp_system_reset(void)
{
	vbus_power_ctrl(0);
	sp_write_reg_or(TX_P2, SP_TX_RST_CTRL_REG, HW_RST);
	sp_tx_variable_init();
	sp_tx_set_sys_state(STATE_WAITTING_CABLE_PLUG);
	msleep(500);
}


void system_power_down(void)
{
	vbus_power_ctrl(0);
	slimport_block_power_ctrl(SP_TX_PWR_REG, 0);
	slimport_block_power_ctrl(SP_TX_PWR_TOTAL, 0);
	hardware_power_ctl(0);
	sp_tx_variable_init();
}

void sp_tx_clean_state_machine(void)
{
	sp_tx_system_state = STATE_INIT;
	sp_tx_system_state_bak = STATE_INIT;
	sp_tx_sc_state = SC_INIT;
	sp_tx_LT_state = LT_INIT;
	#ifndef HDCP_AUTO_EN
	HDCP_state = HDCP_CAPABLE_CHECK;
	#endif
	sp_tx_vo_state = VO_WAIT_VIDEO_STABLE;
	sp_tx_ao_state = AO_INIT;
}
enum CHARGING_STATUS downstream_charging_status_get(void)
{
	return downstream_charging_status;
}
void downstream_charging_status_set(void)
{
	unchar c1;

	if (AUX_OK == sp_tx_aux_dpcdread_bytes
		(0x00, 0x05, 0x22, 1, &c1)) {
		if ((c1&0x01) == 0x00) {
			downstream_charging_status = FAST_CHARGING;
			sp_write_reg_or(TX_P2, TX_ANALOG_CTRL,  SHORT_DPDM);
			debug_puts("fast charging!\n");

			sp_set_ds_charge_type(FAST_CHARGING);
		} else {
			downstream_charging_status = NO_FAST_CHARGING;
			sp_write_reg_and(TX_P2, TX_ANALOG_CTRL, ~SHORT_DPDM);
			debug_puts("no charging!\n");
		}
	}
}

unchar sp_tx_cur_states(void)
{
	return sp_tx_system_state;
}

unchar sp_tx_cur_cable_type(void)
{
	return sp_tx_rx_type;
}
unchar sp_tx_cur_bw(void)
{
	return g_changed_bandwidth;
}

void sp_tx_set_bw(unchar bw)
{
	g_changed_bandwidth = bw;
}

static void sp_tx_variable_init(void)
{
	uint idata i;

	sp_tx_system_state = STATE_INIT;
	sp_tx_system_state_bak = STATE_INIT;
	sp_tx_rx_type = DWN_STRM_IS_NULL;
	#ifdef ENABLE_READ_EDID
	#ifdef CO3_DEUBG_MSG
	bedid_print = 0;
	#endif
	g_edid_break = 0;
	g_read_edid_flag = 0;
	g_edid_checksum = 0;
       for (i=0; i<256; i++)
		edid_blocks[i] =0;
	#endif
	sp_tx_LT_state = LT_INIT;

	if (sp_repeater_mode == false)
		HDCP_state = HDCP_CAPABLE_CHECK;

	g_need_clean_status = 0;
	sp_tx_sc_state = SC_INIT;
	sp_tx_vo_state = VO_WAIT_VIDEO_STABLE;
	sp_tx_ao_state = AO_INIT;
	g_changed_bandwidth = LINK_5P4G;
	g_hdmi_dvi_status = HDMI_MODE;

	sp_tx_test_lt = 0;
	sp_tx_test_bw = 0;
	sp_tx_test_edid = 0;
	ds_hpd_lost = false;
	audio_info_process = false;

	counter_unstable_video = 0;
	edid_reading_times = 0;
	edid_header_good = false;
	edid_checksum_good = false;
	hdcp_repeater_passed = false;
	downstream_charging_status = NO_CHARGING_CAPABLE;
	ds_auth_done = false;
	auth_done = false;
	#ifdef CEC_ENABLE
	cec_states = CEC_IDLE;
	cec_status_set(CEC_NOT_READY);
	#endif

}
static void hdmi_rx_tmds_phy_initialization(void)
{
	sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG2, 0xa9);
	//sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG4, 0x28);
	//sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG5, 0xe3);
	sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG7, 0x80);
	//sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG19, 0x00);
	//sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG21, 0x04);
	//sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG22, 0x38);

	/* HDMI RX PHY */
	sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG1, 0x90);
	sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG6, 0x92);
	sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG20, 0xf2);
}


void hdmi_rx_initialization(void)
{
	#ifdef CHANGE_TX_P0_ADDR
	/*Change 0x70 I2C address to 0x78*/
	sp_write_reg(TX_P2, SP_TX_DP_ADDR_REG1, 0xBC);
	#endif
	sp_write_reg(RX_P0, RX_MUTE_CTRL, AUD_MUTE | VID_MUTE);
	sp_write_reg_or(RX_P0, RX_CHIP_CTRL,
		MAN_HDMI5V_DET | PLLLOCK_CKDT_EN | DIGITAL_CKDT_EN);
	//sp_write_reg_or(RX_P0, RX_AEC_CTRL, AVC_OE);

	sp_write_reg_or(RX_P0, RX_SRST, HDCP_MAN_RST |SW_MAN_RST |
		TMDS_RST |VIDEO_RST );
	sp_write_reg_and(RX_P0, RX_SRST, (~HDCP_MAN_RST) & (~SW_MAN_RST) &
		(~TMDS_RST) & (~VIDEO_RST) );

	sp_write_reg_or(RX_P0, RX_AEC_EN0, AEC_EN06 |AEC_EN05);//Sync detect change , GP set mute
	sp_write_reg_or(RX_P0, RX_AEC_EN2, AEC_EN21);
	sp_write_reg_or(RX_P0, RX_AEC_CTRL, AVC_EN |AAC_OE | AAC_EN);

	sp_write_reg_and(RX_P0, RX_SYS_PWDN1, ~PWDN_CTRL);

	/*
	//default value is 0x00
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK1_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK2_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK3_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK4_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK5_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK6_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK7_REG, 0x00);
	*/
	sp_write_reg_or(RX_P0, RX_VID_DATA_RNG, R2Y_INPUT_LIMIT);
	#ifdef CEC_ENABLE
	sp_write_reg(RX_P0, RX_CEC_CTRL, CEC_RST);
	sp_write_reg(RX_P0, RX_CEC_SPEED, CEC_SPEED_27M);
	#endif
	//sp_write_reg(RX_P0, RX_CEC_CTRL, CEC_RX_EN);
	hdmi_rx_tmds_phy_initialization();
	hdmi_rx_set_hpd(0);
	hdmi_rx_set_termination(0);
	debug_puts("HDMI Rx is initialized...\n");
}

clock_Data const code pXTAL_data[XTAL_CLK_NUM] = {
	{19, 192},
	{24, 240},
	{25, 250},
	{26, 260},
	{27, 270},
	{38, 384},
	{52, 520},
	{27, 270},
};

void xtal_clk_sel(void)
{
	debug_printf("define XTAL_CLK:  %x\n ", (uint)XTAL_CLK);
	sp_write_reg_and_or(TX_P2, TX_ANALOG_DEBUG2, (~0x3c), 0x3c & (XTAL_CLK_DEF << 2) );
	sp_write_reg(TX_P0, 0xEC, (unchar)(((uint)XTAL_CLK_M10 )));
	sp_write_reg(TX_P0, 0xED,(unchar)(((uint)XTAL_CLK_M10 & 0xFF00) >> 2) | XTAL_CLK);

	sp_write_reg(TX_P0, I2C_GEN_10US_TIMER0, (unchar)(((uint)XTAL_CLK_M10 )));
	sp_write_reg(TX_P0, I2C_GEN_10US_TIMER1,(unchar)(((uint)XTAL_CLK_M10 & 0xFF00) >> 8));
	sp_write_reg(TX_P0, 0xBF, (unchar)(((uint)XTAL_CLK - 1)));

	/* CEC function need to change the value. */
	sp_write_reg_and_or(RX_P0, 0x49, 0x07, (unchar)(((((uint)XTAL_CLK) >> 1) - 2) << 3));
	/* sp_write_reg(RX_P0, 0x49, 0x5b); cec test */

}
void sp_tx_initialization(void)
{
	unchar c;

	/* I2C clock stretching */
	sp_read_reg(TX_P0, SP_TX_EXTRA_ADDR_REG, &c);
	c &= 0x7f;
	sp_write_reg(TX_P0, SP_TX_EXTRA_ADDR_REG, c);
	debug_printf("%s: %c\n", __func__, c );

	sp_write_reg(TX_P0, AUX_CTRL2, 0x30);  //xjh add set terminal reistor to 50ohm

	sp_tx_clean_hdcp_status();
	sp_read_reg(RX_P1, 0x2a, &c);
	if (c & SP_REPEATER_MODE) {
		debug_printf("ANX3618 in repeater mode\n");
		sp_repeater_mode = true;
	} else {
		debug_printf("ANX3618 in non-repeater mode\n");
		sp_repeater_mode = false;
	}

	if (sp_repeater_mode == false) {
		sp_write_reg_and(TX_P0, TX_HDCP_CTRL, (~AUTO_EN) & (~AUTO_START));
		sp_write_reg(TX_P0, OTP_KEY_PROTECT1, OTP_PSW1);
		sp_write_reg(TX_P0, OTP_KEY_PROTECT2, OTP_PSW2);
		sp_write_reg(TX_P0, OTP_KEY_PROTECT3, OTP_PSW3);
		sp_write_reg_or(TX_P0, HDCP_KEY_CMD, DISABLE_SYNC_HDCP);
	}

	sp_write_reg(TX_P2, SP_TX_VID_CTRL8_REG, VID_VRES_TH);

	sp_write_reg(TX_P0, HDCP_AUTO_TIMER, HDCP_AUTO_TIMER_VAL);
	sp_write_reg_or(TX_P0, TX_HDCP_CTRL ,LINK_POLLING);

	//sp_write_reg_or(TX_P0, 0x65 , 0x30);

	sp_write_reg_or(TX_P0, TX_LINK_DEBUG , M_VID_DEBUG);
	sp_write_reg_or(TX_P0, TX_DEBUG1, FORCE_HPD);

	/*
	sp_write_reg_or(TX_P2, TX_PLL_FILTER, AUX_TERM_50OHM);
	sp_write_reg_and(TX_P2, TX_PLL_FILTER5,
		(~P5V_PROTECT_PD) & (~SHORT_PROTECT_PD));
		*/
	sp_write_reg_or(TX_P2, TX_ANALOG_DEBUG2, POWERON_TIME_1P5MS);//.....................
	/*
	sp_write_reg_or(TX_P0, TX_HDCP_CTRL0,
		BKSV_SRM_PASS |KSVLIST_VLD);
		*/
	//sp_write_reg(TX_P2, TX_ANALOG_CTRL, 0xC5);

	xtal_clk_sel();
	sp_write_reg(TX_P0, AUX_DEFER_CTRL, 0x8C);

	sp_write_reg_or(TX_P0, TX_DP_POLLING, AUTO_POLLING_DISABLE);
	/*Short the link intergrity check timer to speed up bstatus
	polling for HDCP CTS item 1A-07 */
	sp_write_reg(TX_P0, SP_TX_LINK_CHK_TIMER, 0x1d);
	sp_write_reg_or(TX_P0, TX_MISC, EQ_TRAINING_LOOP);

	/* power down main link by default */
	sp_write_reg_or(TX_P0, SP_TX_ANALOG_PD_REG, CH0_PD);

	/*
	sp_write_reg(TX_P2, SP_COMMON_INT_MASK1, 0X00);
	sp_write_reg(TX_P2, SP_COMMON_INT_MASK2, 0X00);
	sp_write_reg(TX_P2, SP_COMMON_INT_MASK3, 0X00);
	sp_write_reg(TX_P2, SP_COMMON_INT_MASK4, 0X00);
	*/
	sp_write_reg(TX_P2, SP_INT_MASK, 0X90);
	sp_write_reg(TX_P2, SP_TX_INT_CTRL_REG, 0X01);
	/*disable HDCP mismatch function for VGA dongle*/
	/* sp_write_reg(TX_P0, SP_TX_LT_SET_REG, 0); */
	sp_tx_link_phy_initialization();
	gen_M_clk_with_downspeading();
	down_sample_en = 0;
}
void slimport_chip_initial(void)
{
	sp_tx_variable_init();
	vbus_power_ctrl(0);
	hardware_power_ctl(0);
	//sp_tx_set_sys_state(STATE_CABLE_PLUG);
}

bool slimport_chip_detect(void)
{
	uint c;
	unchar i;
	bool big_endian;
	unchar *ptemp;
	/*check whether CPU is big endian*/
	c = 0x1122;
	ptemp = (unchar *)&c;
	if (*ptemp == 0x11 && *(ptemp + 1) == 0x22)
		big_endian = 1;
	else
		big_endian = 0;
	hardware_power_ctl(1);
	c = 0;
	/*check chip id*/
	if (big_endian) {
		sp_read_reg(TX_P2, SP_TX_DEV_IDL_REG, (unchar *)(&c) + 1);
		sp_read_reg(TX_P2, SP_TX_DEV_IDH_REG, (unchar *)(&c));
	} else {
		sp_read_reg(TX_P2, SP_TX_DEV_IDL_REG, (unchar *)(&c));
		sp_read_reg(TX_P2, SP_TX_DEV_IDH_REG, (unchar *)(&c) + 1);
	}

	pr_info("%s %s : CHIPID: ANX%x\n", LOG_TAG, __func__, c & 0x0000FFFF);
	for(i = 0; i < CO3_NUMS; i++){
		pr_info("chipid_list: %x\n",  chipid_list[i]);//birdy
		if (c == chipid_list[i])
			return 1;
	}
	return 0;
}

//========cable plug
unchar is_cable_detected(void)
{
	return slimport_is_connected();
	/*
	if(cable_detected()) {
		mdelay(50);
		return cable_detected();
	}
	return 0;
	*/
}

void slimport_waitting_cable_plug_process(void)
{
	if (is_cable_detected()) {
		hardware_power_ctl(1);
		goto_next_system_state();
	} else {
		hardware_power_ctl(0);
	}
}
//========sink connection
void eeprom_reload(void)
{
	unchar sByteBuf[5];

	sByteBuf[0] = 0x61;
	sByteBuf[1] = 0x6e;
	sByteBuf[2] = 0x61;
	sByteBuf[3] = 0x6f;
	sByteBuf[4] = 0x6e;

	if(sp_tx_aux_dpcdwrite_bytes(0x00,0x04,0xf5,5,sByteBuf) == AUX_ERR){
		debug_puts("aux load initial error\n");
		return;
	}

	sByteBuf[0] = 0x04;
	sByteBuf[1] = 0x00;
	sByteBuf[2] = 0x00;
	sByteBuf[3] = 0x06;
	sByteBuf[3] = 0x40;


	if (sp_tx_aux_dpcdwrite_bytes(0x00,0x04,0xf0,5,sByteBuf) == AUX_ERR){
		debug_puts("reload error!\n");
	}

}


static unchar sp_tx_get_cable_type(CABLE_TYPE_STATUS det_cable_type_state, bool bdelay)
{
	unchar ds_port_preset;
	unchar aux_status;
	unchar data_buf[16];
	unchar cur_cable_type;

	ds_port_preset = 0;
	cur_cable_type = DWN_STRM_IS_NULL;
	downstream_charging_status = NO_CHARGING_CAPABLE;  //xjh add for charging

	aux_status = sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x05, 1, &ds_port_preset);
	debug_printf("DPCD 0x005: %x \n", (int)ds_port_preset);
	switch(det_cable_type_state)
	{
		case CHECK_AUXCH:
			if(AUX_OK == aux_status) {
				sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0, 0x0c, data_buf);
				det_cable_type_state = GETTED_CABLE_TYPE;
			} else {
				delay_ms(50);
				debug_puts(" AUX access error");
				break;
			}
		case GETTED_CABLE_TYPE:
			switch ((ds_port_preset  & (_BIT1 | _BIT2) ) >>1) {
			case 0x00:
				cur_cable_type = DWN_STRM_IS_DIGITAL;
				#ifdef DEMO_4K_2K
				sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x00, 3, data_buf);
				if ((data_buf[0] == 0x00) && (data_buf[1] == 0x22)
				    && (data_buf[2] == 0xb9)) {
					sp_write_reg_and_or(TX_P2, TX_ANALOG_DEBUG2, (~0x3c), 0x3c & (XTAL_24M << 2));
					down_sample_en = 1;
					pr_info("%s %s : Downstream is DP 4K dongle.\n", LOG_TAG, __func__);

				} else
					down_sample_en = 0;
				#endif
				debug_puts("Downstream is DP dongle.\n");
				break;
			case 0x01:
			case 0x03:
				sp_tx_aux_dpcdread_bytes(0x00, 0x04, 0x00, 8, data_buf);
				if (((data_buf[0] == 0x00) && (data_buf[1] == 0x22)
				    && (data_buf[2] == 0xb9) && (data_buf[3] == 0x61)
				    && (data_buf[4] == 0x39) && (data_buf[5] == 0x38)
				    && (data_buf[6] == 0x33))){
					cur_cable_type = DWN_STRM_IS_VGA_9832;
					debug_puts("Downstream is VGA dongle.\n");
				} else {
					cur_cable_type = DWN_STRM_IS_ANALOG;
					//for 7732
					if((data_buf[0] == 0x00) && (data_buf[1] == 0x22)    //xjh add for charging
					    && (data_buf[2] == 0xb9) && (data_buf[3] == 0x73)
					    && (data_buf[4] == 0x69) && (data_buf[5] == 0x76)
					    && (data_buf[6] == 0x61)) {
						downstream_charging_status = NO_FAST_CHARGING;
						if (bdelay) {
							/* eeprom_reload(); */
							delay_ms(150);
						}
						debug_puts("Downstream is general DP2VGA converter.\n");
					}
				}
				sp_tx_video_mute(false);
				external_block_en = false;
				break;
			case 0x02:
				if(AUX_OK == sp_tx_aux_dpcdread_bytes(0x00, 0x04, 0x00, 8, data_buf)){
					if ((data_buf[0] == 0xb9) && (data_buf[1] == 0x22)
					    && (data_buf[2] == 0x00) && (data_buf[3] == 0x00)
					    && (data_buf[4] == 0x00) && (data_buf[5] == 0x00)
					    && (data_buf[6] == 0x00)) {
						sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x23, 1, data_buf); //xjh add for charging
						if((data_buf[0]&0x7f)>=0x15){
							downstream_charging_status = NO_FAST_CHARGING;
						}
						//sp_tx_send_message(MSG_OCM_EN);
						cur_cable_type = DWN_STRM_IS_HDMI_7730;
						debug_puts("Downstream is HDMI dongle.\n");

					} else {
						#ifdef DEMO_4K_2K
						sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x00, 3, data_buf);
						if ((data_buf[0] == 0x00) && (data_buf[1] == 0x22)
							&& (data_buf[2] == 0xb9)) {
							/* ANX7748/6470 OUI */
							sp_write_reg_and_or(TX_P2, TX_ANALOG_DEBUG2, (~0x3c), 0x3c & (XTAL_24M << 2));
							down_sample_en = 1;
							pr_info("%s %s : Downstream is HDMI 4K dongle.\n", LOG_TAG, __func__);

						} else {
								down_sample_en = 0;
						}
						#endif

						cur_cable_type = DWN_STRM_IS_DIGITAL;
						debug_puts("Downstream is general DP2HDMI converter.\n");
					}
				}else
					debug_puts("dpcd read error!.\n");

				break;
			default:
				cur_cable_type = DWN_STRM_IS_NULL;
				debug_puts("Downstream can not recognized.\n");
				break;
			}
		default:
			break;
	}
	return cur_cable_type;
}

unchar sp_tx_get_hdmi_connection(void)
{
	unchar c;
	//msleep(200);//why delay here? 20130217?

	if(AUX_OK != sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x18, 1, &c)){
		return 0;
	}

	if ((c & 0x41) == 0x41) {
		//sp_tx_aux_dpcdwrite_byte(0x00, 0x05, 0xf3, 0x70);//removed by span, no use 20130217
		return 1;
	} else
		return 0;

}

unchar sp_tx_get_vga_connection(void)
{
	unchar c;
	if(AUX_OK != sp_tx_aux_dpcdread_bytes(0x00, 0x02, DPCD_SINK_COUNT, 1, &c)){
		debug_puts("aux error.\n");
		return 0;
	}

	if (c & 0x01)
		return 1;
	else
		return 0;
}
unchar sp_tx_get_dp_connection(void)
{
	unchar c;

	if(AUX_OK != sp_tx_aux_dpcdread_bytes(0x00, 0x02, DPCD_SINK_COUNT, 1, &c))
		return 0;

	if (c & 0x1f) {
		sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x04, 1, &c);
		if (c & 0x20)
			sp_tx_aux_dpcdwrite_byte(0x00, 0x06, 0x00, 0x20);
		return 1;
	} else
		return 0;
}
unchar sp_tx_get_downstream_connection(void)
{
	switch(sp_tx_rx_type) {
	case DWN_STRM_IS_HDMI_7730:
		return sp_tx_get_hdmi_connection();
	case DWN_STRM_IS_DIGITAL:
		return sp_tx_get_dp_connection();
	case DWN_STRM_IS_ANALOG:
	case DWN_STRM_IS_VGA_9832:
		return sp_tx_get_vga_connection();
	case DWN_STRM_IS_NULL:
	default:
		return 0;
	}
	return 0;
}
void slimport_sink_connection(void)
{
	if (!is_cable_detected()) {
		sp_system_reset();
		return;
	}
	switch(sp_tx_sc_state) {
		case SC_INIT:
			sp_tx_sc_state++;
		case SC_CHECK_CABLE_TYPE:
		case SC_WAITTING_CABLE_TYPE:
		default:
			sp_tx_rx_type = sp_tx_get_cable_type(CHECK_AUXCH, 1);
			if(sp_tx_rx_type == DWN_STRM_IS_NULL) {
				sp_tx_sc_state++;
				if(sp_tx_sc_state >= SC_WAITTING_CABLE_TYPE){
					sp_tx_sc_state = SC_NOT_CABLE;
					debug_puts("Can not get cable type!\n");
				}
				break;
			}
			//dongle has fast charging detection capability
			if(downstream_charging_status != NO_CHARGING_CAPABLE){
				downstream_charging_status_set();
			}
			sp_tx_sc_state = SC_SINK_CONNECTED;
		case SC_SINK_CONNECTED:
			if (sp_tx_get_downstream_connection()) {
				#ifdef CEC_ENABLE
				cec_states = CEC_INIT;
				#endif
				goto_next_system_state();
			}
			break;
		case SC_NOT_CABLE:
			sp_system_reset();
			break;
	}
}
/******************start EDID process********************/
void sp_tx_enable_video_input(unchar enable)
{
	unchar c;
	sp_read_reg(TX_P2, VID_CTRL1, &c);
	if (enable) {
		if((c & VIDEO_EN) != VIDEO_EN) {
			c = (c & 0xf7) | VIDEO_EN;
			sp_write_reg(TX_P2, VID_CTRL1, c);
			debug_puts("Slimport Video is enabled!\n");
		}
	} else {
		if( (c & VIDEO_EN) == VIDEO_EN){
			c &= ~VIDEO_EN;
			sp_write_reg(TX_P2, VID_CTRL1, c);
			debug_puts("Slimport Video is disabled!\n");
		}
	}
}
void sp_tx_send_message(enum SP_TX_SEND_MSG message)
{
	unchar c;

	switch (message) {
	default:
		break;
	case MSG_INPUT_HDMI:
		if (sp_tx_rx_type == DWN_STRM_IS_HDMI_7730)
			sp_tx_aux_dpcdwrite_byte(0x00, 0x05, 0x26, 0x01);
		break;

	case MSG_INPUT_DVI:
		if (sp_tx_rx_type == DWN_STRM_IS_HDMI_7730)
			sp_tx_aux_dpcdwrite_byte(0x00, 0x05, 0x26, 0x00);
		break;

	case MSG_CLEAR_IRQ:
		//debug_puts("clear irq start!\n");
		sp_tx_aux_dpcdread_bytes(0x00, 0x04, 0x10, 1, &c);
		//debug_puts("clear irq middle!\n");
		c |= 0x01;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x04, 0x10, c);
		//debug_puts("clear irq end!\n");
		break;
	}

}

#ifdef ENABLE_READ_EDID
static unchar get_edid_detail(unchar *data_buf)
{
	uint	pixclock_edid;

#if 0  //xjh mod2
	unchar	interlaced;
	uint	active_h;
	uint	active_v;
	ulong temp;


	/* See VESA Spec */
	/* EDID_TIMING_DESC_UPPER_H_NIBBLE[0x4]: Relative Offset to the EDID
	 *   detailed timing descriptors - Upper 4 bit for each H active/blank
	 *   field */
	/* EDID_TIMING_DESC_H_ACTIVE[0x2]: Relative Offset to the EDID detailed
	 *   timing descriptors - H active */
	active_h = ((((uint)data_buf[0x4] >> 0x4) & 0xF) << 8) | data_buf[0x2];
	/* EDID_TIMING_DESC_UPPER_V_NIBBLE[0x7]: Relative Offset to the EDID
	 *   detailed timing descriptors - Upper 4 bit for each V active/blank
	 *   field */
	/* EDID_TIMING_DESC_V_ACTIVE[0x5]: Relative Offset to the EDID detailed
	 *   timing descriptors - V active */
	active_v = ((((uint)data_buf[0x7] >> 0x4) & 0xF) << 8)
		| data_buf[0x5];
	/*
	 * CEA 861-D: interlaced bit is bit[7] of byte[0x11]
	 */
	interlaced = (data_buf[0x11] & 0x80) >> 7;
	temp = active_v;
	if(interlaced)
		temp = temp >>  2;
	if(temp >= 1024)
		return LINK_5P4G;
	else if(temp > 900000)
		return LINK_1P62G;
	else
		return LINK_2P7G;
	debug_printf("=============EDID Data 0: %.2x============\n", (uint)data_buf[0] );
	debug_printf("=============EDID Data 1: %.2x============\n", (uint)data_buf[1] );
	debug_printf("=============EDID Data 2: %.2x============\n", (uint)data_buf[2] );
	debug_printf("=============EDID Data 3: %.2x============\n", (uint)data_buf[3] );
	debug_printf("=============EDID Data 4: %.2x============\n", (uint)data_buf[4] );
#else

	pixclock_edid =((((uint)data_buf[1] << 8) ) |((uint)data_buf[0] & 0xFF));
	//debug_printf("=============pixclock  via EDID : %d\n", (uint)pixclock_edid);
	if (pixclock_edid <= 5300)
		return LINK_1P62G;
	else if ((5300 < pixclock_edid) && (pixclock_edid <= 8900))
		return LINK_2P7G;
	else if ((8900 < pixclock_edid) && (pixclock_edid <= 18000))
		return LINK_5P4G;
	else
		return LINK_6P75G;
#endif

}
static unchar parse_edid_to_get_bandwidth(void)
{
	unchar desc_offset = 0;
	unchar i, bandwidth, temp;
	bandwidth = LINK_1P62G;
	temp = LINK_1P62G;
	i = 0;
	while (4 > i && 0 != edid_blocks[0x36+desc_offset]) {
		temp = get_edid_detail(edid_blocks+0x36+desc_offset);
		debug_printf("bandwidth via EDID : %x\n", (uint)temp);
		if(bandwidth < temp)
			bandwidth = temp;
		if(bandwidth > LINK_5P4G)  //xjh mod2   >=
			break;
		desc_offset += 0x12;
		++i;
	}
	return bandwidth;

}
static void sp_tx_aux_wr(unchar offset)
{
	sp_write_reg(TX_P0, BUF_DATA_0, offset);
	sp_write_reg(TX_P0, AUX_CTRL, 0x04);
	sp_write_reg_or(TX_P0, AUX_CTRL2, AUX_OP_EN);
	wait_aux_op_finish(&g_edid_break);
}

static void sp_tx_aux_rd(unchar len_cmd)
{
	sp_write_reg(TX_P0, AUX_CTRL, len_cmd);
	sp_write_reg_or(TX_P0, AUX_CTRL2, AUX_OP_EN);
	wait_aux_op_finish(&g_edid_break);
}

unchar sp_tx_get_edid_block(void)
{
	unchar c;
#ifdef CONFIG_AMAZON_METRICS_LOG
	char buf[SLIMPORT_METRICS_STR_LEN] = {0};
#endif

	sp_tx_aux_wr(0x7e);
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, BUF_DATA_0, &c);

	debug_printf("EDID Block = %d\n", (int)(c + 1));

#ifdef CONFIG_AMAZON_METRICS_LOG
	snprintf(buf, sizeof(buf),
		"slimport:def:edid_block=%d;CT;1:NR", (int)(c + 1));
	log_to_metrics(ANDROID_LOG_INFO, "kernel", buf);
#endif
	return c;
}
void edid_read(unchar offset, unchar *pblock_buf)
{
	unchar  data_cnt,cnt;
	unchar idata c;
	sp_tx_aux_wr(offset);
	sp_tx_aux_rd(0xf5);//set I2C read com 0x05 mot = 1 and read 16 bytes
	data_cnt = 0;
	cnt = 0;

	while((data_cnt) < 16)
	{
		sp_read_reg(TX_P0, BUF_DATA_COUNT, &c);

		if(c != 0x10)
			debug_printf("edid read: len : %x \n", (uint)c);

		if((c & 0x1f) != 0){
			data_cnt = data_cnt + c;
			do{
				sp_read_reg(TX_P0, BUF_DATA_0 + c -1, &(pblock_buf[c - 1]));
				//debug_puts("index: %x, data: %x ", (uint)c, (uint)pblock_buf[c]);
				//debug_puts("\n");
				if(c == 1)
					break;
			}while(c--);


		}else {
			debug_puts("edid read : length: 0\n");
			if(cnt++ <= 2){
				sp_tx_rst_aux();
				c = 0x05 | ((0x0f - data_cnt) << 4);
				sp_tx_aux_rd(c);
			}else {
				 g_edid_break = 1;
				 break;
			}
		}
	}
	//issue a stop every 16 bytes read
	sp_write_reg(TX_P0, AUX_CTRL, 0x01);
	sp_write_reg_or(TX_P0, AUX_CTRL2, ADDR_ONLY_BIT | AUX_OP_EN);
	wait_aux_op_finish(&g_edid_break);
	sp_tx_addronly_set(0);

}
void sp_tx_edid_read_initial(void)
{
	sp_write_reg(TX_P0, AUX_ADDR_7_0, 0x50);
	sp_write_reg(TX_P0, AUX_ADDR_15_8, 0);
	sp_write_reg_and(TX_P0, AUX_ADDR_19_16, 0xf0);
}
static void segments_edid_read(unchar segment, unchar offset)
{
	unchar c,cnt;
	int i;

	sp_write_reg(TX_P0, AUX_CTRL, 0x04);

	sp_write_reg(TX_P0, AUX_ADDR_7_0, 0x30);

	sp_write_reg_or(TX_P0, AUX_CTRL2, ADDR_ONLY_BIT | AUX_OP_EN);
	sp_tx_addronly_set(0);

	sp_read_reg(TX_P0, AUX_CTRL2, &c);
	wait_aux_op_finish(&g_edid_break);
	sp_read_reg(TX_P0, AUX_CTRL, &c);

	sp_write_reg(TX_P0, BUF_DATA_0, segment);

	//set I2C write com 0x04 mot = 1
	sp_write_reg(TX_P0, AUX_CTRL, 0x04);

	sp_write_reg_and_or(TX_P0, AUX_CTRL2, ~ADDR_ONLY_BIT, AUX_OP_EN);
	cnt = 0;
	sp_read_reg(TX_P0, AUX_CTRL2, &c);
	while(c&AUX_OP_EN)
	{
		delay_ms(1);
		cnt ++;
		if(cnt == 10)
		{
			debug_puts("write break");
			sp_tx_rst_aux();
			cnt = 0;
			g_edid_break = 1;
			return;// bReturn;
		}
		sp_read_reg(TX_P0, AUX_CTRL2, &c);

	}

	sp_write_reg(TX_P0, AUX_ADDR_7_0, 0x50);

	sp_tx_aux_wr(offset);

	sp_tx_aux_rd(0xf5);
       cnt = 0;
	for(i=0; i<16; i++)
	{
		sp_read_reg(TX_P0, BUF_DATA_COUNT, &c);
		while((c & 0x1f) == 0)
		{
			delay_ms(2);
			cnt ++;
			sp_read_reg(TX_P0, BUF_DATA_COUNT, &c);
			if(cnt == 10)
			{
				debug_puts("read break");
				sp_tx_rst_aux();
				g_edid_break = 1;
				return;
			}
		}


		sp_read_reg(TX_P0, BUF_DATA_0+i, &c);
	}

	sp_write_reg(TX_P0, AUX_CTRL, 0x01);
	sp_write_reg_or(TX_P0, AUX_CTRL2, ADDR_ONLY_BIT | AUX_OP_EN);
	sp_write_reg_and(TX_P0, AUX_CTRL2, ~ADDR_ONLY_BIT);
	sp_read_reg(TX_P0, AUX_CTRL2, &c);
	while(c & AUX_OP_EN)
		sp_read_reg(TX_P0, AUX_CTRL2, &c);
//	sp_tx_addronly_set(0);


}
static bool edid_checksum_result(unchar *pBuf)
{
	unchar idata cnt, checksum;
	checksum = 0;

	for (cnt=0; cnt <0x80; cnt++)
		checksum = checksum + pBuf[cnt];

	g_edid_checksum = checksum -pBuf[0x7f];
	g_edid_checksum = ~g_edid_checksum + 1;

//	debug_printf("====g_edid_checksum  ==%x \n", (uint)g_edid_checksum);
//	debug_printf("====pBuf[0x7f]  ==%x \n", (uint)pBuf[0x7f]);


	return checksum == 0 ? 1 : 0;
}
static void edid_header_result(unchar *pBuf)
{
	if ((pBuf[0] == 0) && (pBuf[7] == 0) && (pBuf[1] == 0xff) &&
		(pBuf[2] == 0xff) && (pBuf[3] == 0xff) &&
		(pBuf[4] == 0xff) && (pBuf[5] == 0xff) && (pBuf[6] == 0xff)) {
		edid_header_good = true;
		debug_puts("Good EDID header!\n");
	} else {
		edid_header_good = false;
		debug_puts("Bad EDID header!\n");
	}

}

static void check_edid_data(unchar *pblock_buf)
{
	unchar i;
	bool checksum_status = true;
	edid_header_result(pblock_buf);
	for (i = 0; i <= ((pblock_buf[0x7e] > 1) ? 1 : pblock_buf[0x7e]); i++) {
		if (!edid_checksum_result(pblock_buf + i * 128)) {
			checksum_status = false;
			debug_printf("Block %x edid checksum error\n", (uint)i);
			break;
		} else
			debug_printf("Block %x edid checksum OK\n", (uint)i);
	}
	edid_checksum_good = checksum_status;
}

unchar retry_read_edid_block(int times)
{
	int retry_time = 0;
	unchar block_num = 0xff;
#ifdef CONFIG_AMAZON_METRICS_LOG
	char buf[SLIMPORT_METRICS_STR_LEN];
#endif
	for (retry_time = 0; retry_time < times; retry_time++) {
		block_num = sp_tx_get_edid_block();
		msleep(20);
		if ((block_num <= 3) && (block_num >= 0)) {
#ifdef CONFIG_AMAZON_METRICS_LOG
			snprintf(buf, sizeof(buf),
				"slimport:def:read_edid_times=%d;CT;1:NR",
				times);
			log_to_metrics(ANDROID_LOG_INFO, "kernel", buf);
			snprintf(buf, sizeof(buf),
				"slimport:def:block_num=%d;CT;1:NR",
				(int)block_num);
			log_to_metrics(ANDROID_LOG_INFO, "kernel", buf);
#endif
			return block_num;
		}
	}
#ifdef CONFIG_AMAZON_METRICS_LOG
	snprintf(buf, sizeof(buf),
		"slimport:def:read_edid_times=%d;CT;1:NR", times);
	log_to_metrics(ANDROID_LOG_INFO, "kernel", buf);
	snprintf(buf, sizeof(buf),
		"slimport:def:block_num=%d;CT;1:NR", (int)block_num);
	log_to_metrics(ANDROID_LOG_INFO, "kernel", buf);
#endif
	return block_num;
}
bool sp_tx_edid_read(unchar *pedid_blocks_buf)
{
	unchar offset = 0;
	unchar count, blocks_num;
	unchar pblock_buf[16];
	unchar idata i, j, c;
	unchar aux_read;

	g_edid_break = 0;
	sp_tx_edid_read_initial();
	sp_write_reg(TX_P0, AUX_CTRL, 0x04);
	sp_write_reg_or(TX_P0, AUX_CTRL2, 0x03);
	wait_aux_op_finish(&g_edid_break);
	sp_tx_addronly_set(0);

	blocks_num = sp_tx_get_edid_block();

	if (g_edid_break && (sp_tx_rx_type == DWN_STRM_IS_HDMI_7730)) {
		/*DDC reset*/
		debug_printf("%s:====reset anx7730 DDC====\n", __func__);
		/*0x50:06*/
		i2c_master_write_reg(0x0, 0x06, 0x02);
		/*0x72:07*/
		i2c_master_write_reg(0x5, 0x07, 0x10);
		/*0x7a:43*/
		i2c_master_write_reg(0x6, 0x43, 0x0);
		i2c_master_write_reg(0x6, 0x43, 0x06);
		delay_ms(50);
		g_edid_break = 0;
		sp_tx_edid_read_initial();
		sp_write_reg(TX_P0, AUX_CTRL, 0x04);
		sp_write_reg_or(TX_P0, AUX_CTRL2, 0x03);
		wait_aux_op_finish(&g_edid_break);
		sp_tx_addronly_set(0);
		blocks_num = sp_tx_get_edid_block();
	}
	if (!(blocks_num <= 3) && (blocks_num >= 0)) {
		blocks_num = retry_read_edid_block(5);
		if (blocks_num > 3) {
			g_edid_break = 1;
			return false;
		}
	}
	count = 0;
	do {
		switch(count) {
			case 0:
			case 1:
				for (i = 0; i < 8; i++) {
					offset = (i+count*8) * 16;
					edid_read(offset, pblock_buf);
					if (g_edid_break == 1)
						break;
					for(j = 0; j<16; j++){
						pedid_blocks_buf[offset + j] = pblock_buf[j];
					}
				}
				break;
			case 2:
				offset = 0x00;
				for (j = 0; j < 8; j++) {
					if (g_edid_break == 1)
						break;
					segments_edid_read(count / 2, offset);
					offset = offset + 0x10;
				}
				break;
			case 3:
				offset = 0x80;
				for (j = 0; j < 8; j++) {
					if (g_edid_break == 1)
						break;
					segments_edid_read(count /2, offset);
					offset = offset + 0x10;
				}
				break;
			default:
				break;
		}
		count++;
		if (g_edid_break == 1)
			break;
	}while(blocks_num >= count);

	#ifdef CO3_DEUBG_MSG
	/* For print edid command "dumpedid" */
	if(bedid_print){
		uint k;
		bedid_print = 0;
		debug_printf("             0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F\n");
		for(k=0; k<(128*((uint)blocks_num+1)); k++)
		{
			if((k&0x0f)==0)
				debug_printf("\n edid: [%.2x]  %.2x  ", (uint)(k % 0x0f), (uint)pedid_blocks_buf[k]);
			else
				debug_printf("%.2x   ", (uint)pedid_blocks_buf[k]);

			if((k&0x0f)==0x0f)
				debug_printf("\n");
		}

		check_edid_data(pedid_blocks_buf);
	}
	#endif

	sp_tx_rst_aux();
	//check edid data
	if(g_read_edid_flag == 0){
		check_edid_data(pedid_blocks_buf);
		g_read_edid_flag = 1;
	}
	 // test edid <<

	aux_read = sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x18, 1, &c);
	if ((c & 0x04) && (aux_read != AUX_ERR))
	{
		debug_printf("check sum = %.2x\n",  (uint)g_edid_checksum);
		c = g_edid_checksum;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x61, 1, &c);

		c = 0x04;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60, 1, &c);
		debug_puts("Test EDID done\n");

	}
	 // test edid  >>
	return true;
}
static bool check_with_pre_edid(unchar *org_buf)
{
	unchar idata i;
	unchar temp_buf[16];
	bool return_flag;
	return_flag = 0;
	//check checksum and blocks number
	g_edid_break = 0;
	sp_tx_edid_read_initial();
	sp_write_reg(TX_P0, AUX_CTRL, 0x04);
	sp_write_reg_or(TX_P0, AUX_CTRL2, 0x03);
	wait_aux_op_finish(&g_edid_break);
	sp_tx_addronly_set(0);

	edid_read(0x70, temp_buf);

	if(g_edid_break == 0) {

		for(i = 0; i < 16; i++){
			if(org_buf[0x70 + i] != temp_buf[i]){
				debug_puts("different checksum and blocks num\n");
				return_flag = 1;  //need re-read edid
				break;
			}
		}
	}else
		return_flag = 1;

	if(return_flag)
		goto return_point;

	//check edid information
	edid_read(0x08, temp_buf);
	if(g_edid_break == 0) {
		for(i = 0; i < 16; i++){
			if(org_buf[i + 8] != temp_buf[i]){
				debug_puts("different edid information\n");
				return_flag = 1;
				break;
			}
		}
	}else
		return_flag = 1;

return_point:

	sp_tx_rst_aux();
	return return_flag;
}

void edid_failed_process(void)
{
	unchar value = 0;
	unchar counter = 2;

	debug_puts("edid reading failure, back to state process\n");
	vbus_power_ctrl(0);
	slimport_block_power_ctrl(SP_TX_PWR_REG, 0);
	slimport_block_power_ctrl(SP_TX_PWR_TOTAL, 0);
	delay_ms(150);
	slimport_block_power_ctrl(SP_TX_PWR_REG, SP_POWER_ON);
	slimport_block_power_ctrl(SP_TX_PWR_TOTAL, SP_POWER_ON);
	vbus_power_ctrl(1);

	/*
	 * Check 7730 dongle DDC status, DDC SDA/SCL is high means DDC level
	 * is normal, if value is not 0x0c such as 0x00, 0x04,0x08,it means the
	 * DDC SDA/SCL is pulled down.
	 */
	if (sp_tx_rx_type == DWN_STRM_IS_HDMI_7730) {
		i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x7a,
			SP_RX_7730_HDMI_CHIP_STATUS, &value);

		if ((value & SP_RX_7730_HDMI_DDC_LEVEL_NORMAL) ==
		    SP_RX_7730_HDMI_DDC_LEVEL_NORMAL) {
			debug_puts("7730 dongle DDC is normal\n");
		} else {
			counter = 1;
			debug_puts("7730 dongle DDC is pulled low\n");
		}
	}

	if (edid_reading_times < counter) {
		++edid_reading_times;
		sp_tx_set_sys_state(STATE_SP_INITIALIZED);
		pr_err("%s: edid reading failure, transfer to STATE_SP_INITIALIZED\n",
			LOG_TAG);
	} else {
		edid_reading_times = 0;
		hdmi_rx_set_hpd(1);
		hdmi_rx_set_termination(1);
		sp_tx_get_rx_bw(&g_changed_bandwidth);
		goto_next_system_state();
		pr_err("%s: edid reading failure over 3 times, transfer to STATE_LINK_TRAINING\n",
			LOG_TAG);
	}
}

void slimport_edid_process(void)
{
	unchar temp_value, temp_value1;
	unchar idata i;
	unchar value;
	u32 timeout;

	debug_puts("edid_process\n");
	init_completion(&hdmi_edid_read_done);

	if(g_read_edid_flag == 1){
		if(check_with_pre_edid(edid_blocks))
			g_read_edid_flag = 0;
		else
			debug_puts("Don`t need to read edid!\n");
	}

	if(g_read_edid_flag == 0){
		sp_tx_edid_read(edid_blocks);
		if(g_edid_break)
			debug_puts("ERR:EDID corruption!\n");
	}

	if (edid_header_good && edid_checksum_good && !g_edid_break) {
		edid_reading_times = 0;
		/*Release the HPD after the OTP loaddown*/
		i = 10;
		do {
			value = __read_reg(TX_P0, HDCP_KEY_STATUS);
			if ((value & 0x01))
				break;
			else {
				debug_puts("waiting HDCP KEY loaddown\n");
				delay_ms(1);
			}
		} while (--i);

		sp_write_reg(RX_P0, HDMI_RX_INT_MASK1_REG, 0xe2);
		hdmi_rx_set_hpd(1);
		debug_puts("hdmi_rx_set_hpd 1 !\n");
#ifdef CONFIG_AMAZON_METRICS_LOG
		log_to_metrics(ANDROID_LOG_INFO, "kernel",
			"slimport:def:HPD=ON;DV;1:NR");
#endif
		hdmi_rx_set_termination(1);


		timeout = wait_for_completion_timeout(&hdmi_edid_read_done,
						msecs_to_jiffies(500));
		if (!timeout)
			debug_puts("HDMI EDID read timeout\n");
		else
			debug_puts("HDMI EDID read completed\n");

		sp_tx_get_rx_bw(&temp_value);
		temp_value1 = parse_edid_to_get_bandwidth();
		if (temp_value <= temp_value1)
			temp_value1 = temp_value;
		debug_printf("set link bw in edid %x\n", (uint)temp_value1);
		g_changed_bandwidth = temp_value1;
		goto_next_system_state();
	} else {
#ifdef CONFIG_AMAZON_METRICS_LOG
		log_to_metrics(ANDROID_LOG_INFO, "kernel",
			"slimport:def:edid_read_fail=true;DV;1:NR");
#endif
		edid_failed_process();
	}
}
#endif
/******************End EDID process********************/
/******************start Link training process********************/

static void sp_tx_lvttl_bit_mapping(void)
{
	unchar c, colorspace;
	unchar vid_bit;

	vid_bit = 0;
	sp_read_reg(RX_P1,HDMI_RX_AVI_DATA00_REG, &colorspace);
	colorspace &= 0x60;
	switch (((__read_reg(RX_P0, HDMI_RX_VIDEO_STATUS_REG1) & COLOR_DEPTH) >> 4)) {
	default:
	case Hdmi_legacy:
		c = IN_BPC_8BIT;
		vid_bit = 0;
		break;
	case Hdmi_24bit:
		c = IN_BPC_8BIT;
		if(colorspace == 0x20)
			vid_bit = 5;
		else
			vid_bit = 1;
		break;
	case Hdmi_30bit:
		c = IN_BPC_10BIT;
		if(colorspace == 0x20)
			vid_bit = 6;
		else
			vid_bit = 2;
		break;
	case Hdmi_36bit:
		c = IN_BPC_12BIT;
		if(colorspace == 0x20)
			vid_bit = 6;
		else
			vid_bit = 3;
		break;
	}
	sp_write_reg_and_or(TX_P2, SP_TX_VID_CTRL2_REG, 0x8c, colorspace >> 5 | c);
	//delay_ms(500);
	if (down_sample_en == 1 && c == IN_BPC_10BIT)
		vid_bit = 3;
	sp_write_reg_and_or(TX_P2, BIT_CTRL_SPECIFIC, 0x00, ENABLE_BIT_CTRL | vid_bit << 1);

	if (sp_tx_test_edid ){
		//set color depth to 18-bit for link cts
		sp_write_reg_and(TX_P2, SP_TX_VID_CTRL2_REG, 0x8f);
		sp_tx_test_edid = 0;
		debug_puts("***color space is set to 18bit***");
	}

	if (colorspace) {
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET1, 0x80);
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET2, 0x00);
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET3, 0x80);
	} else {
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET1, 0x0);
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET2, 0x0);
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET3, 0x0);
	}


}

ulong sp_tx_pclk_calc(void)
{
	ulong str_plck;
	uint vid_counter;
	 unchar c;
	sp_read_reg(RX_P0,0x8d, &c);
	vid_counter = c;
	vid_counter = vid_counter << 8;
	sp_read_reg(RX_P0,0x8c, &c);
	vid_counter |=  c;
	str_plck = ((ulong)vid_counter * XTAL_CLK_M10)  >> 12;
	debug_printf("PCLK = %d.%d \n", (((uint)(str_plck))/10), ((uint)str_plck - (((uint)str_plck/10)*10)));
	return str_plck ;
}

static unchar sp_tx_bw_lc_sel(ulong pclk)
{
	ulong pixel_clk;
	unchar c1;
	switch (((__read_reg(RX_P0, HDMI_RX_VIDEO_STATUS_REG1) & COLOR_DEPTH) >> 4)) {
	case Hdmi_legacy:
	case Hdmi_24bit:
	default:
		pixel_clk = pclk;
		break;
	case Hdmi_30bit:
		pixel_clk = (pclk * 5) >> 2;
		break;
	case Hdmi_36bit:
		pixel_clk = (pclk * 3) >> 1;
		break;
	}
	debug_printf("pixel_clk = %d.%d\n", (((uint)(pixel_clk))/10), ((uint)pixel_clk - (((uint)pixel_clk/10)*10)));

	if (pixel_clk <= 530) {
		c1 = LINK_1P62G;
	} else if ((530 < pixel_clk) && (pixel_clk <= 890)) {
			c1 = LINK_2P7G;
	} else if ((890 < pixel_clk) && (pixel_clk <= 1800)) {
			c1 = LINK_5P4G;
	} else {
		 c1 = LINK_6P75G;
		 if (pixel_clk > 2240)
			down_sample_en = 1;
		 else
			down_sample_en = 0;
	}

	if(sp_tx_get_link_bw() != c1){
		g_changed_bandwidth = c1;
		debug_printf("It is different bandwidth between sink support and cur video!%.2x\n",(uint)c1);
		return 1;
	}
     return 0;
}
void sp_tx_spread_enable(unchar benable)
{
	unchar c;

	sp_read_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, &c);

	if (benable) {
		c |= SP_TX_SSC_DWSPREAD;
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, c);

		sp_tx_aux_dpcdread_bytes(0x00, 0x01,
			DPCD_DOWNSPREAD_CTRL, 1, &c);
		c |= SPREAD_AMPLITUDE;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01, DPCD_DOWNSPREAD_CTRL, c);

	} else {
		c &= ~SP_TX_SSC_DISABLE;
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, c);

		sp_tx_aux_dpcdread_bytes(0x00, 0x01,
			DPCD_DOWNSPREAD_CTRL, 1, &c);
		c &= ~SPREAD_AMPLITUDE;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01, DPCD_DOWNSPREAD_CTRL, c);
	}

}
void sp_tx_config_ssc(enum SP_SSC_DEP sscdep)
{
	sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, 0x0); //clear register
	sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, sscdep);
	sp_tx_spread_enable(1);
}


void sp_tx_enhancemode_set(void)
{
	unchar c;
	sp_tx_aux_dpcdread_bytes(0x00, 0x00, DPCD_MAX_LANE_COUNT, 1, &c);
	if (c & ENHANCED_FRAME_CAP) {
		sp_write_reg_or(TX_P0, SP_TX_SYS_CTRL4_REG, ENHANCED_MODE);
		sp_tx_aux_dpcdread_bytes(0x00, 0x01,
			DPCD_LANE_COUNT_SET, 1, &c);
		c |= ENHANCED_FRAME_EN;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01,
			DPCD_LANE_COUNT_SET, c);

		debug_puts("Enhance mode enabled\n");
	} else {

		sp_write_reg_and(TX_P0, SP_TX_SYS_CTRL4_REG, ~ENHANCED_MODE);
		sp_tx_aux_dpcdread_bytes(0x00, 0x01,
			DPCD_LANE_COUNT_SET, 1, &c);
		c &= ~ENHANCED_FRAME_EN;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01,
			DPCD_LANE_COUNT_SET, c);

		debug_puts("Enhance mode disabled\n");
	}
}
uint sp_tx_link_err_check(void)
{
	uint errl = 0, errh = 0;
	unchar bytebuf[2];

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x10, 2, bytebuf);
	delay_ms(5);
	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x10, 2, bytebuf);
	errh = bytebuf[1];

	if (errh & 0x80) {
		errl = bytebuf[0];
		errh = (errh & 0x7f) << 8;
		errl = errh + errl;
	}

	debug_printf(" Err of Lane = %d\n", errl);
	return errl;
}
void slimport_link_training(void)
{
	unchar temp_value, return_value, c;
	return_value = 1;
	debug_printf("sp_tx_LT_state : %x\n", (int)sp_tx_LT_state);
	switch(sp_tx_LT_state) {
		case LT_INIT:
			slimport_block_power_ctrl(SP_TX_PWR_VIDEO, SP_POWER_ON);
			sp_tx_video_mute(1);
			sp_tx_enable_video_input(0);
			sp_tx_LT_state++;
			sp_tx_send_message(
				(g_hdmi_dvi_status == HDMI_MODE)
				? MSG_INPUT_HDMI : MSG_INPUT_DVI);
		case LT_WAIT_PLL_LOCK:
			if (!sp_tx_get_pll_lock_status()) {
				//pll reset when plll not lock. by span 20130217
				sp_read_reg(TX_P0, SP_TX_PLL_CTRL_REG, &temp_value);
				temp_value |= PLL_RST;
				sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, temp_value);
				temp_value &=~PLL_RST;
				sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, temp_value);
				debug_puts("PLL not lock!");
			}else
				sp_tx_LT_state = LT_CHECK_LINK_BW;
			SP_BREAK(LT_WAIT_PLL_LOCK, sp_tx_LT_state);
		case LT_CHECK_LINK_BW:
			sp_tx_get_rx_bw(&temp_value);
			if(temp_value < g_changed_bandwidth){
				debug_puts("****Over bandwidth****\n");
				g_changed_bandwidth = temp_value;
			}
			else
				sp_tx_LT_state++;
		case LT_START:
			if(sp_tx_test_lt) {
				sp_tx_test_lt = 0;
				g_changed_bandwidth = sp_tx_test_bw;
				sp_write_reg_and(TX_P2, SP_TX_VID_CTRL2_REG, 0x8f);
			}

			/* power on main link before link training */
			sp_write_reg_and(TX_P0, SP_TX_ANALOG_PD_REG, ~CH0_PD);
			sp_tx_config_ssc(SSC_DEP_4000PPM);
			sp_tx_set_link_bw(g_changed_bandwidth);
			sp_tx_enhancemode_set();

			/* judge downstream DP version  and set downstream sink to D0 (normal operation mode) */
			sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x00, 0x01, &c);
			sp_tx_aux_dpcdread_bytes(0x00, 0x06, 0x00, 0x01, &temp_value);
			if (c >= 0x12) { /* DP 1.2 and above */
				temp_value &= 0xf8;
			} else { /* DP 1.1 or 1.0 */
				temp_value &= 0xfc;
			}
			temp_value |= 0x01;
			sp_tx_aux_dpcdwrite_byte(0x00, 0x06, 0x00, temp_value);

			/*serdes reset 20140610*/
			sp_read_reg(TX_P2, RST_CTRL2, &c);
			sp_write_reg(TX_P2, RST_CTRL2, c|SERDES_FIFO_RST);
			msleep(20);
			sp_write_reg(TX_P2, RST_CTRL2, c);
			debug_puts("serdes reset\n");

			sp_write_reg_or(TX_P2, SP_INT_MASK, 0X20); /* hardware interrupt mask enable */

			#ifdef DEMO_4K_2K
			if (down_sample_en)
				sp_write_reg(TX_P0, SP_TX_LT_SET_REG, 0x09);
			else
			#endif
			sp_write_reg(TX_P0, SP_TX_LT_SET_REG, 0x0);

			sp_write_reg(TX_P0, LT_CTRL, SP_TX_LT_EN);
			sp_tx_LT_state = LT_WAITTING_FINISH;
			//There is no break;
		case LT_WAITTING_FINISH:
			/*here : waitting interrupt to change training state.*/
			break;
		case LT_ERROR:
			redo_cur_system_state();
			sp_tx_LT_state = LT_INIT;
			break;
		case LT_FINISH:
			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x02, 1, &temp_value);
			if ((temp_value&0x07) == 0x07) {//by span 20130217. for one lane case.
				/* if there is link error, adjust pre-emphsis to check error again.
				If there is no error,keep the setting, otherwise use 400mv0db */
				if(!sp_tx_test_lt) {
				if (sp_tx_link_err_check()) {
					sp_read_reg(TX_P0, SP_TX_LT_SET_REG, &temp_value);
					if(!(temp_value & MAX_PRE_REACH)){
						sp_write_reg(TX_P0, SP_TX_LT_SET_REG, (temp_value + 0x08));//increase one pre-level
						//if error still exist, return to the link traing value
						if (sp_tx_link_err_check())
							sp_write_reg(TX_P0, SP_TX_LT_SET_REG, temp_value);
					}
				}
				sp_write_reg(TX_P0, SP_TX_LT_SET_REG, 0x02);
				sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &temp_value);
				if (temp_value == g_changed_bandwidth){
					debug_printf("LT succeed, bw: %.2x ", (uint) temp_value);
					debug_printf("Lane0 Set: %.2x\n", (uint) __read_reg(TX_P0, SP_TX_LT_SET_REG));
					sp_tx_LT_state = LT_INIT;
					goto_next_system_state();
				}else {
					debug_printf("bw cur:%.2x, per:%.2x \n", (uint)temp_value, (uint)g_changed_bandwidth);
					sp_tx_LT_state = LT_ERROR;
				}
				} else {
					sp_tx_LT_state = LT_INIT;
					goto_next_system_state();
				}
			}else {
				debug_printf("LANE0 Status error: %.2x\n", (uint)(temp_value&0x07));
				sp_tx_LT_state = LT_ERROR;
			}
			break;
		default:
			break;
	}

}
/******************End Link training process********************/
/******************Start Output video process********************/
void sp_tx_set_colorspace(void)
{
	unchar c;
	unchar color_space;

	if (down_sample_en) {
		sp_read_reg(RX_P1, HDMI_RX_AVI_DATA00_REG, &color_space);
		color_space &= 0x60;
		if (color_space == 0x20) {
			pr_info("%s %s : YCbCr4:2:2 ---> PASS THROUGH.\n", LOG_TAG, __func__);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL6_REG, 0x00);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, 0x00);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, 0x11);
		} else if (color_space == 0x40) {
			pr_info("%s %s : YCbCr4:4:4 ---> YCbCr4:2:2\n", LOG_TAG, __func__);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL6_REG, 0x41);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, 0x00);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, 0x12);
		} else if (color_space == 0x00) {
			pr_info("%s %s : RGB4:4:4 ---> YCbCr4:2:2\n", LOG_TAG, __func__);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL6_REG, 0x41);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, 0x83);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, 0x10);
		}
	} else {
		switch (sp_tx_rx_type) {
		case DWN_STRM_IS_VGA_9832:
		case DWN_STRM_IS_ANALOG:
		case DWN_STRM_IS_DIGITAL:
			sp_read_reg(TX_P2, SP_TX_VID_CTRL2_REG, &color_space);
			if ((color_space & 0x03) == 0x01) { /* YCBCR422 */
				sp_write_reg_or(TX_P2, SP_TX_VID_CTRL5_REG, RANGE_Y2R | CSPACE_Y2R);  /* YUV->RGB */

				sp_read_reg(RX_P1, (HDMI_RX_AVI_DATA00_REG + 3), &c);
				/* vic for BT709 */
				if ((c == 0x04) || (c == 0x05) || (c == 0x10) ||
					(c == 0x13) || (c == 0x14) || (c == 0x1F) ||
					(c == 0x20) || (c == 0x21) || (c == 0x22) ||
					(c == 0x27) || (c == 0x28) || (c == 0x29) ||
					(c == 0x2E) || (c == 0x2F) || (c == 0x3C) ||
					(c == 0x3D) || (c == 0x3E) || (c == 0x3F) ||
					(c == 0x40))
					sp_write_reg_or(TX_P2, SP_TX_VID_CTRL5_REG, CSC_STD_SEL);
				else
					sp_write_reg_and(TX_P2, SP_TX_VID_CTRL5_REG, ~CSC_STD_SEL);

				sp_write_reg_or(TX_P2, SP_TX_VID_CTRL6_REG, VIDEO_PROCESS_EN | UP_SAMPLE); /* up sample */
			} else if ((color_space & 0x03) == 0x02) { /* YCBCR444 */

				sp_write_reg_or(TX_P2, SP_TX_VID_CTRL5_REG, RANGE_Y2R | CSPACE_Y2R);  /* YUV->RGB */

				sp_read_reg(RX_P1, (HDMI_RX_AVI_DATA00_REG + 3), &c);
				/* vic for BT709 */
				if ((c == 0x04) || (c == 0x05) || (c == 0x10) ||
					(c == 0x13) || (c == 0x14) || (c == 0x1F) ||
					(c == 0x20) || (c == 0x21) || (c == 0x22) ||
					(c == 0x27) || (c == 0x28) || (c == 0x29) ||
					(c == 0x2E) || (c == 0x2F) || (c == 0x3C) ||
					(c == 0x3D) || (c == 0x3E) || (c == 0x3F) ||
					(c == 0x40))
					sp_write_reg_or(TX_P2, SP_TX_VID_CTRL5_REG, CSC_STD_SEL);
				else
					sp_write_reg_and(TX_P2, SP_TX_VID_CTRL5_REG, ~CSC_STD_SEL);

				sp_write_reg_or_and(TX_P2, SP_TX_VID_CTRL6_REG, VIDEO_PROCESS_EN, ~UP_SAMPLE);
			} else if ((color_space & 0x03) == 0x00)  { /* RGB */
				sp_write_reg_and(TX_P2, SP_TX_VID_CTRL5_REG, (~RANGE_Y2R) & (~CSPACE_Y2R) & (~CSC_STD_SEL));
				sp_write_reg_and(TX_P2, SP_TX_VID_CTRL6_REG, (~VIDEO_PROCESS_EN) & (~UP_SAMPLE));
			}
			break;
		case DWN_STRM_IS_HDMI_7730:
			sp_write_reg(TX_P2, SP_TX_VID_CTRL6_REG, 0x00);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, 0x00);
			break;
		default:
			break;
		}
	}
}
void sp_tx_avi_setup(void)
{
	unchar c;
	int i;
	for (i = 0; i < 13; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_AVI_DATA00_REG + i), &c);
		sp_tx_packet_avi.AVI_data[i] = c;
	}

	if (down_sample_en) {
		sp_tx_packet_avi.AVI_data[0] = (sp_tx_packet_avi.AVI_data[0] & 0x9f) | 0x20;
	} else {
		switch (sp_tx_rx_type) {
		case DWN_STRM_IS_VGA_9832:
		case DWN_STRM_IS_ANALOG:
		case DWN_STRM_IS_DIGITAL:
			sp_tx_packet_avi.AVI_data[0] &= ~0x60;
			break;
		case DWN_STRM_IS_HDMI_7730:
			break;
		default:
			break;
		}
	}
}
static void sp_tx_load_packet(enum PACKETS_TYPE type)
{
	int i;
	unchar c;

	switch (type) {
	case AVI_PACKETS:
		sp_write_reg(TX_P2, SP_TX_AVI_TYPE, 0x82);
		sp_write_reg(TX_P2, SP_TX_AVI_VER, 0x02);
		sp_write_reg(TX_P2, SP_TX_AVI_LEN, 0x0d);

		for (i = 0; i < 13; i++) {
			sp_write_reg(TX_P2, SP_TX_AVI_DB0 + i,
					sp_tx_packet_avi.AVI_data[i]);
		}

		break;

	case SPD_PACKETS:
		sp_write_reg(TX_P2, SP_TX_SPD_TYPE, 0x83);
		sp_write_reg(TX_P2, SP_TX_SPD_VER, 0x01);
		sp_write_reg(TX_P2, SP_TX_SPD_LEN, 0x19);

		for (i = 0; i < 25; i++) {
			sp_write_reg(TX_P2, SP_TX_SPD_DB0 + i,
					sp_tx_packet_spd.SPD_data[i]);
		}

		break;

	case VSI_PACKETS:
		sp_write_reg(TX_P2, SP_TX_MPEG_TYPE, 0x81);
		sp_write_reg(TX_P2, SP_TX_MPEG_VER, 0x01);
		sp_read_reg(RX_P1, HDMI_RX_MPEG_LEN_REG, &c);
		sp_write_reg(TX_P2, SP_TX_MPEG_LEN, c);

		for (i = 0; i < 10; i++) {
			sp_write_reg(TX_P2, SP_TX_MPEG_DB0 + i,
					sp_tx_packet_mpeg.MPEG_data[i]);
		}

		break;
	case MPEG_PACKETS:
		sp_write_reg(TX_P2, SP_TX_MPEG_TYPE, 0x85);
		sp_write_reg(TX_P2, SP_TX_MPEG_VER, 0x01);
		sp_write_reg(TX_P2, SP_TX_MPEG_LEN, 0x0d);

		for (i = 0; i < 10; i++) {
			sp_write_reg(TX_P2, SP_TX_MPEG_DB0 + i,
					sp_tx_packet_mpeg.MPEG_data[i]);
		}

		break;
	case AUDIF_PACKETS:
		sp_write_reg(TX_P2, SP_TX_AUD_TYPE, 0x84);
		sp_write_reg(TX_P2, SP_TX_AUD_VER, 0x01);
		sp_write_reg(TX_P2, SP_TX_AUD_LEN, 0x0a);
		for (i = 0; i < 10; i++) {
			sp_write_reg(TX_P2, SP_TX_AUD_DB0 + i,
					sp_tx_audioinfoframe.pb_byte[i]);
		}

		break;

	default:
		break;
	}
}
void sp_tx_config_packets(enum PACKETS_TYPE bType)
{
	switch (bType) {
	case AVI_PACKETS:
		sp_write_reg_and(TX_P0, SP_TX_PKT_EN_REG, ~AVI_IF_EN);
		sp_tx_load_packet(AVI_PACKETS);
		sp_write_reg_or(TX_P0, SP_TX_PKT_EN_REG, AVI_IF_UD | AVI_IF_EN);
		break;
	case SPD_PACKETS:
		sp_write_reg_and(TX_P0, SP_TX_PKT_EN_REG, ~SPD_IF_EN);
		sp_tx_load_packet(SPD_PACKETS);
		sp_write_reg_or(TX_P0, SP_TX_PKT_EN_REG, SPD_IF_UD | SPD_IF_EN);
		break;
	case VSI_PACKETS:
		sp_write_reg_and(TX_P0, SP_TX_PKT_EN_REG,  ~MPEG_IF_EN);
		sp_tx_load_packet(VSI_PACKETS);
		sp_write_reg_or(TX_P0, SP_TX_PKT_EN_REG, MPEG_IF_UD | MPEG_IF_EN);
		break;
	case MPEG_PACKETS:
		sp_write_reg_and(TX_P0, SP_TX_PKT_EN_REG, ~MPEG_IF_EN);
		sp_tx_load_packet(MPEG_PACKETS);
		sp_write_reg_or(TX_P0, SP_TX_PKT_EN_REG, MPEG_IF_UD | MPEG_IF_EN);
		break;
	case AUDIF_PACKETS:
		sp_write_reg_and(TX_P0, SP_TX_PKT_EN_REG, ~AUD_IF_EN);
		sp_tx_load_packet(AUDIF_PACKETS);
		sp_write_reg_or(TX_P0, SP_TX_PKT_EN_REG, AUD_IF_UP | AUD_IF_EN);
		break;
	default:
		break;
	}

}
void slimport_config_video_output(void)
{
	unchar temp_value;
#ifdef CONFIG_AMAZON_METRICS_LOG
	char buf[SLIMPORT_METRICS_STR_LEN] = {0};
#endif

	switch(sp_tx_vo_state){
		default:
		case VO_WAIT_VIDEO_STABLE:
			/* HDMI RX video */
			sp_read_reg(RX_P0, HDMI_RX_SYS_STATUS_REG, &temp_value);
			/* Input video stable */
			if ((temp_value & (TMDS_DE_DET | TMDS_CLOCK_DET)) ==
				(TMDS_DE_DET | TMDS_CLOCK_DET)) {
				sp_tx_enable_video_input(0);
				sp_tx_avi_setup();
				sp_tx_config_packets(AVI_PACKETS);
				sp_tx_enable_video_input(1);
				sp_tx_vo_state = VO_WAIT_TX_VIDEO_STABLE;
			} else {
				/* about 5 seconds */
				if (counter_unstable_video++ > 50) {
					pr_err("Reset due HDMI input video not stable errors\n");
					sp_system_reset();
					return;
				}
				pr_err("HDMI input video not stable [%d]!\n",
					counter_unstable_video);
			}
			SP_BREAK(VO_WAIT_VIDEO_STABLE, sp_tx_vo_state);

		case VO_WAIT_TX_VIDEO_STABLE:
			/* DP TX video */
			sp_read_reg(TX_P0, SP_TX_SYS_CTRL2_REG, &temp_value);
			sp_write_reg(TX_P0, SP_TX_SYS_CTRL2_REG, temp_value);
			sp_read_reg(TX_P0, SP_TX_SYS_CTRL2_REG, &temp_value);
			if(temp_value & CHA_STA) {
				debug_puts("Stream clock not stable!\n");
			}else {
				sp_read_reg(TX_P0, SP_TX_SYS_CTRL3_REG, &temp_value);
				sp_write_reg(TX_P0, SP_TX_SYS_CTRL3_REG, temp_value);
				sp_read_reg(TX_P0, SP_TX_SYS_CTRL3_REG, &temp_value);
				if(!(temp_value & STRM_VALID))
				{
					debug_puts("video stream not valid!\n");
				} else
					sp_tx_vo_state++;
			}
			SP_BREAK(VO_WAIT_TX_VIDEO_STABLE, sp_tx_vo_state);
			/*
		case VO_WAIT_PLL_LOCK:
			if (!sp_tx_get_pll_lock_status()) {
				//pll reset when plll not lock. by span 20130217
				sp_read_reg(TX_P0, SP_TX_PLL_CTRL_REG, &temp_value);
				temp_value |= PLL_RST;
				sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, temp_value);
				temp_value &=~PLL_RST;
				sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, temp_value);
				debug_puts("PLL not lock!");
			}else
				sp_tx_vo_state = VO_CHECK_BW;
			SP_BREAK(VO_WAIT_PLL_LOCK, sp_tx_vo_state);
			*/
		case VO_CHECK_VIDEO_INFO:
			temp_value = __read_reg(RX_P0, HDMI_STATUS) & HDMI_MODE;
			if (!sp_tx_bw_lc_sel(sp_tx_pclk_calc())
				&& g_hdmi_dvi_status == temp_value)
				sp_tx_vo_state++;
			else{
				if(g_hdmi_dvi_status != temp_value){
					debug_puts("Different mode of DVI or HDMI mode \n");
					g_hdmi_dvi_status = temp_value;
					if(g_hdmi_dvi_status != HDMI_MODE)
						hdmi_rx_mute_audio(0);
					sp_tx_send_message(
						(g_hdmi_dvi_status == HDMI_MODE)
						? MSG_INPUT_HDMI : MSG_INPUT_DVI);
				}
				sp_tx_set_sys_state(STATE_LINK_TRAINING);
			}
			SP_BREAK(VO_CHECK_VIDEO_INFO, sp_tx_vo_state);
		case VO_FINISH:
			slimport_block_power_ctrl(SP_TX_PWR_AUDIO, SP_POWER_DOWN);
			sp_read_reg(RX_P0, HDMI_STATUS, &temp_value);
#ifdef CONFIG_AMAZON_METRICS_LOG
			snprintf(buf, sizeof(buf),
				"slimport:def:hdmi_mode=%s;DV;1:NR",
				(temp_value & HDMI_MODE ? "true" : "false"));
			log_to_metrics(ANDROID_LOG_INFO, "kernel", buf);
#endif

			if (temp_value & HDMI_MODE) {
				sp_tx_send_message(MSG_INPUT_HDMI);
				debug_puts("HDMI mode: Video is stable.\n");
				//hdmi_rx_set_sys_state(HDMI_AUDIO_CONFIG);
			} else {
				debug_puts("DVI mode: Video is stable.\n");
				sp_tx_send_message(MSG_INPUT_DVI);
				hdmi_rx_mute_audio(0);
			}
			hdmi_rx_mute_video(0);
			sp_tx_lvttl_bit_mapping();
			sp_tx_video_mute(0);
			sp_tx_set_colorspace();
			sp_tx_show_infomation();
			goto_next_system_state();
			break;
		}

}
/******************End Output video process********************/
/******************Start HDCP process********************/
static void sp_tx_hdcp_encryption_disable(void)
{
	sp_write_reg_and(TX_P0, TX_HDCP_CTRL0, ~ENC_EN);
}

static void sp_tx_hdcp_encryption_enable(void)
{
	sp_write_reg_or(TX_P0, TX_HDCP_CTRL0, ENC_EN);
}

static void sp_tx_hw_hdcp_enable(void)
{
	unchar c;

	sp_write_reg_and(TX_P0, TX_HDCP_CTRL0, (~ENC_EN) & (~HARD_AUTH_EN));
	sp_write_reg_or(TX_P0, TX_HDCP_CTRL0, HARD_AUTH_EN | BKSV_SRM_PASS | KSVLIST_VLD | ENC_EN);
	sp_read_reg(TX_P0, TX_HDCP_CTRL0, &c);
	debug_printf("TX_HDCP_CTRL0 = %.2x\n", (uint)c);
	sp_write_reg(TX_P0, SP_TX_WAIT_R0_TIME, 0xb0);
	sp_write_reg(TX_P0, SP_TX_WAIT_KSVR_TIME, 0xc8);

	//sp_write_reg(TX_P2, SP_COMMON_INT_MASK2, 0xfc);
	debug_puts("Hardware HDCP is enabled.\n");

}

static bool sp_tx_get_ds_video_status(void)
{
	unchar c;
	sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x27, 1, &c);
	debug_printf("0x00527 = %.2x.\n", (uint)c);
	if (c & 0x01)
		return 1;
	else
		return 0;
}

void slimport_hdcp_repeater_reauth(void)
{
	unchar c, hdcp_ctrl, hdcp_status, hdcp_pass;

	if (!(sp_repeater_mode && (sp_tx_system_state > STATE_HDCP_AUTH)
		&& (sp_tx_rx_type != DWN_STRM_IS_ANALOG)
		&& (sp_tx_rx_type != DWN_STRM_IS_VGA_9832)))
		return;

	sp_read_reg(RX_P1, HDMI_RX_HDCP_STATUS_REG, &c);
	if (c & UPSTREAM_HDMI_HDCP) {
		if (sp_tx_system_state < STATE_PLAY_BACK)
			msleep(100);

		sp_read_reg(TX_P0, TX_HDCP_CTRL0, &hdcp_ctrl);
		if ((hdcp_ctrl & HARD_AUTH_EN) != HARD_AUTH_EN) {
			debug_puts("Repeater Mode: Enable HW HDCP!\n");
			slimport_block_power_ctrl(SP_TX_PWR_HDCP,
						SP_POWER_ON);
			sp_tx_hw_hdcp_enable();
		} else {
			sp_read_reg(TX_P0, SP_TX_HDCP_STATUS, &hdcp_status);
			hdcp_status = (hdcp_status & SP_TX_HDCP_AUTH_PASS);
			hdcp_pass = (hdcp_status == SP_TX_HDCP_AUTH_PASS);
			if (auth_done && (!hdcp_pass ||
			   (ds_auth_done && !hdcp_repeater_passed))) {
				pr_err("3618: 0x70:0x00 = 0x%.2x, >>> HDCP repeater failure case!\n",
					(uint)hdcp_status);
#ifdef CONFIG_AMAZON_METRICS_LOG
				log_to_metrics(ANDROID_LOG_INFO, "kernel",
				    "slimport:def:hdcp_failure=true;DV;1:NR");
#endif
				sp_system_reset();
			}
		}
	}
}

void slimport_hdcp_process(void)
{
	//unchar c;
	static unchar ds_vid_stb_cntr = 0, HDCP_fail_count = 0;

	switch(HDCP_state) {
	case HDCP_CAPABLE_CHECK:
		ds_vid_stb_cntr = 0;
		HDCP_fail_count = 0;
		HDCP_state = HDCP_WAITTING_VID_STB;

		if (!external_block_en) {
			if ((sp_tx_rx_type == DWN_STRM_IS_ANALOG)
				|| (sp_tx_rx_type == DWN_STRM_IS_VGA_9832)
				|| (slimport_hdcp_cap_check() == 0)
				|| !get_slimport_hdcp_status())
				HDCP_state = HDCP_NOT_SUPPORT;
		}
		debug_printf("Downstream HDCP_CAPABLE = %s\n",
			(HDCP_state == HDCP_NOT_SUPPORT ? "False" : "True"));

		SP_BREAK(HDCP_CAPABLE_CHECK, HDCP_state);
	case HDCP_WAITTING_VID_STB:
		/*In case ANX7730 can not get ready video*/
		if(sp_tx_rx_type == DWN_STRM_IS_HDMI_7730) {
			if (!sp_tx_get_ds_video_status()) {
				if (ds_vid_stb_cntr >= 50) {
					sp_system_reset();
					ds_vid_stb_cntr = 0;
				} else {
					ds_vid_stb_cntr++;
					delay_ms(10);
				}
				#ifdef SLIMPORT_DRV_DEBUG
				debug_puts("downstream video not stable\n");//for debug
				#endif
			} else {
				ds_vid_stb_cntr = 0;
				HDCP_state = HDCP_HW_ENABLE;
			}
		}else {
			HDCP_state = HDCP_HW_ENABLE;
		}
		SP_BREAK(HDCP_WAITTING_VID_STB, HDCP_state);
	case HDCP_HW_ENABLE:
		sp_tx_video_mute(1);
		//sp_tx_clean_hdcp_status();
		slimport_block_power_ctrl(SP_TX_PWR_HDCP, SP_POWER_ON);
		sp_write_reg(TX_P2, SP_COMMON_INT_MASK2, 0x01);
		//sp_tx_video_mute(0);
		//sp_tx_aux_polling_disable();
		//sp_tx_clean_hdcp_status();
		delay_ms(50); //removed by span. 20130217
		//disable auto polling during hdcp.
		sp_tx_hw_hdcp_enable();
		HDCP_state = HDCP_WAITTING_FINISH;
	case HDCP_WAITTING_FINISH:
		break;
	case HDCP_FINISH:
		sp_tx_hdcp_encryption_enable();
		hdmi_rx_mute_video(0);
		sp_tx_video_mute(0);
		//enable auto polling after hdcp.
		//sp_tx_aux_polling_enable();
		goto_next_system_state();
		HDCP_state = HDCP_CAPABLE_CHECK;
		debug_puts("@@@@@@@hdcp_auth_pass@@@@@@\n");
		break;
	case HDCP_FAILE:
		if(HDCP_fail_count > 5) {
			sp_system_reset();
			HDCP_state = HDCP_CAPABLE_CHECK;
			HDCP_fail_count = 0;
#ifdef CONFIG_AMAZON_METRICS_LOG
			log_to_metrics(ANDROID_LOG_INFO, "kernel",
					"slimport:def:hdcp_auth_failed=true;DV;1:NR");
#endif
			debug_puts("*********hdcp_auth_failed*********\n");
		}else {
			HDCP_fail_count++;
			HDCP_state = HDCP_WAITTING_VID_STB;
		}
		break;
	default:
	case HDCP_NOT_SUPPORT:
		debug_puts("Sink is not capable HDCP");
		slimport_block_power_ctrl(SP_TX_PWR_HDCP, SP_POWER_DOWN);
		sp_tx_video_mute(0);
		//sp_tx_aux_polling_enable();
		goto_next_system_state();
		HDCP_state = HDCP_CAPABLE_CHECK;
		break;
	}
}
/******************End HDCP process********************/
/******************Start Audio process********************/
static void sp_tx_audioinfoframe_setup(void)
{
	int i;
	unchar c;
	sp_read_reg(RX_P1, HDMI_RX_AUDIO_TYPE_REG, &c);
	sp_tx_audioinfoframe.type = c;
	sp_read_reg(RX_P1, HDMI_RX_AUDIO_VER_REG, &c);
	sp_tx_audioinfoframe.version = c;
	sp_read_reg(RX_P1, HDMI_RX_AUDIO_LEN_REG, &c);
	sp_tx_audioinfoframe.length = c;
	for (i = 0; i < 11; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_AUDIO_DATA00_REG + i), &c);
		sp_tx_audioinfoframe.pb_byte[i] = c;
	}

}
static void info_ANX7730_AUIF_changed(void)
{
	unchar temp, count;
	unchar pBuf[3] = {0x01,0xd1,0x02};
	msleep(20);
	if(sp_tx_rx_type == DWN_STRM_IS_HDMI_7730) {//assuming it is anx7730
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x23, 1, &temp);
		if(temp < 0x94){
			count = 3;
			do{
				 delay_ms(20);
				if (sp_tx_aux_dpcdwrite_bytes(0x00, 0x05,0xf0,3,pBuf) == AUX_OK)
					break;
				if (!count)
					debug_puts("dpcd write error\n");
			}while(--count);
		}
	}
}
static void sp_tx_enable_audio_output(unchar benable)
{
	unchar c;

	sp_read_reg(TX_P0, SP_TX_AUD_CTRL, &c);
	if (benable) {

		if(c & AUD_EN){//if it has been enabled, disable first.
			c &= ~AUD_EN;
			sp_write_reg(TX_P0, SP_TX_AUD_CTRL, c);
		}
		sp_tx_audioinfoframe_setup();
		sp_tx_config_packets(AUDIF_PACKETS);

		//for audio multi-ch
		info_ANX7730_AUIF_changed();
		//end audio multi-ch
		c |= AUD_EN;
		sp_write_reg(TX_P0, SP_TX_AUD_CTRL, c);


	} else {
		c &= ~AUD_EN;
		sp_write_reg(TX_P0, SP_TX_AUD_CTRL, c);
		sp_write_reg_and(TX_P0, SP_TX_PKT_EN_REG, ~AUD_IF_EN);
	}

}
static void sp_tx_config_audio(void)
{
	unchar c;
	int i;
	ulong M_AUD, LS_Clk = 0;
	ulong AUD_Freq = 0;
	debug_puts("**Config audio **\n");
	slimport_block_power_ctrl(SP_TX_PWR_AUDIO, SP_POWER_ON);
	sp_read_reg(RX_P0, 0xCA, &c);

	switch (c & 0x0f) {
	case 0x00:
		AUD_Freq = 44.1;
		break;
	case 0x02:
		AUD_Freq = 48;
		break;
	case 0x03:
		AUD_Freq = 32;
		break;
	case 0x08:
		AUD_Freq = 88.2;
		break;
	case 0x0a:
		AUD_Freq = 96;
		break;
	case 0x0c:
		AUD_Freq = 176.4;
		break;
	case 0x0e:
		AUD_Freq = 192;
		break;
	default:
		break;
	}


	switch (sp_tx_get_link_bw()) {
	case LINK_1P62G:
		LS_Clk = 162000;
		break;
	case LINK_2P7G:
		LS_Clk = 270000;
		break;
	case LINK_5P4G:
		LS_Clk = 540000;
		break;
	default:
		break;
	}

	debug_printf("AUD_Freq = %ld , LS_CLK = %ld\n", AUD_Freq, LS_Clk);

	M_AUD = ((512 * AUD_Freq) / LS_Clk) * 32768;
	M_AUD = M_AUD + 0x05;
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL4, (M_AUD & 0xff));
	M_AUD = M_AUD >> 8;
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL5, (M_AUD & 0xff));
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL6, 0x00);

	sp_write_reg_and(TX_P1, SP_TX_AUD_INTERFACE_CTRL0, ~AUD_INTERFACE_DISABLE);

	sp_write_reg_or(TX_P1, SP_TX_AUD_INTERFACE_CTRL2, M_AUD_ADJUST_ST);

	sp_read_reg(RX_P0, HDMI_STATUS, &c);
	if (c & HDMI_AUD_LAYOUT)
		sp_write_reg_or(TX_P2, SP_TX_AUD_CH_NUM_REG5,  CH_NUM_8 | AUD_LAYOUT);
	 else
		sp_write_reg_and(TX_P2, SP_TX_AUD_CH_NUM_REG5, (~CH_NUM_8) & (~AUD_LAYOUT));

	/* transfer audio chaneel status from HDMI Rx to Slinmport Tx */
	for (i = 0; i < 5; i++) {
		sp_read_reg(RX_P0, (HDMI_RX_AUD_IN_CH_STATUS1_REG + i), &c);
		sp_write_reg(TX_P2, (SP_TX_AUD_CH_STATUS_REG1 + i), c);
	}

	/* enable audio */
	sp_tx_enable_audio_output(1);
	/*
	sp_read_reg(TX_P2, SP_COMMON_INT_MASK1, &c);
	c |= 0x04;
	sp_write_reg(TX_P2, SP_COMMON_INT_MASK1, c);
	*/
	/* To get some logs */
	sp_read_reg(TX_P2, SP_TX_SPDID_AUD_STATUS, &c);
	debug_printf("3618: 0x72:0x38 = 0x%.2x\n", (uint)c);
	if (sp_tx_rx_type == DWN_STRM_IS_HDMI_7730) {
		i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x72,
			SP_RX_7730_SPDIF_AUD_STATUS, &c);
		debug_printf("7730: 0x72:0x38 = 0x%.2x\n", (uint)c);
		i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x72,
			SP_RX_7730_SPDIF_AUD_CTRL, &c);
		debug_printf("7730: 0x72:0x36 = 0x%.2x\n", (uint)c);
		i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x7a,
			SP_RX_7730_HDMI_AUD_CTRL, &c);
		debug_printf("7730: 0x7a:0x09 = 0x%.2x\n", (uint)c);
		i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x7a,
			SP_RX_7730_HDMI_INFO_PACKET_CTRL1, &c);
		debug_printf("7730: 0x7a:0x70 = 0x%.2x\n", (uint)c);
		i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x7a,
			SP_RX_7730_HDMI_INFO_PACKET_CTRL2, &c);
		debug_printf("7730: 0x7a:0x71 = 0x%.2x\n", (uint)c);
	}

}

void slimport_config_audio_output(void)
{
	static unchar count = 0;
	hdmi_rx_mute_video(0);
	switch(sp_tx_ao_state){
		default:
		case AO_INIT:
		case AO_CTS_RCV_INT:
		case AO_AUDIO_RCV_INT:
			if (!(__read_reg(RX_P0, HDMI_STATUS) & HDMI_MODE)) {
				sp_tx_ao_state = AO_INIT;
				goto_next_system_state();
			}
			break;
		case AO_RCV_INT_FINISH:
			if(count++ > 2)
				sp_tx_ao_state = AO_OUTPUT;
			else
				sp_tx_ao_state = AO_INIT;
			SP_BREAK(AO_INIT, sp_tx_ao_state);
		case AO_OUTPUT:
			count = 0;
			sp_tx_ao_state = AO_INIT;
			hdmi_rx_mute_audio(0);
			sp_tx_config_audio();
			goto_next_system_state();
			break;
	}

}
/******************End Audio process********************/
void slimport_initialization(void)
{
	#ifdef ENABLE_READ_EDID
	g_read_edid_flag = 0;
	#endif
	slimport_block_power_ctrl(SP_TX_PWR_REG, SP_POWER_ON);
	slimport_block_power_ctrl(SP_TX_PWR_TOTAL, SP_POWER_ON);
	/*Driver Version*/
	sp_write_reg(TX_P1, FW_VER_REG, FW_VERSION);
	/*disable OCM*/
	sp_write_reg_or(TX_P1, OCM_REG3, OCM_RST);
	vbus_power_ctrl(1);
	hdmi_rx_initialization();
	sp_tx_initialization();
	msleep(100);
	sp_tx_aux_polling_enable();
	goto_next_system_state();
}
void slimport_cable_monitor(void)
{
	unchar cur_cable_type;

	if (sp_repeater_mode == false) {
		if (sp_tx_system_state_bak != sp_tx_system_state
			&& HDCP_state != HDCP_WAITTING_FINISH
			&& sp_tx_LT_state != LT_WAITTING_FINISH) {
			if (sp_tx_cur_states() > STATE_SINK_CONNECTION) {
				cur_cable_type = sp_tx_get_cable_type(GETTED_CABLE_TYPE, 0);
				if (cur_cable_type != sp_tx_rx_type) { /* cable changed */
					sp_system_reset();
					return;
				}
			}
			sp_tx_system_state_bak = sp_tx_system_state;
		}
	} else {
		if (sp_tx_system_state_bak != sp_tx_system_state
			&& sp_tx_LT_state != LT_WAITTING_FINISH) {
			if (sp_tx_cur_states() > STATE_SINK_CONNECTION) {
				cur_cable_type = sp_tx_get_cable_type(GETTED_CABLE_TYPE, 0);
				if (cur_cable_type != sp_tx_rx_type) { /* cable changed */
					sp_system_reset();
					return;
				}
			}
			sp_tx_system_state_bak = sp_tx_system_state;
		}
	}
}

static void slimport_playback_process(void)
{
	unchar reg_value;

	if (ds_hpd_lost) {
		ds_hpd_lost = false;

		if (!is_cable_detected()) {
			sp_tx_set_sys_state(STATE_WAITTING_CABLE_PLUG);
			return;
		}

		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x18, 1, &reg_value);
		if (!(reg_value & 0x01)) {
			debug_puts("2nd: Downstream HDMI is unplugged!\n");
			vbus_power_ctrl(0);
			reg_hardware_reset();
			sp_tx_set_sys_state(STATE_SP_INITIALIZED);
			return;
		}
	}
}

void hdcp_external_ctrl_flag_monitor(void)
{
	static unchar cur_flag;
	if ((sp_tx_rx_type == DWN_STRM_IS_ANALOG)
	|| (sp_tx_rx_type == DWN_STRM_IS_VGA_9832)) {
		if (external_block_en != cur_flag) {
			cur_flag = external_block_en;
			system_state_change_with_case(STATE_HDCP_AUTH);
		}

	}
}
void slimport_state_process(void)
{

	switch(sp_tx_system_state) {
	case STATE_INIT:
		goto_next_system_state();
	case STATE_WAITTING_CABLE_PLUG:
		hdmi_hdcp_auth_complete();
		slimport_waitting_cable_plug_process();
		SP_BREAK(STATE_WAITTING_CABLE_PLUG, sp_tx_system_state);
	case STATE_SP_INITIALIZED:
		slimport_initialization();
		SP_BREAK(STATE_SP_INITIALIZED, sp_tx_system_state);
	case STATE_SINK_CONNECTION:
		slimport_sink_connection();
		SP_BREAK(STATE_SINK_CONNECTION, sp_tx_system_state);
	#ifdef ENABLE_READ_EDID
	case STATE_PARSE_EDID:
		slimport_edid_process();
		SP_BREAK(STATE_PARSE_EDID, sp_tx_system_state);
	#endif
	case STATE_LINK_TRAINING:
		slimport_link_training();
		SP_BREAK(STATE_LINK_TRAINING, sp_tx_system_state);
	case STATE_VIDEO_OUTPUT:
		slimport_config_video_output();
		SP_BREAK(STATE_VIDEO_OUTPUT, sp_tx_system_state);
	case STATE_HDCP_AUTH:
		if (sp_repeater_mode == false) {
			slimport_hdcp_process();
			#ifdef DBG_DONGLE_AUD
				dump_7730_regs(STATE_HDCP_AUTH);
			#endif
		} else {
			hdcp_repeater_passed = false;
			ds_auth_done = false;
			auth_done = false;
			goto_next_system_state();
		}
		SP_BREAK(STATE_HDCP_AUTH, sp_tx_system_state);
	case STATE_AUDIO_OUTPUT:
		slimport_config_audio_output();
		SP_BREAK(STATE_AUDIO_OUTPUT, sp_tx_system_state);
	case STATE_PLAY_BACK:
		hdmi_hdcp_auth_complete();
		slimport_playback_process();
		SP_BREAK(STATE_PLAY_BACK, sp_tx_system_state);
	default:
		break;
	}

}
/******************Start INT process********************/
void sp_tx_int_rec(void)
{
	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1, &COMMON_INT1);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1, COMMON_INT1);

	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1+1, &COMMON_INT2);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1 +1, COMMON_INT2);

	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1+2, &COMMON_INT3);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1 +2, COMMON_INT3);

	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1+3, &COMMON_INT4);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1 +3, COMMON_INT4);

	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1+6, &COMMON_INT5);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1 +6, COMMON_INT5);

}
void hdmi_rx_int_rec(void)
{
//	 if((hdmi_system_state < HDMI_CLOCK_DET)||(sp_tx_system_state <STATE_CONFIG_HDMI))
	//   return;
	 sp_read_reg(RX_P0,HDMI_RX_INT_STATUS1_REG , &HDMI_RX_INT1);
	 sp_write_reg(RX_P0,HDMI_RX_INT_STATUS1_REG , HDMI_RX_INT1);
	 sp_read_reg(RX_P0,HDMI_RX_INT_STATUS2_REG , &HDMI_RX_INT2);
	 sp_write_reg(RX_P0,HDMI_RX_INT_STATUS2_REG , HDMI_RX_INT2);
	 sp_read_reg(RX_P0,HDMI_RX_INT_STATUS3_REG , &HDMI_RX_INT3);
	 sp_write_reg(RX_P0,HDMI_RX_INT_STATUS3_REG , HDMI_RX_INT3);
	 sp_read_reg(RX_P0,HDMI_RX_INT_STATUS4_REG , &HDMI_RX_INT4);
	 sp_write_reg(RX_P0,HDMI_RX_INT_STATUS4_REG , HDMI_RX_INT4);
	 sp_read_reg(RX_P0,HDMI_RX_INT_STATUS5_REG , &HDMI_RX_INT5);
	 sp_write_reg(RX_P0,HDMI_RX_INT_STATUS5_REG , HDMI_RX_INT5);
	 sp_read_reg(RX_P0,HDMI_RX_INT_STATUS6_REG , &HDMI_RX_INT6);
	 sp_write_reg(RX_P0,HDMI_RX_INT_STATUS6_REG , HDMI_RX_INT6);
	 sp_read_reg(RX_P0,HDMI_RX_INT_STATUS7_REG , &HDMI_RX_INT7);
	 sp_write_reg(RX_P0,HDMI_RX_INT_STATUS7_REG , HDMI_RX_INT7);
}
void slimport_int_rec(void)
{
	sp_tx_int_rec();
	hdmi_rx_int_rec();
	/*
	if(!sp_tx_pd_mode ){
	       sp_tx_int_irq_handler();
		hdmi_rx_int_irq_handler();
	}
	*/
}
/******************End INT process********************/
/******************Start task process********************/

static void sp_tx_pll_changed_int_handler(void)
{
	if (sp_tx_system_state >= STATE_LINK_TRAINING) {
		if (!sp_tx_get_pll_lock_status()) {
			debug_puts("PLL:PLL not lock!\n");
			sp_tx_set_sys_state(STATE_LINK_TRAINING);
		}
	}
}
static void sp_tx_hdcp_link_chk_fail_handler(void)
{
	if (sp_repeater_mode == false) {
		system_state_change_with_case(STATE_HDCP_AUTH);

#ifdef CONFIG_AMAZON_METRICS_LOG
		log_to_metrics(ANDROID_LOG_INFO, "kernel",
			"slimport:def:hdcp_sync_lost=true;DV;1:NR");
#endif
		debug_puts("hdcp_link_chk_fail:HDCP Sync lost!\n");
	}
}
static unchar link_down_check(void)
{
	unchar return_value;
	unchar err_counter[2];
	unchar aux_state;

	return_value= 0;
	debug_puts("link_down_check\n");

	if (!(sp_tx_system_state > STATE_LINK_TRAINING))
		return return_value;

	/* link down flag */
	if (!(__i2c_read_byte(TX_P0, DPCD_204) & 0x01) ||
	   ((__i2c_read_byte(TX_P0, DPCD_202) & (0x01 | 0x04))) != 0x05) {

		if (sp_tx_get_downstream_connection()) {
			sp_tx_set_sys_state(STATE_LINK_TRAINING);
			debug_puts("INT:re-LT request!\n");
		} else {
			vbus_power_ctrl(0);
			reg_hardware_reset();
			return_value = 1;
		}
	} else {
		debug_printf("Lane align %x\n",
			    (uint)__read_reg(TX_P0, DPCD_204));
		debug_printf("Lane clock recovery %x\n",
			    (uint)__read_reg(TX_P0, DPCD_202));
		aux_state = sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x10, 2,
						    err_counter);
		if (aux_state == AUX_OK) {
			/*
			 * 1. Error counter should be valid, by
			 *    checking err_counter[1] bit7
			 * 2. Low 8 bits, err_counter[0] is >10,
			 *    or high 8 bits, err_counter[1] is > 0
			 */
			if (((err_counter[1] & 0x80) != 0x00) &&
			    ((err_counter[0] > 0x0a) ||
			    ((err_counter[1] & 0x7f) > 0x00))) {
				pr_err("ESD: Main link has error!\n");
				vbus_power_ctrl(0);
				reg_hardware_reset();
				sp_tx_set_sys_state(STATE_SP_INITIALIZED);
				return_value = 1;
			}
		}
	}

	return return_value;

}

static void sp_tx_phy_auto_test(void)
{
	unchar b_sw;
	unchar c1;
	unchar bytebuf[16];
	unchar link_bw;

	/*DPCD 0x219 TEST_LINK_RATE*/
	sp_tx_aux_dpcdread_bytes(0x0, 0x02, 0x19, 1, bytebuf);
	debug_printf("DPCD:0x00219 = %.2x\n", (uint)bytebuf[0]);
	switch (bytebuf[0]) {
	case 0x06:
		sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, 0x06);
		debug_puts("test BW= 1.62Gbps\n");
		break;
	case 0x0a:
		sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, 0x0a);
		debug_puts("test BW= 2.7Gbps\n");
		break;
	case 0x14:
		sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, 0x14);
		debug_puts("test BW= 5.4Gbps\n");
		break;
	case 0x19:
		sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, 0x19);
		debug_puts("test BW= 6.75Gbps\n");
		break;
	}

	/*DPCD 0x248 PHY_TEST_PATTERN*/
	sp_tx_aux_dpcdread_bytes(0x0, 0x02, 0x48, 1, bytebuf);
	debug_printf("DPCD:0x00248 = %.2x\n", (uint)bytebuf[0]);
	switch (bytebuf[0]) {
	case 0:
		debug_puts("No test pattern selected\n");
		break;
	case 1:
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x04);
		debug_puts("D10.2 Pattern\n");
		break;
	case 2:
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x08);
		debug_puts("Symbol Error Measurement Count\n");
		break;
	case 3:
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x0c);
		debug_puts("PRBS7 Pattern\n");
		break;
	case 4:
		sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x50, 0xa, bytebuf);
		sp_write_reg(TX_P1, 0x80, bytebuf[0]);
		sp_write_reg(TX_P1, 0x81, bytebuf[1]);
		sp_write_reg(TX_P1, 0x82, bytebuf[2]);
		sp_write_reg(TX_P1, 0x83, bytebuf[3]);
		sp_write_reg(TX_P1, 0x84, bytebuf[4]);
		sp_write_reg(TX_P1, 0x85, bytebuf[5]);
		sp_write_reg(TX_P1, 0x86, bytebuf[6]);
		sp_write_reg(TX_P1, 0x87, bytebuf[7]);
		sp_write_reg(TX_P1, 0x88, bytebuf[8]);
		sp_write_reg(TX_P1, 0x89, bytebuf[9]);
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x30);
		debug_puts("80bit custom pattern transmitted\n");
		break;
	case 5:
		sp_write_reg(TX_P0, 0xA9, 0x00);
		sp_write_reg(TX_P0, 0xAA, 0x01);
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x14);
		debug_puts("HBR2 Compliance Eye Pattern\n");
		break;
	}

	sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x03, 1, bytebuf);
	debug_printf("DPCD:0x00003 = %.2x\n", (uint)bytebuf[0]);
	switch (bytebuf[0] & 0x01) {
	case 0:
		sp_tx_spread_enable(0);
		debug_puts("SSC OFF\n");
		break;
	case 1:
		sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c1);
		switch (c1) {
		case 0x06:
			link_bw = 0x06;
			break;
		case 0x0a:
			link_bw = 0x0a;
			break;
		case 0x14:
			link_bw = 0x14;
			break;
		case 0x19:
			link_bw = 0x19;
			break;
		default:
			link_bw = 0x00;
			break;
		}
		sp_tx_config_ssc(SSC_DEP_5000PPM);
		debug_puts("SSC ON\n");
		break;
	}

	/*get swing and emphasis adjust request*/
	sp_read_reg(TX_P0, 0xA3, &b_sw);

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x06, 1, bytebuf);
	debug_printf("DPCD:0x00206 = %.2x\n", (uint)bytebuf[0]);
	c1 = bytebuf[0] & 0x0f;
	switch (c1) {
	case 0x00:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x00);
		debug_puts("lane0,Swing200mv, emp 0db.\n");
		break;
	case 0x01:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x01);
		debug_puts("lane0,Swing400mv, emp 0db.\n");
		break;
	case 0x02:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x02);
		debug_puts("lane0,Swing600mv, emp 0db.\n");
		break;
	case 0x03:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x03);
		debug_puts("lane0,Swing800mv, emp 0db.\n");
		break;
	case 0x04:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x08);
		debug_puts("lane0,Swing200mv, emp 3.5db.\n");
		break;
	case 0x05:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x09);
		debug_puts("lane0,Swing400mv, emp 3.5db.\n");
		break;
	case 0x06:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x0a);
		debug_puts("lane0,Swing600mv, emp 3.5db.\n");
		break;
	case 0x08:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x0b);
		debug_puts("lane0,Swing200mv, emp 6db.\n");
		break;
	case 0x09:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x10);
		debug_puts("lane0,Swing400mv, emp 6db.\n");
		break;
	case 0x0c:
		sp_write_reg(TX_P0, 0xA3, (b_sw & ~0x1b) | 0x11);
		debug_puts("lane0,Swing200mv, emp 9.5db.\n");
		break;
	default:
		break;
	}
}


static bool sp_tx_sink_int(char int_vector_0, int mask)
{
	char c;

	switch (int_vector_0 & mask) {
	case _BIT0:
		/* downstream HPD changed from 0 to 1 */
		if ((int_vector_0 & 0x01) == 0x01) {
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x18, 1, &c);
			if (c & 0x01)
				debug_puts("Downstream HDMI is plugged!\n");
		}
		break;
	case _BIT1:
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x18, 1, &c);
		if ((c & 0x01) != 0x01) {
			if (sp_tx_system_state > STATE_VIDEO_OUTPUT) {
				debug_puts("Downstream HDMI is unplugged!\n");
				vbus_power_ctrl(0);
				reg_hardware_reset();
				sp_tx_set_sys_state(STATE_SP_INITIALIZED);
			} else {
				ds_hpd_lost = true;
			}
			return false;
		}
		break;
	case _BIT2:
		debug_puts("Here is check: Link is down!\n");
		if ((sp_tx_system_state > STATE_VIDEO_OUTPUT)) {
			debug_puts("Rx specific  IRQ: Link is down!\n");
			return !link_down_check();
		}
		break;
	case _BIT3:
		ds_auth_done = true;
		if ((int_vector_0 & UPSTREAM_HDMI_HDCP) !=
			UPSTREAM_HDMI_HDCP) {
			debug_puts("Downstream HDCP is passed!\n");
			if (sp_repeater_mode) {
				hdcp_repeater_passed = true;
				audio_info_process = true;
				sp_read_reg(TX_P0, TX_HDCP_CTRL0, &c);
				if ((c & ENC_EN) == 0x00) {
					pr_debug("Repeater Mode: Enable HDCP encryption\n");
					sp_tx_hdcp_encryption_enable();
				}
			}
			sp_tx_video_mute(false);
		} else {
			if (sp_tx_system_state >= STATE_HDCP_AUTH) {
				hdcp_repeater_passed = false;
				sp_tx_video_mute(1);
				sp_tx_set_sys_state(STATE_HDCP_AUTH);
				pr_debug("Re-authentication due to downstream HDCP failure!\n");
				sp_tx_clean_hdcp_status();
			}
		}
		break;
	case _BIT4:
		break;
	case _BIT5:
		debug_puts(" Downstream HDCP link integrity check fail!\n");
		if (sp_repeater_mode == false) {
			system_state_change_with_case(STATE_HDCP_AUTH);
			/* 20130217 by span for samsung monitor */
			sp_tx_clean_hdcp_status();
		} else {
			hdcp_repeater_passed = false;
			sp_tx_video_mute(1);
			sp_tx_clean_hdcp_status();
			delay_ms(50);
			sp_write_reg(TX_P0, TX_HDCP_CTRL0, 0x0f);
		}
		debug_puts("IRQ:____________HDCP Sync lost!\n");
		break;
	case _BIT6:
		#ifdef CEC_ENABLE
		cec_states = CEC_DOWNSTREAM_HDMI_READ;
		#endif
		debug_puts("Receive CEC command from downstream done!\n");
		break;
	case _BIT7:
		debug_puts("CEC command transfer to downstream done!\n");
		break;
	default:
		break;
	}

	return true;
}

static void sp_tx_7732_dongle_check(void)
{
	unchar c;
	int i;

	pr_debug("Downstream is 7732\n");
	/* Sense VGA sink presence */
	for (i = 0; i < 3; i++) {
		msleep(500);
		sp_read_reg(TX_P0, DPCD_200, &c);
		debug_printf("0x70:0xB9 = 0x%.2x\n", (uint)c);
		if (c & DS_VGA_SINK_COUNT)
			return;
	}
	sp_system_reset();
}

static void sp_tx_sink_irq_int_handler(void)
{
	unchar c, c1;
	unchar IRQ_Vector;// Int_vector1, Int_vector2;
	unchar Int_vector[2];//by span 20130217
	unchar anx_id[2];
	unchar test_vector;
	unchar temp;

	debug_puts("sp_tx_sink_irq_int_handler\n");
	IRQ_Vector = __read_reg(TX_P0, DPCD_201);
	if(IRQ_Vector != 0)
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, DPCD_SERVICE_IRQ_VECTOR, 1, &IRQ_Vector);
	else
		return;

	/* HDCP IRQ */
	if (IRQ_Vector & CP_IRQ) {
		if (sp_repeater_mode == false) {
			if (HDCP_state > HDCP_WAITTING_FINISH
				|| sp_tx_system_state > STATE_HDCP_AUTH) {
				sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x29, 1, &c1);
				if (c1 & HDCP_SYNC_LOST) {
					system_state_change_with_case(STATE_HDCP_AUTH);
					sp_tx_clean_hdcp_status();
					debug_puts("IRQ:____________HDCP Sync lost!\n");
				}
			}
		} else {
			sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x29, 1, &c1);
			if (c1 & HDCP_SYNC_LOST)
				debug_puts("IRQ:____________HDCP Sync lost!\n");
		}

	}

	/* specific int */
	if (IRQ_Vector & SINK_SPECIFIC_IRQ) {

		//dongle has fast charging detection capability
		if(downstream_charging_status != NO_CHARGING_CAPABLE){
			downstream_charging_status_set();
		}

		if (sp_tx_rx_type == DWN_STRM_IS_HDMI_7730) {
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, DPCD_SPECIFIC_INTERRUPT1,
						 2, Int_vector);
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, DPCD_SPECIFIC_INTERRUPT1,
						 2, Int_vector);

			temp = 0x01;
			do {
				/* Process sink interrupts */
				if (!sp_tx_sink_int(Int_vector[0], temp))
					return;

				temp = (temp << 1);
			}while(temp != 0);


			if ((Int_vector[1] & 0x04) == 0x04) {
				sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x18, 1, &c);
				if ((c & 0x40) == 0x40)
					debug_puts("Downstream HDMI termination is detected!\n");
			}
		} else if (sp_tx_rx_type != DWN_STRM_IS_HDMI_7730) {
			//sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x00, 1, &c);
			c = __read_reg(TX_P0, DPCD_200);
			if (!(c & 0x01)) {
				if (sp_tx_system_state > STATE_SINK_CONNECTION) {
					debug_puts("System power off check!\n");
					sp_tx_aux_dpcdread_bytes(DPCD_00503_H,
						DPCD_00503_M, DPCD_00503_L,
						2, anx_id);
					if ((anx_id[0] == ANX7732_ID_H) &&
					    (anx_id[1] == ANX7732_ID_L)) {
						sp_tx_7732_dongle_check();
					} else {
						sp_system_reset();
					}
				} else {
					if (sp_tx_rx_type == DWN_STRM_IS_VGA_9832)
						sp_tx_send_message(MSG_CLEAR_IRQ);
				}

				if (sp_tx_system_state >= STATE_LINK_TRAINING) {
					link_down_check();
				}
			}
		}
	}

	/* AUTOMATED TEST IRQ */
	if (IRQ_Vector & TEST_IRQ) {
		sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x18, 1, &test_vector);

		if (test_vector & 0x01) {//test link training
			sp_tx_test_lt = 1;

			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x19,1,&c);
			sp_tx_test_bw = c;
			debug_printf(" test_bw = %.2x\n", (uint)sp_tx_test_bw);

			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60,1,&c);
			c = c | TEST_ACK;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60,1, &c);  //xjh modify for link CTS 22

			debug_puts("Set TEST_ACK!\n");
			if (sp_tx_system_state >= STATE_LINK_TRAINING){	//xjh modify for link CTS 22   >
				sp_tx_LT_state = LT_INIT;
				sp_tx_set_sys_state(STATE_LINK_TRAINING);
			}
			debug_puts("IRQ:test-LT request!\n");
		}

		if (test_vector & 0x02) {
			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
			c = c | TEST_ACK;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60, 1, &c);
		}
		if (test_vector & 0x04) {/* test edid */
			if (sp_tx_system_state > STATE_PARSE_EDID)
				sp_tx_set_sys_state(STATE_PARSE_EDID);
			sp_tx_test_edid = 1;
			debug_puts("Test EDID Requested!\n");
		}

		if (test_vector & 0x08) {//phy test pattern
			sp_tx_phy_auto_test();

			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
			c = c | 0x01;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60, 1, &c);
/*
			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
			while((c & 0x03) == 0){
				c = c | 0x01;
				sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
				sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1, &c);
			}*/
		}
	}

}
static void sp_tx_vsi_setup(void)
{
	unchar c;
	int i;

	for (i = 0; i < 10; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_MPEG_DATA00_REG + i), &c);
		sp_tx_packet_mpeg.MPEG_data[i] = c;
	}
}


static void sp_tx_mpeg_setup(void)
{
	unchar c;
	int i;

	for (i = 0; i < 10; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_MPEG_DATA00_REG + i), &c);
		sp_tx_packet_mpeg.MPEG_data[i] = c;
	}
}

static void sp_tx_auth_done_int_handler(void)
{
	if (sp_repeater_mode == false) {
		unchar bytebuf[2];
		if (HDCP_state > HDCP_HW_ENABLE
			&& sp_tx_system_state == STATE_HDCP_AUTH) {
			sp_read_reg(TX_P0, SP_TX_HDCP_STATUS, bytebuf);
			if (bytebuf[0] & SP_TX_HDCP_AUTH_PASS) {
				sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x2A, 2, bytebuf);
				/*max cascade exceeded or max devs exceed, disable encryption*/
				if ((bytebuf[1] & 0x08) || (bytebuf[0] & 0x80)) {
					debug_puts("max cascade/devs exceeded!\n");
					sp_tx_hdcp_encryption_disable();
				} else
					debug_puts("Authentication pass in Auth_Done\n");

				HDCP_state = HDCP_FINISH;

			} else {
				debug_puts("Authentication failed in AUTH_done\n");
#ifdef CONFIG_AMAZON_METRICS_LOG
				log_to_metrics(ANDROID_LOG_INFO, "kernel",
					"slimport:def:auth_done_fail=true;DV;1:NR");
#endif
				sp_tx_video_mute(1);
				sp_tx_clean_hdcp_status();
				HDCP_state = HDCP_FAILE;
			}
		}
	}
	auth_done = true;
	debug_puts("sp_tx_auth_done_int_handler\n");

}
static void sp_tx_polling_err_int_handler(void)
{
//	unchar temp;
	if ((sp_tx_system_state < STATE_SINK_CONNECTION))
		return;
	debug_puts("sp_tx_polling_err_int_handler\n");
	//if(AUX_ERR== sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x00, 1, &temp))
		system_power_down();
}
static void sp_tx_lt_done_int_handler(void)
{
	unchar c;
	if(sp_tx_LT_state == LT_WAITTING_FINISH
		&& sp_tx_system_state == STATE_LINK_TRAINING) {
		sp_read_reg(TX_P0, LT_CTRL, &c);
		if (c & 0x70) {
			c = (c & 0x70) >> 4;
			debug_printf("LT failed in interrupt, ERR code = %.2x\n", (uint) c);
			sp_tx_LT_state = LT_ERROR;
		} else {
			debug_puts("lt_done: LT Finish \n");
			sp_tx_LT_state = LT_FINISH;
		}
	}

}
static void sp_tx_link_change_int_handler(void)
{
	debug_puts("sp_tx_link_change_int_handler \n");
	if (sp_tx_system_state < STATE_LINK_TRAINING)
		return;
	sp_tx_link_err_check();
	link_down_check();
}
static void hdmi_rx_clk_det_int(void)
{
	debug_puts("*HDMI_RX Interrupt: Pixel Clock Change.\n");
	if (sp_tx_system_state > STATE_VIDEO_OUTPUT) {
		sp_tx_video_mute(1);
		sp_tx_enable_audio_output(0);
		sp_tx_set_sys_state(STATE_VIDEO_OUTPUT);
	}
}
static void hdmi_rx_sync_det_int(void)
{
	debug_puts("*HDMI_RX Interrupt: Sync Detect.\n");
}
static void hdmi_rx_hdmi_dvi_int(void)
{
	unchar c;
	debug_puts("hdmi_rx_hdmi_dvi_int. \n");
	sp_read_reg(RX_P0, HDMI_STATUS, &c);
	if ((c & HDMI_MODE) != g_hdmi_dvi_status) {
		debug_printf("hdmi_dvi_int: Is HDMI MODE: %x.\n", (uint)(c & HDMI_MODE));
		hdmi_rx_mute_audio(0);
		system_state_change_with_case(STATE_VIDEO_OUTPUT);
	}
}
/*
static void hdmi_rx_avmute_int(void)
{
	unchar avmute_status, c;

	sp_read_reg(RX_P0, HDMI_STATUS,
		       &avmute_status);
	if (avmute_status & MUTE_STAT) {
		debug_puts("HDMI_RX AV mute packet received.\n");
		hdmi_rx_mute_video(1);
		hdmi_rx_mute_audio(1);
		c = avmute_status & (~MUTE_STAT);
		sp_write_reg(RX_P0, HDMI_STATUS, c);
	}
}
*/
static void hdmi_rx_new_avi_int(void)
{
	debug_puts("*HDMI_RX Interrupt: New AVI Packet.\n");
	sp_tx_lvttl_bit_mapping();
	sp_tx_set_colorspace();
	sp_tx_avi_setup();
	sp_tx_config_packets(AVI_PACKETS);
}
static void hdmi_rx_new_vsi_int(void)
{
	unchar c;
	unchar hdmi_video_format,vsi_header,v3d_structure;
	debug_puts("*HDMI_RX Interrupt: NEW VSI packet.\n");
	sp_read_reg(TX_P0, SP_TX_3D_VSC_CTRL, &c);
	if(!(c&INFO_FRAME_VSC_EN))
	{
		sp_read_reg(RX_P1, HDMI_RX_MPEG_TYPE_REG, &vsi_header);
		sp_read_reg(RX_P1, HDMI_RX_MPEG_DATA03_REG, &hdmi_video_format);
		if((vsi_header==0x81)&&((hdmi_video_format &0xe0)== 0x40))
		{
			debug_puts("3D VSI packet is detected. Config VSC packet\n");
			sp_tx_vsi_setup(); //use mpeg packet as mail box to send vsi packet
			sp_tx_config_packets(VSI_PACKETS);


			sp_read_reg(RX_P1, HDMI_RX_MPEG_DATA05_REG, &v3d_structure);
			switch(v3d_structure&0xf0){
			case 0x00://frame packing
				v3d_structure = 0x02;
				break;
			case 0x20://Line alternative
				v3d_structure = 0x03;
				break;
			case 0x30://Side-by-side(full)
				v3d_structure = 0x04;
				break;
			default:
				v3d_structure = 0x00;
				debug_puts("3D structure is not supported\n");
				break;
			}

			sp_write_reg(TX_P0, SP_TX_VSC_DB1, v3d_structure);

			sp_write_reg_or(TX_P0, SP_TX_3D_VSC_CTRL, INFO_FRAME_VSC_EN);

			sp_write_reg_and(TX_P0, SP_TX_PKT_EN_REG, ~SPD_IF_EN);

			sp_write_reg_or(TX_P0, SP_TX_PKT_EN_REG, SPD_IF_UD);

			sp_write_reg_or(TX_P0, SP_TX_PKT_EN_REG, SPD_IF_EN);

		}

	}

}


static void hdmi_rx_no_vsi_int(void)
{

	unchar c;
	sp_read_reg(TX_P0, SP_TX_3D_VSC_CTRL, &c);
	if(c&INFO_FRAME_VSC_EN)
	{
		debug_puts("No new VSI is received, disable  VSC packet\n");
		c &= ~INFO_FRAME_VSC_EN;
		sp_write_reg(TX_P0, SP_TX_3D_VSC_CTRL, c);
		sp_tx_mpeg_setup();
		sp_tx_config_packets(MPEG_PACKETS);
	}

}
static void hdmi_rx_restart_audio_chk(void)
{
	debug_puts("WAIT_AUDIO: hdmi_rx_restart_audio_chk.\n");
	system_state_change_with_case(STATE_AUDIO_OUTPUT);
}
static void hdmi_rx_cts_rcv_int(void)
{
	if(sp_tx_ao_state == AO_INIT)
		sp_tx_ao_state = AO_CTS_RCV_INT;
	else if(sp_tx_ao_state == AO_AUDIO_RCV_INT)
		sp_tx_ao_state = AO_RCV_INT_FINISH;

	//debug_puts("*hdmi_rx_cts_rcv_int. \n");
	return;
}
static void hdmi_rx_audio_rcv_int(void)
{
	unchar c;
	if(sp_tx_ao_state == AO_INIT)
		sp_tx_ao_state = AO_AUDIO_RCV_INT;
	else if (sp_tx_ao_state == AO_CTS_RCV_INT) {
		sp_tx_ao_state = AO_RCV_INT_FINISH;
		sp_read_reg(TX_P0, SP_TX_AUD_CTRL, &c);
		if (!(c & AUD_EN) && (sp_tx_system_state >= STATE_AUDIO_OUTPUT))
			slimport_config_audio_output();
	}
	// debug_puts("*hdmi_rx_audio_rcv_int\n");

	return;
}

static void hdmi_rx_audio_samplechg_int(void)
{
	uint i;
	unchar c;
	/* transfer audio chaneel status from HDMI Rx to Slinmport Tx */
	for (i = 0; i < 5; i++) {
		sp_read_reg(RX_P0, (HDMI_RX_AUD_IN_CH_STATUS1_REG + i), &c);
		sp_write_reg(TX_P2, (SP_TX_AUD_CH_STATUS_REG1 + i), c);
	}
	return;
}

static void hdmi_rx_hdcp_error_int(void)
{
	static unchar count = 0;
	debug_puts("*HDMI_RX Interrupt: hdcp error.\n");
	if(count >= 40)	{
		count = 0;
		debug_puts("Lots of hdcp error occured ...\n");
		hdmi_rx_mute_audio(1);
		hdmi_rx_mute_video(1);
		hdmi_rx_set_hpd(0);
		delay_ms(10);
		hdmi_rx_set_hpd(1);
		sp_system_reset();
	}else
		count++;
}
static void hdmi_rx_new_gcp_int(void)
{
	unchar c;
	sp_read_reg(RX_P1, HDMI_RX_GENERAL_CTRL, &c);
	debug_printf("*HDMI_RX Interrupt: New GCP Packet.c=%d\n", (uint)c);
	if (c&SET_AVMUTE) {
			/* hdmi_rx_mute_video(1); */
			hdmi_rx_mute_audio(1);

	} else if (c&CLEAR_AVMUTE) {
			hdmi_rx_mute_video(0);
			hdmi_rx_mute_audio(0);
	}
}
void system_isr_handler(void)
{
	//if(sp_tx_system_state < STATE_PLAY_BACK)
		//debug_puts("=========system_isr==========\n");

	if (COMMON_INT1 & PLL_LOCK_CHG)
		sp_tx_pll_changed_int_handler();

	if (COMMON_INT2 & HDCP_AUTH_DONE)
		sp_tx_auth_done_int_handler();

	if (COMMON_INT3 & HDCP_LINK_CHECK_FAIL)
		sp_tx_hdcp_link_chk_fail_handler();


	if (COMMON_INT5 & DPCD_IRQ_REQUEST)
		sp_tx_sink_irq_int_handler();

	if (COMMON_INT5 & POLLING_ERR)
		sp_tx_polling_err_int_handler();

	if (COMMON_INT5 & TRAINING_Finish)
		sp_tx_lt_done_int_handler();

	if (COMMON_INT5 & LINK_CHANGE)
		sp_tx_link_change_int_handler();

	if(sp_tx_system_state > STATE_SINK_CONNECTION) {
		if (HDMI_RX_INT6 & NEW_AVI)
			hdmi_rx_new_avi_int();
	}
	if(sp_tx_system_state >= STATE_VIDEO_OUTPUT) {
		if (HDMI_RX_INT1 & CKDT_CHANGE)
			hdmi_rx_clk_det_int();

		if (HDMI_RX_INT1 & SCDT_CHANGE)
			hdmi_rx_sync_det_int();

		if (HDMI_RX_INT1 & HDMI_DVI)
			hdmi_rx_hdmi_dvi_int();
		/*
		if (HDMI_RX_INT1 & SET_MUTE)
			hdmi_rx_avmute_int();
		*/

		if (HDMI_RX_INT7 & NEW_VS) {
			//if(V3D_EN)
			hdmi_rx_new_vsi_int();
		}

		if (HDMI_RX_INT7 & NO_VSI)
			hdmi_rx_no_vsi_int();

#ifdef CEC_ENABLE
		if (HDMI_RX_INT7 & CEC_RX_READY)
			cec_states = CEC_HDMI_RX_REC;
#endif

		if((HDMI_RX_INT6 & NEW_AUD) || (HDMI_RX_INT3 & AUD_MODE_CHANGE))
			hdmi_rx_restart_audio_chk();


		if (HDMI_RX_INT6 & CTS_RCV)
			hdmi_rx_cts_rcv_int();

		if (HDMI_RX_INT5 & AUDIO_RCV)
			hdmi_rx_audio_rcv_int();


		if ( HDMI_RX_INT2 & HDCP_ERR)
			hdmi_rx_hdcp_error_int();

		if (HDMI_RX_INT6 & NEW_CP)
			hdmi_rx_new_gcp_int();

		if (HDMI_RX_INT2 & AUDIO_SAMPLE_CHANGE)
			hdmi_rx_audio_samplechg_int();

		if (HDMI_RX_INT1 & PCLK_CHANGE)
				debug_puts("Pclk change.\n");

		if (HDMI_RX_INT1 & PLL_UNLOCK)
				debug_puts("Pll unlock.\n");

		if (HDMI_RX_INT2 & ECC_ERR)
				debug_puts("Ecc error.\n");

		if (HDMI_RX_INT3 & AUDIO_SAMPLE_CHANGE)
				debug_puts("Audio mode change.\n");

		if (HDMI_RX_INT4 & SYNC_POL_CHANGE)
				debug_puts("Sync pol change.\n");

		if (HDMI_RX_INT4 & V_RES_CHANGE)
				debug_puts("V res change.\n");

		if (HDMI_RX_INT4 & H_RES_CHANGE)
				debug_puts("H res change.\n");

		if (HDMI_RX_INT4 & I_P_CHANGE)
				debug_puts("I/P change.\n");

		if (HDMI_RX_INT4 & DP_CHANGE)
				debug_puts("Dp change.\n");

		if (HDMI_RX_INT5 & VFIFO_OVERFLOW)
				debug_puts("Video fifo over.\n");

		if (HDMI_RX_INT5 & VFIFO_UNDERFLOW)
				debug_puts("Video fifo under.\n");

		if (HDMI_RX_INT5 & NO_AVI)
				debug_puts("No avi.\n");
	}

}
void sp_tx_show_infomation(void)
{
	unchar c,c1;
	uint h_res,h_act,v_res,v_act;
	uint h_fp,h_sw,h_bp,v_fp,v_sw,v_bp;
	ulong fresh_rate;
	ulong pclk;

	debug_puts("\n******************SP Video Information*******************\n");


	sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c);
	switch(c){
		case 0x06:
			debug_puts("BW = 1.62G\n");
			break;
		case 0x0a:
			debug_puts("BW = 2.7G\n");
			break;
		case 0x14:
			debug_puts("BW = 5.4G\n");
			break;
		case 0x19:
			debug_puts("BW = 6.75G\n");
			break;
		default:
			break;
	}

	pclk =sp_tx_pclk_calc();
	pclk = pclk / 10;
	debug_puts("SSC On");


	sp_read_reg(TX_P2, SP_TX_TOTAL_LINE_STA_L,&c);
	sp_read_reg(TX_P2, SP_TX_TOTAL_LINE_STA_H,&c1);

	v_res = c1;
	v_res = v_res << 8;
	v_res = v_res + c;


	sp_read_reg(TX_P2, SP_TX_ACT_LINE_STA_L,&c);
	sp_read_reg(TX_P2, SP_TX_ACT_LINE_STA_H,&c1);

	v_act = c1;
	v_act = v_act << 8;
	v_act = v_act + c;


	sp_read_reg(TX_P2, SP_TX_TOTAL_PIXEL_STA_L,&c);
	sp_read_reg(TX_P2, SP_TX_TOTAL_PIXEL_STA_H,&c1);

	h_res = c1;
	h_res = h_res << 8;
	h_res = h_res + c;


	sp_read_reg(TX_P2, SP_TX_ACT_PIXEL_STA_L,&c);
	sp_read_reg(TX_P2, SP_TX_ACT_PIXEL_STA_H,&c1);

	h_act = c1;
	h_act = h_act << 8;
	h_act = h_act + c;

	sp_read_reg(TX_P2, SP_TX_H_F_PORCH_STA_L,&c);
	sp_read_reg(TX_P2, SP_TX_H_F_PORCH_STA_H,&c1);

	h_fp = c1;
	h_fp = h_fp << 8;
	h_fp = h_fp + c;

	sp_read_reg(TX_P2, SP_TX_H_SYNC_STA_L,&c);
	sp_read_reg(TX_P2, SP_TX_H_SYNC_STA_H,&c1);

	h_sw = c1;
	h_sw = h_sw << 8;
	h_sw = h_sw + c;

	sp_read_reg(TX_P2, SP_TX_H_B_PORCH_STA_L,&c);
	sp_read_reg(TX_P2, SP_TX_H_B_PORCH_STA_H,&c1);

	h_bp = c1;
	h_bp = h_bp << 8;
	h_bp = h_bp + c;

	sp_read_reg(TX_P2, SP_TX_V_F_PORCH_STA,&c);
	v_fp = c;

	sp_read_reg(TX_P2, SP_TX_V_SYNC_STA,&c);
	v_sw = c;

	sp_read_reg(TX_P2, SP_TX_V_B_PORCH_STA, &c);
	v_bp = c;

	debug_printf("Total resolution is %d * %d \n", h_res, v_res);

	debug_printf("HF=%d, HSW=%d, HBP=%d\n", h_fp, h_sw, h_bp);
	debug_printf("VF=%d, VSW=%d, VBP=%d\n", v_fp, v_sw, v_bp);
	debug_printf("Active resolution is %d * %d ", h_act, v_act);

	if (h_res == 0 || v_res == 0) {
		fresh_rate = 0;
	} else {
		fresh_rate = pclk * 1000;
		fresh_rate = fresh_rate / h_res;
		fresh_rate = fresh_rate * 1000;
		fresh_rate = fresh_rate / v_res;
	}
	pr_info("   @ %ldHz\n", fresh_rate);

	sp_read_reg(TX_P0, SP_TX_VID_CTRL, &c);

	if ((c & 0x06) == 0x00)
		debug_puts("ColorSpace: RGB,");
	else if ((c & 0x06) == 0x02)
		debug_puts("ColorSpace: YCbCr422,");
	else if ((c & 0x06) == 0x04)
		debug_puts("ColorSpace: YCbCr444,");

	sp_read_reg(TX_P0, SP_TX_VID_CTRL, &c);

	if ((c & 0xe0) == 0x00)
		debug_puts("6 BPC\n");
	else if ((c & 0xe0) == 0x20)
		debug_puts("8 BPC\n");
	else if ((c & 0xe0) == 0x40)
		debug_puts("10 BPC\n");
	else if ((c & 0xe0) == 0x60)
		debug_puts("12 BPC\n");


	if(sp_tx_rx_type == DWN_STRM_IS_HDMI_7730) {//assuming it is anx7730
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x23, 1, &c);
		debug_printf("ANX7730 BC current FW Ver %.2x \n", (uint)(c & 0x7f));
	}



	debug_puts("\n***************************************\n");

}
void hdmi_rx_show_video_info(void)
{
    unchar c,c1;
    unchar cl,ch;
    uint n;
    uint h_res,v_res;

    sp_read_reg(RX_P0,HDMI_RX_HACT_LOW, &cl);
    sp_read_reg(RX_P0,HDMI_RX_HACT_HIGH, &ch);
    n = ch;
    n = (n << 8) + cl;
    h_res = n;

    sp_read_reg(RX_P0,HDMI_RX_VACT_LOW, &cl);
    sp_read_reg(RX_P0,HDMI_RX_VACT_HIGH, &ch);
    n = ch;
    n = (n << 8) + cl;
    v_res = n;

    debug_puts(">HDMI_RX Info<\n");
    sp_read_reg(RX_P0,HDMI_STATUS, &c);
    if(c & HDMI_MODE)
        debug_puts("HDMI_RX Mode = HDMI Mode.\n");
    else
        debug_puts("HDMI_RX Mode = DVI Mode.\n");

    sp_read_reg(RX_P0,HDMI_RX_VIDEO_STATUS_REG1, &c);
    if(c & VIDEO_TYPE)
    {
        v_res += v_res;
    }
    debug_printf("HDMI_RX Video Resolution = %d * %d ",h_res,v_res);
    sp_read_reg(RX_P0,HDMI_RX_VIDEO_STATUS_REG1, &c);
    if(c & VIDEO_TYPE)
        debug_puts("Interlace Video.\n");
    else
        debug_puts("Progressive Video.\n");


    sp_read_reg(RX_P0,HDMI_RX_SYS_CTRL1_REG, &c);
    if((c & 0x30) == 0x00)
        debug_puts("Input Pixel Clock = Not Repeated.\n");
    else if((c & 0x30) == 0x10)
        debug_puts("Input Pixel Clock = 2x Video Clock. Repeated.\n");
    else if((c & 0x30) == 0x30)
        debug_puts("Input Pixel Clock = 4x Vvideo Clock. Repeated.\n");

    if((c & 0xc0) == 0x00)
        debug_puts("Output Video Clock = Not Divided.\n");
    else if((c & 0xc0) == 0x40)
        debug_puts("Output Video Clock = Divided By 2.\n");
    else if((c & 0xc0) == 0xc0)
        debug_puts("Output Video Clock = Divided By 4.\n");

    if(c & 0x02)
        debug_puts("Output Video Using Rising Edge To Latch Data.\n");
    else
        debug_puts("Output Video Using Falling Edge To Latch Data.\n");


    debug_puts("Input Video Color Depth = ");
    sp_read_reg(RX_P0,0x70, &c1);
	 c1 &= 0xf0;
	 if(c1 == 0x00)
            debug_puts("Legacy Mode.\n");
	 else if(c1 == 0x40)
            debug_puts("24 Bit Mode.\n");
        else if(c1 == 0x50)
            debug_puts("30 Bit Mode.\n");
        else if(c1 == 0x60)
            debug_puts("36 Bit Mode.\n");
        else if(c1 == 0x70)
            debug_puts("48 Bit Mode.\n");



    debug_puts("Input Video Color Space = ");
    sp_read_reg(RX_P1,HDMI_RX_AVI_DATA00_REG, &c);
    c &= 0x60;
    if(c == 0x20)
        debug_puts("YCbCr4:2:2 .\n");
    else if(c == 0x40)
        debug_puts("YCbCr4:4:4 .\n");
    else if(c == 0x00)
        debug_puts("RGB.\n");
    else
        debug_printf("Unknow 0x44 = 0x%.2x\n",(int)c);



    sp_read_reg(RX_P1,HDMI_RX_HDCP_STATUS_REG, &c);
    if(c & AUTH_EN)
        debug_puts("Authentication is attempted.\n");
    else
        debug_puts("Authentication is not attempted.\n");

    for(cl=0;cl<20;cl++)
    {
        sp_read_reg(RX_P1,HDMI_RX_HDCP_STATUS_REG, &c);
        if(c & DECRYPT_EN)
            break;
        else
            delay_ms(10);
    }
    if(cl < 20)
        debug_puts("Decryption is active.\n");
    else
        debug_puts("Decryption is not active.\n");

}

void clean_system_status(void)
{
	if(g_need_clean_status) {
		debug_puts("clean_system_status. A -> B; \n");
		debug_puts("A:");
		print_sys_state(sp_tx_system_state_bak);
		debug_puts("B:");
		print_sys_state(sp_tx_system_state);

		g_need_clean_status = 0;
		if(sp_tx_system_state_bak >= STATE_LINK_TRAINING){
			if(sp_tx_system_state >= STATE_AUDIO_OUTPUT)
				hdmi_rx_mute_audio(1);
			else {
				hdmi_rx_mute_video(1);
				sp_tx_video_mute(1);
			}
			//sp_tx_enable_video_input(0);
			//sp_tx_enable_audio_output(0);
			//slimport_block_power_ctrl(SP_TX_PWR_VIDEO, SP_POWER_DOWN);
			//slimport_block_power_ctrl(SP_TX_PWR_AUDIO, SP_POWER_DOWN);
		}
		if(sp_tx_system_state_bak >= STATE_HDCP_AUTH
			&& sp_tx_system_state<= STATE_HDCP_AUTH){
			if(__read_reg(TX_P0, TX_HDCP_CTRL0) & 0xFC)
				sp_tx_clean_hdcp_status();
		}

		if (sp_repeater_mode == false) {
			if (HDCP_state != HDCP_CAPABLE_CHECK)
				HDCP_state = HDCP_CAPABLE_CHECK;
		}

		if(sp_tx_sc_state != SC_INIT)
			sp_tx_sc_state = SC_INIT;
		if(sp_tx_LT_state != LT_INIT)
			sp_tx_LT_state = LT_INIT;
		if(sp_tx_vo_state != VO_WAIT_VIDEO_STABLE)
			sp_tx_vo_state = VO_WAIT_VIDEO_STABLE;
	}
}
/* Add for HDCP cap check */
void sp_tx_get_ddc_hdcp_BCAP(unchar *bcap)
{
	unchar c;

	sp_write_reg(TX_P0, AUX_ADDR_7_0, 0X3A); /* 0X74 FOR HDCP DDC */
	sp_write_reg(TX_P0, AUX_ADDR_15_8, 0);
	sp_read_reg(TX_P0, AUX_ADDR_19_16, &c);
	c &= 0xf0;
	sp_write_reg(TX_P0, AUX_ADDR_19_16, c);

	sp_tx_aux_wr(0x00);
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, BUF_DATA_0, &c);


	sp_tx_aux_wr(0x40); /* 0X40: bcap */
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, BUF_DATA_0, bcap);
}

void sp_tx_get_ddc_hdcp_BKSV(unchar *bksv)
{
	unchar c;

	sp_write_reg(TX_P0, AUX_ADDR_7_0, 0X3A);/* 0X74 FOR HDCP DDC */
	sp_write_reg(TX_P0, AUX_ADDR_15_8, 0);
	sp_read_reg(TX_P0, AUX_ADDR_19_16, &c);
	c &= 0xf0;
	sp_write_reg(TX_P0, AUX_ADDR_19_16, c);

	sp_tx_aux_wr(0x00);
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, BUF_DATA_0, &c);


	sp_tx_aux_wr(0x00); /* 0X00: bksv */
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, BUF_DATA_0, bksv);
}

unchar slimport_hdcp_cap_check(void)
{
	unchar g_hdcp_cap = 0;
	unchar c, DDC_HDCP_BCAP, DDC_HDCP_BKSV;

	if ((sp_tx_rx_type == DWN_STRM_IS_VGA_9832)
		|| (sp_tx_rx_type == DWN_STRM_IS_ANALOG)) {
		g_hdcp_cap = 0;	/*  not capable HDCP */
	} else if (sp_tx_rx_type == DWN_STRM_IS_DIGITAL) {
		if (AUX_OK == sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x28, 1, &c)) {
			if (!(c & 0x01)) {
				pr_info("%s %s : Sink is not capable HDCP\n",  LOG_TAG, __func__);
				g_hdcp_cap = 0; /* not capable HDCP */
			} else {
				g_hdcp_cap = 1; /* capable HDCP */
			}
		} else {
			pr_info("%s %s : HDCP CAPABLE: read AUX err!\n", LOG_TAG, __func__);
		}
	} else if (sp_tx_rx_type == DWN_STRM_IS_HDMI_7730) {
			sp_tx_get_ddc_hdcp_BCAP(&DDC_HDCP_BCAP);
			sp_tx_get_ddc_hdcp_BKSV(&DDC_HDCP_BKSV);
			if ((DDC_HDCP_BCAP == 0x80) || (DDC_HDCP_BCAP == 0x82) || (DDC_HDCP_BCAP == 0x83)
				|| (DDC_HDCP_BCAP == 0xc0) || (DDC_HDCP_BCAP == 0xe0) || (DDC_HDCP_BKSV != 0x00))
				g_hdcp_cap = 1;
			else
				g_hdcp_cap = 0;
	} else {
		g_hdcp_cap = 1;
	}

	return g_hdcp_cap;
}
/* End HDCP cap check */
#ifdef CEC_ENABLE
static void hdmi_rx_cec_rcv_int(void)
{
#ifdef CEC_DBG_MSG_ENABLED
	pr_info("%s %s :HDMI RX received CEC\n", LOG_TAG, __func__);
#endif
	if ((sp_tx_rx_type == DWN_STRM_IS_HDMI_7730) && (CEC_IS_READY == cec_status_get()))
		upstream_hdmi_cec_readmessage();

}
void CEC_init_fun(void)
{
	/* Make sure CEC operation only happened after cec path is setup */
	if (sp_tx_rx_type == DWN_STRM_IS_HDMI_7730) {
		cec_init(); /* for CEC */
		pr_info("%s %s :cec initialed!\n", LOG_TAG, __func__);
	}
}
static void CEC_handler(void)
{
	switch (cec_states) {
	case CEC_INIT:
		CEC_init_fun();
		break;
	case CEC_HDMI_RX_REC:
		hdmi_rx_cec_rcv_int();
		break;
	case CEC_DOWNSTREAM_HDMI_READ:
		downstream_hdmi_cec_readmessage();
		pr_info("%s %s :Receive CEC command from downstream done!\n", LOG_TAG, __func__);
		break;
	case CEC_IDLE:
	default:
		break;
	}
	if (cec_states != CEC_IDLE)
		cec_states = CEC_IDLE;
}
#endif
#ifdef SIMULATE_WATCHDOG
void simulate_watchdog_func(void)
{
	static unchar bak_states;

	if ((sp_tx_cur_states() < STATE_AUDIO_OUTPUT)
		&& (sp_tx_cur_states() == sp_tx_system_state_bak)) {
		bak_states++;
		/* 100ms about one main loop, then 100*200 = 20s for watchdog */
		if (bak_states > 200) {
			bak_states = 0;
			system_power_down();
			pr_err("%s %s :SlimPort Watchdog start!\n", LOG_TAG, __func__);
		}
	} else {
		if (bak_states != 0)
			bak_states = 0;
	}
}
#endif

void slimport_monitor_audio(void)
{
	unchar c, c1, cts_2, step_index, cts_stop_step, count;
	int i;

	sp_read_reg(TX_P2, SP_TX_SPDID_AUD_STATUS, &c);
	if ((!(c & SP_TX_SPDIF_AUD_DET)) &&
	    (sp_tx_system_state >= STATE_AUDIO_OUTPUT)) {
		pr_debug("3618: 0x72:0x38 = 0x%.2x, no audio clock!\n",
			(uint)c);
		return;
	}

	if (!(sp_tx_rx_type == DWN_STRM_IS_HDMI_7730
		&& (sp_tx_system_state >= STATE_AUDIO_OUTPUT)))
		return;

	i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x7a, SP_RX_7730_VB_ID, &c);

	if (c & AUD_MUTE_FLAG)
		debug_puts("7730: Audio mute flag is set!\n");

	i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x7a,
			    SP_RX_7730_AUD_ADJUST_ENABLE, &c);

	if (c & SP_RX_7730_AUDIO_STRM_SENDING) {
		i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x7a,
				    SP_RX_7730_AUD_CTS_ADJ_STATUS_2, &cts_2);

		if (cts_2 & SP_RX_7730_AUD_CTS_ADJ_DONE) {
			step_index = cts_2 & 0x0f;

			i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x7a,
					    SP_RX_7730_AUD_CTS_STOP_REG, &c);
			cts_stop_step = c & 0x0f;

			if (step_index != cts_stop_step) {
				pr_debug("STEP_INDEX != CTS_STOP_STEP\n");
				pr_debug("7730: 0x7A:0x8B = 0x%.2x,  0x7A:0x88 = 0x%.2x\n",
					(uint)cts_2, (uint)c);
			}
		} else {
			debug_puts("7730: CTS Adjust not Done!");
		}

		for (i = 0; i < 13; i++) {
			sp_read_reg(RX_P1, (HDMI_RX_AUDIO_TYPE_REG + i), &c);
			i2c_master_read_reg(SP_RX_7730_SEG_ADDR_0x72,
					    AUD_INFOFRM_TYPE_IDX+i, &c1);
			if ((i < 3) && (c1 == 0)) {
				pr_debug("7730: Break. Audio infoframe Audio Infoframe Header/Version/Length should not be zero!\n");
				break;
			}
			count = 0;
			while (c != c1) {
				pr_debug("7730: Audio packet does not match with 3618 received!\n");
				pr_debug("3618: 0x80:0x%2x = 0x%.2x\n",
					(uint)(HDMI_RX_AUDIO_TYPE_REG + i),
					(uint)c);
				pr_debug("7730: 0x72:0x%2x = 0x%.2x\n",
					 (uint)(AUD_INFOFRM_TYPE_IDX + i),
					 (uint)c1);
				/*
				 * In case of an error, read audio packet
				 * 3 times to ensure that is not a false
				 * failure
				 */
				if (count <= 3) {
					i2c_master_read_reg(
						SP_RX_7730_SEG_ADDR_0x72,
						AUD_INFOFRM_TYPE_IDX+i, &c1);
					count++;
				} else if (audio_info_process) {
					audio_info_process = false;
					sp_tx_set_sys_state(STATE_AUDIO_OUTPUT);
					return;
				} else {
					return;
				}
			}
		}
	} else {
		pr_debug("7730: audio stream is not sending!\n");
	}
}

void slimport_tasks_handler(void)
{
	/* isr */
	if(sp_tx_system_state > STATE_WAITTING_CABLE_PLUG)
		system_isr_handler();
	hdcp_external_ctrl_flag_monitor();
	slimport_hdcp_repeater_reauth();
	slimport_monitor_audio();
	clean_system_status();
#ifdef CEC_ENABLE
	CEC_handler();
#endif
#ifdef SIMULATE_WATCHDOG
	simulate_watchdog_func();
#endif
	slimport_cable_monitor();
	/*clear up backup system state*/
	if (sp_tx_system_state_bak != sp_tx_system_state)
		sp_tx_system_state_bak = sp_tx_system_state;
}
/******************End task  process********************/

void slimport_main_process(void)
{
	slimport_state_process();
	if (sp_tx_system_state > STATE_WAITTING_CABLE_PLUG)
		slimport_int_rec();
	slimport_tasks_handler();
}


/*
 * Functions for slimport_rx anx7730
 */

#define SOURCE_AUX_OK 1
#define SOURCE_AUX_ERR 0
#define SOURCE_REG_OK 1
#define SOURCE_REG_ERR 0

#define SINK_DEV_SEL  0x005f0
#define SINK_ACC_OFS  0x005f1
#define SINK_ACC_REG  0x005f2

bool source_aux_read_7730dpcd(long addr, unchar cCount, unchar *pBuf)
{
	unchar c;
	unchar addr_l;
	unchar addr_m;
	unchar addr_h;
	addr_l = (unchar)addr;
	addr_m = (unchar)(addr >> 8);
	addr_h = (unchar)(addr >> 16);
	c = 0;
	while (1) {
		if (sp_tx_aux_dpcdread_bytes(addr_h, addr_m, addr_l, cCount, pBuf) == AUX_OK)
			return SOURCE_AUX_OK;
		c++;
		if (c > 3)
			return SOURCE_AUX_ERR;
	}
}

bool source_aux_write_7730dpcd(long addr, unchar cCount, unchar *pBuf)
{
	unchar c;
	unchar addr_l;
	unchar addr_m;
	unchar addr_h;
	addr_l = (unchar)addr;
	addr_m = (unchar)(addr >> 8);
	addr_h = (unchar)(addr >> 16);
	c = 0;
	while (1) {
		if (sp_tx_aux_dpcdwrite_bytes(addr_h, addr_m, addr_l, cCount, pBuf) == AUX_OK)
			return SOURCE_AUX_OK;
		c++;
		if (c > 3)
			return SOURCE_AUX_ERR;
	}
}

bool i2c_master_read_reg(unchar Sink_device_sel, unchar offset, unchar *Buf)
{
	unchar sbytebuf[2] = {0};
	long a0, a1;
	a0 = SINK_DEV_SEL;
	a1 = SINK_ACC_REG;
	sbytebuf[0] = Sink_device_sel;
	sbytebuf[1] = offset;

	if (source_aux_write_7730dpcd(a0, 2, sbytebuf) == SOURCE_AUX_OK) {
		if (source_aux_read_7730dpcd(a1, 1, Buf) == SOURCE_AUX_OK)
			return SOURCE_REG_OK;
	}
	return SOURCE_REG_ERR;
}

bool i2c_master_write_reg(unchar Sink_device_sel, unchar offset, unchar value)
{
	unchar sbytebuf[3] = {0};
	long a0;
	a0 = SINK_DEV_SEL;
	sbytebuf[0] = Sink_device_sel;
	sbytebuf[1] = offset;
	sbytebuf[2] = value;

	if (source_aux_write_7730dpcd(a0, 3, sbytebuf) == SOURCE_AUX_OK)
		return SOURCE_REG_OK;
	else
		return SOURCE_REG_ERR;
}

#ifdef ANX_LINUX_ENV
MODULE_DESCRIPTION("Slimport transmitter ANX7816 driver");
MODULE_AUTHOR("<swang@analogixsemi.com>");
MODULE_LICENSE("GPL");
#endif
#undef _SP_TX_DRV_C_

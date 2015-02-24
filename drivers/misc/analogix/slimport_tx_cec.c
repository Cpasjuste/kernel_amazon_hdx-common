/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
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


#include "slimport.h"
#include "slimport_tx_drv.h"
#include "slimport_tx_cec.h"

/* for debug messgage */
#ifdef CEC_DBG_MSG_ENABLED
#include <stdio.h>
#include <stdarg.h>
#endif

#ifdef CEC_ENABLE

#define CEC_DEVICE_TYPE	CEC_DEVICE_FREEUSE

unchar  g_CECRecvBuf[CEC_RECV_BUF_SIZE];
unchar  *g_pCECRecvHead;
unchar  *g_pCECRecvTail;
unchar  g_CECSendBuf[CEC_SEND_BUF_SIZE];
unchar  *g_pCECSendHead;
unchar  *g_pCECSendTail;

unchar  g_CECRecvBuf_HDMI[CEC_RECV_BUF_SIZE];
unchar  *g_pCECRecvHead_HDMI;
unchar  *g_pCECRecvTail_HDMI;
unchar  g_CECSendBuf_HDMI[CEC_SEND_BUF_SIZE];
unchar  *g_pCECSendHead_HDMI;
unchar  *g_pCECSendTail_HDMI;

unchar bCECStatus = CEC_NOT_READY;
struct tagCECFrame g_CECFrame;
struct tagCECFrame g_CECFrame_HDMI;

unchar g_LogicAddr;
unchar g_TimeOut;

#ifdef CEC_PHYCISAL_ADDRESS_INSERT
unchar downstream_physicaladdrh;
unchar downstream_physicaladdrl;
#endif

void cec_init(void)
{
	unsigned char c;
	unsigned char i;

	/* set status to un-initialied. */
	bCECStatus = CEC_NOT_READY;

	g_LogicAddr = CEC_DEVICE_TYPE;

	/*
	 * set uptream CEC logic address to 0x0(worked as TV),
	 * reset CEC, set to RX
	 */
	sp_write_reg(RX_P0, RX_CEC_CTRL, 0x09);


	/*
	 * set downtream CEC logic address to 0x04(worked as upstream
	 * DVD playback) reset cec, enable RX
	 */
	c = 0x49;
	for (i = 0; i < 5; i++) {
		if (AUX_OK == sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x70, 1, &c))
			break;
	}


	/* initial receive and transmitter buffer */
	for (c = 0; c < CEC_RECV_BUF_SIZE; c++) {
		g_CECRecvBuf[c] = 0;
		g_CECRecvBuf_HDMI[c] = 0;
	}

	for (c = 0; c < CEC_SEND_BUF_SIZE; c++) {
		g_CECSendBuf[c] = 0;
		g_CECSendBuf_HDMI[c] = 0;
	}

	g_pCECRecvHead = g_CECRecvBuf;
	g_pCECRecvTail = g_CECRecvBuf;
	g_pCECSendHead = g_CECSendBuf;
	g_pCECSendTail = g_CECSendBuf;

	g_pCECRecvHead_HDMI = g_CECRecvBuf_HDMI;
	g_pCECRecvTail_HDMI = g_CECRecvBuf_HDMI;
	g_pCECSendHead_HDMI = g_CECSendBuf_HDMI;
	g_pCECSendTail_HDMI = g_CECSendBuf_HDMI;

#ifdef CEC_PHYCISAL_ADDRESS_INSERT
	downstream_physicaladdrh = 0xff;
	downstream_physicaladdrl = 0xff;
#endif

	/* set status toinitialied */
	bCECStatus = CEC_IS_READY;
	g_TimeOut = 0;
}


void cec_status_set(CEC_STATUS cStatus)
{
	bCECStatus = cStatus;
}


CEC_STATUS cec_status_get(void)
{
	return bCECStatus;
}



unchar downStream_hdmi_cec_writemessage(struct tagCECFrame *pFrame, unchar Len)
{
	unchar t;

	if (bCECStatus == CEC_NOT_READY)
		return 0;

	t = downstream_cec_sendframe((unsigned char *)pFrame, Len);
	return t;
}

void downstream_hdmi_cec_readmessage(void)
{
	struct tagCECFrame *pFrame;
	unsigned char len;



	if (bCECStatus == CEC_NOT_READY)
		return;

	pFrame = &g_CECFrame;

	downstream_cec_readfifo();
	while ((len = downstream_cec_recvframe((unsigned char *)pFrame)) != 0) {

#ifdef CEC_DBG_MSG_ENABLED
		TRACE("<< ");
		TRACE_ARRAY((unsigned char *)pFrame, len);
#endif
		if (len >= 1)
			upstream_hdmi_cec_writemessage(&g_CECFrame, len);

	}
}


void downstream_cec_readfifo(void)
{
	unsigned char cstatus, c;
	unsigned char i;
	unsigned char FIFOLen;
	unsigned int time_out_counter;

	/* if aux error, abort read */
	if (AUX_ERR == sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x71, 1, &cstatus))
		return;

	time_out_counter = 0;

	do {
		/* get fifo count */
		FIFOLen = cstatus & 0x1F;
		FIFOLen <<= 1;
		for (i = 0; i < FIFOLen; i++) {
			/* cec fifo */
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x80, 1, &c);
			*g_pCECRecvHead = c;

			if (g_pCECRecvHead == g_CECRecvBuf + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvHead = g_CECRecvBuf;
			else
				g_pCECRecvHead++;

			if (g_pCECRecvHead == g_pCECRecvTail) {
#ifdef CEC_DBG_MSG_ENABLED
				TRACE("buffer full!\r\n");
#endif
			}
		}

		if (AUX_ERR == sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x71, 1, &cstatus))
			return;

		time_out_counter++;

		if (time_out_counter >= TIME_OUT_THRESHOLD)
			break;
	/* CEC busy or fifo data count not zero, go on reading */
	} while ((cstatus & 0x80) | (cstatus & 0x1F));
}

unsigned char downstream_cec_checkfullframe(void)
{
	unsigned char *pTmp;
	unsigned char i;
	unsigned int time_out_counter;

	time_out_counter = 0;
	i = 0;
	pTmp = g_pCECRecvTail;

	/* check if end of buffer */
	while (pTmp != g_pCECRecvHead) {
		/* check frame header */

		if (*pTmp != 0)
			i++;

		if (pTmp == g_CECRecvBuf + CEC_RECV_BUF_SIZE - 1)
			pTmp = g_CECRecvBuf;
		else
			pTmp++;

		if (pTmp == g_CECRecvBuf + CEC_RECV_BUF_SIZE - 1)
			pTmp = g_CECRecvBuf;
		else
			pTmp++;

		time_out_counter++;
		if (time_out_counter >= TIME_OUT_THRESHOLD)
			break;
	}


	if (i >= 1) {
		/* one full frame found */
		return 1;
	} else {
		/* no frame found */
		return 0;
	}
}

/* 01 xx 00 xx 00 xx ... 00 xx 01 xx 00 xx */
unsigned char downstream_cec_recvframe(unsigned char *pFrameData)
{
	unsigned char DataLen;
	unsigned int time_out_counter;

	time_out_counter = 0;

	DataLen = 0;

	if (downstream_cec_checkfullframe()) {
		do {
			if (g_pCECRecvTail == g_pCECRecvHead)
				break;

			if (g_pCECRecvTail == g_CECRecvBuf + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvTail = g_CECRecvBuf;
			else
				g_pCECRecvTail++;

			*pFrameData = *g_pCECRecvTail;
			DataLen++;
			pFrameData++;

			if (g_pCECRecvTail == g_CECRecvBuf + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvTail = g_CECRecvBuf;
			else
				g_pCECRecvTail++;

			time_out_counter++;
			if (time_out_counter >= TIME_OUT_THRESHOLD)
				break;
		} while (*g_pCECRecvTail == 0);
	}

	return DataLen;
}

unsigned char downstream_cec_sendframe(unsigned char *pFrameData, unsigned char Len)
{
	unsigned char i, j;
	unsigned char *pDataTmp;
	unsigned char LenTmp;
	unsigned char c;
	unsigned int time_out_counter;

	time_out_counter = 0;

	/*
	 * According to CEC 7.1, re-transmission can be attempted
	 * up to 5 times
	 */
	for (i = 5; i; i--) {
		for (pDataTmp = pFrameData, LenTmp = Len; LenTmp; LenTmp--) {
			c = *pDataTmp;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x80, 1, &c);
			pDataTmp++;
		}

		/* make sure CEC rx is idle */
		time_out_counter = 0;
		do {
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x71, 1, &c);
			time_out_counter++;

			if (time_out_counter >= TIME_OUT_THRESHOLD)
				break;
		} while (c & 0x80);

		/* clear tx done interrupt first */
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x10, 1, &c);
		c |= 0x80;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x10, 1, &c);

		/* switch to CEC tx mode */
		for (j = 0; j < 20; j++) {

			c = CEC_DEVICE_TYPE<<4;
			c |= 0x04;

			if (AUX_OK != sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x70, 1, &c))
				continue;
			else
				break;
		}

		time_out_counter = 0;

		do {
			if (AUX_OK == sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x10, 1, &c)) {
				time_out_counter++;
				if (time_out_counter >= 100 * TIME_OUT_THRESHOLD)
					break;
			} else {
				break;
			}
		} while (!(c & 0x80));

		/* clear the CEC TX done interrupt*/
		c = 0x80;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x10, 1, &c);


		/* Get CEC TX status */
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x11, 1, &c);

		if ((c & 0x03) == 0) {
			break;
		} else {
			/* clear the CEC error interrupt */
			c = 0x03;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x11, 1, &c);

			c = CEC_DEVICE_TYPE<<4;
			c |= 0x05;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x70, 1, &c);

		}
	}

	/* switch to CEC rx mode */
	for (j = 0; j < 20; j++) {

		c = CEC_DEVICE_TYPE << 4;
		c |= 0x08;

		if (AUX_OK != sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x70, 1, &c))
			continue;
		else
			break;
	}

	/* send succeed */
	if (i != 0)
		return 0;
	else
		return 1;
}



/* for HDMI RX CEC */
unsigned char upstream_hdmi_cec_writemessage(struct tagCECFrame *pFrame, unsigned char Len)
{
	unsigned char t;

	if (bCECStatus == CEC_NOT_READY)
		return 0;

	t = upstream_cec_sendframe((unsigned char *)pFrame, Len);
	return t;
}

void upstream_hdmi_cec_readmessage(void)
{
	struct tagCECFrame *pFrame;
	unsigned char len;



	if (bCECStatus == CEC_NOT_READY)
		return;

	pFrame = &g_CECFrame_HDMI;
	upstream_cec_readfifo();


	while ((len = upstream_cec_recvframe((unsigned char *)pFrame)) != 0) {
#ifdef CEC_DBG_MSG_ENABLED
		TRACE(">> ");
		TRACE_ARRAY((unsigned char *)pFrame, len);
#endif
		if (len >= 1) {
#ifdef CEC_PHYCISAL_ADDRESS_INSERT
			uptream_cec_parsemessage();
#endif
			downStream_hdmi_cec_writemessage(&g_CECFrame_HDMI, len);
		}
	}
}



void upstream_cec_readfifo(void)
{
	unsigned char cStatus, c;
	unsigned char i;
	unsigned char FIFOLen;
	unsigned int time_out_counter;

	time_out_counter = 0;

	/* Get CEC length */
	sp_read_reg(RX_P0, HDMI_RX_CEC_RX_STATUS_REG, &cStatus);

	do {
		FIFOLen = cStatus & 0x0F;

		if ((FIFOLen == 0) && (cStatus & 0x20))
			FIFOLen = 0x10;

		FIFOLen <<= 1;
		for (i = 0; i < FIFOLen; i++) {
			sp_read_reg(RX_P0, HDMI_RX_CEC_FIFO_REG, &c);
			*g_pCECRecvHead_HDMI = c;

			if (g_pCECRecvHead_HDMI == g_CECRecvBuf_HDMI + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvHead_HDMI = g_CECRecvBuf_HDMI;
			else
				g_pCECRecvHead_HDMI++;

			if (g_pCECRecvHead_HDMI == g_pCECRecvTail_HDMI) {
#ifdef CEC_DBG_MSG_ENABLED
				TRACE("buffer full!\r\n");
#endif
			}
		}
		sp_read_reg(RX_P0, HDMI_RX_CEC_RX_STATUS_REG, &cStatus);

		time_out_counter++;
		if (time_out_counter > LOCAL_REG_TIME_OUT_THRESHOLD)
			break;

	} while ((cStatus & 0x80) | (cStatus & 0x0F));

}

unsigned char upstream_cec_checkfullframe(void)
{
	unsigned char *pTmp;
	unsigned char i;
	unsigned int time_out_counter;

	time_out_counter = 0;

	i = 0;
	pTmp = g_pCECRecvTail_HDMI;
	while (pTmp != g_pCECRecvHead_HDMI) {

		if (*pTmp != 0)
			i++;

		if (pTmp == g_CECRecvBuf_HDMI + CEC_RECV_BUF_SIZE - 1)
			pTmp = g_CECRecvBuf_HDMI;
		else
			pTmp++;

		if (pTmp == g_CECRecvBuf_HDMI + CEC_RECV_BUF_SIZE - 1)
			pTmp = g_CECRecvBuf_HDMI;
		else
			pTmp++;

		time_out_counter++;
		if (time_out_counter >= TIME_OUT_THRESHOLD)
			break;


	}


	if (i >= 1)
		return 1;
	else
		return 0;
}

/* 01 xx 00 xx 00 xx ... 00 xx 01 xx 00 xx */
unsigned char upstream_cec_recvframe(unsigned char *pFrameData)
{
	unsigned char DataLen;

	unsigned int time_out_counter;

	time_out_counter = 0;

	DataLen = 0;

	if (upstream_cec_checkfullframe()) {
		do {
			if (g_pCECRecvTail_HDMI == g_pCECRecvHead_HDMI)
				break;

			if (g_pCECRecvTail_HDMI == g_CECRecvBuf_HDMI + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvTail_HDMI = g_CECRecvBuf_HDMI;
			else
				g_pCECRecvTail_HDMI++;


			*pFrameData = *g_pCECRecvTail_HDMI;
			DataLen++;
			pFrameData++;

			if (g_pCECRecvTail_HDMI == g_CECRecvBuf_HDMI + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvTail_HDMI = g_CECRecvBuf_HDMI;
			else
				g_pCECRecvTail_HDMI++;

			time_out_counter++;
			if (time_out_counter >= TIME_OUT_THRESHOLD)
				break;
		} while (*g_pCECRecvTail_HDMI == 0);
	}

	return DataLen;
}



unsigned char upstream_cec_sendframe(unsigned char *pFrameData, unsigned char Len)
{
	unsigned char i;
	unsigned char *pDataTmp;
	unsigned char LenTmp;
	unsigned char c;

	unsigned int time_out_counter;

	time_out_counter = 0;

	/* According to CEC 7.1, re-transmission can be attempted up to 5 times */
	for (i = 5; i; i--) {
		for (pDataTmp = pFrameData, LenTmp = Len; LenTmp; LenTmp--) {
			c = *pDataTmp;
			/* write CEC FIFO */
			sp_write_reg(RX_P0, HDMI_RX_CEC_FIFO_REG, c);
			pDataTmp++;

		}

		time_out_counter = 0;
		/* make sure CEC rx is idle */
		do {
			sp_read_reg(RX_P0, HDMI_RX_CEC_RX_STATUS_REG, &c);

			time_out_counter++;
			if (time_out_counter >= LOCAL_REG_TIME_OUT_THRESHOLD)
				break;

		} while (c & 0x80);


		/* clear tx done int first */
		sp_read_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, &c);
		sp_write_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, (c | 0x01));



		/* switch to CEC tx mode */
		sp_read_reg(RX_P0, RX_CEC_CTRL, &c);
		c &= 0xf3;
		c |= 0x04;
		sp_write_reg(RX_P0, RX_CEC_CTRL, c);

		time_out_counter = 0;
		do {

			sp_read_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, &c);

			time_out_counter++;
			if (time_out_counter >= LOCAL_REG_TIME_OUT_THRESHOLD) {
				g_TimeOut = 1;
				break;
			}
		} while (!(c & 0x01));

		/* clear the CEC TX done interrupt */
		c = 0x01;
		sp_write_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, c);


		/* Get CEC TX status */
		sp_read_reg(RX_P0, HDMI_RX_CEC_TX_STATUS_REG, &c);

		if ((c & 0x40) == 0 && g_TimeOut == 0) {
			break;
		} else {
			/* reset CEC */
			sp_read_reg(RX_P0, RX_CEC_CTRL, &c);
			c |= 0x01;
			sp_write_reg(RX_P0, RX_CEC_CTRL, c);
			g_TimeOut = 0;
		}
	}


	/* switch to CEC rx mode */
	sp_read_reg(RX_P0, RX_CEC_CTRL, &c);
	c &= 0xf3;
	c |= 0x08;
	sp_write_reg(RX_P0, RX_CEC_CTRL, c);

	/* send succeed */
	if (i != 0)
		return 0;
	else
		return 1;
}

#ifdef CEC_PHYCISAL_ADDRESS_INSERT
void downstream_cec_phy_add_set(unchar addr0, unchar addr1)
{
	pr_info("%s %s : 00 set phy addr[0] = %.2x,phy addr[1] = %.2x,\n",
		LOG_TAG, __func__, (uint)addr0, (uint)addr1);
	downstream_physicaladdrh = addr0;
	downstream_physicaladdrl = addr1;
}
#endif

#ifdef CEC_DBG_MSG_ENABLED
void trace(const char code *format, ...)
{
	va_list args;

	va_start(args, format);
	vprintf(format, args);
	va_end(args);
}

void TraceArray(unsigned char idata array[], unsigned char len)
{
	unsigned char i;

	for (i = 0; i < len; i++)
		trace(" %02BX", array[i]);

	trace("\r\n");
}
#endif


#ifdef CEC_PHYCISAL_ADDRESS_INSERT
void uptream_cec_parsemessage(void)
{
	switch (g_CECFrame_HDMI.msg.raw[0]) {

	case CEC_OPCODE_REPORT_PHYSICAL_ADDRESS:
		pr_info("%s %s : parse phy addr[0] = %.2x,phy addr[1] = %.2x,\n",
			LOG_TAG, __func__, (uint)downstream_physicaladdrh,
			(uint)downstream_physicaladdrl);
		if ((downstream_physicaladdrh != 0xff) && (downstream_physicaladdrl != 0xff)) {
			g_CECFrame_HDMI.msg.raw[1] = downstream_physicaladdrh;
			g_CECFrame_HDMI.msg.raw[2] = downstream_physicaladdrl;
		}

		break;

	default:
		break;
	}
}
#endif


void Downstream_Report_Physical_Addr(void)
{

	g_CECFrame.header.init = g_LogicAddr;
	g_CECFrame.header.dest = 0x0F;
	g_CECFrame.msg.raw[0] = CEC_OPCODE_REPORT_PHYSICAL_ADDRESS;
	g_CECFrame_HDMI.msg.raw[1] = 0x20;
	g_CECFrame_HDMI.msg.raw[2] = 0x00;
	g_CECFrame_HDMI.msg.raw[3] = CEC_DEVICE_TYPE_PLAYBACK_DEVICE;


	downStream_hdmi_cec_writemessage(&g_CECFrame, 5);


}
#endif


//  Copyright (c) Michael McElligott
// 
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU LIBRARY GENERAL PUBLIC LICENSE
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU LIBRARY GENERAL PUBLIC LICENSE for details.


#include <process.h>
#include <string.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include <windows.h>
#include <time.h>
#include <ntdef.h>
#include <ntddser.h>
#include <conio.h>
#include "ubx.h"
#include "ubxcb.h"




static const uint32_t baudRates[] = {9600, 9600*2, 9600*4, 9600*6, 115200, 115200*2, 115200*4, 115200*8, 0};

#define COM_BAUD_9600			0
#define COM_BAUD_19200			1
#define COM_BAUD_38400			2
#define COM_BAUD_57600			3
#define COM_BAUD_115200			4
#define COM_BAUD_230400			5
#define COM_BAUD_460800			6
#define COM_BAUD_921600			7

#define COM_PORT				(2)
#define COM_BAUD				COM_BAUD_230400
#define COM_BAUD_FWDEFAULT		COM_BAUD_9600
#define COM_BAUD_LASTSAVED		COM_BAUD_115200

#define BAUDRATE(n)				(baudRates[(n)])


static ubx_msg_t ubxRegTable;
static gpsdata_t userData;
static uint32_t msgCt;



typedef struct{
	double lat;
	double lon;
	float alt;
}__attribute__((packed))pos_rec_t;
pos_rec_t pos[1024*1024*10];
volatile static pos_rec_t ave;
volatile static int posTotal = 0;
volatile static int aveTotal = 0;



static const char *fixType[] = {
	"None",
	"Dead reckoning",
	"2D",
	"3D",
	"GNSS+deadReck",
	"Time only"
};


static inline void payloadDumpCt ()
{
	printf("rx msg count:\n");

	for (int i = 0; i < MAX_REGMSG; i++){
		ubx_func_t *handler = &ubxRegTable.msg[i];
		if (handler->ct){
			printf(" %2i %.2X/%.2X - %i\n", i, handler->uClass, handler->uId, handler->ct);
			handler->ct = 0;
		}
	}
	printf("\n");
}

// less than once per second
void msgPostLowCb (void *opaque, const intptr_t unused)
{
	//gpsdata_t *data = (gpsdata_t*)opaque;
	printf("msgPostLowCb\n");
}

// once per second
void msgPostMedCb (void *opaque, const intptr_t unused)
{
	payloadDumpCt();
	
	gpsdata_t *data = (gpsdata_t*)opaque;
	//printf("msgPostMedCb\n");

	printf("Logitude: %.8f\n", data->nav.longitude);
	printf("Latitude: %.8f\n", data->nav.latitude);
	printf("Altitude: %.1f\n", data->nav.altitude);

	printf("fixType: %s\n", fixType[data->fix.type]);
	printf("Sats: %i\n", data->fix.sats);
	printf("hAcc: %.2f\n", data->fix.hAcc/100.0f);
	printf("vAcc: %.2f\n", data->fix.vAcc/100.0f);

	printf("Dop: h:%.2f, v:%.2f, p:%.2f, g:%.2f\n",
		data->dop.horizontal/100.0f, data->dop.vertical/100.0f, data->dop.position/100.0f, data->dop.geometric/100.0f);
	
	printf("Date: %.i.%.02i.%i\n", data->date.day, data->date.month, data->date.year);
	printf("Time: %.02i:%.02i:%.02i.%.2i\n", data->time.hour, data->time.min, data->time.sec, data->time.ms);
	
	printf("Heading: %.2f\n", data->misc.heading/100.0f);
	printf("Speed: %.2f km/h\n", data->misc.speed/100.0f);
	printf("\n");

	printf("rx msgCt: %i\n", msgCt);
	msgCt = 0;	
	
	if (!aveTotal) return;

	printf("::Ave: %u, %.8f %.8f %f\n", aveTotal, ave.lon/(double)aveTotal, ave.lat/(double)aveTotal, ave.alt/(double)aveTotal);
}

// more than once per second
void msgPostHighCb (void *opaque, const intptr_t unused)
{
	gpsdata_t *data = (gpsdata_t*)opaque;
	//printf("msgPostHighCb\n");
	if (data->fix.type == PVT_FIXTYPE_3D){
	
		if (posTotal < 10*1024*1024){
			pos_rec_t *rec = &pos[posTotal++];

			rec->lat = data->nav.latitude;
			rec->lon = data->nav.longitude;
			rec->alt = data->nav.altitude;
		}
	}
}

static inline uint16_t calcChkSum (const uint8_t *hex, const uint32_t htotal, uint8_t *c1, uint8_t *c2)
{
	*c1 = 0; *c2 = 0;
	
	for (int i = 0; i < htotal; i++){
		*c1 += hex[i];
		*c2 += *c1;
	}
	return (*c1<<8)|*c2;
}

static inline void serialClose (ubx_device_t *dev)
{
    if (dev->u.serial != INVALID_HANDLE_VALUE)
        CloseHandle(dev->u.serial);
}

static inline void serialClean (ubx_device_t *dev)
{
	unsigned long comError = 0;
	COMSTAT comstat = {0};
	PurgeComm(dev->u.serial, PURGE_RXCLEAR | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_TXABORT);
	ClearCommError(dev->u.serial, &comError, &comstat);
}
 	
int serialOpen (ubx_device_t *dev, const int port, const int baud)
{
 
	HANDLE hSerial = INVALID_HANDLE_VALUE;
	char dev_name[MAX_PATH] = "";
	
 	if (port < 1){
        int scanMax = 96;
        int scanMin = 0;
 
    	for (int n = scanMax; n >= scanMin; --n){
        	sprintf(dev_name, "\\\\.\\COM%d", n);
        	hSerial = CreateFile(dev_name, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        	if (hSerial != INVALID_HANDLE_VALUE)
            	break;
    	}
 	}else{
		sprintf(dev_name, "\\\\.\\COM%d", port);
		hSerial = CreateFile(dev_name, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	}

	if (hSerial == INVALID_HANDLE_VALUE)
		return 0;
	   
	DCB dcb;
 	GetCommState(hSerial, &dcb);
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fOutX = dcb.fInX = FALSE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = 0;
	dcb.BaudRate = baud;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	SetCommState(hSerial, &dcb);

	COMMTIMEOUTS CommTimeouts;
 	GetCommTimeouts(hSerial, &CommTimeouts);
	CommTimeouts.WriteTotalTimeoutConstant = 50;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	
	CommTimeouts.ReadTotalTimeoutConstant = 50;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadIntervalTimeout = MAXWORD;
	SetCommTimeouts(hSerial, &CommTimeouts);

 	dev->u.serial = hSerial;
	serialClean(dev);
 	return 1;
}

static inline int serialWrite (ubx_device_t *dev, void *buffer, const uint32_t bufferSize, uint32_t *bytesWritten)
{
	int ret = WriteFile(dev->u.serial, buffer, bufferSize, (unsigned long*)bytesWritten, NULL);
	//int er = GetLastError();
	//printf("serialWrite %i %i, %X\n", ret, *bytesWritten, er);
	return ret;
}

static inline int serialRead (ubx_device_t *dev, void *buffer, const uint32_t bufferSize, uint32_t *bytesRead)
{
	int ret = ReadFile(dev->u.serial, buffer, bufferSize, (unsigned long*)bytesRead, NULL);
	//int er = GetLastError();
	//printf("serialRead %i %i, %X\n", ret, *bytesRead, er);

	return ret;
}

static inline void bufferDump (const uint8_t *buffer, const int32_t bufferSize)
{
	for (int i = 0; i < bufferSize; i++)
		printf("0x%02X ", buffer[i]);

	printf("\n");
}

static inline int findSync (const uint8_t *buffer, const int32_t bufferSize)
{
	for (int i = 0; i < bufferSize-6; i++){
		if (buffer[i] == MSG_UBX_B1 && buffer[i+1] == MSG_UBX_B2)
			return i;
	}
	return -1;
}

static inline ubx_func_t *payloadHandlerFuncGet (uint8_t msg_class, uint8_t msg_id)
{
	for (int i = 0; i < MAX_REGMSG; i++){
		ubx_func_t *handler = &ubxRegTable.msg[i];
		if (msg_class == handler->uClass && msg_id == handler->uId)
			return handler;
	}
	return NULL;
}

static inline int payloadHandlerFuncSet (uint8_t msg_class, uint8_t msg_id, void *payloadFunc, const uint32_t status)
{
	for (int i = 0; i < MAX_REGMSG; i++){
		ubx_func_t *handler = &ubxRegTable.msg[i];
		if (!handler->uClass && !handler->uId){
			handler->uClass = msg_class;
			handler->uId = msg_id;
			handler->func = payloadFunc;
			handler->enabled = status&0x01;
			return 1;
		}
	}
	return 0;
}

static inline void payloadPostCbSet (const uint8_t type, void *postCbFunc, const intptr_t unused)
{
	if (!type || type > CBFREQ_TOTAL){
		printf("payloadPostCbSet: invalid type (%i)\n", type);
		return;
	}
	ubxRegTable.postCb[type-1].func = postCbFunc;
	return;
}

static inline int payloadHandlerSet (uint8_t msg_class, uint8_t msg_id, void *payloadFunc, const uint32_t status)
{
	if (!payloadHandlerFuncGet(msg_class, msg_id))
		return payloadHandlerFuncSet(msg_class, msg_id, payloadFunc, status);
	else
		printf("payloadHandlerSet() failed: %X/%X\n", msg_class, msg_id);
	return 0;
}

static inline ubx_func_t *payloadHandlerGet (uint8_t msg_class, uint8_t msg_id)
{
	return payloadHandlerFuncGet(msg_class, msg_id);
}

static inline void payloadOpaqueSet (void *ptr)
{
	ubxRegTable.userDataPtr = ptr;
}

static inline void *payloadOpaqueGet ()
{
	return ubxRegTable.userDataPtr;
}

static inline int payloadEnable (uint8_t msg_class, uint8_t msg_id)
{
	ubx_func_t *handler = payloadHandlerFuncGet(msg_class, msg_id);
	if (handler){
		handler->enabled = 1;
		return 1;
	}
	return 0;
}

static inline int payloadDisable (uint8_t msg_class, uint8_t msg_id)
{
	ubx_func_t *handler = payloadHandlerFuncGet(msg_class, msg_id);
	if (handler){
		handler->enabled = 0;
		return 1;
	}
	return 0;
}

static inline int payloadHandlerFunc (ubx_func_t *handler, const uint8_t *payload, const uint16_t msg_len)
{
	//if (handler->enabled){
		const int32_t type = handler->func(payload, msg_len, ubxRegTable.userDataPtr);
		if (type == CBFREQ_INVALID){		// payload wasn't what was expected
			// do nothing
			return 0;

		}else if (type != CBFREQ_NONE){
			ubxRegTable.postCb[type-1].func(payloadOpaqueGet(), 0);
		}
		return 1;
	//}
	//return 0;
}

#define ALERT_DISABLED		0
#define ALERT_BEFORE		1
#define ALERT_AFTER			2
#define ALERT_SINGLE		4

static inline void ubxPayloadDispatch (uint8_t msg_class, uint8_t msg_id, uint16_t msg_len, const uint8_t *payload)
{
	ubx_func_t *handler = payloadHandlerFuncGet(msg_class, msg_id);
	if (handler){
		handler->ct++;
		
		if (handler->enabled){
			if (handler->alert&ALERT_BEFORE){
				// fire off alert
				// do something
			}

			if (payloadHandlerFunc(handler, payload, msg_len)){
				if (handler->alert&ALERT_AFTER){
					// fire off alert
					// do something here
	
					printf("#### ubxPayloadDispatch: Alert signaled for %X/%X\n", msg_class, msg_id);
				}
			}
			
			if (handler->alert&ALERT_SINGLE){
				handler->alert = ALERT_DISABLED;
			}
		}
	}else{
		printf("ubxPayloadDispatch: unknown class/id: %.2X/%.2X\n", msg_class, msg_id);	
	}
}

static inline int ubxBufferFrameDecompose (uint8_t *buffer, const int32_t bufferSize)
{
	const uint8_t msg_class = buffer[0];
	const uint8_t msg_id = buffer[1];
	const uint16_t msg_len = (buffer[3]<<8) | buffer[2];
	const uint16_t msg_crc = (buffer[3+msg_len+1]<<8) | buffer[3+msg_len+2];

	int frameEnd = 3+msg_len+2; 
	if (frameEnd > bufferSize){
		//printf("multipart frame: %i %i\n", frameEnd, bufferSize);
		return -frameEnd;
	}
	
	//printf("class: %.2X/%.2X\n", msg_class, msg_id);
	//printf("len: %i\n", msg_len);

	uint8_t c1 = 0;
	uint8_t c2 = 0;
	const uint16_t crc = calcChkSum(buffer, msg_len+4, &c1, &c2);
	if (msg_crc != crc){
		printf("crc mismatch: %X, %X\n", msg_crc, crc);
		return 0;
	}	

	//printf("crc: %X, %X\n", msg_crc, crc);
	
	//we have a valid frame
	ubxPayloadDispatch(msg_class, msg_id, msg_len, &buffer[4]);
	
	return msg_len+4+2;	// 
}

static inline int ubxBufferDecompose (uint8_t *buffer, const int32_t bufferSize, uint32_t *ct)
{
	for (int i = 0; i < bufferSize-6; ){
		int found = findSync(&buffer[i], bufferSize-i);
		if (found < 0) return -i;
		i += found + 2;
		
		(*ct)++;
		int ret = ubxBufferFrameDecompose(&buffer[i], bufferSize-i);
		if (ret < 0) return -(i-2);
		i += ret;
	}
	return 1;
}

static inline int writeBin (ubx_device_t *dev, void *buffer, const uint32_t bufferSize)
{
	uint32_t bytesWritten = 0;
	return serialWrite(dev, buffer, bufferSize, &bytesWritten);
}

static inline int ubx_send (ubx_device_t *dev, const uint8_t *buffer, const uint16_t len)
{
	if (len < 3){
		printf("ubx_send: invalid msg length (%i)\n", len);
		return 0;
	}
	
	uint8_t ubxMsg[len+4];
	memset(ubxMsg, 0, sizeof(ubxMsg));
	
	ubxMsg[0] = MSG_UBX_B1;
	ubxMsg[1] = MSG_UBX_B2;
	memcpy(&ubxMsg[2], buffer, len);
	calcChkSum(buffer, len, &ubxMsg[sizeof(ubxMsg)-2], &ubxMsg[sizeof(ubxMsg)-1]);
	
	int ret = writeBin(dev, ubxMsg, sizeof(ubxMsg));
	Sleep(1);
	return ret; 
}

static inline int ubx_sendEx (ubx_device_t *dev, const uint16_t waitMs, const uint8_t clsId, const uint8_t msgId, const void *buffer, const uint16_t len)
{
	if (len < 1){
		printf("ubx_sendEx: invalid msg length (%i)\n", len);
		return 0;
	}
	
	size_t blen = len + 8;	// include space for ubx ident, class/msg Id and CRC
	uint8_t ubxMsg[blen];
	//memset(ubxMsg, 0, sizeof(ubxMsg));
	
	ubxMsg[0] = MSG_UBX_B1;
	ubxMsg[1] = MSG_UBX_B2;
	ubxMsg[2] = clsId;
	ubxMsg[3] = msgId;
	ubxMsg[4] = (len&0x00FF);
	ubxMsg[5] = (len&0xFF00)>>8;
	memcpy(&ubxMsg[6], buffer, len);
	
	// crc includes class/msgId through to data [buffer] end
	// crc excludes ubx ident and crc byte space
	calcChkSum(&ubxMsg[2], blen-4, &ubxMsg[sizeof(ubxMsg)-2], &ubxMsg[sizeof(ubxMsg)-1]);
	//printf("ubx_sendEx %X %X, %i\n", ubxMsg[sizeof(ubxMsg)-2], ubxMsg[sizeof(ubxMsg)-1], sizeof(ubxMsg));
	//serialBufferDump(ubxMsg, sizeof(ubxMsg));
	
	int ret = writeBin(dev, ubxMsg, sizeof(ubxMsg));
	if (waitMs) Sleep(waitMs);
	return ret;
}

static inline void ubx_msgPoll (ubx_device_t *dev, const uint8_t clsId, const uint8_t msgId)
{
	const uint8_t buffer[] = {clsId, msgId, 0x00, 0x00};
	ubx_send(dev, buffer, sizeof(buffer));
}

static inline void ubx_msgEnableEx (ubx_device_t *dev, const uint8_t clsId, const uint8_t msgId, const uint8_t rate)
{
	//printf("ubx_msgEnableEx %.2X/%.2X %i\n", clsId, msgId, rate);
	const uint8_t buffer[] = {UBX_CFG, UBX_CFG_MSG, 0x03, 0x00, clsId, msgId, rate};
	ubx_send(dev, buffer, sizeof(buffer));
}

static inline void ubx_msgEnable (ubx_device_t *dev, const uint8_t clsId, const uint8_t msgId)
{
	ubx_msgEnableEx(dev, clsId, msgId, 1);
}

static inline void ubx_msgDisable (ubx_device_t *dev, const uint8_t clsId, const uint8_t msgId)
{
	const uint8_t buffer[] = {UBX_CFG, UBX_CFG_MSG, 0x03, 0x00, clsId, msgId, 0x00};
	ubx_send(dev, buffer, sizeof(buffer));
}

static inline void ubx_msgDisableAll (ubx_device_t *dev)
{
	const uint8_t buffer[] = {UBX_CFG, UBX_CFG_MSG, 0x03, 0x00, 0xFF, 0xFF, 0x00};
	ubx_send(dev, buffer, sizeof(buffer));
}

static inline void ubx_msgEnableAll (ubx_device_t *dev)
{
	const uint8_t buffer[] = {UBX_CFG, UBX_CFG_MSG, 0x03, 0x00, 0xFF, 0xFF, 0x01};
	ubx_send(dev, buffer, sizeof(buffer));
}

static inline void disableUnknowns (ubx_device_t *dev)
{
#if 0
	ubx_msgDisable(dev, 0x03, 0x09);
	ubx_msgDisable(dev, 0x03, 0x11);
	ubx_msgDisable(dev, 0x09, 0x13);
	ubx_msgDisable(dev, 0x0C, 0x10);
	ubx_msgDisable(dev, 0x0C, 0x1A);
	ubx_msgDisable(dev, 0x0C, 0x31);
	ubx_msgDisable(dev, 0x0C, 0x34);
	ubx_msgDisable(dev, 0x0C, 0x35);
	ubx_msgDisable(dev, 0x27, 0x00);
	ubx_msgDisable(dev, UBX_SEC, 0x00);
	ubx_msgDisable(dev, UBX_RXM, 0x23);
	ubx_msgDisable(dev, UBX_RXM, 0x51);
	ubx_msgDisable(dev, UBX_RXM, 0x52);
	ubx_msgDisable(dev, UBX_RXM, 0x57);
	ubx_msgDisable(dev, UBX_NAV, 0x36);
	ubx_msgDisable(dev, UBX_MON, 0x0D);
	ubx_msgDisable(dev, UBX_MON, 0x11);
	ubx_msgDisable(dev, UBX_MON, 0x1B);
	ubx_msgDisable(dev, UBX_MON, 0x1D);
	ubx_msgDisable(dev, UBX_MON, 0x1E);
	ubx_msgDisable(dev, UBX_MON, 0x1F);
	ubx_msgDisable(dev, UBX_MON, 0x22);
	ubx_msgDisable(dev, UBX_MON, 0x23);
	ubx_msgDisable(dev, UBX_MON, 0x25);
	ubx_msgDisable(dev, UBX_MON, 0x26);
	ubx_msgDisable(dev, UBX_MON, 0x2B);
#endif
}

static inline uint32_t ubxWinRun (ubx_device_t *dev, uint8_t *buffer, const uint32_t bufferSize, uint32_t *bOffset)
{
 	memset(buffer+*bOffset, 0, bufferSize-*bOffset);
 	uint32_t bytesRead = 0;
 	uint32_t ct = 0;

	int ret = serialRead(dev, buffer+*bOffset, bufferSize-*bOffset, &bytesRead);
	if (ret > 0){
		bytesRead += *bOffset;
		*bOffset = 0;

		ret = ubxBufferDecompose(buffer, bytesRead, &ct);
		if (ret < 0){
			*bOffset = -ret;
			memcpy(buffer, &buffer[*bOffset], (bytesRead - *bOffset)+1);
			*bOffset = (bytesRead - *bOffset);
		}
	}else{
		Sleep(10);
	}
	return ct;
}

unsigned int __stdcall readThread (void *ptr)
{
	ubx_device_t *dev = (ubx_device_t*)ptr;
	
	const uint32_t bufferSize = 8192 * 16;
	uint8_t buffer[bufferSize];
	uint32_t bOffset = 0;
 		
	while (!kbhit())
		msgCt += ubxWinRun(dev, buffer, bufferSize, &bOffset);
	
	_endthreadex(1);
	return 1;
}

static inline void ubx_rst_hotStart (ubx_device_t *dev)
{
	cfg_rst_t rst = {0};
	
	rst.navBbrMask = RST_BBRMASK_HOTSTART;
	rst.resetMode = RST_RESETMODE_SWGNSSONLY;
	
	ubx_sendEx(dev, 10, UBX_CFG, UBX_CFG_RST, &rst, sizeof(rst));	
}

static inline void ubx_rst_warmStart (ubx_device_t *dev)
{
	cfg_rst_t rst = {0};
	
	rst.navBbrMask = RST_BBRMASK_WARMSTART;
	rst.resetMode = RST_RESETMODE_SWGNSSONLY;
	
	ubx_sendEx(dev, 10, UBX_CFG, UBX_CFG_RST, &rst, sizeof(rst));	
}

static inline void ubx_rst_coldStart (ubx_device_t *dev)
{
	cfg_rst_t rst = {0};
	
	rst.navBbrMask = RST_BBRMASK_COLDSTART;
	rst.resetMode = RST_RESETMODE_SWGNSSONLY;
	
	ubx_sendEx(dev, 10, UBX_CFG, UBX_CFG_RST, &rst, sizeof(rst));	
}

static inline void configureGNSS (ubx_device_t *dev)
{

	const int cfgBlks = 7;
	const int glen = CFG_GNSS_SIZE(cfgBlks);
	uint8_t _gnss[glen];
	cfg_gnss_t *gnss = (cfg_gnss_t*)_gnss;

	cfg_cfgblk_t *cfg = &gnss->cfgblk[0];
	cfg->gnssId = GNSSID_GPS;
	cfg->resTrkCh = 4;
	cfg->maxTrkCh = 24;
	cfg->flags = GNSS_CFGBLK_ENABLED | GNSS_CFGBLK_SIGENABLED;
	
	cfg = &gnss->cfgblk[1];
	cfg->gnssId = GNSSID_GLONASS;
	cfg->resTrkCh = 4;
	cfg->maxTrkCh = 20;
	cfg->flags = GNSS_CFGBLK_ENABLED | GNSS_CFGBLK_SIGENABLED;
	
	cfg = &gnss->cfgblk[2];
	cfg->gnssId = GNSSID_GALILEO;
	cfg->resTrkCh = 4;
	cfg->maxTrkCh = 10;
	cfg->flags = GNSS_CFGBLK_DISABLED | GNSS_CFGBLK_SIGENABLED;

	cfg = &gnss->cfgblk[3];
	cfg->gnssId = GNSSID_BEIDOU;
	cfg->resTrkCh = 8;
	cfg->maxTrkCh = 16;
	cfg->flags = GNSS_CFGBLK_DISABLED | GNSS_CFGBLK_SIGENABLED;
	
	cfg = &gnss->cfgblk[4];
	cfg->gnssId = GNSSID_IMES;
	cfg->resTrkCh = 0;
	cfg->maxTrkCh = 8;
	cfg->flags = GNSS_CFGBLK_DISABLED | GNSS_CFGBLK_SIGENABLED;
	
	cfg = &gnss->cfgblk[5];
	cfg->gnssId = GNSSID_QZSS;
	cfg->resTrkCh = 0;
	cfg->maxTrkCh = 3;
	cfg->flags = GNSS_CFGBLK_DISABLED | GNSS_CFGBLK_SIGENABLED;
	
	cfg = &gnss->cfgblk[6];
	cfg->gnssId = GNSSID_SBAS;
	cfg->resTrkCh = 1;
	cfg->maxTrkCh = 4;
	cfg->flags = GNSS_CFGBLK_DISABLED | GNSS_CFGBLK_SIGENABLED;

	gnss->msgVer = 0;
	gnss->numTrkChHw = 32;
	gnss->numTrkChUse = 32;
	gnss->numConfigBlocks = cfgBlks;

	ubx_sendEx(dev, 10, UBX_CFG, UBX_CFG_GNSS, gnss, glen);
}

static inline void configurePorts (ubx_device_t *dev)
{
	cfg_prt_uart_t prt = {0};
	
	prt.portId = CFG_PORTID_UART1;
	prt.mode.bits.charLen = CFG_CHARLEN_8BIT;
	prt.mode.bits.nStopBits = CFG_STOPBITS_1;
	prt.mode.bits.partity = CFG_PARTITY_NONE;
	prt.mode.bits.bitOrder = CFG_BITORDER_LSB;
	prt.baudRate = BAUDRATE(COM_BAUD);
	prt.inProtoMask = CFG_PROTO_UBX;
	prt.outProtoMask = CFG_PROTO_UBX;

	ubx_sendEx(dev, 10, UBX_CFG, UBX_CFG_PRT, &prt, sizeof(prt));
}

static inline void configureRate (ubx_device_t *dev)
{
	cfg_rate_t rate = {0};
	
	rate.measRate = 53;		// ms. 53ms = ~18-19hz
	rate.navRate = 1;		// 1 measurement per navigation
	rate.timeRef = CFG_TIMEREF_UTC;
	
	ubx_sendEx(dev, 10, UBX_CFG, UBX_CFG_RATE, &rate, sizeof(rate));
}

static inline void configureNav5 (ubx_device_t *dev)
{
	cfg_nav5_t nav = {0};
	
	nav.mask  = NAV5_MASK_DYN | NAV5_MASK_MINEL | NAV5_MASK_POSFIXMODE | NAV5_MASK_DRLIM;
	nav.mask |= NAV5_MASK_POSMASK | NAV5_MASK_TIMEMASK | NAV5_MASK_STATICHOLDMASK;
	nav.mask |= NAV5_MASK_DGPSMASK | NAV5_MASK_CNOTHRESHOLD | NAV5_MASK_UTC;
		
		
	nav.dynModel = NAV5_DYNMODEL_STATIONARY;
	nav.fixMode = NAV5_FIXMODE_3D;
	nav.fixedAlt = 37.0f * 100;				// meters, when using NAV5_FIXMODE_2D
	nav.fixedAltVar = 0.5f * 10000;			// deviation,  ^^^ 
	nav.minElv = 5;
	nav.drLimit = 0;
	nav.pDop = 25.0f * 10;
	nav.tDop = 25.0f * 10;
	nav.pAcc = 100;
	nav.tAcc = 350;
	nav.pAccADR = 0;						// also known as reserved1;
	nav.staticHoldThresh = 0;
	nav.dynssTimeout = 60;
	nav.cnoThreshNumSVs = 0;
	nav.cnoThresh = 0;
	nav.staticHoldMaxDist = 0;
	nav.utcStandard = NAV5_UTCSTD_AUTO;
	
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_NAV5, &nav, sizeof(nav));
}

static inline void configureNavX5 (ubx_device_t *dev)
{
	cfg_navx5_t nav = {0};
	nav.version = NAVX5_VERSION_PROTO18;
	nav.mask1  = NAVX5_MASK1_MINMAX | NAVX5_MASK1_MINCNO | NAVX5_MASK1_INITIAL3DFIX;
	nav.mask1 |= NAVX5_MASK1_WKNROLL | NAVX5_MASK1_ACKAID | NAVX5_MASK1_PPP | NAVX5_MASK1_AOP;
	
	nav.mask2 = NAVX5_MASK2_ADR;
	nav.minSVs = 3;
	nav.maxSVs = 32;
	nav.minCNO = 6;
	nav.iniFix3D = 1;
	nav.ackAiding = 0;
	nav.wknRollover = 1867;						// 0 = firmware default.
	nav.sigAttenCompMode = NAVX5_SACM_AUTO;		
	nav.usePPP = 1;
	nav.aopCfg = NAVX5_AOPCFG_USEAOP;
	nav.aopOrbMaxErr = 100;
	nav.useAdr = 0;
	
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_NAVX5, &nav, sizeof(nav));
}

static inline void ubx_msgInfPoll (ubx_device_t *dev, const uint8_t protocolID)
{
	
	cfg_inf_poll_t inf = {0};
	inf.protocolID = protocolID;
	
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));
}

static inline void ubx_msgInfEnable (ubx_device_t *dev, const uint8_t portId, const uint8_t infmsg)
{
	
	cfg_inf_t inf = {0};
	inf.protocolID = portId;
	inf.infMsgMask[CFG_PORTID_I2C] = 0;
	inf.infMsgMask[CFG_PORTID_UART1] = infmsg;
	inf.infMsgMask[CFG_PORTID_UART2] = 0;
	inf.infMsgMask[CFG_PORTID_USB] = 0;
	inf.infMsgMask[CFG_PORTID_SPI] = 0;
	
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));
}

static inline void ubx_msgInfDisable (ubx_device_t *dev, const uint8_t portId)
{
	
	cfg_inf_t inf = {0};
	inf.protocolID = portId;
	inf.infMsgMask[CFG_PORTID_I2C] = 0;
	inf.infMsgMask[CFG_PORTID_UART1] = 0;
	inf.infMsgMask[CFG_PORTID_UART2] = 0;
	inf.infMsgMask[CFG_PORTID_USB] = 0;
	inf.infMsgMask[CFG_PORTID_SPI] = 0;
	
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));
}

static inline void ubx_msgInfDisableAll (ubx_device_t *dev)
{
	
	cfg_inf_t inf = {0};
	inf.infMsgMask[CFG_PORTID_I2C] = 0;
	inf.infMsgMask[CFG_PORTID_UART1] = 0;
	inf.infMsgMask[CFG_PORTID_UART2] = 0;
	inf.infMsgMask[CFG_PORTID_USB] = 0;
	inf.infMsgMask[CFG_PORTID_SPI] = 0;

	inf.protocolID = INF_PROTO_UBX;
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));

	inf.protocolID = INF_PROTO_NEMA;
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));
	
	inf.protocolID = INF_PROTO_RAW;
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));
	
	inf.protocolID = INF_PROTO_RTCM3;
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));
	
	inf.protocolID = INF_PROTO_USER0;
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));
	
	inf.protocolID = INF_PROTO_USER1;
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));
	
	inf.protocolID = INF_PROTO_USER2;
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));
	
	inf.protocolID = INF_PROTO_USER3;
	ubx_sendEx(dev, 1, UBX_CFG, UBX_CFG_INF, &inf, sizeof(inf));
	
}

static inline void configureInf (ubx_device_t *dev)
{
	//ubx_msgInfPoll(dev, INF_PROTO_UBX);
	//ubx_msgInfPoll(dev, INF_PROTO_NEMA);
	ubx_msgInfDisableAll(dev);
	//ubx_msgInfEnable(dev, INF_PROTO_UBX, INF_MSG_WARNING|INF_MSG_NOTICE|INF_MSG_DEBUG);
}

static inline void configureGeofence (ubx_device_t *dev)
{
	const int geoFences = 2;
	const int glen = CFG_GEOFENCE_SIZE(geoFences);
	uint8_t _geo[glen];
	cfg_geofence_t *geo = (cfg_geofence_t*)_geo;
	
	
	geo->version = 0;
	geo->numFences = geoFences;
	geo->confLvl = GEOFENCE_CONFIDIENCE_99999;
	geo->pioEnabled = 0;
	geo->pinPolarity = 0;
	geo->pin = 0;
	
	geo->fence[0].lat = FLT2UBX(54.6161497);
	geo->fence[0].lon = FLT2UBX(-5.9366867);
	geo->fence[0].radius = 25 * 100.0f;

	geo->fence[1].lat = FLT2UBX(54.619988);
	geo->fence[1].lon = FLT2UBX(-6.502265);
	geo->fence[1].radius = 25 * 100.0f;	
	
	ubx_sendEx(dev, 10, UBX_CFG, UBX_CFG_GEOFENCE, geo, glen);
}


// reset module rate to what we need client-side
static inline void baudReset (ubx_device_t *dev)
{
	for (int i = 0; baudRates[i]; i++){
		if (serialOpen(dev, COM_PORT, baudRates[i])){
			configurePorts(dev);
			Sleep(20);
			serialClose(dev);
		}
	}
}


int main (const int argc, const char *argv[])
{   

	payloadOpaqueSet(&userData);
	
	payloadPostCbSet(CBFREQ_LOW, msgPostLowCb, 0);
	payloadPostCbSet(CBFREQ_MEDIUM, msgPostMedCb, 0);
	payloadPostCbSet(CBFREQ_HIGH, msgPostHighCb, 0);

	payloadHandlerSet(UBX_ACK, UBX_ACK_NAK, ack_nak, 0);
	payloadHandlerSet(UBX_ACK, UBX_ACK_ACK, ack_ack, 0);
	payloadHandlerSet(UBX_NAV, UBX_NAV_GEOFENCE, nav_geofence, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_NAV, UBX_NAV_DOP, nav_dop, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_NAV, UBX_NAV_PVT, nav_pvt, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_NAV, UBX_NAV_SVINFO, nav_svinfo, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_NAV, UBX_NAV_STATUS, nav_status, MSG_STATUS_ENABLED);	
	payloadHandlerSet(UBX_NAV, UBX_NAV_SAT, nav_sat, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_NAV, UBX_NAV_EOE, nav_eoe, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_NAV, UBX_NAV_POSLLH, nav_posllh, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_AID, UBX_AID_ALM, aid_alm, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_AID, UBX_AID_AOP, aid_aop, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_AID, UBX_AID_EPH, aid_eph, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_MON, UBX_MON_VER, mon_ver, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_MON, UBX_MON_IO, mon_io, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_RXM, UBX_RXM_SFRBX, rxm_sfrbx, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_INF, UBX_INF_ERROR, inf_debug, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_INF, UBX_INF_WARNING, inf_debug, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_INF, UBX_INF_NOTICE, inf_debug, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_INF, UBX_INF_TEST, inf_debug, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_INF, UBX_INF_DEBUG, inf_debug, MSG_STATUS_ENABLED);	
	payloadHandlerSet(UBX_CFG, UBX_CFG_INF, cfg_inf, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_CFG, UBX_CFG_NAV5, cfg_nav5, MSG_STATUS_ENABLED);	
	payloadHandlerSet(UBX_CFG, UBX_CFG_NAVX5, cfg_navx5, MSG_STATUS_ENABLED);	
	payloadHandlerSet(UBX_CFG, UBX_CFG_GEOFENCE, cfg_geofence, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_CFG, UBX_CFG_PRT, cfg_prt, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_CFG, UBX_CFG_USB, cfg_usb, MSG_STATUS_ENABLED);


	ubx_device_t dev = {0};
	baudReset(&dev);

	if (serialOpen(&dev, COM_PORT, BAUDRATE(COM_BAUD))){
		unsigned int tid = 0;
		HANDLE hReadThread = (HANDLE)_beginthreadex(NULL, 0, readThread, &dev, 0, &tid);
		Sleep(10);

		if (1) configurePorts(&dev);
		if (1) configureInf(&dev);
		if (1) configureRate(&dev);
		if (1) configureNav5(&dev);
		if (1) configureNavX5(&dev);
		if (0) configureGNSS(&dev);		// will auto generate a warm-start
		if (0) configureGeofence(&dev);

		ubx_msgDisableAll(&dev);
		ubx_msgPoll(&dev, UBX_MON, UBX_MON_VER);
		ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_USB);
		ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_PRT);
		ubx_msgEnable(&dev, UBX_MON, UBX_MON_IO);
		ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_DOP);
		ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_PVT);
		ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_EOE);
		//ubx_msgEnableEx(&dev, UBX_NAV, UBX_NAV_GEOFENCE, 60);
		ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_GEOFENCE);
		//ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_NAV5);
		//ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_NAVX5);
		//ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_SAT);
		//ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_SVINFO);
		//ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_STATUS);
		//ubx_msgPoll(&dev, UBX_AID, UBX_AID_EPH);
		//ubx_msgPoll(&dev, UBX_AID, UBX_AID_ALM);
		//ubx_msgPoll(&dev, UBX_AID, UBX_AID_AOP);
		//ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_POSLLH);


		while(!kbhit()){
			volatile pos_rec_t _ave = {0};
			
			aveTotal = posTotal;
			for (int i = 0; i < aveTotal; i++){
				pos_rec_t *rec = &pos[i];
				_ave.lon += rec->lon;
				_ave.lat += rec->lat;
				_ave.alt += rec->alt;
			}
			ave = _ave;

			Sleep(1000);
		}

	
		WaitForSingleObject(hReadThread, INFINITE);
		CloseHandle(hReadThread);
	
		ubx_msgDisable(&dev, UBX_MON, UBX_MON_IO);
		ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_SAT);
		Sleep(20);
		
		serialClean(&dev);
		serialClose(&dev);
	}
	
	
	return 1;
};


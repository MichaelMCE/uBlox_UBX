
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

// Tabs at 4 spaces

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
#include "extra.h"


static int oncePerSecond = 0;

static const uint32_t baudRates[] = {9600, 9600*2, 9600*4, 9600*6, 115200, 115200*2, 115200*4, 115200*8, 0};

#define COM_BAUD_9600			0
#define COM_BAUD_19200			1
#define COM_BAUD_38400			2
#define COM_BAUD_57600			3
#define COM_BAUD_115200			4
#define COM_BAUD_230400			5
#define COM_BAUD_460800			6
#define COM_BAUD_921600			7

//#define COM_PORT				(29)
//#define COM_PORT				(5)
//#define COM_PORT				(19)
#define COM_PORT				(9)
#define COM_BAUD				COM_BAUD_115200
#define COM_BAUD_FWDEFAULT		COM_BAUD_9600
#define COM_BAUD_LASTSAVED		COM_BAUD_115200

#define BAUDRATE(n)				(baudRates[(n)])


static ubx_msg_t ubxRegTable;
static gpsdata_t userData;
static uint32_t msgCt;
static uint32_t rxt = 0;



static const char *fixType[] = {
	"None",
	"Dead reckoning",
	"2D",
	"3D",
	"GNSS+deadReck",
	"Time only"
};


int nWeekdayOfMonth (int week, int wday, int month, int year, struct tm *result)
{
   memset(result, 0, sizeof(*result));
   time_t t;

   year -= 1900;
   result->tm_year = year;
   result->tm_mon  = month;
   result->tm_mday = 1;

   t = mktime(result);
   if (t == (time_t)-1)
      return 0;

   if (wday != result->tm_wday)
      result->tm_mday += wday - result->tm_wday + 7 * (wday < result->tm_wday);

   result->tm_mday += 7 * week;

   t = mktime(result);
   if (t == (time_t)-1)
      return 0;

   return 1;
}

void adjustUTCToTimeZone (utcTime_t *gps, uint16_t tz, uint8_t tzdir)
{
	uint8_t maxDayInAnyMonth[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //gps->Mth 1-12 (not zero)


	if (gps->Yr%4 == 0){
		maxDayInAnyMonth[2] = 29;
	}//adjust for leapyear
    
	uint8_t maxDayUtcMth = maxDayInAnyMonth[gps->Mth];
	uint8_t maxDayPrevMth = maxDayInAnyMonth[gps->Mth-1];
        
	// month before utc month
	if (!maxDayPrevMth) maxDayPrevMth = 31;


	uint16_t hr = (gps->Hm/100)*100;
	uint16_t m = gps->Hm-hr;  //2115 --> 2100 hr and 15 min
	
	if (tzdir){ //adjusting forwards
		tz += gps->Hm;
		if (tz >= 24){
			gps->Hm = tz-24;
			gps->Day++;                //spill over to next day
			
			if (gps->Day > maxDayUtcMth){
				gps->Day = 1;
				gps->Mth++; //spill over to next month
				
				if (gps->Mth > 12){
					gps->Mth = 1;
					gps->Yr++;        //spill over to next year
				}
			}
		}else{
			gps->Hm = tz;
		}
	}else{ //adjusting backwards
		if (tz > gps->Hm){
			gps->Hm = (24-(tz-hr))+m;
			gps->Day--;  // back to previous day
			
			if (gps->Day == 0){                             //back to previous month
				gps->Mth--;
				gps->Day = maxDayPrevMth;
				
				if (!gps->Mth){
					gps->Mth = 12;                 //back to previous year
					gps->Yr--;
				}
			}
		}else{
			gps->Hm -= tz;
		}
	}
}


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
/*
static bool IsDst (int day, int month, int dow)
    {
        if (month < 3 || month > 10)  return false; 
        if (month > 3 && month < 10)  return true; 

        int previousSunday = day - dow;

        if (month == 3) return previousSunday >= 25;
        if (month == 10) return previousSunday < 25;

        return false; // this line never gonna happend
    }
*/    
// once per second
void msgPostMedCb (void *opaque, const intptr_t unused)
{
	//printf("msgPostMedCb\n");
	
	gpsdata_t datacopy = *(gpsdata_t*)opaque;
	gpsdata_t *data = &datacopy;

	printf("Logitude: %.7f\n", data->nav.longitude);
	printf("Latitude: %.7f\n", data->nav.latitude);
	printf("Altitude: %.1f\n", data->nav.altitude);
	printf("  Avg:\n");
	printf("Logitude: %.8f\n", data->navAvg.longitude);
	printf("Latitude: %.8f\n", data->navAvg.latitude);
	printf("Altitude: %.1f\n", data->navAvg.altitude);

	printf("fixType: %s\n", fixType[data->fix.type]);
	printf("Sats: %i\n", data->fix.sats);
	printf("3D Acc: %.2f\n", data->fix.pAcc/100.0f);
	printf("2D Acc: %.2f\n", data->fix.hAcc/100.0f);
	printf("vAcc: %.2f\n", data->fix.vAcc/100.0f);
	
	printf("Dop: H:%.2f, V:%.2f, P:%.2f, G:%.2f\n",
		data->dop.horizontal/100.0f, data->dop.vertical/100.0f, data->dop.position/100.0f, data->dop.geometric/100.0f);


	int lastSunMarch = 0;
	int lastSunOctober = 0;
	struct tm result;
	
	int resultOk = nWeekdayOfMonth(-1, 0/*Sunday*/, 3/*March*/, data->date.year, &result);
	if (resultOk)
		lastSunMarch = result.tm_mday;
	resultOk = nWeekdayOfMonth(-1, 0/*Sunday*/, 10/*October*/, data->date.year, &result);
	if (resultOk)
		lastSunOctober = result.tm_mday;

	if ((data->date.month == 3 && data->date.day >= lastSunMarch) || data->date.month > 3){
		if ((data->date.month == 10 && data->date.day < lastSunOctober) || data->date.month < 10){
			utcTime_t utc;
			utc.Yr = data->date.year;
			utc.Mth = data->date.month;
			utc.Day = data->date.day;
			utc.Hm = data->time.hour;			
			
			adjustUTCToTimeZone(&utc, 0001, 1);
			data->time.hour = utc.Hm;
			data->date.day = utc.Day;
			data->date.month = utc.Mth;
			data->date.year = utc.Yr;
		}
	}
	
	printf("Time: %.02i:%.02i:%.02i.%.2i\n", data->time.hour, data->time.min, data->time.sec, data->time.ms);
	printf("Date: %.i.%.02i.%i\n", data->date.day, data->date.month, data->date.year);
	printf("Heading: %.1f\n", data->misc.heading);
	printf("Speed: %.1f km/h\n", data->misc.speed);

	printf("rx msgCt: %i, %i\n", msgCt, rxt);
	printf("\n");
	msgCt = 0;
	rxt = 0;
}

// more than once per second
void msgPostHighCb (void *opaque, const intptr_t unused)
{
	//printf("msgPostHighCb\n");

	gpsdata_t *data = (gpsdata_t*)opaque;
	if (data->fix.type == PVT_FIXTYPE_3D){
		if (0){		// do something with the data
			//lat = data->nav.latitude;
			//lon = data->nav.longitude;
			//alt = data->nav.altitude;
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
/*
uint16_t calcChkSum2 (const uint8_t *hex, const uint32_t htotal, uint16_t *c1, uint16_t *c2)
{
	*c1 = 0; *c2 = 0;
	
	for (int i = 0; i < htotal; i++){
		*c1 = (*c1 + hex[i]) % 256 ;
		*c2 = (*c2 + *c1) % 256;
	}
	
	*c1 &= 0xFF;
	*c2 &= 0xFF;
	
	printf("%X %X\n", *c1, *c2);

	return (*c1<<8)|*c2;
}*/

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
	dcb.fDtrControl = DTR_CONTROL_ENABLE;
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
	CommTimeouts.WriteTotalTimeoutConstant = 200;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	
	CommTimeouts.ReadTotalTimeoutConstant = 200;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadIntervalTimeout = MAXWORD;
	SetCommTimeouts(hSerial, &CommTimeouts);

 	dev->u.serial = hSerial;
	serialClean(dev);
	Sleep(100);
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
	//int ret = ReadFile(dev->u.serial, buffer, bufferSize, (unsigned long*)bytesRead, NULL);
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

int ubxBufferFrameProcess (uint8_t *buffer, const int32_t bufferSize)
{
	const uint8_t msg_class = buffer[0];
	const uint8_t msg_id = buffer[1];
	const uint16_t msg_len = (buffer[3]<<8) | buffer[2];
	const uint16_t msg_crc = (buffer[3+msg_len+1]<<8) | buffer[3+msg_len+2];

	int frameEnd = 4+msg_len+2; 
	if (frameEnd > bufferSize){
		printf("multipart frame: %i %i\n", frameEnd, bufferSize);
		return -1;
	}
	
	//printf("class: %.2X/%.2X\n", msg_class, msg_id);
	//printf("len: %i\n", msg_len);

	uint8_t c1 = 0;
	uint8_t c2 = 0;
	const uint16_t crc = calcChkSum(buffer, msg_len+4, &c1, &c2);
	if (msg_crc != crc){
		printf("class: %.2X/%.2X\n", msg_class, msg_id);
		printf("len: %i\n", msg_len);
		printf("crc mismatch: %X, %X\n", msg_crc, crc);
		return 0;
	}	
	//printf("CRC: %X, %X, %i\n", msg_crc, crc, msg_len+4+2);
	
	//we have a valid frame
	ubxPayloadDispatch(msg_class, msg_id, msg_len, &buffer[4]);
	
	return msg_len+4+2;
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

int searchSync (uint8_t *buffer, uint32_t len)
{
	if (len > 1){
		for (int i = 0; i < len-1; i++){
			if (buffer[i] == MSG_UBX_B1 && buffer[i+1] == MSG_UBX_B2)
				return i;
		}
	}
	return -1;
}

int searchFrame (uint8_t *buffer, const int32_t bufferSize)
{
	//const uint8_t msg_class = buffer[0];
	//const uint8_t msg_id = buffer[1];
	const uint16_t msg_len = (buffer[3]<<8) | buffer[2];
	const uint16_t msg_crc = (buffer[3+msg_len+1]<<8) | buffer[3+msg_len+2];

	int frameEnd = 4+msg_len+2; 
	if (frameEnd > bufferSize)
		return 0;
	
	uint8_t c1 = 0;
	uint8_t c2 = 0;
	const uint16_t crc = calcChkSum(buffer, msg_len+4, &c1, &c2);
	if (msg_crc != crc)
		return -(msg_len+4+2);
	
	return msg_len+4+2;
}

uint32_t ubxMsgRun (ubx_device_t *dev, uint8_t *inBuffer, uint32_t bufferSize, int32_t *writePos)
{
	uint32_t writeLength = 0;
	uint32_t bytesRead = 0;
	uint8_t readBuffer[8] = {0}; // Should be no larager than the smallest [message+control bytes]


	serialRead(dev, readBuffer, sizeof(readBuffer), &bytesRead);
	if (bytesRead > 0){
		rxt += bytesRead;

		for (int i = 0; i < bytesRead; i++)
			inBuffer[(*writePos)++] = readBuffer[i];

		writeLength = *writePos;
		if (writeLength < 6) return 0;
		
		// check if there is a complete message
		int foundSync = searchSync(inBuffer, writeLength);
		if (foundSync < 0){
			printf("sync not found %i %i\n", foundSync, writeLength);
			//bufferDump(inBuffer, writeLength);
			return 0;
		}

		// advance past sync bytes
		foundSync += 2;

		int foundFrame = searchFrame(&inBuffer[foundSync], writeLength-foundSync);
		if (foundFrame > 0){
			int process = ubxBufferFrameProcess(&inBuffer[foundSync], writeLength-foundSync);

			if (foundSync+foundFrame < writeLength){
				for (int i = 0; i < sizeof(readBuffer); i++)
					inBuffer[i] = inBuffer[foundSync+foundFrame+i];

				*writePos = writeLength - (foundSync+foundFrame);
				memset(inBuffer+sizeof(readBuffer), 0, bufferSize-sizeof(readBuffer));
			}else{
				memset(inBuffer, 0, bufferSize);
				*writePos = 0;
			}
			return (process > 0);

		}else if (foundFrame < 0){
			printf("mismatch crc %i, %i %i\n", -foundFrame, *writePos, foundSync);
			memset(inBuffer, 0, bufferSize);
			*writePos = 0;
		}
	}
	return 0;
}

unsigned int __stdcall readThread (void *ptr)
{
	ubx_device_t *dev = (ubx_device_t*)ptr;
	
	const uint32_t bufferSize = 2048;
	uint8_t buffer[bufferSize+8];
	int32_t bOffset = 0;
 		
	while (!kbhit())
		msgCt += ubxMsgRun(dev, buffer, bufferSize, &bOffset);
	
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
	cfg->maxTrkCh = 24;
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
	cfg->flags = GNSS_CFGBLK_ENABLED | GNSS_CFGBLK_SIGENABLED;

	gnss->msgVer = 0;
	gnss->numTrkChHw = 72;
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
	
	rate.measRate = 58;		// ms. 53ms = ~18-19hz
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
	nav.fixMode = NAV5_FIXMODE_AUTO;
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

void setHNR (ubx_device_t *dev, uint8_t rate)
{
	cfg_hnr_t hnr = {0};
	
	hnr.highNavRate = rate;
	ubx_sendEx(dev, 10, UBX_CFG, UBX_CFG_HNR, &hnr, sizeof(hnr));
}

static inline void configureHNR (ubx_device_t *dev)
{
	setHNR(dev, 17);	// set rate to 17hz
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
	payloadHandlerSet(UBX_NAV, UBX_NAV_POSECEF, nav_posecef, MSG_STATUS_ENABLED);
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
	payloadHandlerSet(UBX_CFG, UBX_CFG_RATE, cfg_rate, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_CFG, UBX_CFG_NAV5, cfg_nav5, MSG_STATUS_ENABLED);	
	payloadHandlerSet(UBX_CFG, UBX_CFG_NAVX5, cfg_navx5, MSG_STATUS_ENABLED);	
	payloadHandlerSet(UBX_CFG, UBX_CFG_GNSS, cfg_gnss, MSG_STATUS_ENABLED);	
	payloadHandlerSet(UBX_CFG, UBX_CFG_GEOFENCE, cfg_geofence, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_CFG, UBX_CFG_PRT, cfg_prt, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_CFG, UBX_CFG_USB, cfg_usb, MSG_STATUS_ENABLED);
	payloadHandlerSet(UBX_NAV, UBX_NAV_TIMEBDS, nav_timebds, MSG_STATUS_ENABLED);
	


	ubx_device_t dev = {0};
	baudReset(&dev);

	if (serialOpen(&dev, COM_PORT, BAUDRATE(COM_BAUD))){
		printf("Port %i Open @ %i\n\n", COM_PORT, BAUDRATE(COM_BAUD));
		
		unsigned int tid = 0;
		HANDLE hReadThread = (HANDLE)_beginthreadex(NULL, 0, readThread, &dev, 0, &tid);
		Sleep(10);

#if 0
		if (1) configurePorts(&dev);
		if (1) configureInf(&dev);
		if (1) configureRate(&dev);
		if (1) configureNav5(&dev);
		if (1) configureNavX5(&dev);
		if (1) configureHNR(&dev);
		if (1) configureGNSS(&dev);		// will auto generate a warm-start
		if (0) configureGeofence(&dev);

		Sleep(600);
#endif
		
#if 0
		ubx_msgDisableAll(&dev);
		//ubx_msgEnable(&dev, UBX_MON, UBX_MON_IO);
		ubx_msgPoll(&dev, UBX_MON, UBX_MON_VER);
		ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_USB);
		ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_PRT);
		
		ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_POSLLH);
		ubx_msgEnableEx(&dev, UBX_NAV, UBX_NAV_DOP, 14);
		ubx_msgEnableEx(&dev, UBX_NAV, UBX_NAV_PVT, 14);
		ubx_msgEnableEx(&dev, UBX_NAV, UBX_NAV_POSECEF, 14);
		ubx_msgEnableEx(&dev, UBX_NAV, UBX_NAV_SAT, 20);

		//ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_EOE);
		//ubx_msgEnableEx(&dev, UBX_NAV, UBX_NAV_GEOFENCE, 60);
		//ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_GEOFENCE);
		//ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_NAVX5);
		//ubx_msgEnableEx(&dev, UBX_NAV, UBX_NAV_SVINFO, 7);
		//ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_STATUS);
		//ubx_msgPoll(&dev, UBX_AID, UBX_AID_EPH);
		//ubx_msgPoll(&dev, UBX_AID, UBX_AID_ALM);
		//ubx_msgPoll(&dev, UBX_AID, UBX_AID_AOP);
#endif
		ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_GNSS);
		ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_NAV5);
		ubx_msgPoll(&dev, UBX_CFG, UBX_CFG_RATE);
		

		while(!kbhit()){
			// do something
			Sleep(50);
			if (++oncePerSecond >= 20){
				oncePerSecond = 0;
				//ubx_msgPoll(&dev, UBX_MON, UBX_MON_IO);
			}
		}
	
		WaitForSingleObject(hReadThread, INFINITE);
		CloseHandle(hReadThread);
	
		ubx_msgDisable(&dev, UBX_MON, UBX_MON_IO);
		//ubx_msgDisable(&dev, UBX_MON, UBX_MON_VER);
		//ubx_msgEnable(&dev, UBX_NAV, UBX_NAV_SAT);
		//Sleep(100);
		
		serialClean(&dev);
		serialClose(&dev);
	}else{
		printf("could not open serial port %i\n", COM_PORT);
	}
	
	
	return EXIT_SUCCESS;
};



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


#ifndef _UBXCB_H_
#define _UBXCB_H_





typedef struct {
	struct{
	  float longitude;
	  float latitude;
	  float altitude;		// hMSL
	}nav;
	
	struct{
	  uint8_t type;
	  uint8_t sats;			// Satellites used for this fix
	  uint8_t hAcc;
	  uint8_t vAcc;
	}fix;

	struct{
	  uint8_t horizontal;
	  uint8_t vertical;
	  uint8_t position;
	  uint8_t geometric;
	}dop;
	
	struct{
		uint16_t speed;		//		km/h   kilo meter per hour
		int16_t heading; 
	}misc;

	struct{
		uint16_t year;
		uint8_t month;
		uint8_t day;
	}date;
	
	struct{
		uint8_t hour;
		uint8_t min;
		uint8_t sec;
		uint8_t ms;			// ms*10 
	}time;
}gpsdata_t;



int ack_nak (const uint8_t *payload, uint16_t msg_len, void *opaque);
int ack_ack (const uint8_t *payload, uint16_t msg_len, void *opaque);

int aid_aop (const uint8_t *payload, uint16_t msg_len, void *opaque);
int aid_alm (const uint8_t *payload, uint16_t msg_len, void *opaque);
int aid_eph (const uint8_t *payload, uint16_t msg_len, void *opaque);

int cfg_nav5 (const uint8_t *payload, uint16_t msg_len, void *opaque);
int cfg_navx5 (const uint8_t *payload, uint16_t msg_len, void *opaque);

int nav_dop (const uint8_t *payload, uint16_t msg_len, void *opaque);
int nav_eoe (const uint8_t *payload, uint16_t msg_len, void *opaque);
int nav_pvt (const uint8_t *payload, uint16_t msg_len, void *opaque);
int nav_sat (const uint8_t *payload, uint16_t msg_len, void *opaque);
int nav_svinfo (const uint8_t *payload, uint16_t msg_len, void *opaque);
int nav_status (const uint8_t *payload, uint16_t msg_len, void *opaque);
int nav_posllh (const uint8_t *payload, uint16_t msg_len, void *opaque);
int rxm_sfrbx (const uint8_t *payload, uint16_t msg_len, void *opaque);

int cfg_geofence (const uint8_t *payload, uint16_t msg_len, void *opaque);
int nav_geofence (const uint8_t *payload, uint16_t msg_len, void *opaque);


int mon_ver (const uint8_t *payload, uint16_t msg_len, void *opaque);
int mon_io (const uint8_t *payload, uint16_t msg_len, void *opaque);

int inf_debug (const uint8_t *payload, uint16_t msg_len, void *opaque);
int cfg_inf (const uint8_t *payload, uint16_t msg_len, void *opaque);
int cfg_prt (const uint8_t *payload, uint16_t msg_len, void *opaque);
int cfg_usb (const uint8_t *payload, uint16_t msg_len, void *opaque);

#endif


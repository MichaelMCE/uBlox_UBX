
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





#include <string.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>


#include "ubx.h"
#include "ubxcb.h"
#include "extra.h"


static const char *GNSSIDs[8]		= {"GPS", "SBAS", "Galileo", "BeiDou", "IMES", "QZSS", "GLONASS", ""};
static const char *fixType[6]		= {"None", "Dead reckoning", "2D", "3D", "GNSS+deadReck", "Time only"};
static const char *geoConfidence[6]	= {"None", "68%", "95%", "99.7%", "99.9999%", "99.999999%"};
static const char *geoState[4]		= {"Unknown", "Inside", "Outside", ""};
//static const char *UTCStandard[8]	= {"Auto", "1", "2", "GPS", "4", "Galileo", "GLONASS", "BeiDou", "NavIC"};
static const char *dynModel[14]		= {"Portable", "1", "Stationary", "Pedestrian", "Automotive", "Sea", "Airborne < 1g", "Airborne < 2g", "Airborne < 4g", "Wrist", "MBike", "Lawn Mower", "Kick Scooter", ""};
static const char *fixMode[4]		= {"Unknown", "2D only", "3D only", "Auto 2D/3D"};
static const char *psmState[4]		= {"ACQUISITION", "TRACKING", "POWER OPTIMIZED TRACKING", "INACTIVE"};
static const char *spoofDetState[4]	= {"Unknown or deactivated", "No spoofing indicated", "Spoofing indicated", "Multiple spoofing indications"};
static const char *portId[8]        = {"I2C", "UART1", "UART2", "USB", "SPI", "USER0", "USER1", ""};
//static const char *status[2] = {"Disabled", "Enabled"};
static const char *timeRef[6] 		= {"UTC", "GPS", "GLONASS", "Beidou", "Galieo", "NavIC"};




static sat_stats_t sats;



int nav_sat (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	//printf("nav_sat %i\n", msg_len);
	
	const nav_sat_t *sat = (nav_sat_t*)payload;
	sats.numSvs = sat->numSvs;

	for (int i = 0; i < sat->numSvs; i++){
		const nav_sat_sv_t *sv = &sat->sv[i];
		
		sats.sv[i].gnssId = sv->gnssId;
		sats.sv[i].svId = sv->svId;
		sats.sv[i].cno = sv->cno;
		sats.sv[i].elev = sv->elev;
		sats.sv[i].azim = sv->azim;
		sats.sv[i].prRes = sv->prRes;
	}

#if 0
	printf(" iTow:    %i\n", sat->iTow);
	printf(" version: %i\n", sat->version);
	printf(" numSvs:  %i\n", sat->numSvs);

	for (int i = 0; i < sat->numSvs; i++){
		const nav_sat_sv_t *sv = &sat->sv[i];
		printf("  %i:\n", i);
		printf("   gnssId: %s\n", GNSSIDs[sv->gnssId&0x07]);
		printf("   svId:   %i\n", sv->svId);
		printf("   cno:    %i\n", sv->cno);
		printf("   elev:   %i\n", sv->elev);
		printf("   azim:   %i\n", sv->azim);
		printf("   prRes:  %.1f\n", sv->prRes/10.0f);
		printf("   flags:  %X\n", sv->flags);
	}
	printf("\n");
	printf("\n");
#endif
	
	return CBFREQ_NONE;
}

int nav_timebds (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
#if 0
	printf("nav_timebds %i\n", msg_len);

	const nav_timebds_t *bds = (nav_timebds_t*)payload;
	

	printf(" iTow:       %i\n", bds->iTow);
	printf(" SOW:        %i\n", bds->SOW);
	printf(" fSOW:       %ui\n", bds->fSOW);
	printf(" week:       %i\n", bds->week);
	printf(" leapS:      %i\n", bds->leapS);
	printf(" valid:      %i\n", bds->valid);
	printf(" tAcc:       %i\n", bds->tAcc);
	printf("\n");
#endif
	return CBFREQ_NONE;
}

int nav_svinfo (const uint8_t *payload, uint16_t msg_len, void *opaque)
{

//	printf("nav_svinfo %i\n", msg_len);
	
	const nav_svinfo_t *svinfo = (nav_svinfo_t*)payload;
	sats.numCh = svinfo->numCh;

	for (int i = 0; i < svinfo->numCh; i++){
		const nav_svinfo_chn_t *sat = &svinfo->sats[i];
		
		sats.sv[i].gnssId = 0;
		sats.sv[i].svId = sat->svid;
		sats.sv[i].cno = sat->cno;
		sats.sv[i].elev = sat->elev;
		sats.sv[i].azim = sat->azim;
		sats.sv[i].prRes = sat->prRes;
		
		sats.sv[i].flags = sat->flags;
		sats.sv[i].quality = sat->quality;
		sats.sv[i].chn = sat->chn;
	}


#if 0

	printf(" iTow:        %i\n", svinfo->iTow);
	printf(" numCh:       %i\n", svinfo->numCh);
	printf(" globalFlags: %X\n", svinfo->globalFlags);
	//printf("  0: %i\n", svinfo->reserverd1[0]);
	//printf("  2: %i\n", svinfo->reserverd1[1]);
	
	for (int i = 0; i < svinfo->numCh; i++){
		const nav_svinfo_chn_t *sat = &svinfo->sats[i];
		
		printf(" Sv: %i\n", i);
		printf("  chn:     %i\n", sat->chn);
		printf("  svid:    %i\n", sat->svid);
		printf("  flags:   %X\n", sat->flags);
		printf("  quality: %i\n", sat->quality);
		printf("  cno:     %i\n", sat->cno);
		printf("  elev:    %i\n", sat->elev);
		printf("  azim:    %i\n", sat->azim);
		printf("  prRes:   %.2f\n", dec32flt2(sat->prRes));
	}
	printf("\n");
#endif
	return CBFREQ_NONE;
}

int nav_posecef (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	const nav_posecef_t *cef = (nav_posecef_t*)payload;
	
	gpsdata_t *gps = (gpsdata_t*)opaque;
	gps->fix.pAcc = cef->pAcc;

#if 0
	printf("iTow: %u\n", cef->iTow);
	printf("ecefX: %i\n", cef->ecefX);
	printf("ecefY: %i\n", cef->ecefY);
	printf("ecefZ: %i\n", cef->ecefZ);
	printf("pAcc: %.2f\n", dec32flt2(cef->pAcc));
#endif	
	
	return CBFREQ_HIGH;
}

int nav_pvt (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	//printf("nav_pvt %i\n", msg_len);

	const nav_pvt_t *pvt = (nav_pvt_t*)payload;
	gpsdata_t *gps = (gpsdata_t*)opaque;
	
	gps->nav.longitude = dec32flt7(pvt->lon);
	gps->nav.latitude = dec32flt7(pvt->lat);
	gps->nav.altitude = dec32flt3(pvt->hMSL);


    if (pvt->flags&PVT_FLAGS_GNSSFIXOK)			// test for gnssFixOK
    	gps->fix.type = PVT_FIXTYPE_3D;
    else
    	gps->fix.type = 0;
    gps->fix.sats = pvt->numSv;
    gps->fix.hAcc = pvt->hAcc/10.0f;
    gps->fix.vAcc = pvt->vAcc/10.0f;


    gps->date.year = pvt->year;
	gps->date.month = pvt->month;
	gps->date.day = pvt->day;
	
	gps->time.hour = pvt->hour;
	gps->time.min = pvt->min;
	gps->time.sec = pvt->sec;
	gps->time.ms = dec32flt7(pvt->nano);
	
	gps->misc.speed = dec32flt3(pvt->gSpeed)*3600.0f;
	gps->misc.heading = dec32flt3(pvt->headMot);

#if 0
	printf(" Lon: %f\n", dec32flt7(pvt->lon));
	printf(" Lat: %f\n", dec32flt7(pvt->lat));
	printf(" Alt: %f\n", dec32flt3(pvt->hMSL));
	printf(" SVs: %i\n", pvt->numSv);
	
#elif 0
	printf(" iTow:    %i\n", pvt->iTow);
	printf(" year:    %i\n", pvt->year);
	printf(" month:   %i\n", pvt->month);
	printf(" day:     %i\n", pvt->day);
	printf(" hour:    %i\n", pvt->hour);
	printf(" min:     %i\n", pvt->min);
	printf(" sec:     %i\n", pvt->sec);
	printf(" valid:   %i\n", pvt->valid);
	printf(" tAcc:    %u\n", pvt->tAcc);
	printf(" nano:    %i\n", pvt->nano);
	printf(" fixType: %s\n", fixType[pvt->fixType]);
	printf(" flags:   %X\n", pvt->flags);
	printf(" flags2:  %X\n", pvt->flags2);
	printf(" numSv:   %i\n", pvt->numSv);
	printf(" lon:     %.8f\n", dec32flt7(pvt->lon));
	printf(" lat:     %.8f\n", dec32flt7(pvt->lat));
	printf(" height:  %f\n", dec32flt3(pvt->height));
	printf(" hMSL:    %f\n", dec32flt3(pvt->hMSL));
	printf(" hAcc:    %f\n", dec32flt3(pvt->hAcc));
	printf(" vAcc:    %f\n", dec32flt3(pvt->vAcc));
	printf(" velN:    %f\n", dec32flt3(pvt->velN));
	printf(" velE:    %f\n", dec32flt3(pvt->velE));
	printf(" velD:    %f\n", dec32flt3(pvt->velD));
	printf(" gSpeed:  %f\n", dec32flt3(pvt->gSpeed));
	printf(" headMot: %f\n", dec32flt5(pvt->headMot));
	
	printf(" sAcc:    %f\n", dec32flt3(pvt->sAcc));
	printf(" headAcc: %f\n", dec32flt5(pvt->headAcc));
	printf(" pDop:    %f\n", dec32flt2(pvt->pDop));
	
	//for (int i = 0; i < sizeof(pvt->reserved1); i++)
		//printf("  %i: %i\n", i, pvt->reserved1[i]);
	
	printf(" headVeh: %f\n", dec32flt5(pvt->headVeh));
	printf(" magDec:  %f\n", dec32flt2(pvt->magDec));
	printf(" magAcc:  %f\n", dec32flt2(pvt->magAcc));
	printf("\n");
#endif	

	return CBFREQ_HIGH;
}

int ack_ack (const uint8_t *payload, uint16_t msg_len, void *opaque)
{   
	//printf("ack_ack %i\n", msg_len);
	const ack_ack_t *ack = (ack_ack_t*)payload;
	
	printf("\nack_ack: %.2X/%.2X\n\n", ack->clsId, ack->msgId);
	
	return CBFREQ_NONE;
}

int ack_nak (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	//printf("ack_nak %i\n", msg_len);
	const ack_nak_t *nak = (ack_nak_t*)payload;
	
	printf("\nack_nak: %.2X/%.2X\n\n", nak->clsId, nak->msgId);

	return CBFREQ_NONE;
}


int nav_status (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("nav_status %i\n", msg_len);
	const nav_status_t*status = (nav_status_t*)payload;

	printf(" iTow:    %u\n", status->iTow);
	printf(" fixType: %s\n", fixType[status->gpsFix]);
	printf(" flags:   %X\n", status->flags);
	printf(" fixStat: %X\n", status->fixStat);
	printf(" flags2 - psmState:      %s\n", psmState[status->flags2&STATUS_FLAGS2_PSMSTATE]);
	printf(" flags2 - spoofDetState: %s\n", spoofDetState[(status->flags2&STATUS_FLAGS2_SPOOFDETSTATE)>>3]);
	printf(" ttff:    %ims\n", status->ttff);
	printf(" msss:    %ims\n", status->msss);
	printf("\n");

	return CBFREQ_NONE;
}

int nav_eoe (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	//printf("nav_eoe %i\n", msg_len);
	//const nav_eoe_t *eoe = (nav_eoe_t*)payload;	

	return CBFREQ_HIGH;
}

int aid_eph (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("aid_eph %i\n", msg_len);
	
	const aid_eph_t *eph = (aid_eph_t*)payload;
	
	printf(" SvId: %i\n", eph->svid);
	printf(" how:  %X\n", eph->how);
	
	if (msg_len == 104){	// 104 as per ublox8 ubx spec PDF (33.9.3.3)
		for (int i = 0; i < 8; i++) printf(" %.8X", eph->sf1d[i]);
		printf("\n");
		for (int i = 0; i < 8; i++) printf(" %.8X", eph->sf2d[i]);
		printf("\n");
		for (int i = 0; i < 8; i++) printf(" %.8X", eph->sf3d[i]);
		printf("\n");
	}
	printf("\n");	
	
	return CBFREQ_NONE;
}

int aid_alm (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("aid_alm %i\n", msg_len);

	const aid_alm_t *alm = (aid_alm_t*)payload;
	
	printf(" SvId: %i\n", alm->svid);
	printf(" week: %i\n", alm->week);
	
	if (msg_len == 40){
		for (int i = 0; i < 8; i++) printf(" %.8X", alm->dwrd[i]);
		printf("\n");
	}
	printf("\n");
	
	return CBFREQ_NONE;
}


int aid_aop (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("aid_aop %i\n", msg_len);

	const aid_aop_t *aop = (aid_aop_t*)payload;
	
	printf(" gnssId: %s\n", GNSSIDs[aop->gnssId&0x07]);
	printf(" svId:   %i\n", aop->svId);
	
	if (msg_len == 68){
		for (int i = 0; i < 64; i++) printf(" %.2X", aop->data[i]);
		printf("\n");
	}
	printf("\n");
	return CBFREQ_NONE;
}

int nav_dop (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	//printf("nav_dop %i\n", msg_len);

	const nav_dop_t *dop = (nav_dop_t*)payload;
	gpsdata_t *gps = (gpsdata_t*)opaque;
	
	gps->dop.horizontal = dop->hDOP;
	gps->dop.vertical = dop->vDOP;
	gps->dop.position = dop->pDOP;
	gps->dop.geometric = dop->gDOP;

#if 0
	printf(" iTow: %i\n", dop->iTow);
	printf(" gDOP: %.2f\n", dec32flt2(dop->gDOP));
	printf(" pDOP: %.2f\n", dec32flt2(dop->pDOP));
	printf(" tDOP: %.2f\n", dec32flt2(dop->tDOP));
	printf(" vDOP: %.2f\n", dec32flt2(dop->vDOP));
	printf(" hDOP: %.2f\n", dec32flt2(dop->hDOP));
	printf(" nDOP: %.2f\n", dec32flt2(dop->nDOP));
	printf(" eDOP: %.2f\n", dec32flt2(dop->eDOP));
	printf("\n");
#endif
	
	return CBFREQ_NONE;
}

int nav_posllh (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	//printf("nav_posllh %i\n", msg_len);
	
	const nav_posllh_t *posllh = (nav_posllh_t*)payload;
	gpsdata_t *gps = (gpsdata_t*)opaque;


	gps->nav.longitude = dec32flt7(posllh->lon);
	gps->nav.latitude = dec32flt7(posllh->lat);
	gps->nav.altitude = dec32flt3(posllh->hMSL);
	
	gps->fix.hAcc = posllh->hAcc/10.0f;
    gps->fix.vAcc = posllh->vAcc/10.0f;
    
	gps->time.hour = (((posllh->iTow/1000)/60)/60)%24;
	gps->time.min = ((posllh->iTow/1000)/60)%60;
	gps->time.sec = ((posllh->iTow/1000)%60)*0.6f;
    gps->time.ms = (posllh->iTow%1000)/10;


#if 0
	printf(" iTow:   %u\n", posllh->iTow);
	printf(" lon:    %.8f\n", dec32flt7(posllh->lon));
	printf(" lat:    %.8f\n", dec32flt7(posllh->lat));
	printf(" height: %.3f\n", dec32flt3(posllh->height));
	printf(" hMSL:   %.3f\n", dec32flt3(posllh->hMSL));
	printf(" hAcc:   %.3f\n", dec32flt3(posllh->hAcc));
	printf(" vAcc:   %.3f\n", dec32flt3(posllh->vAcc));
	printf("\n");
#endif

	return CBFREQ_NONE;
}

int mon_ver (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("mon_ver %i\n", msg_len);
	
	if (msg_len < 40 || (msg_len%30 != 10)){
		printf("mon_ver: msg corrupt\n");
		return CBFREQ_INVALID;
	}
	const int32_t tInfo = (msg_len - 40) / 30;
	if (tInfo > 0){
		
		const mon_ver_t *ver = (mon_ver_t*)payload;
		printf("'%s'\n", ver->swVersion);
		printf("'%s'\n", ver->hwVersion);

		uint8_t *str = (uint8_t*)ver->extension;
		for (int i = 0; i < tInfo; i++){
			printf("  %i: '%s'\n", i, str);
			str += 30;
		}
	}
	
	return CBFREQ_NONE;
}

int mon_io (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
#if 0
	//printf("mon_io %i\n", msg_len);
	
	const mon_io_t *io = (mon_io_t*)payload;
	const int tPorts = msg_len / 20;
	
	if (tPorts < 1 || (msg_len%20)){		// according to ubx protocol 18
		printf("mon_io: msg corrupt\n");
		return CBFREQ_INVALID;
	}
	
	static size_t txBytesPre = 0;	
	const mon_io_port_t *port = &io->port1;
	
	for (int i = 0; i < tPorts; i++, port++){
		if (port->rxBytes || port->txBytes){
			printf("  port:        %s\n", portId[i&0x07]);
			
			printf("  rxBytes:     %i\n", port->rxBytes);
			printf("  txBytes:     %i\n", port->txBytes);
			printf("  parityErrs:  %i\n", port->parityErrs);
			printf("  framingErrs: %i\n", port->framingErrs);
			printf("  overrunErrs: %i\n", port->overrunErrs);
			printf("  breakCond:   %i\n", port->breakCond);
			printf("  rxBusy:      %i\n", port->rxBusy);
			printf("  txBusy:      %i\n", port->txBusy);
			
			printf("  txBytes per second: %i\n", port->txBytes - txBytesPre);
			txBytesPre = port->txBytes;
			printf("\n");
		}
	}
#endif
	return CBFREQ_MEDIUM;
}

int rxm_sfrbx (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	//printf("rxm_sfrbx %i\n", msg_len);
	
	const rxm_sfrbx_t *sfrbx = (rxm_sfrbx_t*)payload;
	
	printf("gnssId:     %i\n", sfrbx->gnssId);
	printf("svId:       %i\n", sfrbx->svId);
	printf("reserved1:  %i\n", sfrbx->reserved1);
	printf("freqId:     %i\n", sfrbx->freqId);
	printf("numWords:   %i\n", sfrbx->numWords);
	printf("chn:        %i\n", sfrbx->chn);
	printf("version:    %i\n", sfrbx->version);
	printf("reserved2:  %i\n", sfrbx->reserved2);
	
	for (int i = 0; i < sfrbx->numWords; i++)
		printf("%X ", sfrbx->dwrd[i]);
	printf("\n\n");
	
	return CBFREQ_NONE;
}

int cfg_geofence (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("\ncfg_geofence %i\n", msg_len);
	
	const cfg_geofence_t *geo = (cfg_geofence_t*)payload;
	
	printf(" version:     %X\n", geo->version);
	printf(" numFences:   %i\n", geo->numFences);
	printf(" confLvl:     %i - %s\n", geo->confLvl, geoConfidence[geo->confLvl]);
	printf(" pioEnabled:  %i\n", geo->pioEnabled);
	printf(" pinPolarity: %i\n", geo->pinPolarity);
	printf(" pin: %i\n", geo->pin);

	
	for (int i = 0; i < geo->numFences; i++){
		printf("  fence %i: lat:    %.8f\n", i+1, dec32flt7(geo->fence[i].lat));
		printf("  fence %i: lon:    %.8f\n", i+1, dec32flt7(geo->fence[i].lon));
		printf("  fence %i: radius: %.2fm\n",i+1, dec32flt2(geo->fence[i].radius));
	}
	printf("\n");
	
	return CBFREQ_NONE;
}

int nav_geofence (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("\nnav_geofence %i\n", msg_len);
	
	const nav_geofence_t *geo = (nav_geofence_t*)payload;
	
	printf("iTow:      %u\n", geo->iTow);
	printf("version:   %X\n", geo->version);
	printf("status:    %i\n", geo->status);
	printf("numFences: %i\n", geo->numFences);
	printf("combState: %i\n", geo->combState);
	
	for (int i = 0; i < geo->numFences; i++)
		printf("  fence %i: state: %i - %s\n",i+1, geo->fence[i].state, geoState[geo->fence[i].state&0x03]);
	printf("\n");
	
	return CBFREQ_NONE;
}	

int inf_debug (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("\ninf_debug %i\n", msg_len);

	uint8_t msg[msg_len+1];
	memcpy(msg, payload, msg_len);
	msg[msg_len] = 0;
	
	printf("## \"%s\"\n\n", msg);

	return CBFREQ_NONE;	
}

int cfg_rate (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("\ncfg_rate %i\n", msg_len);
	
	const cfg_rate_t *rate = (cfg_rate_t*)payload;

	printf(" measRate: %i\n", rate->measRate);
	printf(" navRate: %i\n", rate->navRate);
	
	if (rate->timeRef < sizeof(timeRef) / sizeof(*timeRef))
		printf(" timeRef: %s\n", timeRef[rate->timeRef]);
	else
		printf(" timeRef invalid\n");

	printf("\n");
	
	return CBFREQ_NONE;	
}

int cfg_inf (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("\ncfg_inf %i\n", msg_len);
	
	const cfg_inf_t *inf = (cfg_inf_t*)payload;

	printf(" protocolID: %i\n", inf->protocolID);
	printf(" infMsgMask I2C:   %X\n", inf->infMsgMask[CFG_PORTID_I2C]);
	printf(" infMsgMask UART1: %X\n", inf->infMsgMask[CFG_PORTID_UART1]);
	printf(" infMsgMask UART2: %X\n", inf->infMsgMask[CFG_PORTID_UART2]);
	printf(" infMsgMask USB:   %X\n", inf->infMsgMask[CFG_PORTID_USB]);
	printf(" infMsgMask SPI:   %X\n", inf->infMsgMask[CFG_PORTID_SPI]);
	
	printf("\n");

	return CBFREQ_NONE;	
}

int cfg_nav5 (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	//printf("\ncfg_nav5 %i\n", msg_len);
	
	const cfg_nav5_t *nav = (cfg_nav5_t*)payload;
	char str[64] = {0};	

#if 0	
	printf(" mask:              %X\n", nav->mask);
	printf(" dynModel:          %s\n", dynModel[nav->dynModel]);
	printf(" fixMode:           %s\n", fixMode[nav->fixMode&0x03]);
	printf(" fixedAlt:          %.2f\n", dec32flt2(nav->fixedAlt));
	printf(" fixedAltVar:       %.4f\n", dec32flt4(nav->fixedAltVar));
	printf(" minElv:            %i\n", nav->minElv);
	printf(" drLimit:           %i\n", nav->drLimit);
	printf(" pDop:              %.1f\n", nav->pDop/10.0f);
	printf(" tDop:              %.1f\n", nav->tDop/10.0f);
	printf(" pAcc:              %i\n", nav->pAcc);
	printf(" tAcc:              %i\n", nav->tAcc);
	printf(" staticHoldThresh:  %i\n", nav->staticHoldThresh);
	printf(" dynssTimeout:      %i\n", nav->dynssTimeout);
	printf(" cnoThreshNumSVs:   %i\n", nav->cnoThreshNumSVs);
	printf(" cnoThresh:         %i\n", nav->cnoThresh);
	printf(" pAccADR:           %i\n", nav->pAccADR);
	printf(" staticHoldMaxDist: %i\n", nav->staticHoldMaxDist);
	printf(" utcStandard:       %s\n", UTCStandard[nav->utcStandard&0x07]);
	printf("\n");
#endif

	snprintf(str, sizeof(str), "Model: %s", dynModel[nav->dynModel]);
	printf("%s\n", str);
	snprintf(str, sizeof(str), "FixMode: %s", fixMode[nav->fixMode&0x03]);
	printf("%s\n", str);
	printf("\n");
	
	return CBFREQ_NONE;	
}

int cfg_navx5 (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("\ncfg_navx5 %i\n", msg_len);
	
	const cfg_navx5_t *nav = (cfg_navx5_t*)payload;

	printf(" version:          %i\n", nav->version);
	printf(" mask1:            %X\n", nav->mask1);
	printf(" mask2:            %X\n", nav->mask2);
	printf(" minSVs:           %i\n", nav->minSVs);
	printf(" maxSVs:           %i\n", nav->maxSVs);
	printf(" minCNO:           %i\n", nav->minCNO);
	printf(" reserved2:        %i\n", nav->reserved2);
	printf(" iniFix3D:         %i\n", nav->iniFix3D);
	printf(" ackAiding:        %i\n", nav->ackAiding);
	printf(" wknRollover:      %i\n", nav->wknRollover);
	printf(" sigAttenCompMode: %i\n", nav->sigAttenCompMode);
	printf(" reserved4:        %i\n", nav->reserved4);
	printf(" usePPP:           %i\n", nav->usePPP);
	printf(" aopCfg:           %X\n", nav->aopCfg);
	printf(" aopOrbMaxErr:     %i\n", nav->aopOrbMaxErr);
	printf(" useAdr:           %i\n", nav->useAdr);
	printf("\n");
	
	return CBFREQ_NONE;	
}

int cfg_gnss (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	//printf("\ncfg_gnss %i\n", msg_len);
	
	const cfg_gnss_t *gnss = (cfg_gnss_t*)payload;
	char str[64] = {0};
	
	for (int i = 0; i < gnss->numConfigBlocks; i++){
		const cfg_cfgblk_t *blk = &gnss->cfgblk[i];
		//printf("%i: %s - %s\n", blk->gnssId, GNSSIDs[blk->gnssId&0x07], status[(blk->flags&GNSS_CFGBLK_ENABLED&0x01)]);

		if (blk->flags&GNSS_CFGBLK_ENABLED){
			if (str[0])
				strcat(str, ", ");
			else
				strcat(str, "Enabled: ");
			strcat(str, GNSSIDs[blk->gnssId&0x07]);
		}
		
	}
	printf("%s\n", str);
	//printf("\n");
	
	return CBFREQ_NONE;	
}
	
int cfg_prt (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("\ncfg_prt %i\n", msg_len);
	
	const cfg_prt_t *prt = (cfg_prt_t*)payload;
	printf(" portId:       %s\n", portId[prt->port.id&0x07]);

	if (prt->port.id == CFG_PORTID_UART1 || prt->port.id == CFG_PORTID_UART2){
		const cfg_prt_uart_t *uart = (cfg_prt_uart_t*)payload;	// or use 'prt.uart.'

		printf(" reserved1:    %i\n", uart->reserved1);
		printf(" txReady:      %.4X - en=%i, pol=%i, pin=%i, thres=%i\n", uart->txReady.flags, 
			uart->txReady.bits.en, uart->txReady.bits.pol, uart->txReady.bits.pin, uart->txReady.bits.thres);
		printf(" mode:         %.8X - charLen=%X, partity=%X, nStopBits=%X, bitOrder=%X\n", uart->mode.flags, 
			uart->mode.bits.charLen, uart->mode.bits.partity, uart->mode.bits.nStopBits, uart->mode.bits.bitOrder);

		printf(" baudRate:     %i\n", uart->baudRate);
		printf(" inProtoMask:  %.4X - ", uart->inProtoMask);
		if (uart->inProtoMask&CFG_PROTO_UBX)   printf("UBX ");
		if (uart->inProtoMask&CFG_PROTO_NMEA)  printf("NMEA ");
		if (uart->inProtoMask&CFG_PROTO_RTCM2) printf("RTCM2 ");
		if (uart->inProtoMask&CFG_PROTO_RTCM3) printf("RTCM3 ");
		printf("\n");

		printf(" outProtoMask: %.4X - ", uart->outProtoMask);
		if (uart->outProtoMask&CFG_PROTO_UBX)   printf("UBX ");
		if (uart->outProtoMask&CFG_PROTO_NMEA)  printf("NMEA ");
		if (uart->outProtoMask&CFG_PROTO_RTCM2) printf("RTCM2 ");
		if (uart->outProtoMask&CFG_PROTO_RTCM3) printf("RTCM3 ");
		printf("\n");
		
		printf(" flags:        %.4X\n", uart->flags);
	}
	printf("\n");
	
	return CBFREQ_NONE;	
}

int cfg_usb (const uint8_t *payload, uint16_t msg_len, void *opaque)
{
	printf("\ncfg_usb %i\n", msg_len);
	
	const cfg_usb_t *usb = (cfg_usb_t*)payload;
	
	if (msg_len == 108){		// 108 as per ubx proto18 spec
		printf(" vendorID:         %.4X\n", usb->vendorID);
		printf(" productID:        %.4X\n", usb->productID);
		printf(" powerConsumption: %ima\n", usb->powerConsumption);
		printf(" flags:            %.4X\n", usb->flags);
		printf(" vendorString:    '%s'\n",  usb->vendorString);
		printf(" productString :  '%s'\n",  usb->productString);
		printf(" serialNumber:    '%s'\n",  usb->serialNumber);
	}
	return CBFREQ_NONE;	
}

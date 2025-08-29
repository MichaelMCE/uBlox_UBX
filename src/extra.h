

#ifndef _EXTRA_H_
#define _EXTRA_H_


typedef struct {
	uint8_t gnssId;				// GNSSID_
	uint8_t svId;
	uint8_t cno;
	int8_t elev;
	
	int16_t azim;
	int16_t prRes;
	
	uint8_t chn;
	uint8_t quality;			// SVINFO_QUALITY_
	uint8_t flags;
	uint8_t stub;
}sat_status_t;

typedef struct {
	uint8_t numSvs;
	uint8_t numCh;
	uint8_t stub[2];
	
	sat_status_t sv[72];
}sat_stats_t;


sat_stats_t *getSats ();



typedef struct {
	uint16_t Yr;
	uint16_t Mth;
	uint16_t Day;
	uint16_t Hm;
}utcTime_t;



#endif


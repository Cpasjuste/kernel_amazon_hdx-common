/*
 * dsmParamStruc.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Yuancheng.Zheng
 */

#ifndef DSMPARAMSTRUC_H_
#define DSMPARAMSTRUC_H_


#ifdef __cplusplus
extern "C"
{
#endif

/** \brief Structure for DSM filter coefficients for DSM module */
typedef struct dsm_FB_params_t dsm_FB_params_t;
struct dsm_FB_params_t {
	  /** \ Sequence of DSM filter parameters */
	  uint32_t dcResistance;
	  uint32_t coilTemp;
	  uint32_t qualityfactor;
	  uint32_t resonanceFreq;
	  uint32_t excursionMeasure;
};

typedef struct dsm_control_params_t dsm_control_params_t;
struct dsm_control_params_t {
	 /** \ sequence of control parameters */
	  uint32_t rdcroomtemp;
	  uint32_t releasetime;
	  uint32_t coilthermallimit;
	  uint32_t excursionlimit;
	  uint32_t dsmenabled;
	  uint32_t staticgain;
	  uint32_t lfxgain;
	  uint32_t pilotgain;
	  uint32_t flagToWrite;
	  uint32_t featureSetEnable;
	  uint32_t smooFacVoltClip;
	  uint32_t highPassCutOffFactor;
};

typedef struct dsm_warning_flag_t	dsm_warning_flag_t;
struct dsm_warning_flag_t{
	union{
		struct{
			uint32_t	coilTempWarning:1;
			uint32_t	RdcWarning:1;
			uint32_t	QcWarning:1;
			uint32_t	FcWarning:1;
			uint32_t	ExcWarning:1;
			uint32_t	VWarning:1;
			uint32_t	IWarning:1;
		};
		uint32_t		isWarning;
	};

    //data while last error happens
    float	wrongV;
    float	wrongI;
    float	wrongRdc;
    float	wrongQc;
    float	wrongFc;
    float	wrongExc;
    float	wrongCoilTemp;
};

typedef struct dsm_new_params_t dsm_new_params_t;
struct dsm_new_params_t {
	long	endianess;//=='DEADBEEF'
	struct{
		long    setCalibratedData:1;
		long	bypass:1;
		long	clearWarningFlag:1;
		long	dump_PCM:1;
		long	dump_IV:1;
	};
	union{
		struct{
			long	checkI:1;
			long	checkV:1;
			long	checkRdc:1;
			long	checkQc:1;
			long	checkFc:1;
			long	checkExc:1;
			long	checkCoilTemp:1;
		};
		long	isCheck;
	};
	long	channelIndex;//1=left channel; 2=right channel

	//calibrated parameters
    float    room_temperature;
    float    rdc_calib_left_chan;
    float    rdc_calib_right_chan;

    //internal parameters
    float	leadRdc;

    //restriction parameters
    float	minCoilTemp;
    float	maxCoilTemp;
    float	minRdc;
    float	maxRdc;
    float	minQc;
    float	maxQc;
    float	minFc;
    float	maxFc;
    float	minExc;
    float	maxExc;
    float	minV;
    float	maxV;
    float	minI;
    float	maxI;

    //warning flags
    dsm_warning_flag_t	leftChan_warning;
    dsm_warning_flag_t	rightChan_warning;
};

typedef struct DSM_Params_Ext_t DSM_Params_Ext_t;
struct DSM_Params_Ext_t {
	  /** DSM feedback parameters of left channel*/
	dsm_FB_params_t			fb_left;

	 /** DSMcontrol parameters */
	dsm_control_params_t	cntrlParams;

	/** DSM feedback paramters of right channel*/
	dsm_FB_params_t			fb_right;

	dsm_new_params_t		newParams;

};

#define DSM_LEFT_CHAN_ID		1
#define DSM_RIGHT_CHAN_ID		2

// define the status of music playing
typedef enum{
	MAXIM_PCM_STOPPED,
	MAXIM_PCM_PLAYING,
	MAXIM_PCM_PAUSED,
	MAXIM_PCM_RESUM,
	MAXIM_PCM_CLOSED
} Maxim_PCM_Status;

#ifdef __cplusplus
}
#endif

#endif /* DSMPARAMSTRUC_H_ */

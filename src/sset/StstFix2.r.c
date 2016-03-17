#include <stdio.h>
#include <sys/types.h>
#include "../hdr/sys.h"
#include "../hdr/cnf.h"
#include "../hdr/proc.h"
#include "../hdr/cdsp.h"
#include "../hdr/idsp.h"
#include "../hdr/buf.h"
#include "../hdr/menu.h"
#include "../hdr/state.h"
#include "../hdr/ecode.h"
#include "../hdr/device.h"
#include "../hdr/int.h"
#include "StstFix2.r.h"
#include "pcsSocket.h"
#include "GLcommand.h"
#include "lcode_tst.h"
#include "ldev_tst.h"
#include "position.h"
#define WIND0 0
#define WIND1 1

/*
 * Subroutine that determines the position of the
 * fixation target
 */
int trialCode;
int fixOn = OBJ_ON;
int fixOff = OBJ_OFF;
float fpx;
float fpy;
int errlst[NUM_POS * 2] = { 0 };
int trlcntr = -1;
int curstm = 0;
int range = 1.0;		/* scale factor to increase range of fixation point positions */
int stmerr = 0;
int fixerr = 0;
int cyc_total = 0;	/* total number of stimulus cycles */

int pick_fp_location()
{
	static int ptrlst[NUM_POS * 2] = { 0 };
	static int rs_shift = 10;
	int i, j;
	
	if(!range) range = 1;
	if(--trlcntr <= NUM_POS) {
		if (stmerr > 10 ) {	/* present errors if more than 10 */
			trlcntr = stmerr;
			for (j = 0; j < trlcntr; j++) ptrlst[j] = errlst[j];
			fixerr++;
		}

		/* If there were 10 or less errors, shuffle the error */
		/* conditions into the next block of trials. */
		else {
			trlcntr = NUM_POS + stmerr;
			for (i = 0; i < NUM_POS; i++ ) ptrlst[i] = i;
			for (j = 0; j < stmerr; j++) ptrlst[i + j] = errlst[j];
			fixerr = 0;
		}

		/* reset the error counter after moving error conditions */
		/* into the trial block */
		stmerr = 0;

		/* randomize the stimulus conditions in the trial block */
		shuffle(trlcntr, rs_shift, ptrlst);

		/* if the fixerr flag is 0, increment the counter that */
		/* keeps track of the number of blocks of trials completed */
		if(!fixerr) {
			cyc_total++;
		}
	}

	curstm = ptrlst[trlcntr-1];
	fpx = stm_list[curstm].x;
	fpy = stm_list[curstm].y;
	trialCode = 2000 + curstm;
	
	return(0);
}

/*
 * Subroutine that controls window size and counts total trials
 */
long wndsiz = 20;
int trials = 0;
int otheyeflag = 0;

int set_wnd()
{
	static long wndxctr = 0;
	static long wndyctr = 0;

	trials++;
	wndxctr = (long) stm_list[curstm].x;
	wndyctr = (long) stm_list[curstm].y;
	if(!otheyeflag) {
		wd_pos(WIND0, wndxctr, wndyctr);
		wd_disp(D_W_EYE_X);
		wd_siz(WIND0, wndsiz, wndsiz);
		wd_cntrl(WIND0, WD_ON);
	}
	else {
		wd_pos(WIND1, wndxctr, wndyctr);
		wd_disp(D_W_EYE_O);
		wd_siz(WIND1, wndsiz, wndsiz);
		wd_cntrl(WIND1, WD_ON);
	}
	return (0);
}

int beye = 0;
int mistake()
{
	errlst[stmerr++] = curstm;
	beye++;
	return(0);
}

void rinitf(void)
{
	/* char *vexHost = "bl-opt-vex"; */
	char *vexHost = "lsr-jwmvex";
	/*char *mexHost = "lsr-labmex"; */

	pcsConnectVex(vexHost);
	/*pcsConnectMex(mexHost);*/
	
	wd_src_pos(WIND0, WD_DIRPOS, 0, WD_DIRPOS, 0);
	wd_src_pos(WIND1, WD_DIRPOS, 0, WD_DIRPOS, 0);
	wd_src_check(WIND0, WD_SIGNAL, 0, WD_SIGNAL, 1);
	wd_src_check(WIND1, WD_SIGNAL, 2, WD_SIGNAL, 3);
	wd_cntrl(WIND0, WD_ON);
	wd_cntrl(WIND1, WD_ON);
}

// BEGIN GENERATED CODE

VLIST state_vl[] = {
	{"win siz",	&wndsiz, NP, NP, 0, ME_DEC}, // 0 0
	{"trials",	&trials, NP, NP, 0, ME_DEC}, // 0 35
	{"other_eye?",	&otheyeflag, NP, NP, 0, ME_DEC}, // 0 70
	{"range",	&range, NP, NP, 0, ME_DEC}, // 0 105
	{"total_cycles",	&cyc_total, NP, NP, 0, ME_DEC}, // 0 140
	NS
};
char hm_sv_vl[] = "";

MENU umenus[] = {
	{"state_vars",	&state_vl, NP, NP, 0, NP, hm_sv_vl}, // 1256, 413, 150, 350, 1
	NS,
};

USER_FUNC ufuncs[] = { // 1277 11 150 350 1
	{""},
};

RTVAR rtvars[] = { // 1084 8 150 350 1
	{""},
};

STATE sfirst = {  // 162 0
	{"first"},0,0,
	0,0,
	{0,0,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&sdisabl,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,0
};
STATE sdisabl = {  // 162 35
	{"disabl"},0,0,
	0,0,
	{0,0,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&senable,BITOFF,&softswitch,PSTOP},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,35
};
STATE senable = {  // 162 70
	{"enable"},ENABLECD,0,
	0,0,
	{0,0,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&spckfix,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,70
};
STATE spckfix = {  // 162 105
	{"pckfix"},0,&trialCode,
	0,0,
	{pick_fp_location,0,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&ssetfp,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,105
};
STATE ssetfp = {  // 162 140
	{"setfp"},1500,0,
	0,0,
	{PvexSetFixLocation,&fpx,&fpy,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&sawnopn,FUNC,&tst_rx_new,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,140
};
STATE sawnopn = {  // 162 175
	{"awnopn"},0,0,
	0,0,
	{awind,OPEN_W,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&sfpncmd,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,175
};
STATE sfpncmd = {  // 162 210
	{"fpncmd"},3232,0,
	0,0,
	{PvexSwitchFix,&fixOn,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&sfpon,FUNC,&tst_rx_new,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,210
};
STATE sfpon = {  // 162 245
	{"fpon"},FPONCD,0,
	0,0,
	{0,0,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&swinset,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,245
};
STATE swinset = {  // 162 280
	{"winset"},0,0,
	0,0,
	{set_wnd,0,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&seyein,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,280
};
STATE seyein = {  // 162 315
	{"eyein"},0,0,
	0,0,
	{0,0,0,0,0,0,0,0,0,0,0},
	1000,500,
	{
		{0,0,&sfpdcmd,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,315
};
STATE sfpdcmd = {  // 162 350
	{"fpdcmd"},0,0,
	0,0,
	{PvexDimFix,0,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&sfpdim,FUNC,&tst_rx_new,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,350
};
STATE sfpdim = {  // 162 385
	{"fpdim"},FPDIMCD,0,
	0,0,
	{0,0,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&seyeout,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,385
};
STATE seyeout = {  // 162 420
	{"eyeout"},0,0,
	0,0,
	{0,0,0,0,0,0,0,0,0,0,0},
	100,0,
	{
		{0,0,&sclosew,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,420
};
STATE sclosew = {  // 162 455
	{"closew"},0,0,
	0,0,
	{awind,CLOSE_W,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&scorrect,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,455
};
STATE scorrect = {  // 162 490
	{"correct"},0,0,
	0,0,
	{score,YES,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&sfpfcmd,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,490
};
STATE sfpfcmd = {  // 162 525
	{"fpfcmd"},0,0,
	0,0,
	{PvexSwitchFix,&fixOff,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,0,&sfpoff,FUNC,&tst_rx_new,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,525
};
STATE sfpoff = {  // 162 560
	{"fpoff"},FPOFFCD,0,
	0,0,
	{0,0,0,0,0,0,0,0,0,0,0},
	1000,0,
	{
		{0,0,&sfirst,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
	},
	0,127,128,162,560
};


STATE *snames[] = {
	&sfirst, 
	&sdisabl, 
	&senable, 
	&spckfix, 
	&ssetfp, 
	&sawnopn, 
	&sfpncmd, 
	&sfpon, 
	&swinset, 
	&seyein, 
	&sfpdcmd, 
	&sfpdim, 
	&seyeout, 
	&sclosew, 
	&scorrect, 
	&sfpfcmd, 
	&sfpoff, 
0};


STATE *aborta[] = {
	0};

int sf_init();

STATE sps_state;

AWAKE init_state[] = {
	{0,ON,ON,&sps_state,&sps_state,0,(STATE *)&init_state[1],0, &sps_state.escape},
	{0,0,0,0,0,0,0,0,&sps_state.escape[1]}
};
STATE sps_state = {
	{"spec"},1,0,1,0,
	{sf_init,100,0,0,0,0,0,0,0,0,0},
	0,0,
	{
		{0,init_state,&sps_state,TIME,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0},
		{0,0,0,0,0,0}
	}
};
AWAKE nowstate[] = {
	{0,ON,ON,&sfirst,&sfirst,aborta,0,0,0}, // main_set 0 0 631 998 1
	{0,0,0,0,0,0,0,0,0}
};

int (*init_list[])() = {
	0
};
/**********************************
APPLICATION PROPERTIES
PARADIGM_NUMBER 100
BASE_WINDOW 1455 1143
WIDGET_DIALOG 800 0
RESOURCE_DIALOG 700 300 351 365
CHAIN main_set
CHAIN_DONE
MENU state_vars
MENU_DONE
FUNCTION User Functions
FUNCTION_DONE
VARIABLE Real Time Variables
VARIABLE_DONE
APP_DONE
 **********************************/
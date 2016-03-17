/* $Id: ecal.d,v 1.3 2007/06/25 21:30:43 dan Exp $ */

/*  
 * Paradigm for fixation (and eventually saccade) training. Allows the
 * experimenter to control background luminance and fixation spot size
 * and color.
 *
 * This version updated to be compatible with Rex version 5.4
 * KHB, 12/21/95
 * 
 * Saccade.d generated from fix.d (and bits of the old saccade.d) 2/16/96
 *
 * ecal.d copied from saccade.d, updated to use new render. 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


#include "ldev_tst.h"	/* hardware device defines */
#include "lcode_tst.h"	/* Ecodes */

#include "actions.h"	/* includes all files needed for use with new render. */

/* TODO - I don't think this is needed here, besides I can't find the file. */
/* #include "ivunif.c"	*/ /* the random-number generator */

/*
 * codes local to Gabor stimulus paradigm.
 */
#define FALSE 0
#define TRUE (~FALSE)  
#define PI 3.14159265

#define WIND0	    0
#define EYEH_SIG    0
#define EYEV_SIG    1

// use these as the first arg to targ_ask
// ASK_ON means turn it on, ASK_OFF means turn it off. 
#define ASK_ON HANDLE_ON
#define ASK_OFF HANDLE_OFF

// use these as the SECOND arg to targ_ask
// ASK_FP means we're talking about the fixpt, ASK_TARG means we're talking about the target. 
#define ASK_FP 2
#define ASK_TARG 3


/**********************************************************************
 * Global variables.
 **********************************************************************/
 
int errcnt;
char outbuf[512];									/* used in pr_info */
static unsigned char fphandle, fprh, trh;      /* handles for fixation point */  
static int isfpon = 0, iston = 0;

/************************************************************************
 * Declaration of statelist variables.
 ************************************************************************/
 
int	fixx = 0,
	fixy = 0,
	targx = NULLI,
	targy = NULLI,
	red = 255,
	green = 0,
	blue = 0,
	stimz = 57, 
	fpsiz = 8,
	axis = 0,
	dist = 100,
	ntrials = 100,
	msec = 1000,
	remain,
	bg = 0;		/* background grayscale value */

	float cam_position[3]; 							/* Current camera position */
	float cam_looking[3];

	float cam_up[3] = { 0, 1, 0 };

	char local_addr[16] = "192.168.1.1";
	char remote_addr[16]="192.168.1.2";
	int remote_port = 2000;


	/* For fixpt and target  - struct is in rexr3c/target.h*/

	int targ_handle;
	TargetStruct f_targ;	
	TargetStruct f_fp;	

/************************************************************************
 * Control functions to make things happen.
 ************************************************************************/

int my_render_init()
{
	int status;

	// issue world command
	// This initializes render for a particular set of commands -- in this case the "pursuit" world uses commands 
	// for the pursuit task. 
	// TODO this will require the "render" world - but that world is not yet ready for prime time! 
	render_init_world("pursuit");

	// background color
	render_bgcolor(bg, bg, bg);

	// Setup viewing volume -- this should be done prior to the groundplane init! */
	render_perspective(90, 1, 1000);

	/* Setup init camera position */
	cam_position[0] = 0;
	cam_position[1] = 0;
	cam_position[2] = 0;
	cam_looking[0] = 0;
	cam_looking[1] = 0;
	cam_looking[2] = 1;
	render_camera(cam_position, cam_looking, cam_up);

	/* */
	/*render_dot(&dot);*/

	/* Configure target.  */
	f_targ.xoffset = -sin(0);
	f_targ.zoffset = cos(0);
	f_targ.xsize = 3;
	f_targ.ysize = 1; // targ_size;
	f_targ.d = 20;
	f_targ.h = 0;
	f_targ.r = 0;
	f_targ.g = 255,
	f_targ.b = 0;

	f_fp.xoffset = -sin(0);
	f_fp.zoffset = cos(0);
	f_fp.xsize = 1;
	f_fp.ysize = 3; // fp_size;
	f_fp.d = 20;
	f_fp.h = 0;
	f_fp.r = 255;
	f_fp.g = 0,
	f_fp.b = 0;


	render_target(&f_fp);
	
	/* HANDLE message will be returned by render. We need the handle before we can turn the target off. */

	return 0;
}



/* my_check_for_targ_handle *********************************/

int my_check_for_targ_handle()
{
	int status = 0;
	status = render_check_for_handle(&targ_handle);
	if (status == 1)
	{
		dprintf("my_check_for_targ_handle: targ_handle=%d\n", targ_handle);
	}
	return status;
}


/********************************* setbg() *******************************
 *
 * sets the background luminance
 */
int setbg()
{
	/* TODO */
	render_bgcolor(bg, bg, bg);
	return 0;
}

/******************************** trcount() *******************************
 * 
 * bookkeeping function
 */
int trcount(int flag)
{
	if (flag) score(1);	/* Rex trial counter */
	else score(0);
	if (!flag) errcnt++;
	return 0;
}

/******************************* initial() *******************************
 * 
 * setup function
 */
int initial(void)
{
	remain = ntrials;
	errcnt = 0;
	return 0;
}


/****************************** targ_ask(int on, int which) *******************************
 *
 * requests that a saccade target be displayed or turned off. 
 * If 'on' is ASK_ON(=1), turn it on, if 'on' is ASK_OFF (=0) turn it off. 
 * if 'which' is ASK_FP (ASK_TARG) it means we're talking about the fixpt (target)
 */
int targ_ask(int on, int which)
{
	float xloc, yloc;
	static int lastx, lasty;

	if (which == ASK_FP)
	{
		if (on == ASK_ON)
		{
			/* do an update here to make sure target position is right */
			render_update(targ_handle, &f_fp, sizeof(TargetStruct), HANDLE_ON);
			isfpon = on;
		}
		else
		{
			render_onoff(&targ_handle, HANDLE_OFF, ONOFF_NO_FRAME);
		}			
		dprintf("targ_ask: FP %d\n", on);
	}
	else if (which == ASK_TARG)
	{
		// Update target position in the struct, then issue render update command
		if (on == ASK_ON)
		{
			f_targ.xoffset = -sin(fixx*PI/180);
			f_targ.zoffset = cos(fixy*PI/180);
			render_update(targ_handle, &f_targ, sizeof(TargetStruct), HANDLE_ON);
			iston = on;
		}
		else
		{
			render_onoff(&targ_handle, HANDLE_OFF, ONOFF_NO_FRAME);
		}			
		dprintf("targ_ask: TARG %d\n", on);
	}
	else
	{
		dprintf("ERROR: targ_ask(on, which) - unknown which value (%d)\n", which);
		return 1;	// no FRAME issued here!
	}

// TODO: re-implement alternating scheme. NOTE frame command below #if 0/ #endif block
#if 0
		if (targx != NULLI && targy != NULLI)
		{
			xloc = (float)targx;
			yloc = (float)targy;
		}
		else if (axis)  /* an up-down alternation */                              
		{
			xloc = fixx;
			if ((lasty - fixy) > 0) yloc = (float)(fixy - dist);
			else yloc = (float)(fixy + dist);
			lasty = (int)yloc;
		}
		else                    /* a left-right alternation by default  */
		{
			yloc = fixy;
			if ((lastx - fixx) > 0) xloc = (float)(fixx - dist);
			else xloc = (float)(fixx + dist);
			lastx = (int)xloc;
		}
		da_cntrl_2(0, DA_STBY, 0, 1, DA_STBY, 0);
		da_set_2(0, (long)xloc, 1, (long)yloc);
		/* DJS WAS CU_DA_PLUS - but that's not defined. Use CU_DA_ONE instead. */
		da_cursor(0, 1, CU_DA_ONE);
		wd_disp(D_W_EYE_X | D_W_DA_PLUS);
		wd_pos(WIND0, (long)xloc, (long)yloc);
#endif

	render_frame(0);
	return 0;
}


/****************************** winon() **********************************
 * 
 * opens fixation window
 */
int winon(long xsiz, long ysiz)
{
	wd_src_pos(WIND0, WD_DIRPOS, 0, WD_DIRPOS, 0);
	wd_pos(WIND0, (long)fixx, (long)fixy);
	wd_siz(WIND0, (long)xsiz, (long)ysiz);
	wd_cntrl(WIND0, WD_ON);
	wd_src_check(WIND0, WD_SIGNAL, EYEH_SIG, WD_SIGNAL, EYEV_SIG);
	return 0;
}

/***************************** alloff() **********************************
 *
 * both fixation point and window off. Sends FRAME, doesn't wait for WENT
 */
int alloff()
{
	dprintf("alloff\n");
	render_onoff(&targ_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);
	isfpon = iston = 0;
	wd_cntrl(WIND0, WD_OFF);
	return 0;
}

/*************************** pr_info *************************************
 * 
 * prints bookeeping information
 */
void pr_info(void)
{
	/* this extern gives us an undefined reference error. I'm not sure why this is necessary. I stuck the local declaration of outbuf above and used it here. */
	/* extern char outbuf[]; */
	setbuf(stdout, &outbuf);
	printf("%d trials, %d errors\n",(ntrials-remain), errcnt);
	fflush(stdout);
	setbuf(stdout, 0);
}

/**************************** n_exlist() **********************************
 *
 * enables 't e' requests to print bookeeping info
 */
void n_exlist(char *vstr)
{
	switch(*vstr){
		case 't':	/* type a list */
			pr_info();
			break;
		default:
			badverb();
			break;
	}
}

int report(void)
{
	dprintf("eyeflag=%d\n", eyeflag);
	return 0;
}

static void null(void)
{
}


void rinitf(void)
{
	int status=0;

	// initialize tcpip - must only do this OR gpib - NOT BOTH!!!!
	status = init_tcpip(local_addr, remote_addr, remote_port);

}




VLIST state_vl[] = {
"fp_x",		&fixx,	NP, NP,	0, ME_DEC,
"fp_y",		&fixy,	NP, NP,	0, ME_DEC,
"duration",	&msec, NP, NP, 0, ME_DEC, 
"bg(0-255)", 	&bg, NP, NP, 0, ME_DEC, 
"axis(0,1)",	&axis, NP, NP, 0, ME_DEC,
"fp_R(0-255)", 	&red, NP, NP, 0, ME_DEC, 
"fp_G(0-255)", 	&green, NP, NP, 0, ME_DEC, 
"fp_B(0-255)", 	&blue, NP, NP, 0, ME_DEC, 
"targ_dist", 	&dist, NP, NP, 0, ME_DEC,
"targ_x_ovrd",	&targx, NP, NP, 0, ME_NVALD,
"targ_y_ovrd",	&targy, NP, NP, 0, ME_NVALD,
"CRT_dist(cm)",		&stimz, NP, NP, 0, ME_DEC,
"fp_size(pix)",		&fpsiz, NP, NP, 0, ME_DEC,
"trials",		&ntrials, NP, NP, 0, ME_DEC,
"local ip addr", local_addr, NP, NP, 0, ME_STR,
"remote ip addr", remote_addr, NP, NP, 0, ME_STR,
"remote port", &remote_port, NP, NP, 0, ME_DEC,
NS,
};

/*
 * Help message.
 */
char hm_sv_vl[] = "";

#define TESTF(a) ((a)*100)

/*
 * User-supplied noun table.
 */
NOUN unouns[] = {
"exlist",       &n_exlist,
"",
};





%%

id 10
restart rinitf
main_set {
status ON
begin	first:
		code STARTCD
		rl 0
		to sendping
	sendping:
		rl 5
		do render_send_ping()
		to waitping
	waitping:
		rl 10
		time 100
		to checkping
	checkping:
		rl 15
		to reset on 1 % render_check_for_ping
		to pauseping
	pauseping:
		rl 20
		time 1000
		to sendping
	reset:
		rl 25
		do render_reset()
		to pause1
	pause1:
		to pause2 on +PSTOP & softswitch
		to start on -PSTOP & softswitch
	pause2:	
		to start on -PSTOP & softswitch
	start:
		do initial()
		to renderinit 
	renderinit:
		do my_render_init()
		to targhandle
	targhandle:
		to loop on 1 % my_check_for_targ_handle
	loop:
		time 100
		to pause3
	pause3:
		to pause4 on +PSTOP & softswitch
		to setbg2 on -PSTOP & softswitch
	pause4:
		code PAUSECD
		to setbg2 on -PSTOP & softswitch
	setbg2:
		do setbg()
		to isi
	isi:
		rl 30
		time 1000
		to fpon
	fpon:
		/* DJS THIS CODE NOT DEFINED code FIXASK*/
		do targ_ask(ASK_ON, ASK_FP)
		rl 50
		to fpon_went
	fpon_went:
		to winon on 1 % render_check_for_went
	winon:
		code FPONCD
		do winon(30,30)
		time 20
		to grace
	grace:
		time 4000
		to fixtim on -WD0_XY & eyeflag
		to off
	off:
		do alloff()
		to off_went
	off_went:
		to loop on 1 % render_check_for_went
	fixtim:
		time 150
		rl 50
		to noise on +WD0_XY & eyeflag
		to dncnt
	noise:
		do alloff()
		to noise_went
	noise_went:
		to loop on 1 % render_check_for_went
	dncnt:
		time 500
		to bad on +WD0_XY & eyeflag
		to fpoff
	fpoff:
		do targ_ask(ASK_OFF, ASK_FP)
		to fpoff_went
	fpoff_went:
		to targon on 1 % render_check_for_went
	targon:
		do targ_ask(ASK_ON, ASK_TARG)
		rl 40
		to targon_went
	targon_went:
		to grace2 on 1 % render_check_for_went
	grace2:
		time 2000
		to tfix on -WD0_XY & eyeflag
		to bad
	tfix:
		time 500
		to bad on +WD0_XY & eyeflag
		to good
	bad:
		do alloff()
		rl 10
		to bad_went
	bad_went:
		to punish on 1 % render_check_for_went
	punish:
		/* DJS THIS CODE NOT DEFINED code BREAKFIXCD */
		do trcount(FALSE)
		to beep
	beep:
		do dio_on(BEEP)
		time 1000
		to bpoff
	bpoff:
		do dio_off(BEEP)
		to timout
	timout:
		time 1000
		to trlend
	good:
		do trcount(TRUE)
		to toff
	toff:
		do alloff()
		to toff_went
	toff_went:
		to reward on 1 % render_check_for_went
	reward:
		code REWCD
		do dio_on(REW)
		time 500
		to rewoff
	rewoff:
		do dio_off(REW)
		to trlend
	trlend:
		to blkdone on 0 ? remain
		to loop
	blkdone:
		rl 0
		to pause1 on 0 < remain
	exit:
		to pause1

abort list:
		exit
}

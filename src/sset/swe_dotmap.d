/*  
 * Paradigm for fixation (and eventually saccade) training. Allows the
 * experimenter to control background luminance and fixation spot size
 * and color.
 *
 * This version updated to be compatible with Rex version 5.4
 * KHB, 12/21/95
 * 
 * joymap.d spawned from fix.d, to handle joystick mapping of RFs. 
 * dotmap.d is almost identical to joymap.
 *
 * Tertiary state list set control fixation monitoring LED added
 * 4/03/98 HWH.  Allows user control of fixation LED by setting
 * a variable in the statelist (st menu) true. This variable, fix_led,
 * is by default false (0); setting it to 1 turns on the monitoring.
 * 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "cnf.h"

#include "ldev.h"	/* hardware device defines */
#include "lcode.h"	/* Ecodes */
#include "lcalib.h"
#include "pixels.h"		/* pixel <==> degree conversion */

#include "make_msg.h"	/* generates msgs, and provides all structs for commands.See /rexr3c/*.h */
#include "actions.h"	/* action lib for render */


/* New render stuff
 * 
 */

DotStruct f_fpStruct;		// fp parameters
static FF2DStruct f_ff2dStruct;	// ff2d parameters
int f_fpHandle = 0;			// handle for dot
int f_ff2dHandle = 0;		// handle for ff2d
int f_handleCount=0;		// used in my_check_for_handles()

/* End of new render stuff. */


#define FALSE 0
#define TRUE 1  

#define WIND0	    0
#define EYEH_SIG    0
#define EYEV_SIG    1

#define JOY_X	    6
#define JOY_Y	    7

#define PTSIZ	4
#define FPSIZ	1

#define PI 3.14159265

/**********************************************************************
 * Global variables.
 **********************************************************************/
 
int framecnt,  errcnt;

static unsigned char fph, fprh, dh, drh;      /* handles for FP, bar */  
int isfpon = 0, isdotconf = 0, window_on = 0;
static int coh_seed, loc_seed;
static float joy_x, joy_y;	    /* in pixel coordinates */
static int dir = 0;
static int ptsiz = PTSIZ;
static int fpsiz = 2;
static int framerate = 85;
static int fpred = 255;
static int fpgreen = 0;
static int fpblue = 0;

/************************************************************************
 * Declaration of statelist variables.
 ************************************************************************/
 
int	fixx = 0,
	fixy = 0,
	stimz = 290, //in mm
	coh = 750,
	width = 0,
	islinear = 1,
	bar_R = 255,
	bar_G = 255,
	bar_B = 255,
	ntrials = 1000,
	msec = 3000,
    fix_led = 0,
	remain,
	density = 50,
	radius = 20,
	speed = 50,
	bg = 0;		/* background grayscale value */

static int f_joyXMin=160;
static int f_joyXMax=1600;
static int f_joyYMin=160;
static int f_joyYMax=1600;
static int f_joyDebug = 0;
static int f_forceDotsOn = 0;

char local_addr[16] = "192.168.1.1";
char remote_addr[16]="192.168.1.2";
int remote_port = 2000;


/************************************************************************
 * Control functions to make things happen.
 ************************************************************************/

/******************************** rinitf() *********************************/
void rinitf(void)
{
	int status=0;

	// initialize tcpip - The last arg (0) turns autoflush off.
	status = init_tcpip(local_addr, remote_addr, remote_port, 0);

}


/********************************* setbg() *******************************
 *
 * sets the background luminance
 */
int setbg()
{
//	vsend(MSG_SETBG, "bbb", (unsigned char)bg, (unsigned char)bg,
//		(unsigned char)bg);
	render_bgcolor(bg, bg, bg);
	return 0;
}

/******************************** trcount() *******************************
 * 
 * bookkeeping function
 */
int trcount(int flag)
{
	score(flag);	/* Rex trial counter */
	//msec = msec - 1;
	framecnt = msec/11;	/* frame counter (approximate time) */
	if (!flag) errcnt++;
	return 0;
}

void get_dot_param(DotStruct *pdot)
{
	// set dot parameters. Note color is hard-coded!
	int siz;
	siz = to_pixels((float)fpsiz/10);
	pdot->xorigin = fixx;
	pdot->yorigin = fixy;
	pdot->xsize = siz;
	pdot->ysize = siz;
	pdot->depth = 10;
	pdot->r = fpred;
	pdot->g = fpgreen;
	pdot->b = fpblue;
	pdot->a = 0;
	
}

void get_ff2d_param(FF2DStruct* pff2d)
{
	int npoints;
	float degperframe = (float)speed/10/framerate;
	float area = 1.0;
	float rad = to_pixels((float)radius/10);
	float den = (float)density/100; 
	area = PI*radius*radius/100;
	npoints = (int)(den*area);
	pff2d->linear = islinear;
	pff2d->npts = npoints;
	pff2d->prob = (float)coh/1000.0f;
	pff2d->radius = rad;			
	pff2d->pseed = loc_seed;
	pff2d->cseed = coh_seed;		
	pff2d->pixsz = PTSIZ;
	pff2d->depth = 20;			// TODO: NO HARDCODING
	pff2d->v = to_pixels(degperframe);
	//pff2d->dx = 0;
	//pff2d->dy = 0;
	pff2d->x = joy_x;
	pff2d->y = joy_y;
	pff2d->width = (float)width/10.0f;			// TESTING
	pff2d->angle = dir;
	pff2d->r = bar_R;
	pff2d->g = bar_G;
	pff2d->b = bar_B;
	
	//dprintf("%d\n", npoints);
}

/***************************** my_conf_all() **********************************
 * 
 * Configure dot and ff2d. Sends commands, but does not fetch handles. See my_check_for_handles
 * 
 */

int my_conf_all()
{
	// background color
	render_bgcolor(bg, bg, bg);

	// Get dot parameters, send DOT command
	get_dot_param(&f_fpStruct);
	render_dot(&f_fpStruct);		// will need to get handle return

	// Get ff2d parameters, send FF2D command
	get_ff2d_param(&f_ff2dStruct);
	render_ff2d(&f_ff2dStruct);
}

/* 
 * my_check_for_handles 
 * 
 * Escape function that returns 1 when two handles have been received. 
 * The first handle received will be the fp handle. 
 * The second handle received will be the dotfield handle. 
 * 
 * Uses a global int, f_handleCount. When that count reaches 2 we're done. 
 * 
 */

int my_check_for_handles()
{
	int h=0;
	while (render_check_for_handle(&h))
	{
		if (f_handleCount == 0)
		{
			f_fpHandle = h;
			f_handleCount = 1;
			dprintf("fp handle %d\n", f_fpHandle);
		}
		else if (f_handleCount == 1)
		{
			f_ff2dHandle = h;
			f_handleCount = 2;
			dprintf("ff2d handle %d\n", f_ff2dHandle);
		}
		else
		{
			dprintf("ERROR: Handle count is %d\n", f_handleCount);
		}
	}
	return (f_handleCount == 2);
}

/******************************* conf_fp() *****************************
* 
* configures the fixation point
*/
void conf_fp(DotStruct *pdot)
{
	// set dot parameters. Note color is hard-coded!
	pdot->xorigin = fixx;
	pdot->yorigin = fixy;
	pdot->xsize = to_pixels((float)fpsiz/10);
	pdot->ysize = to_pixels((float)fpsiz/10);
	pdot->depth = 10;
	pdot->r = fpred;
	pdot->g = fpgreen;
	pdot->b = fpblue;
	pdot->a = 0;
}

/******************************* initial() *******************************
 * 
 * setup function
 */
int initial(void)
{
	initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, x_resolution, y_resolution, stimz);	// Initializes to_pixels function properties
	framecnt = msec/11;
	remain = ntrials;
	errcnt = 0;
	coh_seed = joy_x;
	loc_seed = joy_y;
	return 0;
}

/****************************** fpask() *******************************
 *
 * requests that a FP be displayed
 */
int fpask(void)
{
	render_update(f_fpHandle, &f_fpStruct, sizeof(f_fpStruct)	, HANDLE_ON);
	render_frame(0);
	return 0;
}

/***************************** ff2d_update ***************************/
int ff2d_update()
{
	get_ff2d_param(&f_ff2dStruct);
	return 0;
}


/***************************** fpoff() *********************************
 *
 * turns off the fixation point
 */
int fpoff(void)
{
	/* Added by swe 8/22 */
	render_onoff(&f_fpHandle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_frame(0);
	return 0;
}

/***************************** dotoff() *********************************
 *
 * turns off the dot field     
 */
int dotoff(void)
{
	render_onoff(&f_ff2dHandle, HANDLE_OFF, ONOFF_NO_FRAME);
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

/* my_check_for_went *********************************/

int swe_check_for_went()
{
	int frames = 0;
	int wstatus;
	wstatus = render_check_for_went(&frames);
	if (wstatus < 0)
	{
		// ERROR! not clear what to do ...
		dprintf("ERROR in render_check_for_went!\n");
	}
	else if (wstatus == 1)
	{
		//if (frames > 1) dprintf("Drop %d\n", frames);
	}	
	return wstatus;
}


/******************************wincheck() *******************************
 *
 * checks if fixation window on (window_on = 1) for LED control
 */

int wincheck()
{
 	dprintf("wincheck\n");
	window_on = 1;
	return 0;
}


/***************************** alloff() **********************************
 *
 * both fixation point and window off, for clean state list
 */
int alloff()
{
	render_onoff(&f_fpHandle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_ff2dHandle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_frame(0);
	wd_pos(WIND0, (long)9999, (long)9999);// disappear the window on eyd display
	window_on = 0;
	return 0;
}

/***************************** showdot() *********************************
 * 
 * shows (reshows) dot field if necessary
 */
int showdot()
{
 	if (JOY_BUT_0 & dina) dprintf("button 0 down\n");
	if ((JOY_BUT_0 & dina) || f_forceDotsOn)	/* on-off button down */
	{
	    render_update(f_ff2dHandle, &f_ff2dStruct, sizeof(f_ff2dStruct), HANDLE_ON);
	    render_frame(0);
	}
    else			/* button up - want no stim */
	{
	    dotoff();
	}
	
    return 0;
}

/**************************** getjoy() ***********************************
 * 
 * gets analog joystick values from A/D channels 6 & 7
 */
int getjoy()
{
	float fx, fy;
	float fxn, fyn;	// normalized

	fx = (float)(addh);
	fy = (float)(addv);
	if (f_joyDebug==1)
	{
		dprintf("%d addxy: %d %d   joyxy: %d %d eye %d %d\n", getClockTime(), addh, addv, joyh, joyv, eyeh, eyev);
	}

	// assume the joystick AD values run from xmin-xmax and ymin-ymax. 
	// Scale fx and fy against this and shift to [-0.5, 0.5]
	fxn = (fx - f_joyXMin)/(f_joyXMax-f_joyXMin) - 0.5;
	fyn = (fy - f_joyYMin)/(f_joyYMax-f_joyYMin) - 0.5;

	// now scale these by the x and y resolutions
	joy_x = fxn * x_resolution;
	joy_y = -1.0 * fyn * y_resolution;	

	
	if (f_joyDebug == 2)
	   dprintf("t %d dina %x\n", getClockTime(), dina);
	
	
	return 0;
}


/*int report(void)
{
	dprintf("eyeflag=%d\n", eyeflag);
	return 0;
}*/

VLIST joy_vl[] = {
  "xmin(ADC)",		    &f_joyXMin, NP, NP, 0, ME_DEC,
  "xmax(ADC)",		    &f_joyXMax, NP, NP, 0, ME_DEC,
  "ymin(ADC)",		    &f_joyYMin, NP, NP, 0, ME_DEC,
  "ymax(ADC)",		    &f_joyYMax, NP, NP, 0, ME_DEC,
  "debug(0/1)",         &f_joyDebug, NP, NP, 0, ME_DEC,
  "forceDotOn", 		&f_forceDotsOn, NP, NP, 0, ME_DEC,
  NS,
};	

MENU joy_me = 
{
  "joy_params", &joy_vl, NP, NP, 0, NP, NS,
};

char hm_joy[] = "";

VLIST state_vl[] = {
  "fp_x",		&fixx,	NP, NP,	0, ME_DEC,
  "fp_y",		&fixy,	NP, NP,	0, ME_DEC,
  "fp_size",		&fpsiz,	NP, NP,	0, ME_DEC,     
  "duration",	        &msec, NP, NP, 0, ME_DEC, 
  "bg(0-255)", 	        &bg, NP, NP, 0, ME_DEC, 
  "CRT_dist(mm)",	&stimz, NP, NP, 0, ME_DEC,
  "radius(1/10deg)?",	&radius, NP, NP, 0, ME_DEC,
  "density?(100*d/d^2)",&density, NP, NP, 0, ME_DEC,
  "coherence(0-1000)",		&coh, NP, NP, 0, ME_DEC,
  "speed(deg/sec) ",		&speed, NP, NP, 0, ME_DEC,
  "linear?(0,1)",	&islinear, NP, NP, 0, ME_DEC,
  "width(1/10deg)", &width, NP, NP, 0, ME_DEC,
  "dot_R",		&bar_R, NP, NP, 0, ME_DEC,
  "dot_G",		&bar_G, NP, NP, 0, ME_DEC,
  "dot_B",		&bar_B, NP, NP, 0, ME_DEC,
  "background",		&bg, NP, NP, 0, ME_DEC,
  "fix_led",            &fix_led, NP, NP, 0, ME_DEC,
  "trials",		&ntrials, NP, NP, 0, ME_DEC,
  "local_ip_addr", local_addr, NP, NP, 0, ME_STR,
  "remote_ip_addr", remote_addr, NP, NP, 0, ME_STR,
  "remote_port", &remote_port, NP, NP, 0, ME_DEC,
NS,
};


VLIST fp_vl[] = {
  "fixpt_R",		&fpred, NP, NP, 0, ME_DEC,
  "fixpt_G",		&fpgreen, NP, NP, 0, ME_DEC,
  "fixpt_B",		&fpblue, NP, NP, 0, ME_DEC,
  "background",		&bg, NP, NP, 0, ME_DEC,
  "dot_R",		&bar_R, NP, NP, 0, ME_DEC,
  "dot_G",		&bar_G, NP, NP, 0, ME_DEC,
  "dot_B",		&bar_B, NP, NP, 0, ME_DEC,
NS,
};

char hm_fp[] = "";

//  "stim_params",  	&stim_me, NP, NP, 0, ME_SUBMENU,

/*
 * Help message.
 */
char hm_sv_vl[] = "";




MENU umenus[] = {
{"state_vars", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{"separator", NP}, 
{"fp_vars", &fp_vl, NP, NP, 0, NP, hm_fp},
{"joy vars", &joy_vl, NP, NP, 0, NP, hm_joy},
{NS},
};



















#define TESTF(a) ((a)*100)

/*
 * User-supplied noun table.
 */
NOUN unouns[] = {
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
		to initial
	initial:
		do initial()
		to pause1
	pause1:
		to pause2 on +PSTOP & softswitch
		to initial on -PSTOP & softswitch
	pause2:	
		code PAUSECD
		to confall on -PSTOP & softswitch
	confall:
		do my_conf_all()
		to loop on 1 % my_check_for_handles
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
		time 1000
		to fpon
	fpon:
		code FIXASK
		do fpask() /* fpask() sends a MSG_FRAME and returns */
		to winon on 1 % swe_check_for_went
	winon:
		code FPONCD
		do winon(20,20)
		time 20
		to grace
	grace:
		time 4000
		to fixtim on -WD0_XY & eyeflag
		to off
	off:
		do alloff()
		to loop on 1 % swe_check_for_went
	fixtim:
		time 150
		rl 20
		do wincheck()     
		to noise on +WD0_XY & eyeflag
		to dncnt
	noise:
		do alloff()
		to loop on 1 % swe_check_for_went
	dncnt:
		do ff2d_update();
		to bad on +WD0_XY & eyeflag
		to fpoff on 0 ? framecnt
		to showdot
	showdot:
		do showdot()
		to dncnt on 1 % swe_check_for_went
	fpoff:
		do alloff()
		rl 10
		to good on 1 % swe_check_for_went
	bad:
		do alloff()
		to punish on 1 % swe_check_for_went
	punish:
		code BREAKFIXCD
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
		to reward
	reward:
		code REWCD
		do dio_on(REW)
		time 50
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

abort list:
		initial 
}



/*
 * Auxiliary set for updating joystick location variables.
 */

joy_set {
status ON
begin	no_op:
	    to getjoy
	getjoy:
		time 5
	    do getjoy()
	    to getjoy2
	getjoy2:     /* identical to state getjoy; needed to avoid recursion */
		time 5
	    do getjoy()
	    to getjoy	
} 


/*
 * Auxiliary set for LED control 
 * used for fixation monitoring
 */


fixled_set{
status ON
begin	led_control:
		to led_off on 1 = fix_led	
	led_on:
		do dio_on(FIX_LED) 
	 	to led_off on 0 = window_on
	led_off:
		do dio_off(FIX_LED)
		to led_on on 1 = window_on
} 

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


// structs and handles for render objects

DotStruct f_fpStruct;		// fp parameters
static FF2DStruct f_ff2dStruct;	// ff2d parameters
int f_fpHandle = 0;			// handle for dot
int f_ff2dHandle = 0;		// handle for ff2d
int f_handleCount=0;		// used in my_check_for_handles()


#define FALSE 0
#define TRUE 1  

#define WIND0	    0
#define EYEH_SIG    0
#define EYEV_SIG    1

#define JOYDEBUG_RESPONSE 0x1


/**********************************************************************
 * Global variables.
 **********************************************************************/

int dots_on = 1;		   /* controlled by button 1 */
int joystick_enabled = 1;  /* NOT under button control! */

int window_on=0;   /* can't make this static or compiler chokes */
int framecnt;	   /* ditto. */
static int coh_seed, loc_seed;
static float joy_x = 0, joy_y = 0;	    /* in pixel coordinates */
static int dir = 0;
static int dirstep = 45;
static int ptsiz = 2;
static int fpsiz = 2;
static int fpwinsize = 20;
static int fpred = 255;
static int fpgreen = 0;
static int fpblue = 0;
static int f_reward_time = 50;
static int f_reward_random = 0;

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

static int f_joyDead  = 5;
static int f_joyPixPerFrame = 5;

static int f_joyXMin  = -1000;
static int f_joyXMax  = 1000;
static int f_joyXZero = 0;

static int f_joyYMin=-1000;
static int f_joyYMax=1000;
static int f_joyYZero = 0;
static int f_joyDebug = 0;
static int f_joyDebugThrottle = 500;
static int f_joyDebugCount = 0;

static int f_width;
static int f_height;
static double f_framerate;

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
	return 0;
}



void get_dot_param(DotStruct *pdot)
{
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


int get_single_response(int ival, int imax, int imin, int izero, int idead, float *response)
{
	int status = 0;
	int sign = 1;
	
	if (imax == izero || imin == izero) {
		status = -1;
		*response = 0;
	}
	if (abs(ival - izero) <= idead) {
		*response = 0;
	} else if ((ival-izero)*(imax-izero) > 0) {
		// response is positive
		*response = (float)(ival-izero)/(float)(imax - izero);
	} else {
		// response is negative
		*response = -1 * (float)(ival - izero)/(float)(imin - izero);
	}
	return status;
}


int get_joystick_response(float *pxresponse, float *pyresponse)
{
	int status = 
	  get_single_response(addh, f_joyXMax, f_joyXMin, f_joyXZero, f_joyDead, pxresponse) + 
	  get_single_response(addv, f_joyYMax, f_joyYMin, f_joyYZero, f_joyDead, pyresponse);
	return status;
}
	

void get_ff2d_param(FF2DStruct* pff2d)
{
    float xresponse = 0.0;
	float yresponse = 0.0;
	int npoints;
	float degperframe = (float)speed/10/f_framerate;
	float area = 1.0;
	float rad = to_pixels((float)radius/10);
	float den = (float)density/100; 
	area = M_PI*radius*radius/100;
	npoints = (int)(den*area);
	pff2d->linear = islinear;
	pff2d->npts = npoints;
	pff2d->prob = (float)coh/1000.0f;
	pff2d->radius = rad;			
	pff2d->pseed = loc_seed;
	pff2d->cseed = coh_seed;		
	pff2d->pixsz = ptsiz;
	pff2d->depth = 20;			// TODO: NO HARDCODING
	pff2d->v = to_pixels(degperframe);
	pff2d->width = (float)width/10.0f;			// TESTING
	pff2d->angle = dir;
	pff2d->r = bar_R;
	pff2d->g = bar_G;
	pff2d->b = bar_B;

	if (joystick_enabled)
	{
	    if (!get_joystick_response(&xresponse, &yresponse))
		{	
			// djs xresponse has a sign inversion. 
		    joy_x = joy_x + (xresponse * f_joyPixPerFrame);
			joy_y = joy_y + (yresponse * f_joyPixPerFrame);
			if (f_joyDebug  & JOYDEBUG_RESPONSE)
			{
			   if (f_joyDebugCount >= f_joyDebugThrottle)
			   {
			   		dprintf("get_ff2d_param: addh (resp/delta/joy_x) %d (%d/%d/%d), addv (resp/delta/joy_y) %d (%d/%d/%d)\n",
									addh, (int)(xresponse*100), (int)(xresponse*f_joyPixPerFrame*100), (int)joy_x, 
									addv, (int)(yresponse*100), (int)(yresponse*f_joyPixPerFrame*100), (int)joy_y);
			   }
			}
			if (joy_x < -f_width/2) joy_x = -f_width/2;
			if (joy_x > f_width/2)  joy_x = f_width/2;
			if (joy_y < -f_height/2) joy_y = -f_height/2;
			if (joy_y > f_height/2) joy_y = f_height/2;
		}
		else
		{
		    dprintf("get_ff2d_param: get_joystick_response failed!\n");
		}
	}
    if (f_joyDebugCount >= f_joyDebugThrottle)
	{
	 	f_joyDebugCount = 0;
	}
	else
	{
	   f_joyDebugCount++;
	}
	pff2d->x = joy_x;
	pff2d->y = joy_y;	
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
	int status = 0;
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
			status = 1;
			dprintf("ff2d handle %d\n", f_ff2dHandle);
		}
		else
		{
			dprintf("ERROR: Handle count is %d\n", f_handleCount);
		}
	}
	return status;
}


/******************************* initial() *******************************
 * 
 * setup function
 */

int initial(void)
{
	render_get_parameters(&f_width, &f_height, &f_framerate);
	dprintf("render parameters: %dx%d@%d\n", f_width, f_height, (int)f_framerate);
	initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, f_width, f_height, stimz);	// Initializes to_pixels function properties
	remain = ntrials;
	coh_seed = joy_x;
	loc_seed = joy_y;
	joy_x = joy_y = 0;
	f_fpHandle = f_ff2dHandle = f_handleCount = 0;
	setbg();
	
	/* Set reward time */
	set_times("reward", (long)f_reward_time, (long)f_reward_random);
	
	return 0;
}

/*
 * fpask() - update fixation point with current parameters, turn on,
 *           and issue FRAME command (does not check for WENT)
 *
 */

int fpask(void)
{
	render_update(f_fpHandle, &f_fpStruct, sizeof(f_fpStruct), HANDLE_ON);
	render_frame(0);
	return 0;
}


/*
 * dottie_update() - updates values in FF2DStruct with current parameters 
 *                 from menus, and current joystick position value.
 *
 */
 
int dottie_update()
{
 	/* If dots are ON, then get params and issue update */
	if (dots_on)
	{
		get_ff2d_param(&f_ff2dStruct);
	    render_update(f_ff2dHandle, &f_ff2dStruct, sizeof(f_ff2dStruct), HANDLE_ON);
	}
	render_frame(0);
	return 0;
}


/*
 * fpoff() - send command to turn off fixpt, issue FRAME, 
 *           does not check for went!
 *
 */

int fpoff(void)
{
	render_onoff(&f_fpHandle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);
	return 0;
}


/*
 * dotoff() - send command to turn off ff2d, issue FRAME, does not check
 *            for WENT
 */

int dotoff(void)
{
	render_onoff(&f_ff2dHandle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);
	return 0;
}


/*
 * winon() - configures fixation window using 
 *           parameters from menus (fixx, fixy, xsiz, ysiz). 
 * 
 */

int winon()
{
	wd_src_pos(WIND0, WD_DIRPOS, 0, WD_DIRPOS, 0);
	wd_pos(WIND0, (long)fixx, (long)fixy);
	wd_siz(WIND0, (long)fpwinsize, (long)fpwinsize);
	wd_cntrl(WIND0, WD_ON);
	wd_src_check(WIND0, WD_SIGNAL, EYEH_SIG, WD_SIGNAL, EYEV_SIG);
	return 0;
}

/* 
 * my_check_for_went() 
 */


int my_check_for_went()
{
	int frames = 0;
	int wstatus;
	wstatus = render_check_for_went(&frames);
	if (wstatus < 0)
	{
		// ERROR! not clear what to do ...
		dprintf("ERROR in render_check_for_went!\n");
	}
	return wstatus;
}


/****************************** my_fixtim()() *******************************
 *
 * checks if fixation window on (window_on = 1) for LED control
 */

int my_fixtim()
{
	window_on = 1;
	framecnt = msec / 1000 * f_framerate;
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


int my_button1_toggled()
{
    dots_on = 1 - dots_on;
	render_onoff(&f_ff2dHandle, (dots_on ? HANDLE_ON : HANDLE_OFF), ONOFF_NO_FRAME);
	dprintf("my_button1_toggled(): dots_on = %d\n", dots_on);
	return 0;
}

int my_button1_reset()
{
    //dots_on = 0;
	//dprintf("my_button1_reset(): dots_on = %d\n", dots_on);
	return 0;
}

int my_button2_toggled()
{
	dir = dir + dirstep;
	dir = dir % 360;
	dprintf("my_button2_toggled(): dir = %d\n", dir);
	return 0;
}

int my_button2_reset()
{
    //joystick_enabled = 0;
	//dprintf("my_button2_reset(): joystick_enabled = %d\n", joystick_enabled);
	return 0;
}

int my_button3_toggled()
{
	dir = dir - dirstep;
	while (dir < 0) dir = dir + 360;
	dprintf("my_button3_toggled(): dir = %d\n", dir);
	return 0;
}

int my_button3_reset()
{
    //joystick_enabled = 0;
	//dprintf("my_button2_reset(): joystick_enabled = %d\n", joystick_enabled);
	return 0;
}


VLIST joy_vl[] = {
  "xmin(ADC)",		    &f_joyXMin, NP, NP, 0, ME_DEC,
  "xmax(ADC)",		    &f_joyXMax, NP, NP, 0, ME_DEC,
  "xzero(ADC)",         &f_joyXZero, NP, NP, 0, ME_DEC,
  "ymin(ADC)",		    &f_joyYMin, NP, NP, 0, ME_DEC,
  "ymax(ADC)",		    &f_joyYMax, NP, NP, 0, ME_DEC,
  "yzero(ADC)",         &f_joyYZero, NP, NP, 0, ME_DEC,
  "dead(ADC)",          &f_joyDead, NP, NP, 0, ME_DEC,
  "speed(ADC/frame)",   &f_joyPixPerFrame, NP, NP, 0, ME_DEC,
  "debug(0/1)",         &f_joyDebug, NP, NP, 0, ME_DEC,
  "debug-display-1/N",  &f_joyDebugThrottle, NP, NP, 0, ME_DEC,
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
  "fp_window", &fpwinsize, NP, NP, 0, ME_DEC,
  "reward", &f_reward_time, NP, NP, 0, ME_DEC,
  "reward_random", &f_reward_random, NP, NP, 0, ME_DEC,
  "duration",	        &msec, NP, NP, 0, ME_DEC, 
  "bg(0-255)", 	        &bg, NP, NP, 0, ME_DEC, 
  "CRT_dist(mm)",	&stimz, NP, NP, 0, ME_DEC,
  "radius(1/10deg)?",	&radius, NP, NP, 0, ME_DEC,
  "dot_size(pixels)", &ptsiz, NP, NP, 0, ME_DEC,
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
		to isi on -PSTOP & softswitch
	pause4:
		code PAUSECD
		to isi on -PSTOP & softswitch
	isi:
		time 1000
		to fpon
	fpon:
		code FIXASK
		do fpask()
		to winon on 1 % my_check_for_went
	winon:
		code FPONCD
		do winon()
		time 20
		to grace
	grace:
		time 4000
		to fixtim on -WD0_XY & eyeflag
		to off
	off:
		do alloff()
		to loop on 1 % my_check_for_went
	fixtim:
		time 150
		rl 20
		do my_fixtim()     
		to noise on +WD0_XY & eyeflag
		to dncnt
	noise:
		do alloff()
		to loop on 1 % my_check_for_went
	dncnt:
		do dottie_update();
		to bad on +WD0_XY & eyeflag
		to fpoff on 0 ? framecnt
		to dottie_wait
	dottie_wait:
		to dncnt on 1 % my_check_for_went
	fpoff:
		do alloff()
		rl 10
		to good on 1 % my_check_for_went
	bad:
		do alloff()
		to punish on 1 % my_check_for_went
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




button1_set {
status ON
begin	no_op1:
	to wait_for_button1_fix
wait_for_button1_fix:
	to wait_for_button1_down on 1 = window_on	
wait_for_button1_down:
	to wait_for_button1_up on -BUTTON1 & dina
	to button1_reset on 0 = window_on	
wait_for_button1_up:
	to button1_toggled on +BUTTON1 & dina
	to button1_reset on 0 = window_on	
button1_toggled:
	do my_button1_toggled()
	to wait_for_button1_down
button1_reset:
    do my_button1_reset()
	to wait_for_button1_fix
} 


button2_set {
status ON
begin	no_op2:
	to wait_for_button2_down
wait_for_button2_down:
	to wait_for_button2_up on -BUTTON2 & dina
wait_for_button2_up:
	to button2_toggled on +BUTTON2 & dina
button2_toggled:
	do my_button2_toggled()
	to wait_for_button2_down
} 

button3_set {
status ON
begin	no_op3:
	to wait_for_button3_down
wait_for_button3_down:
	to wait_for_button3_up on -BUTTON3 & dina
wait_for_button3_up:
	to button3_toggled on +BUTTON3 & dina
button3_toggled:
	do my_button3_toggled()
	to wait_for_button3_down
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

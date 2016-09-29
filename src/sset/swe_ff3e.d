/* $Id: swe_ff3e.d,v 1.6 2012/08/22 18:18:23 devel Exp $ */

/* Modified to emulate headtune3 for new render and rex7.8 by swe 2007/06/29 */

/*
 * Heading judgement psychophysics paradigm. Gives a range of correlation
 * values for 2 headings, symmetrically L and R of the gaze direction (dead
 * ahead). This angle can be varied. All motion in the "ground plane". Works
 * with Art's prototype 3D stuff in render.
 *
 * headpsy: the difference in angle remains constant, and the coherence is
 * varied to produce a psychometric function. Pursuit is required on half 
 * the trials. 
 * 
 * headpsy2: measures a threshold for angle difference of 100% coherent
 * stimuli.
 *
 * headpsy3: same thing for a monkey - eye position operant
 *
 * headpsy4: includes pursuit support for monkeys (ramp stuff from tstramp.d)
 *
 * headtune.d: no psychophysics, just heading tuning in X, with or without
 *             pursuit.
 *
 *	       - modified for PC render 8/15/95
 *~
 * swe_ff1.d: headtune3.d for new rex/render system by swe
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


#include "ldev.h"	/* hardware device defines */
#include "lcode.h"	/* Ecodes */
#include "lcalib.h"	/* per-rig dimension values */

#include "actions.h"	/* includes all files needed for use with new render. */
#include "make_msg.h"
#include "pixels.h"		/* pixel <==> degree conversion */

#include "../hdr/ramp.h"	/* ramps require it */

#include "ivunif.c"	 /*the random-number generator */

#define PI 3.14159265

#define EYEH_SIG    0
#define EYEV_SIG    1

// True and false codes
#define FALSE 0
#define TRUE (~FALSE)

// use these as the first arg to targ_ask
// ASK_ON means turn it on, ASK_OFF means turn it off.
#define ASK_ON HANDLE_ON
#define ASK_OFF HANDLE_OFF

// use these as the SECOND arg to targ_ask
// ASK_FP means we're talking about the fixpt, ASK_TARG means we're talking about the target.
#define ASK_FP 2
#define ASK_TARG 3

// used to define your movie length in msec
#define MOVIE_TIME 1251

// used for eye window ramp
#define WIND0       0
#define WIND1		1
#define EYEH_SIG    0
#define EYEV_SIG    1
#define RAMP0       0
#define X_DA        0
#define Y_DA        1
#define ECODE		8000
#define E_D0	2000

// used for stimulus array
#define READY 0
#define DONE 1
#define END 2

#define BLACKSCR 888

#define PLANED 1500

// used for ramps
#define RAMP0	    0

// ecodes
//define BREAKFIXCD 1180
//define CORRECTCD 1181
#define FFONCD 2000
#define FFOFFCD 2002
//define FIXCD 1184
#define RSTART 8191
#define RAMP_LF 1050
#define RAMP_RT 1051
#define RAMP_NA 1052
#define MOVIE_STRT 2001
#define STOPPED 2003
#define FFON 2004
#define MINIMUMCD 1050
#define MISSEDCD	1505

// codes for timing tests
#define FRAME 8080
#define FRAME2 8081
/**********************************************************************
 * Global variables.
 **********************************************************************/

struct stim
{
    int ang;			/* Heading angle, in tenths of degree */
    int ntot;			/* counter, number of trials completed for this stim */
    float pursuit_ang;	/* pursuit angle, degrees. 0 = right-to-left on screen; pos angles clockwise */
    short fixed;		/* if nonzero, target is fixed */ 
    short stat;			/* flag used in choosing trials */
};

#define MAXLIST 501
struct stim stimlist[MAXLIST];
struct stim *sp;
int nstim;


int errcnt;
char outbuf[8192];									/* used in pr_info */
int seedflg = 0;			/* flag set after random number generator seeded */

/*
 * render parameters
 */

int f_width, f_height;
double f_framerate;


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
	stimz = 280,
	dotnumber = 1000,
	fpsiz = 3,
	ptsiz = 2,
	dist = 100,
	ntrials = 100,		 /* trials per condition */
	bg = 0;		/* background grayscale value */
	dc = 255,
	mov_flg = 1,
 	ang_min = -300,
	ang_max = 300,
	ang_num = 13,
	ang_ovr = NULLI,
	pursuit_ang_min_10thdeg = 0,
	pursuit_ang_max_10thdeg = 1500,
	pursuit_ang_num = 4,
	pursuit_ang_ovr = NULLI,
	dur = 1000,
	init_pt = 150,
	stim_z = 28,
	ncheck = 20,
	stimwid = 10,
	stimht = 1000,
    bf_con = 1,    /* 0: normal ; 1: add one black screen on each eye pursuit condition  */
    blank = 0,
	remain,
	movietime = 1000,		// flow movie time in msec
	movieframes,
	speed = 100;			// 1/10ths of degrees per second
//	framerate = 85;

long seed = 1111;
float fpsize = 0.2,
	pathdelay = 250.0,	// delay in msec
	pathdelayframes;
double translation = 5000.0,		// in mm per sec
	pathlength;
float total_dot_travel_deg;			// distance dot travels (in pursuit) in degrees. 


long eyetraj[] = {0, 0, 0};	/* eye translation */
long vp[] = {0, 0, 100};		/* viewpoint - where the eye is pointed */
long vptraj[] = {0, 0, 0};	/* it's traj, this will be adjusted later */
long vpstart = -10;
double end_x, end_z;		/*speed of camera*/
float fpx_start, fpx_start1, fpx_end, fpx_end1, fpy_start, fpy_start1, fpy_end, fpy_end1;

float cam_position[3]; 							/* Current camera position */
float cam_looking[3];
float cam_up[3] = { 0, 1, 0 };

char local_addr[16] = "192.168.1.1";
char remote_addr[16]="192.168.1.2";
int remote_port = 2000;

/* check for went variables */
int f_wstatus;							/* Status code when checking for went */
int f_went_cycles=0;

/* For fixpt and dotfield */

int f_fpHandle = 0, f_fpHandle2 = 0, f_fpHandle3 = 0;
int f_dotsHandle = 0;
int f_handleCount = 0;
static PathStruct f_path;
static MoveObjectStruct f_moveobject;
static DotStruct f_fpStruct;
static DotFieldStruct f_dotsStruct;


/************************************************************************
 * Control functions to make things happen.
 ************************************************************************/


void dump_stim(int i)
{
	if (i==99) return;
	dprintf("%d: ang %d ntot %d pang %d fixed %d stat %d\n", 
	i, stimlist[i].ang, stimlist[i].ntot, (int)(stimlist[i].pursuit_ang*10), (int)(stimlist[i].fixed), (int)(stimlist[i].stat));
	return;
}
	

/********************************* ecode ********************************
 * Drops an ecode value to efile. 
 */

int ecode(int icode)
{
	EVENT ev;
	int status=0;
	
	if (icode < MINIMUMCD || icode > 8192)
	{
		dprintf("WARNING: ecode out of range %d\n", icode);
		status = -1;
	}
	
	ev.e_code= (short int)icode;
	ev.e_key= i_b->i_time;
	ldevent(&ev);
	return status;
}


/****************************** my_render_init() ***************************
*
* Initializes world, camera, and stimuli in render
*/

int my_render_init()
{
	int status;
	float fsize, fovy, far = 40000;

	dprintf("my_render_init\n");

	// djs initialize handle count
	f_handleCount = 0;
	
	// background color
	render_bgcolor(bg, bg, bg);

	// Setup viewing volume
	fovy = 2*atan2f(y_dimension_mm/2.0, stimz);
	render_perspective(fovy, stimz, far);

	// position camera
	cam_position[0] = 0;
	cam_position[1] = 0;				
	cam_position[2] = 0;
	cam_looking[0] = 0;
	cam_looking[1] = 0;
	cam_looking[2] = -1;
	cam_up[0] = 0;
	cam_up[1] = 1;
	cam_up[2] = 0;
	render_camera(cam_position, cam_looking, cam_up);

	/* Configure fp.  */
	f_fpStruct.xorigin = 0;
	f_fpStruct.yorigin = 0;
	f_fpStruct.xsize = to_pixels(fpsize);
	f_fpStruct.ysize = to_pixels(fpsize);
	f_fpStruct.depth = 10;
	f_fpStruct.r = red;
	f_fpStruct.g = green;
	f_fpStruct.b = blue;
	f_fpStruct.a = 0;
	render_dot(&f_fpStruct);		// will need to get handle return

	// configure dotfield
	f_dotsStruct.xwidth = 20000.0;
	f_dotsStruct.ywidth = 20000.0;
	f_dotsStruct.zwidth = 20000.0;
	f_dotsStruct.xorigin = 0.0;
	f_dotsStruct.yorigin = 0.0;
	f_dotsStruct.zorigin = -15000;
	f_dotsStruct.ndots = dotnumber;
	f_dotsStruct.pointsize = (float)ptsiz; //2.0;
	f_dotsStruct.r = dc;
	f_dotsStruct.g = dc;
	f_dotsStruct.b = dc;
	f_dotsStruct.a = 0;
	render_dotfield(&f_dotsStruct);		// will need to get handle return

	return 0;
}



/*************************** my_check_for_handles *************************
 *
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
			f_dotsHandle = h;
			f_handleCount = 2;
			dprintf("dots handle %d\n", f_dotsHandle);
		}
		else
		{
			dprintf("ERROR: Handle count is %d\n", f_handleCount);
		}
	}
	return (f_handleCount == 2);
}

/************************** my_check_for_went *********************************/

int my_check_for_went()
{
	int frames = 0;
	f_wstatus = render_check_for_went(&frames);
//	dprintf("check_for_went\n");
	
	if (f_wstatus < 0)
	{
		// ERROR! not clear what to do ...
		dprintf("ERROR in render_check_for_went!\n");
	}
	else if (f_wstatus == 0)
	{
		f_went_cycles++;
	}
	else if (f_wstatus == 1)
	{
		// TEST - if frames were missed, dprint it and the cycles...
		if (frames > 1)
		{
			dprintf("%d (%d)\n", frames, f_went_cycles);
			ecode(MISSEDCD);
		}
		f_went_cycles = 0;
	}
	return f_wstatus;
}


int my_check_for_went2()
{
	int frames=0;
	f_wstatus = render_check_for_went(&frames);
	if (f_wstatus == 1 && frames!=1) dprintf("Frames=%d\n", frames);
	return f_wstatus;
}


/************************* my_path_setup *****************************************
 *
 * Send path commands; sets up path and object movies in render.
 *
 */
int my_path_setup()
{
	// configure path parameters
	f_path.ep[0] = 0;
	f_path.ep[1] = 0;
	f_path.ep[2] = 0;

	f_path.epf[0] = eyetraj[0];
	f_path.epf[1] = eyetraj[1];
	f_path.epf[2] = -eyetraj[2];	/* Negative to agree w/ "world" coordinates */

	f_path.vp[0] = 0;
	f_path.vp[1] = 0;
	f_path.vp[2] = vpstart;

	f_path.vpf[0] = vptraj[0];
	f_path.vpf[1] = vptraj[1];
	f_path.vpf[2] = -vptraj[2];		/* Negative to agree w/ "world" coordinates */

	pathdelayframes = pathdelay/1000*f_framerate;
	pathdelayframes = (int)pathdelayframes;

	f_path.nframes = movieframes;
	f_path.ndelay = pathdelayframes;		// delay before starting motion

	// send the PATH command
	render_path(&f_path);

	// configure moveobject parameters
	f_moveobject.handle = f_fpHandle;

	f_moveobject.initial_pos[0] = fpx_start;
	f_moveobject.initial_pos[1] = fpy_start;

	f_moveobject.final_pos[0] = fpx_end;
	f_moveobject.final_pos[1] = fpy_end;

	f_moveobject.nframes = movieframes + pathdelayframes;

	// send the MOVEOBJECT command
	render_moveobject(&f_moveobject);

	f_fpStruct.xorigin = fpx_start;
	f_fpStruct.yorigin = fpy_start;

	return 0;
}


/***************************** my_path_start ***************************************
 *
 * Send start command.This starts the animations which were configured in my_path_setup.
 *
 */
int my_path_start()
{
	render_start();
	return 0;
}

/***************************** my_path_stop ***************************************
 *
 * Send stop command. This stops the animation. Note that this command must be sent even if the animation has
 * already stopped (that is, it appears to have stopped).
 *
 */
int my_path_stop()
{
	render_stop();
	ecode(STOPPED);
	return 0;
}

/****************************** my_check_for_stop *********************************
*
* Checks for animation stop message from render and returns number of frames since start
*
*/
int my_check_for_stop()
{
	int fr_status=0, frames = 0, fcode;
	fr_status = render_check_for_stop(&frames);
	if (fr_status == 1)
	{
		fcode = 7000 + frames;
		ecode(fcode);
		dprintf("fr_status = %d\n",fr_status);
	}
	
	return fr_status;
}
	

/****************************** my_clear *******************************************
 *
 * Clear screen after animation has stopped by turning off all graphic handles.
 *
 */
int my_clear()
{
	render_onoff(&f_fpHandle, HANDLE_OFF, ONOFF_NO_FRAME);
//	render_onoff(&f_fpHandle2, HANDLE_OFF, ONOFF_NO_FRAME);
//	render_onoff(&f_fpHandle3, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_dotsHandle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_frame(1);
	return 0;
}

/******************************* my_show **********************************
 *
 * Turn on fp and dots, issue RENDER command.
 *
 */
int my_show()
{
	render_update(f_fpHandle, &f_fpStruct, sizeof(f_fpStruct), HANDLE_ON);
	render_update(f_dotsHandle, &f_dotsStruct, sizeof(f_dotsStruct), HANDLE_OFF);
	render_camera(cam_position, cam_looking, cam_up);
	ecode(TRLSTART);
	render_frame(1);
	return 0;
}

/****************************** fp_show ***********************************
*
* Turn on fp
*/
int fp_show()
{
	render_onoff(&f_fpHandle, HANDLE_ON, ONOFF_NO_FRAME);
	render_frame(1);
	return 0;
}

/****************************** ff_show ***********************************
*
* Turn on flow field
*/
int ff_show()
{
	if (blank == 1)
	{
		render_onoff(&f_dotsHandle, HANDLE_OFF, ONOFF_NO_FRAME);
		render_frame(1);
	}
	else
	{		
		render_onoff(&f_dotsHandle, HANDLE_ON, ONOFF_NO_FRAME);
		render_frame(1);
		ecode(FFON);
	}
	return 0;
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

/******************************** rinitf() *********************************/
void rinitf(void)
{
	int status=0;
	dprintf("rinitf\n");
	status = init_tcpip(local_addr, remote_addr, remote_port, 1);
}

/******************************fixwin() **********************************
*
* opens fixation window at new fixation point
*/
int fixwin(xsiz,ysiz)
{
	wd_cntrl(WIND0, WD_OFF);
	wd_pos(WIND0, (long)(fpx_start1*10), (long)(fpy_start1*10));
	wd_siz(WIND0,xsiz,ysiz);
	wd_src_pos(WIND0, WD_DIRPOS, RAMP0, WD_DIRPOS, RAMP0);
	wd_src_check(WIND0, WD_SIGNAL, EYEH_SIG, WD_SIGNAL, EYEV_SIG);
	wd_cntrl(WIND0, WD_ON);
	return 0;
}

/****************************** fwinon() **********************************
 *
 * opens fixation window
*/
int fwinon(xsiz, ysiz)
{
	wd_cntrl(WIND0, WD_OFF);
	ra_start(RAMP0,0,0);
	wd_pos(WIND0, (long)(fpx_start1*10), (long)(fpy_start1*10));
	wd_siz(WIND0, xsiz, ysiz);
	wd_src_pos(WIND0, WD_RAMP_X, RAMP0, WD_RAMP_Y, RAMP0);
	wd_src_check(WIND0, WD_SIGNAL, EYEH_SIG, WD_SIGNAL, EYEV_SIG);
	wd_cntrl(WIND0, WD_ON);
	return 0;
}

/******************************winramp*******************************
*
*sets up ramp for moving window
*/
int winramp(void)
{
	int angle;
	int vel;
	
	if (sp->fixed) vel = 0;
	else vel = (int)(speed/10.0f);
	
	if (pursuit_ang_ovr == NULLI) angle = (int)(sp->pursuit_ang);
	else angle = (int)(pursuit_ang_ovr/10.0f);
	
	ecode(4000 + (long)(angle*10));
	ra_new(RAMP0, total_dot_travel_deg * 10, angle, vel, (int)(fpx_start1*10), (int)(fpy_start1*10), 0, RA_BEGINPT);
	return(0);
}

/******************************rampstart******************************
*
* Initiates ramp
*/
int rampstart(long LED)
{
	ra_start(RAMP0, 1, LED);
	return(0);
}

/******************************rampstop******************************
*
* Ends ramp
*/
int rampstop(void)
{
	ra_stop(RAMP0);
	return(0);
}


/****************************** initial() *******************************
 *
 * initializes the stimulus array
 */


int initial(void)
{
	int i, j;
	double trans;
	int n, npurs;

	dprintf("initial()\n");

	// fetch render parameters -- resolution and frame rate
	render_get_parameters(&f_width, &f_height, &f_framerate);
	dprintf("render parameters: %dx%d@%d\n", f_width, f_height, (int)f_framerate);

	
	
	// initialize pixel conversion
	if (initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, f_width, f_height, stimz))
	{
		dprintf("ERROR in initialize_pixel_conversion: \n"
				"x,y dimensions=(%d, %d)\n"
				"x,y resolution=(%d, %d)\n"
				"screen distance=%d\n", (int)x_dimension_mm, (int)y_dimension_mm, (int)f_width, (int)f_height, (int)stimz);
	}
	
	if (!seedflg)
    {
	    ivunif(seed,0);
	    seedflg++;
    }
	
	// Make sure there aren't too many stim
	if (mov_flg == 0) npurs = 1;
	else if (mov_flg == 1) npurs = 1 + pursuit_ang_num*2;
	else npurs = pursuit_ang_num*2;
	n = (ang_num+bf_con) * npurs;
	if (n > MAXLIST-1)
	{
		rxerr("Too many stim conditions.");
		dprintf("Too many (%d) stim conditions (%d * %d)\n", n, (ang_num+bf_con), npurs);
		abort();
	}
	dprintf("There are %d stim conditions (%d heading * %d pursuit)\n", n, (ang_num+bf_con), npurs);


	nstim = 0;
	sp = &stimlist[0];

	for (i=0; i<(ang_num+bf_con); i++)
	{
		if (mov_flg==0 || mov_flg==1)
		{
			/* Stationary target - no pursuit */
			if (i == ang_num)
			{
				sp->ang = BLACKSCR;
			}
			else
			{
				sp->ang = ang_min + i*(int)(((float)ang_max-(float)ang_min)/(float)(ang_num-1));
			}
			sp->ntot = 0;
			sp->pursuit_ang = 0;	/* Not used - no pursuit */			
			sp->fixed = 1;
			sp->stat = READY;
			nstim++;
			sp++;
			//dump_stim(nstim-1);
		}
		if (mov_flg == 1 || mov_flg == 2)
		{
			float p,pstep;
			pstep = (float)(pursuit_ang_max_10thdeg/10.0f - pursuit_ang_min_10thdeg/10.0f)/(float)(pursuit_ang_num - 1);
			//dprintf("max %d min %d num %d pstep*10 %d\n", (int)(pursuit_ang_max/10.0f), (int)(pursuit_ang_min/10.0f), pursuit_ang_num,  (int)(pstep*10));
			for (p=pursuit_ang_min_10thdeg/10.0f; p<180 && p<=pursuit_ang_max_10thdeg/10.0f; p+=pstep)
			{
				if (i == ang_num)
				{
					sp->ang = BLACKSCR;
				}
				else
				{
					sp->ang = ang_min + i*(int)(((float)ang_max-(float)ang_min)/(float)(ang_num-1));
				}
				sp->ntot = 0;
				sp->pursuit_ang = p;			
				sp->fixed = 0;
				sp->stat = READY;
				nstim++;
				sp++;
				//dump_stim(nstim-1);

				// same, add 180 degrees to pursuit angle
				if (i == ang_num)
				{
					sp->ang = BLACKSCR;
				}
				else
				{
					sp->ang = ang_min + i*(int)(((float)ang_max-(float)ang_min)/(float)(ang_num-1));
				}
				sp->ntot = 0;
				sp->pursuit_ang = p + 180;			
				sp->fixed = 0;
				sp->stat = READY;
				nstim++;
				sp++;
				//dump_stim(nstim-1);
			}
		}
		if (mov_flg < 0 || mov_flg > 2)
		{
	    	rxerr("initial(): bad fix flag s/b 0, 1 or 2");
		}
	}				
	sp->stat = END;

	remain = nstim*ntrials;						/* Number of trials remaining? */
	movieframes = movietime*f_framerate/1000;		/* movietime is in menus; this is number of frames of dot motion */
	trans = translation/f_framerate;		/* translation is total camera movement distance (time implicitly 1 sec). */
	                                            /* trans is camera movement dist per frame */
	pathlength = movieframes*trans;				/* pathlength is a more careful calculation of camera travel distance */

	return(0);
}

/****************************** next_trl() *******************************
 *
 * figures out the next stimulus type in the sequence
 */
int next_trl(void)
{
	int rindx, active;
	int r_num;


	/*
	 * Select trial parameters from stimlist[]
	 */

	sp = &stimlist[0];

	active = 0;
	while (sp->stat != END) if (sp++->stat == READY) active++;
	
	if (!active)	/* done a rep of all trial types; reset flags */
    {
	    for (sp = &stimlist[0]; sp->stat != END; sp++) sp->stat = READY;
	    active = nstim;
    }
    dprintf("There are %d active stim.\n", active);

	rindx = ivunif(0, (active-1));  /* shouldn't be here if counter=nstim */

	/* now skip down into list by this amount */
	sp = &stimlist[0];
	while(sp->stat != READY) sp++;	    /* start pointing at the first live one */
	while(1)
    {
	    while (sp->stat == DONE) sp++;
	    if(rindx <= 0) break;
	    sp++;
	    rindx--;
    }


	/*
	 * Select trial parameters from stimlist[] - DONE
	 */


	/* 
	 * Set vars that govern the camera movement.
	 * The PATH command sets a linear path for the camera to follow. 
	 * It also moves the viewpoint along a separate linear path. 
	 * In this expt the viewpoint is always straight ahead.The path of the
	 * camera is determined by the heading angle (sp->ang) and the speed (speed, expressed in 
	 */

	/*
	 * do some math to get the delta-x and delta-y corresponding to the angle
	 * of motion specified. The velocity is specified in stim-box-width units*100.
	 * Angle is in .1 degree units (hence the /1800).
	 */
	if (ang_ovr == NULLI)
    {
    	if (sp->ang == BLACKSCR)
    	{	
    		end_x = 0;
    		end_z = pathlength;
    		blank = 1;
    	}
    	else
    	{	
		    end_x = sin(PI*sp->ang/1800)*pathlength;
    		end_z = cos(PI*sp->ang/1800)*pathlength;
    		blank = 0;
    	}
    }
	else
    {
	    end_x = sin(PI*ang_ovr/1800)*pathlength;
    	end_z = cos(PI*ang_ovr/1800)*pathlength;
    	blank = 0;
    }
	
	eyetraj[0] = (long)(end_x);
	eyetraj[1] = 0;
	eyetraj[2] = (long)(end_z);
	
	vptraj[0] = (long)(end_x);
	vptraj[1] = 0;
	vptraj[2] = eyetraj[2] - vpstart;

	/* 
	 * Set vars that govern the camera movement - DONE
	 */


	/* 
	 * Set vars that control pursuit
	 * 
	 * We'll need the following:
	 * (fpx_start(fpx_start1), fpy_start(fpy_start1)) fixpt pos initial, before pursuit. In pixels (degrees).
	 * (fpx_end(fpx_end1), fpy_end(fpy_end1))     fixpt pos final, after pursuit. In pixels (degrees).
	 * 
	 * We also use fixx, fixy, the initial position/base position of the fixpt. The pursuit travels through the fixpt, 
	 * with the fispt at the midway point. 
	 * 
	 * The values in pixels are used in graphics commands. The values in degrees are used in REX calls (ramp, window).
	 * 
	 */


	if (sp->fixed)
	{
		fpx_start1 = fpx_end1 = (float)fixx/10.0f;
		fpy_start1 = fpy_end1 = (float)fixy/10.0f;	
		total_dot_travel_deg = 0;	
	}
	else
	{	
		float dot_travel_deg, extra_dot_travel_deg;
		float dx, dy;
		float dxtra, dytra;	/* extra dot travel */
		float angle;

		if (pursuit_ang_ovr == NULLI) angle = sp->pursuit_ang;
		else angle = pursuit_ang_ovr/10.0f;
		
		dot_travel_deg = movieframes * speed / 10 / f_framerate;
		extra_dot_travel_deg = pathdelayframes * speed / 10 / f_framerate;
		
		dx = dot_travel_deg * cos(PI*angle/180);
		dy = dot_travel_deg * sin(PI*angle/180);
		dxtra = extra_dot_travel_deg * cos(PI*angle/180);
		dytra = extra_dot_travel_deg * sin(PI*angle/180);
	
		fpx_start1 = (float)fixx/10.0f - dx/2 - dxtra;
		fpy_start1 = (float)fixy/10.0f - dy/2 - dytra;
		
		fpx_end1 = (float)fixx/10.0f + dx/2;
		fpy_end1 = (float)fixy/10.0f + dy/2;
		
		total_dot_travel_deg = dot_travel_deg + extra_dot_travel_deg;
		
		//dprintf("dot travel, extra deg %d, %d dx,dy,dxtra,dytra %d,%d,%d,%d start1 %d, %d end1 %d,%d\n",
		//		(int)(10*dot_travel_deg), (int)(10*extra_dot_travel_deg), (int)(dx*10), (int)(dy*10), (int)(10*dxtra), (int)(10*dytra), (int)(10*fpx_start1), (int)(10*fpy_start1), (int)(10*fpx_end1), (int)(10*fpy_end1));
	}

	fpx_start = to_pixels(fpx_start1);
	fpy_start = to_pixels(fpy_start1);
	fpx_end = to_pixels(fpx_end1);
	fpy_end = to_pixels(fpy_end1);
	
	//dprintf("fpx,y_start = %d, %d\tfpx,y_end = %d, %d\n", (int)fpx_start, (int)fpy_start, (int)fpx_end, (int)fpy_end);
	

	return(0);
}

/**********************************************************************************************
* Bookkeeping commands
*
**********************************************************************************************/
/**************************** trlcd() ********************************
 *
 * drops an identifying Ecode for stimulus condition
 */
int trlcd(void)
{
	dprintf("trlcd = %d\n",sp->ang);
	return(4000 + sp->ang);
}

/**************************** purscd() ********************************
 *
 * drops an identifying Ecode for pursuit condition
 * djs - drops sp->fixed. 1 means pursuit, 0 means no pursuit
 */
int purscd(void)
{
	dprintf("purscd = %d\n",sp->fixed);
	return(3000 + sp->fixed);
}

/**************************** ovrcd() ********************************
 *
 * drops an flag Ecode if override is set
 */
int ovrcd(void)
{
	if (ang_ovr != NULLI)
	{
		dprintf("angle override %d\n",ang_ovr);
		return(OVRDCD);
	}
	else return(0);
}

/******************************* record() *******************************
 *
 * score response
 */
record()
{
	score (1);
	sp->stat = DONE;
	sp->ntot++;
	return(0);
}



/*************************** pr_info *************************************
 * 
 * prints bookeeping information
 */

pr_info(void)
{
	int i;
	char pstr[2];
	setbuf(stdout, &outbuf);
	printf("angle\t\tpursuit\t\t#\n");
	for (i=0; i<nstim; i++)
    {
	    if (stimlist[i].fixed == 1) sprintf(pstr,"-----");
	    else sprintf(pstr, "%5d", (int)(stimlist[i].pursuit_ang*10));
    	printf("%5d\t\t%s\t\t%3d\n",stimlist[i].ang, pstr, stimlist[i].ntot);
    }
	fflush(stdout);
	setbuf(stdout, 0);
	//dprintf("pr_info outbuf length = %d\n", strlen(outbuf));
	return(0);
}

/************************************************************************
 * Ecode-returning functions for data recording. The order in which these
 * are called is the key to interpreting the header info.
 ************************************************************************/
 
int fpxcd(void) {return(HEADBASE+fixx);}
int fpycd(void) {return(HEADBASE+fixy);}
int fpvcd(void) {int fp_dx = speed; return(HEADBASE+fp_dx);}
int stzcd(void) {return(HEADBASE+stimz);}
int mincd(void) {return(HEADBASE+ang_min);}
int maxcd(void) {return(HEADBASE+ang_max);}
int numcd(void) {return(HEADBASE+ang_num);}
int nptcd(void) {return(HEADBASE+dotnumber);}
int ptszcd(void) {return(HEADBASE+ptsiz);}
int iptcd(void) {return(HEADBASE+init_pt);}
int fpdxcd(void) {int vel = speed/10; return(HEADBASE+vel);}
int sthtcd(void) {return(HEADBASE+stimht);}
int bgcd(void) {return(HEADBASE+bg);}
int fgcd(void) {return(HEADBASE+dc);}
int pursuit_min_cd(void) { return HEADBASE + pursuit_ang_min_10thdeg; };
int pursuit_max_cd(void) { return HEADBASE + pursuit_ang_max_10thdeg; };
int pursuit_num_cd(void) { return HEADBASE + pursuit_ang_num; };


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


VLIST state_vl[] = {
"day_seed",		&seed, NP, NP, 0, ME_DEC,
"dot_number", &dotnumber, NP, NP, 0, ME_DEC,
"fp_x",		&fixx,	NP, NP,	0, ME_DEC,
"fp_y",		&fixy,	NP, NP,	0, ME_DEC,
"fp_vel(deg/10sec)", &speed, NP, NP, 0, ME_DEC,
"duration",	&movietime, NP, NP, 0, ME_DEC,
"bg(0-255)", 	&bg, NP, NP, 0, ME_DEC,
"dot_color(0-255)",		&dc, NP, NP, 0, ME_DEC,
"fp_R(0-255)", 	&red, NP, NP, 0, ME_DEC,
"fp_G(0-255)", 	&green, NP, NP, 0, ME_DEC,
"fp_B(0-255)", 	&blue, NP, NP, 0, ME_DEC,
"CRT_dist(mm)",		&stimz, NP, NP, 0, ME_DEC,
"fp_size(pix)",		&fpsiz, NP, NP, 0, ME_DEC,
"trials",		&ntrials, NP, NP, 0, ME_DEC,
"ang_override",		&ang_ovr, NP, NP, 0, ME_NVALD,
"ang_min",		&ang_min, NP, NP, 0, ME_DEC,
"ang_max",		&ang_max, NP, NP, 0, ME_DEC,
"ang_num",		&ang_num, NP, NP, 0, ME_DEC,
"eye_movement?(0,1,2)",	&mov_flg, NP, NP, 0, ME_DEC,
"pursuit_ang_override", &pursuit_ang_ovr, NP, NP, 0, ME_NVALD,
"pursuit_min_ang", &pursuit_ang_min_10thdeg, NP, NP, 0, ME_DEC,
"pursuit_max_ang", &pursuit_ang_max_10thdeg, NP, NP, 0, ME_DEC,
"pursuit_ang_num", &pursuit_ang_num, NP, NP, 0, ME_DEC,
"local_ip_addr", local_addr, NP, NP, 0, ME_STR,
"remote_ip_addr", remote_addr, NP, NP, 0, ME_STR,
"remote_port", &remote_port, NP, NP, 0, ME_DEC,
NS,
};

/*
 * Help message.
 */

char hm_sv_vl[] = "ang_min, ang_max, pursuit_ang_min,max in 10th degree units\nDO NOT COLLECT DATA WITH THIS PARADIGM!";


/*
 * User-supplied function table.
 */
USER_FUNC ufuncs[] = {
	{"print_info",	&pr_info, "%d"},
	{""},
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
		to stiminit on -PSTOP & softswitch
	pause2:
		to stiminit on -PSTOP & softswitch
	stiminit:
		code PAUSECD
		do initial()
		to renderinit
	renderinit:
		rl 50
		do my_render_init()
		to targhandle
	targhandle:
		to headcd on 1 % my_check_for_handles
	headcd:
		code HEADCD
		time 10
		to fpxcd
	fpxcd:
		do fpxcd()
		to fpycd
	fpycd:
		do fpycd()
		to fpvcd
	fpvcd:
		do fpvcd()
		to stzcd
	stzcd:
		do stzcd()
		to mincd
	mincd:
		do mincd()
		to numcd
	numcd:
		do numcd()
		to maxcd
	maxcd:
		do maxcd()
		to nptcd
	nptcd:
		do nptcd()
		to ptszcd
	ptszcd:
		do ptszcd()
		to iptcd
	iptcd:
		do iptcd()
		to fpdxcd
	fpdxcd:
		do fpdxcd()
		to sthtcd
	sthtcd:
		do sthtcd()
		to bgcd
	bgcd:
		do bgcd()
		to fgcd
	fgcd:
		do fgcd()
		to loop
	loop:
		rl 75
		time 400
		to pause3
	pause3:
		to pause4 on +PSTOP & softswitch
		to nexttrl on -PSTOP & softswitch
	pause4:
		to nexttrl on -PSTOP & softswitch
	nexttrl:
		rl 50
		do next_trl()
		to trlcd
	trlcd:
		do trlcd()
		to purscd
	purscd:
		do purscd()
		to ovrcd
	ovrcd:
		do ovrcd()
		to initpath
	initpath:
		do my_path_setup()
		to rampwin
	rampwin:
		do winramp()
		to frame
	frame:
		rl 10
		do render_frame(0)
		to showinit on 1 % my_check_for_went
	showinit:
		rl 20
		do my_show()
		to winon
	winon:
		code FIXASK
		time 20
		do fixwin(40,30)
		to grace
	grace:
		code FPONCD
		time 4000
		to noise on -WD0_XY & eyeflag
		to wrong
	noise:
		time 100
		to fixtim
	fixtim:
		code FIXCD
		to wrong on +WD0_XY & eyeflag
		to frame2
	frame2:
		do render_frame(0)
		to flowon on 1 % render_check_for_went
	flowon:
		do ff_show()
		to waiting
	waiting:
		code FFONCD
		to grace2
	grace2:
		time 1000
		to noise2 on -WD0_XY & eyeflag
		to wrong
	noise2:
		time 225
		to wrong on +WD0_XY & eyeflag
		to startramp
	startramp:
		do fwinon(40,30)
		to startpath on +RA_STARTED & ramp[RAMP0].ra_rampflag
	startpath:
		code RSTART
		do my_path_start()
		to waitpath
	waitpath:
		code MOVIE_STRT
		to wrong on +WD0_XY & eyeflag
		to stoppath on 1 % my_check_for_stop
	stoppath:
		code FFOFFCD
		to stopramp
	stopramp:
		do ra_stop(RAMP0,0,0)
		to right
	right:
		code CORRECTCD
		do my_clear()
		to reward
	reward:
		time 38
		do dio_on(REW)
		to rewoff
	rewoff:
		do dio_off(REW)
		to goodscr
	goodscr:
		do record()
		to trlcnt
	trlcnt:
		to blkdone on 1 ? remain
		to loop
	blkdone:
		to loop on +TRUE & remain
	wrong:
		code BREAKFIXCD
		do my_path_stop()
		to wrong2 on 1 % my_check_for_stop
	wrong2:
		do my_clear()
		to stopramp2 on +RA_STARTED & ramp[RAMP0].ra_rampflag
		to beep
	stopramp2:
		do ra_stop(RAMP0,0,0)
		to beep
	beep:
		time 300
		do dio_on(BEEP)
		to bpoff
	bpoff:
		do dio_off(BEEP)
		to timout
	timout:
		time 700
		to loop
	pause5:
		to pause6 on +PSTOP & softswitch
		to nothing on -PSTOP & softswitch
	pause6:
		code PAUSECD
		to nothing on -PSTOP & softswitch
	nothing:
		to exit on 1 = f_fpHandle
	exit:
		to pause1

abort list:
		wrong
}

/* $Id: swe_ff3d.d,v 1.4 2008/09/12 22:32:09 devel Exp $ */

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
#define MAXLIST 101

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

int errcnt;
char outbuf[2048];									/* used in pr_info */
static unsigned char fphandle, fprh, trh;      /* handles for fixation point */
static int isfpon = 0, iston = 0;
static int seedflg = 0;

static float ppd;			/* pixels per degree */
static int nstim;

struct stim
    {
    int ang;
    int ntot;
    short fixmov;
    short stat;
    };
static struct stim stimlist[MAXLIST];
static struct stim *sp;

/************************************************************************
 * Declaration of statelist variables.
 ************************************************************************/

int	fixx = 0,
	fixy = 0,
	xpos,
	ypos,
	targx = NULLI,
	targy = NULLI,
	/* eyeh = targx, */
	/* eyev = targy, */
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
	dc = 255;
	mov_flg = 1,
 	ang_min = -300,
	ang_max = 300,
	ang_num = 13,
	ang_ovr = NULLI,
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
	framerate = 85;
long seed = 1111;
float fpsize = 0.2,
	fp_startx,
	fp_starty,
	fpstart,
	pathdelay = 250.0,	// delay in msec
	pathdelayframes,
	velx,
	vely;
double translation = 5000.0,		// in mm per sec
	pathlength;


static long eye[] = {0, 0, 100};		/* location of observer */
static long eyetraj[] = {0, 0, 0};	/* eye translation */
static long vp[] = {0, 0, 100};		/* viewpoint - where the eye is pointed */
static long vptraj[] = {0, 0, 0};	/* it's traj, this will be adjusted later */
static long vpstart = -10;
static double end_x, end_z;		/*speed of camera*/
static float fpx, fpx_start, fpx_start1, fpx_end, fpx_end1, fpy, fpy_start, fpy_start1, fpy_end, fpy_end1, fpdx, fpdy, xinit;
static long planed = PLANED;
static int pursflg;

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
static DotStruct f_fpStruct2;
static DotStruct f_fpStruct3;
static DotFieldStruct f_dotsStruct;

/* For ramp and window movement */
int length;
int angle = 0;
int velocity;
int xoff = 0;
int yoff = 0;


/************************************************************************
 * Control functions to make things happen.
 ************************************************************************/

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

	f_fpStruct2.xorigin = 0;
	f_fpStruct2.yorigin = 0;
	f_fpStruct2.xsize = to_pixels(0.4);
	f_fpStruct2.ysize = to_pixels(0.4);
	f_fpStruct2.depth = 11;
	f_fpStruct2.r = 0;
	f_fpStruct2.g = 255;
	f_fpStruct2.b = 0;
	f_fpStruct2.a = 0;
//	render_dot(&f_fpStruct2);		//will need to get handle return


	f_fpStruct3.xorigin = 0;
	f_fpStruct3.yorigin = 0;
	f_fpStruct3.xsize = to_pixels(0.4);
	f_fpStruct3.ysize = to_pixels(0.4);
	f_fpStruct3.depth = 11;
	f_fpStruct3.r = 0;
	f_fpStruct3.g = 0;
	f_fpStruct3.b = 255;
	f_fpStruct3.a = 0;
//	render_dot(&f_fpStruct3);		//will need to get handle return


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
//			render_onoff(&f_fpHandle, HANDLE_ON, ONOFF_NO_FRAME);
		}/*
		else if (f_handleCount == 1)
		{
			f_fpHandle2 = h;
			f_handleCount = 2;
			dprintf("p handle %d\n",f_fpHandle2);
		}
		else if (f_handleCount == 2)
		{
			f_fpHandle3 = h;
			f_handleCount = 3;
			dprintf("p2 handle %d\n",f_fpHandle3);
		}*/
		else if (f_handleCount == 1)
		{
			f_dotsHandle = h;
			f_handleCount = 2;
			dprintf("dots handle %d\n", f_dotsHandle);
//			render_onoff(&f_dotsHandle, HANDLE_OFF, ONOFF_NO_FRAME);
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

	pathdelayframes = pathdelay/1000*(float)framerate;
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
	f_fpStruct2.xorigin = to_pixels(fpx);
	f_fpStruct3.xorigin = to_pixels(-fpx);

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

	fpx = 0;
	fpy = 0;

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
//	render_update(f_fpHandle2, &f_fpStruct2, sizeof(f_fpStruct2), HANDLE_ON);
//	render_update(f_fpHandle3, &f_fpStruct3, sizeof(f_fpStruct3), HANDLE_ON);
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
	xpos = xoff;
	ypos = yoff;
	wd_pos(WIND0, xpos,ypos);
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
	int rcode;
	if (angle == 180)	{rcode = RAMP_RT;}	else {rcode = RAMP_NA;}
	if (angle == 0)	{rcode = RAMP_LF;}
	
	ecode(rcode);
	ra_new(RAMP0,length,angle,velocity,(int)(fpx_start1*10),(int)(fpy_start1*10),rcode,RA_BEGINPT);
	return(0);
}

/******************************rampstart******************************
*
* Initiates ramp
*/
int rampstart(long LED)
{
	ra_start(RAMP0,1,LED);
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
	int i, j, fmin, fmax;
//	extern struct stim *sp;

	double trans;

	dprintf("initial()\n");
	
	initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, x_resolution, y_resolution, stimz);

if (!seedflg)
    {
    ivunif(seed,0);
    seedflg++;
    }

//ppd = 10;

/*
 * Generate the running array of stimuli.
 */

if (mov_flg == 1)	/* static and moving */
    {
    fmin = 0;
    fmax = 2;
    }
else if (mov_flg == 2)  /* moving only */
    {
    fmin = 1;
    fmax = 2;
    }
else if (!mov_flg)	/* static only */
    fmin = fmax = 0;
else
    rxerr("initial(): bad fix flag");

nstim = 0;
sp = &stimlist[0];

/*
 * all conditions in one loop
 */
for (i=0; i < (ang_num+bf_con); i++)	/* outermost is angle ; detect black screen on/off */
    for (j=fmin; j<=fmax; j++)  	/* next is fixation movement */
	{
	  if (i == ang_num)  /* TRUE when bf_con = 1 */
      {
	  	sp->ang = BLACKSCR;  /* put one more data set with a special num. in condition matrix */
      }
      else
      {
      	sp->ang = ang_min + i*(int)(((float)ang_max-(float)ang_min)/(float)(ang_num-1));
      }
	sp->fixmov = j;
	sp->ntot = 0;
	sp->stat = READY;
	nstim++;
	sp++;
	}

sp->stat = END;

remain = nstim*ntrials;
movieframes = movietime*framerate/1000;
velx = (float)speed/10/(float)framerate;
vely = 0;
fp_startx = velx*movieframes/2;
fp_starty = vely*movieframes/2;
fpstart = to_pixels(fp_startx);
fpx = 0;
fpy = 0;
trans = translation/85.0;
pathlength = movieframes*trans;

//dprintf("speed = %d\tvelx = %d\tfp_startx = %d\tfpstart = %d\n", (int)(speed*1000), (int)(velx*1000), (int)(fp_startx*1000), (int)(fpstart*1000));

return(0);
}

/****************************** next_trl() *******************************
 *
 * figures out the next stimulus type in the sequence
 */
static int next_trl(void)
{
//extern struct stim *sp;
	int rindx, active;
	long sign;
	int r_num;
	float extra;

	sp = &stimlist[0];

	/* get random number for this presentation */
	active = 0;
	while (sp->stat != END) if (sp++->stat == READY) active++;
	
	if (!active)	/* done a rep of all trial types; reset flags */
    {
	    for (sp = &stimlist[0]; sp->stat != END; sp++) sp->stat = READY;
	    active = nstim;
    }

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
dprintf("nstim: %d, active: %d, rindx: %d, sp: %d \n", nstim, active, rindx, (int)(sp-&stimlist[0]));
*/

/* do initial eye position */
eye[0] = (long)init_pt * 10;

/* and projection plane depth: 0=eye, + is towards viewpoint */
planed = (long)((float)eye[0] * 0.1);

/*
NOTE 3/29/96; this warning was removed because with the new, bigger field of
view, we have to get into the cloud of points to keep the velocities consistent
with what we had, and keep the boundaries of the cloud out of view.
if (eye[0] - planed - (vel * 10) < 500) rxerr("warning: approaching too closely");
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
vptraj[0] = (long)(end_x);

/* calculate this trial's trajectories for eye and viewpoint */
eyetraj[2] = (long)(end_z);
vptraj[2] = eyetraj[2] - vpstart;


/* set up the (real, physical) fixation point trajectory */
if (sp->fixmov == 0)
	{
		sign = 0;
		pursflg = 0;
		velocity = 0;
		velx = 0;
		vely = 0;
	}
else if (sp->fixmov == 1) 
	{
		sign = -1;
		pursflg = 1;
		velocity = (int)(speed/10);
		velx = (float)speed/10/(float)framerate;
		vely = 0;
	}
else if (sp->fixmov == 2)
	{
		sign = 1;
		pursflg = 1;
		velocity = (int)(speed/10);
		velx = (float)speed/10/(float)framerate;
		vely = 0;
	}
else dprintf(" fixmov flg not valid \n");

fpx = (float)fixx/10 - sign * fp_startx;
fpx_end1 = (float)fixx/10 + sign * fp_startx;
fpy = (float)fixy/10 - sign * fp_starty;
fpy_end1 = (float)fixy/10 + sign * fp_starty;

/* OLD COMMANDS REPLACED BY STATEMENTS ABOVE
if (sp->fixmov)
    {
    if (sp->fixmov == 1) sign = -1; else sign = 1;
    fpx = (float)fixx/10 - sign * fp_startx;
    fpx_end1 = (float)fixx/10 + sign * fp_startx;
    fpy = (float)fixy/10 - sign * fp_starty;
    fpy_end1 = (float)fixy/10 + sign * fp_starty;
    pursflg = 1;
    }
else
    {
    sign = 0;
    fpx = (float)fixx/10;
    fpx_end1 = (float)fixx/10;
    fpy = (float)fixy/10;
    fpy_end1 = (float)fixy/10;
    pursflg = 0;
    }


if (fpx == (float)fixx/10)
	{
	velocity = 0;
	velx = 0;
	vely = 0;
	fpx_end1 = fpx;
	fpy_end1 = fpy;
	}
else
	{
	velocity = (int)(speed/10);		//degrees per sec
	velx = (float)speed/10/(float)framerate;
	vely = 0;
	fpx_end1 = fpx_end1;
	fpy_end1 = fpy_end1;
	}
*/

extra = sign*(float)pathdelayframes*velx;
fpx_start1 = fpx - extra;
fpx_start = to_pixels(fpx_start1);
fpx_end = to_pixels(fpx_end1);
fpy_start = to_pixels(fpy);
fpy_start1 = fpy;
fpy_end = to_pixels(fpy_end1);
xoff = (long)(fpx_start1*10);
yoff = (long)(fpy_start1*10);
//dprintf("fpx_start = %d\tfpx_end = %d\textra = %d\tfpx = %d\n", (int)fpx_start1, (int)fpx_end1, (int)(extra*1000),(int)(fpx*1000));

if (sign == -1) {length = (int)abs((fpx_start1-fpx_end1)*10); angle = 180; velocity = velocity;}
else if (sign == 1) {length = (int)abs((fpx_start1-fpx_end1)*10); angle = 0; velocity = velocity;}
else {length = (int)abs((fpx_start1-fpx_end1)*10); angle = 0; velocity = velocity;}

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
static int trlcd(void)
{
	dprintf("trlcd = %d\n",sp->ang);
	return(4000 + sp->ang);
}

/**************************** purscd() ********************************
 *
 * drops an identifying Ecode for pursuit condition
 */
static int purscd(void)
{
	dprintf("pruscd = %d\n",sp->fixmov);
	return(3000 + sp->fixmov);
}

/**************************** ovrcd() ********************************
 *
 * drops an flag Ecode if override is set
 */
static int ovrcd(void)
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
	    if (stimlist[i].fixmov == 0) sprintf(pstr,"-");
    	else if (stimlist[i].fixmov == 1) sprintf(pstr,"l");
    	else sprintf(pstr,"r");
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
"fp_vel (deg/10sec)",	&speed, NP, NP, 0, ME_DEC,
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
"local ip addr", local_addr, NP, NP, 0, ME_STR,
"remote ip addr", remote_addr, NP, NP, 0, ME_STR,
"remote port", &remote_port, NP, NP, 0, ME_DEC,
NS,
};

/*
 * Help message.
 */
char hm_sv_vl[] = "";



/*
 * User-supplied function table.
 */
USER_FUNC ufuncs[] = {
	{"print_info",	&pr_info, "%d"},
	{""},
};


/*restart rinitf*/



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

/* $Id: targetxdx_tune.d,v 1.1 2015/05/12 20:53:48 devel Exp $ */
/* For finding the tuning to target position and velocity in the horizantal (x) direction */

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
    float xpos;
    float xvel;
    int ntot;
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
	ypos = 0,
	targx = NULLI,
	targy = NULLI,
	/* eyeh = targx, */
	/* eyev = targy, */
	red = 255,
	green = 0,
	blue = 0,
	fp_red = 0,
	fp_green = 255,
	fp_blue = 0,
	stimz = 280,
	ptsiz = 2,
	dist = 100,
	ntrials = 10,		 /* trials per condition */
	stim_trl = 3,		 /* stim per trial */
	bg = 0;		/* background grayscale value */
	mov_flg = 1,
 	xpos_min = -300,
	xpos_max = 300,
	xpos_num = 13,
	xpos_ovr = NULLI,
	xvel_min = -100,
	xvel_max = 100,
	xvel_num = 5,
	xvel_ovr = NULLI,
	init_pt = 150,
	stim_z = 28,
	ncheck = 20,
	stimwid = 10,
	stimht = 1000,
    bf_con = 1,    /* 0: normal ; 1: add black screen  */
    blank = 0,
	remain,
	stimdur = 500,
	isi = 500,
	use_gaussian =  0,
	coherence = 95,
	npts = 20,
	orientation = 0,
	framerate = 85;
long seed = 1111;
float fpsize = 0.25,
	targsize = 0.25,
	sigma = 0.25,
	fp_startx,
	fp_starty,
	fpstart,
	pointsize = 1.0,
	velx,
	vely;
double translation = 5000.0,		// in mm per sec
	pathlength;


static long eye[] = {0, 0, 0};		/* location of observer */
static long eyetraj[] = {0, 0, 0};	/* eye translation */
static long vp[] = {0, 0, 0};		/* viewpoint - where the eye is pointed */
static long vptraj[] = {0, 0, 0};	/* it's traj, this will be adjusted later */
static long vpstart = -10;
static double xposition, yposition;		/*location of target for next stim*/
static float xdeg, ydeg, xpix, ypix;				/*for updating target location*/
static float xstart, xstart_pix, ystart, ystart_pix, xend, xend_pix, yend, yend_pix, fpdx, fpdy, xinit;
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

/* For fixpt and target */
float fpx, fpy;
int f_fpHandle = 0, f_fpHandle2 = 0, f_fpHandle3 = 0;
int f_targetHandle = 0;
int f_gaussianHandle = 0;
int f_handleCount = 0;
static DotStruct f_fpStruct;
static DotStruct f_targetStruct;
static GaussianDotStruct f_Gaussian;

/* For ramp and window movement */
int angle = 0;
int velocity;
int xoff = 0;
int yoff = 0;

/* Counters */
static int frame_counter,
		isicnt,
		stimcnt,
		trlcunt;


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
	f_fpStruct.xorigin = to_pixels(fpx);
	f_fpStruct.yorigin = to_pixels(fpy);
	f_fpStruct.xsize = to_pixels(fpsize);
	f_fpStruct.ysize = to_pixels(fpsize);
	f_fpStruct.depth = 10;
	f_fpStruct.r = fp_red;
	f_fpStruct.g = fp_green;
	f_fpStruct.b = fp_blue;
	f_fpStruct.a = 0;
	render_dot(&f_fpStruct);		// will need to get handle return

	if (use_gaussian == 0)
	{
		f_targetStruct.xorigin = 0;
		f_targetStruct.yorigin = 0;
		f_targetStruct.xsize = to_pixels(targsize);
		f_targetStruct.ysize = to_pixels(targsize);
		f_targetStruct.depth = 11;
		f_targetStruct.r = red;
		f_targetStruct.g = green;
		f_targetStruct.b = blue;
		f_targetStruct.a = 0;
		render_dot(&f_targetStruct);		//will need to get handle return
	}
	else
	{
		f_Gaussian.origin[0] = f_Gaussian.origin[1] = 0;
		f_Gaussian.offset[0] = f_Gaussian.offset[1] = 0;
		f_Gaussian.pseed = f_Gaussian.cseed = 0;
		f_Gaussian.radius = to_pixels(targsize);
		f_Gaussian.sigma = sigma;
		f_Gaussian.coherence = (float)coherence/100.0f;
		f_Gaussian.orientation = 0;
		f_Gaussian.npts = npts;
		f_Gaussian.depth = 1;
		f_Gaussian.pointsize = pointsize;
		f_Gaussian.r = red;
		f_Gaussian.g = green;
		f_Gaussian.b = blue;
		f_Gaussian.a = 0;
		render_gaussiandot_p(&f_Gaussian, 0);
	}
	
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
			if (use_gaussian == 0)
			{
				f_targetHandle = h;
				f_handleCount = 2;
				dprintf("target handle %d\n", f_targetHandle);
			}
			else
			{
				f_gaussianHandle = h;
				f_handleCount = 2;
				dprintf("target handle %d\n", f_gaussianHandle);
			}
				
//			render_onoff(&f_targetHandle, HANDLE_OFF, ONOFF_NO_FRAME);
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

/************************* update_target ************************************
 * 
 * 	Updates target structure and sends command to update render object
 */
 static int update_target(void)
 {
	xdeg = xdeg + velx/(framerate-1);
 	ydeg = ydeg + vely/(framerate-1);
 	xpix = to_pixels(xdeg);
 	ypix = to_pixels(ydeg);
 	if (use_gaussian == 0)
 	{
	 	f_targetStruct.xorigin = xpix;
		f_targetStruct.yorigin = ypix;
		if (blank == 1)
		{
			f_targetStruct.r = 0;
			f_targetStruct.g = 0;
			f_targetStruct.b = 0;
		}
		else
		{
			f_targetStruct.r = red;
			f_targetStruct.g = green;
			f_targetStruct.b = blue;	
		}
		render_update(f_targetHandle, &f_targetStruct, sizeof(f_targetStruct), HANDLE_ON);
 	}
 	else
 	{
 		f_Gaussian.origin[0] = xpix;
		f_Gaussian.origin[1] = ypix;
		if (blank == 1)
		{
			f_Gaussian.r = 0;
			f_Gaussian.g = 0;
			f_Gaussian.b = 0;
		}
		else
		{
			f_Gaussian.r = red;
			f_Gaussian.g = green;
			f_Gaussian.b = blue;	
		}
		render_update(f_gaussianHandle, &f_Gaussian, sizeof(f_Gaussian), HANDLE_ON);
 	}
 	
	
	return 0;
 }

/****************************** traget_off *****************************************
 * 
 * Turns target off
 * 
 */
 int target_off(int flag)
 {
 	if (use_gaussian == 0)
 	{
	 	render_onoff(&f_targetHandle, HANDLE_OFF, ONOFF_NO_FRAME);
 	}
 	else
 	{
 		render_onoff(&f_gaussianHandle, HANDLE_OFF, ONOFF_NO_FRAME);
 	}
 	
 	if (flag)
 	{
 	  sp->stat = DONE;
      sp->ntot++;
      remain--;
 	}
 	
 	render_frame(0);
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
	if (use_gaussian == 0)
	{
		render_onoff(&f_targetHandle, HANDLE_OFF, ONOFF_NO_FRAME);
	}
	else
	{
		render_onoff(&f_gaussianHandle, HANDLE_OFF, ONOFF_NO_FRAME);
	}
	render_frame(1);
	
	/* reset stim counter for next trial */
	stimcnt = stim_trl - 1;
	return 0;
}


/****************************** fp_show ***********************************
*
* Turn on fp
*/
int fp_show()
{
	render_update(f_fpHandle, &f_fpStruct, sizeof(DotStruct), HANDLE_ON);
	render_frame(0);
	return 0;
}

/****************************** target_show ***********************************
*
* Turn on flow field
*/
int target_show()
{
	if (blank == 1)
	{
		if (use_gaussian == 0)
		{
			render_onoff(&f_targetHandle, HANDLE_OFF, ONOFF_NO_FRAME);
		}
		else
		{
			render_onoff(&f_gaussianHandle, HANDLE_OFF, ONOFF_NO_FRAME);
		}
	}
	else
	{
		if (use_gaussian == 0)
		{
			render_onoff(&f_targetHandle, HANDLE_ON, ONOFF_NO_FRAME);
		}
		else
		{
			render_onoff(&f_gaussianHandle, HANDLE_ON, ONOFF_NO_FRAME);
		}
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
	status = init_tcpip(local_addr, remote_addr, remote_port, 1);

}

/****************************** winon() **********************************
 * 
 * opens fixation window (calls a bunch of library functions from REX to
 * do so)
 */
static int winon(long xsiz, long ysiz)
{	
	wd_src_pos(WIND0, WD_DIRPOS, 0, WD_DIRPOS, 0); 
	wd_pos(WIND0, (long)fixx, (long)fixy);
	wd_siz(WIND0, (long)xsiz, (long)ysiz);
	wd_cntrl(WIND0, WD_ON);
	wd_src_check(WIND0,WD_SIGNAL, EYEH_SIG, WD_SIGNAL, EYEV_SIG);	
	
	return 0;
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

	initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, x_resolution, y_resolution, stimz);


if (!seedflg)
    {
    ivunif(seed,0);
    seedflg++;
    }

/* Set fixation point */
fpx = (float)fixx/10;
fpy = (float)fixy/10;


/*
 * Generate the running array of stimuli.
 */

nstim = 0;
sp = &stimlist[0];

/*
 * all conditions in one loop
 */
for (i=0; i < (xpos_num); i++)	/* outermost is target position ; detect black screen on/off */
{
    for (j=0; j< (xvel_num); j++)  	/* next is target velocity */
	{
      sp->xpos = xpos_min + i*(int)(((float)xpos_max-(float)xpos_min)/(float)(xpos_num-1));
  	  sp->xvel = xvel_min + j*(int)(((float)xvel_max-(float)xvel_min)/(float)(xvel_num-1));
	  sp->ntot = 0;
	  sp->stat = READY;
	  nstim++;
	  sp++;
	}
}

if (bf_con == 1)
{
	sp->xpos = 888.0;
	sp->xvel = 888.0;
	sp->ntot = 0;
	sp->stat = READY;
	nstim++;
	sp++;
}
sp->stat = END;


remain = nstim*ntrials;

return(0);
}

/****************************** next_stim() *******************************
 *
 * figures out the next stimulus type in the sequence
 */
static int next_stim(void)
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

/* Reset frame and isi counter */
frame_counter = stimdur*85/1000 - 2;	/* minus 2 because first frame of target is before stimulus loop */
isicnt = isi*85/1000 - 1;


/* Get the target velocity */
if (xvel_ovr == NULLI)
	{
		velx = (float)sp->xvel/10;
		vely = 0;
		
		/* drop ecode specifying next stimulus velocity (4000 base) */
		ecode(4000+sp->xvel);
	}
else
	{
		velx = (float)xvel_ovr/10;
		
		/* drop ecode specifying override stimulus velocity (4000 base) */
		ecode(4000+xvel_ovr);
	}

/* find next target position */
if (xpos_ovr == NULLI)
    {
    	if (sp->xpos == 888.0)
    	{
    		/* TODO: blank 
    		 * blank = 1;
    		 */
    		xdeg = 170; //(float)sp->xpos/10 - (float)(frame_counter+1)*velx/(framerate-1)/2 - velx/(framerate-1);
    		ydeg = 170;
    		blank = 1;
    	}
    	else
    	{
    		xdeg = (float)sp->xpos/10 - (float)(frame_counter+1)*velx/(framerate-1)/2 - velx/(framerate-1);
    		ydeg = (float)ypos/10;
    		blank = 0;
    	}
    	/* drop ecodes specifying next stimulus position (3000 base) */
		ecode(3000+sp->xpos);
    	
    }
else
    {
    	if (sp->xpos == 888.0)
    	{
    		xdeg = 170;
    		ydeg = 170;
    		blank = 1;
    	}
    	else
		{    		
    		xdeg = (float)xpos_ovr/10 - (float)(frame_counter+1)*velx/(framerate-1)/2 - velx/(framerate-1);
    		ydeg = (float)ypos/10;
    		blank = 0;
		}
		
    	/* drop ecodes specifying next stimulus position (3000 base) */
		ecode(3000+xpos_ovr);
    	
    }


/* calculate this trial's trajectories for eye and viewpoint */
eyetraj[2] = 0;
vptraj[2] = 0;



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
	dprintf("trlcd = %d\n",(int)(sp->xpos*10));
	
	return(4000 + (int)(sp->xpos*10));
}


/**************************** ovrcd() ********************************
 *
 * drops an flag Ecode if override is set
 */
static int ovrcd(void)
{
	if (xpos_ovr != NULLI)
	{
		dprintf("angle override %d\n",xpos_ovr);
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
	/*score (1);
	sp->stat = DONE;
	sp->ntot++;*/
	dprintf("success\n");
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
	printf("pos\t\tvel\t\t  #\n");
	for (i=0; i<nstim; i++)
    {
	    printf("%3d\t\t%3d\t\t%3d\n",(int)(stimlist[i].xpos),(int)(stimlist[i].xvel*10),stimlist[i].ntot);
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

/* fixation point codes */
int	fpxcd(void){return(HEADBASE+fixx);}
int	fpycd(void){return(HEADBASE+fixy);}
 	
/* position min and max codes */
int	xmincd(void){return(HEADBASE+xpos_min);}
int	xmaxcd(void){return(HEADBASE+xpos_max);}
int	xnumcd(void){return(HEADBASE+xpos_num);}
 	
/* velocity min and max codes */
int dxmincd(void){return(HEADBASE+xvel_min);}
int	dxmaxcd(void){return(HEADBASE+xvel_max);}
int	dxnumcd(void){return(HEADBASE+xvel_num);}
 	
/* stimulus property codes */
int targsizecd(void){return(HEADBASE + (int)(targsize*10));}
int	targtypecd(void){return(HEADBASE+use_gaussian);}
int targcolorR(void){return(HEADBASE+red);}
int targcolorG(void){return(HEADBASE+green);}
int targcolorB(void){return(HEADBASE+blue);}
int targsigma(void){return(HEADBASE+sigma);}
int targcoherence(void){return(HEADBASE+coherence);}
int targorientation(void){return(HEADBASE+orientation);}
int targnpts(void)
	{
		int val = 0;
		if (use_gaussian == 0)
		{
			val = 0;
		}
		else
		{
			val = npts;
		}
		
		return(HEADBASE+val);
	}
int targpointsize(void)
	{
		int val = 0;
		if (use_gaussian == 1)
		{
			val = (int)(pointsize);
		}
		
		return(HEADBASE+val);
	}
	
int stzcd(void) {return(HEADBASE+stimz);}
int durcd(void) {return(HEADBASE + stimdur);}
int isicd(void) {return(HEADBASE + isi);}
int bgcd(void) {return(HEADBASE+bg);}
int dayseedcd(void) {printf("%d\n",HEADBASE+seed); return(HEADBASE + seed);}


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
"fp_x",		&fixx,	NP, NP,	0, ME_DEC,
"fp_y",		&fixy,	NP, NP,	0, ME_DEC,
"duration",	&stimdur, NP, NP, 0, ME_DEC,
"isi", 		&isi, NP, NP, 0, ME_DEC,
"bg(0-255)", 	&bg, NP, NP, 0, ME_DEC,
"targ_R(0-255)", 	&red, NP, NP, 0, ME_DEC,
"targ_G(0-255)", 	&green, NP, NP, 0, ME_DEC,
"targ_B(0-255)", 	&blue, NP, NP, 0, ME_DEC,
"fp_R(0-255)", 	&fp_red, NP, NP, 0, ME_DEC,
"fp_G(0-255)", 	&fp_green, NP, NP, 0, ME_DEC,
"fp_B(0-255)", 	&fp_blue, NP, NP, 0, ME_DEC,
"CRT_dist(mm)",		&stimz, NP, NP, 0, ME_DEC,
"fp_size(deg)",		&fpsize, NP, NP, 0, ME_FLOAT,
"targ_size(deg)",	&targsize, NP, NP, 0, ME_FLOAT,
"targ_sig(deg)",	&sigma,	NP,	NP,	0,	ME_FLOAT,
"trials",		&ntrials, NP, NP, 0, ME_DEC,
"stim/trial",	&stim_trl, NP, NP, 0, ME_DEC,
"xpos_override",		&xpos_ovr, NP, NP, 0, ME_NVALD,
"xpos_min",		&xpos_min, NP, NP, 0, ME_DEC,
"xpos_max",		&xpos_max, NP, NP, 0, ME_DEC,
"xpos_num",		&xpos_num, NP, NP, 0, ME_DEC,
"xvel_override",		&xvel_ovr, NP, NP, 0, ME_NVALD,
"xvel_min",		&xvel_min, NP, NP, 0, ME_DEC,
"xvel_max",		&xvel_max, NP, NP, 0, ME_DEC,
"xvel_num",		&xvel_num, NP, NP, 0, ME_DEC,
"use_gaussian",	&use_gaussian, NP, NP, 0, ME_DEC,
"use_blank",	&bf_con,	NP,	NP,	0,	ME_DEC,
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
		to headbasecd on 1 % my_check_for_handles
	headbasecd:
		code HEADBASE
		to headcd
	headcd:
		code HEADCD
		to fpxcd
	fpxcd:
		do fpxcd()
		to fpycd
	fpycd:
		do fpycd()
		to xmincd
	xmincd:
		do xmincd()
		to xmaxcd
	xmaxcd:
		do xmaxcd()
		to xnumcd
	xnumcd:
		do xnumcd()
		to dxmincd
	dxmincd:
		do dxmincd()
		to dxmaxcd
	dxmaxcd:
		do dxmaxcd()
		to dxnumcd
	dxnumcd:
		do dxnumcd()
		to targsizecd
	targsizecd:
		do targsizecd()
		to targtypecd
	targtypecd:
		do targtypecd()
		to targcolorR
	targcolorR:
		do targcolorR()
		to targcolorG
	targcolorG:
		do targcolorG()
		to targcolorB
	targcolorB:
		do targcolorB()
		to targsigmacd
	targsigmacd:
		do targsigma()
		to targcoherence
	targcoherence:
		do targcoherence()
		to targorientation
	targorientation:
		do targorientation()
		to targnpts
	targnpts:
		do targnpts()
		to targpointsize
	targpointsize:
		do targpointsize()
		to stimzcd
	stimzcd:
		do stzcd()
		to durcd
	durcd:
		do durcd()
		to isicd
	isicd:
		do isicd()
		to bgcd
	bgcd:
		do bgcd()
		to dayseedcd
	dayseedcd:
		do dayseedcd()
		to loop
	loop:
		rl 75
		time 400
		to pause3
	pause3:
		to pause4 on +PSTOP & softswitch
		to ovrcd on -PSTOP & softswitch
	pause4:
		to ovrcd on -PSTOP & softswitch
	ovrcd:
		do ovrcd()
		to targoffinitial
	targoffinitial:
		do target_off()
		to fpshow on 1 % render_check_for_went
	fpshow:
		code FIXASK
		do fp_show()
		to winon on 1 % my_check_for_went
	winon:
		code FPONCD
		time 20
		do winon(30,30)
		to grace
	grace:
		time 4000
		to noise on -WD0_XY & eyeflag
		to wrong
	noise:
		time 100
		to fixtim
	fixtim:
		code FIXCD
		to wrong on +WD0_XY & eyeflag
		to stimloop
	stimloop:
		rl 50
		do next_stim()
		to stimon
	stimon:
		code STIMASK
		do update_target()
		to frame
	frame:
		do render_frame(0)
		to stimcode on 1 % my_check_for_went
	stimcode:
		code STIMON
		to targetupdate
	targetupdate:
		do update_target()
		to frameloop
	frameloop:
		do render_frame(0)
		to framecount on 1 % my_check_for_went
	framecount:
		to wrong on +WD0_XY & eyeflag
		to targoff on 0 ? frame_counter
		to targetupdate
	targoff:
		do target_off(1)
		to offcode on 1 % my_check_for_went
	offcode:
		code STIMOFF
		to isi
	isi:
		to wrong on +WD0_XY & eyeflag
		to stimcount on 0 ? isicnt
		to isi2
	isi2:
		render_frame(0)
		to wrong on +WD0_XY & eyeflag
		to isi on 1 % my_check_for_went
	stimcount:
		to wrong on +WD0_XY & eyeflag
		to right on 0 ? stimcnt
		to stimloop
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
		time 100
		to blkdone on -TRUE & remain
		to loop
	blkdone:
		to loop on +TRUE & remain
	wrong:
		code BREAKFIXCD
		do my_clear()
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


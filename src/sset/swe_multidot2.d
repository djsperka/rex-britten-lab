/* swe_multidot.d, dotfield tuning paradigm.
 * 
 * based on multidot.d for old render
 		- adjusted for rex7.8
 		- replaced old render commands with new ones
 		- created new functions to implement new render correctly
 		- led state list not used
 		- SWE 09.26.07
 		- VERSIONS:
 			v1:
 			v2:swe 12/ /2007; added option of blanks to the stimulus set
 *
 * based on dot.d, which had one 1s stimulus per trial
       - adjusted for rex5 by Richard 3 April 1996
       - minor bug fixes and 't e' support 6/25/96. KHB.
 *      
 * created 2/23/98. HWH.      
 *
 * this paradigm presents spiral space and linear dot-fields
 * to measure direction tuning. The number of stimuli per
 * trial, duration of stimuli, and the isi are all adjustable
 * and are accessed from a submenu of the st menu. These parameters
 * control the total duration of a trial. The number of each type
 * of stimulus (linear or spiral space) is independently controlled
 * in the st menu by the theta_min, theta_max, and theta_num parameters
 * prefixed by l for linear or r for radial/rotary.  The direction
 * of each stimulus is calculated by theta_max-theta_min divided into
 * equal steps (number of steps == theta_num. 
 *
 *
 * Secondary state list set control fixation monitoring LED added
 * 4/03/98 HWH.  Allows user control of fixation LED by setting
 * a variable in the statelist (st menu) true. This variable, fix_led,
 * is by default false (0); setting it to 1 turns on the monitoring.
 * Note that to get this to work, a second step had to be added in
 * the countdown; this results in needing to divide the duration counter
 * by 2. This is done when the counter is reset for each trial, and is
 * out of sight during running conditions.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "ldev.h"	/* hardware device defines */
#include "lcode.h"	/* Ecodes */
#include "lcalib.h"
#include "pixels.h"		/* pixel <==> degree conversion */

#include "make_msg.h"	/* generates msgs, and provides all structs for commands.See /rexr3c/*.h */
#include "actions.h"	/* action lib for render */

#include "ivunif.c"	 /*the random-number generator */

/* OLD COMMANDS
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <i86.h>
#include "ldev.h"	hardware device defines 
#include "lcode.h"	Ecodes 
#include "calib.h"  	SGI stimulus display parameters 
#include "render.h"	 defines for graphics objects 
#include "locfunc.h"	 local, commonly used functions 
#include "msg.h"	messaging functions
#include "ivunif.c"	 the random-number generator */

#define MAXLIST 75   /* maximum number of stimuli possible:
			this limits it to either 5 degree
			steps for a single type (linear or spiral
			space) or 10 degree steps if doing both */
#define OVR 75

/*
 * codes for the status flag in the stimulus array
 */
#define READY 0
#define DONE 1
#define END 2

#define PIXSIZE 3
#define BLANK 3

#define FALSE 0
#define TRUE (~FALSE)

#define WIND0		0
#define EYEH_SIG	0
#define EYEV_SIG	1

#define PI 3.14159265


/* Ecodes */
#define MINIMUMCD 1025
#define TTEST 7777


/**********************************************************************
 * Global variables.
 **********************************************************************/

// Render Structures
DotStruct f_fpStruct;		// fp parameters
static FF2DStruct f_ff2dStruct[(MAXLIST + 1)];	// ff2d parameters
int f_fpHandle = 0;			// handle for dot
int f_ff2dHandle[MAXLIST + 1];		// handle for ff2d
int f_handleCount=0;		// used in my_check_for_handles()
int f_i = 0;


// Render related variables
static int framerate = 85;

/* counters, flags, and seed declarations */
static int seedflg = 0;
static int nstim,                 /* number of stimuli configured */
	   stim_number,
       stimcnt,               /* counter for stimuli per trial */
	   durcnt,		  /* counter for duration */
	   isicnt,                /* counter for isi */
       errcnt,                /* error count */
       gtrcnt;                /* good trial count */
static int alldone;               /* through with this block! */
static long cseed = 0, pseed = 0; /* random seeds for flow field */
static int fpon = 0, dfon = 0;    /* flags for fixation point & stimuli on */
static int window_on = 0;         /* flag for fixation window on */
static int fix_led = 0;		  /* if 1, have fixation point LED on */
static int stimindex = 76;

/* set luminance and dot size */
static unsigned char dotsize = PIXSIZE, fg_grey, bg_grey;

/*now the stimulus structure:
  a stimulus is either linear or spiral-space;
  has a direction parameter theta;
  and has a count of number of times completed */

struct stim {
   int ind;
   int handle;
   int islinear;
   int theta;
   short stat;
   int ngood;
};

/* stimulus array */
static struct stim stimlist[(MAXLIST+1)];

/* pointer for moving through stimulus array */
static struct stim *sp = NULL;

/* fixation point and dotfield handles */
static unsigned char fphandle,
 	dflhandle, 
	dfrhandle,
	ffhandle;
static int fprh, ffrh;            /* realization handles */

//extern Msg *msg;

/************************************************************************
 * Declaration of statelist variables.
 ************************************************************************/

static long seed = 1111,  /*day seed for random generator */
			rtheta_ovr = NULLI, /*spiral space direction over-ride: if both on, default*/
			ltheta_ovr = NULLI; /*linear direction over-ride */

static int 	
	rtheta_min = 0,
	rtheta_max = 315,
	rtheta_num = 3,  /* default is 45 degree steps */
	ltheta_min = 0,
	ltheta_max = 315,
	ltheta_num = 3, /*default is 90 degree steps */
	blnk_con = 1;
	speed = 200,
	fixx = 0,
	fixy = 0,
	fpsiz = 3,		/* in tenths of degrees */
	stimx = 0,
	stimy = 0,
	stimr = 100,
	stimz = 280,		/* in mm */	
	ptsiz = 2;		/* pixel size of dots */
	bar_R = 255,	/* dot color */
	bar_G = 255,	/* dot color */
	bar_B = 255,	/* dot color */
	coh = 750,		/* in parts per 1000 */
	stim_trial = 3, /*stimuli per trial */
	nreps = 1000,	/* trials per condition */
	density = 60,	/* hundred dots per cm*cm */
	duration = 500, /*duration of stimulus in ms */
	isi = 450, /* duration of inter-stimulus interval */		
	bg = 0;
	remain;


/* TCP/IP info for render */
char local_addr[16] = "192.168.1.1";
char remote_addr[16]="192.168.1.2";
int remote_port = 2000;


/************************************************************************
 * 
 * Control functions to make things happen.
 *
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


/******************************** rinitf ********************************
 * 
 * reinitializes lan connection on "reset states"
 */
void rinitf(void)
{
	int status=0;
	status = init_tcpip(local_addr, remote_addr, remote_port, 0);
} 

/******************************** setbg() ********************************
 * 
 * Sets background lumaniance
 * swe 09/27/07
 *
 */

int setbg()
{
	render_bgcolor(bg, bg, bg);
	return 0;
}

/************************ swe_check_for_handles() ***********************
 * 
 * Checks for and recieves all handles for following trial
 * swe 09.28.07
 * 
 */
 
 int swe_check_for_handles()
 {
 	int h=0;
 	int stimN = rtheta_num + ltheta_num + blnk_con;
 	
	while (render_check_for_handle(&h))
	{
		if (f_handleCount == 0)
		{
			f_fpHandle = h;
			f_handleCount = 1;
			dprintf("fp handle %d\n", f_fpHandle);
			render_onoff(&h,HANDLE_OFF, ONOFF_NO_FRAME);
		}
		else if (f_handleCount >= 1 && f_handleCount <= stimN)
		{
			f_ff2dHandle[f_i] = h;
			stimlist[f_i].handle = h;
			dprintf("ff2d handle %d\n", f_ff2dHandle[f_i]);
			render_onoff(&h, HANDLE_OFF, ONOFF_NO_FRAME);
			f_handleCount++;
			f_i++;
		}
		else
		{
			f_ff2dHandle[OVR] = h;
			stimlist[OVR].handle = h;
			f_handleCount++;
			dprintf("Override Handle %d\n", f_ff2dHandle[OVR]);
			render_onoff(&h, HANDLE_OFF, ONOFF_NO_FRAME);
		}
	}
	
	return(f_handleCount >= (stimN+2));
 }
 
/********************** swe_check_for_went *********************************/

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
	//	dprintf("%d\n", frames);	* for checking timing with frames *
	}	
	
	return wstatus;
}

 
/********************************* my_conf_all() *************************
 * 
 * configures all flow fields for next trial and configures fp
 * swe 09.27.07
 * 
 */
 
int my_conf_all()
{
	int i;
	
	conf_fp(&f_fpStruct);	// configure and render fp
	render_dot(&f_fpStruct);
	
	// configure and render stim on sitmlist
	for (i = 0; stimlist[i].stat != END; i++)
	{
		conf_ff2d(&f_ff2dStruct[i],i);
		render_ff2d(&f_ff2dStruct[i]);
	}
	// configure and render override stim
	conf_ff2d(&f_ff2dStruct[OVR],OVR);
	render_ff2d(&f_ff2dStruct[OVR]);
	
	return 0;
}
/********************************* conf_fp() *****************************
 * 
 * configures the fixation point
 * 
 * 9/27/07 swe
 * changed to make configure the fixation point (size, location, color, etc)
 * 
 */
int conf_fp(DotStruct *pdot)
{
	// set dot parameters. Note color is hard-coded!
	int siz;
	float fx = to_pixels((float)fixx/10);
	float fy = to_pixels((float)fixy/10);
	siz = to_pixels((float)fpsiz/10);
	pdot->xorigin = fx;
	pdot->yorigin = fy;
	pdot->xsize = siz;
	pdot->ysize = siz;
	pdot->depth = 10;
	pdot->r = 255;
	pdot->g = 0;
	pdot->b = 0;
	pdot->a = 0;
	
	//vsend(MSG_CFGPT, "bbbbb", CIRCLE, 8, 255, bg_grey, bg_grey); OLD FUNCTION
	return 0;
}

/***************************** get_fphandle() *****************************
 * 
 * gets the handle for the fixation point
 *
int get_fphandle(void)
{
	if ((fphandle=gethandle())==-1)
	{
		rxerr("get_fphandle(): invalid message: expecting handle");
		return ERRORCD;
	}
	return 0;
}
*/

/****************************** fpask() *******************************
 *
 * requests that a FP be displayed
 */
int fpask(void)
{
	/* swe 09/28/07
	 * new render commands */
	render_update(f_fpHandle, &f_fpStruct, sizeof(DotStruct), HANDLE_ON);
	render_frame(0);
	fpon = 1;
	
	/* OLD COMMANDS
	vsend(MSG_SHOW, "bbllll", fphandle, 0,
		xpix(fixx, stimz),
		ypix(fixy, stimz),
		0,0);
	fprh = gethandle();
	send(MSG_FRAME);
	fpon = 1;	
	*/
	
	return 0;
}


/**************************** fpoff() ************************************
 *
 * turns off fixation point
 */

static int fpoff(void)
{
   
   if (fpon)
   {
		/* swe 09.28.07 
		 * new render commands */
		render_onoff(&f_fpHandle, HANDLE_OFF, ONOFF_NO_FRAME);
		render_frame(1);
		fpon = 0;
		
		ecode(FPOFFCD);
   		
      /* OLD COMMANDS
      vsend(MSG_UNSHOW, "bb", fphandle, fprh);
      send(MSG_FRAME);
      getmsg();
      fpon = 0;
      */
   }

   return 0;
}



/***************************** stimoff() *********************************
 *
 * turns off the fixation point and the dot fields if they're on.
 */
static int stimoff(int flag)
{
	if (rtheta_ovr != NULLI || ltheta_ovr != NULLI)
	{
		render_onoff(&f_ff2dHandle[OVR], HANDLE_OFF, ONOFF_NO_FRAME);
		render_frame(1);
	}
	else
	{  
    	render_onoff(&f_ff2dHandle[stimindex], HANDLE_OFF, ONOFF_NO_FRAME);
    	render_frame(1);
      /* OLD COMMANDS
      vsend(MSG_UNSHOW, "bb", ffhandle, ffrh);
      send(MSG_FRAME);
      getmsg();
      dfon = 0;
      */
    }

  if (flag)
    {
      sp->stat = DONE;
      sp->ngood++;
      stimcnt --;
    }
	 
  return 0;
}



/****************************** initial() *******************************
 *
 * initializes the stimulus array
 */
static int initial(void)
{
   int i;
   int ndx = 0; /* stimulus array index */
   int theta; /* direction parameter for stimulus array configuration*/

	// Set f_handleCount and f_i to zero after reset states
	f_handleCount = 0;
	f_i = 0;
	
	
	// Initialize "to_pixels();
	initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, x_resolution, y_resolution, stimz);	// Initializes to_pixels function properties
	
	
  /* initialize counters */
   errcnt = 0;
   gtrcnt = 0;
   stimcnt = stim_trial;
   
   /* free up any old handles lying around
   send(MSG_FREE); */
   
   /*initialize random generator with today's seed*/
   if (!seedflg)
     {
       ivunif(seed, 0);
       seedflg++;
     }
   
   /* Generate the running array of stimuli. */
	
	stim_number = rtheta_num + ltheta_num + blnk_con;
	
      if ((nstim = rtheta_num + ltheta_num) >= MAXLIST)
	{
	  rxerr("initial(): too many stimuli in array");
       	  return 0;
	}

      for (i = 0, theta = rtheta_min; i < rtheta_num; i++, ndx++)
	{
	  stimlist[ndx].ind = ndx;
	  stimlist[ndx].islinear = 0;
	  stimlist[ndx].theta = theta;
	  stimlist[ndx].stat  = READY;
	  stimlist[ndx].ngood = 0;
	  theta += (rtheta_max - rtheta_min)/(rtheta_num - 1);
	}
		
      for (i = 0, theta = ltheta_min; i < ltheta_num; i++, ndx++)
	{
	  stimlist[ndx].ind = ndx;
	  stimlist[ndx].islinear = 1;
	  stimlist[ndx].theta = theta;
	  stimlist[ndx].stat = READY;
	  stimlist[ndx].ngood = 0;
	  theta += (ltheta_max - ltheta_min)/(ltheta_num - 1);
	}
	  stimlist[ndx].ind = ndx;
	  stimlist[ndx].islinear = BLANK;
	  stimlist[ndx].theta = 0;
	  stimlist[ndx].stat = READY;
	  stimlist[ndx].ngood = 0;	
      stimlist[(ndx+1)].stat = END;
      
      // Generate override stimlist
      stimlist[OVR].ind = OVR;
      stimlist[OVR].islinear = 1;
      stimlist[OVR].theta = theta;
      stimlist[OVR].stat = OVR;
      stimlist[OVR].ngood = 0;

   /* set up number of trials necessary to complete 
    * repetitions of all stimuli given the specified number of stimuli
    * per trial as given in the st menu. If the number of stimuli
    * is not evenly divisible by the number of stimuli per trial,
    * an extra trial is required, and some stimuli may be presented
    * an extra time to complete that trial */

   remain = (nreps * nstim) / stimcnt;
   if ((nstim % stimcnt) != 0) remain++;
   
   return 0;
}

/***************************** newtrial() ********************************
 *
 * resets the counters for stimuli per trial, duration, and isi
 * also reseeds random dot fields, and configures the dotfields 
 * with those seeds
 */
int newtrial(void)
{
	conf_fp(&f_fpStruct);  /*fixation point configuration because handle freed*/

	getseeds();  /* get pseed and cseed for this trial */
	//conf_ff2d(&f_ff2dStruct[stimindex],stimindex);	* use this trial's seeds and configure both dot fields *

	stimcnt = stim_trial; /*initialize stimulus count parameter*/
	
	return 0;
}

/***************************** getseeds() ********************************
 *
 * gets the random seeds for this trial
 * called by newtrial()
 */

int getseeds(void)
{
   pseed = (long)ivunif(0, 1000);
   cseed = (long)ivunif(0, 1000);
   
   //printf("p = %d\tc= %d\n",pseed,cseed);
   
   return 0;
}

/***************************** conf_ff2d() *********************************
 *
 * 
 * swe 9/26/07
 * now used to configure any flow field by filling struct called by function
 * similar to get_ff2d_params in "dotmap.d"
 * 
 */

// New Commands
void conf_ff2d(FF2DStruct* pff2d,int i)
{
	int npoints;
	int dc_R;
	int dc_G;
	int dc_B;
	float degperframe = (float)speed/10/framerate;
	float area = 1.0;
	float rad = to_pixels((float)stimr/10);
	float den = (float)density/100; 
	float sx = to_pixels((float)stimx/10);
	float sy = to_pixels((float)stimy/10);
	
	if (stimlist[i].islinear == BLANK)
	{
		dc_R = 0;
		dc_G = 0;
		dc_B = 0;
	}
	else
	{
		dc_R = bar_R;
		dc_G = bar_G;
		dc_B = bar_B;
	}
	/*
	if (i == 0)
	{
		bar_R = 255;
		bar_G = 0;
		bar_B = 0;
	}
	else if (i == 1)
	{
		bar_R = 0;
		bar_G = 255;
		bar_B = 255;
	}
	else if (i == 2)
	{
		bar_R = 0;
		bar_G = 0;
		bar_B = 255;
	}
	else
	{
		bar_R = 255;
		bar_G = 255;
		bar_B = 255;
	}
	*/
		
	area = PI*stimr*stimr/100;
	npoints = (int)(den*area);
	pff2d->linear = stimlist[i].islinear;
	pff2d->npts = npoints;
	pff2d->prob = (float)coh/1000;
	pff2d->radius = rad;			
	pff2d->pseed = pseed;
	pff2d->cseed = cseed;		
	pff2d->pixsz = ptsiz;
	pff2d->depth = 20;			// TODO: NO HARDCODING
	pff2d->v = to_pixels(degperframe);
	//pff2d->dx = 0;
	//pff2d->dy = 0;
	pff2d->x = sx;
	pff2d->y = sy;
	pff2d->width = 0;			// TODO: NOT IMPLEMENTED
	pff2d->angle = stimlist[i].theta;
	pff2d->r = dc_R;
	pff2d->g = dc_G;
	pff2d->b = dc_B;
	
	//dprintf("stimr = %d %d\n",stimr,(int)(100*rad));
}

/***************************** conf_ff() *********************************
 *
 * OLD FUNCTION
 * replaced by conf_ff2d(), functionallity changed considerably swe 09.28.07
 * configures the dot fields for this trial using the seeds
 * retrieved by getseeds()
 * called by newtrial()
 * 
 *

int conf_ff(void)
{
   long npoints;  
   float rcm; 
   long rpix = (long)(stimz * SINE_1DEG * XRES/XDIM) * (stimr * 0.1) + 0.5;
   const float pi = acos(-1);


   rcm = (XDIM/XRES) * rpix;
   npoints = (long) ((density/100.0) * (pi * rcm * rcm) );

#if 0
   if (dflhandle > 0){
	 dflhandle = 0;
  	} 
   if (dfrhandle > 0){
	 dfrhandle = 0; 
	}
#endif
   vsend(MSG_CFG2DFFL, "lllllbbbb", 
	 npoints, 
	 coherence, 
	 rpix, 
	 pseed, 
	 cseed,
	 dotsize,
	 fg_grey,
	 fg_grey,
	 fg_grey);
   dflhandle = gethandle();
   	    
   vsend(MSG_CFG2DFFR, "lllllbbbb", 
	 npoints, 
	 coherence, 
	 rpix, 
	 pseed, 
	 cseed,
	 dotsize,
	 fg_grey,
	 fg_grey,
	 fg_grey);
   dfrhandle = gethandle();
   return 0; 

}
*/

/****************************** next_dotf() *******************************
 * 
 * requests next dotfield within trial
 * resets the duration and isi counters as well
 */ 

static int next_dotf(void)
{
  /* OLD COMMANDS:
   * calculate stimulus position in pixels 
   long xpos = xpix(stimx, stimz);  
   long ypos = ypix(stimy, stimz);*/

   /* OLD COMMANDS: get degrees per second (speed)*/
   //long dps = (long) (1000.0 *(stimz * speed/10.0) * SINE_1DEG * XRES/XDIM);
   long width = 0; /* width of directions * 100  */

   /* reset counters for duration and isi; 
      must happen for each stim
      divide by 2 because of the extra step necessary
      in the state list for the auxiliary 
      (fix_led control) state set to work  */
    durcnt = framerate*duration/1000;	/* number of frames for each stimulus */	
    isicnt = framerate*isi/1000;		/* number of frames for each isi */


   if (ltheta_ovr != NULLI && rtheta_ovr !=NULLI)
	rxerr("next_dotf(): WARNING, both overrides set, using rtheta");
      
   if (rtheta_ovr != NULLI || ltheta_ovr != NULLI)
	{
		if (rtheta_ovr != NULLI)
		{
			stimlist[OVR].theta = rtheta_ovr * 100;
			stimlist[OVR].islinear = 0;
			conf_ff2d(&f_ff2dStruct[OVR],OVR);
			render_update(f_ff2dHandle[OVR], &f_ff2dStruct[OVR], sizeof(FF2DStruct), HANDLE_ON);
			/* OLD COMMANDS
			vsend(MSG_SHOW, "bblllll", 
			dfrhandle, 
			0, 
			dps,
			(rtheta_ovr * 100),
			width, 
			xpos, 
			ypos);
	 		ffhandle = dfrhandle;
	 		*/
	 	}
		else if (ltheta_ovr != NULLI) 
	 	{
	 		stimlist[OVR].theta = ltheta_ovr * 100;
			stimlist[OVR].islinear = 1;
			conf_ff2d(&f_ff2dStruct[OVR],OVR);
			render_update(f_ff2dHandle[OVR], &f_ff2dStruct[OVR], sizeof(FF2DStruct), HANDLE_ON);
	 		/* OLD COMMANDS
	 		vsend(MSG_SHOW, "bblllll", 
		    dflhandle, 
		    0, 
		    dps, 
		    (ltheta_ovr * 100),
		    width, 
		    xpos, 
		    ypos);
			ffhandle = dflhandle;
			*/
	 	}
	}
	else
	{
		//dprintf("nextdot= %d\t%d\n",stimindex,f_ff2dHandle[stimindex]);
		conf_ff2d(&f_ff2dStruct[stimindex],stimindex);
		render_update(f_ff2dHandle[stimindex], &f_ff2dStruct[stimindex], sizeof(FF2DStruct), HANDLE_ON);
		
    	/* OLD COMMANDS
    	 if (sp->islinear)
		 {
		 	vsend(MSG_SHOW, "bblllll",
			dflhandle, 
			0, 
			dps, 
			(long)(sp->theta * 100), 
			width, 
			xpos, 
			ypos);
			ffhandle = dflhandle;
			
		}
		else
		{
			vsend(MSG_SHOW, "bblllll", 
			dfrhandle, 
			0, 
			dps, 
			(long)(sp->theta * 100),
			width, 
			xpos, 
			ypos);
	  		ffhandle = dfrhandle;
	  		
		}*/
     }
   
   /*turn on the stimulus on flag */
   dfon = 1;
   
   /*send the frame to synchronize*/
   render_frame(0);
   ecode(TTEST);
   
   return 0;
}

/****************************** next_stim() *******************************
 * 
 * figures out the next stimulus type in the sequence
 */ 

static int next_stim(void)
{
   int index, rindx,
   active = 0;
   stimindex = OVR;
   
   /* figure out number of active entries */
   for (sp = &stimlist[0]; sp->stat != END; sp++) 
     if (sp->stat == READY) active++;
   
   /* reset all if this "block" complete */
   if (!active)
     {
       for(sp = &stimlist[0]; sp->stat != END; sp++) 
	   sp->stat = READY;
       active = nstim;
     }
   
   /* get random number for this presentation */
   rindx = ivunif(0, active - 1); /* shouldn't be here if counter=nstim */
   
   /* now skip down into list by this amount */
   sp = stimlist;
   while (sp->stat != READY) sp++; /* start pointing at the first live one */
   while (1)
   {
      while (sp->stat == DONE) sp++;
      if (rindx <= 0) break;
      sp++;
      rindx--;
   } 
   
/* now do the calculations necessary to provice a unique ECODE
 * to identify this stimulus. The method here is exactly as used
 * in the original dot tuning paradigm. The index number is simply 
 * the number of the stimulus in the stimulus array, which is zero-
 * based.  That index is then added to 3500 for spiral space stimuli
 * and to 3000 for linear stimuli. 
 */ 
   index = (sp - &stimlist[0]);
   stimindex = sp->ind;

   if (sp->islinear) 
     {
       return 3000 + index;
     }
 
   else 
     {	
       return 3500 + index;
     }
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

/******************************wincheck() *******************************
 *
 * checks if fixation window on (window_on = 1) for LED control
 */

int wincheck()
{
	window_on = 1;
	return 0;
}


/***************************** alloff() **********************************
 * 
 * closes up shop
 */
static int alloff(void)
{
  wd_cntrl(WIND0, WD_OFF);  /*this turns off the fixation window */
  fpoff();  /*this turns off the fixation point */
  window_on = 0;
  stimoff(0); /*this turns off any stimuli on without marking them done */
  /*send(MSG_FREE); clear all configured handles*/	
  alldone = 0;
  return 0;
}


/**************************** trlcount() *********************************
 *
 * scores the trial correct or incorrect; counts down number of trials
 */
static int trlcount(int flag)
{
  score(flag);  /* if TRUE, REX wants to know about the reward */
  if (flag) remain --; /* if TRUE, the trial's complete so count it */
  return 0;
}


/***************************** killslave() ********************************
 *
 * sends MSG_DONE on nonzero argument
 *
static int killslave(int flag)
{
   if (flag) send(MSG_DONE);
   return(flag);
}
*/

/************************************************************************
 * Ecode-returning functions for data recording. 
 * These are called by states in the state set. The order in which these
 * are called is the key to interpreting the header info.
 * Each ECODE here is defined as HEADBASE (which is defined as 1500)
 * + the value of the variable.  
 ************************************************************************/
 
int fpxcd(void) {return(HEADBASE + fixx);}
int fpycd(void) {return(HEADBASE + fixy);}
int stxcd(void) {return(HEADBASE + stimx);}
int stycd(void) {return(HEADBASE + stimy);}
int strcd(void) {return(HEADBASE + stimr);}
int stzcd(void) {return(HEADBASE + stimz);}
int rtmincd(void) {return(HEADBASE + rtheta_min);}
int rtmaxcd(void) {return(HEADBASE + rtheta_max);}
int rtnumcd(void) {return(HEADBASE + rtheta_num);}
int ltmincd(void) {return(HEADBASE + ltheta_min);}
int ltmaxcd(void) {return(HEADBASE + ltheta_max);}
int ltnumcd(void) {return(HEADBASE + ltheta_num);}
int spdcd(void) {return(HEADBASE + speed);}
int nptcd(void) {return(HEADBASE + density);}
int ptszcd(void) {return(HEADBASE + dotsize);}
int cohcd(void) {return(HEADBASE + coh);}
int fgcd(void) {return(HEADBASE + fg_grey);}
int bgcd(void) {return(HEADBASE + bg_grey);}
int numcd(void) {return(HEADBASE + stim_trial);}
int durcd(void) {return(HEADBASE + duration);}
int isicd(void) {return(HEADBASE + isi);}


/*************************************************************************
 * The next are used on the fly.
 *************************************************************************/

static int ovrcd(void)
{
  /* if either over-ride is set, drop an ECODE to note that */
  if (rtheta_ovr != NULLI || ltheta_ovr != NULLI) 
    return OVRDCD;

  else return 0; /*if neither over-ride is set, don't need an ECODE */
}

/*************************** pr_info *************************************
 * 
 * prints book-keeping information
 */
static void pr_info(void)
{
   int i;
   //OLD BUFFER COMMANDS REMOVED by swe 10.01.07
   //extern char outbuf[];
   //setbuf(stdout, &outbuf);
   
   /* print out the header information: here
    * index #, linear or no, dir, status, and number done
    */ 
   printf("index\tlin?\ttheta\tstat\tcount\thandle\n");

   /* for each stimulus, loop through and print out the
    * above information 
    */
   for(i = 0; stimlist[i].stat != END; i++) 
     {
       printf("%2d\t%1d\t%3d\t%1d\t%2d\t%3d\n", 
	      stimlist[i].ind,
	      stimlist[i].islinear,
	      stimlist[i].theta, 
	      stimlist[i].stat, 
	      stimlist[i].ngood,
	      stimlist[i].handle);
     }
   printf("\n");
   
   /* calculate the number of trials done, and then print out
    * number of trials done followed by number of errors */
   printf("%d trials, %d errors\n", (nstim * nreps)/stim_trial - remain, errcnt);

   /* clear out the buffers */
   //fflush(stdout);
   //setbuf(stdout, 0);
}

/**************************** n_exlist() **********************************
 *
 * enables 't e' requests to print bookeeping info
 *
void n_exlist(char *vstr)
{
   switch(*vstr)
     {
     case 't':	 type a list 
	  pr_info();
	  break;
     default:
       badverb();  this is REX error message call 
       break;
     }
}*/

/************************************************************************
 * Ecode-returning functions for data recording. The order in which these
 * are called is the key to interpreting the header info.
 ************************************************************************/

/* the following constructs a secondary statelist which
 * is called as a submenu from the st menu; the items 
 * included in this submenu pertain to stimulus attributes
 * that are somewhat less likely to be adjusted under normal
 * running conditions. Note that the first three parameters
 * here (stimulus duration, isi, and number of stimuli per
 * trial) are adjustable and together control the duration
 * of a single trial.
 */
 
VLIST stim_vl[] = {
	"duration",	&duration, NP, NP, 0, ME_DEC,
    "isi", 		&isi, NP, NP, 0, ME_DEC,
	"stim/trial",	&stim_trial, NP, NP, 0, ME_DEC,
	"coherence",	&coh, NP, NP, 0, ME_DEC,
	"density",	&density, NP, NP, 0, ME_DEC,
	"stim_R", &bar_R, NP, NP, 0, ME_DEC,
	"stim_G", &bar_G, NP, NP, 0, ME_DEC,
	"stim_B", &bar_B, NP, NP, 0, ME_DEC,
	NS
};

/* the following function takes the secondary statelist as
 * defined above and returns a menu; this allows it to be
 * included in your primary statelist (the st menu) as 
 * a submenu. Note that VLIST and MENU are return types 
 * specified deep in the bowels of REX. */

MENU stim_me = 
	{
	 "stim_params", &stim_vl, NP, NP, 0, NP, NS,
	};

char hm_stim[] = "";

/* now the primary statelist, as usually defined. For the
 * st menu to work, this MUST be called state_vl[]. In 
 * this particular statelist (for this paradigm!) the
 * inclusion of the stimulus parameter submenu occurs as
 * the last item of the statelist; look there to see
 * how to include a submenu. The order is not essential;
 * you may include multiple submenus within a statelist,
 * and they don't have to be the last items. 
 */

VLIST state_vl[] = {
   "day_seed",		&seed, NP, NP, 0, ME_DEC,
   "fp_x",		&fixx,	NP, NP,	0, ME_DEC,
   "fp_y",		&fixy,	NP, NP,	0, ME_DEC,
   "fp_size",	&fpsiz, NP, NP, 0, ME_DEC,
   "stim_x",		&stimx,	NP, NP,	0, ME_DEC,
   "stim_y",		&stimy,	NP, NP,	0, ME_DEC,
   "stim_r",		&stimr,	NP, NP,	0, ME_DEC,
   "point_size",	&ptsiz, NP, NP, 0, ME_DEC,
   "CRT_dist(cm)",	&stimz, NP, NP, 0, ME_DEC,
   "rtheta_min",	&rtheta_min, NP, NP, 0, ME_DEC,
   "rtheta_max",	&rtheta_max, NP, NP, 0, ME_DEC,
   "rtheta_num",	&rtheta_num, NP, NP, 0, ME_DEC,
   "rtheta_override",	&rtheta_ovr, NP, NP, 0, ME_NVALD,
   "ltheta_min",	&ltheta_min, NP, NP, 0, ME_DEC,
   "ltheta_max",	&ltheta_max, NP, NP, 0, ME_DEC,
   "ltheta_num",	&ltheta_num, NP, NP, 0, ME_DEC,
   "ltheta_override",	&ltheta_ovr, NP, NP, 0, ME_NVALD,
   "speed",	        &speed, NP, NP, 0, ME_DEC,
   "trials",		&nreps, NP, NP, 0, ME_DEC,
   "fix_led",		&fix_led, NP, NP, 0, ME_DEC,
   NS,
};

/*
 * Help message.
 */
char hm_sv_vl[] = "";


MENU umenus[] = {
{"state_vars", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{"separator", NP}, 
{"stim_params", &stim_vl, NP, NP, 0, NP, hm_stim},
{NS},
};







/*
 * User-supplied noun table.
 */
NOUN unouns[] = {
"",
};


/*
 * User-supplied function table.
 */
USER_FUNC ufuncs[] = {
	{"print_info",	&pr_info, "%d"},
	{""},
};



%%
id 121  /*unique paradigm identifier*/
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
		rl 20
		to pause2 on +PSTOP & softswitch
		to setbg on -PSTOP & softswitch
	pause2:
		code PAUSECD
		rl 30
		to setbg on -PSTOP & softswitch	
	setbg:
		do setbg()
		rl 50
		to setup
	setup:
		time 1000
		do initial()
		to myconfall
	myconfall:
		do my_conf_all()
		to headcd on 1 % swe_check_for_handles 
	headcd:
		code HEADCD
		to fpxcd
	fpxcd:
		do fpxcd()
		to fpycd
	fpycd:
		do fpycd()
		to stxcd
	stxcd:
		do stxcd()
		to stycd
	stycd:
		do stycd()
		to strcd
	strcd:
		do strcd()
		to stzcd
	stzcd:
		do stzcd()
		to rtmincd
	rtmincd:
		do rtmincd()
		to rtmaxcd
	rtmaxcd:
		do rtmaxcd()
		to rtnumcd
	rtnumcd:
		do rtnumcd()
		to ltmincd
	ltmincd:
		do ltmincd()
		to ltmaxcd
	ltmaxcd:
		do ltmaxcd()
		to ltnumcd
	ltnumcd:
		do ltnumcd()
		to spdcd
	spdcd:
		do spdcd()
		to nptcd
	nptcd:
		do nptcd()
		to ptszcd
	ptszcd:
		do ptszcd()
		to cohcd
	cohcd:
		do cohcd()
		to fgcd
	fgcd:
		do fgcd()
		to bgcd
	bgcd:
		do bgcd()
		to numcd
	numcd: 
		do numcd()
        to durcd
	durcd:
		do durcd()
		to isicd
	isicd:
		do isicd() 
		to loop
	loop:
		time 10
		rl 70
		to pause3
	pause3:
		to pause4 on +PSTOP & softswitch
		to next on -PSTOP & softswitch
	pause4:
		code PAUSECD
		to next on -PSTOP & softswitch
	next:
		do newtrial()
		time 10
		to ovrcd
	ovrcd:
		do ovrcd()
		to iti
	iti:
		time 1500	/* rest of the ISI */
		to fpon
	fpon:
		code FIXASK
		do fpask()
		to winon on 1 % swe_check_for_went
	winon:
		code FPONCD
		time 20
		do winon(20,20)
		to grace
	grace:
		time 3000
		to fixtim on -WD0_XY & eyeflag
		to off
	off:
		do alloff()
		to loop
	fixtim:
		code FIXCD
		time 250
		rl 20
		do wincheck()     
		to noise on +WD0_XY & eyeflag /*don't punish for brkfix*/
		to trstart
	noise:
		do alloff()
		to loop
	trstart:
		code TRLSTART	
	    to stimloop
    stimloop:
		do next_stim() /* choose the next stimulus */
		to stim
	stim:
		do next_dotf() /* send the show messages */
		to check on 1 % swe_check_for_went
	check:
		to punish on +WD0_XY & eyeflag
		to went
	went:
		code STIMON
		rl 60
		to delay
	delay:
		to punish on +WD0_XY & eyeflag
		to stimdone on 0 ? durcnt
		to delay2
	delay2:
		do render_frame(0)
		to delay on 1 % swe_check_for_went
	stimdone:
		code STIMOFF
		do stimoff(1)
		rl 30
		to isi
	isi:
		to punish on +WD0_XY & eyeflag
		to stimcount on 0 ? isicnt
		to isi2
	isi2:
		do render_frame(0)
		to punish on +WD0_XY & eyeflag
		to isi on 1 % swe_check_for_went
	stimcount:
		to good on -TRUE & stimcnt
		to stimloop     
	good:
		code FPOFFCD
		do fpoff()
		rl 10
		to reward
	reward:
		code REWCD
		do dio_on(REW)
		time 50
		to rewoff
	rewoff:
		do dio_off(REW)
		to score
	score:
		do trlcount(1)
	    to trlend
	punish:
		code BREAKFIXCD
		do alloff()
		rl 10
		to beep
	beep:
		do dio_on(BEEP)	
		time 100
		to bpoff	
	bpoff:
		do dio_off(BEEP)
		to timout
	timout:
		do trlcount(0)
		time 1000
		to trlend
	/*qryerr:
		do qryerr()
		to trlend*/
	trlend:
		do alloff()
		to blkdone on -TRUE & remain
		to loop
	blkdone:
		rl 0
		to loop on +TRUE & remain
	exit:
		/* do killslave(0)*/
		to loop

abort	list:
		trlend blkdone exit
}






/*
%%
id 121  unique paradigm identifier
restart rinitf
main_set {
status ON
begin	first:
		code STARTCD
		rl 0
		to pause1
	pause1:
		to pause2 on +PSTOP & drinput
		to setbg on -PSTOP & drinput
	pause2:
		code PAUSECD
		to setbg on -PSTOP & drinput	
	setbg:
		do setbg()
		to setup
	setup:
		time 1000
		do initial()
		to headcd
	headcd:
		code HEADCD
		to fpxcd
	fpxcd:
		do fpxcd()
		to fpycd
	fpycd:
		do fpycd()
		to stxcd
	stxcd:
		do stxcd()
		to stycd
	stycd:
		do stycd()
		to strcd
	strcd:
		do strcd()
		to stzcd
	stzcd:
		do stzcd()
		to rtmincd
	rtmincd:
		do rtmincd()
		to rtmaxcd
	rtmaxcd:
		do rtmaxcd()
		to rtnumcd
	rtnumcd:
		do rtnumcd()
		to ltmincd
	ltmincd:
		do ltmincd()
		to ltmaxcd
	ltmaxcd:
		do ltmaxcd()
		to ltnumcd
	ltnumcd:
		do ltnumcd()
		to spdcd
	spdcd:
		do spdcd()
		to nptcd
	nptcd:
		do nptcd()
		to ptszcd
	ptszcd:
		do ptszcd()
		to cohcd
	cohcd:
		do cohcd()
		to fgcd
	fgcd:
		do fgcd()
		to bgcd
	bgcd:
		do bgcd()
		to numcd
	numcd: 
		do numcd()
        to durcd
	durcd:
		do durcd()
		to isicd
	isicd:
		do isicd() 
		to loop
	loop:
		time 10
		to pause3
	pause3:
		to pause4 on +PSTOP & drinput
		to next on -PSTOP & drinput
	pause4:
		code PAUSECD
		to next on -PSTOP & drinput
	next:
		do newtrial()
		time 10
		to ovrcd
	ovrcd:
		do ovrcd()
		to iti
	iti:
		time 1500	 rest of the ISI 
		to fpon
	fpon:
		code FIXASK
		do fpask()
		to winon
	winon:
		code FPONCD
		time 20
		do winon(10,10)
		to grace
	grace:
		time 3000
		to fixtim on -WD0_XY & eyeflag
		to off
	off:
		do alloff()
		to loop
	fixtim:
		time 250
		rl 20
		do wincheck()     
		to noise on +WD0_XY & eyeflag don't punish for brkfix
		to trstart
	noise:
		do alloff()
		to loop
	trstart:
		code TRLSTART	
	    to stimloop
    stimloop:
		do next_stim()  choose the next stimulus 
		to stim
	stim:
		do next_dotf()  send the show messages 
		to wait1
	wait1:
		to check on 1 % msg_pending
	check:
		chkstim()
		to punish on +WD0_XY & eyeflag
		to went
	went:
		code STIMON
		rl 60
		to delay
	delay:
		to punish on +WD0_XY & eyeflag
		to stimdone on 0 ? durcnt
		to delay2
	delay2:
		to delay avoids complete recursion
	stimdone:
		code STIMOFF
		do stimoff(1)
		rl 30
		to isi
	isi:
		to punish on +WD0_XY & eyeflag
		to stimcount on 0 ? isicnt
		to isi2
	isi2:
		to punish on +WD0_XY & eyeflag
		to isi avoids complete recursion
	stimcount:
		to good on -TRUE & stimcnt
		to stimloop     
	good:
		code FPOFFCD
		do fpoff()
		rl 10
		to reward
	reward:
		code REWCD
		do dio_on(REW)
		time 50
		to rewoff
	rewoff:
		do dio_off(REW)
		to score
	score:
		do trlcount(1)
	    to trlend
	punish:
		code BREAKFIXCD
		do alloff()
		rl 10
		to beep
	beep:
		do dio_on(BEEP)	
		time 100
		to bpoff	
	bpoff:
		do dio_off(BEEP)
		to timout
	timout:
		do trlcount(0)
		time 1000
		to qryerr
	qryerr:
		do qryerr()
		to trlend
	trlend:
		do alloff()
		to blkdone on -TRUE & remain
		to loop
	blkdone:
		rl 0
		to loop on +TRUE & remain
	exit:
		do killslave(0)
		to loop

abort	list:
		trlend blkdone exit
}
*/



/* now for the fixation LED control state set; this controls the
  fixation monitoring LED.  To turn on monitoring, select fix_led
  in the st menu and set it equal to 1. To turn it off, select
  fix_led and set it to 0 (false).  This turns on an LED whenever 
  the critter's fixating;  turns off the LED whenever the eye's not
  in the window. 

DISABLED 6/4/98 for debugging. KHB.

fix_set {
status ON
begin isledon:
		to led_off on 1 = fix_led
      led_on:
      		do dio_on(FIX_LED)  	
		to led_off on 0 = window_on
      led_off:
		do dio_off(FIX_LED)
		to led_on on 1 = window_on
}	
*/



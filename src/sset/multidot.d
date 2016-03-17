/* multidot.d, dotfield tuning paradigm.
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
#include <i86.h>
#include "ldev.h"	/* hardware device defines */
#include "lcode.h"	/* Ecodes */
#include "calib.h"  	/* SGI stimulus display parameters */
#include "render.h"	/* defines for graphics objects */
#include "locfunc.h"	/* local, commonly used functions */
#include "msg.h"	/* messaging functions */
#include "ivunif.c"	/* the random-number generator */

#define MAXLIST 75   /* maximum number of stimuli possible:
			this limits it to either 5 degree
			steps for a single type (linear or spiral
			space) or 10 degree steps if doing both */

/*
 * codes for the status flag in the stimulus array
 */
#define READY 0
#define DONE 1
#define END 2

#define PIXSIZE 3

#define FALSE 0
#define TRUE (~FALSE)

#define WIND0		0
#define EYEH_SIG	0
#define EYEV_SIG	1

/**********************************************************************
 * Global variables.
 **********************************************************************/

/* counters, flags, and seed declarations */
static int seedflg = 0;
static int nstim,                 /* number of stimuli configured */
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

/* set luminance and dot size */
static unsigned char dotsize = PIXSIZE, fg_grey, bg_grey;

/*now the stimulus structure:
  a stimulus is either linear or spiral-space;
  has a direction parameter theta;
  and has a count of number of times completed */

struct stim {
   int islinear;
   int theta;
   short stat;
   int ngood;
};

/* stimulus array */
static struct stim stimlist[MAXLIST];

/* pointer for moving through stimulus array */
static struct stim *sp = NULL;

/* fixation point and dotfield handles */
static unsigned char fphandle,
 	dflhandle, 
	dfrhandle,
	ffhandle;
static int fprh, ffrh;            /* realization handles */

extern Msg *msg;

/************************************************************************
 * Declaration of statelist variables.
 ************************************************************************/

static long seed = 1111,  /*day seed for random generator */
   rtheta_ovr = NULLI, /*spiral space direction over-ride: if both on, default*/
   ltheta_ovr = NULLI, /*linear direction over-ride */
   coherence = 750;

static int 	
   rtheta_min = 0,
   rtheta_max = 315,
   rtheta_num = 8,  /* default is 45 degree steps */
   ltheta_min = 0,
   ltheta_max = 315,
   ltheta_num = 8, /*default is 90 degree steps */
   speed = 200,
   fixx = 0,
   fixy = 0,
   stimx = 0,
   stimy = 0,
   stimr = 200,
   stimz = 28,
   stim_trial = 3, /*stimuli per trial */
   nreps = 5,	/* trials per condition */
   density = 30,	/* hundred dots per cm*cm */
   duration = 500, /*duration of stimulus in ms */
   isi = 450, /* duration of inter-stimulus interval */		
   remain;

/************************************************************************
 * 
 * Control functions to make things happen.
 *
 ************************************************************************/



/********************************* conf_fp() *****************************
 * 
 * configures the fixation point
 */
int conf_fp()
{
	vsend(MSG_CFGPT, "bbbbb", CIRCLE, 8, 255, bg_grey, bg_grey);
	return 0;
}

/***************************** get_fphandle() *****************************
 * 
 * gets the handle for the fixation point
 */
int get_fphandle(void)
{
	if ((fphandle=gethandle())==-1)
	{
		rxerr("get_fphandle(): invalid message: expecting handle");
		return ERRORCD;
	}
	return 0;
}

/****************************** fpask() *******************************
 *
 * requests that a FP be displayed
 */
int fpask(void)
{
	vsend(MSG_SHOW, "bbllll", fphandle, 0,
		xpix(fixx, stimz),
		ypix(fixy, stimz),
		0,0);
	fprh = gethandle();
	send(MSG_FRAME);
	fpon = 1;	

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
      vsend(MSG_UNSHOW, "bb", fphandle, fprh);
      send(MSG_FRAME);
      getmsg();
      fpon = 0;
   }

   return 0;
}



/***************************** stimoff() *********************************
 *
 * turns off the fixation point and the dot fields if they're on.
 */
static int stimoff(int flag)
{
  if (dfon)
    {
      vsend(MSG_UNSHOW, "bb", ffhandle, ffrh);
      send(MSG_FRAME);
      getmsg();
      dfon = 0;
    }

  if (flag)
    {
      sp->stat = DONE;
      sp->ngood++;
      stimcnt --;
    }
	 
  return 0;
}


/********************************* setbg() *******************************
 *
 * sets the luminance of both foreground and background
 */
int setbg(long dot, long bg)
{
  fg_grey = (unsigned char)dot;
  bg_grey = (unsigned char)bg;
  vsend(MSG_SETBG, "bbb", bg_grey, bg_grey, bg_grey);
  return(0);
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
	
  /* initialize counters */
   errcnt = 0;
   gtrcnt = 0;
   stimcnt = stim_trial;
   
   /* free up any old handles lying around */
   send(MSG_FREE);
   
   /*initialize random generator with today's seed*/
   if (!seedflg)
     {
       ivunif(seed, 0);
       seedflg++;
     }
   
   /* Generate the running array of stimuli. */

      if ((nstim = rtheta_num + ltheta_num) >= MAXLIST)
	{
	  rxerr("initial(): too many stimuli in array");
       	  return 0;
	}

      for (i = 0, theta = rtheta_min; i < rtheta_num; i++, ndx++)
	{
	  stimlist[ndx].islinear = 0;
	  stimlist[ndx].theta = theta;
	  stimlist[ndx].stat  = READY;
	  stimlist[ndx].ngood = 0;
	  theta += (rtheta_max - rtheta_min)/(rtheta_num - 1);
	}
		
      for (i = 0, theta = ltheta_min; i < ltheta_num; i++, ndx++)
	{
	  stimlist[ndx].islinear = 1;
	  stimlist[ndx].theta = theta;
	  stimlist[ndx].stat = READY;
	  stimlist[ndx].ngood = 0;
	  theta += (ltheta_max - ltheta_min)/(ltheta_num - 1);
	}
		
      stimlist[ndx].stat = END;

   /* set up number of trials necessary to complete 
    * repetitions of all stimuli given the specified number of stimuli
    * per trial as given in the st menu. If the number of stimuli
    * is not evenly divisible by the number of stimuli per trial,
    * an extra trial is required, and some stimuli may be presented
    * an extra time to complete that trial */

   remain = (nreps * nstim) / stimcnt;
   if ((nstim % stimcnt) != 0) remain++;

   /* set up fixation point
    */
   conf_fp();
   get_fphandle();
   
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
  conf_fp();  /*fixation point configuration because handle freed*/
  get_fphandle(); /*handle from fp configuration */

  getseeds(); /*get pseed and cseed for this trial */
  conf_ff(); /*use this trial's seeds and configure both dot fields*/

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
   
   return 0;
}

/***************************** conf_ff() *********************************
 *
 * configures the dot fields for this trial using the seeds
 * retrieved by getseeds()
 * called by newtrial()
 */

int conf_ff(void)
{
   long npoints;  /*number of dots in the stimulus */
   float rcm; /* radii: rcm = cm; rpix in pixels */
   long rpix = (long)(stimz * SINE_1DEG * XRES/XDIM) * (stimr * 0.1) + 0.5;
   const float pi = acos(-1);


   rcm = (XDIM/XRES) * rpix;
   npoints = (long) ((density/100.0) * (pi * rcm * rcm) );

#if 0
   if (dflhandle > 0){
	 dflhandle = 0; /* reset linear handle */
  	} 
   if (dfrhandle > 0){
	 dfrhandle = 0; /* reset spiral space handle*/ 
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
   dflhandle = gethandle();  /*handle for linear field */
   	    
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
   dfrhandle = gethandle(); /*handle for spiral space field */

   return 0; 

}

/****************************** next_dotf() *******************************
 * 
 * requests next dotfield within trial
 * resets the duration and isi counters as well
 */ 

static int next_dotf(void)
{
  /*calculate stimulus position in pixels */
   long xpos = xpix(stimx, stimz);  
   long ypos = ypix(stimy, stimz);

   /*get degrees per second (speed)*/
   long dps = (long) (1000.0 *(stimz * speed/10.0) * SINE_1DEG * XRES/XDIM);
   long width = 0; /* width of directions * 100  */

   /* reset counters for duration and isi; 
      must happen for each stim
      divide by 2 because of the extra step necessary
      in the state list for the auxiliary 
      (fix_led control) state set to work  */
    durcnt = duration/2;
    isicnt = isi/2;

   if (ltheta_ovr != NULLI && rtheta_ovr !=NULLI)
      rxerr("next_dotf(): WARNING, both overrides set, using rtheta");
      
   if (rtheta_ovr != NULLI || ltheta_ovr != NULLI)
	{
	  if (rtheta_ovr != NULLI)
		{
		  vsend(MSG_SHOW, "bblllll", 
			dfrhandle, 
			0, 
			dps,
			(rtheta_ovr * 100),
			width, 
			xpos, 
			ypos);
	 	  ffhandle = dfrhandle;
	 	}
	  else if (ltheta_ovr != NULLI) 
	 	{
	 	 vsend(MSG_SHOW, "bblllll", 
		       dflhandle, 
		       0, 
		       dps, 
		       (ltheta_ovr * 100),
		       width, 
		       xpos, 
		       ypos);
		 ffhandle = dflhandle;
	 	}
	}
   else
     {
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
	}
     }
   
   /*turn on the stimulus on flag */
   dfon = 1;
   
   /*get the realization handle*/
   ffrh = gethandle();
   
   /*send the frame to synchronize*/
   send(MSG_FRAME);
   
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
  send(MSG_FREE); /*clear all configured handles*/	
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
 */
static int killslave(int flag)
{
   if (flag) send(MSG_DONE);
   return(flag);
}

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
int cohcd(void) {return(HEADBASE + coherence);}
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
   extern char outbuf[];
   setbuf(stdout, &outbuf);
   
   /* print out the header information: here
    * index #, linear or no, dir, status, and number done
    */ 
   printf("index\tlin?\ttheta\tstat\tcount\n");

   /* for each stimulus, loop through and print out the
    * above information 
    */
   for(i = 0; i < nstim; i++) 
     {
       printf("%2d\t%1d\t%3d\t%1d\t%2d\n", 
	      i, 
	      stimlist[i].islinear,
	      stimlist[i].theta, 
	      stimlist[i].stat, 
	      stimlist[i].ngood);
     }
   printf("\n");
   
   /* calculate the number of trials done, and then print out
    * number of trials done followed by number of errors */
   printf("%d trials, %d errors\n", (nstim * nreps)/stim_trial - remain, errcnt);

   /* clear out the buffers */
   fflush(stdout);
   setbuf(stdout, 0);
}

/**************************** n_exlist() **********************************
 *
 * enables 't e' requests to print bookeeping info
 */
void n_exlist(char *vstr)
{
   switch(*vstr)
     {
     case 't':	/* type a list */
	  pr_info();
	  break;
     default:
       badverb(); /* this is REX error message call */
       break;
     }
}

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
	"coherence",	&coherence, NP, NP, 0, ME_DEC,
	"density",	&density, NP, NP, 0, ME_DEC,
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
   "stim_x",		&stimx,	NP, NP,	0, ME_DEC,
   "stim_y",		&stimy,	NP, NP,	0, ME_DEC,
   "stim_r",		&stimr,	NP, NP,	0, ME_DEC,
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
   "stim_params",  	&stim_me, NP, NP, 0, ME_SUBMENU,
   NS,
};

/*
 * Help message.
 */
char hm_sv_vl[] = "";

/*
 * User-supplied noun table.
 */
NOUN unouns[] = {
"exlist",       &n_exlist,
"",
};

%%
id 121  /*unique paradigm identifier*/
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
		do setbg(255, 0)
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
		time 1500	/* rest of the ISI */
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
		to delay /*avoids complete recursion*/
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
		to isi /*avoids complete recursion*/
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



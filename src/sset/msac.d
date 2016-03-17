/* $Id: msac.d,v 1.5 2015/05/12 20:52:48 devel Exp $ */

#include <math.h>
#include "bcode.h"
#include "ldev.h"	/* hardware device defines */
#include "lcode.h"	/* Ecodes */
#include "lcalib.h"	/* local (per-rig) screen calibration */

#include "make_msg.h"			/* funcs to form command messages */
#include "transport.h"			/* funcs to send command messages to render */
#include "actions.h"			/* higher-level actions for render */
#include "pixels.h"				/* pixel conversion funcs */
#include "ivunif.h"				/* random number routines */
#include "randomtrialgenerator.h"

/*
 * ECODE values for this paradigm
 */
 
#define WENTCD		1509
#define MISSEDCD	1505	// missed frame code
#define TRIALCD		1502 

/* 
 * bit masks for verbosity 
 */
 
#define DEBUG_BIT 0x1
#define DEBUG_REWARD_BIT 0x2
#define DEBUG_EYE_WINDOW_BIT 0x4
#define DEBUG_ANSWER_POINTS_BIT 0x8

/* 
 * Convenience args for my_trial_done
 */
 
#define TRIAL_DONE_NOFIX 0
#define TRIAL_DONE_CORRECT 1
#define TRIAL_DONE_NOANSWER -1
#define TRIAL_DONE_BREAKFIX -2
#define TRIAL_DONE_PAUSE -3

/* 
 * These are values which can be passed to my_eye_window (they may be OR'd together).  
 */

#define EYE_WINDOW_INIT				0x1
#define EYE_WINDOW_ON				0x2
#define EYE_WINDOW_OFF				0x4
#define EYE_WINDOW_MOVE				0x8

/*
 * This is the eye window identifier
 */
 
#define EYE_WINDOW_ID	0


DotStruct f_fixpt;				/* fixation point graphic object */
int f_fixpt_handle = 0;			/* handle for fixpt */
DotStruct f_target;				/* target point graphic object */
int f_target_handle = 0;		/* handle for target */
int f_handle_count = 0;			/* used to count handles by my_check_for_handles */
int f_seed_used = 0;			/* set to 1 when random seed is applied */
float *f_ptarget_x;	
float *f_ptarget_y;
int f_trial_index;				/* index of target position for current trial */
RTGenerator *f_prtgen = NULL;
int f_all_done = 0;				/* Set to 1 by my_trial_init to indicate all trials have been completed. */
int f_never = 0;

/*
 * state_vl menu
 */

int f_seed = 9999;						/* random number seed */
int f_ntrials = 4;						/* number of trials for each condition */
int f_nblocksize = 2;					/* block size that trials are gathered in */
int f_screen_distance_MM = 300;			/* screen distance in mm */
int f_ntargets = 8;						/* Number of target positions */
int f_verbose = 0;						/* debugging flag */
int f_reward_preset = 15;				/* reward size preset value */
int f_reward_random = 10;				/* reward size random value */
#define ANGLE_RAD(i) (i * 2.0f * M_PI / (float)f_ntargets)
#define ANGLE_DEG(i) (i * 360.0f / (float)f_ntargets)


VLIST state_vl[] = {
"day_seed",		&f_seed, NP, NP, 0, ME_DEC,
"#_of_trials/condition", &f_ntrials, NP, NP, 0, ME_DEC,
"block_size", &f_nblocksize, NP, NP, 0, ME_DEC,
"screen_dist(mm)", &f_screen_distance_MM, NP, NP, 0, ME_DEC,
"#_target_positions", &f_ntargets, NP, NP, 0, ME_DEC,
"reward_preset", &f_reward_preset, NP, NP, 0, ME_DEC,
"reward_random", &f_reward_random, NP, NP, 0, ME_DEC,
"verbose", &f_verbose, NP, NP, 0, ME_DEC,
NS,
};

char hm_sv_vl[] = "";


/* 
 * Background color menu  
 */

int f_background_color[3] = {0};		/* background color [0,255] */

VLIST background_vl[] = {
"Background_color(R)", &f_background_color[0], 0, NP, 0, ME_DEC,
"Background_color(G)", &f_background_color[1], 0, NP, 0, ME_DEC,
"Background_color(B)", &f_background_color[2], 0, NP, 0, ME_DEC,
NS,
};

char hm_background[] = "";



/*
 * Fixation point menu
 */
 
float f_fixpt_diameter = .25;			/* diameter of fixpt (degrees) */
float f_fixpt_window = 2;				/* size of fixation point window */
int f_fixpt_color[3] = {255, 0, 0};		/* fixpt color [0,255] */
float f_fixpt_x = 0;					/* fixpt x position */
float f_fixpt_y = 0;					/* fixpt y position */

VLIST fixpt_vl[] = {
"fixpt_x(deg)", &f_fixpt_x, NP, NP, 0, ME_FLOAT,
"fixpt_y(deg)", &f_fixpt_y, NP, NP, 0, ME_FLOAT,
"fixpt_diameter(deg)", &f_fixpt_diameter, NP, NP, 0, ME_FLOAT,
"fixpt_window(deg)", &f_fixpt_window, NP, NP, 0, ME_FLOAT,
"fixpt_color(R)", &f_fixpt_color[0], 0, NP, 0, ME_DEC,
"fixpt_color(G)", &f_fixpt_color[1], 0, NP, 0, ME_DEC,
"fixpt_color(B)", &f_fixpt_color[2], 0, NP, 0, ME_DEC,
NS,
};

char hm_fixpt[] = "";

/*
 * Targets menu
 */
 
float f_target_offset = 8;				/* target offset from fixpt, degrees */
float f_target_diameter = 0.25;			/* diameter of fixpt (degrees) */
float f_target_window = 2;				/* size of target point window */
int f_target_color[3] = {255, 0, 0};		/* target color [0,255] */

VLIST target_vl[] = {
"target_offset(deg)", &f_target_offset, NP, NP, 0, ME_FLOAT,
"target_diameter(deg)", &f_target_diameter, NP, NP, 0, ME_FLOAT,
"target_window(deg)", &f_target_window, NP, NP, 0, ME_FLOAT,
"target_color(R)", &f_target_color[0], 0, NP, 0, ME_DEC,
"target_color(G)", &f_target_color[1], 0, NP, 0, ME_DEC,
"target_color(B)", &f_target_color[2], 0, NP, 0, ME_DEC,
NS,
};

char hm_target[] = "";


/* 
 * Timing menu - all values in milliseconds
 */

int f_trial_init_pause_time = 500;	/* pause after trial init, prior to fixpt on */
int f_fixpt_acq_time = 2000;		/* max time to acquire fixpt */
int f_fixpt_acq_fail_pause_time = 1000;	/* if acq fails, wait this long before restarting */
int f_fixpt_hold_time = 500;		/* how long until fixation */
int f_fixpt_acq_noise_pause_time = 250;	/* if acq fails due to noise, wait this long before restarting */
int f_target_fixpt_overlap_time = 125;	/* time target and fixpt on screen together */
int f_target_off_wait_time = 800;	/* after target off, maintain fix for this time */
int f_target_wait_for_answer_time = 1500;	/* max time to saccade to answer */
int f_target_fix_time = 200;		/* time to fixate on target window for answer */
int f_intertrial_pause_time = 250;		/* time between trials */

VLIST timing_vl[] = {
"trial_init_pause", &f_trial_init_pause_time, NP, NP, 0, ME_DEC,
"acq_time(ms)", &f_fixpt_acq_time, NP, NP, 0, ME_DEC,
"acq_timeout(ms)", &f_fixpt_acq_fail_pause_time, NP, NP, 0, ME_DEC,
"fix_time(ms)", &f_fixpt_hold_time, NP, NP, 0, ME_DEC,
"noise_timeout(ms)", &f_fixpt_acq_noise_pause_time, NP, NP, 0, ME_DEC,
"targ/fp_overlap(ms)", &f_target_fixpt_overlap_time, NP, NP, 0, ME_DEC,
"targoff_wait_time(ms)", &f_target_off_wait_time, NP, NP, 0, ME_DEC,
"answer_time(ms)", &f_target_wait_for_answer_time, NP, NP, 0, ME_DEC,
"targ_fix_time(ms)", &f_target_fix_time, NP, NP, 0, ME_DEC,
"intertrial_time(ms)", &f_intertrial_pause_time, NP, NP, 0, ME_DEC,
NS,
};

char hm_timing[] = "";

int f_frames_per_second = 85;			/* frame rate for render */
char f_local_addr[32]="192.168.1.1";	/* ip address of local machine */
char f_remote_addr[32]="192.168.1.2";	/* ip address of render machine */
int f_remote_port=2000;					/* port to use on render machine */


VLIST comm_vl[] = {
"frame_rate(1/s)", &f_frames_per_second, NP, NP, 0, ME_DEC,
"local_ip", f_local_addr, NP, NP, 0, ME_STR,
"render_host_ip", f_remote_addr, NP, NP, 0, ME_STR,
"render_port", &f_remote_port, NP, NP, 0, ME_DEC,
NS,
};

char hm_comm[] = "";
 

MENU umenus[] = {
{"Main", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{"timing", &timing_vl, NP, NP, 0, NP, hm_timing}, 
{"fixation", &fixpt_vl, NP, NP, 0, NP, hm_fixpt}, 
{"target", &target_vl, NP, NP, 0, NP, hm_target}, 
{"background", &background_vl, NP, NP, 0, NP, hm_background}, 
{"communication", &comm_vl, NP, NP, 0, NP, hm_comm}, 
{NS},
};



/*
 * Channel numbers for bcodes saved in efile. 
 */

#define CH_TARGET_X 1
#define CH_TARGET_Y 2
#define CH_TRIALINDEX 3
 
#define CH_SEED 101
#define CH_NTRIALS 102
#define CH_NBLOCKSIZE 103
#define CH_SCREENDIST 104
#define CH_NTARGETS 105
#define CH_FIXPT_X 106
#define CH_FIXPT_Y 107
#define CH_FIXPT_DIAMETER 108
#define CH_FIXPT_WINDOW 109
#define CH_TARGET_OFFSET 110
#define CH_TARGET_DIAMETER 111
#define CH_TARGET_WINDOW 112


/***************************** local_init *****************************
 *                                                                       *
 * This function is the "reset" function in the state set. In other words, 
 * the state set specification contains the line:
 * 
 * restart local_init
 * 
 * That line makes this function special, in that it will be called at the 
 * following times:
 * 
 * - the first time the clock is started
 * - whenever reset state happens
 *                                                                       *
 ************************************************************************/
void local_init(void)
{
	int status=0;
	
	dprintf("Run local initializations...\n");

	// initialize tcpip - must only do this OR gpib - NOT BOTH!!!!
	dprintf("Look for render host at %s:%d\n", f_remote_addr, f_remote_port);
	status = init_tcpip(f_local_addr, f_remote_addr, f_remote_port, 0);

	// initialize pixel conversion
	dprintf("Screen dimensions (see lcalib.h): %dx%d (mm)\n", (int)x_dimension_mm, (int)y_dimension_mm);
	dprintf("Screen resolution (see lcalib.h): %dx%d (pixels)\n", (int)x_resolution, (int)y_resolution);
	dprintf("Screen distance (see lcalib.h): %d (mm)\n", (int)f_screen_distance_MM);
	if (initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, x_resolution, y_resolution, f_screen_distance_MM))
	{
		dprintf("ERROR in initialize_pixel_conversion: \n"
				"x,y dimensions=(%d, %d)\n"
				"x,y resolution=(%d, %d)\n"
				"screen distance=%d\n", (int)x_dimension_mm, (int)y_dimension_mm, (int)x_resolution, (int)y_resolution, (int)f_screen_distance_MM);
	}
	
}


/***************************** my_render_init ****************************
 *                                                                       *
 * This function is called after render has been reset, but before any
 * trials have begun. 
 * 
 ************************************************************************/

int my_render_init()
{
	int status=0;
	float fovy;

	dprintf("Initializing render\n");

	/*
	 *  seed random number generator if necessary
	 */

	if (!f_seed_used)
	{
		ivunif(f_seed, 1);
		f_seed_used = 1;
	}
	
	/*
	 * zero out handles and initialize counters.
	 */

	f_fixpt_handle = f_target_handle = 0;
	f_handle_count = 0;
	f_all_done = 0;

	/* 
	 * background color
	 */

	render_bgcolor(f_background_color[0], f_background_color[1], f_background_color[2]);

	/* 
	 * Configure fixation point 
	 */
	
	f_fixpt.xorigin = 0;
	f_fixpt.yorigin = 0;
	f_fixpt.xsize = to_pixels(f_fixpt_diameter);
	f_fixpt.ysize = to_pixels(f_fixpt_diameter);
	f_fixpt.depth = 1;
	f_fixpt.r = f_fixpt_color[0];
	f_fixpt.g = f_fixpt_color[1];
	f_fixpt.b = f_fixpt_color[2];
	render_dot(&f_fixpt);
	
	/* 
	 * Configure target point 
	 */
	
	f_target.xorigin = 0;
	f_target.yorigin = 0;
	f_target.xsize = to_pixels(f_target_diameter);
	f_target.ysize = to_pixels(f_target_diameter);
	f_target.depth = 1;
	f_target.r = f_target_color[0];
	f_target.g = f_target_color[1];
	f_target.b = f_target_color[2];
	render_dot(&f_target);

	return status;
}


/* 
 * my_check_for_handle 
 * 
 * Called as an escape function ... e.g. 
 * 
 * to wherever on 1 % my_check_for_handle
 * 
 * This function will fetch 2 handles from render. They are saved
 * as f_fixpt_handle and f_target_handle respectively. 
 * The counter f_handle_count must be zeroed before this is used. 
 * Since the allocation of graphic objects takes place early in the 
 * state set (render_init), the handle counter should get zeroed 
 * in the my_render_init() function. 
 *
 * On each invocation it checks once for a handle.If a handle is found 
 * it is assigned to the proper variable and the handle counter is 
 * incremented.  
 * 
 * Returns 0 if the handle counter is less than 2, or 1 if the handle 
 * counter is 2.
 */

int my_check_for_handle()
{
	int status = 0;
	int handle = 0;
	if (render_check_for_handle(&handle))
	{
		if (f_handle_count == 0)
		{
			f_fixpt_handle = handle;
			f_handle_count = 1;
			if (f_verbose & DEBUG_BIT)
			{
				dprintf("my_check_for_handle(): fixpt handle = %d\n", f_fixpt_handle);
			}
		}
		else if (f_handle_count == 1)
		{
			f_target_handle = handle;
			f_handle_count = 2;
			status = 1;
			if (f_verbose & DEBUG_BIT)
			{
				dprintf("my_check_for_handle(): target handle = %d\n", f_target_handle);
			}
		}
	}
	return status;
}


/* 
 * my_exp_init()
 * 
 * Initializations for overall experiment.
 * 
 */
 
int my_exp_init()
{
	int i;
	
	/*
	 * Init counters etc. 
	 */
	
	if (f_ptarget_x)
	{
		free(f_ptarget_x);
		f_ptarget_x = NULL;
	}
	if (f_ptarget_y)
	{
		free(f_ptarget_y);
		f_ptarget_y = NULL;
	}
	
	f_ptarget_x = (float *)calloc(f_ntargets, sizeof(float));
	f_ptarget_y = (float *)calloc(f_ntargets, sizeof(float));

	for (i=0; i<f_ntargets; i++)
	{
		f_ptarget_x[i] = f_fixpt_x + f_target_offset * cos(ANGLE_RAD(i));
		f_ptarget_y[i] = f_fixpt_y + f_target_offset * sin(ANGLE_RAD(i));
	}
	 
	/* 
	 * Clean up existing trial generator, if any. 
	 */
	 
	if (f_prtgen)
	{
		dprintf("Cleanup existing trial generator...\n");
		rtg_destroy(f_prtgen);
	}
	dprintf("Create random trial generator: %d conditions, %d trials, blocksize %d\n", f_ntargets, f_ntrials, f_nblocksize);
	f_prtgen = rtg_create(f_ntargets, f_ntrials, f_nblocksize);

	/*
	 * bcodes
	 */
	 
	bcode_int(CH_SEED, f_seed);
	bcode_int(CH_NTRIALS, f_ntrials);
	bcode_int(CH_NBLOCKSIZE, f_nblocksize);
	bcode_int(CH_SCREENDIST, f_screen_distance_MM);
	bcode_int(CH_NTARGETS, f_ntargets);
	bcode_float(CH_FIXPT_X, f_fixpt_x);
	bcode_float(CH_FIXPT_Y, f_fixpt_y);
	bcode_float(CH_FIXPT_DIAMETER, f_fixpt_diameter);
	bcode_float(CH_FIXPT_WINDOW, f_fixpt_window);
	bcode_float(CH_TARGET_OFFSET, f_target_offset);
	bcode_float(CH_TARGET_DIAMETER, f_target_diameter);
	bcode_float(CH_TARGET_WINDOW, f_target_window);
	
	
	return 0;
}



/* 
 * my_trial_init()
 * 
 * Initializations for a single trial. A set of trial conditions is chosen
 * and configured (i.e. render commands sent, experimental variables selected).
 * The trial will not be aborted during the fixation phase of the state
 * set. There are three ways the trial will be aborted (and hence new trial
 * conditions chosen when this function is called): successful completion, 
 * unsuccessful completion (no answer given), or a PAUSE during the trial. 
 * See my_trial_done(). 
 * 
 */


int my_trial_init()
{
	int status = 0;		/* if set nonzero an ecode will be dropped with that value */

	if ((f_trial_index = f_prtgen->next(f_prtgen)) < 0)
	{
		dprintf("All trials completed.\n");
		f_all_done = 1;
	}
	else
	{
		dprintf("Trial type %d: target angle %d\n", f_trial_index, ANGLE_DEG(f_trial_index)); 

		/*
		 * bcodes
		 */

		ecode(TRIALCD); 
		bcode_float(CH_TARGET_X, f_ptarget_x[f_trial_index]);
		bcode_float(CH_TARGET_Y, f_ptarget_y[f_trial_index]);
		bcode_int(CH_TRIALINDEX, f_trial_index);



		/* 
		 * Update target and fixpt position, but do not display either. 
		 */

		f_fixpt.xorigin = to_pixels(f_fixpt_x);
		f_fixpt.yorigin = to_pixels(f_fixpt_y);
		render_update(f_fixpt_handle, &f_fixpt, sizeof(f_fixpt), HANDLE_OFF);

		f_target.xorigin = to_pixels(f_ptarget_x[f_trial_index]);
		f_target.yorigin = to_pixels(f_ptarget_y[f_trial_index]);
		render_update(f_target_handle, &f_target, sizeof(f_target), HANDLE_OFF);

		render_frame(0);		


		/*
		 * Set state times
		 */

		set_times("trial_init_pause", f_trial_init_pause_time, -1);
		set_times("fixpt_acq", f_fixpt_acq_time, -1);
		set_times("fixpt_acq_fail_pause", f_fixpt_acq_fail_pause_time, -1);
		set_times("fixpt_hold", f_fixpt_hold_time, -1);
		set_times("fixpt_acq_noise_pause", f_fixpt_acq_noise_pause_time, -1);
		set_times("target_fixpt_overlap", f_target_fixpt_overlap_time, -1);
		set_times("target_off_wait", f_target_off_wait_time, -1);
		set_times("target_wait_for_answer", f_target_wait_for_answer_time, -1);
		set_times("target_fix", f_target_fix_time, -1);
		set_times("intertrial_pause", f_intertrial_pause_time, -1);
		set_times("reward_on", f_reward_preset, f_reward_random);

	}

	return status;
}


/*
 * my_trial_done()
 * 
 * Action called after a trial completes. Closes analog window.
 * Do cleanup of stuff allocated at the beginning of the trial.
 *  
 * 
 */

int my_trial_done(int icorrect)
{
	/*
	 * Close eye windows
	 */
	 
	my_eye_window(EYE_WINDOW_OFF);
	
	/* 
	 * Clear screen. Sending FRAME - be sure to catch WENT!
	 */
	
	alloff();
	render_frame(0);
	
	/*
	 * Record response and reward if a response was given.
	 */
	 
	switch(icorrect)
	{
		case TRIAL_DONE_CORRECT:
			dprintf("Correct answer.(answer tallies not implemented!)\n");
			f_prtgen->mark(f_prtgen, f_trial_index);
			break;
		case TRIAL_DONE_NOFIX:
			dprintf("Did not fixate at target location. (answer tallies not implemented!)\n");
			break;
		case TRIAL_DONE_NOANSWER:
			dprintf("No answer given!\n");
			break;
		case TRIAL_DONE_BREAKFIX:
			dprintf("Fixation broken!\n");
			break;
		case TRIAL_DONE_PAUSE:
			dprintf("Pause during trial!\n");
			break;
		default:
			dprintf("ERROR: Unrecognized arg to my_trial_done (%d)\n", icorrect);
			break;

	}
		
	/* Show status of random trial counts. */
	show_trials_status();

	
	return 0;
}



void show_trials_status()
{
	int i, j;
	dprintf("===================Trial counts=================\n");
	dprintf("ind\ttally\n");
	dprintf("---\t-------------------------------\n");
	for (i=0; i<f_ntargets; i++)
	{
		dprintf("%d\t", i);
		for (j=0; j<f_prtgen->count(f_prtgen, i); j++) dprintf("X");
		dprintf("\n");
	}
	dprintf("\n");  
	return;		
}



/* 
 * my_reward
 * 
 * Called as an action from reward states. 
 */
 
int my_reward(int ireward)
{
	int status = 0;
	if (ireward)
	{
		dio_on(REW);
		if (f_verbose & DEBUG_REWARD_BIT) dprintf("reward on\tnow=%d\n", getClockTime());
	}
	else
	{
		dio_off(REW);
		if (f_verbose & DEBUG_REWARD_BIT) dprintf("reward off\tnow=%d\n", getClockTime());
	}
	return status; 
}


/* 
 * my_eye_window()
 * 
 * Open/close eye window(s).
 * The arguments have these effects:
 * 
 * EYE_WINDOW_INIT: 		set eye window to fixpt pos, turn OFF
 * EYE_WINDOW__ON: 			Turn on checking of eye window
 * EYE_WINDOW_OFF:			Turn off checking of eye window
 * EYE_WINDOW_MOVE: 		set eye window to target pos
 */


int my_eye_window(int iflag)
{
	int status = 0;

	if (iflag & EYE_WINDOW_INIT)
	{
		wd_src_pos(EYE_WINDOW_ID, WD_DIRPOS, 0, WD_DIRPOS, 0);
		wd_pos(EYE_WINDOW_ID, (long)(to_degrees(f_fixpt.xorigin)*10.0), (long)(to_degrees(f_fixpt.yorigin)*10.0));
		wd_siz(EYE_WINDOW_ID, (long)(f_fixpt_window*10.0), (long)(f_fixpt_window*10.0));
		wd_cntrl(EYE_WINDOW_ID, WD_OFF);
		wd_src_check(EYE_WINDOW_ID, WD_SIGNAL, 0, WD_SIGNAL, 1);
	}

	if (iflag & EYE_WINDOW_ON)
	{
		wd_cntrl(EYE_WINDOW_ID, WD_ON);
	}
	
	if (iflag & EYE_WINDOW_MOVE)
	{
		if (f_verbose & DEBUG_BIT) dprintf("my_eye_window(EYE_WINDOW_MOVE): %d, %d\n", (int)(to_degrees(f_fixpt.xorigin)*10.0), (int)(to_degrees(f_fixpt.yorigin)*10.0));
		wd_pos(EYE_WINDOW_ID, (long)(f_ptarget_x[f_trial_index]*10.0), (long)(f_ptarget_y[f_trial_index]*10.0));
		wd_siz(EYE_WINDOW_ID, (long)(f_target_window*10.0), (long)(f_target_window*10.0));
	}

	if (iflag & EYE_WINDOW_OFF)
	{
		wd_cntrl(EYE_WINDOW_ID, WD_OFF);
	}

	return status;
}

int fixpt_onoff(int onoff)
{
	if (onoff)
	{
		render_onoff(&f_fixpt_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT);
	}
	else
	{
		render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);
	}
	return 0;
}

int target_onoff(int onoff)
{
	if (onoff)
	{
		render_onoff(&f_target_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT);
	}
	else
	{
		render_onoff(&f_target_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);
	}
	return 0;
}

int alloff()
{
	fixpt_onoff(0);
	target_onoff(0);
	return 0;
}


/*
 * all_trials_done()
 * 
 * Called from state all_done - where we end up when all trials and blocks
 * have been completed. It'd probably be nice to print out statistics or some-
 * thing here.....
 */

int all_trials_done()
{
	dprintf("All trials done.\n");
	return 0;
}




/* REX state set starts here 
 * djs 5-13-09 id=501 
 */

%%

id 52
restart local_init
main_set {
status ON
begin	first:
		code HEADCD
		rl 0
		to p1 
	p1:
		to p2 on +PSTOP & softswitch
		to sendping on -PSTOP & softswitch
	p2:	
		code PAUSECD
		to sendping on -PSTOP & softswitch
	sendping:
		do render_send_ping()
		to reset on 1 % render_check_for_ping
	reset:
		rl 25
		do render_reset()
		to render_init
	render_init:
		do my_render_init()
		to exp_init on 1 % my_check_for_handle
	exp_init:
		do my_exp_init()
		to trial_init
	trial_init:
		do my_trial_init()
		to all_done_wait on 1 = f_all_done
		to trial_init_pause on 1 % render_check_for_went
	trial_init_pause:
		time 500			/* f_trial_init_pause_time */
		to pause_detected on +PSTOP & softswitch
		to fixpt_on
	fixpt_on:
		do fixpt_onoff(1);
		to fixpt_window on 1 % render_check_for_went
	fixpt_window:
		code FPONCD
		time 20
		do my_eye_window(EYE_WINDOW_INIT | EYE_WINDOW_ON)
		to fixpt_acq
	fixpt_acq:
		time 4000			/* f_fixpt_acq_time */
		to pause_detected on +PSTOP & softswitch
		to fixpt_hold on -WD0_XY & eyeflag
		to fixpt_acq_fail
	fixpt_acq_fail:
		do fixpt_onoff(0)
		to fixpt_acq_fail_pause on 1 % render_check_for_went
	fixpt_acq_fail_pause:
		code FPOFFCD
		time 500			/* f_fixpt_acq_fail_pause_time */
		do my_eye_window(EYE_WINDOW_OFF)
		to fixpt_on
	fixpt_hold:
		time 150			/* f_fixpt_hold_time */
		to pause_detected on +PSTOP & softswitch
		to fixpt_acq_noise on +WD0_XY & eyeflag
		to fixation
	fixpt_acq_noise:
		code BREAKFIXCD
		do fixpt_onoff(0)
		to fixpt_acq_noise_pause on 1 % render_check_for_went
	fixpt_acq_noise_pause:
		code FPOFFCD
		time 500			/* f_fixpt_acq_noise_pause_time */
		do my_eye_window(EYE_WINDOW_OFF)
		to fixpt_on
	fixation:
		code FIXCD
		to target_on
	target_on:
		do target_onoff(1)
		to target_fixpt_overlap on 1 % render_check_for_went
	target_fixpt_overlap:
		code STIMON
		time 125		/* f_target_fixpt_overlap_time */
		to fixation_fail on +WD0_XY & eyeflag
		to target_off
	target_off:
		do target_onoff(0)
		to target_off_wait on 1 % render_check_for_went
	target_off_wait:
		code STIMOFF
		time 800		/* f_target_off_wait_time */
		to fixation_fail on +WD0_XY & eyeflag
		to fixpt_off
	fixpt_off:
		do fixpt_onoff(0)
		to target_window on 1 % render_check_for_went
	target_window:
		time 20
		do my_eye_window(EYE_WINDOW_MOVE)
		to target_wait_for_answer
	target_wait_for_answer:
		time 1500		/* f_target_wait_for_answer_time */
		to target_fix on -WD0_XY & eyeflag
		to target_no_answer
	target_fix:
		time 200		/* f_target_fix_time */
		to target_fix_fail on -WD0_XY & eyeflag
		to target_fix_success
	target_fix_success:
		code CORRECTCD
		do my_trial_done(TRIAL_DONE_CORRECT)
		to reward_on on 1 % render_check_for_went
	reward_on:
		code REWCD
		do my_reward(1)
		to reward_off
	reward_off:
		do my_reward(0)
		to intertrial_pause		
	target_fix_fail:
		code WRONGCD
		do my_trial_done(TRIAL_DONE_NOFIX)
		to intertrial_pause on 1 % render_check_for_went
	target_no_answer:
		do my_trial_done(TRIAL_DONE_NOANSWER)
		to intertrial_pause on 1 % render_check_for_went			
	fixation_fail_wait:
		to fixation_fail on 1 % render_check_for_went
	fixation_fail:
		code BREAKFIXCD
		do my_trial_done(TRIAL_DONE_BREAKFIX)
		to intertrial_pause on 1 % render_check_for_went
	intertrial_pause:
		time 500		/* f_intertrial_pause_time */
		to trial_init
	pause_detected_wait:
		to pause_detected on 1 % render_check_for_went
	pause_detected:
		code PAUSECD
		do my_trial_done(TRIAL_DONE_PAUSE)
		to pause_wait on 1 % render_check_for_went
	pause_wait:
		to trial_init on -PSTOP & softswitch
	all_done_wait:
		to all_done on 1 % render_check_for_went
	all_done:
		do all_trials_done()
		to first on 1 = f_never
/*abort	list:
		fixation_fail*/
}

		

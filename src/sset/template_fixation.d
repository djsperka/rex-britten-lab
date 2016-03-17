/* $Id $ */

/*
 * This is a template for a simple fixation experiment. DO NOT MODIFY THIS FILE!
 * 
 * The experiment takes the subject through a simple fixation paradigm. Time values below are denoted with 
 * the labels used in the "Timing" menu followed by the variable in parentheses. The timing values are updated
 * in my_trial_init(), which is called at the start of each trial. 
 * 
 * 1.) Fixation point is presented.
 * 2.) Eye must be within window within "acq_time(ms)"(f_acq_time) or acquisition fails.A blank screen is shown for 
 *     acq_timeout(ms)"(f_acq_fail_time) before fixation point is presented again. 
 * 3.) If eye is within window, it must stay there for "fix_time(ms)"(f_fixation_time) or we consider it noise.
 *     A blank screen is shown for fix_timeout(ms)"(f_acq_noise_time) before fixation point is presented again.
 * 4.) If eye remains in window for "fix_time(ms)"(f_fix_time) a reward is given. 
 * 5.) After reward is given the trial is over. There is a pause for "intertrial_time(ms"(f_intertrial_time) before
 *     a new trial begins (1). 
 * 
 * Notes:
 * 
 * - When acquisition fails or noise is detected the trial does not end.The fixpt is presented repeatedly until 
 *   fixation is achieved.    
 * - Initialization of trial conditions should be done in my_trial_init. Note that the eye window is initialized and 
 *   opened there, and it is closed in my_trial_done. The eye window is not closed when acquisition fails or noise is 
 *   detected. 
 * - Reward size is specified in the reward menu, and the values are set at the start of each trial. 
 * 
 */


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
#define FFONCD		1544	/* flow field is on screen */
#define FFMOVCD		1545
#define FFOFFCD		1546
#define USTIMONCD	1550
#define USTIMOFFCD	1551


/* 
 * bit masks for verbosity 
 */
 
#define DEBUG_BIT 0x1
#define DEBUG_REWARD_BIT 0x2
#define DEBUG_EYE_WINDOW_BIT 0x4
#define DEBUG_ANSWER_POINTS_BIT 0x8

/* 
 * These are values which can be passed to my_eye_window (they may be OR'd together).  
 */

#define EYE_WINDOW_INIT				0x1
#define EYE_WINDOW_ON				0x2
#define EYE_WINDOW_OFF				0x4

/*
 * This is the eye window identifier
 */
 
#define EYE_WINDOW_ID 0

/*
 * These are arg values for my_trial_done. 
 */
 
#define TRIAL_DONE_SUCCESS 1
#define TRIAL_DONE_BREAKFIX -2
#define TRIAL_DONE_PAUSE -3

char f_charbuf[2048];			/* Really really big char for sprinting errors. */
int f_never = -1;				/* never var, quoth the raven */
int f_seed_used = 0;			/* indicates whether we've seeded the random number generator or not */
int f_fixpt_handle = 0;			/* graphics handle for fixation point */
int f_dots_handle = 0;			/* graphics handle for dot cloud */
int f_handle_count = 0;			/* graphics handle counter */
float f_cam_position[3] = {0, 0, 0};		/* Initial camera position */
float f_cam_looking[3];			/* where the camera is aimed */
float f_cam_up[3];				/* up direction for camera */
int f_trial_condition_index;	/* index into f_pextcond array for current trial */
int f_all_done = 0;				/* flag indicating all trials have been completed */
int f_wstatus = 0;				/* status indicator used by my_check_for_went */
int f_went_cycles = 0;

DotStruct f_fixpt;

struct condition_struct
{
	int pursuit_sgn;		/* -1, 0, +1 */
	int angle_index;		/* [-f_nangles, f_nangles] */
	float angle_degrees;
	int ustim;				/* 0/1 */
	int show_dots;				/* 0/1 - 1 means that dots are displayed this trial type */
};

typedef struct condition_struct ExptCondition;
ExptCondition* f_pexptcond = NULL;
int f_nexptcond = 0;

RTGenerator *f_prtgen = NULL;
int f_trialsCompleted = 0;


void init_steering(void);
int my_render_init();
int my_check_for_handle();
int my_exp_init();
int my_trial_init();
int my_check_for_went();
int my_trial_done(int icorrect);
void show_trials_status();
int alloff();
int my_eye_window(int iflag);
int answer(int icorrect);
int fixpt_onoff(int onoff);
int dots_onoff(int onoff);



/* 
 * **************************************************************************
 * 
 *                     REX menu declarations 
 * 
 * **************************************************************************
 */


RTVAR rtvars[] = {
	{"number of trials",	&f_trialsCompleted},
	{"eyeh", &eyeh}, 
	{"eyev", &eyev}, 
	{"", 0},
};


USER_FUNC ufuncs[] = {
	{"trials",	&show_trials_status, "void"},
	{""},
};



/*
 * state_vl menu
 */

int f_seed = 9999;						/* random number seed */
int f_ntrials = 50;						/* number of trials for each condition */
int f_nblocksize = 1;					/* block size that trials are gathered in */
int f_nangles = 3;						/* Number of positive heading angles. Total angles = 2*f_nangles+1 */
float f_maxangle = 16.0;				/* maximum heading angle */
float f_minangle = 1.0;				/* minimum heading angle >0 */
int f_use_ustim = 0;						/* if 1 then use ustim for this expt */
int f_eye_speed = 30;					/* measured in 10's/sec */
int f_verbose = 0;						/* debugging flag */
int f_reward_preset = 60;				/* reward size preset value */
int f_reward_random = 10;				/* reward size random value */

VLIST state_vl[] = {
"day_seed",		&f_seed, NP, NP, 0, ME_DEC,
"reward_preset", &f_reward_preset, NP, NP, 0, ME_DEC,
"reward_random", &f_reward_random, NP, NP, 0, ME_DEC,
"verbose", &f_verbose, NP, NP, 0, ME_DEC,
NS,
};

char hm_sv_vl[] = "";

/* 
 * Viewing volume menu 
 */

int f_screen_distance_MM = 300;			/* screen distance in mm */
int f_far_plane_distance = 1300;		/* far plane of viewing frustrum */

VLIST vvol_vl[] = {
"screen_dist(mm)", &f_screen_distance_MM, NP, NP, 0, ME_DEC,
"far_plane_distance", &f_far_plane_distance, NP, NP, 0, ME_DEC, 
NS,
};

char hm_vvol[] = "";

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
 
float f_fixpt_diameter = 0.25;				/* diameter of fixpt (degrees) */
float f_fixpt_window = 3;				/* size of fixation point window */
int f_fixpt_color[3] = {255, 0, 0};		/* fixpt color [0,255] */
float f_fixpt_refpos[2] = {0, 0};

VLIST fixpt_vl[] = {
"reference_pt_x(deg)", &f_fixpt_refpos[0], NP, NP, 0, ME_FLOAT,
"reference_pt_y(deg)", &f_fixpt_refpos[1], NP, NP, 0, ME_FLOAT,
"fixpt_diameter(deg)", &f_fixpt_diameter, NP, NP, 0, ME_FLOAT,
"fixpt_window(deg)", &f_fixpt_window, NP, NP, 0, ME_FLOAT,
"fixpt_color(R)", &f_fixpt_color[0], 0, NP, 0, ME_DEC,
"fixpt_color(G)", &f_fixpt_color[1], 0, NP, 0, ME_DEC,
"fixpt_color(B)", &f_fixpt_color[2], 0, NP, 0, ME_DEC,
NS,
};

char hm_fixpt[] = "";

/* 
 * Timing menu - all values in milliseconds
 */


int f_trial_init_pause_time = 500;	/* pause after trial init, prior to fixpt on */
int f_acq_time = 2000;				/* max time to acquire fixpt */
int f_acq_fail_time = 1000;			/* if acq fails, wait this long before restarting */
int f_fixation_time = 500;			/* how long until fixation */
int f_acq_noise_time = 250;			/* if acq fails due to noise, wait this long before restarting */
int f_intertrial_time = 250;		/* time between trials */

VLIST timing_vl[] = {
"trial_init_pause", &f_trial_init_pause_time, NP, NP, 0, ME_DEC,
"acq_time(ms)", &f_acq_time, NP, NP, 0, ME_DEC,
"acq_timeout(ms)", &f_acq_fail_time, NP, NP, 0, ME_DEC,
"fix_time(ms)", &f_fixation_time, NP, NP, 0, ME_DEC,
"fix_timeout(ms)", &f_acq_noise_time, NP, NP, 0, ME_DEC,
"intertrial_time(ms)", &f_intertrial_time, NP, NP, 0, ME_DEC,
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
{"separator", NP}, 
{"timing", &timing_vl, NP, NP, 0, NP, hm_timing}, 
{"fixation", &fixpt_vl, NP, NP, 0, NP, hm_fixpt}, 
{"viewing volume", &vvol_vl, NP, NP, 0, NP, hm_vvol},
{"background", &background_vl, NP, NP, 0, NP, hm_background}, 
{"communication", &comm_vl, NP, NP, 0, NP, hm_comm}, 
{NS},
};


/*
 * Channel definitions for data stuck in E file
 */


/*
 * channel definitions for per-trial bcodes
 */

#define CH_TRIAL_CONDITION	1
#define CH_PURSUIT_SGN		2
#define CH_ANGLE_INDEX		3
#define CH_ANGLE_DEGREES	4
#define CH_USTIM			5
#define CH_FIXPT_X			6
#define CH_FIXPT_Y			7
#define CH_SHOW_DOTS		8
 


/***************************** init_steering *****************************
 *                                                                       *
 * This function is the "reset" function in the state set. In other words, 
 * the state set specification contains the line:
 * 
 * restart init_steering
 * 
 * That line makes this function special, in that it will be called at the 
 * following times:
 * 
 * - the first time the clock is started
 * - whenever reset state happens
 *                                                                       *
 ************************************************************************/
void init_steering(void)
{
	int status=0;
	
	dprintf("Initializing paradigm\n");

	// initialize tcpip - must only do this OR gpib - NOT BOTH!!!!
	status = init_tcpip(f_local_addr, f_remote_addr, f_remote_port, 0);

	// initialize pixel conversion
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
	float dotfield_xmax, dotfield_ymax, dotfield_zmax;

	dprintf("Initializing render\n");

	// seed random number generator if necessary
	// TODO: Make usage of random number generator (and seed) consistent. 

	if (!f_seed_used)
	{
		ivunif(f_seed, 1);
		f_seed_used = 1;
	}
	
	// zero out handles and initialize counters.

	f_fixpt_handle = f_dots_handle = 0;
	f_handle_count = 0;

	// background color

	render_bgcolor(f_background_color[0], f_background_color[1], f_background_color[2]);

	// Setup viewing volume -- this should be done prior to the groundplane init! */

	fovy = 2*atan2f(y_dimension_mm/2.0, f_screen_distance_MM); 	
	render_perspective(fovy, f_screen_distance_MM, f_far_plane_distance);

	/* Configure fixation point */
	
	f_fixpt.xorigin = 0;
	f_fixpt.yorigin = 0;
	f_fixpt.xsize = to_pixels(f_fixpt_diameter);
	f_fixpt.ysize = to_pixels(f_fixpt_diameter);
	f_fixpt.depth = 1;
	f_fixpt.r = f_fixpt_color[0];
	f_fixpt.g = f_fixpt_color[1];
	f_fixpt.b = f_fixpt_color[2];
	render_dot(&f_fixpt);
	
	return status;
}


/* 
 * my_check_for_handle 
 * 
 * Called as an escape function ... e.g. 
 * 
 * to wherever on 1 % my_check_for_handle
 * 
 * This function will fetch 1 handle from render. It is saved
 * as f_fixpt_handle.
 *  
 * The counter f_handle_count must be zeroed before this is used. 
 * Since the allocation of graphic objects takes place early in the 
 * state set (render_init), the handle counter should get zeroed 
 * in the my_render_init() function. 
 *
 * On each invocation it checks once for a handle.If a handle is found 
 * it is assigned to the proper variable and the handle counter is 
 * incremented.  
 * 
 * Returns 0 if the handle counter is less than 1, or 1 if the handle 
 * counter is 1.
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
			status = 1;
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
	int i, j, k;
	int nconditions;
	int nconditions_ustim;
	int nconditions_pursuit;
	int nconditions_pursuit_control;
	int nspeeds;
	int nconditions_angles;
	int index = 0;
	float log_ax;		/* log2(max angle) */
	float log_an;		/* log2(min angle) */
	float frac;			/* used in computing angle */
	int sign;
	float angle;
	
	dprintf("Initializing experiment\n");

	f_all_done = 0;
	f_trialsCompleted = 0;
	alloff();
	render_frame(0);

	/* Free experimental condition struct array if it exists */
	if (f_pexptcond)
	{
		dprintf("Cleanup existing experimental condition list...\n");
		free(f_pexptcond);
	}
	
#if 0	
	/* 
	 * Figure out how many trial types there are. 
	 * There are 4 condition variables: ustim(0/1), angle, pursuit speed, pursuit control.
	 * The "heading 0" condition means we have a trial type with a heading angle of 0 degrees. 
	 * The "pursuit control" condition means that the trial should have no dots displayed during pursuit.
	 */
	 
	nconditions_ustim = (f_use_ustim ? 2 : 1); 
	if (!f_use_heading_0_condition)
	{
		nconditions_angles = f_nangles*2;			/* Note: must add 1 to this if you want to use heading angle = 0 */
	}
	else
	{
		nconditions_angles = f_nangles*2 + 1;
	}		
	nspeeds = (f_pursuit_speed>0 ? 1 : 0);
	nconditions_pursuit = nspeeds*2 + 1;
	if (!f_use_pursuit_control_condition )
	{
		nconditions_pursuit_control = 0;
	}
	else
	{
		nconditions_pursuit_control = 1;
	}

	/*
	 * Now allocate the array and zero it.
	 */
	 	 
	f_nexptcond =  nconditions_ustim * (nconditions_angles + nconditions_pursuit_control) * nconditions_pursuit;
	dprintf("Generate list of %d trial conditions; min/max angles %d/%d.\n", f_nexptcond, (int)(f_minangle*10), (int)(f_maxangle*10));
	f_pexptcond = (ExptCondition *)calloc(f_nexptcond, sizeof(ExptCondition));
	memset(f_pexptcond, 0, f_nexptcond * sizeof(ExptCondition));
	index = 0;

	/* 
	 * TODO: better test values of min and max angle in dialogs!!! 
	 * TODO: better verify that f_nangles > 0!!!Some of the logic below covers that case, but we are NOT using 
	 * angle=0 at this point!!!!
	 * 
	 * Compute angles for each condition. The setting in the dialog is 
	 * the number of heading angles desired between the min and max angles. 
	 * The negatives of each of the angles between the min and max are 
	 * assumed to be included. The angles between min and max are
	 * log spaced - that computation is below. Special cases to be covered
	 * are:
	 * f_nangles = 0 - there is one and only one angle = 0. NOTE: NOT VALID UNLESS ANGLE=0 ALLOWED!!!
	 * f_nangles = 1 = the formula is dangerous; the only angle used is the max!
	 * f_nangles > 1 = fancy formula. 
	 * 
	 */

	log_an = log2(f_minangle);
	log_ax = log2(f_maxangle);

	for (i=0; i<nconditions_ustim; i++)
	{

		/* 
		 * This loop creates trial types that use dots (and hence the heading angle is important). 
		 * Trial types with "pursuit_control" are handled below. 
		 */ 

		for (j=-1*f_nangles; j<=f_nangles; j++)
		{
			/*
			 * WARNING: Skip index j==0 here - this removes the heading angle = 0 from the list 
			 * of trial conditions. 
			 */
			if (j == 0 && !f_use_heading_0_condition) continue;
			
			if (f_nangles == 0) angle = 0;
			else if (f_nangles == 1)
			{
				if (j==0) angle = 0;
				else if (j==-1) angle = -f_maxangle;
				else angle = f_maxangle;
			}
			else
			{
				if (j == 0) angle = 0;
				else
				{
					frac = (float)(abs(j)-1)/(float)(f_nangles-1);
					sign = j<0 ? -1 : 1;
					angle = sign * exp2(log_an + frac * (log_ax - log_an));
				}
			}

			for (k=-1*nspeeds; k<=nspeeds; k++)
			{
				f_pexptcond[index].ustim = i;
				f_pexptcond[index].angle_index = j;
				f_pexptcond[index].angle_degrees = angle;
				f_pexptcond[index].pursuit_sgn = k;
				f_pexptcond[index].show_dots = 1;
				index++;
				dprintf("%d i=%d j=%d k=%d angle=%d show_dots=1\n", index, i, j, k, (int)(angle*100));
			}
		}
		
		/* 
		 * Now handle conditions with no dots. Here we do NOT loop over angles -- the heading angle is irrelevant
		 * when the dots are not shown! We must have one trial type for each pursuit speed (and
		 * for each ustim condition).
		 */

		if (f_use_pursuit_control_condition)
		{
			for (k=-1*nspeeds; k<=nspeeds; k++)
			{
				f_pexptcond[index].ustim = i;
				f_pexptcond[index].angle_index = 0;
				f_pexptcond[index].angle_degrees = 0.0;
				f_pexptcond[index].pursuit_sgn = k;
				f_pexptcond[index].show_dots = 0;
				index++;
				dprintf("%d i=%d j=%d k=%d angle=%d show_dots=0\n", index, i, j, k, (int)(angle*100));
			}
		}			
	}

	

	/* 
	 * Initialize trial generator. The trial generator will generate an index
	 * into the array f_exptcond[] which we just created and initialized.
	 */

	if (f_prtgen)
	{
		dprintf("Cleanup existing trial generator...\n");
		rtg_destroy(f_prtgen);
	}
	dprintf("Create random trial generator: %d conditions, %d trials, blocksize %d\n", f_nexptcond, f_ntrials, f_nblocksize);
	f_prtgen = rtg_create(f_nexptcond, f_ntrials, f_nblocksize);
	
#endif
	
#if 0
	/*
	 * Drop experimental settings into efile.
	 */
	 
	bcode_int(CH_RANDOM_SEED, f_seed);
	bcode_int(CH_NTRIALS, f_ntrials);
	bcode_int(CH_NBLOCKSIZE, f_nblocksize);
	bcode_int(CH_NANGLES, f_nangles);
	bcode_int(CH_MAXANGLE, f_maxangle);
	bcode_int(CH_MINANGLE, f_minangle);
	bcode_float(CH_PURSUIT_SPEED, f_pursuit_speed);
	bcode_int(CH_USE_USTIM, f_use_ustim);
	bcode_int(CH_EYE_SPEED, f_eye_speed);
	bcode_int(CH_REWARD_PRESET, f_reward_preset);
	bcode_int(CH_REWARD_RANDOM, f_reward_random);
	bcode_int(CH_SCREEN_DISTANCE_MM, f_screen_distance_MM);
	bcode_int(CH_FAR_PLANE_DISTANCE, f_far_plane_distance);
	bcode_int(CH_BKGD_R, f_background_color[0]);
	bcode_int(CH_BKGD_G, f_background_color[1]);
	bcode_int(CH_BKGD_B, f_background_color[2]);
	bcode_float(CH_FIXPT_DIAMETER, f_fixpt_diameter);
	bcode_float(CH_FIXPT_WINDOW, f_fixpt_window);
	bcode_int(CH_FIXPT_R, f_fixpt_color[0]);
	bcode_int(CH_FIXPT_G, f_fixpt_color[1]);
	bcode_int(CH_FIXPT_B, f_fixpt_color[2]);
	bcode_float(CH_FIXPT_REFPOS_X, f_fixpt_refpos[0]);
	bcode_float(CH_FIXPT_REFPOS_Y, f_fixpt_refpos[1]);
	bcode_float(CH_TARGET_DIAMETER, f_target_diameter_degrees);
	bcode_float(CH_TARGET_OFFSET_H, f_target_offset_horizontal_degrees);
	bcode_float(CH_TARGET_OFFSET_V, f_target_offset_vertical_degrees);
	bcode_float(CH_TARGET_WINDOW, f_target_window);
	bcode_int(CH_TARGET_R, f_target_color[0]);
	bcode_int(CH_TARGET_G, f_target_color[1]);
	bcode_int(CH_TARGET_B, f_target_color[2]);
	bcode_int(CH_TRIAL_INIT_PAUSE_TIME, f_trial_init_pause_time);
	bcode_int(CH_ACQ_TIME, f_acq_time);
	bcode_int(CH_ACQ_FAIL_TIME, f_acq_fail_time);
	bcode_int(CH_FIXATION_TIME, f_fixation_time);
	bcode_int(CH_ACQ_NOISE_TIME, f_acq_noise_time);
	bcode_int(CH_INTERTRIAL_TIME, f_intertrial_time);
	bcode_int(CH_PRE_PURSUIT_TIME, f_pre_pursuit_time);
	bcode_int(CH_PRE_DOTS_MOTION_TIME, f_pre_dots_motion_time);
	bcode_int(CH_DOTS_MOTION_TIME, f_dots_motion_time);
	bcode_int(CH_ANSWER_TIME, f_answer_time);
	bcode_float(CH_DOTFIELD_SIZE_X, f_dotfield.xwidth);	
	bcode_float(CH_DOTFIELD_SIZE_Y, f_dotfield.ywidth);	
	bcode_float(CH_DOTFIELD_SIZE_Z, f_dotfield.zwidth);	
	bcode_float(CH_DOTFIELD_ORIGIN_X, f_dotfield.xorigin);
	bcode_float(CH_DOTFIELD_ORIGIN_Y, f_dotfield.yorigin);
	bcode_float(CH_DOTFIELD_ORIGIN_Z, f_dotfield.zorigin);
	bcode_int(CH_DOTFIELD_NDOTS, f_dotfield_ndots);
	bcode_float(CH_DOTFIELD_POINTSIZE, f_dotfield_pointsize);
	bcode_int(CH_DOTFIELD_R, f_dotfield_color[0]);
	bcode_int(CH_DOTFIELD_G, f_dotfield_color[1]);
	bcode_int(CH_DOTFIELD_B, f_dotfield_color[2]);
	bcode_int(CH_FRAMES_PER_SECOND, f_frames_per_second);
#endif

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
	int i;
	
	/*
	 * Initialize eye window. This should be done after we decide where the fixpt will be.
	 */
	 
	my_eye_window(EYE_WINDOW_INIT | EYE_WINDOW_ON);
	
	render_frame(0);
	
#if 0
	/* 
	 * Check for fixpt condition to apply to this trial.If the index
	 * returned is -1 it means all trials have been completed.  
	 */
	 
	f_trial_condition_index = f_prtgen->next(f_prtgen);
	if (f_trial_condition_index < 0)
	{
		dprintf("All trials completed.\n");
		f_all_done = 1;
	}
	else
	{
		dprintf("Trial type %d: angle %d (%d) pursuit sign %d ustim %d\n", f_trial_condition_index, 
				(int)(f_pexptcond[f_trial_condition_index].angle_degrees*10), 
				f_pexptcond[f_trial_condition_index].angle_index,
				f_pexptcond[f_trial_condition_index].pursuit_sgn,
				f_pexptcond[f_trial_condition_index].ustim);

#endif


		/* 
		 * Assign time values to states 
		 */
		 
		set_times("trial_init_pause_time", (long)f_trial_init_pause_time, -1); 
		set_times("fixpt_acq", (long)f_acq_time, -1);
		set_times("fixpt_acq_fail_pause", (long)f_acq_fail_time, -1);
		set_times("fixpt_hold", (long)f_fixation_time, -1);
		set_times("fixpt_acq_noise_pause", (long)f_acq_noise_time, -1);
		set_times("intertrial_pause", (long)f_intertrial_time, -1);
		set_times("reward_on", (long)f_reward_preset, (long)f_reward_random);
		if (f_verbose & DEBUG_REWARD_BIT)
		{
			dprintf("setting state times: \n");
			dprintf("trial_init_pause: %d\n", f_trial_init_pause_time);
			dprintf("fixpt_acq: %d\n", f_acq_time);
			dprintf("fixpt_acq_fail_pause: %d\n", f_acq_fail_time);
			dprintf("fixpt_acq_noise_pause: %d\n", f_acq_noise_time);
			dprintf("fixpt_hold: %d\n", f_fixation_time);
			dprintf("intertrial_pause: %d\n", f_intertrial_time);
			dprintf("reward_on: preset %d random %d\n", f_reward_preset, f_reward_random);
		}
#if 0
		/*
		 * Open the analog window. 
		 */

		awind(OPEN_W);
		
		
		/*
		 * bcodes for this trials conditions
		 */

		bcode_int(CH_TRIAL_CONDITION, f_trial_condition_index);
		bcode_int(CH_PURSUIT_SGN, f_pexptcond[f_trial_condition_index].pursuit_sgn);
		bcode_int(CH_ANGLE_INDEX, f_pexptcond[f_trial_condition_index].angle_index);
		bcode_float(CH_ANGLE_DEGREES, f_pexptcond[f_trial_condition_index].angle_degrees);
		bcode_int(CH_USTIM, f_pexptcond[f_trial_condition_index].ustim);
		bcode_int(CH_SHOW_DOTS, f_pexptcond[f_trial_condition_index].show_dots);
	}
#endif
	

	return status;
}




/* 
 * my_check_for_went()
 * 
 * 
 * This is a state escape which checks for a WENT retured from render. 
 * It is designed to be called multiple times while waiting for the WENT, and
 * it will count the number of times its called until the WENT finally arrives. 
 * 
 * A counter, f_went_cycles, is incremented each time a WENT is received. In 
 * addition a frame counter is also incremented. The frame counter may not be 
 * the same as the went counter if the paradigm was paused or if there were 
 * missed frames. 
 * 
 * This function is designed to be used in combination with my_animate() - the
 * state action that drives the animations for this paradigm. There are 6 possible 
 * return values:
 * 
 * 0 - no WENT received
 * 1 - WENT received, continue animation
 * 2 - WENT received, start the ustim
 * 3 - WENT received, animation is complete, stop ustim
 * 4 - WENT received, animation is complete, move on
 * -1 - ERROR!
 * 
 * When no WENT is received, f_went_cycles is incremented. 
 * 
 * When a WENT is received there are several conditions to consider. When the 
 * dot motion starts the WENT will have a flag indicating that a key frame just
 * went up (TODO: EXPLAIN THIS). If this trial type includes ustim, then we set 
 * the return value so the escape goes to the state which turns on the ustim. 
 * Similarly, if the animation has stopped, then my_animate() will have turned off
 * the fixpt and dots and turned on the answer points -- in this case the 
 * return value is set so that we escape to the state where the ustim is turned off. 
 * Finally, if the animation has stopped but this trial is NOT a ustim trial, then we 
 * set the return value so we escape to the state where we begin waiting for an answer. 
 */

int my_check_for_went()
{
	int frames = 0;
	f_wstatus = render_check_for_went(&frames);
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
		/*
		 * A WENT was received. Record a WENT ecode and the fixation point x and y positions. 
		 */
		 
		ecode(WENTCD);
		
#if 0
		bcode_float(CH_FIXPT_X, to_degrees(f_fixpt.xorigin));
		bcode_float(CH_FIXPT_Y, to_degrees(f_fixpt.yorigin));
#endif
				
		/*
		 * If frames were missed, dprint the number missed and the cycles and drop an ecode.
		 * This paradigm has a forced delay prior to animation starting (this method is called
		 * during the animation phase) -- the delay is determined in the state dots_on_wait, and 
		 * the time f_pre_dots_motion_time (see "timing" menu) is the wait time in that state. 
		 * That state turns on the dots and there is a FRAME there. The next FRAME is issued after
		 * the first animation step. Since the step counter is incremented in my_animate and the 
		 * WENT is received here the step counter will be 2 when we see the effect of the delay -- 
		 * dropped frames. For example, if the f_pre_dots_motion_time is 1000 (1000ms = 1s), expect 
		 * a message like this:
		 * 
		 * Missed 85 frames (10 check_went cycles, step_counter 2)
		 * 
		 * The number of frames is determined by f_pre_dots_motion_time and the frames_per_second, 
		 * and you can expect the number of check_went cycles to vary slightly. The step counter 
		 * should be 2 -- if you see missed frame messages and step_counter is NOT 2, then something
		 * else is happening and it should be investigated. As it is, this "error" message is expected. 
		 * The data file will contain a MISSEDCD ecode which can be safely ignored.   
		 */ 
		 
		if (frames > 1)
		{
			dprintf("Missed %d frames (%d check_went cycles)\n", frames, f_went_cycles);
			ecode(MISSEDCD);
		}

	}
	return f_wstatus;
}





/*
 * my_trial_done()
 * 
 * Action called after a trial completes. Closes analog window.
 * Do cleanup of stuff allocated at the beginning of the trial.
 *  
 * 
 */

int my_trial_done(int istatus)
{

	/*
	 * Close eye windows
	 */

	my_eye_window(EYE_WINDOW_OFF);


	switch(istatus)	
	{
		case TRIAL_DONE_SUCCESS:
		{
			dprintf("Success!\n");
			break;
		}
		case TRIAL_DONE_PAUSE:
		{
			dprintf("Pause.\n");
			break;
		}
		case TRIAL_DONE_BREAKFIX:
		{
			dprintf("Break fixation.\n");
			break;
		}
		default:
		{
			dprintf("UNKNOWN STATUS!\n");
			break;
		}
	}



#if 0
	/* 
	 * Close analog window.
	 */
	
	awind(CLOSE_W);
#endif

	/* 
	 * Clear screen. Sending FRAME - be sure to catch WENT!
	 */
	
	alloff();
	render_frame(0);
	
	/* Show status of random trial counts. */
	show_trials_status();

	
	return 0;
}

void show_trials_status()
{
	int i, j, sum;
	dprintf("===================Trial counts=================\n");
	dprintf("TODO\n");
	return;		
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
 */


int my_eye_window(int iflag)
{
	int status = 0;

	if (iflag & EYE_WINDOW_INIT)
	{
		wd_src_pos(EYE_WINDOW_ID, WD_DIRPOS, 0, WD_DIRPOS, 0);
		wd_pos(EYE_WINDOW_ID, (long)(to_degrees(f_fixpt.xorigin)*10.0), (long)(to_degrees(f_fixpt.yorigin)*10.0));
		wd_siz(EYE_WINDOW_ID, (long)(f_fixpt_window*10.0), (long)(f_fixpt_window*10.0));
		/*wd_cntrl(EYE_WINDOW_ID, WD_OFF);*/
		wd_src_check(EYE_WINDOW_ID, WD_SIGNAL, 0, WD_SIGNAL, 1);
	}

	if (iflag & EYE_WINDOW_ON)
	{
		wd_cntrl(EYE_WINDOW_ID, WD_ON);
	}
	
	if (iflag & EYE_WINDOW_OFF)
	{
		wd_cntrl(EYE_WINDOW_ID, WD_OFF);
	}

	return status;
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
 * alloff()
 * 
 * Turns off fixpt and dots. Does not issue a render or frame. 
 * 
 */
 
int alloff()
{
	render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_NO_FRAME);
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




int my_fixation_fail_wait()
{
	return 0;
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



/* REX state set starts here 
 * djs 5-13-09 id=501 
 * djs/tao 8-10-09 id=502 (add heading=0, pursuit_control conditions)
 */

%%

id 502
restart init_steering
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
		do my_exp_init()	/* initialize trial counters */
		to trial_init on 1 % render_check_for_went
	trial_init:
		code TRLSTART
		do my_trial_init()	/* initialization for a single trial */
		to all_done_wait on 1 = f_all_done
		to trial_init_pause on 1 % render_check_for_went
	trial_init_pause:
		time 500			/* This time will be updated based on menu entry - see my_trial_init */
		to pause_detected on +PSTOP & softswitch
		to fixpt_on
	fixpt_on:
		do fixpt_onoff(1);
		to fixpt_acq
	fixpt_acq:
		time 4000
		to pause_detected on +PSTOP & softswitch
		to fixpt_hold on -WD0_XY & eyeflag
		to fixpt_acq_fail
	fixpt_acq_fail:
		do fixpt_onoff(0)
		to fixpt_acq_fail_pause on 1 % render_check_for_went
	fixpt_acq_fail_pause:
		time 500			/* f_acq_fail_pause_time */
		to fixpt_on
	fixpt_hold:
		time 150			/* f_fixation_time */
		to pause_detected on +PSTOP & softswitch
		to fixpt_acq_noise on +WD0_XY & eyeflag
		to fixation
	fixpt_acq_noise:
		do fixpt_onoff(0)
		to fixpt_acq_noise_pause on 1 % render_check_for_went
	fixpt_acq_noise_pause:
		time 500			/* f_acq_noise_pause_time */
		to fixpt_on
	fixation:
		code FIXCD
		to trial_success
	trial_success:
		do my_trial_done(TRIAL_DONE_SUCCESS)
		to reward_on
	reward_on:
		code REWCD
		do my_reward(1)
		to reward_off
	reward_off:
		do my_reward(0)
		to intertrial_pause		
	fixation_fail_wait:
		do my_fixation_fail_wait()
		to fixation_fail on 1 % render_check_for_went
	fixation_fail:
		code BREAKFIXCD
		do my_trial_done(TRIAL_DONE_BREAKFIX)
		to intertrial_pause on 1 % render_check_for_went
	intertrial_pause:
		time 500
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



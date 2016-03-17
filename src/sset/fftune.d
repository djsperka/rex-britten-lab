/* $Id: fftune.d,v 1.9 2010/08/10 22:56:53 devel Exp $ */

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
 
#define DEBUG_INITIALIZATION_BIT 0x1
#define DEBUG_TRIAL_BIT 0x2
#define DEBUG_REWARD_BIT 0x4
#define DEBUG_UPDATE_BIT 0x8

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
int f_handle_count = 0;			/* graphics handle counter */
int f_trial_condition_index;	/* index into f_pextcond array for current trial */
int f_all_done = 0;				/* flag indicating all trials have been completed */
int f_went_cycles = 0;
int f_trial_frame_counter = 0;	/* incremented once each frame during update cycle */
int f_trial_frames_per_stim = 0;/* each stim gets this many updates. I'th stim turned ON at i*f_trial_frames_per_stim frame */
int f_went_status = 0;

/*
 * Render structure for dot, and (empty) array of FF2D's
 */
 
DotStruct f_fixpt;

FF2DStruct *f_pFF2D = (FF2DStruct *)NULL;
int *f_pHandle = (int *)NULL;
int *f_pTrialList = (int *)NULL;

/* 
 * The condition_struct describes the experimental variables unique to a single trial type. 
 * There is one condition_struct for each trial type. 
 */
 
struct condition_struct
{
	int id;
	int ix, iy;
	float x, y;				/* Center of FF2D */
	int ilinear; 			/* 0=linear, 1=radial, -1=no FF2D */
	float angle;			/* relevant angle for FF2D config */
};
typedef struct condition_struct ExptCondition;
ExptCondition* f_pexptcond = NULL;
int f_nexptcond = 0;
char * exptcondition_str(ExptCondition* i_pexptcond, char *str);	/* Helper function for printing. Make sure str is long enough! */

/*
 * Random trial generator
 */
 
RTGenerator *f_prtgen = NULL;
int f_trialsCompleted = 0;
int f_trialsStarted = 0;
unsigned int f_frametag=0;
#define FRAMETAG_MULTIPLIER 1000

void init_steering(void);
int my_render_init();
int my_check_for_handle();
int my_exp_init();
int my_trial_init();
int my_check_for_went();
int my_check_for_went_extended();
int my_trial_done(int icorrect);
void show_trials_status();
int alloff();
int my_eye_window(int iflag);
int answer(int icorrect);
int fixpt_onoff(int onoff);
int dots_onoff(int onoff);
int my_update();



/* 
 * **************************************************************************
 * 
 *                     REX menu declarations 
 * 
 * **************************************************************************
 */


/*
 * state_vl menu
 */

int f_seed = 9999;						/* random number seed */
int f_ntrials = 2;						/* number of trials for each condition */
int f_nblocksize = 1;					/* block size that trials are gathered in */
int f_nstim_per_fixation = 3;			/* Number of stimuli shown each fixation period */
int f_verbose = 11;						/* debugging flag */
int f_reward_preset = 60;				/* reward size preset value */
int f_reward_random = 10;				/* reward size random value */
int f_screen_distance_MM = 280;			/* screen distance in mm */

VLIST state_vl[] = {
"day_seed",		&f_seed, NP, NP, 0, ME_DEC,
"ntrials_each_type", &f_ntrials, NP, NP, 0, ME_DEC,
"blocksize", &f_nblocksize, NP, NP, 0, ME_DEC,
"num_stim_per_fixation", &f_nstim_per_fixation, NP, NP, 0, ME_DEC,
"reward_preset", &f_reward_preset, NP, NP, 0, ME_DEC,
"reward_random", &f_reward_random, NP, NP, 0, ME_DEC,
"screen_distance",	&f_screen_distance_MM,	NP,	NP,	0,	ME_DEC,
"verbose", &f_verbose, NP, NP, 0, ME_DEC,
NS,
};

char hm_sv_vl[] = "Trial types are set in flow field types menu and grid menu.\n";

/* 
 * Flow field types
 */
 
int f_fflinear_min_angle = 0;
int f_fflinear_max_angle = 180;
int f_fflinear_num_angle = 3;
int f_fflinear_ovr_angle = NULLI;
int f_ffradial_min_angle = 0;
int f_ffradial_max_angle = 90;
int f_ffradial_num_angle = 3;
int f_ffradial_ovr_angle = NULLI;
int f_ffnone_enable = 1;			/* enables blank as a ff type */

VLIST fftypes_vl[] = {
	"linear_min_angle", &f_fflinear_min_angle, NP, NP, 0, ME_DEC,
	"linear_max_angle", &f_fflinear_max_angle, NP, NP, 0, ME_DEC,
	"linear_num_angle", &f_fflinear_num_angle, NP, NP, 0, ME_DEC,
	"linear_override_angle", &f_fflinear_ovr_angle, NP, NP, 0, ME_NVALD,
	"radial_min_angle", &f_ffradial_min_angle, NP, NP, 0, ME_DEC,
	"radial_max_angle", &f_ffradial_max_angle, NP, NP, 0, ME_DEC,
	"radial_num_angle", &f_ffradial_num_angle, NP, NP, 0, ME_DEC,
	"radial_override_angle", &f_ffradial_ovr_angle, NP, NP, 0, ME_NVALD,
	"use_blank_ff", &f_ffnone_enable, NP, NP, 0, ME_DEC,
	NS,
};



char hm_fftypes[] = "All angles in degrees (integer values only)\n"
					"Setting \"use_blank_ff\" to nonzero value adds an\n"
					"empty flow field as a flow field type.\n";

float f_grid_center[2] = {0.0f, 0.0f};
int f_grid_nx = 3;
int f_grid_ny = 3;
float f_grid_spacing = 3.0f;

VLIST grid_vl[] = {
	"grid_center_x", &f_grid_center[0], NP, NP, 0, ME_FLOAT,
	"grid_center_y", &f_grid_center[1], NP, NP, 0, ME_FLOAT,
	"grid_num_x", &f_grid_nx, NP, NP, 0, ME_DEC,
	"grid_num_y", &f_grid_ny, NP, NP, 0, ME_DEC,
	"grid_spacing", &f_grid_spacing, NP, NP, 0, ME_FLOAT,
	NS,
};	

char hm_grid[] = "center (x,y) and spacing in degrees.\n"
				 "num_x is number of grid positions in x dimension.\n"
				 "num_y is number of grid positions in y dimension.\n"; 

/* 
 * flow field specs not covered in fftypes
 */
 
float f_ff_speed = 3.0f;		
float f_ff_width = 0.0f;
float f_ff_radius = 2.0f;
int f_ff_npts = 20;
float f_ff_dotsize = 2.0;
int f_ff_color[3] = {255, 255, 255};

VLIST ffspecs_vl[] = {
	"radius", &f_ff_radius, NP, NP, 0, ME_FLOAT, 
	"flow_speed", &f_ff_speed, NP, NP, 0, ME_FLOAT,
	"ndots", &f_ff_npts, NP, NP, 0, ME_DEC, 
	"dotsize", &f_ff_dotsize, NP, NP, 0, ME_FLOAT,
	"width", &f_ff_width, NP, NP, 0, ME_FLOAT,
	"dot_red", &f_ff_color[0], NP, NP, 0, ME_DEC,
	"dot_green", &f_ff_color[1], NP, NP, 0, ME_DEC,
	"dot_blue", &f_ff_color[2], NP, NP, 0, ME_DEC,
	NS,
};

char hm_ffspecs[] = "radius in degrees.\n"
				  "flow speed in degrees/sec\n"
				  "dotsize in pixels\n"
				  "radius, width in degrees.\n"
				  "red, green, blue in [0, 255]\n";

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
float f_fixpt_pos[2] = {0, 0};

VLIST fixpt_vl[] = {
"fixpt_x(deg)", &f_fixpt_pos[0], NP, NP, 0, ME_FLOAT,
"fixpt_y(deg)", &f_fixpt_pos[1], NP, NP, 0, ME_FLOAT,
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
int f_stimulus_time = 250;			/* time for each stimulus */

VLIST timing_vl[] = {
"trial_init_pause", &f_trial_init_pause_time, NP, NP, 0, ME_DEC,
"acq_time(ms)", &f_acq_time, NP, NP, 0, ME_DEC,
"acq_timeout(ms)", &f_acq_fail_time, NP, NP, 0, ME_DEC,
"fix_time(ms)", &f_fixation_time, NP, NP, 0, ME_DEC,
"fix_timeout(ms)", &f_acq_noise_time, NP, NP, 0, ME_DEC,
"stimulus_time(ms)", &f_stimulus_time, NP, NP, 0, ME_DEC,
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
{"FFTypes", &fftypes_vl, NP, NP, 0, NP, hm_fftypes},
{"FFSpecs", &ffspecs_vl, NP, NP, 0, NP, hm_ffspecs},
{"Grid", &grid_vl, NP, NP, 0, NP, hm_grid},
{"separator", NP}, 
{"timing", &timing_vl, NP, NP, 0, NP, hm_timing}, 
{"fixation", &fixpt_vl, NP, NP, 0, NP, hm_fixpt}, 
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

#define CH_ID			1
#define CH_IX			2
#define CH_IY			3
#define CH_X			4
#define CH_Y			5
#define CH_LINEAR		6
#define CH_ANGLE		7
 
/*
 * bcode ids
 */

#define CH_DAY_SEED					91
#define CH_NTRIALS_EACH_TYPE		92
#define CH_BLOCKSIZE				93
#define CH_NUM_STIM_PER_FIXATION	94
#define CH_REWARD_PRESET			95
#define CH_REWARD_RANDOM			96

#define CH_FFLINEAR_MIN_ANGLE		101
#define CH_FFLINEAR_MAX_ANGLE		102
#define CH_FFLINEAR_NUM_ANGLE		103
#define CH_FFLINEAR_OVR_ANGLE		104
#define CH_FFRADIAL_MIN_ANGLE		105
#define CH_FFRADIAL_MAX_ANGLE		106
#define CH_FFRADIAL_NUM_ANGLE		107
#define CH_FFRADIAL_OVR_ANGLE		108
#define CH_FFNONE_ENABLE			109

#define CH_GRID_CENTER_X			110
#define CH_GRID_CENTER_Y			111
#define CH_GRID_NUM_X				112
#define CH_GRID_NUM_Y				113
#define CH_GRID_SPACING				114

#define CH_FF_RADIUS				115
#define CH_FF_SPEED					116
#define CH_FF_NPTS					117
#define CH_FF_DOTSIZE				118
#define CH_FF_WIDTH					119
#define CH_FF_RED					120
#define CH_FF_BLUE					121
#define CH_FF_GREEN					122

#define CH_BKGD_R					123
#define CH_BKGD_G					124
#define CH_BKGD_B					125

#define CH_FIXPT_X					126
#define CH_FIXPT_Y					127
#define CH_FIXPT_DIAMETER			128
#define CH_FIXPT_WINDOW				129
#define CH_FIXPT_R					130
#define CH_FIXPT_G					131
#define CH_FIXPT_B					132
 
#define CH_TRIAL_INIT_PAUSE_TIME	133
#define CH_ACQ_TIME					134
#define CH_ACQ_FAIL_TIME			135
#define CH_FIXATION_TIME			136
#define CH_ACQ_NOISE_TIME			137
#define CH_INTERTRIAL_TIME			138
#define CH_STIMULUS_TIME			139
#define CH_FRAMES_PER_SECOND		140


void my_bcodes()
{
	bcode_int(CH_DAY_SEED, f_seed);
	bcode_int(CH_NTRIALS_EACH_TYPE, f_ntrials);
	bcode_int(CH_BLOCKSIZE, f_nblocksize);
	bcode_int(CH_NUM_STIM_PER_FIXATION, f_nstim_per_fixation);
	bcode_int(CH_REWARD_PRESET, f_reward_preset);
	bcode_int(CH_REWARD_RANDOM, f_reward_random);

	bcode_int(CH_FFLINEAR_MIN_ANGLE, f_fflinear_min_angle);		
	bcode_int(CH_FFLINEAR_MAX_ANGLE, f_fflinear_max_angle);
	bcode_int(CH_FFLINEAR_NUM_ANGLE, f_fflinear_num_angle);
	bcode_int(CH_FFLINEAR_OVR_ANGLE, f_fflinear_ovr_angle);
	bcode_int(CH_FFRADIAL_MIN_ANGLE, f_ffradial_min_angle);
	bcode_int(CH_FFRADIAL_MAX_ANGLE, f_ffradial_max_angle);
	bcode_int(CH_FFRADIAL_NUM_ANGLE, f_ffradial_num_angle);
	bcode_int(CH_FFRADIAL_OVR_ANGLE, f_ffradial_ovr_angle);
	bcode_int(CH_FFNONE_ENABLE, f_ffnone_enable);
	
	bcode_float(CH_GRID_CENTER_X, f_grid_center[0]);
	bcode_float(CH_GRID_CENTER_Y, f_grid_center[1]);
	bcode_int(CH_GRID_NUM_X, f_grid_ny);
	bcode_int(CH_GRID_NUM_Y, f_grid_nx);
	bcode_float(CH_GRID_SPACING, f_grid_spacing);
	
	bcode_float(CH_FF_RADIUS, f_ff_radius);
	bcode_float(CH_FF_SPEED, f_ff_speed);
	bcode_int(CH_FF_NPTS, f_ff_npts);
	bcode_float(CH_FF_DOTSIZE, f_ff_dotsize);
	bcode_float(CH_FF_WIDTH, f_ff_width);
	bcode_int(CH_FF_RED, f_ff_color[0]);
	bcode_int(CH_FF_GREEN, f_ff_color[1]);
	bcode_int(CH_FF_BLUE, f_ff_color[2]);
	
	bcode_int(CH_BKGD_R, f_background_color[0]);
	bcode_int(CH_BKGD_G, f_background_color[1]);
	bcode_int(CH_BKGD_B, f_background_color[2]);
	bcode_float(CH_FIXPT_X, f_fixpt_pos[0]);
	bcode_float(CH_FIXPT_Y, f_fixpt_pos[1]);
	bcode_float(CH_FIXPT_DIAMETER, f_fixpt_diameter);
	bcode_float(CH_FIXPT_WINDOW, f_fixpt_window);
	bcode_int(CH_FIXPT_R, f_fixpt_color[0]);
	bcode_int(CH_FIXPT_G, f_fixpt_color[1]);
	bcode_int(CH_FIXPT_B, f_fixpt_color[2]);
	bcode_int(CH_TRIAL_INIT_PAUSE_TIME, f_trial_init_pause_time);
	bcode_int(CH_ACQ_TIME, f_acq_time);
	bcode_int(CH_ACQ_FAIL_TIME, f_acq_fail_time);
	bcode_int(CH_FIXATION_TIME, f_fixation_time);
	bcode_int(CH_ACQ_NOISE_TIME, f_acq_noise_time);
	bcode_int(CH_INTERTRIAL_TIME, f_intertrial_time);
	bcode_int(CH_STIMULUS_TIME, f_stimulus_time);
	bcode_int(CH_FRAMES_PER_SECOND, f_frames_per_second);
}



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
	status = init_tcpip(f_local_addr, f_remote_addr, f_remote_port, 1);

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
	int i;

	dprintf("Initializing...\n");

	/* 
	 * Dump bcodes to efile. These are all the menu parameters. 
	 */
	 
	my_bcodes();
	
	/*
	 * seed random number generator if necessary
	 */

	if (!f_seed_used)
	{
		dprintf("Set random number seed.\n");
		ivunif(f_seed, 1);
		f_seed_used = 1;
	}


	/*
	 * Now initialize render objects et al.
	 */
	 
	dprintf("Initialize render graphic objects.");

	f_fixpt_handle = 0;
	f_handle_count = 0;

	
	/*
	 *  background color
	 */

	render_bgcolor(f_background_color[0], f_background_color[1], f_background_color[2]);

	/* 
	 * Fixation point 
	 */
	
	f_fixpt.xorigin = to_pixels(f_fixpt_pos[0]);
	f_fixpt.yorigin = to_pixels(f_fixpt_pos[1]);
	f_fixpt.xsize = to_pixels(f_fixpt_diameter);
	f_fixpt.ysize = to_pixels(f_fixpt_diameter);
	f_fixpt.depth = 1;
	f_fixpt.r = f_fixpt_color[0];
	f_fixpt.g = f_fixpt_color[1];
	f_fixpt.b = f_fixpt_color[2];
	render_dot(&f_fixpt);

	/* 
	 * Configure FF2Ds.
	 * If f_pFF2D array is non-null, free it and regenerate.
	 * Initialize each FF2D to dummy values (they'll all get updated on each trial).
	 * Issue the commands -- handles will get caught in my_check_for_handle.  
	 */
	
	dprintf("Configuring %d FF2Ds\n", f_nstim_per_fixation);
	if (f_pFF2D) free(f_pFF2D);
	if (f_pHandle) free(f_pHandle);
	f_pFF2D = (FF2DStruct *)calloc(f_nstim_per_fixation, sizeof(FF2DStruct));
	memset(f_pFF2D, 0, f_nstim_per_fixation * sizeof(FF2DStruct));
	f_pHandle = (int *)calloc(f_nstim_per_fixation, sizeof(int));
	memset(f_pHandle, 0, f_nstim_per_fixation * sizeof(int));

	for (i=0; i<f_nstim_per_fixation; i++)
	{
		f_pFF2D[i].linear = 1;
		f_pFF2D[i].npts = f_ff_npts;
		f_pFF2D[i].prob = 1.0;
		f_pFF2D[i].radius = to_pixels(f_ff_radius);
		f_pFF2D[i].pseed = 1;
		f_pFF2D[i].cseed = 1;
		f_pFF2D[i].pixsz = 1;
		f_pFF2D[i].depth = 1;
		f_pFF2D[i].x = 0;
		f_pFF2D[i].y = 0;
		f_pFF2D[i].v = 1;
		f_pFF2D[i].width = 0;
		f_pFF2D[i].angle = 0;
		f_pFF2D[i].r = 255;
		f_pFF2D[i].g = 255;
		f_pFF2D[i].b = 255;
		f_pFF2D[i].a = 0;
		render_ff2d(f_pFF2D + i);
	}
	
	return status;
}


/* 
 * my_check_for_handle 
 * 
 * Called as an escape function ... e.g. 
 * 
 * to wherever on 1 % my_check_for_handle
 * 
 * This function will fetch handles from render. 
 * The first handle is saved as f_fixpt_handle.
 * All subsequent handles are saved in f_pHandle[] -- these are the FF2D handles.
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
 * Returns 0 if the handle counter is less than 1 + f_nstim_per_fixation, or 1 if the handle 
 * counter == 1+f_nstim_per_fixation..
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
			dprintf("Fixpt handle = %d\n", f_fixpt_handle); 
		}
		else
		{
			f_pHandle[f_handle_count-1] = handle;
			dprintf("FF2D handle[%d] = %d\n", f_handle_count-1, f_pHandle[f_handle_count-1]);
			f_handle_count++;
			if (f_handle_count == 1+f_nstim_per_fixation) status = 1;
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
	int nconditions_grid;
	int nconditions_ff2d;
	int count = 0;
	float x, y;
	float angle_step;

	dprintf("Initializing experimental variables.\n");

	f_all_done = 0;
	f_trialsStarted = 0;
	f_trialsCompleted = 0;

	/* Free experimental condition struct array if it exists */
	if (f_pexptcond)
	{
		dprintf("Cleanup existing experimental condition list...\n");
		free(f_pexptcond);
	}

	/*
	 * Figure out how many conditions and allocate the array 
	 */
	 
	nconditions_grid = f_grid_nx * f_grid_ny;
	nconditions_ff2d = 	(f_fflinear_ovr_angle == NULLI ? f_fflinear_num_angle : 1) + 
						(f_ffradial_ovr_angle == NULLI ? f_ffradial_num_angle : 1) +
						(f_ffnone_enable ? 1 : 0);
	f_nexptcond = nconditions_ff2d * nconditions_grid; 
	
	dprintf("nconditions_grid = %d nconditions_ff2d=%d, expecting %d\n",nconditions_grid, nconditions_ff2d, f_nexptcond); 
	
	f_pexptcond = (ExptCondition *)calloc(f_nexptcond, sizeof(ExptCondition));
	memset(f_pexptcond, 0, f_nexptcond * sizeof(ExptCondition));

	for (i=0; i<f_grid_nx; i++)
	{
		x = f_grid_center[0] + 0.5 * f_grid_spacing * (1 + 2*i - f_grid_nx);
		for (j=0; j<f_grid_ny; j++)
		{
			y = f_grid_center[1] + 0.5 * f_grid_spacing * (1 + 2*j - f_grid_ny);

			/*
			 * First, linear FF2Ds
			 */
			if (f_fflinear_ovr_angle == NULLI)
			{
				angle_step = (f_fflinear_max_angle - f_fflinear_min_angle)/(f_fflinear_num_angle-1);
				for (k=0; k<f_fflinear_num_angle; k++)
				{
					f_pexptcond[count].id = count;
					f_pexptcond[count].ix = i;
					f_pexptcond[count].iy = j;
					f_pexptcond[count].x = x;
					f_pexptcond[count].y = y;
					f_pexptcond[count].ilinear = 1;
					f_pexptcond[count].angle = f_fflinear_min_angle + k*angle_step;
					count++;
				}
			}
			else
			{
				f_pexptcond[count].id = count;
				f_pexptcond[count].ix = i;
				f_pexptcond[count].iy = j;
				f_pexptcond[count].x = x;
				f_pexptcond[count].y = y;
				f_pexptcond[count].ilinear = 1;
				f_pexptcond[count].angle = f_fflinear_ovr_angle;
				count++;
			}
			
			/*
			 * Now radial FF2Ds
			 */
			if (f_ffradial_ovr_angle == NULLI)
			{
				angle_step = (f_ffradial_max_angle - f_ffradial_min_angle)/(f_ffradial_num_angle-1);
				for (k=0; k<f_ffradial_num_angle; k++)
				{
					f_pexptcond[count].id = count;
					f_pexptcond[count].ix = i;
					f_pexptcond[count].iy = j;
					f_pexptcond[count].x = x;
					f_pexptcond[count].y = y;
					f_pexptcond[count].ilinear = 0;
					f_pexptcond[count].angle = f_ffradial_min_angle + k*angle_step;
					count++;
				}
			}
			else
			{
				f_pexptcond[count].id = count;
				f_pexptcond[count].ix = i;
				f_pexptcond[count].iy = j;
				f_pexptcond[count].x = x;
				f_pexptcond[count].y = y;
				f_pexptcond[count].ilinear = 0;
				f_pexptcond[count].angle = f_ffradial_ovr_angle;
				count++;
			}
			 
			/*
			 * Now a blank if needed 
			 */
			if (f_ffnone_enable != 0)
			{
				f_pexptcond[count].id = count;
				f_pexptcond[count].ix = i;
				f_pexptcond[count].iy = j;
				f_pexptcond[count].x = x;
				f_pexptcond[count].y = y;
				f_pexptcond[count].ilinear = -1;
				f_pexptcond[count].angle = 0;
				count++;
			}
		}
	}	
	
	if (f_nexptcond != count)
	{
		dprintf("WARNING! Error in enumarating experimental conditions! (expecting %d, got %d). Expect disaster!\n", f_nexptcond, count);
	}


	if (f_verbose & DEBUG_INITIALIZATION_BIT)
	{
		dprintf( "ID \tX posn\tY posn\tFFtype\tAngle\n");
		for (i=0; i<count; i++)
		{
			dprintf( "%d\t%d(%d)\t%d(%d)\t%d\t%d\n", 
					f_pexptcond[i].id,
					f_pexptcond[i].ix, (int)(f_pexptcond[i].x*10), 
					f_pexptcond[i].iy, (int)(f_pexptcond[i].y*10), 
					f_pexptcond[i].ilinear, (int)(f_pexptcond[i].angle*10));
		}
		

	}
	dprintf("Count is %d\n", count);

	
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
	f_prtgen = rtlistg_create(f_nexptcond, f_ntrials, f_nblocksize, f_nstim_per_fixation);
	
	/*
	 * make sure screen is clear and render
	 */
	 
	alloff();
	render_frame(0);


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

	f_trialsStarted++;	
	f_frametag = f_trialsStarted*FRAMETAG_MULTIPLIER;
	
	if (f_verbose & DEBUG_TRIAL_BIT)
	{
		dprintf("Initializing trial %d.\n", f_trialsStarted);
	}
	
	/*
	 * Initialize eye window. This should be done after we decide where the fixpt will be.
	 */
	 
	my_eye_window(EYE_WINDOW_INIT | EYE_WINDOW_ON);
	
	/* 
	 * Check for fixpt condition to apply to this trial.If the index
	 * returned is -1 it means all trials have been completed.  
	 */
	 
	status = f_prtgen->list(f_prtgen, &f_pTrialList);
	if (!status)
	{
		dprintf("All trials completed.\n");
		f_all_done = 1;
	}
	else
	{
		
		/* 
		 * Update the FF2Ds for this trial
		 */
		 
		if (f_verbose & DEBUG_TRIAL_BIT)
		{
			dprintf("Updating FF2Ds for this trial\n");
			dprintf("ID \tX posn\tY posn\tFFtype\tAngle\n");
		}
		for (i=0; i<f_nstim_per_fixation; i++)
		{

			if (f_verbose & DEBUG_TRIAL_BIT)
			{
				dprintf( "%d\t%d(%d)\t%d(%d)\t%d\t%d\n", 
						f_pexptcond[f_pTrialList[i]].id,
						f_pexptcond[f_pTrialList[i]].ix, (int)(f_pexptcond[f_pTrialList[i]].x*10), 
						f_pexptcond[f_pTrialList[i]].iy, (int)(f_pexptcond[f_pTrialList[i]].y*10), 
						f_pexptcond[f_pTrialList[i]].ilinear, (int)(f_pexptcond[f_pTrialList[i]].angle*10));
			}

			f_pFF2D[i].linear = f_pexptcond[f_pTrialList[i]].ilinear;
			f_pFF2D[i].x = to_pixels(f_pexptcond[f_pTrialList[i]].x);
			f_pFF2D[i].y = to_pixels(f_pexptcond[f_pTrialList[i]].y);
			f_pFF2D[i].angle = f_pexptcond[f_pTrialList[i]].angle;
			render_update(f_pHandle[i], &f_pFF2D[i], sizeof(FF2DStruct), HANDLE_OFF);
		}

		/*
		 * Reset update counter (frame counter) and compute frame numbers for the transitions.
		 */
		 
		f_trial_frame_counter = 0;
		f_trial_frames_per_stim = (int)((float)f_stimulus_time*f_frames_per_second/1000.0f);
		if (f_verbose & DEBUG_TRIAL_BIT)
		{
			dprintf("Stim time is %d frames for each (%d) stim.\n", f_trial_frames_per_stim, f_nstim_per_fixation);
		}
		
		 
		/* 
		 * Assign time values to states 
		 */
		 
		set_times("trial_init_pause", (long)f_trial_init_pause_time, -1); 
		set_times("fixpt_acq", (long)f_acq_time, -1);
		set_times("fixpt_acq_fail_pause", (long)f_acq_fail_time, -1);
		set_times("fixpt_hold", (long)f_fixation_time, -1);
		set_times("fixpt_acq_noise_pause", (long)f_acq_noise_time, -1);
		set_times("intertrial_pause", (long)f_intertrial_time, -1);
		set_times("reward_on", (long)f_reward_preset, (long)f_reward_random);
		if (f_verbose & DEBUG_TRIAL_BIT)
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

	}

	if (f_verbose & DEBUG_TRIAL_BIT)
	{
		dprintf("Sending frame...\n");
	}
	render_frame(0);	
	if (f_verbose & DEBUG_TRIAL_BIT)
	{
		dprintf("Sending frame...done.\n");
	}

	return 0;
}



/* 
 * my_update
 * 
 * This function is called when fixation is achieved and is only responsible for making sure the 
 * correct FF2D object is on screen. To do this we use f_trial_frame_counter (this is zeroed out in
 * my_trial_init and incremented in my_check_for_went upon receipt of the WENT).
 */

int my_update()
{
	int status = 0;
	int istim = 0;

	if (f_trial_frame_counter%f_trial_frames_per_stim == 0)	
	{
		istim = f_trial_frame_counter / f_trial_frames_per_stim;


		/*
		 * If istim > 0, it means that there is an FF2D currently on screen. Turn it off.
		 * Better check that its not a blank - in that case its not on the screen. 
		 */
		 
		if (istim>0)
		{
			if (f_verbose & DEBUG_UPDATE_BIT)
			{
				dprintf("my_update: Turn off stim handle[%d]=%d, index %d\n", istim-1, f_pHandle[istim-1], f_pTrialList[istim-1]);
			}
			if (f_pexptcond[f_pTrialList[istim-1]].ilinear>-1)
			{
				render_onoff(&f_pHandle[istim-1], HANDLE_OFF, ONOFF_NO_FRAME);
			}
			
			/*
			 * Drop ecode indicating that stim is off
			 */
			 
			ecode(FFOFFCD);
		}
		
		/*
		 * Turn on the next FF2D unless the last one is on screen now.
		 */
		 
		if (istim < f_nstim_per_fixation)
		{
			if (f_verbose & DEBUG_UPDATE_BIT)
			{
				dprintf("my_update: Turn on stim handle[%d]=%d, index %d\n", istim, f_pHandle[istim], f_pTrialList[istim]);
			}
			if (f_pexptcond[f_pTrialList[istim]].ilinear>-1)
			{
				render_onoff(&f_pHandle[istim], HANDLE_ON, ONOFF_NO_FRAME);
			}
			
			/* 
			 * Put bcodes into efile 
			 */
			 
			bcode_int(CH_ID, f_pexptcond[f_pTrialList[istim]].id);
			bcode_int(CH_IX, f_pexptcond[f_pTrialList[istim]].ix);
			bcode_int(CH_IY, f_pexptcond[f_pTrialList[istim]].iy);
			bcode_float(CH_X, f_pexptcond[f_pTrialList[istim]].x);
			bcode_float(CH_Y, f_pexptcond[f_pTrialList[istim]].y);
			bcode_int(CH_LINEAR, f_pexptcond[f_pTrialList[istim]].ilinear);
			bcode_float(CH_ANGLE, f_pexptcond[f_pTrialList[istim]].angle);
		}
		 
	}

	render_frame_tagged(f_frametag);

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
	f_went_status = render_check_for_went(&frames);
	if (f_went_status < 0)
	{
		// ERROR! not clear what to do ...
		dprintf("ERROR in render_check_for_went!\n");
	}
	else if (f_went_status == 0)
	{
		f_went_cycles++;
	}
	else if (f_went_status == 1)
	{
		/*
		 * If frames were missed, dprint the number missed and the cycles and drop an ecode.
		 * This paradigm has a forced delay prior to the start of presentation. This always results in 
		 * APPARENT missed frames - its not really so. If f_trial_frame_counter is 0 ignore a nonzero
		 * value for missed frames. 
		 */ 
		 
		if (frames > 1 && f_trial_frame_counter>0)
		{
			dprintf("Missed %d frames (%d check_went cycles)\n", frames, f_went_cycles);
			ecode(MISSEDCD);
		}
		f_went_cycles = 0;
		
		/*
		 * A WENT was received. Record a WENT ecode and the fixation point x and y positions. 
		 */
		 
		f_trial_frame_counter++;
		if (f_trial_frame_counter == (f_nstim_per_fixation*f_trial_frames_per_stim + 1))
		{
			f_went_status = 2;
		}
		ecode(WENTCD);
		
#if 0
		bcode_float(CH_FIXPT_X, to_degrees(f_fixpt.xorigin));
		bcode_float(CH_FIXPT_Y, to_degrees(f_fixpt.yorigin));
#endif
				

	}
	return f_went_status;
}




int my_check_for_went_extended()
{
	int status = 0;
	int frames = 0;
	unsigned int framenumber=0;
	unsigned int frametag=0;
	status = render_check_for_went_extended(&frames, &framenumber, &frametag);
	if (status < 0)
	{
		// ERROR! not clear what to do ...
		dprintf("ERROR in render_check_for_went!\n");
		f_went_status = status;
	}
	else if (status == 0)
	{
		f_went_cycles++;
		f_went_status = 0;
	}
	else if (status >0)
	{
		ecode(WENTCD);		
		f_went_status = 1;

		/*
		 * If frames were missed, dprint the number missed and the cycles and drop an ecode.
		 * This paradigm has a forced delay prior to the start of presentation. This always results in 
		 * APPARENT missed frames - its not really so. If f_trial_frame_counter is 0 ignore a nonzero
		 * value for missed frames. 
		 */ 
		 
		if (frames > 1 && f_trial_frame_counter>0)
		{
			dprintf("Missed %d frames (%d check_went cycles)\n", frames, f_went_cycles);
			ecode(MISSEDCD);
		}
		f_went_cycles = 0;

		/*
		 * Check the tag to make sure we haven't skipped over any WENTs 
		 */
		
		if (frametag != f_frametag)
		{
			dprintf("ERROR: SENT/GOT FRAMETAG %d/%d\n", (int)f_frametag, (int)frametag);
		}

		/*
		 * Increment inter-trial counters and check if the trial is done.
		 */
		 		
		f_frametag++;
		f_trial_frame_counter++;
		if (f_trial_frame_counter == (f_nstim_per_fixation*f_trial_frames_per_stim + 1))
		{
			f_went_status = 2;
		}
		
	}
	return f_went_status;
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
			f_prtgen->marklist(f_prtgen, f_nstim_per_fixation, f_pTrialList);
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
	int i;
	render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	for (i=0; i<f_nstim_per_fixation; i++)
	{
		render_onoff(&f_pHandle[i], HANDLE_OFF, ONOFF_NO_FRAME);
	}
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

id 503
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
		to fixpt_acq on 1 % render_check_for_went
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
		to update
	update:
		do my_update()
		to update_wait
	update_wait:
		to fixation_fail_wait on +WD0_XY & eyeflag
		to update on 1 % my_check_for_went_extended
		to trial_success on 2 = f_went_status
	trial_success:
		do my_trial_done(TRIAL_DONE_SUCCESS)
		to reward_on on 1 % render_check_for_went
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
		code TRLEND
		time 500
		to trial_init
	pause_detected_wait:
		to pause_detected on 1 % render_check_for_went
	pause_detected:
		code PAUSECD
		do my_trial_done(TRIAL_DONE_PAUSE)
		to pause_wait on 1 % render_check_for_went
	pause_wait:
		to intertrial_pause on -PSTOP & softswitch
	all_done_wait:
		to all_done on 1 % render_check_for_went
	all_done:
		do all_trials_done()
		to first on 1 = f_never
/*abort	list:
		fixation_fail*/
}



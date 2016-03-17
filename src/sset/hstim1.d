/* $Id: hstim1.d,v 1.16 2009/11/25 01:02:24 devel Exp $ */

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
#include "segtree.h"			/* segment tree routines */
#include "slider.h"
#include "randomtrialgenerator.h"
#include "animhelper.h"
 
 
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
 * These are eye window identifiers.
 */

#define EYE_WINDOW_FIXPT    	0
#define EYE_WINDOW_TARGET_CORRECT	1
#define EYE_WINDOW_TARGET_INCORRECT	2
#define EYEFLAG_FIXPT	WD0_XY
#define EYEFLAG_CORRECT WD1_XY
#define EYEFLAG_INCORRECT WD2_XY


/* 
 * These are values which can be passed to my_eye_window (they may be OR'd together).  
 */

#define EYE_WINDOW_INIT				0x1
#define EYE_WINDOW_FIXPT_UPDATE		0x2
#define EYE_WINDOW_FIXPT_ON			0x4
#define EYE_WINDOW_FIXPT_OFF		0x8
#define EYE_WINDOW_TARGETS_UPDATE	0x20
#define EYE_WINDOW_TARGETS_ON		0x40
#define EYE_WINDOW_TARGETS_OFF		0x80
#define EYE_WINDOW_ALL_OFF			(EYE_WINDOW_FIXPT_OFF | EYE_WINDOW_TARGETS_OFF)

/* 
 * ustim flag. This is set to tell the ustim loop to turn on 
 * the microstim. It is turned off by a time setting, or by 
 * a pause in the paradigm.
 */

int f_ustim_flag=0;
int f_ustim_ready_flag=0;


/*
 * These are arg values for my_trial_done. 
 */
 
#define TRIAL_DONE_INCORRECT 0
#define TRIAL_DONE_CORRECT 1
#define TRIAL_DONE_NOANSWER -1
#define TRIAL_DONE_BREAKFIX -2
#define TRIAL_DONE_PAUSE -3

/*
 * Eye speed conversion. In the paradigm from old render days, the speed was entered as a number from 0-100. 
 * That number was considered a percentage, and the eye would "travel" that percentage of 1000 units. In other
 * words, convert the eye speed in "percent" to eye speed in arbitrary 3D units by multiplying by this number....
 */
 
#define EYE_SPEED_TO_3D_UNITS	10


char f_charbuf[2048];			/* Really really big char for sprinting errors. */
int f_never = -1;				/* never var, quoth the raven */
int f_seed_used = 0;			/* indicates whether we've seeded the random number generator or not */
int f_fixpt_handle = 0;			/* graphics handle for fixation point */
int f_dots_handle = 0;			/* graphics handle for dot cloud */
int f_target_left_handle = 0;	/* graphics handle for left target */
int f_target_right_handle = 0;	/* graphics handle for right target */
int f_handle_count = 0;			/* graphics handle counter */
float f_cam_position[3] = {0, 0, 0};		/* Initial camera position */
float f_cam_looking[3];			/* where the camera is aimed */
float f_cam_up[3];				/* up direction for camera */
int f_trial_condition_index;	/* index into f_pextcond array for current trial */
int f_all_done = 0;				/* flag indicating all trials have been completed */
RexAnimHelper *f_panim_path = NULL;	/* Animation helper for camera path (dot motion) */
RexAnimHelper *f_panim_pursuit = NULL;	/* animation helper for moveobject (pursuit) */
int f_step_counter = 0;			/* step counter for animations */
int f_wstatus = 0;				/* status indicator used by my_check_for_went */
int f_animation_done = 0;		/* indicator for animation loop */
int f_went_cycles = 0;
DotFieldStruct f_dotfield;
DotStruct f_fixpt;
DotStruct f_target_left;
DotStruct f_target_right;
DotStruct *f_ptarget_correct = NULL;
DotStruct *f_ptarget_incorrect = NULL;
MoveObjectStruct f_moveobject;	/* animation struct for fixpt pursuit */
PathStruct f_path;				/* animation struct for camera motion */
CameraStruct f_camera;
int f_frames_pre_pursuit;		/* frames before pursuit begins */
int f_frames_pre_dots_motion;	/* frames after pursuit begins, before dot motion */
int f_frames_dots_motion;		/* frames of dot motion - rounded to even number */
int f_frames_dots_motion_midpoint;
int f_dots_motion_ready_flag;	/* flag set when first dot motion frame is imminent */
struct condition_struct
{
	int pursuit_sgn;		/* -1, 0, +1 */
	int angle_index;		/* [-f_nangles, f_nangles] */
	float angle_degrees;
	int ustim;				/* 0/1 */
	int show_dots;				/* 0/1 - 1 means that dots are displayed this trial type */
};

/*
 * show_dots flag
 * This flag is set when we want to show the dots (duh). Conversely, when the
 * show_dots flag is 0 we will NOT display the dots -- this implements the 
 * "pursuit_control" condition. The "pursuit control condition" is a test condition
 * where there are no dots or dot motion -- only pursuit. Thus the fixation point
 * is present, and pursuit happens, but the dots should NOT appear. We will set
 * this flag in my_trial_init  - it is used in the state set (see the transitions
 * from state "fixation")
 */
 
int f_show_dots_flag = 0;


typedef struct condition_struct ExptCondition;
ExptCondition* f_pexptcond = NULL;
int f_nexptcond = 0;

RTGenerator *f_prtgen = NULL;
int f_trialsCompleted = 0;




void print_dot(DotStruct *pdot);
void init_steering(void);
int my_render_init();
int my_check_for_handle();
int my_exp_init();
int my_trial_init();
int my_animate();
int my_check_for_went();
int my_trial_done(int icorrect);
void show_trials_status();
int alloff();
int my_ustim(int iustim);
int my_reward(int ireward);
int my_eye_window(int iflag);
int answer(int icorrect);
int fixpt_onoff(int onoff);
int dots_onoff(int onoff);
float get_eye_travel_distance();
void get_dotfield_extents(float *pxmax, float *pymax, float *pzmax);



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
float f_pursuit_speed = 10.0;			/* degrees/sec; <=0 means no pursuit */
int f_use_pursuit_control_condition = 0;/* if 1 then include no dots condition */
int f_use_heading_0_condition = 0;		/* if 1 then we include 0 heading with dots */
int f_use_ustim = 0;						/* if 1 then use ustim for this expt */
int f_eye_speed = 30;					/* measured in 10's/sec */
int f_verbose = 0;						/* debugging flag */
int f_reward_preset = 60;				/* reward size preset value */
int f_reward_random = 10;				/* reward size random value */
int f_useless_variable = 0;

VLIST state_vl[] = {
"day_seed",		&f_seed, NP, NP, 0, ME_DEC,
"#_of_trials/condition", &f_ntrials, NP, NP, 0, ME_DEC,
"block_size", &f_nblocksize, NP, NP, 0, ME_DEC,
"#_of_angles>0", &f_nangles, NP, NP, 0, ME_DEC,
"minimum_angle", &f_minangle, NP, NP, 0, ME_FLOAT,
"maximum_angle", &f_maxangle, NP, NP, 0, ME_FLOAT,
"pursuit_speed(deg/s)", &f_pursuit_speed, NP, NP, 0, ME_FLOAT,
"pursuit_control?", &f_use_pursuit_control_condition, NP, NP, 0, ME_DEC,
"heading_0(w/dots)?", &f_use_heading_0_condition, NP, NP, 0, ME_DEC,
"ustim(1/0)", &f_use_ustim, NP, NP, 0, ME_DEC,
"eye_speed", &f_eye_speed, NP, NP, 0, ME_DEC,
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
 * Targets menu
 */
 
float f_target_diameter_degrees = 1;				/* diameter of fixpt (degrees) */
float f_target_offset_horizontal_degrees = 10;		/* offset of targets from center */
float f_target_offset_vertical_degrees = 0;			/* offset of targets from center */
float f_target_window = 5;				/* size of target point window */
int f_target_color[3] = {255, 0, 0};		/* target color [0,255] */

VLIST target_vl[] = {
"X_target_offset(deg)", &f_target_offset_horizontal_degrees, NP, NP, 0, ME_FLOAT,
"Y_target_offset(deg)", &f_target_offset_vertical_degrees, NP, NP, 0, ME_FLOAT,
"target_diameter(deg)", &f_target_diameter_degrees, NP, NP, 0, ME_FLOAT,
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
int f_acq_time = 2000;				/* max time to acquire fixpt */
int f_acq_fail_time = 1000;			/* if acq fails, wait this long before restarting */
int f_fixation_time = 500;			/* how long until fixation */
int f_acq_noise_time = 250;			/* if acq fails due to noise, wait this long before restarting */
int f_intertrial_time = 250;		/* time between trials */
int f_pre_pursuit_time = 250;		/* time from dots on until pursuit starts */
int f_pre_dots_motion_time = 250;	/* time from pursuit start to dots motion */
int f_dots_motion_time = 1000;		/* time for dots motion */
int f_answer_time = 2000;			/* max time for answer */

VLIST timing_vl[] = {
"trial_init_pause", &f_trial_init_pause_time, NP, NP, 0, ME_DEC,
"acq_time(ms)", &f_acq_time, NP, NP, 0, ME_DEC,
"acq_timeout(ms)", &f_acq_fail_time, NP, NP, 0, ME_DEC,
"fix_time(ms)", &f_fixation_time, NP, NP, 0, ME_DEC,
"fix_timeout(ms)", &f_acq_noise_time, NP, NP, 0, ME_DEC,
"pre_pursuit_time", &f_pre_pursuit_time, NP, NP, 0, ME_DEC,
"pre_dots_motion_time", &f_pre_dots_motion_time, NP, NP, 0, ME_DEC,
"dots__motion_time", &f_dots_motion_time, NP, NP, 0, ME_DEC,
"intertrial_time(ms)", &f_intertrial_time, NP, NP, 0, ME_DEC,
"answer_time(ms)", &f_answer_time, NP, NP, 0, ME_DEC,
NS,
};

char hm_timing[] = "";

/*
 * dots. Note that camera travel at heading of 0 degrees is along the positive y axis!
 */
 
int f_dotfield_ndots = 400;			/* This is the density, measured in dots per 1000**3 */
float f_dotfield_pointsize = 2.0;
int f_dotfield_color[3] = {255, 255, 255};

VLIST dotfield_vl[] = {
"dots/1000**3", &f_dotfield_ndots, NP, NP, 0, ME_DEC,
"pointsize(pixels)", &f_dotfield_pointsize, NP, NP, 0, ME_FLOAT,
"dot_color(R)", &f_dotfield_color[0], 0, NP, 0, ME_DEC,
"dot_color(G)", &f_dotfield_color[1], 0, NP, 0, ME_DEC,
"dot_color(B)", &f_dotfield_color[2], 0, NP, 0, ME_DEC,
NS,
};

char hm_dotfield[] = "";

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
{"dots", &dotfield_vl, NP, NP, 0, NP, hm_dotfield}, 
{"targets", &target_vl, NP, NP, 0, NP, hm_target}, 
{"viewing volume", &vvol_vl, NP, NP, 0, NP, hm_vvol},
{"background", &background_vl, NP, NP, 0, NP, hm_background}, 
{"communication", &comm_vl, NP, NP, 0, NP, hm_comm}, 
{NS},
};


/*
 * Channel definitions for data stuck in E file
 */

#define CH_RANDOM_SEED 		100
#define CH_NTRIALS 			101
#define CH_NBLOCKSIZE		102
#define CH_NANGLES			103
#define CH_MAXANGLE			104
#define CH_MINANGLE			105
#define CH_PURSUIT_SPEED	106
#define CH_USE_USTIM		107
#define CH_EYE_SPEED		108
#define CH_REWARD_PRESET	109
#define CH_REWARD_RANDOM	110
#define CH_SCREEN_DISTANCE_MM	111
#define CH_FAR_PLANE_DISTANCE	112
#define CH_BKGD_R				120
#define CH_BKGD_G				121
#define CH_BKGD_B				122
#define CH_FIXPT_DIAMETER		123
#define CH_FIXPT_WINDOW			124
#define CH_FIXPT_R				125
#define CH_FIXPT_G				126
#define CH_FIXPT_B				127
#define CH_FIXPT_REFPOS_X		128
#define CH_FIXPT_REFPOS_Y		129
#define CH_TARGET_DIAMETER 130
#define CH_TARGET_OFFSET_H	131 
#define CH_TARGET_OFFSET_V	132 
#define CH_TARGET_WINDOW	133
#define CH_TARGET_R			134
#define CH_TARGET_G			135
#define CH_TARGET_B			136
#define CH_TRIAL_INIT_PAUSE_TIME 	137
#define CH_ACQ_TIME					138
#define CH_ACQ_FAIL_TIME			139
#define CH_FIXATION_TIME			140
#define CH_ACQ_NOISE_TIME			141
#define CH_INTERTRIAL_TIME			142
#define CH_PRE_PURSUIT_TIME			143
#define CH_PRE_DOTS_MOTION_TIME		144
#define CH_DOTS_MOTION_TIME			145
#define CH_ANSWER_TIME				146
#define CH_DOTFIELD_SIZE_X			147
#define CH_DOTFIELD_SIZE_Y			148
#define CH_DOTFIELD_SIZE_Z			149
#define CH_DOTFIELD_ORIGIN_X		150
#define CH_DOTFIELD_ORIGIN_Y		151
#define CH_DOTFIELD_ORIGIN_Z		152
#define CH_DOTFIELD_NDOTS			153
#define CH_DOTFIELD_POINTSIZE		154
#define CH_DOTFIELD_R				155
#define CH_DOTFIELD_G				156
#define CH_DOTFIELD_B				157
#define CH_FRAMES_PER_SECOND	158

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
 
/*
 * print_dot()
 * 
 * dprint the contents of the given DotStruct. For debugging.
 */
 
void print_dot(DotStruct *pdot)
{
	dprintf("Dot size (%dx%d) org (%d,%d) depth %d rgb (%d,%d,%d)\n", 
		(int)(pdot->xsize*10), (int)(pdot->ysize*10), (int)(pdot->xorigin), (int)(pdot->xorigin),
		pdot->depth, (int)pdot->r,(int)pdot->g, (int)pdot->b);  
	return;
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

	/* Setup init camera position */

	f_camera.ex = 0;
	f_camera.ey = 0;
	f_camera.ez = 0;
	f_camera.dx = 0;
	f_camera.dy = 0;
	f_camera.dz = 0;
	f_camera.ux = 0;
	f_camera.uy = 0;
	f_camera.uz = 1;

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
	
	/* 
	 * configure dotfield 
	 * 
	 * 8-6-2009 djs
	 * Compute the size needed for the dotfield based on the 
	 * eye speed, fovx and max heading angle. 
	 */

	get_dotfield_extents(&dotfield_xmax, &dotfield_ymax, &dotfield_zmax);	 

//	f_dotfield.xwidth = f_dotfield_size[0];
//	f_dotfield.ywidth = f_dotfield_size[1];
//	f_dotfield.zwidth = f_dotfield_size[2];
//	f_dotfield.xorigin = f_dotfield_origin[0];
//	f_dotfield.yorigin = f_dotfield_origin[1];
//	f_dotfield.zorigin = f_dotfield_origin[2];
	f_dotfield.xwidth = dotfield_xmax*2;
	f_dotfield.ywidth = dotfield_ymax;
	f_dotfield.zwidth = dotfield_zmax*2;
	f_dotfield.xorigin = 0;
	f_dotfield.yorigin = dotfield_ymax/2;
	f_dotfield.zorigin = 0;
//	f_dotfield.ndots = (uint32_t)((float)f_dotfield_ndots * (float)(f_dotfield.xwidth * f_dotfield.ywidth * f_dotfield.zwidth) / (1000.0f * 1000.0f * 1000.0f));
	f_dotfield.ndots = (uint32_t)((float)f_dotfield_ndots * (float)f_dotfield.xwidth/1000.0f * (float)f_dotfield.ywidth/1000.0f * (float)f_dotfield.zwidth/1000.0f);
	f_dotfield.pointsize = f_dotfield_pointsize;
	f_dotfield.r = f_dotfield_color[0];
	f_dotfield.g = f_dotfield_color[1];
	f_dotfield.b = f_dotfield_color[2];
	f_dotfield.a = 0;

	dprintf("dotfield ndots %u xmax*10, ymax*10, zmax*10 %d, %d, %d\n", f_dotfield.ndots, (int)(dotfield_xmax*10), (int)(dotfield_ymax*10), (int)(dotfield_zmax*10));
	
	render_dotfield(&f_dotfield);	

	/* Configure target points */
	
	f_target_left.xorigin = -to_pixels(f_target_offset_horizontal_degrees);
	f_target_left.yorigin = to_pixels(f_target_offset_vertical_degrees);
	f_target_left.xsize = to_pixels(f_target_diameter_degrees);
	f_target_left.ysize = to_pixels(f_target_diameter_degrees);
	f_target_left.depth = 1;
	f_target_left.r = f_target_color[0];
	f_target_left.g = f_target_color[1];
	f_target_left.b = f_target_color[2];
	render_dot(&f_target_left);

	f_target_right.xorigin = to_pixels(f_target_offset_horizontal_degrees);
	f_target_right.yorigin = to_pixels(f_target_offset_vertical_degrees);
	f_target_right.xsize = to_pixels(f_target_diameter_degrees);
	f_target_right.ysize = to_pixels(f_target_diameter_degrees);
	f_target_right.depth = 1;
	f_target_right.r = f_target_color[0];
	f_target_right.g = f_target_color[1];
	f_target_right.b = f_target_color[2];
	render_dot(&f_target_right);

	return status;
}


/* 
 * my_check_for_handle 
 * 
 * Called as an escape function ... e.g. 
 * 
 * to wherever on 1 % my_check_for_handle
 * 
 * This function will fetch 4 handles from render. They are saved
 * as f_fixpt_handle, f_dots_handle, f_target_left_handle, and 
 * f_target_right_handle respectively. 
 * The counter f_handle_count must be zeroed before this is used. 
 * Since the allocation of graphic objects takes place early in the 
 * state set (render_init), the handle counter should get zeroed 
 * in the my_render_init() function. 
 *
 * On each invocation it checks once for a handle.If a handle is found 
 * it is assigned to the proper variable and the handle counter is 
 * incremented.  
 * 
 * Returns 0 if the handle counter is less than 4, or 1 if the handle 
 * counter is 4.
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
		}
		else if (f_handle_count == 1)
		{
			f_dots_handle = handle;
			f_handle_count = 2;
		}
		else if (f_handle_count == 2)
		{
			f_target_left_handle = handle;
			f_handle_count = 3;
		}
		else if (f_handle_count == 3)
		{
			f_target_right_handle = handle;
			f_handle_count = 4;
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
	f_ustim_flag = 0;
	f_ustim_ready_flag = 0;
	f_dots_motion_ready_flag = 0;
	f_trialsCompleted = 0;
	alloff();
	render_frame(0);

	/* Free experimental condition struct array if it exists */
	if (f_pexptcond)
	{
		dprintf("Cleanup existing experimental condition list...\n");
		free(f_pexptcond);
	}
	
	
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
	int pre_pursuit_frames;
	int pre_dots_motion_frames;
	int dots_motion_frames;
	float pursuit_speed_pixels;		/* pursuit speed converted to pixels/frame */
	float vcam[3];					/* vector direction of camera motion(along heading angle) */
	float distance;					/* distance travelled by camera */	
	int i;
	
	/* 
	 * Misc initializations for trial
	 */
	 
	f_animation_done = 0;
	f_ustim_ready_flag = 0;
	f_dots_motion_ready_flag = 0;
	f_show_dots_flag = 1;

	/* 
	 * Clean up animation helpers if necessary. Doing it here ensures its 
	 * done even if all trials are completed. 
	 */

	if (f_panim_path) 
	{
		destroy_animation_helper(f_panim_path);
		f_panim_path = NULL;
	}
	if (f_panim_pursuit)
	{
		destroy_animation_helper(f_panim_pursuit);
		f_panim_pursuit = NULL;
	}
		

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

		/*
		 * Compute detailed trial conditions. 
		 * Camera/eye starts at (0,0,0) and travels in the xy plane. 
		 * Heading=0 corresponds to the positive x-axis.
		 * Heading>0 corresponds to clockwise rotation about z axis (travel right)
		 * Heading>0 corresponds to counter-clockwise rotation about z axis (travel left)
		 */

		if (f_pexptcond[f_trial_condition_index].show_dots == 0)
		{
			f_show_dots_flag = 0;
		}
			
		if (f_pexptcond[f_trial_condition_index].angle_degrees >= 0)
		{
			f_ptarget_correct = &f_target_right;
			f_ptarget_incorrect = &f_target_left;
		}
		else
		{
			f_ptarget_correct = &f_target_left;
			f_ptarget_incorrect = &f_target_right;
		}
					
		f_frames_pre_pursuit = (float)f_pre_pursuit_time*f_frames_per_second/1000.0f;
		f_frames_pre_dots_motion = (float)f_pre_dots_motion_time*f_frames_per_second/1000.0f;
		f_frames_dots_motion = (float)f_dots_motion_time*f_frames_per_second/1000.0f;
		f_frames_dots_motion_midpoint = f_frames_dots_motion/2;
		pursuit_speed_pixels =  f_pexptcond[f_trial_condition_index].pursuit_sgn * f_pursuit_speed * to_pixels(1) / f_frames_per_second;

		f_moveobject.handle = f_fixpt_handle;
		f_moveobject.initial_pos[0] = to_pixels(f_fixpt_refpos[0]) - pursuit_speed_pixels * (f_frames_pre_dots_motion + f_frames_dots_motion_midpoint);
		f_moveobject.initial_pos[1] = to_pixels(f_fixpt_refpos[1]);
		f_moveobject.final_pos[0] = to_pixels(f_fixpt_refpos[0]) + pursuit_speed_pixels * (f_frames_dots_motion - f_frames_dots_motion_midpoint);
		f_moveobject.final_pos[1] = to_pixels(f_fixpt_refpos[1]);
		f_moveobject.ndelay = f_frames_pre_pursuit;
		f_moveobject.nframes = f_frames_pre_dots_motion + f_frames_dots_motion;

		distance = get_eye_travel_distance();
		vcam[0] = sin(f_pexptcond[f_trial_condition_index].angle_degrees * M_PI/180.0f);
		vcam[1] = cos(f_pexptcond[f_trial_condition_index].angle_degrees * M_PI/180.0f);
		vcam[2] = 0;
		for (i=0; i<3; i++)
		{
			f_path.ep[i] = f_cam_position[i];
			f_path.vp[i] = f_path.ep[i];	/* not really true for [1] */
			f_path.epf[i] = f_cam_position[i] + distance * vcam[i];
			f_path.vpf[i] = f_path.epf[i];	/* not really true for [1] */
		}
		f_path.vp[1] += 100;
		f_path.vpf[1] += 100;
		f_path.ndelay = f_frames_pre_pursuit + f_frames_pre_dots_motion;
		f_path.nframes = f_frames_dots_motion;

		dprintf("path distance %d vcam (%d,%d) pp %d pdm %d dm %d\n", (int)(distance), (int)(vcam[0]*100), (int)(vcam[1]*100), f_frames_pre_pursuit, f_frames_pre_dots_motion, f_frames_dots_motion); 

		/* 
		 * Create and init animation helpers. 
		 */
		 
		f_panim_path = create_path_animation_helper(&f_camera, &f_path);
		f_panim_pursuit = create_pursuit_animation_helper(f_fixpt_handle, &f_fixpt, &f_moveobject);
		f_step_counter = 0;
		f_panim_path->step(f_panim_path, f_step_counter);
		f_panim_pursuit->step(f_panim_pursuit, f_step_counter);
		f_step_counter = 1;
		render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_NO_FRAME);
		render_onoff(&f_dots_handle, HANDLE_OFF, ONOFF_NO_FRAME);

		/* 
		 * The anim helper initialization initializes the fixpt position. Target positions are also ready, so
		 * now we can init the eye windows (they remain OFF however). 
		 */
		 
		my_eye_window(EYE_WINDOW_INIT);

		
		/*
		 * Send a FRAME command
		 */
		
		render_frame(0);
		
		/*
		 * TODO: test angle=0 condition in dialog
		 */

		/* 
		 * Assign time values to states 
		 */
		 
		set_times("trial_init_pause_time", (long)f_trial_init_pause_time, -1); 
		set_times("fixpt_acq", (long)f_acq_time, -1);
		set_times("fixpt_acq_fail_pause", (long)f_acq_fail_time, -1);
		set_times("fixpt_hold", (long)f_fixation_time, -1);
		set_times("fixpt_acq_noise_pause", (long)f_acq_noise_time, -1);
		set_times("dots_on_wait", (long)f_pre_dots_motion_time, -1);
		set_times("intertrial_pause", (long)f_intertrial_time, -1);
		set_times("reward_on", (long)f_reward_preset, (long)f_reward_random);
		set_times("ustim_on", (long)f_dots_motion_time, -1);
		set_times("wait_for_answer", (long)f_answer_time, -1);
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
			dprintf("ustim_on: %d\n", f_dots_motion_time);
			dprintf("answer_time: %d\n", f_answer_time);
		}

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
	

	return status;
}



/*
 * get_eye_travel_distance()
 * 
 * Returns the distance (in 3d world coordinates) the eye travels
 * during a trial (during the "dots motion time").
 */
 

float get_eye_travel_distance()
{
	return (float)f_eye_speed * EYE_SPEED_TO_3D_UNITS * (float)f_dots_motion_time / 1000.0f;
}

void get_dotfield_extents(float *pxmax, float *pymax, float *pzmax)
{
	float hxpos, hypos;		/* eye travel distance at max positive heading angle */
	float fovx_over_2;		/* half the x field of view angle */
	float fovy_over_2;
	float x1, x2;
	float y1, y2;
	float x1pos, x2pos;
	float y1pos, y2pos;
	float ang;
	float xmax, ymax;
	
	hxpos = get_eye_travel_distance() * sin(f_maxangle * M_PI/180.0f);
	hypos = get_eye_travel_distance() * cos(f_maxangle * M_PI/180.0f);
	fovx_over_2 = atan2f((float)x_dimension_mm/2.0f, f_screen_distance_MM);
	fovy_over_2 = atan2f((float)y_dimension_mm/2.0f, f_screen_distance_MM); 	
	
	x1 = f_far_plane_distance * tan(fovx_over_2);
	y1 = f_far_plane_distance;
	x2 = -x1;
	y2 = y1;
	ang = f_maxangle * M_PI/180.0f;

	/* 
	 * p1 and p2 are the far corners of the viewing frustum. Rotate them clockwise 
	 * through the max heading angle + fovx_over_2, then translate by the travel distance. 
	 */
	 
	x1pos = x1 * cos(ang) + y1 * sin(ang) + hxpos;
	y1pos = -x1 * sin(ang) + y1 * cos(ang) + hypos;
	x2pos = x2 * cos(ang) + y2 * sin(ang) + hxpos;
	y2pos = -x2 * sin(ang) + y2 * cos(ang) + hypos;

	*pxmax = max(max(x1pos, x2pos), hxpos);
	*pymax = max(max(y1pos, y2pos), hypos);
	*pzmax = f_far_plane_distance * tan(fovy_over_2);
	return;
}


int my_animate()
{
	int path_status = 0;
	int pursuit_status = 0;
	int status = 0;
	
	/*
	 * Init counter and status
	 */

	f_went_cycles = 0;
	f_wstatus = 0;
	
	/* 
	 * step the animation
	 */
	 
	path_status = f_panim_path->step(f_panim_path, f_step_counter);
	pursuit_status = f_panim_pursuit->step(f_panim_pursuit, f_step_counter);

	if (f_step_counter == 1) dprintf("f_step_counter=1, path_status=%d, pursuit_status=%d\n", path_status, pursuit_status);
	
	if (f_verbose & DEBUG_BIT)
	{
		if ((f_step_counter % 20)==0)
		{
			dprintf("step_counter %d: path_status %d pursuit_status %d\n", f_step_counter, path_status, pursuit_status);
			dprintf("dot pos: %d, %d\n", (int)(to_degrees(f_fixpt.xorigin)*100), (int)(to_degrees(f_fixpt.yorigin)*100));
			dprintf("camera pos (%d, %d, %d) direc (%d, %d, %d) up (%d, %d, %d)\n",
				(int)(f_camera.ex*100), (int)(f_camera.ey*100), (int)(f_camera.ez*100),
			    (int)(f_camera.dx*100), (int)(f_camera.dy*100), (int)(f_camera.dz*100),
			    (int)(f_camera.ux*100), (int)(f_camera.uy*100), (int)(f_camera.uz*100));
		}
	}
	
	f_step_counter++;

	/*
	 * When the dot motion starts the path animation stepper returns
	 * 2, which indicates the next frame is its second key frame
	 * (the first being the first frame of the delay period). 
	 * If the paradigm calls for it, this would be the place to 
	 * turn on the microstim.We set a "ready" flag which is converted to 
	 * the actual flag (which is then detected by the ustim loop) once the
	 * WENT is received. 
	 */
	 
	if (path_status == 2)
	{
		f_dots_motion_ready_flag = 1;
		if (f_pexptcond[f_trial_condition_index].ustim)
		{
			f_ustim_ready_flag = 1;
		}
	}
	
	
	
	/*
	 * If the path animation is complete then the status from the 
	 * step() is negative. (Actually, both animations should be 
	 * the same length -- so both should have negative status). 
	 * In that case we will turn off the dots and fixpt and turn 
	 * on the targets. 
	 */
	 
	if (path_status < 0)
	{
		if (f_verbose & DEBUG_BIT) dprintf("path status = %d step_counter=%d\n", path_status, f_step_counter);
		render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_NO_FRAME);
		render_onoff(&f_dots_handle, HANDLE_OFF, ONOFF_NO_FRAME);
		render_onoff(&f_target_left_handle, HANDLE_ON, ONOFF_NO_FRAME);
		render_onoff(&f_target_right_handle, HANDLE_ON, ONOFF_NO_FRAME);
		f_animation_done = 1;
	}
	render_frame(0);

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
		bcode_float(CH_FIXPT_X, to_degrees(f_fixpt.xorigin));
		bcode_float(CH_FIXPT_Y, to_degrees(f_fixpt.yorigin));
				
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
			dprintf("Missed %d frames (%d check_went cycles, step counter %d)\n", frames, f_went_cycles, f_step_counter);
			ecode(MISSEDCD);
		}


		/* 
		 * Check if ustim ready flag is set. If it is, then set ustim flag now and reset 
		 * ustim ready flag to 0. 
		 */
		 
		if (f_ustim_ready_flag)
		{
			f_ustim_flag = 1;
			f_ustim_ready_flag = 0;
		}

		/* 
		 * Check if dotmotion ready flag is set. It it is, then drop ecode indicating that
		 * dot motion is starting on this frame. 
		 */

		if (f_dots_motion_ready_flag)
		{
			f_dots_motion_ready_flag = 0;
			ecode(FFMOVCD);
		}		 
		

		/* 
		 * Set f_wstatus for other conditions if needed. The flag f_animation_done is set
		 * in my_animation when the animation helpers return -1. 
		 */
		 
		if (f_animation_done)
		{
			f_wstatus = 2;
			ecode(FFOFFCD);
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

int my_trial_done(int icorrect)
{
	/*
	 * Close eye windows
	 */
	 
	my_eye_window(EYE_WINDOW_ALL_OFF);
	
	/* 
	 * Close analog window.
	 */
	
	awind(CLOSE_W);

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
		case 1:
			dprintf("Correct answer.(answer tallies not implemented!)\n");
			f_prtgen->mark(f_prtgen, f_trial_condition_index);
			break;
		case 0:
			dprintf("Incorrect answer.(answer tallies not implemented!)\n");
			f_prtgen->mark(f_prtgen, f_trial_condition_index);
			break;
		case -1:
			dprintf("No answer given!\n");
			break;
		case -2:
			dprintf("Fixation broken!\n");
			break;
		case -3:
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
	int i, j, sum;
	dprintf("===================Trial counts=================\n");
	dprintf("ind\ttally\n");
	dprintf("---\t-------------------------------\n");
	sum = 0;
	for (i=0; i<f_nexptcond; i++)
	{
		dprintf("%d\t", i);
		for (j=0; j<f_prtgen->count(f_prtgen, i); j++) dprintf("X");
		sum += f_prtgen->count(f_prtgen, i);
		dprintf("\n");
	}
	dprintf("\n");
	f_trialsCompleted = sum;  
	return;		
}

  
/* 
 * alloff()
 * 
 * Turns off fixpt and dots. Does not issue a render or frame. 
 * 
 */
 
int alloff()
{
	render_onoff(&f_dots_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_target_left_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_target_right_handle, HANDLE_OFF, ONOFF_NO_FRAME);
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



/*
 * my_ustim
 * 
 * Called as an action from the ustim loop. When turning off (iustim==0) be sure to 
 * reset the flag. 
 */
 
 int my_ustim(int iustim)
 {
 	if (iustim)
 	{
 		dprintf("Ustim ON\n");
 		dio_on(MUSTIM);
 		f_ustim_flag = 0;
 	}
 	else
 	{
 		dprintf("Ustim OFF\n");
 		dio_off(MUSTIM);
 	} 		
 	return 0;
 }

/* 
 * my_reward
 * 
 * Called as an action from reward loop. 
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
 * EYE_WINDOW_FIXPT_UPDATE: Update positions of fixpt eye window (EYE_WINDOW_FIXPT)
 * EYE_WINDOW_FIXPT_ON: 	Turn on checking of fixpt window
 * EYE_WINDOW_FIXPT_OFF:	Turn off checking of fixpt window
 * EYE_WINDOW_TARGETS_UPDATE: Update positions of target eye windows
 * EYE_WINDOW_TARGETS_ON: 	Turn on checking of target windows
 * EYE_WINDOW_TARGETS_OFF: 	Turn off checking of target windows
 * EYE_WINDOW_INIT: Create eye windows for fixpt and targets. The positions of the windows are 
 *                  set but their positions ARE defined. Call this function with EYE_WINDOW_INIT from 
 *                  my_trial_init. During animation (pursuit) you must call this function with
 *                  arg EYE_WINDOW_FIXPT_UPDATE to account for the motion of the fixpt. Also 
 *                  remember that calling with this arg leaves all eye windows OFF (i.e. you 
 *                  still must call with one of the *_ON args).
 */


int my_eye_window(int iflag)
{
	int status = 0;

	if (iflag & EYE_WINDOW_INIT)
	{
		wd_src_pos(EYE_WINDOW_FIXPT, WD_DIRPOS, 0, WD_DIRPOS, 0);
		wd_pos(EYE_WINDOW_FIXPT, (long)(to_degrees(f_fixpt.xorigin)*10.0), (long)(to_degrees(f_fixpt.yorigin)*10.0));
		wd_siz(EYE_WINDOW_FIXPT, (long)(f_fixpt_window*10.0), (long)(f_fixpt_window*10.0));
		wd_cntrl(EYE_WINDOW_FIXPT, WD_OFF);
		wd_src_check(EYE_WINDOW_FIXPT, WD_SIGNAL, 0, WD_SIGNAL, 1);

		wd_src_pos(EYE_WINDOW_TARGET_CORRECT, WD_DIRPOS, 0, WD_DIRPOS, 0);
		wd_pos(EYE_WINDOW_TARGET_CORRECT, (long)(to_degrees(f_ptarget_correct->xorigin)*10.0), (long)(to_degrees(f_ptarget_correct->yorigin)*10.0));
		wd_siz(EYE_WINDOW_TARGET_CORRECT, (long)(f_target_window*10.0), (long)(f_target_window*10.0));
		wd_cntrl(EYE_WINDOW_TARGET_CORRECT, WD_OFF);
		wd_src_check(EYE_WINDOW_TARGET_CORRECT, WD_SIGNAL, 0, WD_SIGNAL, 1);

		wd_src_pos(EYE_WINDOW_TARGET_INCORRECT, WD_DIRPOS, 0, WD_DIRPOS, 0);
		wd_pos(EYE_WINDOW_TARGET_INCORRECT, (long)(to_degrees(f_ptarget_incorrect->xorigin)*10.0), (long)(to_degrees(f_ptarget_incorrect->yorigin)*10.0));
		wd_siz(EYE_WINDOW_TARGET_INCORRECT, (long)(f_target_window*10.0), (long)(f_target_window*10.0));
		wd_cntrl(EYE_WINDOW_TARGET_INCORRECT, WD_OFF);
		wd_src_check(EYE_WINDOW_TARGET_INCORRECT, WD_SIGNAL, 0, WD_SIGNAL, 1);
	}
	
	if (iflag & EYE_WINDOW_FIXPT_UPDATE)
	{
		wd_pos(EYE_WINDOW_FIXPT, (long)(to_degrees(f_fixpt.xorigin)*10.0), (long)(to_degrees(f_fixpt.yorigin)*10.0));
	}

	if ((iflag & EYE_WINDOW_FIXPT_ON) && !(f_verbose & DEBUG_EYE_WINDOW_BIT))
	{
		wd_cntrl(EYE_WINDOW_FIXPT, WD_ON);
	}

	if (iflag & EYE_WINDOW_FIXPT_OFF)
	{
		wd_cntrl(EYE_WINDOW_FIXPT, WD_OFF);
	}

	if (iflag & EYE_WINDOW_TARGETS_UPDATE)
	{
		/*
		 * The pointers to the correct and incorrect targets are set in my_trial_init. 
		 */
		wd_pos(EYE_WINDOW_TARGET_CORRECT, (long)(to_degrees(f_ptarget_correct->xorigin)*10.0), (long)(to_degrees(f_ptarget_correct->yorigin)*10.0));
		wd_pos(EYE_WINDOW_TARGET_INCORRECT, (long)(to_degrees(f_ptarget_incorrect->xorigin)*10.0), (long)(to_degrees(f_ptarget_incorrect->yorigin)*10.0));
		dprintf("correct %d %d incorrect %d %d\n", 
		(long)(to_degrees(f_ptarget_correct->xorigin)*10.0), (long)(to_degrees(f_ptarget_correct->yorigin)*10.0),
		(long)(to_degrees(f_ptarget_incorrect->xorigin)*10.0), (long)(to_degrees(f_ptarget_incorrect->yorigin)*10.0));
	}

	if (iflag & EYE_WINDOW_TARGETS_ON  && !(f_verbose & DEBUG_EYE_WINDOW_BIT))
	{
		wd_cntrl(EYE_WINDOW_TARGET_CORRECT, WD_ON);
		wd_cntrl(EYE_WINDOW_TARGET_INCORRECT, WD_ON);
	}

	if (iflag & EYE_WINDOW_TARGETS_OFF)
	{
		wd_cntrl(EYE_WINDOW_TARGET_CORRECT, WD_OFF);
		wd_cntrl(EYE_WINDOW_TARGET_INCORRECT, WD_OFF);
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

int dots_onoff(int onoff)
{
	if (onoff)
	{
		/* 
		 * Turn on the dotfield. Sending this as an update also updates (regenerates)
		 * the dots themselves.
		 */
		render_update(f_dots_handle, &f_dotfield, sizeof(DotFieldStruct), HANDLE_ON);
		render_frame(0);
		/*render_onoff(&f_dots_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT);*/
	}
	else
	{
		render_onoff(&f_dots_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);
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
		to fixpt_window on 1 % render_check_for_went
	fixpt_window:
		do my_eye_window(EYE_WINDOW_FIXPT_UPDATE | EYE_WINDOW_FIXPT_ON)
		to fixpt_acq
	fixpt_acq:
		time 4000
		to pause_detected on +PSTOP & softswitch
		to fixpt_hold on -EYEFLAG_FIXPT & eyeflag
		to fixpt_acq_fail
	fixpt_acq_fail:
		do fixpt_onoff(0)
		to fixpt_acq_fail_pause on 1 % render_check_for_went
	fixpt_acq_fail_pause:
		time 500			/* f_acq_fail_pause_time */
		do my_eye_window(EYE_WINDOW_FIXPT_OFF)
		to fixpt_on
	fixpt_hold:
		time 150			/* f_fixation_time */
		to pause_detected on +PSTOP & softswitch
		to fixpt_acq_noise on +EYEFLAG_FIXPT & eyeflag
		to fixation
	fixpt_acq_noise:
		do fixpt_onoff(0)
		to fixpt_acq_noise_pause on 1 % render_check_for_went
	fixpt_acq_noise_pause:
		time 500			/* f_acq_noise_pause_time */
		do my_eye_window(EYE_WINDOW_FIXPT_OFF)
		to fixpt_on
	fixation:
		code FIXCD
		to dots_on on 1 = f_show_dots_flag
		to no_dots_on
	no_dots_on:
		do dots_onoff(0)
		to dots_on_wait on 1 % render_check_for_went
	dots_on:
		do dots_onoff(1)
		to dots_on_wait on 1 % render_check_for_went
	dots_on_wait:
		code FFONCD
		time 500 			/* f_pre_dots_motion_time */
		to pause_detected on +PSTOP & softswitch
		to fixation_fail on +EYEFLAG_FIXPT & eyeflag
		to animate
	animate:
		do my_animate()
		to animate_wait
	animate_wait:
		do my_eye_window(EYE_WINDOW_FIXPT_UPDATE)
		to fixation_fail_wait on +EYEFLAG_FIXPT & eyeflag
		to pause_detected_wait on +PSTOP & softswitch
		to animate on 1 % my_check_for_went
		to answer_points on 2 = f_wstatus
	answer_points:
		time 20
		do my_eye_window(EYE_WINDOW_FIXPT_OFF | EYE_WINDOW_TARGETS_UPDATE | EYE_WINDOW_TARGETS_ON)
		to wait_for_answer
	wait_for_answer:
		time 2000		/* f_answer_time */
		to answer_debug on +DEBUG_ANSWER_POINTS_BIT & f_verbose
		to answer_incorrect on -EYEFLAG_INCORRECT & eyeflag
		to answer_correct on -EYEFLAG_CORRECT & eyeflag
		to no_answer
	answer_debug:
		time 500
		to no_answer
	answer_correct:
		code CORRECTCD
		do my_trial_done(TRIAL_DONE_CORRECT)
		to reward_on on 1 % render_check_for_went
	answer_incorrect:
		code WRONGCD
		do my_trial_done(TRIAL_DONE_INCORRECT)
		/* to reward_on on 1 % render_check_for_went */
		/*time 500
		to trial_init*/
		to intertrial_pause on 1 % render_check_for_went
	reward_on:
		code REWCD
		do my_reward(1)
		to reward_off
	reward_off:
		do my_reward(0)
		to intertrial_pause		
	no_answer:
		code NOCHCD
		do my_trial_done(TRIAL_DONE_NOANSWER)
		to intertrial_pause on 1 % render_check_for_went			
	fixation_fail_wait:
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

ustim_set {
status ON
begin	ustim_begin:
		to ustim_init
	ustim_init:
		do my_ustim(0)
		to ustim_wait
	ustim_wait:
		to ustim_paused on +PSTOP & softswitch
		to ustim_on on 1 = f_ustim_flag
	ustim_on:
		code USTIMONCD
		do my_ustim(1)
		time 1000
		to ustim_paused on +PSTOP & softswitch
		to ustim_off
	ustim_off:
		code USTIMOFFCD
		do my_ustim(0)
		to ustim_wait
	ustim_paused:
		do my_ustim(0)
		to ustim_init on -PSTOP & softswitch
}


/* $Id: st19aft.d,v 1.12 2015/05/12 20:53:48 devel Exp $  */

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
#include "slider.h"				/* sliding window used for spin detection */
#include "randomtrialgenerator.h"	/* tool for generating trials in blocks*/
#include "fcounter.h"			/* fancy counter */


/*
 * Ecodes local to the steering paradigm.
 * These have to be defined here so the spot translator will see them. 
 * Spot does not process #include files, so any macros used in the state
 * set definition must be in this file. 
 */
#define MINIMUMCD 1500
#define BASECD	1550
#define INITCD	1501 
#define TRIALCD	1502 
#define UPDCD	1503
#define MISSEDCD	1505	// missed frame code
#define TRIALENDCD 1506
#define JENDCD		1508
#define WENTCD		1509
#define JBEGINCD	1510
#define JLEFTCD		1512
#define JRIGHTCD	1513
#define BLIPBEGINCD	1514
#define BLIPENDCD	1515
#define BLINKBEGINCD	1516
#define BLINKENDCD		1517
#define SPINNINGCD		1518
#define NOTSPINNINGCD	1519
#define FLOWCD 1520
#define BADCD	8192

/*
 * ecode/bcode defines
 */ 
#include "st19aft_defines.h"

/*
 * These are the experiment types one can choose. 
 * One of the menu variables f_exptype_steering, etc, should be set to 1
 * prior to a RESET STATES command. 
 */

#define EXPTYPE_STEERING 991
#define EXPTYPE_STEERING_WITH_FIXATION 992
#define EXPTYPE_FIXATION_ONLY 933
#define EXPTYPE_FIXATION_TUNING 994
#define EXPTYPE_UNKNOWN 999
int f_exptype = EXPTYPE_UNKNOWN;
int f_exptype_steering = 1;
int f_exptype_steering_with_fixation = 0;
int f_exptype_fixation_only = 0;
int f_exptype_fixation_tuning = 0;

/* Really really big char for sprinting errors. */
char f_charbuf[2048];

/* never var, quoth the raven */
int f_never = -1;

/* 
 * This var can be set to 1 to yield a beep at the onset of a 
 * steering trial. Take a look at state 'trial_beep' below to 
 * see which DIO line is relevant.
 */
int f_use_trial_beep = 0;

/* Debug flag for tracking down the unknown action bug */
int f_unknown_action_flag = 0;

/* How many failures can we take when trying to place blips and blinks? */
int f_max_failures = 1000;

/* counters handled by my_check_for_went */
int f_wstatus = 0;
int f_frame_counter = 0;
int f_went_counter = 0;
int f_went_cycles = 0;
int f_trial_end_flag = 0;

/* State list menu vars and associated vars*/
int f_seed = 0;
int f_seed_used = 0;
int f_jumps_per_trial = 10;
int f_ms_jump_min = 1000;
int f_ms_jump_avg = 1500;
int f_ms_jump_guard_before = 250;
int f_ms_jump_guard_after = 250;
int f_ntrials = 1;
int f_trial_counter = 0;
int f_all_trials_done = 0;
int f_frames_per_second = 85;
char f_local_addr[32]="192.168.1.1";
char f_remote_addr[32]="192.168.1.2";
int f_remote_port=2000;
float f_steer_max_degrees = 3.0;
int f_steer_zero = 1024;
int f_steer_dead = 0;
int f_speed = 3;
char f_jumpstr[1024]={0};
int f_verbose = 0;

/* bit masks for verbosity */
#define DEBUG_BIT 0x1
#define DEBUG_REWARD_BIT 0x2
#define DEBUG_JOY_BIT 0x4
static int f_ival_last=0, f_ival_max_diff=-999;

/* Perspective/camera parameters */
int f_screen_distance_MM = 290;
int f_far_plane_distance = 5000;

/* camera vars */
static float f_cam_position[3]; 							/* Current camera position */
static float f_cam_start[3] = { 0, 0, 0 };		/* Initial eye position at start of trial */
static int f_cam_height = 200;						/* Initial cam height - will be applied to cam_start (and used in updates) */
static float f_cam_trajectory = 0;						/* trajectory - direction camera is moving - IN RADIANS*/
static float f_cam_looking[3];
static float f_cam_up[3] = { 0, 1, 0 };

/* target parameters. */	
float f_targ_size = 0.25;
int f_targ_color[3] = { 255, 0, 0 };
float f_targ_dir = 0;							/* radians */
float f_targ_h = 0;
float f_targ_dist = 1000;
#define MAX_TARG_OFFSET_DEGREES 5
float f_targ_offset_degrees[MAX_TARG_OFFSET_DEGREES] = {15, 0.0, 0.0, 0.0, 0.0};
int f_targ_offset_weight[MAX_TARG_OFFSET_DEGREES] = {100, NULLI, NULLI, NULLI, NULLI};
int f_num_targ_offset_degrees = 0;
int f_targ_offset_weight_sum = 0;
int f_jump_one_sided = 0;
int f_jump_pong_sgn = 0;	/* When jump_one_sided==2, this is sgn last time through */
int f_use_incremental_jumps = 0;
float f_incremental_jump_min_degrees = 2.0;
float f_incremental_jump_step_degrees = 0.2;
int f_incremental_jump_up_threshhold = 2;
int f_incremental_jump_dn_threshhold = 2;
int f_all_jump_counter;
int f_per_jump_good_jump_counter;
int f_incremental_jump_bad_trial_counter;
float f_incremental_jump_current_jump_magnitude;
int f_incremental_jump_current_jump_magnitude_display;
int f_per_jump_reward_counter = 0;
int f_total_reward_counter = 0;
int f_trial_start_time = 0;
int f_abssteererror_holdoff = 1000;		/* How long to wait after jump before measuring abs steering error */

/* groundplane parameters */
int f_gp_length=1000;
int f_gp_width=1000;
int f_gp_dots=200;
int f_gp_pool=500;
int f_gp_plane_color[3]= { 50, 50, 50 };
int f_gp_dot_color[3]= { 255, 255, 255 };
int f_gp_flags = 0;
float f_gp_dotsize=2.0;

/* fixation point parameters */
float f_fixpt_x = 0;
float f_fixpt_y = 0;
float f_fixpt_diameter = 0.25;
float f_fixpt_window = 3.0;
int f_acq_time = 4006;
int f_fix_time = 501;
int f_acq_timeout = 502;
int f_fix_timeout = 503;
int f_fixpt_color[3] = { 0, 255, 0 };
int f_flow_duration_ms = 2000;
int f_flow_duration_frames = 0;


/* handles */
int f_handle_count = 0;
int f_gp_handle;
int f_targ_handle;
int f_fixpt_handle;

/* background color */
int f_background_color[3] = { 0, 0, 0 };

/* messaging structures. */
TargetStruct f_target;
MessageStruct f_msg;
DotStruct f_fixpt;

/* Fake joystick */
int f_fake_joy_enable = 0;
int f_fake_joy_value = 1024;


/* blink - target spot off/on */
int f_blink_debug=0;
float f_blink_rate = 0;		/* blinks per second */
int f_blink_duration_ms=0;
int f_blink_guard_ms=0;

/* blip - target spot off/on */
int f_blip_debug=0;
float f_blip_rate = 0;		/* blips per second */
int f_blip_duration_ms=0;
int f_blip_guard_ms=0;
float f_blip_max_ang_velocity_deg_per_sec = 0;

/* Actions */

struct action
{
	int start_frame;	/* What frame this action first has an effect */
	int end_frame;		/* Remove after action is performed on this frame */
	int type;   /* One of the ACTIONTYPE_* macros */
	float fval;	/* A number useful for the action, like jump size, etc. */
	int ival;	/* Another number useful for the action; an index, e.g. */
};
typedef struct action * PAction;

PAction f_all_actions = NULL;		/* Storage for all actions. Not in order */
int n_all_actions = 0;				/* How many actions were calloc'd in f_all_actions */

PAction* f_ordered_actions = NULL;	/* pointers to all actions, ordered by their start frame */
int f_candidate_action_index = 0;	/* Next action that may become current */

#define MAX_CURRENT_ACTIONS 12
PAction f_current_actions[MAX_CURRENT_ACTIONS];
int n_current_actions = 0;


/*
 * These are the action types defined. Place one in PActionStruct->type
 */

#define ACTIONTYPE_JUMP 1
#define ACTIONTYPE_BLIP 2
#define ACTIONTYPE_BLINK 3
#define ACTIONTYPE_TRIALEND 4

char *f_cActionStr[4] = 
{
	"JUMP",
	"BLIP",
	"BLNK", 
	"TRLE"
};

/* Helper struct used in building segtree */
struct segment_builder_struct
{
	int last;
	int jmin;
	float tau;
	int guard_before;
	int guard_after;
};


/* Reward stuff */
int f_going = 0;		/* this is 1 during update loop, 0 at end of trial. */
int f_spinflag = 0;
int f_jumpflag = 0;					/* Set when update specifies a jump. Cleared when WENT is received. */
int f_last_check_steering_went_counter = -1;	/* value of went counter the last time steering was checked. */
int f_last_check_steering_value = 0;
int f_went_counter_last_jump = -1;				/* went counter value when last jump happened.*/
int f_time_last_jump = INT_MAX;						/* not for reward - for tallying recovery time */
int f_next_reward_time = INT_MAX;
int f_last_reward_time = 0; 
int f_reward_on_target_sum=0;		/* Running total of time on target WITHOUT failing acquisition time test */
int f_grace_period_expire = INT_MAX;
Slider *f_pslider=NULL;					/* sliding window for spin detection */
int f_spinning = 0;
int f_reward_flag = 0;				/* Used for fixation and fixation/tuning */
float f_cos_reward_window = 0;		/* at start of a trial this is set to cos(reward_window) */
float f_reward_spatial_fraction;	/* pseudo-linear fraction = 1 when on target exactly, = 0 when off target at edge of window */

/* Reward menu stuff */
int f_reward_window_degrees = 4;
int f_reward_acquisition_time=1000;	/* acquisition time, converted to ms in menu callback function */
int f_reward_use_spatial_ramp=0;		/* if not zero then use spatial ramp for reward, not temporal */
int f_reward_ramp_time=5000;		/* time for reward wait to ramp from max to min */
int f_reward_wait_max = 1000;		/* max time for reward wait */
int f_reward_wait_min = 500;		/* min time for reward wait */
int f_spin_window_ms = 0;			/* Length of sliding window for spin detection */
int f_spin_deviation = 0;			/* max deviation for spinning */
int f_reward_preset = 10;
int f_reward_random = 20;

/* Experiment type selection */
int f_do_steering = 1;
int f_do_steering_with_fixation = 0;
int f_do_fixation_only = 0;
int f_do_fixation_tuning = 0;

/* Counter stuff */
PFCounter f_pfcounter = NULL;
int f_fcindex_alltrials = 0;
int f_fcindex_completetrials = 0;
int f_fcindex_breakfix = 0;
int f_fcindex_pause = 0;
int f_fcindex_goodjumps = 0;
int f_fcindex_alljumps = 0;
int f_fcindex_leftjumps = 0;
int f_fcindex_rightjumps = 0;
int f_fcindex_triallength = 0;
int f_fcindex_reacquisition = 0;
int f_fcindex_abssteererror = 0;
int f_fcindex_meanabssteererror = 0;
int f_fcindex_meanabssteererrorcumulativetime = 0;
int f_fcindex_totalrewardcounter = 0;
int f_iPctComplete = 0;
int f_iPctBreak = 0;
int f_iAllTrials = 0;
int f_iAllJumps = 0;
int f_iLeftJumps = 0;
int f_iRightJumps = 0;
int f_iGoodJumps = 0;
int f_iTrialLengthMSAvg = 0;
int f_iTrialLengthMSStd = 0;
int f_iReacquisitionMSAvg = 0;
int f_iReacquisitionMSStd = 0;
int f_iAbsSteerError = 0;
int f_iMeanAbsSteerError = 0;
int f_iMeanAbsSteerErrorCumulativeTime = 0;
int f_iTotalRewardCounter = 0;

/* REX menu declarations */

VLIST state_vl[] = {
"day_seed",		&f_seed, NP, NP, 0, ME_DEC,
"do_steering(=1)", &f_do_steering, NP, NP, 0, ME_DEC,
"do_steering_w_fix(=1)", &f_do_steering_with_fixation, NP, NP, 0, ME_DEC,
"do_fixation(=1)", &f_do_fixation_only, NP, NP, 0, ME_DEC,
"do_fixation_tuning(=1)", &f_do_fixation_tuning, NP, NP, 0, ME_DEC,
"frame_rate(1/s)", &f_frames_per_second, NP, NP, 0, ME_DEC,
"local_ip", f_local_addr, NP, NP, 0, ME_STR,
"render_host_ip", f_remote_addr, NP, NP, 0, ME_STR,
"render_port", &f_remote_port, NP, NP, 0, ME_DEC,
"verbose", &f_verbose, NP, NP, 0, ME_DEC,
NS,
};

/*
 * Help message.
 */
char hm_sv_vl[] = "";

/* steering menu */
VLIST steering_vl[] = {
"ntrials(IGNORED)", &f_ntrials, NP, NP, 0, ME_DEC,
"jumps_per_trial",	&f_jumps_per_trial, NP, NP, 0, ME_DEC,
"min_jump_time_ms",	&f_ms_jump_min, NP, NP, 0, ME_DEC,
"avg_jump_time_ms", &f_ms_jump_avg, NP, NP, 0, ME_DEC,
"jump_guard_before_ms", &f_ms_jump_guard_before, NP, NP, 0, ME_DEC,
"jump_guard_after_ms", &f_ms_jump_guard_after, NP, NP, 0, ME_DEC,
"Steering_max", &f_steer_max_degrees, NP, NP, 0, ME_FLOAT,
"Steering_zero_point", &f_steer_zero, NP, NP, 0, ME_DEC,
"Steering_dead_zone", &f_steer_dead, NP, NP, 0, ME_DEC,
"rew_win(deg)",	&f_reward_window_degrees, NP, NP, 0, ME_DEC,
"speed",	&f_speed, NP, NP, 0, ME_DEC,
NS,
};

/*
 * Help message.
 */
char hm_steering[] = "";

/* Jump parameters menu */

VLIST jump_vl[] = {
"1_sided?(1=R,-1=L)", &f_jump_one_sided, 0, NP, 0, ME_DEC,
"Incremental_jumps", &f_use_incremental_jumps, 0, NP, 0, ME_DEC,
"Inc_jump_min(deg)", &f_incremental_jump_min_degrees, 0, NP, 0, ME_FLOAT,
"Inc_jump_step(deg)", &f_incremental_jump_step_degrees, 0, NP, 0, ME_FLOAT,
"Inc_jump_num+", &f_incremental_jump_up_threshhold, 0, NP, 0, ME_DEC,
"Inc_jump_num-", &f_incremental_jump_dn_threshhold, 0, NP, 0, ME_DEC,
"Jump_Degrees_0(deg)", &f_targ_offset_degrees[0], 0, NP, 0, ME_FLOAT,
"Jump_Weight_0", &f_targ_offset_weight[0], 0, NP, 0, ME_NVALD,
"Jump_Degrees_1(deg)", &f_targ_offset_degrees[1], 0, NP, 0, ME_FLOAT,
"Jump_Weight_1", &f_targ_offset_weight[1], 0, NP, 0, ME_NVALD,
"Jump_Degrees_2(deg)", &f_targ_offset_degrees[2], 0, NP, 0, ME_FLOAT,
"Jump_Weight_2", &f_targ_offset_weight[2], 0, NP, 0, ME_NVALD,
"Jump_Degrees_3(deg)", &f_targ_offset_degrees[3], 0, NP, 0, ME_FLOAT,
"Jump_Weight_3", &f_targ_offset_weight[3], 0, NP, 0, ME_NVALD,
"Jump_Degrees_4(deg)", &f_targ_offset_degrees[4], 0, NP, 0, ME_FLOAT,
NS,
};

char hm_jump[] = 	"1_sided applies to incremental jumps and weighted jumps.\n1_sided = -1(1) force L(R) all the time.\n1_sided = 0 random jump direction\n1_sided = 2 alternate L,R,L,R...\n\n" 
					"Inc_jump_num+: # of good jumps (in a single trial) before incrementing jump size\n"
					"Inc_jump_num-: # of trials with NO good jums before decrementing jump size\n";


/* Viewing volume menu */

VLIST vvol_vl[] = {
"screen_dist(mm)", &f_screen_distance_MM, NP, NP, 0, ME_DEC,
"far_plane_distance", &f_far_plane_distance, NP, NP, 0, ME_DEC, 
"Camera_height", &f_cam_height, NP, NP, 0, ME_DEC, 
NS,
};

char hm_vvol[] = "";

/* Target parameters menu */

VLIST target_vl[] = {
"Target_diam(degrees)", &f_targ_size, NP, NP, 0, ME_FLOAT,
"Target_color(R)", &f_targ_color[0], 0, NP, 0, ME_DEC,
"Target_color(G)", &f_targ_color[1], 0, NP, 0, ME_DEC,
"Target_color(B)", &f_targ_color[2], 0, NP, 0, ME_DEC,
"Target_distance", &f_targ_dist, 0, NP, 0, ME_FLOAT,
"Target_height(degrees)", &f_targ_h, 0, NP, 0, ME_FLOAT,
NS,
};

char hm_target[] = "";

VLIST groundplane_vl[] = {
"Grid_length", &f_gp_length, NP, NP, 0, ME_DEC,
"Grid_width", &f_gp_width, NP, NP, 0, ME_DEC,
"Dots_per_grid", &f_gp_dots, NP, NP, 0, ME_DEC,
"Grid_pool_size", &f_gp_pool, NP, NP, 0, ME_DEC,
"Plane_color(R)", &f_gp_plane_color[0], 0, NP, 0, ME_DEC,
"Plane_color(G)", &f_gp_plane_color[1], 0, NP, 0, ME_DEC,
"Plane_color(B)", &f_gp_plane_color[2], 0, NP, 0, ME_DEC,
"Dot_color(R)", &f_gp_dot_color[0], 0, NP, 0, ME_DEC,
"Dot_color(G)", &f_gp_dot_color[1], 0, NP, 0, ME_DEC,
"Dot_color(B)", &f_gp_dot_color[2], 0, NP, 0, ME_DEC,
"Flags", &f_gp_flags, 0, NP, 0, ME_DEC,
"Dot_size", &f_gp_dotsize, NP, NP, 0, ME_FLOAT,
NS,
};

char hm_groundplane[] = "";

/* Background color menu  */

VLIST background_vl[] = {
"Background_color(R)", &f_background_color[0], 0, NP, 0, ME_DEC,
"Background_color(G)", &f_background_color[1], 0, NP, 0, ME_DEC,
"Background_color(B)", &f_background_color[2], 0, NP, 0, ME_DEC,
NS,
};

char hm_background[] = "";

VLIST fakejoy_vl[] = {
"Enable_fake_joy", &f_fake_joy_enable, 0, NP, 0, ME_DEC,
"Fake_joy_value", &f_fake_joy_value, 0, NP, 0, ME_DEC,
NS,
};

char hm_fakejoy[] = "";

VLIST blink_vl[] = {
"blink_rate(blinks/sec)", &f_blink_rate, 0, NP, 0, ME_FLOAT,
"off_duration(ms)", &f_blink_duration_ms, 0, NP, 0, ME_DEC,
"guard(ms)", &f_blink_guard_ms, 0, NP, 0, ME_DEC,
"debug_msgs", &f_blink_debug, 0, NP, 0, ME_DEC,
NS,
};

char hm_blink[] = "";

VLIST blip_vl[] = {
"blip_rate(blips/sec)", &f_blip_rate, 0, NP, 0, ME_FLOAT,
"blip_duration(ms)", &f_blip_duration_ms, 0, NP, 0, ME_DEC,
"guard(ms)", &f_blip_guard_ms, 0, NP, 0, ME_DEC,
"max_vel(deg/sec)", &f_blip_max_ang_velocity_deg_per_sec, 0, NP, 0, ME_FLOAT,
"debug_msgs", &f_blip_debug, 0, NP, 0, ME_DEC,
NS,
};

char hm_blip[] = "";

/* Reward parameters menu! */

VLIST reward_vl[] = {
"acquisition_time(ms)", &f_reward_acquisition_time, NP, NP, 0, ME_DEC,
"reward_wait_max(ms)", &f_reward_wait_max, NP, NP, 0, ME_DEC,
"reward_wait_min(ms)", &f_reward_wait_min, NP, NP, 0, ME_DEC,
"reward_ramp_time(ms)", &f_reward_ramp_time, NP, NP, 0, ME_DEC,
"use_spatial_ramp(1/0)?", &f_reward_use_spatial_ramp, NP, NP, 0, ME_DEC,
"spin_window_time(ms)", &f_spin_window_ms, NP, NP, 0, ME_DEC,
"spin_deviation(ADC)", &f_spin_deviation, NP, NP, 0, ME_DEC,
"reward_size_preset(ms)", &f_reward_preset, NP, NP, 0, ME_DEC, 
"reward_size_random(ms)", &f_reward_random, NP, NP, 0, ME_DEC, 
NS,
};

char hm_reward[] = "";

VLIST fixpt_vl[] = {
"fixpt_x(deg)", &f_fixpt_x, NP, NP, 0, ME_FLOAT,
"fixpt_y(deg)", &f_fixpt_y, NP, NP, 0, ME_FLOAT,
"fixpt_diameter(deg)", &f_fixpt_diameter, NP, NP, 0, ME_FLOAT,
"fixpt_window(deg)", &f_fixpt_window, NP, NP, 0, ME_FLOAT,
"fixpt_color(R)", &f_fixpt_color[0], 0, NP, 0, ME_DEC,
"fixpt_color(G)", &f_fixpt_color[1], 0, NP, 0, ME_DEC,
"fixpt_color(B)", &f_fixpt_color[2], 0, NP, 0, ME_DEC,
"acq_time(ms)", &f_acq_time, NP, NP, 0, ME_DEC,
"acq_timeout(ms)", &f_acq_timeout, NP, NP, 0, ME_DEC,
"fix_time(ms)", &f_fix_time, NP, NP, 0, ME_DEC,
"fix_timeout(ms)", &f_fix_timeout, NP, NP, 0, ME_DEC,
"flow_duration(ms)", &f_flow_duration_ms, NP, NP, 0, ME_DEC,
NS,
};

char hm_fixpt[] = "";

MENU umenus[] = {
{"Main", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{"separator", NP}, 
{"steering", &steering_vl, NP, NP, 0, NP, hm_steering},
{"fixation", &fixpt_vl, NP, NP, 0, NP, hm_fixpt}, 
{"jump", &jump_vl, NP, NP, 0, NP, hm_jump}, 
{"reward", &reward_vl, NP, NP, 0, NP, hm_reward}, 
{"blink", &blink_vl, NP, NP, 0, NP, hm_blink}, 
{"blip", &blip_vl, NP, NP, 0, NP, hm_blip}, 
{"viewing volume", &vvol_vl, NP, NP, 0, NP, hm_vvol},
{"separator", NP}, 
{"target", &target_vl, NP, NP, 0, NP, hm_target},
{"groundplane", &groundplane_vl, NP, NP, 0, NP, hm_groundplane},
{"background", &background_vl, NP, NP, 0, NP, hm_background}, 
{"fakjoy", &fakejoy_vl, NP, NP, 0, NP, hm_fakejoy}, 
{NS},
};


RTVAR rtvars[] = {
	{"number of trials",	&f_iAllTrials},
	{"avg trial length ms", &f_iTrialLengthMSAvg},
	{"% completed", 	&f_iPctComplete},
	{"% break fix",	&f_iPctBreak},
	{"All jump counter", &f_iAllJumps},
	{"Right jump counter", &f_iRightJumps},
	{"Left jump counter", &f_iLeftJumps},
	{"Good jump counter", &f_iGoodJumps},
	{"Avg reacq. time", &f_iReacquisitionMSAvg},
	{"Mean abs steer err", &f_iMeanAbsSteerError},
	{"Steer err sum time", &f_iMeanAbsSteerErrorCumulativeTime},
	{"Reward counter", &f_iTotalRewardCounter},
	{"Bad trial counter", &f_incremental_jump_bad_trial_counter},
	{"Current jump size", &f_incremental_jump_current_jump_magnitude_display},
	{"", 0},
};

/* local functions. */

int my_check_for_handle();
void init_steering(void);
int my_render_init();
int my_check_for_handle();
int my_exp_init();
int alloff();
int my_trial_done();
int joystick_value();
void blink_prepare();
int blink_update(int frame);
void blip_prepare();
int blip_update(int frame, float *poffset);
int reward_check_window();
int reward_init();
int stpause(int isPaused);
int check_grace_time();
int check_steering_and_wait_time();
int check_steering();
float random_targ_offset_degrees();
char *my_pct(unsigned int i_numerator, unsigned int i_denominator, char *result);
void my_report(int full);
int my_target_reacq();

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

	
//	// In case we did a Reset States
	f_going = 0;
	f_cam_trajectory = 0;

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

	dprintf("Initializing render\n");

	// seed random number generator if necessary
	// TODO: Make usage of random number generator (and seed) consistent. 
	if (!f_seed_used)
	{
		ivunif(f_seed, 1);
		f_seed_used = 1;
	}
	
	// zero out handles and initialize counters.

	f_gp_handle = f_targ_handle = f_fixpt_handle = 0;
	f_handle_count = 0;

	// If f_ms_before_jump is changed during a pause, then a reset will update frames_before_jump, but not
	// this counter. If f_ms_before is DECREASED, and the new value for f_frames_before_jump is greater than 
	// the current value of f_went_counter, then the jump will never happen. Fix this by resetting the 
	// went counter here. djs 2/26/08

	f_went_counter = 0; 
	
	// background color

	render_bgcolor(f_background_color[0], f_background_color[1], f_background_color[2]);

	// Setup viewing volume -- this should be done prior to the groundplane init! */

	fovy = 2*atan2f(y_dimension_mm/2.0, f_screen_distance_MM); 	
	render_perspective(fovy, f_screen_distance_MM, f_far_plane_distance);

	// Initialize groundplane	

	render_groundplane(f_gp_length, f_gp_width, f_gp_dots, f_gp_pool, f_gp_plane_color, f_gp_dot_color, f_gp_dotsize, f_gp_flags);

	/* Setup init camera position */

	f_cam_position[0] = 0;
	f_cam_position[1] = (float)f_cam_height;
	f_cam_position[2] = 0;
	f_cam_looking[0] = -sin(f_cam_trajectory);
	f_cam_looking[1] = 0;
	f_cam_looking[2] = cos(f_cam_trajectory);
	render_camera(f_cam_position, f_cam_looking, f_cam_up);

	/* Configure target.*/
	
	f_targ_dir = 0;
	f_target.xoffset = -sin(f_targ_dir);
	f_target.zoffset = cos(f_targ_dir);
	f_target.xsize = f_target.ysize = f_targ_dist * sin(f_targ_size * M_PI / 180.0);
	f_target.d = f_targ_dist;
	f_target.h = f_targ_dist * sin(f_targ_h * M_PI / 180.0);
	f_target.r = f_targ_color[0];
	f_target.g = f_targ_color[1];
	f_target.b = f_targ_color[2];
	render_target(&f_target);
	
	/* Configure fixation point */
	
	f_fixpt.xorigin = to_pixels(f_fixpt_x);
	f_fixpt.yorigin = to_pixels(f_fixpt_y);
	f_fixpt.xsize = to_pixels(f_fixpt_diameter);
	f_fixpt.ysize = to_pixels(f_fixpt_diameter);
	f_fixpt.depth = 1;
	f_fixpt.r = f_fixpt_color[0];
	f_fixpt.g = f_fixpt_color[1];
	f_fixpt.b = f_fixpt_color[2];
	render_dot(&f_fixpt);
	
	/* ecodes and bcodes for parameters governing this expt */
	bcode_int(CH_FRAMES_PER_SECOND, f_frames_per_second);
	bcode_int(CH_SPEED, f_speed);
//	bcode_int(CH_MS_BEFORE_JUMP, f_ms_before_jump);
//	bcode_int(CH_MS_AFTER_JUMP, f_ms_after_jump);
	bcode_int(CH_NTRIALS, f_ntrials);
	bcode_int(CH_JUMPS_PER_TRIAL, f_jumps_per_trial);
	bcode_float(CH_STEER_MAX_DEGREES, f_steer_max_degrees);
	bcode_int(CH_TARG_OFFSET_DEGREES_0, f_targ_offset_degrees[0]);
	bcode_int(CH_TARG_OFFSET_DEGREES_1, f_targ_offset_degrees[1]);
//	bcode_int(CH_BLINK_PROBABILITY_PER_JUMP, f_blink_probability_per_jump);
	bcode_int(CH_BLINK_DURATION_MS, f_blink_duration_ms);
	bcode_int(CH_BLINK_GUARD_MS, f_blink_guard_ms);
//	bcode_int(CH_BLIP_PROBABILITY_PER_JUMP, f_blip_probability_per_jump);
	bcode_int(CH_BLIP_DURATION_MS, f_blip_duration_ms);
	bcode_int(CH_BLIP_GUARD_MS, f_blip_guard_ms);
	bcode_float(CH_BLIP_MAX_ANG_VELOCITY_DEG_PER_SEC, f_blip_max_ang_velocity_deg_per_sec);
	bcode_int(CH_TARG_OFFSET_DEGREES_2, f_targ_offset_degrees[2]);
	bcode_int(CH_TARG_OFFSET_DEGREES_3, f_targ_offset_degrees[3]);
	bcode_int(CH_STEER_ZERO, f_steer_zero);
	bcode_int(CH_STEER_DEAD, f_steer_dead);
	bcode_int(CH_REWARD_WINDOW_DEGREES, f_reward_window_degrees);
	bcode_int(CH_SCREEN_DISTANCE_MM, f_screen_distance_MM);
	bcode_int(CH_FAR_PLANE_DISTANCE, f_far_plane_distance);
	bcode_int(CH_CAMERA_HEIGHT, f_cam_height);
	bcode_float(CH_TARGET_DIAM_DEGREES, f_targ_size);
	bcode_int(CH_TARG_COLOR_R, f_targ_color[0]);
	bcode_int(CH_TARG_COLOR_G, f_targ_color[1]);
	bcode_int(CH_TARG_COLOR_B, f_targ_color[2]);
	bcode_float(CH_TARG_DIST, f_targ_dist);
	bcode_float(CH_TARG_HEIGHT_DEGREES, f_targ_h);
	bcode_int(CH_GP_LENGTH, f_gp_length);
	bcode_int(CH_GP_WIDTH, f_gp_width);
	bcode_int(CH_DOTS_PER_GRID, f_gp_dots);
	bcode_int(CH_GRID_POOL_SIZE, f_gp_pool);
	bcode_int(CH_PLANE_R, f_gp_plane_color[0]);
	bcode_int(CH_PLANE_G, f_gp_plane_color[1]);
	bcode_int(CH_PLANE_B, f_gp_plane_color[2]);
	bcode_int(CH_DOT_R, f_gp_dot_color[0]);
	bcode_int(CH_DOT_G, f_gp_dot_color[1]);
	bcode_int(CH_DOT_B, f_gp_dot_color[2]);
	bcode_int(CH_GRID_FLAGS, f_gp_flags);
	bcode_float(CH_DOT_SIZE, f_gp_dotsize);
	bcode_int(CH_BACKGROUND_R, f_background_color[0]);
	bcode_int(CH_BACKGROUND_G, f_background_color[1]);
	bcode_int(CH_BACKGROUND_B, f_background_color[2]);
	bcode_float(CH_BLINK_RATE, f_blink_rate);
	bcode_int(CH_BLINK_MS, f_blink_duration_ms);
	bcode_int(CH_BLINK_DEBUG, f_blink_debug);
	bcode_float(CH_BLIP_RATE, f_blip_rate);
	bcode_int(CH_BLIP_MS, f_blip_duration_ms);
	bcode_float(CH_BLIP_MAX_VEL, f_blip_max_ang_velocity_deg_per_sec);
	bcode_int(CH_BLIP_DEBUG, f_blip_debug);
	bcode_int(CH_REW_ACQUISITION_MS, f_reward_acquisition_time);
	bcode_int(CH_REW_WAIT_MAX, f_reward_wait_max);
	bcode_int(CH_REW_WAIT_MIN, f_reward_wait_min);
	bcode_int(CH_REW_RAMP, f_reward_ramp_time);
	bcode_int(CH_TARG_OFFSET_DEGREES_4, f_targ_offset_degrees[4]);
	bcode_int(CH_TARG_OFFSET_DEGREES_5, f_targ_offset_degrees[5]);
	bcode_int(CH_TARG_OFFSET_DEGREES_6, f_targ_offset_degrees[6]);
	bcode_int(CH_TARG_OFFSET_DEGREES_7, f_targ_offset_degrees[7]);
	bcode_int(CH_TARG_OFFSET_DEGREES_8, f_targ_offset_degrees[8]);
	bcode_int(CH_TARG_OFFSET_DEGREES_9, f_targ_offset_degrees[9]);
	bcode_float(CH_TARG_OFFSET_WEIGHT_0, f_targ_offset_weight[0]);
	bcode_float(CH_TARG_OFFSET_WEIGHT_1, f_targ_offset_weight[1]);
	bcode_float(CH_TARG_OFFSET_WEIGHT_2, f_targ_offset_weight[2]);
	bcode_float(CH_TARG_OFFSET_WEIGHT_3, f_targ_offset_weight[3]);
	bcode_float(CH_TARG_OFFSET_WEIGHT_4, f_targ_offset_weight[4]);
	bcode_float(CH_TARG_OFFSET_WEIGHT_5, f_targ_offset_weight[5]);
	bcode_float(CH_TARG_OFFSET_WEIGHT_6, f_targ_offset_weight[6]);
	bcode_float(CH_TARG_OFFSET_WEIGHT_7, f_targ_offset_weight[7]);
	bcode_float(CH_TARG_OFFSET_WEIGHT_8, f_targ_offset_weight[8]);
	bcode_float(CH_TARG_OFFSET_WEIGHT_9, f_targ_offset_weight[9]);
	bcode_float(CH_FIXPT_X, f_fixpt_x);
	bcode_float(CH_FIXPT_Y, f_fixpt_y);
	bcode_float(CH_FIXPT_DIAMETER, f_fixpt_diameter);
	bcode_float(CH_FIXPT_WINDOW, f_fixpt_window);
	bcode_int(CH_FIXPT_ACQTIME, f_acq_time);
	bcode_int(CH_FIXPT_ACQTIMEOUT, f_acq_timeout);
	bcode_int(CH_FIXPT_FIXTIME, f_fix_time);
	bcode_int(CH_FIXPT_FIXTIMEOUT, f_fix_timeout);
	bcode_int(CH_FIXPT_R, f_fixpt_color[0]);
	bcode_int(CH_FIXPT_G, f_fixpt_color[1]);
	bcode_int(CH_FIXPT_B, f_fixpt_color[2]);

	return status;
}


/* my_check_for_handle *********************************/

int my_check_for_handle()
{
	int status = 0;
	int handle = 0;
	if (render_check_for_handle(&handle))
	{
		if (f_handle_count == 0)
		{
			f_gp_handle = handle;
			f_handle_count = 1;
		}
		else if (f_handle_count == 1)
		{
			f_targ_handle = handle;
			f_handle_count = 2;
		}
		else if (f_handle_count == 2)
		{
			f_fixpt_handle = handle;
			f_handle_count = 3;
			status = 1;
		}
	}
	return status;
}


/* 
 * my_exp_init()
 * 
 * Initializations for overall experiment. Trial counters...
 * 
 */
 
int my_exp_init()
{
	dprintf("Initializing experiment\n");
	f_all_trials_done = 0;
	f_trial_counter = 0;
	f_incremental_jump_current_jump_magnitude = f_incremental_jump_min_degrees;
	f_incremental_jump_current_jump_magnitude_display = f_incremental_jump_current_jump_magnitude*100;
	
	alloff();
	render_frame(0);

	/* Initialize spin sliding window */
	if (f_pslider)
	{
		slider_destroy(f_pslider);
	}
	dprintf("Creating sliding window length %d\n", (long)((float)f_spin_window_ms/1000.0f*f_frames_per_second));
	f_pslider = slider_create((long)((float)f_spin_window_ms/1000.0f*f_frames_per_second));
	
	
	/* Initialize fancy counter. Only do this once - will not be cleared on reset! */
	
	if (!f_pfcounter)
	{
		f_pfcounter = fcounter_create(20, 20);
		if (f_pfcounter)
		{
			f_fcindex_alltrials = fcounter_init_counter(f_pfcounter);
			f_fcindex_completetrials = fcounter_init_counter(f_pfcounter);
			f_fcindex_breakfix = fcounter_init_counter(f_pfcounter);
			f_fcindex_pause = fcounter_init_counter(f_pfcounter);
			f_fcindex_goodjumps = fcounter_init_counter(f_pfcounter);
			f_fcindex_alljumps = fcounter_init_counter(f_pfcounter);
			f_fcindex_leftjumps = fcounter_init_counter(f_pfcounter);
			f_fcindex_rightjumps = fcounter_init_counter(f_pfcounter);
			f_fcindex_triallength = fcounter_init_recorder(f_pfcounter, 1000);
			f_fcindex_reacquisition = fcounter_init_recorder(f_pfcounter, 1000);
			f_fcindex_abssteererror = fcounter_init_recorder(f_pfcounter, 5000);
			f_fcindex_meanabssteererror = fcounter_init_recorder(f_pfcounter, 500);
			f_fcindex_meanabssteererrorcumulativetime = fcounter_init_counter(f_pfcounter);
			f_fcindex_totalrewardcounter = fcounter_init_counter(f_pfcounter);
			
			fcounter_rtvar_counter(f_pfcounter, &f_iAllTrials, f_fcindex_alltrials);
			fcounter_rtvar_pct(f_pfcounter, &f_iPctComplete,f_fcindex_completetrials, f_fcindex_alltrials);
			fcounter_rtvar_pct(f_pfcounter, &f_iPctBreak,f_fcindex_breakfix, f_fcindex_alltrials);
			fcounter_rtvar_counter(f_pfcounter, &f_iGoodJumps, f_fcindex_goodjumps);
			fcounter_rtvar_counter(f_pfcounter, &f_iAllJumps, f_fcindex_alljumps);
			fcounter_rtvar_counter(f_pfcounter, &f_iLeftJumps, f_fcindex_leftjumps);
			fcounter_rtvar_counter(f_pfcounter, &f_iRightJumps, f_fcindex_rightjumps);
			fcounter_rtvar_avg(f_pfcounter, &f_iTrialLengthMSAvg, 1.0, NULL, 1.0, f_fcindex_triallength); 
			fcounter_rtvar_avg(f_pfcounter, &f_iReacquisitionMSAvg, 1.0, NULL, 1.0, f_fcindex_reacquisition); 
			fcounter_rtvar_avg(f_pfcounter, &f_iAbsSteerError, 1.0, NULL, 1.0, f_fcindex_abssteererror); 
			fcounter_rtvar_avg(f_pfcounter, &f_iMeanAbsSteerError, 1.0, NULL, 1.0, f_fcindex_meanabssteererror); 
			fcounter_rtvar_counter(f_pfcounter, &f_iMeanAbsSteerErrorCumulativeTime, f_fcindex_meanabssteererrorcumulativetime);
			fcounter_rtvar_counter(f_pfcounter, &f_iTotalRewardCounter, f_fcindex_totalrewardcounter);			
		}
	}
	
	return 0;
}


PstSegment trial_segment_builder(void *pdata, int num)
{
	int jumpTimeFrames=0;
	PstSegment pseg;
	struct segment_builder_struct *psbs = (struct segment_builder_struct *)pdata;
	double r;

	/*
	 * Generate the jump time in frames. 
	 * Modified: Make sure the arg to log() isn't zero! 
	 */

	r = (double)(ivunif(0, 32767)+1)/32768.0;
	jumpTimeFrames = psbs->jmin + (int)(-psbs->tau * log(r));

	/*
	 * Add it to the last jump frame, that gets us this jump frame
	 */
	psbs->last += jumpTimeFrames;

	/*
	 * And create the segment, padding the jump frame with guard frames
	 */
	pseg = segtree_create_segment(psbs->last - psbs->guard_before, psbs->last + psbs->guard_after, NULL);

	return pseg;
}

int trial_action_builder(PstNode pnode, void *pdata)
{
	int irand=0;
	struct segment_builder_struct *psbs = (struct segment_builder_struct *)pdata;
	f_all_actions[n_all_actions].start_frame = pnode->pseg->s0 + psbs->guard_before;
	f_all_actions[n_all_actions].end_frame = f_all_actions[n_all_actions].start_frame;
	f_all_actions[n_all_actions].type = ACTIONTYPE_JUMP;
	f_all_actions[n_all_actions].fval = 0.0;

	/*
	 * Get a jump magnitude and direction. 
	 * 7-15-09 djs 
	 * Move assignment of jump magnitude to my_update(). This applies
	 * to the testing paradigm only!
	 */
	 
	
	/* 
	 * Don't forget to assign the action to the segment
	 */
	 
	pnode->pseg->extra = f_all_actions + n_all_actions;
	n_all_actions++;
	
	return 0;
}

int compare_actions(void *a1, void *a2)
{
	PAction *p1, *p2;
	p1 = (PAction *)a1;
	p2 = (PAction *)a2;
	return (*p1)->start_frame - (*p2)->start_frame;
}

char *action_str(PAction paction, char *str)
{
	sprintf(str, "%d-%d %s %f\n", paction->start_frame, paction->end_frame, f_cActionStr[paction->type-1], paction->fval); 
	return str;
}

int generate_trial()
{
	PstTree ptree;
	struct segment_builder_struct sbs;
	int nactions=0;
	int nblips=0;
	int nblinks=0;
	int i;
	char s[128];
	
	/*
	 * First, generate a balanced tree of jump segments. None of these segments will have any actions attached...yet. 
	 */

 	sbs.last = 0;
	sbs.jmin = (int)((float)f_ms_jump_min * 0.001f * (float)f_frames_per_second);
	sbs.tau = ((float)f_ms_jump_avg - sbs.jmin/f_frames_per_second*1000.0f) * 0.001f * (float)f_frames_per_second;
	sbs.guard_before = (int)((float)f_ms_jump_guard_before * .001f * (float)f_frames_per_second);
	sbs.guard_after = (int)((float)f_ms_jump_guard_after * .001f * (float)f_frames_per_second);

	if (f_verbose & DEBUG_BIT)
	{
		dprintf("Generate balanced tree.\n");
	}
	ptree = segtree_create_balanced(segtree_compare_segments_default, trial_segment_builder, &sbs, f_jumps_per_trial);

	/*
	 * Now we can compute how many blips and blinks we'll need, 
	 * and then we can calculate how many total actions are needed and 
	 * allocate the space for them.The total time for the trial includes 
	 * include a "tail" added after the last jump - so the trial doesn't
	 * end after the last jump occurs. Later I'll add in a segment with 
	 * length equal to the average jump time (jmin + tau), but for now 
	 * include the time in the rate calculations.
	 */

	nblips = (int)((float)(sbs.last + sbs.jmin + sbs.tau)/(float)f_frames_per_second * f_blip_rate);
	nblinks = (int)((float)(sbs.last + sbs.jmin + sbs.tau)/(float)f_frames_per_second * f_blink_rate);
	nactions = f_jumps_per_trial + 1 + nblips + nblinks;
	f_all_actions = (PAction)calloc(nactions, sizeof(struct action));
	n_all_actions = 0; /* Will be incremented as we use these up.... */
	if (f_verbose & DEBUG_BIT)
	{
		dprintf("nblips %d nblinks %d nactions %d\n", nblips, nblinks, nactions);
		if (!f_all_actions) dprintf("f_all_actions is NULL!\n");
	}
	
	/*
	 * Now run over the segtree and generate actions for each jump. Note that at each one we also flip a coin to determine 
	 * the direction of the jump. I'm also being lazy, and the traverse function
	 * will use the global array of actions, not passed data. 
	 */
	 
	if (f_verbose & DEBUG_BIT) dprintf("Generate actions for each jump.\n");
	segtree_traverse(ptree, &sbs, trial_action_builder, SEGTREE_TRAVERSE_INORDER);

	/*
	 * Add a segment for the trial's end.
	 */

	sbs.last += (sbs.jmin+(int)sbs.tau);
	f_all_actions[n_all_actions].start_frame = sbs.last;
	f_all_actions[n_all_actions].end_frame = sbs.last;
	f_all_actions[n_all_actions].type = ACTIONTYPE_TRIALEND;
	f_all_actions[n_all_actions].fval = 0;
	if (f_verbose & DEBUG_BIT) dprintf("Generate and insert trial end action.\n");
	segtree_insert(ptree, segtree_create_segment(sbs.last, sbs.last, f_all_actions + n_all_actions));
	n_all_actions++;

	/*
	 * Now the blips...
	 */

	if (f_blip_rate > 0)
	{
		int blip_duration_frames;	/* The blip is spread over this many frames */
		int blip_guard_frames;		/* guard frames on either side of blip where no jumps may occur */
		int blipcounter = 0;		/* Counter of blips added */
		int failcounter = 0;		/* Counter of failures to add blip...overlaps, get it? */
		
		blip_duration_frames = (int)((float)f_blip_duration_ms / 1000.0f * f_frames_per_second);
		blip_guard_frames = (int)((float)f_blip_guard_ms / 1000.0f * f_frames_per_second);
		
		dprintf("Generating %d blips: ", nblips);
		blipcounter = 0;
		failcounter = 0;
		while (blipcounter < nblips && failcounter < 1000)
		{
			PstSegment pseg;
			int iover;
			double r;
			int blipframe;
			
			/* 
			 * Make sure blipframe is not 0. blipframe (and blinkframe, for that matter)
			 * should be between 1 and sbs.last, inclusive. The reason is that an 
			 * action cannot start on frame 0, because frame 0 has no 'update' prior
			 * to it. Frame 0 is the start frame, and frames 1-sbs.last have updates
			 * prior to them. The last frame will not get used anyways because the 
			 * guard constrains any segment to begin prior to the last frame. 
			 * 
			 * In the code below, uivunif yields an int between 0 and 32767, inclusive. 
			 * The value of r will be in 0 < r <= 1; r cannot be 0. That means
			 * blipframe will be in 1<= blipframe <= sbs.last. 
			 */
			r = (double)(ivunif(0, 32767)+1)/32768.0;
			blipframe = (int)ceil(r*sbs.last);			
			/*printf("Check blip_frame %d seg %d-%d\n", blipframe, blipframe-blip_pad, blipframe+blip_pad);*/
			pseg = segtree_create_segment(blipframe - blip_guard_frames, blipframe + blip_duration_frames + blip_guard_frames, NULL);
			if ((iover=segtree_overlaps(ptree, pseg, 0, NULL)) == 0)
			{
				f_all_actions[n_all_actions].start_frame = blipframe;
				f_all_actions[n_all_actions].end_frame = blipframe+blip_duration_frames;
				f_all_actions[n_all_actions].type = ACTIONTYPE_BLIP;
				f_all_actions[n_all_actions].fval = f_blip_max_ang_velocity_deg_per_sec * M_PI/180.0f / f_frames_per_second * (ifuniv(1)*2-1);
				pseg->extra = f_all_actions + n_all_actions;
				segtree_insert(ptree, pseg);
				blipcounter++;
				n_all_actions++;
				dprintf("+");
			}
			else
			{
				/*printf("FAIL found %d overlaps\n", iover);*/
				segtree_destroy_segment(pseg);
				failcounter++;
				dprintf("X");
			}
		}
		dprintf("\n");
		if (failcounter >= f_max_failures)
		{
			/* TODO: Handle failure gracefully. 
			 */
			 
			dprintf("ERROR! Failed to add enough blips!\n");
		}
	}

	
	/* 
	 * ... and the blinks.
	 */
	 
	if (f_blink_rate > 0)
	{
		int blink_duration_frames;	/* The blink is spread over this many frames */
		int blink_guard_frames;		/* guard frames on either side of blink where no jumps may occur */
		int blinkcounter = 0;		/* Counter of blinks added */
		int failcounter = 0;		/* Counter of failures to add blink...overlaps, get it? */
		double r;
		int blinkframe;
		
		blink_duration_frames = (int)((float)f_blink_duration_ms / 1000.0f * f_frames_per_second);
		blink_guard_frames = (int)((float)f_blink_guard_ms / 1000.0f * f_frames_per_second);
		
		dprintf("Generating %d blinks: ", nblinks);
		blinkcounter = 0;
		failcounter = 0;
		while (blinkcounter < nblinks && failcounter < 1000)
		{
			PstSegment pseg;
			int iover;

			/* 
			 * blinkframe will be in 1<= blinkframe <= sbs.last. See discussion
			 * above for blipframe. 
			 */
			r = (double)(ivunif(0, 32767)+1)/32768.0;
			blinkframe = (int)ceil(r*sbs.last);			
			pseg = segtree_create_segment(blinkframe - blink_guard_frames, blinkframe + blink_duration_frames + blink_guard_frames, NULL);
			if ((iover=segtree_overlaps(ptree, pseg, 0, NULL)) == 0)
			{
				f_all_actions[n_all_actions].start_frame = blinkframe;
				f_all_actions[n_all_actions].end_frame = blinkframe+blink_duration_frames;
				f_all_actions[n_all_actions].type = ACTIONTYPE_BLINK;
				f_all_actions[n_all_actions].fval = 0;
				pseg->extra = f_all_actions + n_all_actions;
				segtree_insert(ptree, pseg);
				blinkcounter++;
				n_all_actions++;
				dprintf("+");
			}
			else
			{
				/*printf("FAIL found %d overlaps\n", iover);*/
				segtree_destroy_segment(pseg);
				failcounter++;
				dprintf("X");
			}
		}
		dprintf("\n");
		if (failcounter >= f_max_failures)
		{
			/* TODO: Handle failure gracefully. 
			 */
			 
			dprintf("ERROR! Failed to add enough blinks!\n");
		}
	}


	/*
	 * OK, now that's all done, and we can sort the actions by their start frame.  
	 */
	 
	f_candidate_action_index = 0;
	f_ordered_actions = (PAction *)calloc(n_all_actions, sizeof(PAction));
	for (i=0; i<n_all_actions; i++) f_ordered_actions[i] = f_all_actions+i;
	qsort(f_ordered_actions, n_all_actions, sizeof(PAction), compare_actions);
	
	
	if (f_verbose & DEBUG_BIT)
	{
		for (i=0; i<n_all_actions; i++)
		{
			dprintf("%d %d %d-%d\n", i, f_ordered_actions[i]->type, f_ordered_actions[i]->start_frame, f_ordered_actions[i]->end_frame);
	
			switch (f_ordered_actions[i]->type)
			{
				case ACTIONTYPE_JUMP:
				case ACTIONTYPE_BLIP:
				case ACTIONTYPE_BLINK:
				case ACTIONTYPE_TRIALEND:
				{
					break;
				}
				default:
				{
					
					dprintf("generate_trial: ERROR! Unknown action type (%d)\n", f_ordered_actions[i]->type);
					break;
				}
			}
	
		}
	}

	/* Finally, destroy the segtree and segments. */
	
	segtree_destroy(ptree, 1, 0);

	/* 
	 * clear unknown action flag
	 */
	f_unknown_action_flag = 0;	


	return 0;
}

void queue_actions(int frame)
{
	char s[128];
	while (f_candidate_action_index < n_all_actions && f_ordered_actions[f_candidate_action_index]->start_frame == frame)
	{
		if (f_verbose & DEBUG_BIT) dprintf("Q:%s", action_str(f_ordered_actions[f_candidate_action_index], s));
		if (n_current_actions >= MAX_CURRENT_ACTIONS)
		{
			/* 
			 * TODO: Error ecode for this situation. 
			 */
			 
			dprintf("ERROR! Too many current actions (%d)- increase MAX_CURRENT_ACTIONS!\n", n_current_actions);
			return;
		}
		/*dprintf("Queueing action: %s\n", action_str(f_ordered_actions[f_candidate_action_index], s));*/
		f_current_actions[n_current_actions++] = f_ordered_actions[f_candidate_action_index++];
	}
	return;
}

void dequeue_actions(int frame)
{
	int i;
	int count=0;
	char s[128];
	for (i=0; i<n_current_actions; i++)
	{
		if (f_current_actions[i]->end_frame == frame)
		{
			if (f_verbose & DEBUG_BIT) dprintf("D:%s", action_str(f_current_actions[i], s));
			f_current_actions[i] = (PAction)NULL;
		}
		
		if (f_current_actions[i])
		{
			f_current_actions[count++] = f_current_actions[i];
		}
	}
	n_current_actions = count;
	return;
}

int my_trial_init()
{
	int i;

	dprintf("Initializing trial %d\n", f_trial_counter+1);

	if (f_verbose & DEBUG_JOY_BIT)
	{
		f_ival_max_diff = -999;
	}

	f_frame_counter = 0;
	f_went_counter = 0;
	f_trial_end_flag = 0;
	f_per_jump_good_jump_counter = 0;
	f_per_jump_reward_counter = 0;
	f_time_last_jump = INT_MAX;
	f_cos_reward_window = cos(f_reward_window_degrees * M_PI/180);
	
	/* set experiment type. The first one that's set is the choice. */
	if (f_do_steering) f_exptype = EXPTYPE_STEERING;
	else
		if (f_do_steering_with_fixation) f_exptype = EXPTYPE_STEERING_WITH_FIXATION;
		else
			if (f_do_fixation_only) f_exptype = EXPTYPE_FIXATION_ONLY;
			else
				if (f_do_fixation_tuning) f_exptype = EXPTYPE_FIXATION_TUNING;
				else f_exptype = EXPTYPE_UNKNOWN;

	/* Assign time values to states */
	set_times("fixpt_acq", (long)f_acq_time, -1);
	set_times("fixpt_acq_fail_pause", (long)f_acq_timeout, -1);
	set_times("fixpt_hold", (long)f_fix_time, -1);
	set_times("fixpt_break_pause", (long)f_fix_timeout, -1);
	set_times("on_target_rewon", (long)f_reward_preset, (long)f_reward_random);

	/* 
	 * Update fixpt parameters prior to each trial. 
	 */
	 
	f_fixpt.xorigin = to_pixels(f_fixpt_x);
	f_fixpt.yorigin = to_pixels(f_fixpt_y);
	f_fixpt.xsize = to_pixels(f_fixpt_diameter);
	f_fixpt.ysize = to_pixels(f_fixpt_diameter);
	f_fixpt.depth = 1;
	f_fixpt.r = f_fixpt_color[0];
	f_fixpt.g = f_fixpt_color[1];
	f_fixpt.b = f_fixpt_color[2];
	render_update(f_fixpt_handle, &f_fixpt, sizeof(f_fixpt), 0);
	
	/*
	 * Reset camera position to origin on each trial!
	 */
	 
	f_cam_trajectory = f_targ_dir = 0;
	f_cam_position[0] = 0;
	f_cam_position[1] = (float)f_cam_height;
	f_cam_position[2] = 0;
	f_cam_looking[0] = -sin(f_cam_trajectory);
	f_cam_looking[1] = 0;
	f_cam_looking[2] = cos(f_cam_trajectory);
	render_camera(f_cam_position, f_cam_looking, f_cam_up);
	
	/*
	 * Set f_exptype - which type of paradigm are we running right now?
	 */
	 
	switch(f_exptype)
	{
		case EXPTYPE_STEERING:
		case EXPTYPE_STEERING_WITH_FIXATION:

			/*
			 * Determine how many target offsets are configured. 
			 * We look at them in order, stopping at the first one set to zero.
			 * Any that are nonzero after that one are ignored. 
			 */
			 
			f_targ_offset_weight_sum = 0;
			for (i=0; i<MAX_TARG_OFFSET_DEGREES; i++) 
			{
				if (f_targ_offset_weight[i] == NULLI) break;
				f_targ_offset_weight_sum += f_targ_offset_weight[i];
			}
			f_num_targ_offset_degrees = i;			
			
			/*
			 * Update fixation point and target, in case certain
			 * values were changed. 
			 */
			 
			f_target.xsize = f_target.ysize = f_targ_dist * sin(f_targ_size * M_PI / 180.0);
			f_target.d = f_targ_dist;
			f_target.h = f_targ_dist * sin(f_targ_h * M_PI / 180.0);
			f_target.r = f_targ_color[0];
			f_target.g = f_targ_color[1];
			f_target.b = f_targ_color[2];
			render_update(f_targ_handle, &f_target, sizeof(f_target), 0);
	
			/* 
			 * Allocate memory for all actions needed this trial and 
			 * initialize counters et al.  
			 */
			generate_trial();	

			break;
		case EXPTYPE_FIXATION_TUNING:
			f_cam_trajectory = f_targ_dir = 0;
			f_cam_position[0] = 0;
			f_cam_position[1] = (float)f_cam_height;
			f_cam_position[2] = 0;
			f_cam_looking[0] = -sin(f_cam_trajectory);
			f_cam_looking[1] = 0;
			f_cam_looking[2] = cos(f_cam_trajectory);
			render_camera(f_cam_position, f_cam_looking, f_cam_up);
			f_flow_duration_frames = (float)f_flow_duration_ms/1000.0f*f_frames_per_second;
			break;
		case EXPTYPE_FIXATION_ONLY:
			break;
		default:
			dprintf("my_trial_init(): exptype %d not handled\n", f_exptype);
	}

	

	return 0;
}

/*
 * my_trial_allon_center()
 * 
 * Action called during start sequence for a new trial. Here, the target is 
 * placed dead-ahead and the frame rendered. 
 * 
 */
 
int my_trial_allon_center()
{
	/* increment counterfor all trials. Save time to measure trial length in my_trial_done() */
	fcounter_increment_counter(f_pfcounter, f_fcindex_alltrials);
	f_trial_start_time = getClockTime();

	render_onoff(&f_gp_handle, HANDLE_ON, ONOFF_NO_FRAME);
	f_targ_dir = f_cam_trajectory;
	f_target.xoffset = -sin(f_targ_dir);
	f_target.zoffset = cos(f_targ_dir);
	render_update(f_targ_handle, (void *)&f_target, sizeof(TargetStruct), HANDLE_ON);
	render_frame(0);
	return 0;
}

/*
 * random_targ_offset_degrees
 *
 * Select a target offset from the current list of available offsets. The sign of the 
 * offset is also randomized (i.e. the sign of the offset in the list is irrelevant 
 * because a random +/-1 is multiplied here). 
 *
 * Modified: 11-14-08 djs Now return a floating point value. 
 * Modified:  7-15-09 djs Implement incremental jump sizes for training
 *                        purposes. If flag f_incremental_jumps is nonzero 
 *                        then return the current value of 
 *                        f_incremental_jumps_current_jump_magnitude. 
 *                        If f_jump_one_sided is 0 then draw a sign at
 *                        random. If f_jump_one_sided is 1 (-1) then set the 
 *                        sign to +1 (-1). 
 * Modified:  7-16-09 djs Make f_jump_one_sided apply to both incremental 
 *                        and weighted jumps.  
 * Modified:  8-14-09 djs Make f_jump_one_sided option "2" to mean "pong"
 *                        style alternate L/R jumps. 
 * 
 * Returns: jump magnitude and sign, as a float. 
 *
 */

float random_targ_offset_degrees()
{
	int irand;
	int i;
	int iwsum = 0;
	int isgn;
	float jump_angle_degrees = 0.0;
	
	/*
	 * isgn is the direction of the jump: 1(-1) means right (left). 
	 * f_jump_one_sided can be -1/1 - one sided to the left/right, 
	 * it can be 0 - random jump direction
	 * it can be 2 - alternate left/right "pong" style. 
	 */

	switch (f_jump_one_sided)
	{
		case 1:
		case -1:
		{
			isgn = f_jump_one_sided;
			break;
		}
		case 0:
		{
			isgn = ifuniv(1)*2-1;
			break;
		}
		case 2:
		{
			if (f_jump_pong_sgn==1)
			{
				isgn = f_jump_pong_sgn = -1;
			}
			else
			{
				isgn = f_jump_pong_sgn = 1;
			}
		}
	}
				 
	if (f_use_incremental_jumps && f_exptype == EXPTYPE_STEERING_WITH_FIXATION)
	{
		jump_angle_degrees = isgn * f_incremental_jump_current_jump_magnitude;
	}
	else
	{
		irand = ifuniv(f_targ_offset_weight_sum-1);
	
		for (i=0; i<f_num_targ_offset_degrees; i++)
		{
			iwsum += f_targ_offset_weight[i];
			if (irand < iwsum) break;
		}
	
		if (i == f_num_targ_offset_degrees)
		{
			dprintf("ERROR! irand=%d f_targ_offset_weight_sum = %d\n", irand, f_targ_offset_weight_sum); 
			i = f_num_targ_offset_degrees-1;
		}
	
		jump_angle_degrees = f_targ_offset_degrees[i] * isgn;
	}
	
	return jump_angle_degrees;
}

/*
 * my_trial_allon_offset
 *
 * This function sets an initial offset to the target when the trial begins. Its not really
 * a jump - when the trial starts the target is placed dead-center prior to motion starting. 
 * There is a pause and this method is called (its a state action) to set an initial offset. 
 * Choose an offset from the list of available offsets and move the target. 
 * 
 * Returns: 0
 *
 */

int my_trial_allon_offset()
{
	f_targ_dir = f_cam_trajectory + random_targ_offset_degrees() * M_PI/180.0f;
	f_target.xoffset = -sin(f_targ_dir);
	f_target.zoffset = cos(f_targ_dir);
	render_update(f_targ_handle, (void *)&f_target, sizeof(TargetStruct), HANDLE_ON);
	render_frame(0);
	return 0;
}

/*
 * my_trial_done(int success)
 * 
 * Action called after a trial completes. Closes analog window and sets flag
 * for reward loop, indicating that there's no steering so stop the reward
 * business. Do cleanup of stuff allocated at the beginning of the trial. 
 */

int my_trial_done(int success)
{
	float meanabssteererror;
	if (success==1) f_trial_counter++;
	if (f_trial_counter == f_ntrials) f_all_trials_done = 1;

	alloff();
	render_frame(0);	/* issues FRAME command - expect a WENT */


	/*
	 * compute steering error average and clear the recorder...
	 */
	 
	if (f_exptype == EXPTYPE_STEERING_WITH_FIXATION || f_exptype == EXPTYPE_STEERING)
	{
		int i;
		i = fcounter_get_recorder(f_pfcounter, f_fcindex_abssteererror, &meanabssteererror, NULL);
		if (fcounter_get_recorder(f_pfcounter, f_fcindex_abssteererror, &meanabssteererror, NULL) > 0)
		{
			fcounter_clear_recorder(f_pfcounter, f_fcindex_abssteererror);
			fcounter_record(f_pfcounter, f_fcindex_meanabssteererror, meanabssteererror); 
			fcounter_addto_counter(f_pfcounter, f_fcindex_meanabssteererrorcumulativetime, getClockTime() - f_time_last_jump);
			dprintf("count %d last jump %d ctime %d\n", i, f_time_last_jump,  getClockTime() - f_time_last_jump);
		}
	}

	switch(f_exptype)
	{
		case EXPTYPE_STEERING:
			f_going = 0;	/* this tells reward loop to stop rewarding */
			if (w_flags & (W_ISOPEN|W_NULLOPEN))
			{
				awind(CLOSE_W);
			}
			break;
		case EXPTYPE_STEERING_WITH_FIXATION:
			f_going = 0;	/* this tells reward loop to stop rewarding */
			wd_cntrl(WIND0, WD_OFF);
			switch (success)
			{
				case 1: 
				{
					fcounter_increment_counter(f_pfcounter, f_fcindex_completetrials);
					fcounter_record(f_pfcounter, f_fcindex_triallength, getClockTime() - f_trial_start_time); 
					break;
				}
				case 0:
				{
					fcounter_increment_counter(f_pfcounter, f_fcindex_breakfix);
					fcounter_record(f_pfcounter, f_fcindex_triallength, (float)(getClockTime() - f_trial_start_time)); 
					break;
				}
				case -1:
				{
					fcounter_increment_counter(f_pfcounter, f_fcindex_pause);
					break;
				}
			}

			/*
			 * Trigger computation of rtvars -- including averages...
			 */

			fcounter_rtvar_trigger(f_pfcounter, 1);

			/* 
			 * Handling for incremental jumps
			 */
			 					
			if (f_use_incremental_jumps)
			{
				if (f_per_jump_good_jump_counter < f_incremental_jump_up_threshhold)
				{
					f_incremental_jump_bad_trial_counter++;
					if (f_incremental_jump_bad_trial_counter == f_incremental_jump_dn_threshhold)
					{
						f_incremental_jump_bad_trial_counter = 0;
						f_incremental_jump_current_jump_magnitude -= f_incremental_jump_step_degrees;
						if (f_incremental_jump_current_jump_magnitude < f_incremental_jump_min_degrees)
						{
							f_incremental_jump_current_jump_magnitude = f_incremental_jump_min_degrees;
						}
						f_incremental_jump_current_jump_magnitude_display = f_incremental_jump_current_jump_magnitude * 100;
					}
				}
				else
				{
					f_incremental_jump_bad_trial_counter = 0;
				}
			}
			
			/* 
			 * Write report
			 */
			 
			my_report(0);
			
			break;
		case EXPTYPE_FIXATION_TUNING:
		case EXPTYPE_FIXATION_ONLY:
			wd_cntrl(WIND0, WD_OFF);
			if (success>0) f_reward_flag = 1;
			break;
		default:
			dprintf("my_trial_done(): exptype %d not handled\n", f_exptype);
	}

	/* Free memory allocated for actions */
	if (f_all_actions) 
	{
		free(f_all_actions);
		f_all_actions = NULL;
	}
	if (f_ordered_actions)
	{
		free(f_ordered_actions);
		f_ordered_actions = NULL;
	}
	n_all_actions = 0;
	return 0;
}


char *my_pct(unsigned int i_numerator, unsigned int i_denominator, char *result)
{
	float pct;
	if (i_denominator==0)
	{
		strcpy(result, "@@.@");
	}
	else
	{
		char *spctsign="%"; /* Can't use double pct signs in sprintf--spot catches it and thinks its the start of the state set spec! */
		pct = (float)i_numerator / (float)i_denominator;
		sprintf(result, "%4.1f%s", pct*100, spctsign);
	}
	return result;
}


/*
 * my_report(int full)
 * 
 * Writes report to screen. If full is zero, then just simple intertrial
 * reporting. Write a more complete report if full is nonzero.
 */
 
void my_report(int full)
{
	char spct1[24];
	char spct2[24];
	char spct3[24];
	char sline[256];
	unsigned int t0, t1, t2, t3;
	t0 = fcounter_get_counter(f_pfcounter, f_fcindex_alltrials);
	t1 = fcounter_get_counter(f_pfcounter, f_fcindex_completetrials);
	t2 = fcounter_get_counter(f_pfcounter, f_fcindex_breakfix);
	t3 = fcounter_get_counter(f_pfcounter, f_fcindex_pause);
	dprintf("\nTrials: ALL |   COMPLETE   |   BREAKFIX  |   PAUSE\n");
	sprintf(sline, "        %3d | %3d (%s) | %3d (%s) | %3d (%s)\n", 
			t0, t1, my_pct(t1, t0, spct1), t2, my_pct(t2, t0, spct2), t3, my_pct(t3, t0, spct3));
	dprintf("%s", sline);
	return;
}

  
/* 
 * alloff()
 * 
 * Turns off groundplane and target. Does not issue a render or frame. 
 * 
 */
 
int alloff()
{
	if (f_gp_handle)
		render_onoff(&f_gp_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	if (f_targ_handle)
		render_onoff(&f_targ_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	if (f_fixpt_handle)
		render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	return 0;
}
  	

/*
 * my_update()
 * 
 */
   
int my_update()
{
	int ival;		/* input joystick value, shifted by zero value */
	float nval;		/* normalized joystick value */
	int ijoyh;		/* in case we're interrupted, better save the value used. */
	int i;
	int status = 0;
	int targ_update_flag = 0;	/* set to 1 if target needs updating.*/

	/* We are always going when in the update loop. */
	f_going = 1;
	
	/* Get joystick value and calculate new camera trajectory. */	

	ijoyh = joystick_value();		// joystick_value will return fakejoy value if fakejoy is enabled
	ival = ijoyh - f_steer_zero;		// subtract off zero point

	if (f_verbose & DEBUG_JOY_BIT)
	{
		int diff = abs(ival-f_ival_last);
		if (diff > f_ival_max_diff) f_ival_max_diff = diff;
		dprintf("diff %d max %d\n", diff, f_ival_max_diff);
		f_ival_last = ival;
	}

	/* Check for spinning. */
	f_pslider->wadd(f_pslider, ijoyh);	// used for spin detection
	if (f_pslider && f_pslider->isFull(f_pslider) && 
		(f_pslider->wmax(f_pslider)-f_pslider->wmin(f_pslider)) < f_spin_deviation)
	{
		if (f_spinning == 0)
		{
			if (f_verbose & DEBUG_REWARD_BIT)
			{	 
				dprintf("spinning\n");
			}
			ecode(SPINNINGCD);
			f_spinning = 1;
		}
	}
	else
	{
		if (f_spinning == 1)
		{
			if (f_verbose & DEBUG_REWARD_BIT)
			{	 
				dprintf("not spinning\n");
			}
			ecode(NOTSPINNINGCD);
			f_spinning = 0;
		}
	}


	if (abs(ival) > f_steer_dead)
	{
		if (ival > 0)
		{
			nval = (float)(ival - f_steer_dead)/(1024.f - (float)f_steer_dead);
		}
		else 
		{
			nval = (float)(ival + f_steer_dead)/(1024.f - (float)f_steer_dead);
		}
		f_cam_trajectory += nval * f_steer_max_degrees * M_PI/180.f;		
	}

	/*
	 * See if any current actions require handling.
	 */

	f_jumpflag = 0;	/* this will be set only if a jump occurs below */
	queue_actions(f_went_counter+1);
	for (i=0; i<n_current_actions; i++)
	{
		char tmpstr[128];
		switch (f_current_actions[i]->type)
		{
			case ACTIONTYPE_JUMP:
			{
				/*
				 * ACTIONTYPE_JUMP should have length of 0 frames (start_frame = end_frame), so this if stmt is
				 * probably not necessary. Test it anyways. 
				 */
				if ((f_went_counter+1) == f_current_actions[i]->start_frame)
				{
					int code;
					
					/*
					 * Get jump magnitude. We fetch the value here in the
					 * testing paradigm (its done at the time of the trial
					 * generation in the regular data paradigm) so that we
					 * can handle the cases where incremental jump sizes are
					 * being used. In those cases the jump magnitude cannot 
					 * be determined at the beginning of the trial - it must
					 * be done now. 
					 */
					 
					f_current_actions[i]->fval = random_targ_offset_degrees();
					
					code = JLEFTCD;
					if (f_current_actions[i]->fval > 0) code = JRIGHTCD;
					if (ecode(code)) status = BADCD;
					
					bcode_float(CH_JUMP_SIZE, f_current_actions[i]->fval);
					bcode_float(CH_JUMP_ACTUAL, f_targ_dir*180.0f/M_PI - f_current_actions[i]->fval);
					f_targ_dir = f_cam_trajectory + f_current_actions[i]->fval * M_PI / 180.0f;
					targ_update_flag = 1;
					f_jumpflag = -1;
					if (f_current_actions[i]->fval > 0) f_jumpflag = 1;
					if (f_verbose & DEBUG_REWARD_BIT) dprintf("update: jump flag wc = %d\n", f_went_counter); 
				}
				break;
			}
			case ACTIONTYPE_BLIP:
			{
				float angle = 0;
				if ((f_went_counter+1) == f_current_actions[i]->start_frame)
				{
					float maxav = f_blip_max_ang_velocity_deg_per_sec;
					if (f_current_actions[i]->fval < 0) maxav *= -1; 
					ecode(BLIPBEGINCD);
					bcode_float(CH_BLIP_OMEGA_MAX, maxav);
					bcode_uint(CH_BLIP_DURATION_FRAMES, f_current_actions[i]->end_frame - f_current_actions[i]->start_frame);
				}
				else if ((f_went_counter+1) == f_current_actions[i]->end_frame)
				{
					ecode(BLIPENDCD);
				}

				/*
				 * djs 11-06-08
				 * Fix bug in blip below. Was using square wave - assigning displacement equal to max angular velocity for each frame 
				 * in the blip period. 
				 * 
				 * Now, take the blip period to be a half sinusoid, where 'frac' is computed below to run over [0, 1] during the 
				 * blip period. Also note that max angular velocity is stored in fval already converted to radians per frame. 
				 */				
				if ((f_went_counter+1) >= f_current_actions[i]->start_frame && (f_went_counter+1) <= f_current_actions[i]->end_frame)
				{
					float frac;
					frac = (float)(f_went_counter+1-f_current_actions[i]->start_frame)/(float)(f_current_actions[i]->end_frame - f_current_actions[i]->start_frame);
					angle = f_current_actions[i]->fval * sin(frac * M_PI);
					if (f_blip_debug)
					{
						dprintf("blip w+1 %d start %d end %d f*100 %d angle(deg*100) %d max*100 %d\n", 
								f_went_counter+1, 
								f_current_actions[i]->start_frame,
								f_current_actions[i]->end_frame,
								(int)(frac*100),
								(int)(angle * 180/M_PI * 100), 
								(int)(f_current_actions[i]->fval * 180/M_PI * 100));
					}
								
				}
				f_targ_dir += angle;
				f_cam_trajectory += angle;
				targ_update_flag = 1;
				break;
			}
			case ACTIONTYPE_BLINK:
			{
				if ((f_went_counter+1) == f_current_actions[i]->start_frame)
				{
					render_onoff(&f_targ_handle, HANDLE_OFF, ONOFF_NO_FRAME);
					ecode(BLINKBEGINCD);
				}
				else if ((f_went_counter+1) == f_current_actions[i]->end_frame)
				{
					ecode(BLINKENDCD);
					render_onoff(&f_targ_handle, HANDLE_ON, ONOFF_NO_FRAME);
				}
				break;
			}
			case ACTIONTYPE_TRIALEND:
			{
				f_trial_end_flag = 1;
				break;
			}
			default:
			{
				if (!f_unknown_action_flag) 
				{
					dprintf("my_update: ERROR! Unknown action type (%d)\n", f_current_actions[i]->type);
					f_unknown_action_flag = 1;
				}

				/* TODO: Error ecode for this situation. */
				break;
			}
		}
	}
	dequeue_actions(f_went_counter+1);


	/*
	 * record steering error if we are outside of the holdoff period, but not
	 * if the jumpflag has been set.
	 */
	 
	if (!f_jumpflag && f_time_last_jump < INT_MAX &&  (getClockTime() > (f_time_last_jump + f_abssteererror_holdoff)))
	{
		fcounter_record(f_pfcounter, f_fcindex_abssteererror, fabs(f_targ_dir - f_cam_trajectory)*180.0/M_PI);
	}

	/* Update target if necessary */
	if (targ_update_flag)
	{
		f_target.xoffset = -sin(f_targ_dir);
		f_target.zoffset = cos(f_targ_dir);
		render_update(f_targ_handle, (void *)&f_target, sizeof(TargetStruct), 0);
	}


	/* Update camera */

	f_cam_looking[0] = -sin(f_cam_trajectory);
	f_cam_looking[1] = 0;
	f_cam_looking[2] = cos(f_cam_trajectory);

	f_cam_position[0] += f_cam_looking[0] * f_speed;
	f_cam_position[1] = (float)f_cam_height;
	f_cam_position[2] += f_cam_looking[2] * f_speed;

	render_camera(f_cam_position, f_cam_looking, f_cam_up);

	/* Now render */			
	render_frame(0);

	/* ecodes */
	/* 9-13-07 target direction code - use actual targ_dir, not offset! Also, convert to degrees and mult * 10 */
	ecode(UPDCD);
	bcode_uint(CH_JOY, ijoyh);
	bcode_float(CH_TRAJ, f_cam_trajectory * 180.0f/M_PI);
	bcode_float(CH_TARG, f_targ_dir * 180.0f/M_PI);
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
 * A counter, f_went_counter, is incremented each time a WENT is received. In 
 * addition a frame counter is also incremented. The frame counter may not be 
 * the same as the went counter if the paradigm was paused or if there were 
 * missed frames. 
 * 
 * When my_update() specifies a jump to occur, a flag (f_jumpflag) is set. When 
 * the subsequent WENT is received here the value of the went counter (after
 * it is incremented) is saved in f_went_counter_last_jump. This is used in 
 * the reward loop to detect when steering is off target due to a jump. The 
 * jumpflag is cleared here as well. 
 * 
 * When my_update encounters the end of trial 'action', a flag (f_trial_end_flag)
 * is set. When the subsequent WENT is received this function returns a special
 * value to indicate end of trial - the state set breaks out of the update 
 * loop at that point. 
 * 
 * Returns 0 if no WENT found, 1 if a WENT was received (and no trial_end_flag),
 * or 2 if WENT was found and a trial end flag was set. 
 */

int my_check_for_went()
{
	int frames = 0;
	int status;
	f_wstatus = 0;
	status = render_check_for_went(&frames);
	if (status < 0)
	{
		// ERROR! not clear what to do ...
		dprintf("ERROR in render_check_for_went!\n");
		return -1;
	}
	else if (status == 0)
	{
		f_went_cycles++;
	}
	else if (status == 1)
	{
		// went was found. 'frames' has frame count
		f_wstatus = 1;
		f_frame_counter += frames;
		f_went_counter += 1;
		if (f_verbose & DEBUG_REWARD_BIT) dprintf("WENT wc: %d\n", f_went_counter);
		if (f_jumpflag)
		{
			f_went_counter_last_jump = f_went_counter;
			f_time_last_jump = getClockTime();
			fcounter_increment_counter(f_pfcounter, f_fcindex_alljumps);
			fcounter_increment_counter(f_pfcounter, f_jumpflag > 0 ? f_fcindex_rightjumps : f_fcindex_leftjumps);
			if (f_verbose & DEBUG_REWARD_BIT) dprintf("WENT jumpflag set wc_last_jump %d\n", f_went_counter_last_jump);
			if (f_per_jump_reward_counter > 0 && !f_spinning)
			{
				f_per_jump_good_jump_counter++;
				fcounter_increment_counter(f_pfcounter, f_fcindex_goodjumps);
				if (f_use_incremental_jumps)
				{					
					if ((f_per_jump_good_jump_counter % f_incremental_jump_up_threshhold) == 0)
					{
						f_incremental_jump_current_jump_magnitude += f_incremental_jump_step_degrees;
						f_incremental_jump_current_jump_magnitude_display = f_incremental_jump_current_jump_magnitude * 100;
					}
				}
			}
			f_per_jump_reward_counter = 0;
			f_jumpflag = 0;
		}
			
		ecode(WENTCD);
				
		// TEST - if frames were missed, dprint it and the cycles...
		if (frames > 1)
		{
			dprintf("Missed %d frames (%d check_went cycles)\n", frames, f_went_cycles);
			ecode(MISSEDCD);
		}
		f_went_cycles = 0;

		if (f_trial_end_flag)
		{
			f_wstatus = 2;
			ecode(TRIALENDCD);
		}

	}
	return f_wstatus;
}


/*
 * joystick_value()
 * 
 * Returns the joystick value. If fake_joy is enabled, the hocus pocus is used
 * to determine a value, otherwise the horizontal value 'joyh' is used. 
 */


int joystick_value()
{
	int value=0;
	if (!f_fake_joy_enable) 
	{
		value = joyh;
	}
	else 
	{
		if (1 == f_fake_joy_enable)
		{
			value = f_fake_joy_value;
		}
		else 
		{
			float C = f_fake_joy_enable;
			float K = M_PI/180.0f;

			/*
			 * Experimental fake steering
			 */

			value = -1*(f_cam_trajectory-f_targ_dir)/(C*K*f_steer_max_degrees)*(1024-f_steer_dead) + f_steer_dead + f_steer_zero;
			if (value < 0) value = 0;
			if (value > 2047) value=2047;
		}
	}
	return value;
}


/*
 * stpause(int isPaused)
 * 
 * Called when a paradigm is paused or stopped. Closes/opens
 * analog window as needed.
 *  
 * We have to close the analog window here in case a reset or 
 * file close happens. Call with isPaused=1 when pausing, and
 * make sure to call again with isPaused=0 when done with pause. 
 * Also should make sure that if user pauses, then does 
 * ResetStates that the analog window will be closed and reopened correctly. 
 * 
 */
 
int stpause(int isPaused)
{
	if (isPaused)
	{
		/* Only close window if its open now */
		/*if (i_b->i_flags & I_WINDOPEN)*/
		if (w_flags & (W_ISOPEN|W_NULLOPEN))
		{
			awind(CLOSE_W);
		}
	}
	else
	{
		/* Only open window if its closed now */
		/*if (!(i_b->i_flags & I_WINDOPEN))*/
		if (!(w_flags & (W_ISOPEN|W_NULLOPEN)))
		{
			awind(OPEN_W);
		}
	}
	return 0;
}





/**********************************************************************
 *
 * Reward state set actions and escape functions. 
 * 
 **********************************************************************/ 



/*
 * reward_init()
 * 
 * Initialize reward loop variables. 
 */

int reward_init()
{
	f_reward_flag = 0;
	f_last_check_steering_went_counter = 0;
	f_last_check_steering_value = -1;
	f_next_reward_time = 0;
	f_last_reward_time = INT_MIN;
	f_reward_on_target_sum = 0;
	f_grace_period_expire = INT_MAX;
	ecode(NOTSPINNINGCD);
	f_spinning = 0;
	if (f_pslider) f_pslider->clear(f_pslider);
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("reward_init\n");
}

/* 
 * check_steering
 * 
 * Test if steering is within reward window. The check should be done only when
 * the went counter changes, 
 * Return 1 if steering is within reward window
 * Return -1 if steering is outside of window, no jump
 * return 0 if steering is outside of window, but there was a jump
 * 
 */ 
 
int check_steering()
{
	double dot;
	int status = -1;
	if (f_went_counter == f_last_check_steering_went_counter)
	{
		if (f_verbose & DEBUG_REWARD_BIT && f_went_counter-f_went_counter_last_jump <5) 
		{
			dprintf("cs: last (%d) wc %d lastcswc %d\n", f_last_check_steering_value, f_went_counter, f_last_check_steering_went_counter); 
		}
		return f_last_check_steering_value;
	}
	else
	{
		dot = cos(f_cam_trajectory)*cos(f_targ_dir) + sin(f_cam_trajectory)*sin(f_targ_dir);
		if (dot > f_cos_reward_window)
		{
			status = 1;
			f_reward_spatial_fraction = (dot - f_cos_reward_window)/(1.0f - f_cos_reward_window);
		}
		else
		{
			/* Was there a jump? */
			status = -1;
			if (f_jumpflag || (f_went_counter == f_went_counter_last_jump))
			{
				status = 0;
			}
		}
		if (f_verbose & DEBUG_REWARD_BIT && f_jumpflag) dprintf("cs: jumpflag set s: %d wc: %d wc_last_jump %d last checked wc %d\n", status, f_went_counter, f_went_counter_last_jump, f_last_check_steering_went_counter);
	}
	f_last_check_steering_value = status;
	f_last_check_steering_went_counter = f_went_counter;

	if (f_verbose & DEBUG_REWARD_BIT && f_went_counter-f_went_counter_last_jump <5) 
	{
		dprintf("   check_steering: status %d wc %d last_jump_wc %d\n", status, f_went_counter, f_went_counter_last_jump);
		if (f_jumpflag) dprintf("   check_steering: jumpflag is set\n");
	}

	return status;
}


/*
 * get_next_reward_time()
 * 
 * Computes the wait time until the next reward comes. Assumes that this
 * is called immediately after a reward was given. The cumulative time on 
 * target is computed, and the reward time computation uses the reward ramp. 
 * The current clock time is added to the wait time, 
 * and the result is saved as f_next_reward_time.
 */

int get_next_reward_time()
{
	float v;
	int wait;
	if (f_last_reward_time > 0)
	{
		f_reward_on_target_sum += (getClockTime() - f_last_reward_time);
	}
	else
	{
		f_reward_on_target_sum = 0;
	}
	v = (float)f_reward_on_target_sum/(float)f_reward_ramp_time;
	if (v>1) v=1;
	wait = f_reward_wait_max - v*(f_reward_wait_max - f_reward_wait_min);
	f_next_reward_time = getClockTime() + wait;
	f_last_reward_time = getClockTime();
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("get_next_reward_time\tnow=%d\twait=%d\tnext=%d wc=%d\n", getClockTime(), wait, f_next_reward_time, f_went_counter);
	return 0;
}


/*
 * off_target()
 * 
 * Action for off_target state. Clears any vars that track on target stuff. 
 */

int off_target()
{
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("off_target\tnow=%d\n", getClockTime());
	f_reward_on_target_sum = 0;
	/*f_next_reward_time = INT_MAX;*/
	return 0;
}


/*
 * check_steering_and_wait_time()
 * 
 * Test steering by calling check_steering(). If off target return 0 or -1, depending
 * on what check_steering says. If on target  (1), then
 * check if the reward wait time is passed. If it has, then return 2 (allowing
 * another reward), otherwise return 1. 
 * 
 * Uses f_next_reward_time, which was computed in get_reward_wait_time() 
 */

int check_steering_and_wait_time()
{
	int status = check_steering();
	int cstatus;
	cstatus = status;
	if (status == 1)
	{
		if (f_reward_use_spatial_ramp)
		{
			f_next_reward_time = f_last_reward_time + f_reward_wait_min + (1-f_reward_spatial_fraction)*(f_reward_wait_max - f_reward_wait_min);
		}
		if (getClockTime() >= f_next_reward_time)
		{
			status = 2;
		}
	}
//	if (f_verbose & DEBUG_REWARD_BIT && (f_jumpflag || (f_went_counter-f_went_counter_last_jump) <5))
	if (f_verbose & DEBUG_REWARD_BIT)
	{
		dprintf("csw: cs %d s %d wc %d ljwc %d\n", cstatus, status, f_went_counter, f_went_counter_last_jump);
	}
	return status;
}

int my_target_reacq()
{
	fcounter_record(f_pfcounter, f_fcindex_reacquisition, getClockTime() - f_time_last_jump);
	return 0;
}


/*
 * get_grace_time()
 * 
 * Computes when grace time expires and saves that time in f_grace_period_expire.
 */

int get_grace_time()
{
	f_grace_period_expire = getClockTime() + f_reward_acquisition_time;
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("get_grace_time\tnow=%d\tgrace_exp=%d\n", getClockTime(), f_grace_period_expire);
	return 0;
}

/*
 * check_grace_time()
 * 
 * Returns 1 if clock is still within grace period, 0 otherwise. 
 */

int check_grace_time()
{
	if (getClockTime() > f_grace_period_expire) return 0;
	return 1;
} 


int reward_on()
{
	dio_on(REW);
	f_per_jump_reward_counter++;
	fcounter_increment_counter(f_pfcounter, f_fcindex_totalrewardcounter);	
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("reward_on\tnow=%d\n", getClockTime());
	return 0;
}

int reward_off()
{
	dio_off(REW);
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("reward_off\tnow=%d\n", getClockTime());
	return 0;
}


int my_fixpt_window()
{
	int status = 0;
	wd_src_pos(WIND0, WD_DIRPOS, 0, WD_DIRPOS, 0);
	wd_pos(WIND0, (long)(f_fixpt_x*10.0), (long)(f_fixpt_y*10.0));
	wd_siz(WIND0, (long)(f_fixpt_window*10.0), (long)(f_fixpt_window*10.0));
	wd_cntrl(WIND0, WD_ON);
	wd_src_check(WIND0, WD_SIGNAL, 0, WD_SIGNAL, 1);
	return status;
}

int fixation_reward()
{
	f_reward_flag = 1;
	return 0;
}

int flow_init()
{
	f_went_counter = 0;
	return 0;
}

   
int tun_update()
{
	int status=0;

	/*
	 * Update camera trajectory -- the direction its moving -- and then
	 * update camera position. Camera looking direction does not change
	 * in this paradigm!
	 * 
	 * f_went_counter is incremented when the WENT's from this function's
	 * FRAME commands are received. Consequently, we use f_went_counter
	 * to determine where we are in this trial, whether we are in a blip, 
	 * etc. The question of whether the trial is ended is answered in 
	 * my_check_for_went(), not here. 
	 */
	 
	f_cam_position[0] += f_speed * sin(f_cam_trajectory * M_PI/180.0f);
	f_cam_position[2] += f_speed * cos(f_cam_trajectory * M_PI/180.0f);
	render_camera(f_cam_position, f_cam_looking, f_cam_up);
	render_frame(0);

	ecode(UPDCD);

	return status;	
}


int tun_check_for_went()
{
	int frames = 0;
	int status;
	f_wstatus = 0;
	status = render_check_for_went(&frames);
	if (status < 0)
	{
		// ERROR! not clear what to do ...
		dprintf("ERROR in render_check_for_went!\n");
		return -1;
	}
	else if (status == 0)
	{
		f_went_cycles++;
	}
	else if (status == 1)
	{
		// went was found. 'frames' has frame count
		f_wstatus = 1;
		f_frame_counter += frames;
		f_went_counter += 1;

		ecode(WENTCD);
				
		// TEST - if frames were missed, dprint it and the cycles...
		if (frames > 1)
		{
			dprintf("Missed %d frames (%d check_went cycles)\n", frames, f_went_cycles);
			ecode(MISSEDCD);
		}
		f_went_cycles = 0;

		if (f_went_counter == f_flow_duration_frames)
		{
			f_wstatus = 2;
			ecode(TRIALENDCD);
		}

	}
	return f_wstatus;
}





/* REX state set starts here 
 * djs 11-5-08
 * Modified id from 402->403
 * djs 11-6-08
 * Modified id from 403->404
 * djs 11-14-08 st17, 404->405
 * djs 01-23-09 st18af 405->406
 * djs 2-17-09 st19aft 406->407
 */

%%

id 407
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
		do my_trial_init()	/* initialization for a single trial */
		to trial_init_pause
	trial_init_pause:
		time 1000
		to trial_init_done
	trial_init_done:
		to fixpt_on on EXPTYPE_STEERING_WITH_FIXATION = f_exptype
		to fixpt_on on EXPTYPE_FIXATION_ONLY = f_exptype
		to fixpt_on on EXPTYPE_FIXATION_TUNING = f_exptype
		to trial_allon_center on EXPTYPE_STEERING = f_exptype

/*  
 * Fixation handling states below.
 * Escape(s) from the state 'fixpt_hold' should vary depending on 
 * experiment type in 'f_exptype'. 
 */ 
 
	fixpt_on:
		do render_onoff(&f_fixpt_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT);
		to fixpt_window on 1 % render_check_for_went
	fixpt_window:
		time 20
		do my_fixpt_window()
		to fixpt_acq
	fixpt_acq:
		time 4000
		to fixpt_pause_clear on +PSTOP & softswitch
		to fixpt_hold on -WD0_XY & eyeflag
		to fixpt_acq_fail
	fixpt_pause_clear:
		do render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);
		to fixpt_paused 
	fixpt_paused:
		to trial_init on -PSTOP & softswitch
	fixpt_acq_fail:
		do render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);
		to fixpt_acq_fail_pause on 1 % render_check_for_went
	fixpt_acq_fail_pause:
		time 500
		do wd_cntrl(WIND0, WD_OFF);
		to fixpt_on
	fixpt_hold:
		time 500
		to fixpt_noise on +WD0_XY & eyeflag
		to fixpt_transition
	fixpt_transition:
		to fixpt_success on EXPTYPE_FIXATION_ONLY = f_exptype
		to trial_allon_center on EXPTYPE_STEERING_WITH_FIXATION = f_exptype
		to tun_gp_on on EXPTYPE_FIXATION_TUNING = f_exptype
	fixpt_noise:
		do render_onoff(&f_fixpt_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT);
		to fixpt_acq_fail_pause on 1 % render_check_for_went
	fixpt_success:
		do render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);
		to fixpt_reward on 1 % render_check_for_went
	fixpt_reward:
		time 500
		do fixation_reward()
		to trial_init

	/* 
	 * Tuning states. These states are entered at tun_gp_on after fixation
	 * has occurred, if f_exptype is EXPTYPE_FIXATION_TUNING
	 */

	tun_gp_on:
		code FIXCD
		do render_onoff(&f_gp_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT);
		to tun_gp_wait on 1 % render_check_for_went
	tun_gp_wait:
		time 500
		to tun_trial_fail on +WD0_XY & eyeflag
		to tun_flow_on
	tun_trial_fail:
		do my_trial_done(0)
		to trial_init on 1 % render_check_for_went
	tun_flow_on:
		code FLOWCD
		do flow_init()
		to tun_update
	tun_update:
		do tun_update()
		to tun_update_wait
	tun_update_wait:
		to tun_trial_fail_wait on +WD0_XY & eyeflag
		to tun_pause_wait on +PSTOP & softswitch 
		to tun_update on 1 % tun_check_for_went
		to tun_trial_success on 2 = f_wstatus
	tun_trial_fail_wait:
		to tun_trial_fail on 1 % render_check_for_went
	tun_pause_wait:
		to tun_pause_clear on 1 % render_check_for_went
	tun_pause_clear:
		do my_trial_done(-1)
		to tun_paused
	tun_paused:
		to trial_init on -PSTOP & softswitch 
	tun_trial_success:
		do my_trial_done(1)
		to trial_init on 1 % render_check_for_went

	/*
	 * Trial beep states are accessed only if the var f_use_trial_beep
	 * is set to 1. Originally the beep was added for use in the training
	 * rig as a reminder to the subject to "wake up" and pay attention. 
	 * Ultimately, it never worked well because of a bug in the electronics
	 * for the buzzer and juicer....basically this was a bad thing and 
	 * the beep isn't used.
	 */
	 
	trial_beep_on:
		dio_on(BEEP)
		time 100
		to trial_beep_off
	trial_beep_off:
		dio_off(BEEP)
		to trial_allon_offset
		
	/*
	 * Steering states below. These states manage the presentation of the
	 * steering world and the update cycle. Note that rewards here are 
	 */		
		
	trial_allon_center:
		do my_trial_allon_center()
		to fixpt_break_wait on +WD0_XY & eyeflag
		to trial_allon_wait on 1 % render_check_for_went
	trial_allon_wait:
		time 500
		to fixpt_break_clear on +WD0_XY & eyeflag
		to trial_beep_on on 1 = f_use_trial_beep
		to trial_allon_offset
	trial_allon_offset:
		do my_trial_allon_offset()
		to fixpt_break_wait on +WD0_XY & eyeflag
		to flow on 1 % render_check_for_went
	flow:
		code FLOWCD
		to update
	update:
		do my_update()
		to update_wait
	update_wait:
		to fixpt_break_wait on +WD0_XY & eyeflag
		to trial_pause_wait on +PSTOP & softswitch
		to update on 1 % my_check_for_went
		to trial_complete on 2 = f_wstatus
	trial_pause_wait:
		to trial_pause_clear on 1 % render_check_for_went
	trial_pause_clear:
		do my_trial_done(-1)
		to trial_paused on 1 % render_check_for_went
	trial_paused:
		to trial_init on -PSTOP & softswitch
	trial_abort:
		do my_trial_done(-1)
		to trial_init on 1 % render_check_for_went
	fixpt_break_wait:
		to fixpt_break_clear on 1 % render_check_for_went
	fixpt_break_clear:
		code BREAKFIXCD
		do my_trial_done(0)
		to fixpt_break_pause on 1 % render_check_for_went
	fixpt_break_pause:
		time 500
		to trial_init
	trial_complete:
		do my_trial_done(1)
		to trial_next on 1 % render_check_for_went
	trial_next:
		/*to all_done on 1 = f_all_trials_done*/
		to trial_init
	all_done:
		to first on 1 = f_never
abort	list:
		trial_abort
}


/*
 * Reward state set
 */

 
reward_set {
status ON
begin	no_op:
		to reward_init
	reward_init:
		do reward_init()
		to off_target on EXPTYPE_STEERING_WITH_FIXATION = f_exptype
		to off_target on EXPTYPE_STEERING = f_exptype
		to fix_check on EXPTYPE_FIXATION_ONLY = f_exptype
		to fix_check on EXPTYPE_FIXATION_TUNING = f_exptype
	off_target:
		do off_target()
		to stopped on 0 = f_going
		to paused on +PSTOP & softswitch
		to spinning on 1 = f_spinning
		to on_target_rewon on 2 % check_steering_and_wait_time
	spinning:
		to paused on +PSTOP & softswitch
		to stopped on 0 = f_going
		to off_target on 0 = f_spinning
	paused:
		to reward_init on -PSTOP & softswitch
	stopped:
		to reward_init on 1 = f_going
	on_target_rewon:
		do reward_on()
		time 5
		rand 15
		to on_target_rewoff
	on_target_rewoff:
		do reward_off()
		to on_target_waiting
	on_target_waiting:
		do get_next_reward_time()
		to paused on +PSTOP & softswitch
		to stopped on 0 = f_going
		to off_target_grace on 0 % check_steering_and_wait_time
		to off_target on -1 % check_steering_and_wait_time
		to on_target_rewon on 2 % check_steering_and_wait_time
	off_target_grace:
		do get_grace_time()
		to paused on +PSTOP & softswitch
		to stopped on 0 = f_going
		to on_target_reacq on 1 % check_steering
		to off_target on 0 % check_grace_time
	on_target_reacq:
		do my_target_reacq()
		to on_target_rewon
	fix_check:
		to fix_paused on +PSTOP & softswitch
		/* Does the fixation loop have a "stopped" state? */
		to fix_rewon on 1 = f_reward_flag
	fix_rewon:
		do reward_on()
		time 5
		rand 15
		to fix_rewoff
	fix_rewoff:
		do reward_off()
		to fix_paused on +PSTOP & softswitch
		to reward_init
	fix_paused:
		to reward_init on -PSTOP & softswitch
	
}

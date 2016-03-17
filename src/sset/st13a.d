/* $Id: st13a.d,v 1.1 2008/10/24 22:54:20 devel Exp $  */

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

/*
 * Ecodes local to the steering paradigm
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

#define ISICD	1900
#define GOCD	1901
#define BADCD	8192

/* Channel numbers for bcodes */
#define CH_JOY					1
#define CH_TRAJ 				2
#define CH_TARG 				3
#define CH_BLIP_OMEGA_MAX 		4
#define CH_BLIP_DURATION_FRAMES	5

#define CH_FRAMES_PER_SECOND		100
#define CH_SPEED 					101
#define CH_MS_BEFORE_JUMP			102
#define CH_MS_AFTER_JUMP			103
#define CH_NTRIALS					104
#define CH_JUMPS_PER_TRIAL			105
#define CH_STEER_MAX_DEGREES		106
#define CH_TARG_OFFSET_DEGREES_0	107
#define CH_TARG_OFFSET_DEGREES_1	108
#define CH_BLINK_PROBABILITY_PER_JUMP	109
#define CH_BLINK_DURATION_MS		110
#define CH_BLINK_GUARD_MS			111
#define CH_BLIP_PROBABILITY_PER_JUMP	112
#define CH_BLIP_DURATION_MS			113
#define CH_BLIP_GUARD_MS			114
#define CH_BLIP_MAX_ANG_VELOCITY_DEG_PER_SEC	115
#define CH_TARG_OFFSET_DEGREES_2	116
#define CH_TARG_OFFSET_DEGREES_3	117


/* Really really big char for sprinting errors. */
char f_charbuf[2048];

/* never var, quoth the raven */
int f_never = -1;

/* How many failures can we take when trying to place blips and blinks? */
int f_max_failures = 1000;

/* counters handled by my_check_for_went */
int f_wstatus = 0;
int f_frame_counter = 0;
int f_went_counter = 0;
int f_jump_counter = 0;
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

/* Perspective/camera parameters */
int f_screen_distance_MM = 290;
int f_far_plane_distance = 5000;

/* camera vars */
float f_cam_position[3]; 							/* Current camera position */
float f_cam_start[3] = { 0, 0, 0 };		/* Initial eye position at start of trial */
int f_cam_height = 200;						/* Initial cam height - will be applied to cam_start (and used in updates) */
float f_cam_trajectory = 0;						/* trajectory - direction camera is moving - IN RADIANS*/
float f_cam_looking[3];
float f_cam_up[3] = { 0, 1, 0 };

/* target parameters. */	
float f_targ_size = 0.25;
int f_targ_color[3] = { 255, 0, 0 };
float f_targ_dir = 0;							/* radians */
float f_targ_h = 0;
float f_targ_dist = 1000;
#define MAX_TARG_OFFSET_DEGREES 4
int f_targ_offset_degrees[MAX_TARG_OFFSET_DEGREES] = {5, 10, 15, 20};
int f_num_targ_offset_degrees = 0;

/* groundplane parameters */
int f_gp_length=1000;
int f_gp_width=1000;
int f_gp_dots=200;
int f_gp_pool=500;
int f_gp_plane_color[3]= { 50, 50, 50 };
int f_gp_dot_color[3]= { 255, 255, 255 };
int f_gp_flags = 0;
float f_gp_dotsize=2.0;

/* handles */
int f_handle_count = 0;
int f_gp_handle;
int f_targ_handle;

/* background color */
int f_background_color[3] = { 0, 0, 0 };

/* messaging structures. */
TargetStruct f_target;
MessageStruct f_msg;

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
/*int f_going = 0;*/
int f_paused = 0;
int f_spinflag = 0;
int f_jumpflag = 0;					/* Set when a jump occurs. Will not be cleared until next update, so will stay on for 1/framerate */
int f_reward_off_target_due_to_jump = 0;	/* Set to indicate off_target condition due to a jump, not bad steering */
int f_reward_window_degrees = 4;
int f_reward_flag = 0;
int f_reward_on_target=0;			/* if 0, currently NOT on target, otherwise this is the last time on target */
int f_reward_off_target=0;			/* if 0, currently NOT off target, otherwise this is the time that target was lost */
int f_reward_on_target_sum=0;		/* Running total of time on target WITHOUT failing acquisition time test */
int f_reward_time=35;				/* time, in ms, that juice is open. Can be set via menu. */
int f_reward_acquisition_time=1000;	/* acquisition time, converted to ms in menu callback function */
int f_reward_ramp_time=5000;		/* time for reward wait to ramp from max to min */
int f_reward_wait_max = 1000;		/* max time for reward wait */
int f_reward_wait_min = 500;		/* min time for reward wait */
int f_reward_wait_counter=0;		/* counter used in rew wait state */
int f_reward_last_reward_time=0;	/* last time a reward was given - started, that is */

int parse_offsets(int flag, MENU *mp, char *astr, VLIST *vlp, int *tvadd);
int random_targ_offset_degrees();



/* REX menu declarations */
VLIST state_vl[] = {
"day_seed",		&f_seed, NP, NP, 0, ME_DEC,
"number_of_trials", &f_ntrials, NP, NP, 0, ME_DEC,
"jumps_per_trial",	&f_jumps_per_trial, NP, NP, 0, ME_DEC,
"min_jump_time_ms",	&f_ms_jump_min, NP, NP, 0, ME_DEC,
"avg_jump_time_ms", &f_ms_jump_avg, NP, NP, 0, ME_DEC,
"jump_guard_before_ms", &f_ms_jump_guard_before, NP, NP, 0, ME_DEC,
"jump_guard_after_ms", &f_ms_jump_guard_after, NP, NP, 0, ME_DEC,
"Target_offset_0(deg)", &f_targ_offset_degrees[0], 0, NP, 0, ME_DEC,
"Target_offset_1(deg)", &f_targ_offset_degrees[1], 0, NP, 0, ME_DEC,
"Target_offset_2(deg)", &f_targ_offset_degrees[2], 0, NP, 0, ME_DEC,
"Target_offset_3(deg)", &f_targ_offset_degrees[3], 0, NP, 0, ME_DEC,
"Steering_max", &f_steer_max_degrees, NP, NP, 0, ME_FLOAT,
"Steering_zero_point", &f_steer_zero, NP, NP, 0, ME_DEC,
"Steering_dead_zone", &f_steer_dead, NP, NP, 0, ME_DEC,
"rew_win(deg)",	&f_reward_window_degrees, NP, NP, 0, ME_DEC,
"speed",	&f_speed, NP, NP, 0, ME_DEC,
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
NS,
};

char hm_reward[] = "";


MENU umenus[] = {
{"Main", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{"separator", NP}, 
{"reward", &reward_vl, NP, NP, 0, NP, hm_reward}, 
{"blink", &blink_vl, NP, NP, 0, NP, hm_blink}, 
{"blip", &blip_vl, NP, NP, 0, NP, hm_blip}, 
{"viewing volume", &vvol_vl, NP, NP, 0, NP, hm_vvol},
{"target", &target_vl, NP, NP, 0, NP, hm_target},
{"groundplane", &groundplane_vl, NP, NP, 0, NP, hm_groundplane},
{"background", &background_vl, NP, NP, 0, NP, hm_background}, 
{"fakjoy", &fakejoy_vl, NP, NP, 0, NP, hm_fakejoy}, 
{NS},
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
	
	dprintf("Initializing st12a\n");

	// initialize tcpip - must only do this OR gpib - NOT BOTH!!!!
	status = init_tcpip(f_local_addr, f_remote_addr, f_remote_port, 0);

	
//	// In case we did a Reset States
	f_paused = 0;
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

	f_gp_handle = f_targ_handle = 0;
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
			status = 1;
		}
	}
	return status;
}


/* my_exp_init *********************************************
 * 
 * Initializations for overall experiment. Trial counters...
 * 
 */
 
int my_exp_init()
{
	dprintf("Initializing experiment\n");
	f_trial_counter = f_ntrials;
	alloff();
	render_frame(0);
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

	/*
	 * Get a jump magnitude and direction. 
	 */
	 
	f_all_actions[n_all_actions].fval = random_targ_offset_degrees();
	
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

	if (f_verbose)
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
	if (f_verbose)
	{
		dprintf("nblips %d nblinks %d nactions %d\n", nblips, nblinks, nactions);
		if (!f_all_actions) dprintf("f_all_actions is NULL!\n");
	}
	
	/*
	 * Now run over the segtree and generate actions for each jump. Note that at each one we also flip a coin to determine 
	 * the direction of the jump. I'm also being lazy, and the traverse function
	 * will use the global array of actions, not passed data. 
	 */
	 
	if (f_verbose) dprintf("Generate actions for each jump.\n");
	segtree_traverse(ptree, &sbs, trial_action_builder, SEGTREE_TRAVERSE_INORDER);

	/*
	 * Add a segment for the trial's end.
	 */

	sbs.last += (sbs.jmin+(int)sbs.tau);
	f_all_actions[n_all_actions].start_frame = sbs.last;
	f_all_actions[n_all_actions].end_frame = sbs.last;
	f_all_actions[n_all_actions].type = ACTIONTYPE_TRIALEND;
	f_all_actions[n_all_actions].fval = 0;
	if (f_verbose) dprintf("Generate and insert trial end action.\n");
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
	
	/*
	for (i=0; i<n_all_actions; i++)
	{
		dprintf("%d %d %d-%d\n", i, f_ordered_actions[i]->type, f_ordered_actions[i]->start_frame, f_ordered_actions[i]->end_frame);
	}
	*/
	/* Finally, destroy the segtree and segments. */
	
	segtree_destroy(ptree, 1, 0);
	
	return 0;
}

void queue_actions(int frame)
{
	char s[128];
	while (f_candidate_action_index < n_all_actions && f_ordered_actions[f_candidate_action_index]->start_frame == frame)
	{
		if (f_verbose) dprintf("Q:%s", action_str(f_ordered_actions[f_candidate_action_index], s));
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
			if (f_verbose) dprintf("D:%s", action_str(f_current_actions[i], s));
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

	dprintf("Initializing trial %d\n", f_ntrials-f_trial_counter+1);

	f_frame_counter = 0;
	f_went_counter = 0;
	f_trial_end_flag = 0;
	f_reward_wait_counter = 0;
	
	/*
	 * Determine how many target offsets are configured. 
	 * We look at them in order, stopping at the first one set to zero.
	 * Any that are nonzero after that one are ignored. 
	 */
	for (i=0; i<MAX_TARG_OFFSET_DEGREES; i++) 
	{
		if (f_targ_offset_degrees[i] == 0) break;
	}
	f_num_targ_offset_degrees = i;			

	/* 
	 * Allocate memory for all actions needed this trial and 
	 * initialize counters et al.  
	 */
	dprintf("Generate trial actions.\n");
	generate_trial();	

	return 0;
}


int my_trial_allon_center()
{
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
 * Returns: 0
 *
 */

int random_targ_offset_degrees()
{
	int irand = ifuniv(f_num_targ_offset_degrees-1);
	int isgn = ifuniv(1)*2-1;
	return f_targ_offset_degrees[irand] * isgn;
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

int my_trial_done()
{
	stpause(1);

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

	if (f_gp_handle && f_targ_handle)
	{
		alloff();
		render_frame(1);
	}
	return 0;
}
  
/* alloff **************************************************
 * 
 * Turns off groundplane and target. Does not issue a render or frame. 
 * 
 */
 
int alloff()
{
	render_onoff(&f_gp_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_targ_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	return 0;
}
  	

   
int my_update()
{
	int ival;		/* input joystick value, shifted by zero value */
	float nval;		/* normalized joystick value */
	int ijoyh;		/* in case we're interrupted, better save the value used. */
	int i;
	int status = 0;
	int targ_update_flag = 0;	/* set to 1 if target needs updating.*/
	float blip_delta=0;
	char s[128];
	
	
	/* Steering: update f_cam_trajectory based on joystick reading.  */
	
	ijoyh = joystick_value();		// joystick_value will return fakejoy value if fakejoy is enabled
	ival = ijoyh - f_steer_zero;		// subtract off zero point

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
					int code = JLEFTCD;
					if (f_current_actions[i]->fval > 0) code = JRIGHTCD;
					if (ecode(code)) status = BADCD;
					f_targ_dir = f_cam_trajectory + f_current_actions[i]->fval * M_PI / 180.0f;
					targ_update_flag = 1;		
					f_jumpflag = 1;
				}
				break;
			}
			case ACTIONTYPE_BLIP:
			{
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
				f_targ_dir += f_current_actions[i]->fval;
				f_cam_trajectory += f_current_actions[i]->fval;
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
				dprintf("ERROR! Unknown action type (%d)\n", f_current_actions[i]->type);
				/* TODO: Error ecode for this situation. */
				break;
			}
		}
	}
	dequeue_actions(f_went_counter+1);


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


/* my_check_for_went *********************************/

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


/***************************** check_reward_window() ********************************
 *
 * Checks reward "window" and sets flag
 */

int reward_check_window()
{
	float dot;
	float v;
	
	// We want to compare the camera trajectory to the target bearing. If trajectory is within rew_window
	// then a reward is given. 

	f_reward_flag = 0;	
	if (!f_paused)
	{
		dot = cos(f_cam_trajectory)*cos(f_targ_dir) + sin(f_cam_trajectory)*sin(f_targ_dir);
		if (dot > cos(f_reward_window_degrees * M_PI/180))
		{
			f_reward_flag = 1;
		}
		if (f_reward_flag)
		{
			/* in reward window last time? If so, add elapsed time to total on target. If not, check acquisition time. */
			if (f_reward_on_target)
			{
				f_reward_on_target_sum += (i_b->i_time - f_reward_on_target);
			}
			else
			{
				if ((i_b->i_time - f_reward_off_target) > f_reward_acquisition_time)
				{
					f_reward_on_target_sum = 0;
				}
			}
			f_reward_off_target = 0;
			f_reward_on_target = i_b->i_time;
			v = (float)f_reward_on_target_sum/(float)f_reward_ramp_time;
			if (v>1) v=1;
			f_reward_wait_counter = f_reward_wait_max - v*(f_reward_wait_max - f_reward_wait_min);
		}
		else
		{
			/* did we just transition from being on-target? */
			if (f_reward_on_target)
			{
				f_reward_on_target = 0;
				f_reward_off_target = i_b->i_time;
			}
		}
	}
	else
	{
		/* During a pause we reset things. Sorry. */
		reward_init();
	}

	return f_reward_flag;
}


#if 0
int reward_check_window()
{
	float dot;
	float v;
	
	// We want to compare the camera trajectory to the target bearing. If trajectory is within rew_window
	// then a reward is given. 

	f_reward_flag = 0;	
	if (!f_paused && !f_spinflag)
	{
		dot = cos(f_cam_trajectory)*cos(f_targ_dir) + sin(f_cam_trajectory)*sin(f_targ_dir);
		if (dot > cos(f_reward_window_degrees * M_PI/180))
		{
			f_reward_flag = 1;
		}
		if (f_reward_flag)
		{
			/* in reward window last time? If so, add elapsed time to total on target. If not, check acquisition time. */
			if (f_reward_on_target)
			{
				f_reward_on_target_sum += (i_b->i_time - f_reward_on_target);
			}
			else
			{
				if (((i_b->i_time - f_reward_off_target) > f_reward_acquisition_time) || !f_reward_off_target_due_to_jump)
				{
					f_reward_on_target_sum = 0;
				}
			}
			f_reward_off_target_due_to_jump = 0;
			f_reward_off_target = 0;
			f_reward_on_target = i_b->i_time;
			v = (float)f_reward_on_target_sum/(float)f_reward_ramp_time;
			if (v>1) v=1;
			f_reward_wait_counter = f_reward_wait_max - v*(f_reward_wait_max - f_reward_wait_min);
		}
		else
		{
			/* did we just transition from being on-target? */
			if (f_reward_on_target)
			{
				f_reward_on_target = 0;
				f_reward_off_target = i_b->i_time;
				if (f_jumpflag)
				{
					f_reward_off_target_due_to_jump = 1;
				}
			}
		}
	}
	else
	{
		/* During a pause, or spin cycle, we reset things. Sorry. */
		reward_init();
		if (f_spinflag && f_verbose) dprintf("Reward off due to spinning.\n");
	}

	/* 
	 * We used to control the wait time via a counter in the reward loop itself. That meant that during the 
	 * wait time no checks were made on the reward window. Instead, we do that test here, which means
	 * that if we go off target during a reward wait the consequences are the same. 
	 */
	if (f_reward_flag && f_reward_wait_counter> 0 && ((i_b->i_time - f_reward_last_reward_time) < f_reward_wait_counter))
	{
		f_reward_flag = 0;
	} 
	else
	{
		f_reward_last_reward_time = i_b->i_time;
	}

	return f_reward_flag;
}
#endif

int reward_init()
{
	f_reward_on_target = 0;
	f_reward_off_target = i_b->i_time;
	f_reward_last_reward_time = 0;
	f_reward_wait_counter = INT_MIN;
	f_reward_off_target_due_to_jump = 0;
	return 0;
}

int reward_pause(int isPaused)
{
	if (isPaused) f_paused = 1;
	else f_paused = 0;
	return 0;
}


/*
 * stpause(int isPaused)
 * 
 * Called when a paradigm is paused or stopped. Closes/opens
 * analog window as needed, and sets flag for reward loop. 
 * We have to close the analog window here in case a reset or 
 * file close happens. Call with isPaused=1 when pausing, and
 * make sure to call again with isPaused=0 when done with pause. 
 * Also should make sure that if user pauses, then does 
 * ResetStates that the analog window and reward are un-paused. 
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
	f_paused = isPaused;
	return 0;
}

int reward_out()
{
	if (f_verbose) dprintf("Reward!\n");
	return 0;
}



/* REX state set starts here */

%%

id 402
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
		to trial_pause
	trial_pause:
		time 2000
		to trial_allon_center
	trial_allon_center:
		do my_trial_allon_center()
		to trial_allon_wait on 1 % render_check_for_went
	trial_allon_wait:
		time 500
		to trial_beep_on
	trial_beep_on:
		dio_on(BEEP)
		time 100
		to trial_beep_off
	trial_beep_off:
		dio_off(BEEP)
		to trial_allon_offset
	trial_allon_offset:
		do my_trial_allon_offset()
		to wind on 1 % render_check_for_went
	wind:
		do awind(OPEN_W)
		to trcd
	trcd:
		code TRIALCD
		to update
	update:
		do my_update()
		to p3
	p3:
		to p4 on +PSTOP & softswitch
		to update_wait on -PSTOP & softswitch
	p4:	
		code PAUSECD
		do stpause(1)
		to update_wait on -PSTOP & softswitch
	update_wait:
		do stpause(0)
		to update on 1 % my_check_for_went
		to trial_done on 2 = f_wstatus
	trial_done:
		do my_trial_done()
		to cwind
	cwind:
		do awind(CLOSE_W)
		to all_done on 1 ? f_trial_counter
		to trial_init
	all_done:
		do stpause(1)
		to first on 1 = f_never
abort	list:
		trial_done
}


/*
 * Auxiliary state set for processing reward contingency.
 */
rew_set {
status ON
begin	no_op:
		to p7
	p7:
		to p8 on +PSTOP & softswitch
		to reward_init on -PSTOP & softswitch
	p8:
		code PAUSECD
		to reward_init on -PSTOP & softswitch
	reward_init:
		do reward_init()
		to check
	check:
		to reward on 1 % reward_check_window
	reward:
		dio_on(REW)
		time 25
		rand 75
		to rewoff
	rewoff:
		dio_off(REW)
		to mark
	mark:
		do score(1)
		to wait
	wait:
		to check on 1 ? f_reward_wait_counter
}

/* $Id: st23af.d,v 1.10 2015/05/12 20:53:48 devel Exp $ */

#include <math.h>
#include "bcode.h"
#include "ldev.h"	/* hardware device defines */
#include "lcode.h"	/* Ecodes */
#include "lcalib.h"	/* local (per-rig) screen calibration */
#include "r3p.h"

#include "make_msg.h"			/* funcs to form command messages */
#include "transport.h"			/* funcs to send command messages to render */
#include "actions.h"			/* higher-level actions for render */
#include "pixels.h"				/* pixel conversion funcs */

/*
 * djs Convert to using nr ran1
 * Stop using ivunif.h
 */
#if 0
#include "ivunif.h"				/* random number routines */
#endif
#include "nrand.h"
#include "segtree.h"			/* segment tree routines */
#include "slider.h"
#include "randomtrialgenerator.h"
#include "eyepos.h"
#include "mbq.h"

#include "st21af_rtvars.h"

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
#define SPINNINGCD		1518
#define NOTSPINNINGCD	1519
#define FLOWCD 1520
#define BADSTEERCD 1521
#define USTIMONCD	1550
#define USTIMOFFCD	1551

#define ISICD	1900
#define GOCD	1901
#define BADCD	8192

/* Channel numbers for bcodes */
#define CH_JOY					1
#define CH_TRAJ 				2
#define CH_TARG 				3
#define CH_BLIP_OMEGA_MAX 		4
#define CH_BLIP_DURATION_FRAMES	5
#define CH_JUMP_SIZE			6
#define CH_JUMP_ACTUAL			7
#define CH_FIXATION_X			8
#define CH_FIXATION_Y			9
#define CH_WDPOS_X				10
#define CH_WDPOS_Y				11
#define CH_TRAJ_JITTER			12
#define CH_TARG_JITTER			13
#define CH_TARG_MEANDER			14
#define CH_CAMPOS_X				15
#define CH_CAMPOS_Z				16
#define CH_TARG_DIFFUSION		17
#define CH_FIXED_LOOKING_ANGLE_DEGREES	18

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
#define	CH_STEER_ZERO				119
#define CH_STEER_DEAD				120
#define CH_REWARD_WINDOW_DEGREES	121
#define CH_SCREEN_DISTANCE_MM		122
#define CH_FAR_PLANE_DISTANCE		123
#define CH_CAMERA_HEIGHT			124
#define CH_TARGET_DIAM_DEGREES		125
#define CH_TARG_COLOR_R				126
#define CH_TARG_COLOR_G				127
#define CH_TARG_COLOR_B				128
#define CH_TARG_DIST				129
#define	CH_TARG_HEIGHT_DEGREES		130
#define CH_GP_LENGTH				131
#define CH_GP_WIDTH					132
#define CH_DOTS_PER_GRID			133
#define CH_GRID_POOL_SIZE			134
#define CH_PLANE_R					135
#define CH_PLANE_G					136
#define CH_PLANE_B					137
#define CH_DOT_R					138
#define CH_DOT_G					139
#define CH_DOT_B					140
#define CH_GRID_FLAGS				141
#define CH_DOT_SIZE					142
#define CH_BACKGROUND_R				143
#define CH_BACKGROUND_G				144
#define CH_BACKGROUND_B				145
#define CH_BLINK_RATE				146
#define CH_BLINK_MS					147
#define CH_BLINK_DEBUG				149
#define CH_BLIP_RATE				150
#define CH_BLIP_MS					151
#define CH_BLIP_MAX_VEL				153
#define CH_BLIP_DEBUG				154
#define CH_REW_ACQUISITION_MS		155
#define CH_REW_WAIT_MAX				156
#define CH_REW_WAIT_MIN				157
#define CH_REW_RAMP					158
#define CH_TARG_OFFSET_DEGREES_4	159
#define CH_TARG_OFFSET_DEGREES_5	160
#define CH_TARG_OFFSET_DEGREES_6	161
#define CH_TARG_OFFSET_DEGREES_7	162
#define CH_TARG_OFFSET_DEGREES_8	163
#define CH_TARG_OFFSET_DEGREES_9	164
#define CH_TARG_OFFSET_WEIGHT_0		165
#define CH_TARG_OFFSET_WEIGHT_1		166
#define CH_TARG_OFFSET_WEIGHT_2		167
#define CH_TARG_OFFSET_WEIGHT_3		168
#define CH_TARG_OFFSET_WEIGHT_4		169
#define CH_TARG_OFFSET_WEIGHT_5		170
#define CH_TARG_OFFSET_WEIGHT_6		171
#define CH_TARG_OFFSET_WEIGHT_7		172
#define CH_TARG_OFFSET_WEIGHT_8		173
#define CH_TARG_OFFSET_WEIGHT_9		174
#define CH_FIXPT_X					180
#define CH_FIXPT_Y					181
#define CH_FIXPT_DIAMETER			182
#define CH_FIXPT_WINDOW				183
#define CH_FIXPT_ACQTIME			184
#define CH_FIXPT_ACQTIMEOUT			185
#define CH_FIXPT_FIXTIME			186
#define CH_FIXPT_FIXTIMEOUT			187
#define CH_FIXPT_R					188
#define CH_FIXPT_G					189
#define CH_FIXPT_B					190
#define CH_INTERTRIAL_TIME			191
#define CH_REPLAY_TIMESTAMP			192
#define CH_FIXED_LOOKING			193
#define CH_NFIXED_LOOKING			194
#define CH_FIXED_LOOKING_DEG_0		195
#define CH_FIXED_LOOKING_DEG_1		196
#define CH_FIXED_LOOKING_DEG_2		197
#define CH_FIXED_LOOKING_DEG_3		198
#define CH_FIXED_LOOKING_DEG_4		199
#define CH_FIXED_LOOKING_DEG_5		200
#define CH_FIXED_LOOKING_DEG_6		201
#define CH_FIXED_LOOKING_DEG_7		202
#define CH_FIXED_LOOKING_DEG_8		203
#define CH_FIXED_LOOKING_DEG_9		204

/* Eye window stuff */
#define WIND0	    0


/* Really really big char for sprinting errors. */
char f_charbuf[2048];

/* never var, quoth the raven */
int f_never = -1;

/* beeper doesn't work. Set to 1 to prove this to yourself. */
int f_use_trial_beep = 0;

/* How many failures can we take when trying to place blips and blinks? */
int f_max_failures = 1000;

/* counters handled by my_check_for_went */
int f_wstatus = 0;
int f_frame_counter = 0;
int f_went_counter = 0;
int f_jump_counter = 0;
int f_went_cycles = 0;
int f_trial_end_flag = 0;
int f_all_done = 0;
int f_exp_abort = 0;

/* State list menu vars and associated vars*/
int f_seed = -999;
int f_seed_used = 0;
int f_jumps_per_trial = 0;
float f_trial_max_length_s = 60;
int f_ms_jump_min = 0;
int f_ms_jump_avg = 0;
int f_ms_jump_guard_before = 250;
int f_ms_jump_guard_after = 250;
int f_ntrialtypes = 0;
int f_ntrials = 1;
int f_nblocksize = 1;
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

int f_fixed_looking = 0;	// if nonzero, will use fixed looking directions 
int f_nfixed_looking = 0;
#define MAX_FIXED_LOOKING_ANGLES 10
float f_fixed_looking_angle_deg[MAX_FIXED_LOOKING_ANGLES] = {0};
float *f_pfixed_looking_angle_deg = NULL;


/* bit masks for verbosity */
#define DEBUG_BIT 0x1
#define DEBUG_REWARD_BIT 0x2
#define DEBUG_ACQ_BIT 0x4
#define DEBUG_BKGD_BIT 0x8
#define DEBUG_BKGD2_BIT 0x10
#define DEBUG_JOY_BIT 0x20
#define DEBUG_FIXED_BIT 0x40

#define FIXATION_STATISTICS_INIT -2
#define FIXATION_STATISTICS_DUMP -1
#define FIXATION_STATISTICS_FAIL 0
#define FIXATION_STATISTICS_NOISE 1
int f_fixation_statistics[2];

/* Perspective/camera parameters */
int f_screen_distance_MM = 290;
int f_far_plane_distance = 5000;
float f_cos_fovx_over_2 = 1;

/* camera vars */
float f_cam_position[3]; 							/* Current camera position */
float f_cam_start[3] = { 0, 0, 0 };		/* Initial eye position at start of trial */
int f_cam_height = 200;						/* Initial cam height - will be applied to cam_start (and used in updates) */
float f_cam_trajectory = 0;						/* trajectory - direction camera is moving - IN RADIANS*/
float f_cam_looking[3];
float f_cam_up[3] = { 0, 1, 0 };

/* target parameters. */	
float f_ttrans_dir = 0;							/* radians */
float f_ttrans_h = 0;
float f_ttrans_dist = 1000;

int f_use_plaindot = 0;
float f_plaindot_radius = .25;
int f_plaindot_depth = 2;
int f_plaindot_color[3] = { 100, 0, 0 };

int f_use_gaussiandot = 1;
int f_gaussiandot_depth = 1;
float f_gaussiandot_radius = .25;
float f_gaussiandot_sigma = 10;
int f_gaussiandot_coherence = 95;
int f_gaussiandot_npts = 20;
float f_gaussiandot_pointsize = 1.0;
int f_gaussiandot_color[3] = { 255, 255, 0 };

#define MAX_TARG_OFFSET_DEGREES 10
float f_targ_offset_degrees[MAX_TARG_OFFSET_DEGREES] = {15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int f_targ_offset_weight[MAX_TARG_OFFSET_DEGREES] = {100, NULLI, NULLI, NULLI, NULLI, NULLI, NULLI, NULLI, NULLI, NULLI};
int f_num_targ_offset_degrees = 0;
int f_targ_offset_weight_sum = 0;

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
float f_fixpt_x = 0;	/* Not used in this paradigm -- see f_fixpt_x_ below */
float f_fixpt_y = 0;	/* Not used in this paradigm -- see f_fixpt_x_ below */
float f_fixpt_diameter = 0.25;
float f_fixpt_window = 3.0;
int f_fixpt_color[3] = { 0, 255, 0 };

#define MAX_FIXPT 10
int f_use_fixpt_pos_menu = 0;
int f_nfixpt = 1;
float f_fixpt_x_degrees[MAX_FIXPT] = {0};
float f_fixpt_y_degrees[MAX_FIXPT] = {0};
RTGenerator *f_prtgen = NULL;
int f_trial_condition_index = -1;

/*
 * These are used for accessing fixation positions and eye window positions. The vars f_fixpt_x/y_degrees
 * above are used in the menus only and then are transferred here (see my_exp_init). When an eyepos config
 * file is used, these are allocated based on the contents of the file. 
 */

int f_use_fixpt_pos_file=0;
char f_fixpt_pos_file[256] = "";	/* Positions for fixation point found here */
float *f_pfixpt_x = NULL;
float *f_pfixpt_y = NULL;
long *f_pwdpos_x = NULL;
long *f_pwdpos_y = NULL;

/* timing variables - all in milliseconds */
int f_acq_time = 4000;		/* Time to first acquire fixation point */
int f_fix_time = 500;		/* Time to fixate */
int f_acq_fail_timeout = 500;	/* time to wait after failure to acquire target */
int f_acq_noise_timeout = 500;	/* time to wait after fixation noise event */
int f_fixation_fail_timeout = 1000;	/* Fixation broken? This is a timeout to discourage it */
int f_intertrial_time = 500;	/* time between trials */
int f_trial_allon_wait_time = 500;	/* time to pause after allon */
int f_trial_init_pause_time = 500;	/* Time to pause after trial init is complete */

/* handles */
int f_handle_count = 0;
int f_gp_handle;
int f_fixpt_handle;
int f_ttrans_handle;
int f_gaussiandot_handle;
int f_plaindot_handle;

/* background color */
int f_background_color[3] = { 0, 0, 0 };

/* messaging structures. */
/* not used any longer! TargetStruct f_target; */
MessageStruct f_msg;
DotStruct f_fixpt;
TargetTransformStruct f_ttrans;
DotStruct f_plaindot;
GaussianDotStruct f_gaussiandot;

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

/* 
 * ustim flag. This is set to tell the ustim loop to turn on 
 * the microstim. It is turned off by a time setting, or by 
 * a pause in the paradigm.
 */

int f_ustim_debug = 0;
float f_ustim_rate = 0;
int f_ustim_duration_ms = 0;
int f_ustim_guard_ms = 0;
int f_ustim_flag=0;
int f_ustim_ready_flag=0;

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

#define MAX_JITTER_ACTIONS 12
struct action f_jitter_actions[MAX_JITTER_ACTIONS];
PMBQ f_pmbq_jitter_actions = NULL;

/*
 * Queues for pre-allocating random numbers
 */
PMBQF f_pmbqf_trajectory_distribution = NULL;
PMBQF f_pmbqf_target_distribution = NULL;
PMBQF f_pmbqf_meander_distribution = NULL;
PMBQF f_pmbqf_diffusion_distribution = NULL;

/*
 * These are the action types defined. Place one in PActionStruct->type
 */

#define ACTIONTYPE_JUMP 1
#define ACTIONTYPE_BLIP 2
#define ACTIONTYPE_BLINK 3
#define ACTIONTYPE_TRIALEND 4
#define ACTIONTYPE_JITTERTRAJ 5
#define ACTIONTYPE_JITTERTARG 6
#define ACTIONTYPE_MEANDERTARG 7
#define ACTIONTYPE_USTIM 8
#define ACTIONTYPE_DIFFUSIONTARG 9

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
int f_next_reward_time = INT_MAX;
int f_last_reward_time = 0; 
int f_reward_on_target_sum=0;		/* Running total of time on target WITHOUT failing acquisition time test */
int f_grace_period_expire = INT_MAX;
Slider *f_pslider=NULL;					/* sliding window for spin detection */
int f_spinning = 0;
int f_reward_preset = 10;
int f_reward_random = 15;
int f_reward_on_time = 0;			/* used for debugging reward delivery time. */

/* Reward menu stuff */
float f_reward_window_degrees = 4;
int f_reward_acquisition_time=1000;	/* acquisition time, converted to ms in menu callback function */
int f_reward_use_spatial_ramp=0;	/* if not zero then use spatial ramp for reward, not temporal */
int f_reward_ramp_time=5000;		/* time for reward wait to ramp from max to min */
int f_reward_wait_max = 1000;		/* max time for reward wait */
int f_reward_wait_min = 500;		/* min time for reward wait */
int f_spin_window_ms = 0;			/* Length of sliding window for spin detection */
int f_spin_deviation = 0;			/* max deviation for spinning */
int f_reward_holdoff = 100;			/* when on target, this is a delay before the first reward */
float f_reward_spatial_fraction;	/* pseudo-linear fraction = 1 when on target exactly, = 0 when off target at edge of window */
float f_cos_reward_window;

/* Added 1-5-2012 for bonus reward */
int f_bonus_reward = 0;				/* flag set when bonus reward is due */
int f_bonus_reward_random = 15;
int f_bonus_reward_max = 15;		/* reward when trial ends with error=0 */
float f_bonus_reward_cutoff_degrees = 20; 	/* no bonus reward when abs(error) > this */
int f_bonus_reward_pause = 1;		/* pause in ms before bonus reward is given (IF it is given) */
int f_bonus_reward_pause_random = 0;		/* NOT IN DIALOG */

/* Jitter menu stuff */
int f_jitter_trajectory_dwell_frames = 0;
float f_jitter_trajectory_sigma = 0;
float f_jitter_trajectory_cutoff = 0;
int f_jitter_target_dwell_frames = 0;
float f_jitter_target_sigma = 0;
float f_jitter_target_cutoff = 0;
int f_meander_target_dwell_frames_min = 0;
int f_meander_target_dwell_frames_max = 0;
float f_meander_target_sigma = 0;
float f_meander_target_cutoff = 0;
int f_jitter_record_campos = 0;
int f_diffusion_target_dwell_frames_min = 0;
int f_diffusion_target_dwell_frames_max = 0;
float f_diffusion_target_sigma = 0;
float f_diffusion_target_cutoff = 0;

/* in process jitter controls */
float f_jitter_trajectory_delta = 0.0;
float f_jitter_target_delta = 0.0;
int f_meander_target_next_action_frame = 0;
int f_diffusion_target_next_action_frame = 1;

/* command saving vars */
int f_savecommands = 0;
int f_savecommandscount = 0;
int f_savecommandsbuffersize = 0;
int f_savecommandsbufferused = 0;
char *f_savecommandsbuffer = NULL;
char f_savecommandsfolder[128];
char f_savecommandsfile[256];	/* This will be full path for the file */
char f_replaypath[32];			/* replay file path */
char f_replaybase[32];         /* replay file basename (excluding .cmd and path)*/
int f_replay_min_frames = 0;
int f_replay_reflect = 0;
float f_replay_cam_trajectory_start = 0;		/* cam trajectory at start of a trial */
int f_use_replay_file = 0;
int f_replay_counter = 0;		/* counter into went data for trial. Not same as f_went_counter! */

/* Reward variables for replay trials */
int f_fix_initial = -1;				/* Initial fixation point in replay trial */
int f_fix_next_reward = -1;	/* Next time reward is due */

/* REX menu declarations */
VLIST state_vl[] = {
"day_seed",		&f_seed, NP, NP, 0, ME_DEC,
"max_trial_length_s", &f_trial_max_length_s, NP, NP, 0, ME_FLOAT,
"jumps_per_trial",	&f_jumps_per_trial, NP, NP, 0, ME_DEC,
"min_jump_time_ms",	&f_ms_jump_min, NP, NP, 0, ME_DEC,
"avg_jump_time_ms", &f_ms_jump_avg, NP, NP, 0, ME_DEC,
"jump_guard_before_ms", &f_ms_jump_guard_before, NP, NP, 0, ME_DEC,
"jump_guard_after_ms", &f_ms_jump_guard_after, NP, NP, 0, ME_DEC,
"Steering_max", &f_steer_max_degrees, NP, NP, 0, ME_FLOAT,
"Steering_zero_point", &f_steer_zero, NP, NP, 0, ME_DEC,
"Steering_dead_zone", &f_steer_dead, NP, NP, 0, ME_DEC,
"rew_win(deg)",	&f_reward_window_degrees, NP, NP, 0, ME_FLOAT,
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
char hm_sv_vl[] = "If jumps_per_trial is zero, will use max_trial_length_s\nIf jumps_per_trial is not zero, max_trial_length_s is ignored.";


/* Jump parameters menu */

VLIST jump_vl[] = {
"Jump_Degrees_0(deg)", &f_targ_offset_degrees[0], 0, NP, 0, ME_FLOAT,
"Jump_Weight_0(deg)", &f_targ_offset_weight[0], 0, NP, 0, ME_NVALD,
"Jump_Degrees_1(deg)", &f_targ_offset_degrees[1], 0, NP, 0, ME_FLOAT,
"Jump_Weight_1(deg)", &f_targ_offset_weight[1], 0, NP, 0, ME_NVALD,
"Jump_Degrees_2(deg)", &f_targ_offset_degrees[2], 0, NP, 0, ME_FLOAT,
"Jump_Weight_2(deg)", &f_targ_offset_weight[2], 0, NP, 0, ME_NVALD,
"Jump_Degrees_3(deg)", &f_targ_offset_degrees[3], 0, NP, 0, ME_FLOAT,
"Jump_Weight_3(deg)", &f_targ_offset_weight[3], 0, NP, 0, ME_NVALD,
"Jump_Degrees_4(deg)", &f_targ_offset_degrees[4], 0, NP, 0, ME_FLOAT,
"Jump_Weight_4(deg)", &f_targ_offset_weight[4], 0, NP, 0, ME_NVALD,
"Jump_Degrees_5(deg)", &f_targ_offset_degrees[5], 0, NP, 0, ME_FLOAT,
"Jump_Weight_5(deg)", &f_targ_offset_weight[5], 0, NP, 0, ME_NVALD,
"Jump_Degrees_6(deg)", &f_targ_offset_degrees[6], 0, NP, 0, ME_FLOAT,
"Jump_Weight_6(deg)", &f_targ_offset_weight[6], 0, NP, 0, ME_NVALD,
"Jump_Degrees_7(deg)", &f_targ_offset_degrees[7], 0, NP, 0, ME_FLOAT,
"Jump_Weight_7(deg)", &f_targ_offset_weight[7], 0, NP, 0, ME_NVALD,
"Jump_Degrees_8(deg)", &f_targ_offset_degrees[8], 0, NP, 0, ME_FLOAT,
"Jump_Weight_8(deg)", &f_targ_offset_weight[8], 0, NP, 0, ME_NVALD,
"Jump_Degrees_9(deg)", &f_targ_offset_degrees[9], 0, NP, 0, ME_FLOAT,
"Jump_Weight_9(deg)", &f_targ_offset_weight[9], 0, NP, 0, ME_NVALD,
NS,
};

char hm_jump[] = "";

/* Trial types menu */
VLIST trialtypes_vl[] = {
"num_each_trial_type", &f_ntrials, NP, NP, 0, ME_DEC,
"blocksize", &f_nblocksize, NP, NP, 0, ME_DEC,
"use_positions_file", &f_use_fixpt_pos_file, NP, NP, 0, ME_DEC,
"Fixpt_positions_file", &f_fixpt_pos_file, NP, NP, 0, ME_STR,
"use_positions_menu", &f_use_fixpt_pos_menu, NP, NP, 0, ME_DEC,
"do_replay(0/1)", &f_use_replay_file, NP, NP, 0, ME_DEC,
"replay_file_path", f_replaypath, NP, NP, 0, ME_STR,
"replay_file_basename", f_replaybase, NP, NP, 0, ME_STR,
"replay_trial_min", &f_replay_min_frames, NP, NP, 0, ME_DEC,
"replay_reflect(0/1)", &f_replay_reflect, NP, NP, 0, ME_DEC,
NS,
};

char hm_trialtypes[] = "Specify trial types here\nSet \"use_positions_file\", \"use_positions_menu\" or \"use_replay\" to 1 (they are checked in this order.. the first of these that is set to 1 wins).\nEither filename can be set to NONE to eliminate Rex complaints when a root file is loaded. Omit .cmd extension for replay file."; 
 



/* Fixation point positions menu */

VLIST fixpt_pos_vl[] = {
"Number_of_fix_pts", &f_nfixpt, NP, NP, 0, ME_DEC,
"Fix_X_0(deg)", &f_fixpt_x_degrees[0], 0, NP, 0, ME_FLOAT,
"Fix_Y_0(deg)", &f_fixpt_y_degrees[0], 0, NP, 0, ME_FLOAT,
"Fix_X_1(deg)", &f_fixpt_x_degrees[1], 0, NP, 0, ME_FLOAT,
"Fix_Y_1(deg)", &f_fixpt_y_degrees[1], 0, NP, 0, ME_FLOAT,
"Fix_X_2(deg)", &f_fixpt_x_degrees[2], 0, NP, 0, ME_FLOAT,
"Fix_Y_2(deg)", &f_fixpt_y_degrees[2], 0, NP, 0, ME_FLOAT,
"Fix_X_3(deg)", &f_fixpt_x_degrees[3], 0, NP, 0, ME_FLOAT,
"Fix_Y_3(deg)", &f_fixpt_y_degrees[3], 0, NP, 0, ME_FLOAT,
"Fix_X_4(deg)", &f_fixpt_x_degrees[4], 0, NP, 0, ME_FLOAT,
"Fix_Y_4(deg)", &f_fixpt_y_degrees[4], 0, NP, 0, ME_FLOAT,
"Fix_X_5(deg)", &f_fixpt_x_degrees[5], 0, NP, 0, ME_FLOAT,
"Fix_Y_5(deg)", &f_fixpt_y_degrees[5], 0, NP, 0, ME_FLOAT,
"Fix_X_6(deg)", &f_fixpt_x_degrees[6], 0, NP, 0, ME_FLOAT,
"Fix_Y_6(deg)", &f_fixpt_y_degrees[6], 0, NP, 0, ME_FLOAT,
"Fix_X_7(deg)", &f_fixpt_x_degrees[7], 0, NP, 0, ME_FLOAT,
"Fix_Y_7(deg)", &f_fixpt_y_degrees[7], 0, NP, 0, ME_FLOAT,
"Fix_X_8(deg)", &f_fixpt_x_degrees[8], 0, NP, 0, ME_FLOAT,
"Fix_Y_8(deg)", &f_fixpt_y_degrees[8], 0, NP, 0, ME_FLOAT,
"Fix_X_9(deg)", &f_fixpt_x_degrees[9], 0, NP, 0, ME_FLOAT,
"Fix_Y_9(deg)", &f_fixpt_y_degrees[9], 0, NP, 0, ME_FLOAT,
NS,
};

char hm_fixpt_pos[] = "Specify fixation point positions. These are ignored unless you specify \"use_positions_menu\" in the \"Trial Types\" menu";



/* Viewing volume menu */

VLIST vvol_vl[] = {
"screen_dist(mm)", &f_screen_distance_MM, NP, NP, 0, ME_DEC,
"far_plane_distance", &f_far_plane_distance, NP, NP, 0, ME_DEC, 
"Camera_height", &f_cam_height, NP, NP, 0, ME_DEC, 
NS,
};

char hm_vvol[] = "";

/* Target parameters menu  */


VLIST dualtarget_vl[] = {
"Target_distance", &f_ttrans_dist, 0, NP, 0, ME_FLOAT,
"Target_height(degrees)", &f_ttrans_h, 0, NP, 0, ME_FLOAT,
"Use_plain_old_dot(1/0)", &f_use_plaindot, NP, NP, 0, ME_DEC,
"Plain_dot_radius(deg)", &f_plaindot_radius, NP, NP, 0, ME_FLOAT,
"Plain_dot_depth", &f_plaindot_depth,  NP, NP, 0, ME_DEC,
"Plain_dot_color(R)", &f_plaindot_color[0], NO, NP, 0, ME_DEC, 
"Plain_dot_color(G)", &f_plaindot_color[1], NO, NP, 0, ME_DEC, 
"Plain_dot_color(B)", &f_plaindot_color[2], NO, NP, 0, ME_DEC, 
"Use_gaussian_dot(1/0)", &f_use_gaussiandot, NP, NP, 0, ME_DEC, 
"G_dot_depth", &f_gaussiandot_depth, NP, NP, 0, ME_DEC, 
"G_dot_max_radius(deg)", &f_gaussiandot_radius, NP, NP, 0, ME_FLOAT,
"G_dot_sigma(deg)", &f_gaussiandot_sigma, NP, NP, 0, ME_FLOAT,
"G_dot_coherence(0-100)", &f_gaussiandot_coherence, NP, NP, 0, ME_DEC,
"G_dot_npts", &f_gaussiandot_npts, NP, NP, 0, ME_DEC,
"G_dot_pointsize(pixels)", &f_gaussiandot_pointsize, NP, NP, 0, ME_FLOAT,
"G_dot_color(R)", &f_gaussiandot_color[0], NP, NP, 0, ME_DEC,
"G_dot_color(G)", &f_gaussiandot_color[1], NP, NP, 0, ME_DEC,
"G_dot_color(B)", &f_gaussiandot_color[2], NP, NP, 0, ME_DEC,
NS,
};

char hm_dualtarget[] = "";


VLIST looking_vl[] = {
	"Use_fixed_looking", &f_fixed_looking, NP, NP, 0, ME_DEC,
	"Number_of_looking_dir", &f_nfixed_looking, NP, NP, 0, ME_DEC,
	"looking_0(deg)", &f_fixed_looking_angle_deg[0], 0, NP, 0, ME_FLOAT,
	"looking_1(deg)", &f_fixed_looking_angle_deg[1], 0, NP, 0, ME_FLOAT,
	"looking_2(deg)", &f_fixed_looking_angle_deg[2], 0, NP, 0, ME_FLOAT,
	"looking_3(deg)", &f_fixed_looking_angle_deg[3], 0, NP, 0, ME_FLOAT,
	"looking_4(deg)", &f_fixed_looking_angle_deg[4], 0, NP, 0, ME_FLOAT,
	"looking_5(deg)", &f_fixed_looking_angle_deg[5], 0, NP, 0, ME_FLOAT,
	"looking_6(deg)", &f_fixed_looking_angle_deg[6], 0, NP, 0, ME_FLOAT,
	"looking_7(deg)", &f_fixed_looking_angle_deg[7], 0, NP, 0, ME_FLOAT,
	"looking_8(deg)", &f_fixed_looking_angle_deg[8], 0, NP, 0, ME_FLOAT,
	"looking_9(deg)", &f_fixed_looking_angle_deg[9], 0, NP, 0, ME_FLOAT,
	NS,
};

char hm_looking[] = "";


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

VLIST ustim_vl[] = {
"ustim_rate(ustim/sec)", &f_ustim_rate, 0, NP, 0, ME_FLOAT,
"ustim_duration(ms)", &f_ustim_duration_ms, 0, NP, 0, ME_DEC,
"guard(ms)", &f_ustim_guard_ms, 0, NP, 0, ME_DEC,
"debug_msgs", &f_ustim_debug, 0, NP, 0, ME_DEC,
NS,
};

char hm_ustim[] = "";

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
"reward_holdoff(ms)", &f_reward_holdoff, NP, NP, 0, ME_DEC, 
"reward_size_preset(ms)", &f_reward_preset, NP, NP, 0, ME_DEC, 
"reward_size_random(ms)", &f_reward_random, NP, NP, 0, ME_DEC, 
"bonus_reward_max(ms)", &f_bonus_reward_max, NP, NP, 0, ME_DEC, 
"bonus_reward_random(ms)", &f_bonus_reward_random, NP, NP, 0, ME_DEC, 
"bonus_cutoff(deg)", &f_bonus_reward_cutoff_degrees, NP, NP, 0, ME_FLOAT,
"bonus_reward_pause(ms)", &f_bonus_reward_pause, NP, NP, 0, ME_DEC, 
NS,
};

char hm_reward[] = "";

VLIST fixpt_vl[] = {
"fixpt_diameter(deg)", &f_fixpt_diameter, NP, NP, 0, ME_FLOAT,
"fixpt_window(deg)", &f_fixpt_window, NP, NP, 0, ME_FLOAT,
"fixpt_color(R)", &f_fixpt_color[0], 0, NP, 0, ME_DEC,
"fixpt_color(G)", &f_fixpt_color[1], 0, NP, 0, ME_DEC,
"fixpt_color(B)", &f_fixpt_color[2], 0, NP, 0, ME_DEC,
"fixpt_pos_file", f_fixpt_pos_file, NP, NP, 0, ME_STR,
NS,
};

char hm_fixpt[] = "";

VLIST timing_vl[] = {
"acq_time(ms)", &f_acq_time, NP, NP, 0, ME_DEC,
"acq_timeout(ms)", &f_acq_fail_timeout, NP, NP, 0, ME_DEC,
"fix_time(ms)", &f_fix_time, NP, NP, 0, ME_DEC,
"fix_timeout(ms)", &f_acq_noise_timeout, NP, NP, 0, ME_DEC,
"fix_break_timeout(ms)", &f_fixation_fail_timeout,  NP, NP, 0, ME_DEC,
"intertrial_time(ms)", &f_intertrial_time, NP, NP, 0, ME_DEC,
"all_on_wait_time(ms)", &f_trial_allon_wait_time, NP, NP, 0, ME_DEC,
NS,
};

char hm_timing[] = "";

VLIST jitter_vl[] = {
"Jitter_traj_dwell", &f_jitter_trajectory_dwell_frames, NP, NP, 0, ME_DEC,
"Jitter_traj_sigma", &f_jitter_trajectory_sigma, NP, NP, 0, ME_FLOAT,
"Jitter_traj_cutoff", &f_jitter_trajectory_cutoff, NP, NP, 0, ME_FLOAT,
"Jitter_targ_dwell", &f_jitter_target_dwell_frames, NP, NP, 0, ME_DEC,
"Jitter_targ_sigma", &f_jitter_target_sigma, NP, NP, 0, ME_FLOAT,
"Jitter_targ_cutoff", &f_jitter_target_cutoff, NP, NP, 0, ME_FLOAT,
"Meander_targ_dwell_min", &f_meander_target_dwell_frames_min, NP, NP, 0, ME_DEC,
"Meander_targ_dwell_max", &f_meander_target_dwell_frames_max, NP, NP, 0, ME_DEC,
"Meander_targ_sigma", &f_meander_target_sigma, NP, NP, 0, ME_FLOAT,
"Meander_targ_cutoff", &f_meander_target_cutoff, NP, NP, 0, ME_FLOAT,
"Diffn_targ_dwell_min", &f_diffusion_target_dwell_frames_min, NP, NP, 0, ME_DEC,
"Diffn_targ_dwell_max", &f_diffusion_target_dwell_frames_max, NP, NP, 0, ME_DEC,
"Diffn_targ_sigma", &f_diffusion_target_sigma, NP, NP, 0, ME_FLOAT,
"Diffn_targ_cutoff", &f_diffusion_target_cutoff, NP, NP, 0, ME_FLOAT,
"record_cam_position", &f_jitter_record_campos, NP, NP, 0, ME_DEC,
NS,
};

char hm_jitter[] = "Enter dwell in frames\nEnter sigma in degrees\nEnter cutoff in units of sigma\n";

VLIST savecommands_vl[] = {
"save_commands(0/1/2))", &f_savecommands, NP, NP, 0, ME_DEC,
"folder", f_savecommandsfolder, NP, NP, 0, ME_STR,
NS,
};

char hm_savecommands[] = "Set save_commands to 1 to save.\nFolder is used to save command files.\nData file name used, .cmd appended.\n";


MENU umenus[] = {
{"Main", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{"separator", NP}, 
{"trial_types", &trialtypes_vl, NP, NP, 0, NP, hm_trialtypes}, 
{"timing", &timing_vl, NP, NP, 0, NP, hm_timing}, 
{"fixation", &fixpt_vl, NP, NP, 0, NP, hm_fixpt}, 
{"fix_positions", &fixpt_pos_vl, NP, NP, 0, NP, hm_fixpt_pos}, 
{"looking_dir", &looking_vl, NP, NP, 0, NP, hm_looking},
{"jump", &jump_vl, NP, NP, 0, NP, hm_jump}, 
{"jitters", &jitter_vl, NP, NP, 0, NP, hm_jitter},
{"reward", &reward_vl, NP, NP, 0, NP, hm_reward}, 
{"blink", &blink_vl, NP, NP, 0, NP, hm_blink}, 
{"blip", &blip_vl, NP, NP, 0, NP, hm_blip}, 
{"ustim", &ustim_vl, NP, NP, 0, NP, hm_ustim}, 
{"separator", NP}, 
{"viewing volume", &vvol_vl, NP, NP, 0, NP, hm_vvol},
{"target", &dualtarget_vl, NP, NP, 0, NP, hm_dualtarget},
{"groundplane", &groundplane_vl, NP, NP, 0, NP, hm_groundplane},
{"background", &background_vl, NP, NP, 0, NP, hm_background}, 
{"fakjoy", &fakejoy_vl, NP, NP, 0, NP, hm_fakejoy}, 
{"savecommands", &savecommands_vl, NP, NP, 0, NP, hm_savecommands},
{NS},
};


/* 
 * Reset camera position and orientation. 
 * CAUTION- this may affect replay????
 * djs 10-30-2014
 */

void campos_reset(void)
{
	f_cam_position[0] = 0;
	f_cam_position[2] = 0;
	f_cam_trajectory = 0.0;
}

USER_FUNC ufuncs[] = {
	{"reset",	&campos_reset, "void"},
	{""},
};






/* local functions. */

int my_check_for_handle();
void init_steering(void);
int my_render_init();
int my_check_for_handle();
int my_exp_init();
int alloff();
int my_trial_done(int i);
int joystick_value();
void blink_prepare();
int blink_update(int frame);
void blip_prepare();
int blip_update(int frame, float *poffset);
int reward_check_window();
int reward_init();
int check_grace_time();
int check_steering_and_wait_time();
int check_steering();
void show_trials_status();
float random_targ_offset_degrees();
int fixation_statistics(int itype);
int save_to_buffer(char *buffer, int *nused, void *data, int nbytes);

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
	char envtmp[256];
	
	dprintf("Initializing paradigm\n");

	// initialize tcpip - must only do this OR gpib - NOT BOTH!!!!
	status = init_tcpip(f_local_addr, f_remote_addr, f_remote_port, 0);

	// Initialize rtvars
	rtvarex_init();
		
	// In case we did a Reset States
	f_going = 0;
	f_cam_trajectory = 0;
	f_jitter_trajectory_delta = 0.0;
	f_jitter_target_delta = 0.0;

	// initialize pixel conversion
	if (initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, x_resolution, y_resolution, f_screen_distance_MM))
	{
		dprintf("ERROR in initialize_pixel_conversion: \n"
				"x,y dimensions=(%d, %d)\n"
				"x,y resolution=(%d, %d)\n"
				"screen distance=%d\n", (int)x_dimension_mm, (int)y_dimension_mm, (int)x_resolution, (int)y_resolution, (int)f_screen_distance_MM);
	}

	/*
	 * Initialize queue for jitter action structs
	 */
	 
	if (!f_pmbq_jitter_actions)
	{
		f_pmbq_jitter_actions = mbq_create(MAX_JITTER_ACTIONS);
		if (!f_pmbq_jitter_actions)
		{
			dprintf("ERROR creating queue for jitter actions!");
		}
	}		
	
	
	/* 
	 * Clear and destroy queue for gaussian distributions
	 */
	 
	if (f_pmbqf_trajectory_distribution)
	{
		mbqf_destroy(f_pmbqf_trajectory_distribution);
		f_pmbqf_trajectory_distribution = NULL;
	}
	if (f_pmbqf_target_distribution)
	{
		mbqf_destroy(f_pmbqf_target_distribution);
		f_pmbqf_target_distribution = NULL;
	}
	if (f_pmbqf_meander_distribution)
	{
		mbqf_destroy(f_pmbqf_meander_distribution);
		f_pmbqf_meander_distribution = NULL;
	}
	if (f_pmbqf_diffusion_distribution)
	{
		mbqf_destroy(f_pmbqf_diffusion_distribution);
		f_pmbqf_diffusion_distribution = NULL;
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
	 * Check seed value (must be negative on first call)
	 */
	if (!f_seed_used)
	{
		if (f_seed > 0) f_seed *= -1;
		f_seed_used = 1;
	}

	// zero out handles and initialize counters.

	f_gp_handle = f_fixpt_handle = 0;
	f_ttrans_handle = f_gaussiandot_handle = f_plaindot_handle = 0;
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

	// compute cos(fovx/2), used in fixed looking trials. 
	f_cos_fovx_over_2 = f_screen_distance_MM / sqrt(x_dimension_mm*x_dimension_mm/2 + f_screen_distance_MM*f_screen_distance_MM);  
	
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

	/* Configure target transform.*/
	
	f_ttrans_dir = 0;
	f_ttrans.xoffset = -sin(f_ttrans_dir);
	f_ttrans.zoffset = cos(f_ttrans_dir);
	f_ttrans.d = f_ttrans_dist;
	f_ttrans.h = f_ttrans_dist * sin(f_ttrans_h * M_PI / 180.0);
	render_targettransform(&f_ttrans);
	
	/* Configure target transform and target dot. Do not send commands! */

/*	
	f_ttrans.xoffset = -sin(f_ttrans_dir);
	f_ttrans.zoffset = cos(f_ttrans_dir);
	f_ttrans.d = f_targ_dist;
	f_ttrans.h = f_targ_dist * sin(f_targ_h * M_PI / 180.0);
	f_targetdot.xorigin = 0;
	f_targetdot.yorigin = 0;
	f_targetdot.xsize = f_targetdot.ysize = f_targ_dist * sin(f_targ_size * M_PI / 180.0);
	f_targetdot.depth = 2;

	 * override color and size for testing
	f_targetdot.r = f_targ_color[0];
	f_targetdot.g = f_targ_color[1];
	f_targetdot.b = f_targ_color[2];
	f_targetdot.r = 255;
	f_targetdot.g = 255;
	f_targetdot.b = 0;
*/
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
	bcode_float(CH_REWARD_WINDOW_DEGREES, f_reward_window_degrees);
	bcode_int(CH_SCREEN_DISTANCE_MM, f_screen_distance_MM);
	bcode_int(CH_FAR_PLANE_DISTANCE, f_far_plane_distance);
	bcode_int(CH_CAMERA_HEIGHT, f_cam_height);
/*
	bcode_float(CH_TARGET_DIAM_DEGREES, f_targ_size);
	bcode_int(CH_TARG_COLOR_R, f_targ_color[0]);
	bcode_int(CH_TARG_COLOR_G, f_targ_color[1]);
	bcode_int(CH_TARG_COLOR_B, f_targ_color[2]);
	bcode_float(CH_TARG_DIST, f_targ_dist);
	bcode_float(CH_TARG_HEIGHT_DEGREES, f_targ_h);
*/
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
	bcode_int(CH_FIXPT_ACQTIMEOUT, f_acq_fail_timeout);
	bcode_int(CH_FIXPT_FIXTIME, f_fix_time);
	bcode_int(CH_INTERTRIAL_TIME, f_intertrial_time);
	bcode_int(CH_FIXPT_R, f_fixpt_color[0]);
	bcode_int(CH_FIXPT_G, f_fixpt_color[1]);
	bcode_int(CH_FIXPT_B, f_fixpt_color[2]);
	
	// djs 2-27-2015 Added in ID 412
	bcode_int(CH_FIXED_LOOKING, f_fixed_looking);
	bcode_int(CH_NFIXED_LOOKING, f_nfixed_looking);
	bcode_float(CH_FIXED_LOOKING_DEG_0, f_fixed_looking_angle_deg[0]);
	bcode_float(CH_FIXED_LOOKING_DEG_1, f_fixed_looking_angle_deg[1]);
	bcode_float(CH_FIXED_LOOKING_DEG_2, f_fixed_looking_angle_deg[2]);
	bcode_float(CH_FIXED_LOOKING_DEG_3, f_fixed_looking_angle_deg[3]);
	bcode_float(CH_FIXED_LOOKING_DEG_4, f_fixed_looking_angle_deg[4]);
	bcode_float(CH_FIXED_LOOKING_DEG_5, f_fixed_looking_angle_deg[5]);
	bcode_float(CH_FIXED_LOOKING_DEG_6, f_fixed_looking_angle_deg[6]);
	bcode_float(CH_FIXED_LOOKING_DEG_7, f_fixed_looking_angle_deg[7]);
	bcode_float(CH_FIXED_LOOKING_DEG_8, f_fixed_looking_angle_deg[8]);
	bcode_float(CH_FIXED_LOOKING_DEG_9, f_fixed_looking_angle_deg[9]);

	return status;
}


/* 
 * my_check_for_handle 
 * 
 * Called as an escape function ... e.g. 
 * 
 * to wherever on 1 % my_check_for_handle
 * 
 * This function will fetch 3 handles from render. They are saved
 * as f_gp_handle, f_targ_handle and f_fixpt_handle respectively. 
 * The counter f_handle_count must be zeroed before this is used. 
 * Since the allocation of graphic objects takes place early in the 
 * state set (render_init), the handle counter should get zeroed 
 * in the my_render_init() function. 
 *
 * On each invocation it checks once for a handle.If a handle is found 
 * it is assigned to the proper variable and the handle counter is 
 * incremented.  
 * 
 * Returns 0 if the handle counter is less than 3, or 1 if the handle 
 * counter is 3.
 */

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
			f_ttrans_handle = handle;
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

int my_target_init()
{
	int status = 0;

	/*
	 * Check that at least one target is used. 
	 */

	if (!f_use_plaindot && !f_use_gaussiandot)
	{
		int i;
		for (i=0; i<10; i++)
			dprintf("ERROR! Must use plain dot or gaussian dot (or both). See Target Menu\n");
	}
	
	if (f_use_plaindot)
	{
		f_plaindot.xsize = f_plaindot.ysize = f_ttrans_dist * sin(f_plaindot_radius * M_PI / 180.0f);
		f_plaindot.xorigin = f_plaindot.yorigin = 0;
		f_plaindot.depth = f_plaindot_depth;
		f_plaindot.r = f_plaindot_color[0];
		f_plaindot.g = f_plaindot_color[1];
		f_plaindot.b = f_plaindot_color[2];
		render_dot_p(&f_plaindot, f_ttrans_handle);
	}
	
	if (f_use_gaussiandot)
	{
		f_gaussiandot.origin[0] = f_gaussiandot.origin[1] = 0;
		f_gaussiandot.offset[0] = f_gaussiandot.offset[1] = 0;
		f_gaussiandot.pseed = f_gaussiandot.cseed = 0;
		f_gaussiandot.radius = f_ttrans_dist * sin(f_gaussiandot_radius * M_PI / 180.0);
		f_gaussiandot.sigma = f_gaussiandot_sigma;
		f_gaussiandot.coherence = (float)f_gaussiandot_coherence/100.0f;
		f_gaussiandot.orientation = 0;
		f_gaussiandot.npts = f_gaussiandot_npts;
		f_gaussiandot.pointsize = f_gaussiandot_pointsize;
		f_gaussiandot.r = f_gaussiandot_color[0];
		f_gaussiandot.g = f_gaussiandot_color[1];
		f_gaussiandot.b = f_gaussiandot_color[2];
		f_gaussiandot.a = 0;
		render_gaussiandot_p(&f_gaussiandot, f_ttrans_handle);
	}

	return status;
}

int my_check_for_target_handle()
{
	int status=0;
	int handle=0;
	
	if (render_check_for_handle(&handle))
	{
		if (f_use_plaindot && f_plaindot_handle==0)
		{
			f_plaindot_handle = handle;
			dprintf("Got plaindot handle = %d\n", f_plaindot_handle);
		}
		else if (f_use_gaussiandot && f_gaussiandot_handle==0)
		{
			f_gaussiandot_handle = handle;
			dprintf("Got gaussiandot handle = %d\n", f_gaussiandot_handle);
		}		
	}

	status = (	(!f_use_plaindot || (f_use_plaindot && f_plaindot_handle!=0)) &&
				(!f_use_gaussiandot || (f_use_gaussiandot && f_gaussiandot_handle!=0)));

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
	int status=0;
	int i, j;
	
	dprintf("Initializing experiment\n");
	f_all_done = 0;
	f_exp_abort = 0;
	f_ustim_flag = 0;
	f_ustim_ready_flag = 0;
	alloff();
	render_frame(0);
	
	/* 
	 * Free mem if already allocated.
	 */

	if (f_pfixpt_x) free(f_pfixpt_x);
	if (f_pfixpt_y) free(f_pfixpt_y);
	if (f_pfixed_looking_angle_deg) free(f_pfixed_looking_angle_deg);
	if (f_pwdpos_x) free(f_pwdpos_x);
	if (f_pwdpos_y) free(f_pwdpos_y);
	 
	/* 
	 * Initialize spin detector  sliding window 
	 */
	 
	if (f_pslider)
	{
		slider_destroy(f_pslider);
		f_pslider = NULL;
	}
	if (f_spin_window_ms > 0)
	{
		dprintf("Creating sliding window length %d\n", (long)((float)f_spin_window_ms/1000.0f*f_frames_per_second));
		f_pslider = slider_create((long)((float)f_spin_window_ms/1000.0f*f_frames_per_second));
	}
	else
	{
		dprintf("Spin window size = 0, no spin window created.\n");
	}

				
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
	 * Determine trial types.There are three ways to specify trial types. 
	 * Check f_use_fixpt_pos_file. If its 1, open the file in f_fixpt_pos_file.
	 * If that was 0, check f_use_fixpt_pos_menu. If its 1, use the values from 
	 * the fixpt positions menu, where f_nfixpt tells us how many to use. 
	 * If f_use_fixpt_pos_menu is also 0, then check  f_use_replay_file. If that's
	 * 1, use the replay file specified in f_replayfile.  
	 * 
	 * Either way, calloc and populate the arrays f_pfixpt_x/y and f_pwdpos_x/y
	 * to specify fixation positions and eye window positions for each trial
	 * type. The menu values f_fixpt_x/y_degrees are ONLY USED IN THE MENU!!!
	 * 
	 */

	if (f_use_fixpt_pos_file)
	{
		if (strlen(f_fixpt_pos_file)>0)
		{
			PEyepos ep = eyepos_load_fp_and_wd(f_fixpt_pos_file);
			if (!ep)
			{
				rxerr("Cannot load eyepos file! Check filename!");
				f_exp_abort = 1;
				return ERRORCD;
			}
			else
			{
				float x, y;
				long wdx, wdy;
				f_ntrialtypes = ep->size(ep);
				f_pfixpt_x = (float *)calloc(f_ntrialtypes, sizeof(float));
				f_pfixpt_y = (float *)calloc(f_ntrialtypes, sizeof(float));
				f_pwdpos_x = (long *)calloc(f_ntrialtypes, sizeof(long));
				f_pwdpos_y = (long *)calloc(f_ntrialtypes, sizeof(long));
		
				for (i=0; i<f_ntrialtypes; i++)
				{
					if (ep->getFP(ep, i, &x, &y) || ep->getWD(ep, i, &wdx, &wdy))
					{
						rxerr("Error fetching eye positions. Check config file.");
						f_exp_abort = 1;
						eyepos_destroy(ep);
						return ERRORCD;
					}
					f_pfixpt_x[i] = x;		 
					f_pfixpt_y[i] = y;		 
					f_pwdpos_x[i] = wdx;
					f_pwdpos_y[i] = wdy;
				}
				eyepos_destroy(ep);
			}
				
		}
		else
		{
			rxerr("Specify eyepos file!");
			f_exp_abort = 1;
		}
	}
	else 
	{
		if (f_use_fixpt_pos_menu)
		{
			if (f_nfixpt > 0)
			{
				if (!f_fixed_looking)
				{
					f_ntrialtypes = f_nfixpt;
					f_pfixpt_x = (float *)calloc(f_nfixpt, sizeof(float));
					f_pfixpt_y = (float *)calloc(f_nfixpt, sizeof(float));
					f_pwdpos_x = (long *)calloc(f_nfixpt, sizeof(long));
					f_pwdpos_y = (long *)calloc(f_nfixpt, sizeof(long));
					f_pfixed_looking_angle_deg = (float *)calloc(f_nfixpt, sizeof(float));
					for (i=0; i<f_nfixpt; i++)
					{
						f_pfixpt_x[i] = f_fixpt_x_degrees[i];		 
						f_pfixpt_y[i] = f_fixpt_y_degrees[i];		 
						f_pwdpos_x[i] = (long)(f_fixpt_x_degrees[i]*10);
						f_pwdpos_y[i] = (long)(f_fixpt_y_degrees[i]*10);
						f_pfixed_looking_angle_deg[i] = 9999.99;
					}
				}
				else
				{
					if (f_nfixed_looking > 0)
					{
						f_ntrialtypes = f_nfixpt * f_nfixed_looking;
						f_pfixpt_x = (float *)calloc(f_ntrialtypes, sizeof(float));
						f_pfixpt_y = (float *)calloc(f_ntrialtypes, sizeof(float));
						f_pwdpos_x = (long *)calloc(f_ntrialtypes, sizeof(long));
						f_pwdpos_y = (long *)calloc(f_ntrialtypes, sizeof(long));
						f_pfixed_looking_angle_deg = (float *)calloc(f_ntrialtypes, sizeof(float));
						for (j=0; j<f_nfixed_looking; j++)
						{
							for (i=0; i<f_nfixpt; i++)
							{
								int index = j*f_nfixpt + i;
								f_pfixpt_x[index] = f_fixpt_x_degrees[i];		 
								f_pfixpt_y[index] = f_fixpt_y_degrees[i];		 
								f_pwdpos_x[index] = (long)(f_fixpt_x_degrees[i]*10);
								f_pwdpos_y[index] = (long)(f_fixpt_y_degrees[i]*10);
								f_pfixed_looking_angle_deg[index] = f_fixed_looking_angle_deg[j];
								dprintf("index %d looking*100=%d\n", index, (int)(f_fixed_looking_angle_deg[j]*100));
							}
						}
					}
					else
					{
						rxerr("Set number of looking directions in looking_dir menu.");
						f_exp_abort = 1;
					}		
				}				
			}
			else
			{
				rxerr("Set number of fixpts in fixpt positions menu.");
				f_exp_abort = 1;
			}
		}
		else
		{
			if (f_use_replay_file)
			{
				/* 
				 * Clear out any previously loaded trials.
				 */
				 
				r3p_clear();

				/*
				 * User must have given a valid filename.
				 */
				if (strlen(f_replaybase)>0 && strcmp(f_replaybase, "NONE"))
				{	
					int nreplayloaded;			/* The number of trials loaded */
					int nreplayfixpt = 0;	/* the # of fixpts that will be generated */
					char fullpath[128];
					FILE *pfile;
	
					/* 
					 * This is the replay case.
					 * We must configure the trial condition arrays and the random trial generator. 
					 */
					 
					dprintf("replay file path \"%s\"\n", f_replaypath); 
					dprintf("replay file base \"%s\"\n", f_replaybase); 
					fullpath[0]=0;
					strcat(strcat(strcat(strcat(fullpath, f_replaypath), "/"), f_replaybase), ".cmd");
					dprintf("replay file name \"%s\"\n", fullpath); 
					pfile = fopen(fullpath, "rb");
					if (!pfile)
					{
						dprintf("Cannot open replay file %s\n", fullpath);
						rxerr("Cannot open replay file.");
						f_exp_abort = 1;
					}
					else
					{
					
						/*
						 * Load trials from this file....
						 */
						
						nreplayloaded = r3p_get_trials_min(pfile, f_replay_min_frames);
						if (nreplayloaded == 0)
						{
							dprintf("No trials loaded from %s\n", fullpath);
							rxerr("No trials loaded from replay file.");
							f_exp_abort = 1;
						}
						else
						{
							dprintf("Loaded %d trials from replay file %s.\n", nreplayloaded, fullpath);
						}
						fclose(pfile);
						
						
						/* 
						 * Now set up fixpt positions. 
						 */
						f_ntrialtypes = nreplayloaded;
						if (f_replay_reflect) f_ntrialtypes = nreplayloaded*2;
						if (!f_replay_reflect)
						{
							R3PTrialP ptrial;
							f_pfixpt_x = (float *)calloc(f_ntrialtypes, sizeof(float));
							f_pfixpt_y = (float *)calloc(f_ntrialtypes, sizeof(float));
							f_pwdpos_x = (long *)calloc(f_ntrialtypes, sizeof(long));
							f_pwdpos_y = (long *)calloc(f_ntrialtypes, sizeof(long));
							for (i=0; i<f_ntrialtypes; i++)
							{
								ptrial = r3p_get_trial(i);
								f_pfixpt_x[i] = ptrial->header.fixpt_x;		 
								f_pfixpt_y[i] = ptrial->header.fixpt_y;
								f_pwdpos_x[i] = (long)(ptrial->header.fixpt_x*10);
								f_pwdpos_y[i] = (long)(ptrial->header.fixpt_y*10);
							}	
						}
						else
						{
							R3PTrialP ptrial;
							f_pfixpt_x = (float *)calloc(f_ntrialtypes, sizeof(float));
							f_pfixpt_y = (float *)calloc(f_ntrialtypes, sizeof(float));
							f_pwdpos_x = (long *)calloc(f_ntrialtypes, sizeof(long));
							f_pwdpos_y = (long *)calloc(f_ntrialtypes, sizeof(long));
							for (i=0; i<nreplayloaded; i++)
							{
								ptrial = r3p_get_trial(i);
								f_pfixpt_x[i] = ptrial->header.fixpt_x;		 
								f_pfixpt_y[i] = ptrial->header.fixpt_y;
								f_pwdpos_x[i] = (long)(ptrial->header.fixpt_x*10);
								f_pwdpos_y[i] = (long)(ptrial->header.fixpt_y*10);
	
								f_pfixpt_x[i+nreplayloaded] = ptrial->header.fixpt_x;		 
								f_pfixpt_y[i+nreplayloaded] = ptrial->header.fixpt_y;
								f_pwdpos_x[i+nreplayloaded] = (long)(ptrial->header.fixpt_x*10);
								f_pwdpos_y[i+nreplayloaded] = (long)(ptrial->header.fixpt_y*10);
							}	
						}						
					}
				}
				else
				{
					rxerr("You must specify a replay filename!");
					f_exp_abort = 1;
				}
			}
			else
			{
				rxerr("You must specify a trial type selector: use_fixpt_pos_file, use_fixpt_pos_menu, or use_replay_file");
				f_exp_abort = 1;
			}
		}		


		/* Don't do any more if we had an error above that will abort expt. */
		if (f_exp_abort) return status;




	
		/* Initialize trial generator */
		if (f_prtgen)
		{
			rtg_destroy(f_prtgen);
		}
		dprintf("Create random trial generator: %d trialtypes, %d trials each, blocksize %d\n", f_ntrialtypes, f_ntrials, f_nblocksize);
		f_prtgen = rtg_create(f_ntrialtypes, f_ntrials, f_nblocksize);
			
		/* 
		 * Initialize buffer for saving data. Don't save render commands, instead save enough data to reproduce the 
		 * commands themselves. Write data in to a char buffer, then dump it later to file after trial is done. 
		 * For now we must calculate how big the buffer should be. 
		 * Calculation of buffer size makes the following assumptions:
		 * Assuming approx 28 bytes per frame of updates. This assumes that for each update we will save the following:
		 * ijoyh
		 * f_camn_trajectory
		 * f_ttrans_dir
		 * f_jitter_trajectory_delta
		 * f_jitter_target_delta
		 * meander_target_delta
		 * diffusion_target_delta
		 * 
		 * There is also some overhead for data saved every trial, including (28 bytes)
		 * fixpt x
		 * fixpt y
		 * f_speed
		 * camera initial x
		 * camera initial y
		 * camera initial z
		 * f_cam_height  
		 */
		if (f_savecommands==1)
		{
			if (f_jumps_per_trial > 0)
			{
				f_savecommandsbuffersize = 50 * f_frames_per_second * f_ms_jump_avg / 1000 * f_jumps_per_trial;
			}
			else
			{
				f_savecommandsbuffersize = 50 * f_frames_per_second * f_trial_max_length_s;
			}  
			dprintf("Allocating buffer of %d bytes for saving trial data.\n", f_savecommandsbuffersize);
			f_savecommandsbuffer = (char *)malloc(f_savecommandsbuffersize);
			f_savecommandsbufferused = 0;
			if (!f_savecommandsbuffer) 
			{
				rxerr("Cannot allocate buffer for saving trial data");
				f_savecommands = 0;
				f_exp_abort = 1;
			}
			
			/* 
			 * Check path and filename. Must be saving data (i.e. we must have a data file name in 
			 * i_b->inthdr.i_name). Hopefully user gave us a valid folder in f_savecommandsfolder....
			 * but if not we just stick the files in /root
			 */
			if (strlen(f_savecommandsfolder) == 0)
			{
				strcpy(f_savecommandsfolder, "/root");
			}
			if (f_savecommandsfolder[strlen(f_savecommandsfolder)-1] == '/')
			{
				f_savecommandsfolder[strlen(f_savecommandsfolder)-1] = 0;
			}
			dprintf("Savecommands path is %s\n", f_savecommandsfolder);
			
			if (strlen(i_b->inthdr.i_name) == 0)
			{
				dprintf("Must be saving data to save commands.\n");
				rxerr("Must be saving data to save commands.");
				f_savecommands = 0;
				f_exp_abort = 1;
			}
			else
			{
				strcat(f_savecommandsfile, f_savecommandsfolder);
				strcat(f_savecommandsfile, "/");
				strcat(f_savecommandsfile, i_b->inthdr.i_name);
				strcat(f_savecommandsfile, ".cmd");
				dprintf("Saving commands to file %s\n", f_savecommandsfile);
			}
			
		}
	} 
	return status;
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

	jumpTimeFrames = (int)nrand_expl(&f_seed, psbs->jmin, psbs->tau, -1); 

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
	char astr[32];
	switch (paction->type)
	{
		case ACTIONTYPE_JUMP:
		{
			strcpy(astr, "JUMP");
			break;
		}
		case ACTIONTYPE_BLIP:
		{
			strcpy(astr, "BLIP");
			break;
		}
		case ACTIONTYPE_BLINK:
		{
			strcpy(astr, "BLNK");
			break;
		}
		case ACTIONTYPE_TRIALEND:
		{
			strcpy(astr, "TEND");
			break;
		}
		case ACTIONTYPE_JITTERTARG:
		{
			strcpy(astr, "JTTG");
			break;
		}
		case ACTIONTYPE_JITTERTRAJ:
		{
			strcpy(astr, "JTTJ");
			break;
		}
		case ACTIONTYPE_MEANDERTARG:
		{
			strcpy(astr, "MNTG");
			break;
		}
		case ACTIONTYPE_USTIM:
		{
			strcpy(astr, "USTM");
			break;
		}
		case ACTIONTYPE_DIFFUSIONTARG:
		{
			strcpy(astr, "DFTG");
			break;
		}
		default:
		{
			sprintf(astr, "???(%d)", paction->type);
			break;
		}	
	}

	sprintf(str, "%d-%d %f %s\n", paction->start_frame, paction->end_frame, paction->fval, astr); 
	return str;
}

int generate_trial()
{
	PstTree ptree;
	struct segment_builder_struct sbs;
	int nactions=0;
	int nblips=0;
	int nblinks=0;
	int nustim=0;
	int i;
	char s[128];
	int status=0;

	if (f_jumps_per_trial == 0)
	{
		sbs.last = f_trial_max_length_s * (float)f_frames_per_second;
		sbs.jmin = 0;
		sbs.tau = 0;
		sbs.guard_before = 0;
		sbs.guard_after = 0;
		ptree = segtree_create(segtree_compare_segments_default);
	}
	else
	{
		
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
	}
	
	/*
	 * Now we can compute how many blips, blinks, and ustim we'll need, 
	 * and then we can calculate how many total actions are needed and 
	 * allocate the space for them.The total time for the trial includes 
	 * include a "tail" added after the last jump - so the trial doesn't
	 * end after the last jump occurs. Later I'll add in a segment with 
	 * length equal to the average jump time (jmin + tau), but for now 
	 * include the time in the rate calculations.
	 */

	nblips = (int)((float)(sbs.last + sbs.jmin + sbs.tau)/(float)f_frames_per_second * f_blip_rate);
	nblinks = (int)((float)(sbs.last + sbs.jmin + sbs.tau)/(float)f_frames_per_second * f_blink_rate);
	nustim = (int)((float)(sbs.last + sbs.jmin + sbs.tau)/(float)f_frames_per_second * f_ustim_rate);
	nactions = f_jumps_per_trial + 1 + nblips + nblinks + nustim;
	f_all_actions = (PAction)calloc(nactions, sizeof(struct action));
	n_all_actions = 0; /* Will be incremented as we use these up.... */
	if (f_verbose & DEBUG_BIT)
	{
		dprintf("nblips %d nblinks %d ustim %d nactions %d\n", nblips, nblinks, nustim, nactions);
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
	 * Generate random numbers for this trial. There are three possible gaussian distributions, 
	 * pre-generate the numbers for each of those. 
	 */
	
	if (f_jitter_trajectory_dwell_frames > 0)
	{
		int n = sbs.last / f_jitter_trajectory_dwell_frames + 1;
		f_pmbqf_trajectory_distribution = mbqf_create(n);
		if (!f_pmbqf_trajectory_distribution)
		{
			rxerr("Cannot allocate memory for traj jitter distruribution!");
			return 1;
		}
		else
		{
			/*
			 * Check that the sigma and cutoff are reasonable. 
			 */
			 
			if (f_jitter_trajectory_sigma <= 0 || (f_jitter_trajectory_cutoff > 0 && f_jitter_trajectory_cutoff < 0.5))
			{
				rxerr("Bad value for jitter trajectory sigma and/or cutoff");
				return 1;
			}
			
			/* 
			 * Now populate the distribution
			 */
			 
			dprintf("Generating %d values for trajectory jitter...\n", n);
			for (i=0; i<n; i++) mbqf_push(f_pmbqf_trajectory_distribution, nrand_gaus(&f_seed, 0.0, f_jitter_trajectory_sigma, f_jitter_trajectory_cutoff)*M_PI/180.0f);
			dprintf("Generating %d values for trajectory jitter...done\n", n);
		}
	}			 
			
	
	if (f_jitter_target_dwell_frames > 0)
	{
		int n = sbs.last / f_jitter_target_dwell_frames + 1;
		f_pmbqf_target_distribution = mbqf_create(n);
		if (!f_pmbqf_target_distribution)
		{
			rxerr("Cannot allocate memory for target jitter distruribution!");
			return 1;
		}
		else
		{
			/*
			 * Check that the sigma and cutoff are reasonable. 
			 */
			 
			if (f_jitter_target_sigma <= 0 || (f_jitter_target_cutoff > 0 && f_jitter_target_cutoff < 0.5))
			{
				rxerr("Bad value for jitter target sigma and/or cutoff");
				return 1;
			}
			
			/* 
			 * Now populate the distribution
			 */
			 
			dprintf("Generating %d values for target jitter...\n", n);
			for (i=0; i<n; i++) mbqf_push(f_pmbqf_target_distribution, nrand_gaus(&f_seed, 0.0, f_jitter_target_sigma, f_jitter_target_cutoff)*M_PI/180.0f);
			dprintf("Generating %d values for target jitter...done\n", n);
		}
	}			 

	if (f_meander_target_dwell_frames_min > 0)
	{
		int n = sbs.last / f_meander_target_dwell_frames_min + 1;
		f_pmbqf_meander_distribution = mbqf_create(n);
		if (!f_pmbqf_meander_distribution)
		{
			rxerr("Cannot allocate memory for target meander distribution!");
			return 1;
		}
		else
		{
			/*
			 * Check that the sigma and cutoff are reasonable. 
			 */
			 
			if (f_meander_target_sigma <= 0 || (f_meander_target_cutoff > 0 && f_meander_target_cutoff < 0.5))
			{
				rxerr("Bad value for meander target sigma and/or cutoff");
				return 1;
			}
			
			/* 
			 * Now populate the distribution
			 */
			 
			dprintf("Generating %d values for target meander...\n", n);
			for (i=0; i<n; i++) mbqf_push(f_pmbqf_meander_distribution, nrand_gaus(&f_seed, 0.0, f_meander_target_sigma, f_meander_target_cutoff)*M_PI/180.0f);
			dprintf("Generating %d values for target meander...done\n", n);
		}
	}			 

	if (f_diffusion_target_dwell_frames_min > 0)
	{
		int n = sbs.last / f_diffusion_target_dwell_frames_min + 1;
		f_pmbqf_diffusion_distribution = mbqf_create(n);
		if (!f_pmbqf_diffusion_distribution)
		{
			rxerr("Cannot allocate memory for target diffusion distribution!");
			return 1;
		}
		else
		{
			/*
			 * Check that the sigma and cutoff are reasonable. 
			 */
			 
			if (f_diffusion_target_sigma <= 0 || (f_diffusion_target_cutoff > 0 && f_diffusion_target_cutoff < 0.5))
			{
				rxerr("Bad value for diffusion target sigma and/or cutoff");
				return 1;
			}
			
			/* 
			 * Now populate the distribution
			 */
			 
			dprintf("Generating %d values for target diffusion...\n", n);
			for (i=0; i<n; i++) mbqf_push(f_pmbqf_diffusion_distribution, nrand_gaus(&f_seed, 0.0, f_diffusion_target_sigma, f_diffusion_target_cutoff)*M_PI/180.0f);
			dprintf("Generating %d values for target diffusion...done\n", n);
		}
	}			 

			
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
			 
			/*
			 * djs 2-3-2010
			 * Modify to use nr ran1 routine.
			 * 
			r = (double)(ivunif(0, 32767)+1)/32768.0;
			blipframe = (int)ceil(r*sbs.last);			
			*/
			blipframe = nrand_intv(&f_seed, 1, sbs.last);
			/*printf("Check blip_frame %d seg %d-%d\n", blipframe, blipframe-blip_pad, blipframe+blip_pad);*/
			pseg = segtree_create_segment(blipframe - blip_guard_frames, blipframe + blip_duration_frames + blip_guard_frames, NULL);
			if ((iover=segtree_overlaps(ptree, pseg, 0, NULL)) == 0)
			{
				f_all_actions[n_all_actions].start_frame = blipframe;
				f_all_actions[n_all_actions].end_frame = blipframe+blip_duration_frames;
				f_all_actions[n_all_actions].type = ACTIONTYPE_BLIP;
				/*f_all_actions[n_all_actions].fval = f_blip_max_ang_velocity_deg_per_sec * M_PI/180.0f / f_frames_per_second * (ifuniv(1)*2-1);*/
				f_all_actions[n_all_actions].fval = f_blip_max_ang_velocity_deg_per_sec * M_PI/180.0f / f_frames_per_second * nrand_sign(&f_seed);
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
			return 1;
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
			blinkframe = nrand_intv(&f_seed, 1, sbs.last);
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
			return 1;
		}
	}


	/* 
	 * ... and the ustim....
	 */
	 
	if (f_ustim_rate > 0)
	{
		int ustim_duration_frames;	/* The ustim is spread over this many frames */
		int ustim_guard_frames;		/* guard frames on either side of ustim where no jumps may occur */
		int ustimcounter = 0;		/* Counter of ustim added */
		int failcounter = 0;		/* Counter of failures to add ustim...overlaps, get it? */
		double r;
		int ustimframe;
		
		ustim_duration_frames = (int)((float)f_ustim_duration_ms / 1000.0f * f_frames_per_second);
		ustim_guard_frames = (int)((float)f_ustim_guard_ms / 1000.0f * f_frames_per_second);
		
		dprintf("Generating %d ustim: ", nustim);
		ustimcounter = 0;
		failcounter = 0;
		while (ustimcounter < nustim && failcounter < 1000)
		{
			PstSegment pseg;
			int iover;

			/* 
			 * ustimframe will be in 1<= ustimframe <= sbs.last. See discussion
			 * above for blipframe. 
			 */
			ustimframe = nrand_intv(&f_seed, 1, sbs.last);
			pseg = segtree_create_segment(ustimframe - ustim_guard_frames, ustimframe + ustim_duration_frames + ustim_guard_frames, NULL);
			if ((iover=segtree_overlaps(ptree, pseg, 0, NULL)) == 0)
			{
				f_all_actions[n_all_actions].start_frame = ustimframe;
				f_all_actions[n_all_actions].end_frame = ustimframe+ustim_duration_frames;
				f_all_actions[n_all_actions].type = ACTIONTYPE_USTIM;
				f_all_actions[n_all_actions].fval = 0;
				pseg->extra = f_all_actions + n_all_actions;
				segtree_insert(ptree, pseg);
				ustimcounter++;
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
			 
			dprintf("ERROR! Failed to add enough ustim!\n");
			return 1;
		}
	}

	/*
	 * OK, now that's all done, and we can sort the actions by their start frame.  
	 */
	 
	f_candidate_action_index = 0;
	n_current_actions = 0;
	f_ordered_actions = (PAction *)calloc(n_all_actions, sizeof(PAction));
	for (i=0; i<n_all_actions; i++) f_ordered_actions[i] = f_all_actions+i;
	qsort(f_ordered_actions, n_all_actions, sizeof(PAction), compare_actions);
	
	if (f_verbose & DEBUG_BIT) 
	{
		dprintf("Action# type startframe-endframe\n");
		for (i=0; i<n_all_actions; i++)
		{
			dprintf("%d %d %d-%d\n", i, f_ordered_actions[i]->type, f_ordered_actions[i]->start_frame, f_ordered_actions[i]->end_frame);
		}
	}
	/* Finally, destroy the segtree and segments. */
	
	segtree_destroy(ptree, 1, 0);

	if (f_verbose & DEBUG_BIT) dprintf("generate_trial - done.\n");
	
	return status;
}

void queue_actions(int frame)
{
	float *pf;
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
	
	/*
	 * Jitter handling. 
	 * 
	 * Jitters and the meandering target are handled like actions (i.e. blips and blinks and jumps)
	 * are handled - using the action queue. However, unlike those action types, jitters are inserted
	 * on the fly, using the frame number and the dwell time for each type of jitter. 
	 * 
	 * The action structs are taken from a queue (and returned to the queue in dequeue_actions) rather than 
	 * pre-allocating them for an entire trial. 
	 */

	if (f_jitter_trajectory_dwell_frames && frame%f_jitter_trajectory_dwell_frames==0)
	{	
		PAction paction;
		paction = (PAction)mbq_pop(f_pmbq_jitter_actions);
		if (!paction)
		{
			rxerr("Cannot get action from queue! Check queue size and whether elements are being returned.");
		}
		else
		{
			paction->start_frame = frame;
			paction->end_frame = frame + f_jitter_trajectory_dwell_frames;
			paction->type = ACTIONTYPE_JITTERTRAJ;
			pf = mbqf_pop(f_pmbqf_trajectory_distribution);
			if (!pf)
			{
				/* TODO: Handle this error gracefully! */
				rxerr("Error: trajectory jitter distribution queue underflow!");
			}
			paction->fval = *pf;
			if (f_verbose & DEBUG_BIT) dprintf("Q:%s", action_str(paction, s));
			if (n_current_actions >= MAX_CURRENT_ACTIONS)
			{
				/* 
				 * TODO: Error ecode for this situation. 
				 */
				 
				dprintf("ERROR! Too many current actions (%d)- increase MAX_CURRENT_ACTIONS!\n", n_current_actions);
				return;
			}
			f_current_actions[n_current_actions++] = paction;
		}
	}

	if (f_jitter_target_dwell_frames && frame%f_jitter_target_dwell_frames==0)
	{	
		PAction paction;
		paction = (PAction)mbq_pop(f_pmbq_jitter_actions);
		if (!paction)
		{
			rxerr("Cannot get action from queue! Check queue size and whether elements are being returned.");
		}
		else
		{
			paction->start_frame = frame;
			paction->end_frame = frame + f_jitter_target_dwell_frames;
			paction->type = ACTIONTYPE_JITTERTARG;
			pf = mbqf_pop(f_pmbqf_target_distribution);
			if (!pf)
			{
				/* TODO: Handle this error gracefully! */
				rxerr("Error: target jitter distribution queue underflow!");
			}
			paction->fval = *pf;
			if (f_verbose & DEBUG_BIT) dprintf("Q:%s", action_str(paction, s));
			if (n_current_actions >= MAX_CURRENT_ACTIONS)
			{
				/* 
				 * TODO: Error ecode for this situation. 
				 */
				 
				dprintf("ERROR! Too many current actions (%d)- increase MAX_CURRENT_ACTIONS!\n", n_current_actions);
				return;
			}
			f_current_actions[n_current_actions++] = paction;
		}
	}

	if (f_meander_target_dwell_frames_min > 0 && f_meander_target_dwell_frames_max >= f_meander_target_dwell_frames_min && frame >= f_meander_target_next_action_frame)
	{	
		PAction paction;
		paction = (PAction)mbq_pop(f_pmbq_jitter_actions);
		if (!paction)
		{
			rxerr("Cannot get action from queue! Check queue size and whether elements are being returned.");
		}
		else
		{
			paction->start_frame = frame;
			paction->end_frame = frame + nrand_intv(&f_seed, f_meander_target_dwell_frames_min, f_meander_target_dwell_frames_max) - 1;
			f_meander_target_next_action_frame = paction->end_frame + 1;
			paction->type = ACTIONTYPE_MEANDERTARG;
			pf = mbqf_pop(f_pmbqf_meander_distribution);
			if (!pf)
			{
				/* TODO: Handle this error gracefully! */
				rxerr("Error: meander target distribution queue underflow!");
			}
			paction->fval = *pf;
			if (f_verbose & DEBUG_BIT) dprintf("Q:%s", action_str(paction, s));
			if (n_current_actions >= MAX_CURRENT_ACTIONS)
			{
				/* 
				 * TODO: Error ecode for this situation.
				 */
				 
				dprintf("ERROR! Too many current actions (%d)- increase MAX_CURRENT_ACTIONS!\n", n_current_actions);
				return;
			}
			f_current_actions[n_current_actions++] = paction;
		}
	}

	if (f_diffusion_target_dwell_frames_min > 0 && f_diffusion_target_dwell_frames_max >= f_diffusion_target_dwell_frames_min && frame == f_diffusion_target_next_action_frame)
	{	
		PAction paction;
		paction = (PAction)mbq_pop(f_pmbq_jitter_actions);
		if (!paction)
		{
			rxerr("Cannot get action from queue! Check queue size and whether elements are being returned.");
		}
		else
		{
			paction->start_frame = frame;
			paction->end_frame = frame; 
			f_diffusion_target_next_action_frame = frame + nrand_intv(&f_seed, f_diffusion_target_dwell_frames_min, f_diffusion_target_dwell_frames_max);
			paction->type = ACTIONTYPE_DIFFUSIONTARG;
			pf = mbqf_pop(f_pmbqf_diffusion_distribution);
			if (!pf)
			{
				/* TODO: Handle this error gracefully! */
				rxerr("Error: diffusion target distribution queue underflow!");
			}
			paction->fval = *pf;
			if (f_verbose & DEBUG_BIT) dprintf("Q:%s", action_str(paction, s));
			if (n_current_actions >= MAX_CURRENT_ACTIONS)
			{
				/* 
				 * TODO: Error ecode for this situation.
				 */
				 
				dprintf("ERROR! Too many current actions (%d)- increase MAX_CURRENT_ACTIONS!\n", n_current_actions);
				return;
			}
			f_current_actions[n_current_actions++] = paction;
		}
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
		/* 
		 * If the current frame is the end_frame for this action, remove it from current_actions. 
		 * If the action is also a JITTER type action, return it to the queue for those actions. 
		 */
		 
		if (f_current_actions[i]->end_frame == frame)
		{
			if (f_verbose & DEBUG_BIT) dprintf("D:%s", action_str(f_current_actions[i], s));
			
			if (f_current_actions[i]->type == ACTIONTYPE_JITTERTARG ||
				f_current_actions[i]->type == ACTIONTYPE_JITTERTRAJ ||
				f_current_actions[i]->type == ACTIONTYPE_MEANDERTARG ||
				f_current_actions[i]->type == ACTIONTYPE_DIFFUSIONTARG)
			{
				mbq_push(f_pmbq_jitter_actions, f_current_actions[i]);
			}
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
	int status = 0;		/* if set nonzero an ecode will be dropped with that value */
	long ttime;			/* When saving commands, save the timestamp associated with TRIALCD */
	

	/* Set next meander action frame so the first action will be queued (if needed) */
	f_meander_target_next_action_frame = 0;

	/* Set next meander action frame so the first action will be queued (if needed) 
	 * This has to be 1 because the first action will not be triggered unless the frame number is a
	 * direct hit. Since updates start happening for frame number 1....*/
	f_diffusion_target_next_action_frame = 1;
		
	/* Check for fixpt condition to apply to this trial. */
	f_trial_condition_index = f_prtgen->next(f_prtgen);
	if (f_trial_condition_index >= 0)
	{
		dprintf("Trial condition index: %d fixpt*100(%d,%d) looking*100(%d)\n", f_trial_condition_index, (int)(f_pfixpt_x[f_trial_condition_index]*100), (int)(f_pfixpt_y[f_trial_condition_index]*100), (int)(f_pfixed_looking_angle_deg[f_trial_condition_index]*100));

		/* 
		 * Initialize trial counters and flags
		 */
		
		f_frame_counter = 0;
		f_went_counter = 0;
		f_trial_end_flag = 0;
		f_jitter_trajectory_delta = 0;
		f_jitter_target_delta = 0;
		f_ustim_flag = 0;
		f_ustim_ready_flag = 0;
		f_replay_counter = 0;
		f_bonus_reward = 0;
		f_cos_reward_window = cos(f_reward_window_degrees * M_PI/180);
		
		/* 
		 * Initialize camera position and trajectory.
		 * Also initialize looking direction - this depends on the value
		 * of f_fixed_looking. If its 0, then the looking direction is the 
		 * same as the camera trajectory -- the direction we're headed. 
		 * If f_fixed_looking is nonzero, then set the looking direction to 
		 * a value here (and do not change it throughout the trial).
		 * 
		 */
		f_cam_position[0] = 0;
		f_cam_position[1] = (float)f_cam_height;
		f_cam_position[2] = 0;
		f_cam_trajectory = 0;

		if (f_fixed_looking)
		{
			f_cam_trajectory = f_pfixed_looking_angle_deg[f_trial_condition_index] * M_PI/180.0f;
			if (f_verbose & DEBUG_FIXED_BIT && f_fixed_looking)
			{
				dprintf("my_trial_init: H: %d\tL: %d\tT: %d\n", (int)(100*f_cam_trajectory * 180.0f/M_PI), (int)(100*f_pfixed_looking_angle_deg[f_trial_condition_index]), (int)(100*f_ttrans_dir * 180.0/M_PI));
			}
		}
		f_cam_looking[0] = -sin(f_cam_trajectory);
		f_cam_looking[1] = 0;
		f_cam_looking[2] = cos(f_cam_trajectory);

		render_camera(f_cam_position, f_cam_looking, f_cam_up);
			

		/* 
		 * ecodes to indicate start of trial and the fixation position condition.
		 */

		if (ecode(TRIALCD)) status = BADCD;
		ttime = i_b->i_time;
		bcode_float(CH_FIXATION_X, f_pfixpt_x[f_trial_condition_index]);
		bcode_float(CH_FIXATION_Y, f_pfixpt_y[f_trial_condition_index]);
		bcode_int(CH_WDPOS_X, f_pwdpos_x[f_trial_condition_index]);
		bcode_int(CH_WDPOS_Y, f_pwdpos_y[f_trial_condition_index]);
		
		// djs 2-27-2015 Added in ID=412
		bcode_float(CH_FIXED_LOOKING_ANGLE_DEGREES, f_pfixed_looking_angle_deg[f_trial_condition_index]);	

		/*
		 * Update fixation point object
		 */

		f_fixpt.xorigin = to_pixels(f_pfixpt_x[f_trial_condition_index]);
		f_fixpt.yorigin = to_pixels(f_pfixpt_y[f_trial_condition_index]);
		render_update(f_fixpt_handle, (void *)&f_fixpt, sizeof(DotStruct), HANDLE_ON);
	
		/* 
		 * Assign time values to states 
		 */
		 
		set_times("trial_init_pause", (long)f_trial_init_pause_time, -1);
		set_times("trial_allon_wait", (long)f_trial_allon_wait_time, -1);
		set_times("fixpt_acq", (long)f_acq_time, -1);
		set_times("fixpt_acq_fail_pause", (long)f_acq_fail_timeout, -1);
		set_times("fixpt_acq_noise_pause", (long)f_acq_noise_timeout, -1);
		set_times("fixpt_hold", (long)f_fix_time, -1);
		set_times("fixation_fail_pause", (long)f_fixation_fail_timeout, -1);
		set_times("intertrial_pause", (long)f_intertrial_time, -1);
		set_times("on_target_holdoff", (long)f_reward_holdoff, -1);
		set_times("on_target_rewon", (long)f_reward_preset, (long)f_reward_random);
		
		/* djs 2012-03-30 bonus reward pause */
		set_times("bonus_rewpause", (long)f_bonus_reward_pause, (long)f_bonus_reward_pause_random);
		if (f_ustim_rate > 0)
		{
			set_times("ustim_on", (long)f_ustim_duration_ms, -1);
		}
		if (f_verbose & DEBUG_REWARD_BIT)
		{
			dprintf("setting state times: \n");
			dprintf("fixpt_acq: %d\n", f_acq_time);
			dprintf("fixpt_acq_fail_pause: %d\n", f_acq_fail_timeout);
			dprintf("fixpt_hold: %d\n", f_fix_time);
			dprintf("intertrial_pause: %d\n", f_intertrial_time);
			dprintf("on_target: %d\n", f_reward_holdoff);
		}


		/* 
		 * These initializations need not be done when replay is in effect.
		 */	

		if (!f_use_replay_file)
		{
			/* 
			 * Allocate memory for all actions needed this trial and 
			 * initialize counters et al.  
			 * If this func returns a nonzero value it failed -- abort. 
			 */
	
			if (generate_trial())
			{
				f_exp_abort = 1;
			}

			/*
			 * Initialize the queue that holds jitter action structs.
			 */
			 
			mbq_clear(f_pmbq_jitter_actions);
			for (i=0; i<MAX_JITTER_ACTIONS; i++)
			{
				mbq_push(f_pmbq_jitter_actions, f_jitter_actions+i);
			}
		}
		else
		{
			R3PTrialP ptrial;
			ptrial = r3p_get_trial(f_trial_condition_index % r3p_get_n_trials());
			if (!ptrial)
			{
				rxerr("Bad trial index!");
			}
			else
			{
				if (f_trial_condition_index < r3p_get_n_trials())
				{
					dprintf("Replay is ACTIVE.\n");
					bcode_int(CH_REPLAY_TIMESTAMP, ptrial->header.time);
				}
				else
				{
					dprintf("Replay is ACTIVE - with REFLECTION.\n");
					bcode_int(CH_REPLAY_TIMESTAMP, -1*ptrial->header.time);
				}
			}
		}
		
		/*
		 * Open the analog window. 
		 */

		awind(OPEN_W);
		
		/* 
		 * Open the eye window
		 */

		my_fixpt_window(1);
		
		/*
		 * Clear acq statistics counters 
		 */
		 
		fixation_statistics(FIXATION_STATISTICS_INIT);


		/* 
		 * If saving command data, push initial data into buffer
		 */
		 
		if (f_savecommands)
		{
			f_savecommandsbufferused = 0;
			save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &ttime, sizeof(ttime));
			save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_pfixpt_x[f_trial_condition_index], sizeof(f_pfixpt_x[f_trial_condition_index]));
			save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_pfixpt_y[f_trial_condition_index], sizeof(f_pfixpt_y[f_trial_condition_index]));
			save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_cam_height, sizeof(f_cam_height));
			save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_speed, sizeof(f_speed));
			save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, f_cam_position, 3*sizeof(float));
			save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_cam_trajectory, sizeof(f_cam_trajectory));
		}
	}
	else
	{
		f_all_done = 1;
	}
	
	return status;
}

int save_to_buffer(char *buffer, int *nused, void *data, int nbytes)
{
	memcpy(buffer+*nused, data, nbytes);
	*nused += nbytes;
	return *nused;
}






/*
 * my_trial_allon_center()
 * 
 * Action called during start sequence for a new trial. Here, the target is 
 * placed dead-ahead and the frame rendered. 
 * 
 * TODO - what to do on fixed_looking trials?
 */
 
int my_trial_allon_center()
{
	render_onoff(&f_gp_handle, HANDLE_ON, ONOFF_NO_FRAME);
	f_ttrans_dir = f_cam_trajectory;
	my_target_update(HANDLE_ON);
	if (f_verbose & DEBUG_BKGD_BIT)
	{
		render_bgcolor(55, 55, 55);
	}		
	
	if (f_verbose & DEBUG_FIXED_BIT)
	{
		dprintf("my_trial_allon_center: H: %d\tL: %d\tT: %d\n", (int)(100*f_cam_trajectory * 180.0f/M_PI), (int)(100*f_pfixed_looking_angle_deg[f_trial_condition_index]), (int)(100*f_ttrans_dir * 180.0/M_PI));
	}
	
	render_frame(0);
	return 0;
}

void my_target_update(int handle_onoff)
{
	f_ttrans.xoffset = -sin(f_ttrans_dir);
	f_ttrans.zoffset = cos(f_ttrans_dir);
	render_update(f_ttrans_handle, (void *)&f_ttrans, sizeof(TargetTransformStruct), handle_onoff);
	return;
}

/*
 * random_targ_offset_degrees
 *
 * Select a target offset from the current list of available offsets. The sign of the 
 * offset is also randomized (i.e. the sign of the offset in the list is irrelevant 
 * because a random +/-1 is multiplied here). 
 *
 * Modified: 11-14-08 djs Now return a floating point value. 
 * Returns: jump magnitude and sign, as a float. 
 *
 */

float random_targ_offset_degrees()
{
	int irand;
	int i;
	int iwsum = 0;
	int isgn;
	
	irand = nrand_intv(&f_seed, 0, f_targ_offset_weight_sum-1);
	isgn = nrand_sign(&f_seed); 

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

	return f_targ_offset_degrees[i] * isgn;
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
	dprintf("my_trial_allon_offset\n");	
	if (f_use_replay_file == 0)
	{
		if (!f_fixed_looking)
		{
			f_ttrans_dir = f_cam_trajectory + random_targ_offset_degrees() * M_PI/180.0f;
		}
		else
		{
			f_ttrans_dir = 0;
			if (f_verbose & DEBUG_FIXED_BIT)
			{
				dprintf("my_trial_allon_offset: H: %d\tL: %d\tT: %d\n", (int)(100*f_cam_trajectory * 180.0f/M_PI), (int)(100*f_pfixed_looking_angle_deg[f_trial_condition_index]), (int)(100*f_ttrans_dir * 180.0/M_PI));
			}
		}			
	}
	else
	{
		int itrial = f_trial_condition_index;
		R3PTrialP ptrial = r3p_get_trial(f_trial_condition_index % r3p_get_n_trials());
		f_ttrans_dir = ptrial->header.ttrans_dir_offset;
		if (f_trial_condition_index >= r3p_get_n_trials())
		{
			f_ttrans_dir = 2*ptrial->header.cam_trajectory - ptrial->header.ttrans_dir_offset;
		}
	}
	my_target_update(HANDLE_ON);
	render_frame(0);
	
	if (f_savecommands)
	{
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_ttrans_dir, sizeof(f_ttrans_dir));
	}
	return 0;
}

/*
 * my_trial_done()
 * 
 * Action called after a trial completes. Closes analog window and sets flag
 * for reward loop, indicating that there's no steering so stop the reward
 * business. Do cleanup of stuff allocated at the beginning of the trial. 
 * 
 * MODIFIED 1-5-2012
 * On successful completion of a trial (i==1) give a reward. The reward size is 
 * dependent on the steering error at the end of the trial. 
 */

int my_trial_done(int i)
{
	if (i<0) dprintf("Trial aborted by PSTOP.\n", i);
	else if (i==0) dprintf("Trial failed.\n", i);
	else 
	{
		double dot;
		double e;
		int rew = 0;
		dprintf("Trial success!\n");
		/* Determine size of bonus reward */
		dot = cos(f_cam_trajectory)*cos(f_ttrans_dir) + sin(f_cam_trajectory)*sin(f_ttrans_dir);
		e = acos(dot) * 180.0/M_PI;		/* (0, pi) */
		if (e < f_bonus_reward_cutoff_degrees)
		{
			rew = f_bonus_reward_max - f_bonus_reward_max * (e * e) / (f_bonus_reward_cutoff_degrees * f_bonus_reward_cutoff_degrees); 
			set_times("bonus_rewon", (long)rew, (long)f_bonus_reward_random);
			f_bonus_reward = 1;
			dprintf("End of trial error=%d/100; max=%d/100, rew = %d\n", (int)(e*100), (int)(f_bonus_reward_cutoff_degrees*100), rew);
		}
		else 
		{
			dprintf("End of trial error = %d/100; max=%d/100; no bonus reward.\n", (int)(e*100), (int)(f_bonus_reward_cutoff_degrees*100));
		}
	}

	if (i>0)
	{
		f_prtgen->mark(f_prtgen, f_trial_condition_index);
	}
	awind(CLOSE_W);
	f_going = 0;	/* this tells reward loop to stop rewarding */
	my_fixpt_window(0);

	/* Show status of random trial counts. */
	show_trials_status();

	/* If saving commands, clear buffer on pause/break, dump buffer on success. */
	if (f_savecommands)
	{
		FILE *fp;
		fp = fopen(f_savecommandsfile, "ab");
		if (!fp)
		{
			rxerr("Cannot open save commands file");
			rxerr(f_savecommandsfile);
		}
		else
		{
			/* Write buffer size then buffer itself. */
			f_savecommandscount++;
			fwrite(&f_savecommandsbufferused, sizeof(f_savecommandsbufferused), 1, fp);
			fwrite(f_savecommandsbuffer, sizeof(char), f_savecommandsbufferused, fp);
			dprintf("Saved %d bytes to file\n", f_savecommandsbufferused);
			f_savecommandsbufferused = 0;
			fclose(fp);
		}
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

	if (f_gp_handle || f_ttrans_handle || f_fixpt_handle)
	{
		alloff();
	}
	
	if (f_verbose & (DEBUG_BKGD_BIT|DEBUG_BKGD2_BIT))
	{
		render_bgcolor(0, 0, 0);
	}		
	
	render_frame(0);
	return 0;
}

void show_trials_status()
{
	int i, j;
	char snum[6];
	dprintf("===================Trial counts=================\n");
	dprintf("ind\ttally\n");
	dprintf("---\t-------------------------------\n");
	for (i=0; i<f_ntrialtypes; i++)
	{
		/*
		 * Put number in ascii format (xxx)
		 */
		if (f_prtgen->count(f_prtgen, i)<10) sprintf(snum, "(  %d)", f_prtgen->count(f_prtgen, i));
		else if (f_prtgen->count(f_prtgen, i)<100) sprintf(snum, "( %d)", f_prtgen->count(f_prtgen, i));
		else if (f_prtgen->count(f_prtgen, i)<1000) sprintf(snum, "(%d)", f_prtgen->count(f_prtgen, i));
		else sprintf(snum, "(***)", f_prtgen->count(f_prtgen, i));
		
		dprintf("%d\t%s\t", i, snum);
		for (j=0; j<f_prtgen->count(f_prtgen, i); j++) dprintf("X");
		dprintf("\n");
	}
	dprintf("\n");  
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
	render_onoff(&f_gp_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_ttrans_handle, HANDLE_OFF, ONOFF_NO_FRAME);
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
	float blip_delta=0;
	char s[128];
	float cam_trajectory_with_jitter = 0.0;
	float meander_target_delta = 0.0;
	float diffusion_target_delta = 0.0;

	/* We are always going when in the update loop. */
	f_going = 1;
	
	/* Get joystick value and calculate new camera trajectory. */	

	ijoyh = joystick_value();		// joystick_value will return fakejoy value if fakejoy is enabled
	ival = ijoyh - f_steer_zero;		// subtract off zero point


	/* Check for spinning. */
	if (f_pslider)
	{
		f_pslider->wadd(f_pslider, ijoyh);	// used for spin detection
		if (f_pslider->isFull(f_pslider) && 
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
					int code = JLEFTCD;
					if (f_current_actions[i]->fval > 0) code = JRIGHTCD;
					if (ecode(code)) status = BADCD;
					bcode_float(CH_JUMP_SIZE, f_current_actions[i]->fval);
					bcode_float(CH_JUMP_ACTUAL, f_ttrans_dir*180.0f/M_PI - f_current_actions[i]->fval);
					f_ttrans_dir = f_cam_trajectory + f_current_actions[i]->fval * M_PI / 180.0f;
					targ_update_flag = 1;
					f_jumpflag = 1;
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
				f_ttrans_dir += angle;
				f_cam_trajectory += angle;
				targ_update_flag = 1;
				break;
			}
			case ACTIONTYPE_BLINK:
			{
				if ((f_went_counter+1) == f_current_actions[i]->start_frame)
				{
					render_onoff(&f_ttrans_handle, HANDLE_OFF, ONOFF_NO_FRAME);
					ecode(BLINKBEGINCD);
				}
				else if ((f_went_counter+1) == f_current_actions[i]->end_frame)
				{
					ecode(BLINKENDCD);
					render_onoff(&f_ttrans_handle, HANDLE_ON, ONOFF_NO_FRAME);
				}
				break;
			}
			case ACTIONTYPE_USTIM:
			{
				/*
				 * We need only set the ready flag for ustim. The ustim is turned off 
				 * after its allotted time, not by the action handling here. 
				 */
				if ((f_went_counter+1) == f_current_actions[i]->start_frame)
				{
					f_ustim_ready_flag = 1;
				}
				break;
			}
			case ACTIONTYPE_TRIALEND:
			{
				f_trial_end_flag = 1;
				break;
			}
			case ACTIONTYPE_JITTERTRAJ:
			{
				if ((f_went_counter+1) == f_current_actions[i]->start_frame)
				{
					f_jitter_trajectory_delta += f_current_actions[i]->fval;
				}
				else if ((f_went_counter+1) == f_current_actions[i]->end_frame)
				{
					f_jitter_trajectory_delta -= f_current_actions[i]->fval;
				}
				break;
			}				
			case ACTIONTYPE_JITTERTARG:
			{
				if ((f_went_counter+1) == f_current_actions[i]->start_frame)
				{
					f_jitter_target_delta += f_current_actions[i]->fval;
					f_ttrans_dir += f_current_actions[i]->fval;
					targ_update_flag = 1;
				}
				else if ((f_went_counter+1) == f_current_actions[i]->end_frame)
				{
					f_jitter_target_delta -= f_current_actions[i]->fval;
					f_ttrans_dir -= f_current_actions[i]->fval;
					targ_update_flag = 1;
				}
				break;
			}				
			case ACTIONTYPE_MEANDERTARG:
			{
				if ((f_went_counter+1) <= f_current_actions[i]->end_frame)
				{
					meander_target_delta += f_current_actions[i]->fval;
					f_ttrans_dir += f_current_actions[i]->fval;
					targ_update_flag = 1;
				}
				break;
			}				
			case ACTIONTYPE_DIFFUSIONTARG:
			{
				if ((f_went_counter+1) <= f_current_actions[i]->end_frame)
				{
					diffusion_target_delta += f_current_actions[i]->fval;
					f_ttrans_dir += f_current_actions[i]->fval;
					targ_update_flag = 1;
				}
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



	/* 
	 * Update camera position. 
	 * The camera trajectory now includes any change due to the joystick response. 
	 * We will add the jitter delta to it and then compute the new camera position. 
	 */
	 
	cam_trajectory_with_jitter = f_cam_trajectory + f_jitter_trajectory_delta;
	
	
	/*
	 * DJS 2-25-15
	 * looking direction can be 
	 * 1. same as current trajectory - i.e. always looking in the same direction 
	 *    that the car is going
	 * 2. a fixed direction. This is the case when f_fixed_looking == 1
	 * 
	 *  int f_fixed_looking = 0;
	 *  float f_camera_looking_angle;
	 * 	f_cam_looking[0] = -sin(f_camera_looking_angle);
	 *  f_cam_looking[1] = 0;
	 * 	f_cam_looking[2] = cos(f_camera_looking_angle);
	 * 
	 * 
	 */

	if (!f_fixed_looking)
	{		
		f_cam_looking[0] = -sin(f_cam_trajectory);
		f_cam_looking[1] = 0;
		f_cam_looking[2] = cos(f_cam_trajectory);
	}
	
	f_cam_position[0] += -sin(cam_trajectory_with_jitter) * f_speed;
	f_cam_position[1] = (float)f_cam_height;
	f_cam_position[2] += cos(cam_trajectory_with_jitter) * f_speed;

	/* Update target if necessary */
	if (targ_update_flag)
	{
		my_target_update(0);
	}

/*
	if (f_verbose & DEBUG_FIXED_BIT)
	{
		dprintf("my_update: H: %d\tL: %d\tT: %d\n", (int)(100*f_cam_trajectory * 180.0f/M_PI), (int)(100*f_pfixed_looking_angle_deg[f_trial_condition_index] * 180.0/M_PI), (int)(100*f_ttrans_dir * 180.0/M_PI));
	}
*/
	render_camera(f_cam_position, f_cam_looking, f_cam_up);

	/* 
	 * Change background to white when testing timing 
	 */	
	if (f_verbose & DEBUG_BKGD2_BIT)
	{
		render_bgcolor(55, 55, 55);
	}		
	

	/* Now render */			
	render_frame(0);

	/* ecodes */
	/* 9-13-07 target direction code - use actual targ_dir, not offset! Also, convert to degrees and mult * 10 */
	ecode(UPDCD);
	bcode_uint(CH_JOY, ijoyh);
	bcode_float(CH_TRAJ, f_cam_trajectory * 180.0f/M_PI);
	bcode_float(CH_TARG, f_ttrans_dir * 180.0f/M_PI);
	bcode_float(CH_TRAJ_JITTER, f_jitter_trajectory_delta * 180.0f/M_PI);
	bcode_float(CH_TARG_JITTER, f_jitter_target_delta * 180.0f/M_PI);
	bcode_float(CH_TARG_MEANDER, meander_target_delta * 180.0f/M_PI);
	bcode_float(CH_TARG_DIFFUSION, diffusion_target_delta * 180.0f/M_PI);
	if (f_jitter_record_campos)
	{
		bcode_float(CH_CAMPOS_X, f_cam_position[0]);
		bcode_float(CH_CAMPOS_Z, f_cam_position[2]);
	}


	if (f_verbose & DEBUG_JOY_BIT)
	{
		dprintf("%d\t%d\t%d\n", ijoyh, (int)(f_cam_position[0]*100), (int)(f_cam_position[2]*100));
	}

	
	
	if (f_savecommands)
	{
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &ijoyh, sizeof(ijoyh));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_cam_trajectory, sizeof(f_cam_trajectory));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_ttrans_dir, sizeof(f_ttrans_dir));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_jitter_trajectory_delta, sizeof(f_jitter_trajectory_delta));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_jitter_target_delta, sizeof(f_jitter_target_delta));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &meander_target_delta, sizeof(meander_target_delta));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &diffusion_target_delta, sizeof(diffusion_target_delta));
	}	
	
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
		dprintf("ERROR in my_check_for_went: status=%d\n", status);
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
		if (f_jumpflag)
		{
			f_went_counter_last_jump = f_went_counter;
			f_jumpflag = 0;
		}

		/* 
		 * If the ustim ready flag was set then we should set the ustim flag.
		 * We do this ready_flag/flag business so that we can align the onset of ustim 
		 * period to a went, just like blips and blinks. Note that the end of the 
		 * ustim period is handled in the ustim loop, and as such it may not be 
		 * aligned with a went. DJS
		 */		

		if (f_ustim_ready_flag)
		{
			f_ustim_flag = 1;
			f_ustim_ready_flag = 0;
		}
			
		ecode(WENTCD);
				
		// TEST - if frames were missed, dprint it and the cycles...
		if (frames > 1 || f_went_cycles > 13)
		{
			dprintf("Missed %d frames (%d check_went cycles)\n", frames, f_went_cycles);
			if (frames > 1) ecode(MISSEDCD);
		}
		f_went_cycles = 0;

		if (f_trial_end_flag)
		{
			f_wstatus = 2;
			ecode(TRIALENDCD);
		}
		else if (f_fixed_looking)
		{
			/* 
			 * For fixed looking trials, test cam trajectory and looking angles....
			 */
	 
			if (cos(f_cam_trajectory - f_pfixed_looking_angle_deg[f_trial_condition_index] * M_PI/180.0f) < f_cos_fovx_over_2)
			{
				dprintf("WIDE H: %d\tL: %d\tT: %d\n", (int)(100*f_cam_trajectory * 180.0f/M_PI), (int)(100*f_pfixed_looking_angle_deg[f_trial_condition_index] * 180.0/M_PI), (int)(100*f_ttrans_dir * 180.0/M_PI));
				f_wstatus = 3;
			}		
		}
			
	}
	return f_wstatus;
}


/*
 * my_replay()
 * 
 * This function takes the place of my_update when replay trials are being used. 
 * 
 * Every call to my_update (only when NOT doing a replay session) generates a set of update
 * data written to the replay file. If you look carefully at the state logic, you will see that
 * there is a single call to my_update in the state "flow", where render_check_for_went is used
 * as the escape function. render_check_for_went does NOT increment the went counter! The next 
 * call to my_update (and all subsequent calls for this trial) are made from state update, 
 * where the escape is my_check_for_went, which DOES increment the went counter. So, when the 
 * first TWO sets of data are written to the replay file, the went counter is 0! Thus, we cannot
 * use f_went_counter here as the index into the went data for the trial, because if we do we
 * will present the very first frame twice (because this function will be called twice
 * with f_went_counter = 0). 
 * 
 */
 
int my_replay()
{
	int status = 0;
	float cam_trajectory_with_jitter;
	R3PTrialP ptrial;
	int ijoyh;
	float meander_target_delta = 0.0;
	float diffusion_target_delta = 0.0;

	/* We are always going when in the update loop. */
	f_going = 1;

	/*
	 * Get updated parameters from replay trial. The counter f_went_counter is 
	 * used to determine the particular frame values to set here. 
	 */
	 
	ptrial = r3p_get_trial(f_trial_condition_index % r3p_get_n_trials());
	if (!ptrial)
	{
		rxerr("Bad trial index!");
	}
	else
	{
		/* 
		 * Each went requires a camera update, a ttrans update and a FRAME
		 */
		ijoyh = ptrial->pwent[f_replay_counter].ijoyh;
		f_cam_trajectory = ptrial->pwent[f_replay_counter].cam_trajectory;
		f_ttrans_dir = ptrial->pwent[f_replay_counter].ttrans_dir;
		f_jitter_trajectory_delta = ptrial->pwent[f_replay_counter].jitter_trajectory_delta;
		f_jitter_target_delta = ptrial->pwent[f_replay_counter].jitter_target_delta;
		meander_target_delta = ptrial->pwent[f_replay_counter].meander_target_delta;
		diffusion_target_delta = ptrial->pwent[f_replay_counter].diffusion_target_delta;		
		
		/* 
		 * When the trial is a reflection trial, reflect camera trajectory, ttrans_dir through 
		 * the line where the original cam trajectory was.
		 */
		 
		if (f_trial_condition_index >= r3p_get_n_trials())
		{
			f_cam_trajectory = 2*ptrial->header.cam_trajectory - ptrial->pwent[f_replay_counter].cam_trajectory; 
			f_ttrans_dir = 2*ptrial->header.cam_trajectory - ptrial->pwent[f_replay_counter].ttrans_dir;
			f_jitter_trajectory_delta *= -1;
			f_jitter_target_delta *= -1;
			meander_target_delta *= -1;
			diffusion_target_delta *= -1;
		}
	}		
	
	
	/* 
	 * Set trial end flag if needed 
	 */
	f_replay_counter++;
	if (f_replay_counter == ptrial->nwent)
	{
		f_trial_end_flag = 1;
	}

	
	/* 
	 * Update camera position. 
	 */

	cam_trajectory_with_jitter = f_cam_trajectory + f_jitter_trajectory_delta;

	f_cam_looking[0] = -sin(f_cam_trajectory);
	f_cam_looking[1] = 0;
	f_cam_looking[2] = cos(f_cam_trajectory);

	f_cam_position[0] += -sin(cam_trajectory_with_jitter) * f_speed;
	f_cam_position[1] = (float)f_cam_height;
	f_cam_position[2] += cos(cam_trajectory_with_jitter) * f_speed;

	my_target_update(0);

	// TODO - DJS must accomodate f_fixed_looking==1 case

	render_camera(f_cam_position, f_cam_looking, f_cam_up);

	/* Now render */			
	render_frame(0);

	/* ecodes */
	/* 9-13-07 target direction code - use actual targ_dir, not offset! Also, convert to degrees and mult * 10 */
	ecode(UPDCD);
	bcode_uint(CH_JOY, ijoyh);
	bcode_float(CH_TRAJ, f_cam_trajectory * 180.0f/M_PI);
	bcode_float(CH_TARG, f_ttrans_dir * 180.0f/M_PI);
	bcode_float(CH_TRAJ_JITTER, f_jitter_trajectory_delta * 180.0f/M_PI);
	bcode_float(CH_TARG_JITTER, f_jitter_target_delta * 180.0f/M_PI);
	bcode_float(CH_TARG_MEANDER, meander_target_delta * 180.0f/M_PI);
	bcode_float(CH_TARG_DIFFUSION, diffusion_target_delta * 180.0f/M_PI);
	if (f_jitter_record_campos)
	{
		bcode_float(CH_CAMPOS_X, f_cam_position[0]);
		bcode_float(CH_CAMPOS_Z, f_cam_position[2]);
	}
	
	
	if (f_savecommands)
	{
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &ijoyh, sizeof(ijoyh));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_cam_trajectory, sizeof(f_cam_trajectory));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_ttrans_dir, sizeof(f_ttrans_dir));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_jitter_trajectory_delta, sizeof(f_jitter_trajectory_delta));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &f_jitter_target_delta, sizeof(f_jitter_target_delta));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &meander_target_delta, sizeof(meander_target_delta));
		save_to_buffer(f_savecommandsbuffer, &f_savecommandsbufferused, &diffusion_target_delta, sizeof(diffusion_target_delta));
	}	
	return status;	
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
 * my_exp_abort()
 * 
 * Called from state exp_abort - where we end up when some bad config thing happens
 * in my_exp_init.
 */

int my_exp_abort()
{
	dprintf("Expt aborted. Check config and Controls>Process Controls>Print Debug\n");
	return 0;
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

			value = -1*(f_cam_trajectory-f_ttrans_dir)/(C*K*f_steer_max_degrees)*(1024-f_steer_dead) + f_steer_dead + f_steer_zero;
			if (value < 0) value = 0;
			if (value > 2047) value=2047;
		}
	}
	return value;
}


int fixation()
{
	int i;
	fixation_statistics(FIXATION_STATISTICS_DUMP);
	i = INCREMENT(AllTrials);
	return 0;
}


int got_fixation()
{
	f_fix_initial = getClockTime();
	f_fix_next_reward = f_fix_initial + f_reward_wait_max;
	return 0;
}


int fix_rewon()
{
	dio_on(REW);
	return 0;
}

int fix_rewoff()
{
	float f;
	dio_off(REW);
	f = (float)(getClockTime() - f_fix_initial)/(float)f_reward_ramp_time;
	if (f>1) f=1;
	f_fix_next_reward = getClockTime() + (1-f)*f_reward_wait_max + f*f_reward_wait_min; 
	return 0;
}

int check_fixation_reward()
{
	if (getClockTime() > f_fix_next_reward)
		return 1;
	else
		return 0;
}


/* 
 * fixation_statistics()
 * 
 * State action used to tally (and debug) escapes during fixpt acquisition. 
 * Set f_verbose bit 4 DEBUG_ACQ_BIT to get debugging dprintf's. 
 * Usage:
 * fixation_statistics(FIXATION_STATISTICS_INIT) - zero counters (called in my_trial_init())
 * fixation_statistics(FIXATION_STATISTICS_FAIL) - tally failure by timeout
 * fixation_statistics(FIXATION_STATISTICS_NOISE) - tally failure by noise
 * fixation_statistics(FIXATION_STATISTICS_DUMP) - dprintf tallies only if (f_verbose & DEBUG_ACQ_BIT)
 * 
 */
 

int fixation_statistics(int itype)
{
	switch(itype)
	{
		case FIXATION_STATISTICS_INIT:
			f_fixation_statistics[FIXATION_STATISTICS_FAIL] = 0; 
			f_fixation_statistics[FIXATION_STATISTICS_NOISE] = 0; 
			break;
		case FIXATION_STATISTICS_DUMP:
			if (f_verbose & DEBUG_ACQ_BIT)
			{
				dprintf("Fixation achieved. Failures: timeout=%d noise=%d\n", f_fixation_statistics[FIXATION_STATISTICS_FAIL], f_fixation_statistics[FIXATION_STATISTICS_NOISE]);
			}
			break;
		case FIXATION_STATISTICS_FAIL:
		case FIXATION_STATISTICS_NOISE:
			f_fixation_statistics[itype]++;
			break;
		default:
			if (f_verbose & DEBUG_ACQ_BIT)
			{
				dprintf("fixation_statistics(): ERROR unknown itype %d\n", itype);
			}
			break;
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
	f_last_check_steering_went_counter = 0;
	f_last_check_steering_value = -1;
	f_next_reward_time = 0;
	f_last_reward_time = INT_MIN;
	f_reward_on_target_sum = 0;
	f_grace_period_expire = INT_MAX;
	ecode(NOTSPINNINGCD);
	f_spinning = 0;
	if (f_pslider) f_pslider->clear(f_pslider);
	
	/*
	 * these are used for replay trial only. Init them anyways
	 */
	 
	f_fix_initial = -1;
	f_fix_next_reward = 999999999;
	
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("reward_init\n");
	return 0;
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
		return f_last_check_steering_value;
	}
	else
	{
		dot = cos(f_cam_trajectory)*cos(f_ttrans_dir) + sin(f_cam_trajectory)*sin(f_ttrans_dir);
		if (dot > cos(f_reward_window_degrees * M_PI/180))
		{
			status = 1;
			f_reward_spatial_fraction = (dot - f_cos_reward_window)/(1.0f - f_cos_reward_window);
		}
		else
		{
			/* Was there a jump? */
			status = -1;
			if (f_went_counter == f_went_counter_last_jump)
			{
				status = 0;
			}
		}
	}
	f_last_check_steering_value = status;
	f_last_check_steering_went_counter = f_went_counter;
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
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("get_next_reward_time\tnow=%d\twait=%d\tnext=%d\n", getClockTime(), wait, f_next_reward_time);
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
 * on_target()
 * 
 * Action for on_target state. 
 */

int on_target()
{
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("on_target\tnow=%d\n", getClockTime());
	return 0;
}


/*
 * check_steering_and_wait_time()
 * 
 * Test if steering is in reward window. If it is not, return 0. If it is, then
 * check if the reward wait time is passed. If it has, then return 1 (allowing
 * another reward), otherwise return 0. 
 * 
 * Uses f_next_reward_time, which was computed in get_reward_wait_time() 
 */

int check_steering_and_wait_time()
{
	int status = check_steering();
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
	return status;
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
	f_reward_on_time = getClockTime();
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("reward_on\tnow=%d\n", getClockTime());
	return 0;
}

int reward_off()
{
	dio_off(REW);
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("reward_off\tnow=%d reward_time=%d\n", getClockTime(), getClockTime() - f_reward_on_time);
	return 0;
}

int bonus_reward_on()
{
	dio_on(REW);
	f_bonus_reward = 0;
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("bonus_reward_on\tnow=%d\n", getClockTime());
	return 0;
}

int bonus_reward_off()
{
	dio_off(REW);
	if (f_verbose & DEBUG_REWARD_BIT) dprintf("bonus_reward_off\tnow=%d\n", getClockTime());
	return 0;
}

int my_fixpt_window(int on)
{
	int status = 0;
	if (on)
	{
		wd_src_pos(WIND0, WD_DIRPOS, 0, WD_DIRPOS, 0);
		wd_pos(WIND0, f_pwdpos_x[f_trial_condition_index], f_pwdpos_y[f_trial_condition_index]);
		wd_siz(WIND0, (long)(f_fixpt_window*10.0), (long)(f_fixpt_window*10.0));
		wd_cntrl(WIND0, WD_ON);
		wd_src_check(WIND0, WD_SIGNAL, 0, WD_SIGNAL, 1);
	}
	else
	{
		wd_cntrl(WIND0, WD_OFF);
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



/* REX state set starts here 
 * djs 11-5-08
 * Modified id from 402->403
 * djs 11-6-08
 * Modified id from 403->404
 * djs 11-14-08 st17, 404->405
 * djs 01-23-09 st18af 405->406
 * djs 04-23-09 st20af 406->407
 * djs 08-05-09 st20af 407->408 Added wdpos to bcodes
 * djs 12-03-09 st21af 408->409
 * djs 1-21-10  st22af 409->410
 * djs 7-26-11  st23af 410->411
 * djs 2-27-14  st23af 411->412
 * 
 */

%%

id 412
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
		/*to exp_init on 1 % my_check_for_handle*/
		to target_init on 1 % my_check_for_handle
	target_init:
		do my_target_init();
		to exp_init on 1 % my_check_for_target_handle
	exp_init:
		do my_exp_init()	/* initialize trial counters */
		to exp_abort on 1 = f_exp_abort
		to trial_init on 1 % render_check_for_went
	trial_init:
		do my_trial_init()	/* initialization for a single trial */
		to exp_abort on 1 = f_exp_abort
		to all_done on 1 = f_all_done
		to trial_init_pause
	trial_init_pause:
		time 500			/* This time will be updated based on menu entry - see my_trial_init */
		to pause_detected on +PSTOP & softswitch
		to fixpt_on
	fixpt_on:
		do fixpt_onoff(1);
		/*do render_onoff(&f_fixpt_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT);*/
		to fixpt_acq on 1 % render_check_for_went
	fixpt_acq:
		time 4000
		to pause_detected on +PSTOP & softswitch
		to fixpt_hold on -WD0_XY & eyeflag
		to fixpt_acq_fail
	fixpt_acq_fail:
		do fixpt_onoff(0)
		/*do render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);*/
		to fixpt_acq_fail_pause on 1 % render_check_for_went
	fixpt_acq_fail_pause:
		time 500			/* f_acq_fail_pause_time */
		do fixation_statistics(FIXATION_STATISTICS_FAIL)
		to fixpt_on
	fixpt_hold:
		time 150			/* f_fix_time */
		to fixpt_acq_noise on +WD0_XY & eyeflag
		to fixation
	fixpt_acq_noise:
		do fixpt_onoff(0)
		/*do render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);*/
		to fixpt_acq_noise_pause on 1 % render_check_for_went
	fixpt_acq_noise_pause:
		time 500			/* f_acq_noise_pause_time */
		do fixation_statistics(FIXATION_STATISTICS_NOISE)
		to fixpt_on
	fixation:
		code FIXCD
		do fixation()
		to trial_allon_center
	trial_allon_center:
		do my_trial_allon_center()
		to trial_allon_wait on 1 % render_check_for_went
	trial_allon_wait:
		code STIMON
		time 500			/* f_trial_allon_wait_time */
		to fixation_fail on +WD0_XY & eyeflag
		to trial_beep_on on 1 = f_use_trial_beep
		to trial_allon_offset
		
	/* 
	 * The beep only used if you set f_use_trial_beep = 1. 
	 * On the training rig the beep doesn't work, or if it does it affects
	 * the juicer, so we don't use it. 
	 */
	 
	trial_beep_on:
		dio_on(BEEP)
		time 100
		to trial_beep_off
	trial_beep_off:
		dio_off(BEEP)
		to trial_allon_offset
	trial_allon_offset:
		do my_trial_allon_offset()
		to flow_or_replay on 1 % render_check_for_went
	flow_or_replay:
		to flow on 0 = f_use_replay_file
		to reflow
	flow:
		code FLOWCD
		do my_update()
		to update on 1 % render_check_for_went
	update:
		do my_update()
		to pause_detected_wait on +PSTOP & softswitch
		to update_wait
	update_wait:
		to fixation_fail_wait on +WD0_XY & eyeflag
		to update on 1 % my_check_for_went
		to trial_success on 2 = f_wstatus
		to fixlook_trial_fail on 3 = f_wstatus
	reflow:
		code FLOWCD
		do my_replay()
		to replay on 1 % render_check_for_went
	replay:
		do my_replay()
		to pause_detected_wait on +PSTOP & softswitch
		to replay_wait
	replay_wait:
		to fixation_fail_wait on +WD0_XY & eyeflag
		to replay on 1 % my_check_for_went
		to trial_success on 2 = f_wstatus
		to fixlook_trial_fail on 3 = f_wstatus
		
		
		
	trial_success:
		do my_trial_done(1)
		to intertrial_pause on 1 % render_check_for_went
	fixlook_trial_fail:
		code BADSTEERCD
		do my_trial_done(0)
		to fixation_fail_pause on 1 % render_check_for_went		
	fixation_fail_wait:
		to fixation_fail on 1 % render_check_for_went
	fixation_fail:
		code BREAKFIXCD
		do my_trial_done(0)
		to fixation_fail_pause on 1 % render_check_for_went
	fixation_fail_pause:
		time 1000			/* f_fixation_fail_timeout */
		to trial_init
	intertrial_pause:
		time 500
		to trial_init
	pause_detected_wait:
		to pause_detected on 1 % render_check_for_went
	pause_detected:
		code PAUSECD
		do my_trial_done(-1)
		to pause_wait on 1 % render_check_for_went
	pause_wait:
		to trial_init on -PSTOP & softswitch
	all_done:
		do all_trials_done()
		to first on 1 = f_never
	exp_abort:
		do my_exp_abort()
		to first on 1 = f_never
/*abort	list:
		fixation_fail*/
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
		to fix_wait on 1 = f_use_replay_file
		to off_target
		
		
	/* 
	 * Take f_fixation==1 to mean fixation is achieved. 
	 * Must set it in fixation(). Where is it set to 0?
	 */
	fix_wait:
		to fix_stopped on 0 = f_going
		to fix_paused on +PSTOP & softswitch
		to fix_got on -WD0_XY & eyeflag
	/*
	 * got_fixation initializes vars governing the fixation reward ramp
	 * f_fix_initial = now
	 * f_fix_next_reward = now+max
	 */
	fix_got:
		do got_fixation()
		to fix_maintain
		
	/*
	 * check_fixation_reward
	 *   if now > f_fix_next_reward return 1
	 *   else return 0
	 */
	 
	fix_maintain:
		to fix_stopped on 0 = f_going
		to fix_paused on +PSTOP & softswitch
		to fix_broken on +WD0_XY & eyeflag
		to fix_rewon on 1 % check_fixation_reward
	fix_paused:
		to reward_init on -PSTOP & softswitch	
	fix_stopped:
		to reward_init on 1 = f_going
	fix_broken:
		to reward_init
	fix_rewon:
		code REWCD
		do fix_rewon()
		time 5
		rand 15
		to fix_rewoff

	/*
	 * fix_rewoff should compute next reward time
	 */
	fix_rewoff:
		do fix_rewoff()
		to fix_maintain
		
		
	off_target:
		do off_target()
		to stopped on 0 = f_going
		to paused on +PSTOP & softswitch
		to spinning on 1 = f_spinning
		/* djs 2-20-09 to on_target_rewon on 2 % check_steering_and_wait_time*/
		to on_target_holdoff on 2 % check_steering_and_wait_time
	on_target_holdoff:
		do on_target()
		time 1000
		to on_target_recheck
	on_target_recheck:
		to on_target_rewon on 1 % check_steering
		to off_target on 0 % check_steering
		to off_target on -1 % check_steering
	spinning:
		to paused on +PSTOP & softswitch
		to stopped on 0 = f_going
		to off_target on 0 = f_spinning
	paused:
		do reward_init()
		to off_target on -PSTOP & softswitch
	stopped:
		do reward_init()
		to bonus_rewpause on 1 = f_bonus_reward
		to off_target on 1 = f_going
	bonus_rewpause:
		time 5
		rand 15
		to bonus_rewon
	bonus_rewon:
		code REWCD
		do bonus_reward_on()
		time 5
		rand 15
		to bonus_rewoff
	bonus_rewoff:
		do bonus_reward_off()
		to stopped
	on_target_rewon:
		code REWCD
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
		to on_target_rewon on 1 % check_steering
		to off_target on 1 % check_grace_time
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

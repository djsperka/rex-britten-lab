/* $Id: bighead.d,v 1.23 2015/09/17 21:12:11 devel Exp $ */

#include <math.h>
#include "bcode.h"
#include "ldev.h"	/* hardware device defines */
#include "lcode.h"	/* Ecodes */
#include "lcalib.h"	/* local (per-rig) screen calibration */
#include "slider.h"
#include "sac.h"

#include "make_msg.h"			/* funcs to form command messages */
#include "transport.h"			/* funcs to send command messages to render */
#include "actions.h"			/* higher-level actions for render */
#include "pixels.h"				/* pixel conversion funcs */
#include "ivunif.h"				/* random number routines */
#include "randomtrialgenerator.h"
#include "animhelper.h"
#include "bh_animhelper.h"
 
 
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
#define DOTS_ON     1560
#define TRANS_START 1561
#define PURSUIT_PAUSE 1562
#define PURSUIT_PRE_START 1563
#define PURSUIT_START 1564
#define PURSUIT_BLINK_START 1565
#define PURSUIT_BLINK_END 1566
#define EVL_WAIT 1570
#define EVL_FILL 1571
#define EVL_SACCADE 1572
#define EVL_DELAY 1573

// For evloop below
#define MY_SF_GOOD_OR_ONSET (SF_GOOD|SF_ONSET) 

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
#define EYE_WINDOW_FIXPT_ON_UPDATE  0x6
#define EYE_WINDOW_FIXPT_OFF		0x8
#define EYE_WINDOW_TARGETS_UPDATE	0x20
#define EYE_WINDOW_TARGETS_ON		0x40
#define EYE_WINDOW_TARGETS_OFF		0x80
#define EYE_WINDOW_ALL_OFF			(EYE_WINDOW_FIXPT_OFF | EYE_WINDOW_TARGETS_OFF)


char f_charbuf[2048];			/* Really really big char for sprinting errors. */
int f_never = -1;				/* never var, quoth the raven */
int f_seed_used = 0;			/* indicates whether we've seeded the random number generator or not */
int f_fixpt_handle = 0;			/* graphics handle for fixation point */
int f_ospace_handle = 0;		/* graphics handle for outerspace */
int f_ptrans_handle = 0;
int f_handle_count = 0;			/* graphics handle counter */
int f_trial_condition_index;	/* index into f_pextcond array for current trial */
int f_all_done = 0;				/* flag indicating all trials have been completed */
int f_step_counter = 0;			/* step counter for animations */
int f_wstatus = 0;				/* status indicator used by my_check_for_went */
int f_went_cycles = 0;
int f_exp_abort = 0;
int f_pursuit_flag = 0;			/* Used with pursuit/saccade loop. When 1(0) pursuit is on (off) */
unsigned int f_last_frametag = 0;
int f_last_time = 0;
DotStruct f_fixpt;
CameraStruct f_camera;
OuterSpaceStruct f_outerspace;
int f_width;                    /* screen resolution */
int f_height;                   /* screen resolution */
double f_frames_per_second = 85;	/* frame rate for render */
int f_ms_per_frame;				/* see my_check_for_went */
int f_frames_pre_pursuit;		/* frames before pursuit begins */
int f_frames_pre_dots_motion;	/* frames after pursuit begins, before dot motion */
int f_frames_dots_motion;		/* frames of dot motion - rounded to even number */
int f_frames_dots_motion_midpoint;
int f_dots_motion_ready_flag;	/* flag set when first dot motion frame is imminent */
int f_animation_complete_flag = 0;	/* my_animate sets this when f_panim->step returns -1 to indicate there is no "next frame". */
RTGenerator *f_prtgen = NULL;
int f_trialsCompleted = 0;
int f_trialsFailed = 0;
int f_trialsInARow = 0;

/* 
 * Unique trial types are defined by the BPSHStruct (Bighead Pursuit Stabilization Heading).
 */
 
#define MAX_BPSHLIST 1000
#define ANG_DIFFERENCE_EPS 0.01
#define ANG_WITHIN_EPS(a, b) (fabs(a-b)<ANG_DIFFERENCE_EPS)
BPSHStruct *f_pbpshlist[MAX_BPSHLIST];
int f_nbpshlist = 0;
BPSHStruct *f_pbpsh = NULL;		// current trial

/*
 * Sliding windows for tracking velocity. 
 */
 
PSlider f_psliderXPos = (PSlider)NULL;
PSlider f_psliderYPos = (PSlider)NULL;
PSlider f_psliderTime = (PSlider)NULL;
int f_evTickCounter = 0;	// count ticks of clock between frames.

/* 
 * Function prototypes
 */

int my_eye_window();
int my_reward(int);
int my_animate();
int create_bpsh_struct(int ttype, int ptype, float az, float el, float hspeed, float p, float pspeed, int is_test, int ecode);
int check_heading_parameters();
int check_pursuit_parameters();
int generate_fixed_elevation_trials();
int generate_heading_trials();
int generate_pursuit_trials();
int generate_test_trials();


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
int f_trialtype=0;						/* type of trials to be run: 1=pref heading, 2=blink, 3=head+pursuit (need subtype) */
int f_trialsubtype = 0;					/* bits: 1=h+p, 2=h+simp 4=h+p+ret.stabilization, 8=p+ret.stabilization */
int f_ntrials = 50;						/* number of trials for each condition */
int f_nblocksize = 1;					/* block size that trials are gathered in */
int f_verbose = 1;						/* debugging flag */
int f_reward_preset = 60;				/* reward size preset value */
int f_reward_random = 10;				/* reward size random value */
int f_reward_max = 0;					/* max reward size when using ramp */
int f_reward_step_size = 0;				/* if > 0, rewards increase by this amount each successful trial to f_reward_max */

#define DEBUG_INFO_BIT 0x1
#define DEBUG_REWARD_BIT 0x2
#define DEBUG_FRAME_BIT 0x4

#define TRIALTYPE_PREFERRED_HEADING 1
#define TRIALTYPE_PREFERRED_PURSUIT 2
#define TRIALTYPE_TEST 3
#define TRIALTYPE_FIXED_ELEVATION 4
#define SUBTYPE_HEADING_PURSUIT 0x1
#define SUBTYPE_HEADING_SIMULATED_PURSUIT 0x2
#define SUBTYPE_HEADING_PURSUIT_RETSTAB 0x4
#define SUBTYPE_PURSUIT_RETSTAB 0x8

VLIST state_vl[] = {
"day_seed",		&f_seed, NP, NP, 0, ME_DEC,
"trial_type(1,2,3)", &f_trialtype, NP, NP, 0, ME_DEC,
"#_of_trials/condition", &f_ntrials, NP, NP, 0, ME_DEC,
"block_size", &f_nblocksize, NP, NP, 0, ME_DEC,
"reward_preset", &f_reward_preset, NP, NP, 0, ME_DEC,
"reward_random", &f_reward_random, NP, NP, 0, ME_DEC,
"reward_max", &f_reward_max, NP, NP, 0, ME_DEC,
"reward_step_size", &f_reward_step_size, NP, NP, 0, ME_DEC,
"verbose", &f_verbose, NP, NP, 0, ME_DEC,
NS,
};

char hm_sv_vl[] = 	"Preferred Heading: 1\n"
					"Preferred Pursuit: 2\n"
					"Tests            : 3\n"
					"FixedElevation   : 4";



/*
 * pursuit menu
 */
int test_parse(int flag, MENU *mp, char *astr, ME_RECUR *rp);
//"test_parse", test_parse_string, 0, test_parse, ME_BEF, ME_STR,
char test_parse_string[100] = {0};

float f_pursuit_min_deg = 0;
float f_pursuit_max_deg = 180;
int f_pursuit_nangles = 5;
float f_pursuit_speed_degpersec = 3;
int f_pursuit_blink_duration = 0;			/* duration of pursuit blink for blink trials only */

VLIST pursuit_vl[] = {
"pursuit_speed(deg/s)", &f_pursuit_speed_degpersec, 0, NP, 0, ME_FLOAT,
"steps_min_to_max", &f_pursuit_nangles, 0, NP, 0, ME_DEC,
"pursuit_ang_min(deg)", &f_pursuit_min_deg, 0, NP, 0, ME_FLOAT,
"pursuit_ang_max(deg)", &f_pursuit_max_deg, 0, NP, 0, ME_FLOAT,
"blink_duration(ms)", &f_pursuit_blink_duration, NP, NP, 0, ME_DEC,
NS,
};

char hm_pursuit[] = "For each step specified, the reverse (add 180 deg) angle is implied. For N steps, you really get 2*N.";



/*
 * heading menu
 */

float f_azimuth_min_deg = 0;
float f_azimuth_max_deg = 360;
float f_elevation_min_deg = -90;
float f_elevation_max_deg = 90;
int f_azimuth_nangles = 8;
int f_elevation_nangles = 5;
float f_heading_speed = 3;

VLIST heading_vl[] = {
"heading_speed", &f_heading_speed, 0, NP, 0, ME_FLOAT,
"num_azimuth_angles", &f_azimuth_nangles, 0, NP, 0, ME_DEC,
"azimuth_ang_min(deg)", &f_azimuth_min_deg, 0, NP, 0, ME_FLOAT,
"azimuth_ang_max(deg)", &f_azimuth_max_deg, 0, NP, 0, ME_FLOAT,
"num_elevation_angles", &f_elevation_nangles, 0, NP, 0, ME_DEC,
"elevation_ang_min(deg)", &f_elevation_min_deg, 0, NP, 0, ME_FLOAT,
"elevation_ang_max(deg)", &f_elevation_max_deg, 0, NP, 0, ME_FLOAT,
NS,
};

char hm_heading[] = "0 <= azimuth <= 720\n"
					"-90 < elevation <= 90\n";

struct test_spec
{
	int not_specified;			/* If nonzero, these values not specified in menu */
	float az, el, vh, rho, vp;
	char str[128];
};

typedef struct test_spec TestSpec;
#define MAX_SPEC 10
TestSpec f_spec[MAX_SPEC] = 
{
	{1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {"-"}},
	{1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {"-"}},
	{1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {"-"}},
	{1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {"-"}},
	{1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {"-"}},
	{1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {"-"}},
	{1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {"-"}},
	{1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {"-"}},
	{1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {"-"}},
	{1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {"-"}}
};

struct condition_spec
{
	int nconditions;
	int conditions[MAX_SPEC];
	char str[128];
};

#define MAX_TESTTYPE 4
typedef struct condition_spec ConditionSpec;
ConditionSpec f_conditions[MAX_TESTTYPE] = 
{
	{0, {0}, {"-"}},
	{0, {0}, {"-"}},
	{0, {0}, {"-"}},
	{0, {0}, {"-"}},
};


int parse_testspec(int flag, MENU *mp, char *astr, VLIST *vlp, int *tvadd);
int parse_conditionspec(int flag, MENU *mp, char *astr, VLIST *vlp, int *tvadd);

VLIST testspec_vl[] = {
	"Test_Condition_0", f_spec[0].str, &f_spec[0], parse_testspec, ME_BEF, ME_STR,
	"Test_Condition_1", f_spec[1].str, &f_spec[1], parse_testspec, ME_BEF, ME_STR,
	"Test_Condition_2", f_spec[2].str, &f_spec[2], parse_testspec, ME_BEF, ME_STR,
	"Test_Condition_3", f_spec[3].str, &f_spec[3], parse_testspec, ME_BEF, ME_STR,
	"Test_Condition_4", f_spec[4].str, &f_spec[4], parse_testspec, ME_BEF, ME_STR,
	"Test_Condition_5", f_spec[5].str, &f_spec[5], parse_testspec, ME_BEF, ME_STR,
	"Test_Condition_6", f_spec[6].str, &f_spec[6], parse_testspec, ME_BEF, ME_STR,
	"Test_Condition_7", f_spec[7].str, &f_spec[7], parse_testspec, ME_BEF, ME_STR,
	"Test_Condition_8", f_spec[8].str, &f_spec[8], parse_testspec, ME_BEF, ME_STR,
	"Test_Condition_9", f_spec[9].str, &f_spec[9], parse_testspec, ME_BEF, ME_STR,
	"HP_conditions", f_conditions[0].str, &f_conditions[0], parse_conditionspec, ME_BEF, ME_STR,
	"HSimP_conditions", f_conditions[1].str, &f_conditions[1], parse_conditionspec, ME_BEF, ME_STR,
	"HP+rs_conditions", f_conditions[2].str, &f_conditions[2], parse_conditionspec, ME_BEF, ME_STR,
	"H0P+rs_conditions", f_conditions[3].str, &f_conditions[3], parse_conditionspec, ME_BEF, ME_STR,
	NS,
};

char hm_testspec[] = "";


/* 
 * fixed elevation testing menu
 */

float f_fixed_elevation_deg = 0.0;
float f_fixed_elevation_azimuth_min_deg = 0;
float f_fixed_elevation_azimuth_max_deg = 180;
int f_fixed_elevation_azimuth_nangles = 0;
float f_fixed_elevation_pursuit_deg = 0;
float f_fixed_elevation_heading_speed = 3;
float f_fixed_elevation_pursuit_speed = 10;
	
VLIST fixedelevation_vl[] = 
{
	"Pursuit_direction", &f_fixed_elevation_pursuit_deg, NP, NP, 0, ME_FLOAT,
	"Pursuit_speed(deg/s)", &f_fixed_elevation_pursuit_speed, NP, NP, 0, ME_FLOAT,
	"Elevation_degrees", &f_fixed_elevation_deg, NP, NP, 0, ME_FLOAT,
	"Azimuth_min_degrees", &f_fixed_elevation_azimuth_min_deg, NP, NP, 0, ME_FLOAT,
	"Azimuth_max_degrees", &f_fixed_elevation_azimuth_max_deg, NP, NP, 0, ME_FLOAT,
	"num_Azimuth_angles", &f_fixed_elevation_azimuth_nangles, NP, NP, 0, ME_DEC,
	"Heading_speed", &f_fixed_elevation_heading_speed, NP, NP, 0, ME_FLOAT,
	NS
};

char hm_fixedelevation[] = "All angles in degrees.\n 0(right) <= azimuth <= 720\n0 <= pursuit <= 180\nPursuit angle and its inverse will be used.";




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
int f_fixation_fail_time = 1000;	/* break fixation timeout */
int f_acq_noise_time = 500;			/* if acq fails due to noise, wait this long before restarting */
int f_intertrial_time = 250;		/* time between trials */
int f_pre_pursuit_start_time = 250;	/* time when pursuit motion starts - this is the eps period prior to the measurement period */
int f_pursuit_start_time = 500;		/* time from fixation/dots on until pursuit starts */
int f_translation_start_time = 250;	/* time from fixation/dots on until dots motion */
int f_trial_end_time = 1000;		/* time from fixation/dots on until trial ends */

VLIST timing_vl[] = {
"trial_init_pause", &f_trial_init_pause_time, NP, NP, 0, ME_DEC,
"acq_time(ms)", &f_acq_time, NP, NP, 0, ME_DEC,
"acq_timeout(ms)", &f_acq_fail_time, NP, NP, 0, ME_DEC,
"acq_noise_timeout(ms)", &f_acq_noise_time, NP, NP, 0, ME_DEC,
"fix_time(ms)", &f_fixation_time, NP, NP, 0, ME_DEC,
"fix_fail_timeout(ms)", &f_fixation_fail_time, NP, NP, 0, ME_DEC,
"epsilon_start", &f_pre_pursuit_start_time, NP, NP, 0, ME_DEC,
"pursuit_start", &f_pursuit_start_time, NP, NP, 0, ME_DEC,
"translation_start_time", &f_translation_start_time, NP, NP, 0, ME_DEC,
"trial_end_time", &f_trial_end_time, NP, NP, 0, ME_DEC,
"intertrial_time(ms)", &f_intertrial_time, NP, NP, 0, ME_DEC,
NS,
};

char hm_timing[] = "";


int f_evWindowSize = 10;		// Size of window for computing running eye velocity average
int f_evUpdateHoldoff = 0;	// If nonzero, updates are held until tick counter reaches this
int f_evWxTest = 0;
int f_evWyTest = 0;
int f_evTest = 0;

VLIST retstab_vl[] = {
"window_size", &f_evWindowSize, NP, NP, 0, ME_DEC,
"update_holdoff", &f_evUpdateHoldoff, NP, NP, 0, ME_DEC,
"wx_test", &f_evWxTest, NP, NP, 0, ME_DEC,
"wy_test", &f_evWyTest, NP, NP, 0, ME_DEC,
"test", &f_evTest, NP, NP, 0, ME_DEC,
NS,
};

char hm_retstab[] = "";

/*
 * dots
 */
 
int f_ospace_block_size = 1000;
int f_ospace_dots_per_block = 1000;
int f_ospace_dot_color[4] = {255, 255, 255, 1};
float f_ospace_dot_size = 2.0;
int f_ospace_block_pool = 0;


VLIST ospace_vl[] = {
"dots/1000**3", &f_ospace_dots_per_block, NP, NP, 0, ME_DEC,
"pointsize(pixels)", &f_ospace_dot_size, NP, NP, 0, ME_FLOAT,
"dot_color(R)", &f_ospace_dot_color[0], 0, NP, 0, ME_DEC,
"dot_color(G)", &f_ospace_dot_color[1], 0, NP, 0, ME_DEC,
"dot_color(B)", &f_ospace_dot_color[2], 0, NP, 0, ME_DEC,
"pool_size", &f_ospace_block_pool, 0, NP, 0, ME_DEC,
NS,
};

char hm_ospace[] = "";

char f_local_addr[32]="192.168.1.1";	/* ip address of local machine */
char f_remote_addr[32]="192.168.1.2";	/* ip address of render machine */
int f_remote_port=2000;					/* port to use on render machine */


VLIST comm_vl[] = {
"local_ip", f_local_addr, NP, NP, 0, ME_STR,
"render_host_ip", f_remote_addr, NP, NP, 0, ME_STR,
"render_port", &f_remote_port, NP, NP, 0, ME_DEC,
NS,
};

char hm_comm[] = "";
 

MENU umenus[] = {
{"Main", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{"separator", NP}, 
{"PrefHeadingDir(1)", &heading_vl, NP, NP, 0, NP, hm_heading},
{"PrefPursuitDir(2)", &pursuit_vl, NP, NP, 0, NP, hm_pursuit},
{"Tests(3)", &testspec_vl, NP, NP, 0, NP, hm_testspec},
{"FixedElevation(4)", &fixedelevation_vl, NP, NP, 0, NP, hm_fixedelevation},
{"timing", &timing_vl, NP, NP, 0, NP, hm_timing}, 
{"stabilization", &retstab_vl, NP, NP, 0, NP, hm_retstab},
{"fixation", &fixpt_vl, NP, NP, 0, NP, hm_fixpt}, 
{"dots", &ospace_vl, NP, NP, 0, NP, hm_ospace}, 
{"viewing volume", &vvol_vl, NP, NP, 0, NP, hm_vvol},
{"background", &background_vl, NP, NP, 0, NP, hm_background}, 
{"communication", &comm_vl, NP, NP, 0, NP, hm_comm}, 
{NS},
};


/*
 * Channel definitions for data stuck in E file
 */

#define CHI_RANDOM_SEED 	100		// f_seed
#define CHI_TRIALTYPE		101		// f_trialtype
#define CHI_NTRIALS 		102		// f_ntrials
#define CHI_NBLOCKSIZE		103		// f_nblocksize
#define CHI_REWARD_PRESET	104		// f_reward_preset
#define CHI_REWARD_RANDOM	105		// f_reward_random
#define CHI_PURSUIT_STEPS	106		// f_pursuit_nangles
#define CHF_PURSUIT_MIN_DEG 107		// f_pursuit_min_deg
#define CHF_PURSUIT_MAX_DEG 108		// f_pursuit_max_deg
#define CHI_AZIMUTH_NANGLES 109		// f_azimuth_nangles
#define CHF_AZIMUTH_MIN_DEG 110		// f_azimuth_min_deg
#define CHF_AZIMUTH_MAX_DEG 111		// f_azimuth_max_deg
#define CHI_ELEVATION_NANGLES 112		// f_elevation_nangles
#define CHF_ELEVATION_MIN_DEG 113		// f_elevation_min_deg
#define CHF_ELEVATION_MAX_DEG 114		// f_elevation_max_deg
#define CHI_SCREEN_DISTANCE_MM	115		// f_screen_distance_MM
#define CHI_FAR_PLANE_DISTANCE	116		// f_far_plane_distance
#define CHI_BKGD_R				117		// f_background_color[0]
#define CHI_BKGD_G				118		// f_background_color[1]
#define CHI_BKGD_B				119		// f_background_color[2]

#define CHF_FIXPT_DIAMETER		123		// f_fixpt_diameter
#define CHF_FIXPT_WINDOW		124		// f_fixpt_diameter
#define CHI_FIXPT_R				125		// f_fixpt_color[0]
#define CHI_FIXPT_G				126		// f_fixpt_color[1]
#define CHI_FIXPT_B				127		// f_fixpt_color[2]
#define CHF_FIXPT_REFPOS_X		128		// f_fixpt_refpos[0]
#define CHF_FIXPT_REFPOS_Y		129		// f_fixpt_refpos[1]

#define CHI_TRIAL_INIT_PAUSE_TIME 	137	// f_trial_init_pause_time
#define CHI_ACQ_TIME				138	// f_acq_time
#define CHI_ACQ_FAIL_TIME			139	// f_acq_fail_time
#define CHI_FIXATION_TIME			140	// f_fixation_time
#define CHI_ACQ_NOISE_TIME			141	// f_acq_noise_time
#define CHI_INTERTRIAL_TIME			142	// f_intertrial_time
#define CHI_PRE_PURSUIT_START_TIME	143	// f_pre_pursuit_start_time
#define CHI_PURSUIT_START_TIME		144	// f_pursuit_start_time
#define CHI_TRANSLATION_START_TIME	144	// f_translation_start_time
#define CHI_TRIAL_END_TIME			145	// f_trial_end_time
#define CHI_BLINK_DURATION			146	// f_pursuit_blink_duration

#define CHI_OSPACE_NDOTS			153	// f_ospace_dots_per_block
#define CHI_OSPACE_POINTSIZE		154	// f_ospace_dot_size
#define CHI_OSPACE_R				155	// f_ospace_dot_color[0]
#define CHI_OSPACE_G				156 // f_ospace_dot_color[1]
#define CHI_OSPACE_B				157 // f_ospace_dot_color[2]
#define CHI_FRAMES_PER_SECOND		158	// f_frames_per_second

/*
 * channel definitions for per-trial bcodes
 */
 
#define CHI_TRIAL_CONDITION	1
#define CHF_AZIMUTH			2
#define CHF_ELEVATION  		3
#define CHF_HEADING_SPEED	4
#define CHF_RHO				5
#define CHF_PURSUIT_SPEED	6
#define CHI_TTYPE			7
#define CHI_PTYPE			8

#define CHF_EYEPOS_X		21
#define CHF_EYEPOS_Y		22
#define CHF_EYEPOS_Z		23
#define CHF_EYEDIR_A0		24
#define CHF_EYEDIR_A1		25
#define CHF_EYEDIR_A2		26
#define CHI_EYEDIR_FLAG		27
#define CHF_PTRANS_PHI		28
#define CHF_PTRANS_BETA		29
#define CHF_PTRANS_RHO		30
#define CHI_EVOK			31
#define CHI_EVX				32
#define CHI_EVY				33
#define CHF_AZCORR			34
#define CHF_ELCORR			35


/*
 * parse_testspec(int flag, MENU *mp, char *astr, VLIST *vlp, int *tvadd)
 * 
 * Called by REX menu system on changes to a menu. In this case, its changes to any of the menu
 * items where specific trial conditions are specified. See definition of VLIST testspec_vl[] above.
 * 
 * We are expecting 5 floating point numbers, space separated. This function is called before the 
 * variable specified in VLIST testspec_vl[] is set. If we return a non-zero value, it indicates there
 * was an error parsing, or that the values given are invalid, etc. Unfortunately, in that case the 
 * bad value remains in the dialog, even though its value doesn't represent the actual internal
 * variables. 
 * 
 * Anyways, the format of the input is: 
 * 
 * azimuth elevation heading_velocity pursuit_angle pursuit_speed
 * 
 * All angles (azimuth, elevation, pursuit) are in degrees. 
 * Heading velocity is in density-block-sizes per second. 
 * Pursuit velocity is in degrees per second. 
 * All rates are converted to "per-frame" values by dividing by 
 * the specified frame rate and truncating. 
 */

int parse_testspec(int flag, MENU *mp, char *astr, VLIST *vlp, int *tvadd)
{
	int status = 0;
	int num;
	TestSpec* spec = (TestSpec *)vlp->vl_basep;
	char tmpstr[P_LNAME];
	char *pc;

	/*
	 * Make a copy of the input strint 'astr' and convert all '/' to ' '.
	 */	
	strncpy(tmpstr, astr, P_LNAME);
	for (pc = tmpstr; *pc; pc++) if (*pc == '/') *pc = ' ';
	dprintf("parse_testspec - flag %x astr >>%s<< tmp >>%s<< tvadd.s >>%s<<\n", flag, astr, tmpstr, (char *)tvadd);
	
	
	if (!strcmp(tmpstr, "-"))
	{
		spec->not_specified = 1;
		dprintf("parse_testspec(%s) not_specified OK\n", tmpstr);
	}
	else
	{
		if ((num = sscanf(tmpstr, "%f %f %f %f %f", &spec->az, &spec->el, &spec->vh, &spec->rho, &spec->vp)) != 5)
		{
			status = -1;
			dprintf("parse_testspec(%s): expect 5 values, got %d. not specified ERR\n", astr, num);
			strcpy(spec->str, "-");
			spec->not_specified = 1;
		}
		else
		{
			dprintf("parse_testspec(%s): %d, %d, %d, %d, %d OK\n", tmpstr, (int)(spec->az*100), (int)(spec->el*100), (int)(spec->vh*100), (int)(spec->rho*100), (int)(spec->vp*100));
			spec->not_specified = 0;
		}
	}
	return status;
}


int parse_conditionspec(int flag, MENU *mp, char *astr, VLIST *vlp, int *tvadd)
{
	int status = 0;
	int num;
	char *pc;
	int i;
 	char tmpstr[P_LNAME];
	ConditionSpec* spec = (TestSpec *)vlp->vl_basep;
	spec->nconditions = 0;
	
	dprintf("parse_conditionspec - flat %x astr >>%s<< tvadd.s >>%s<<\n", flag, astr, (char *)tvadd);
	
	if (!strcmp(astr, "-"))
	{
		spec->nconditions = 0;
		dprintf("parse_conditionspec: %s : not_specified OK\n", astr);
	}
	else 
	{
		/*
		 * Expecting '/' separated numerals. Make a copy of the input strint 'astr' and convert all '/' to ' '.
		 */	
		strncpy(tmpstr, astr, P_LNAME);
		for (pc=tmpstr; *pc; pc++)
		{
			if (*pc == '/') *pc = ' ';
			else if (!isdigit(*pc)) status = 1;
		}
		if (status)
		{
			dprintf("parse_conditionspec: slash(/)-separated digits expected!\n");
		}
		else
		{
			num = sscanf(tmpstr, "%d %d %d %d %d %d %d %d %d %d", &spec->conditions[0], &spec->conditions[1], &spec->conditions[2], &spec->conditions[3], &spec->conditions[4], &spec->conditions[5], &spec->conditions[6], &spec->conditions[7], &spec->conditions[8], &spec->conditions[9]);
			for (i=0; i<num; i++)
			{
				if (spec->conditions[i] < 0 || spec->conditions[i] >= MAX_SPEC)
				{
					dprintf("parse_conditionspec(%s): condition index (%d) out of range.\n", astr, spec->conditions[i]);
					status = 1;
				}
			}
			if (!status)
			{
				spec->nconditions = num;
				dprintf("parse_conditionspec(%s) got %d conditions\n", astr, num);
			}
		}
	}
	
	return status;
}


/***************************** init_bighead *****************************
 *                                                                       *
 * This function is the "reset" function in the state set. In other words, 
 * the state set specification contains the line:
 * 
 * restart init_bighead
 * 
 * That line makes this function special, in that it will be called at the 
 * following times:
 * 
 * - the first time the clock is started
 * - whenever reset state happens
 *                                                                       *
 ************************************************************************/
void init_bighead(void)
{
	int status=0;
	int i;
	dprintf("init_bighead()\n");

	// initialize tcpip - must only do this OR gpib - NOT BOTH!!!!
	status = init_tcpip(f_local_addr, f_remote_addr, f_remote_port, 0);

	dprintf("init_bighead() - done.\n");
	
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
	PursuitTransformStruct ptrans;

	dprintf("Initializing render\n");

	// Get screen resolution and frame rate, then init pixel conversion.
	render_get_parameters(&f_width, &f_height, &f_frames_per_second);
	if (initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, f_width, f_height, f_screen_distance_MM))
	{
		dprintf("ERROR in initialize_pixel_conversion: \n"
				"x,y dimensions=(%d, %d)\n"
				"x,y resolution=(%d, %d)\n"
				"screen distance=%d\n", (int)x_dimension_mm, (int)y_dimension_mm, (int)f_width, (int)f_height, (int)f_screen_distance_MM);
	}

	
	// seed random number generator if necessary
	// TODO: Make usage of random number generator (and seed) consistent. 

	if (!f_seed_used)
	{
		ivunif(f_seed, 1);
		f_seed_used = 1;
	}
	
	// zero out handles and initialize counters.

	f_fixpt_handle = f_ospace_handle = f_ptrans_handle = 0;
	f_handle_count = 0;

	// background color

	render_bgcolor(f_background_color[0], f_background_color[1], f_background_color[2]);

	// Setup viewing volume -- this should be done prior to the groundplane init! */

	fovy = 2*atan2f(y_dimension_mm/2.0, f_screen_distance_MM); 	
	render_perspective(fovy, f_screen_distance_MM, f_far_plane_distance);
	dprintf("fovy %d dist %d far %d\n", (int)(fovy*180.0/M_PI * 100), f_screen_distance_MM, f_far_plane_distance);

	/* 
	 * Setup init camera position. 
	 * The camera will be at the default opengl position and orientation.
	 * Heading travel will be along the negative z azis. 
	 */

	f_camera.ex = 0;
	f_camera.ey = 0;
	f_camera.ez = 0;
	f_camera.dx = 0;
	f_camera.dy = 0;
	f_camera.dz = -1;
	f_camera.ux = 0;
	f_camera.uy = 1;
	f_camera.uz = 0;
	f_camera.flag = CAMERA_DEFAULT;
	render_camera_s(&f_camera);

	/*
	 * TODO: Configure outer space here. 
	 */
	f_outerspace.block_size_x = f_outerspace.block_size_y = f_outerspace.block_size_z = f_ospace_block_size;
	f_outerspace.dots_per_block = f_ospace_dots_per_block;
	f_outerspace.pool_size = f_ospace_block_pool;
	f_outerspace.dot_r = (unsigned char)f_ospace_dot_color[0];
	f_outerspace.dot_g = (unsigned char)f_ospace_dot_color[1];
	f_outerspace.dot_b = (unsigned char)f_ospace_dot_color[2];
	f_outerspace.dot_a = (unsigned char)f_ospace_dot_color[3];
	f_outerspace.dot_size = f_ospace_dot_size;
	f_outerspace.flags = 0;
	render_outerspace(&f_outerspace);

	ptrans.phi = 0;
	ptrans.beta = 0;
	ptrans.rho = 0;
	ptrans.ghandle = 0;
	ptrans.placeholder = 0;
	render_pursuittrans_p(&ptrans, PBeaconGHandle);

	return status;
}

int my_dot_init()
{
	int status = 0;
	
	/* 
	 * Configure fixation point.
	 * The dot lives in 3d, so its diameter must be converted at the right distance. 
	 * We do NOT use pixels - in the 2D case we do because of the orthographic projection, 
	 * but in the perspective 3D case we use mm (or whatever units distance to screen is measured in). 
	 * Note that I use 2*screendistance as the distance to the dot. That makes certain that the dot 
	 * appears on the screen. If we used just screendistance, then it would be in front of 
	 * the near plane and not visible unless at the very center. 
	 */
	
	f_fixpt.xorigin = 0;
	f_fixpt.yorigin = 0;
	f_fixpt.xsize = f_fixpt.ysize = f_screen_distance_MM*2 * tan(f_fixpt_diameter/2.0 * M_PI/180.0);
	f_fixpt.depth = f_screen_distance_MM*2;
	f_fixpt.r = f_fixpt_color[0];
	f_fixpt.g = f_fixpt_color[1];
	f_fixpt.b = f_fixpt_color[2];
	f_fixpt.flags = DOT_OVERWRITE;
	render_dot_p(&f_fixpt, f_ptrans_handle);
	
	return status;
}

	
/* 
 * my_check_for_handle 
 * 
 * Called as an escape function ... e.g. 
 * 
 * to wherever on 1 % my_check_for_handle
 * 
 * On each invocation it checks once for a handle.If a handle is found 
 * it is assigned to the proper variable and the handle counter is 
 * incremented.  
 * 
 * Returns 1 when both handles received.
 */

int my_check_for_handle()
{
	int status = 0;
	int handle = 0;
	if (render_check_for_handle(&handle))
	{
		if (f_handle_count == 0)
		{
			f_ospace_handle = handle;
			f_handle_count = 1;
			dprintf("Got ospace handle %d\n", f_ospace_handle);
		}
		else if (f_handle_count == 1)
		{
			f_ptrans_handle = handle;
			f_handle_count = 2;
			status = 1;
			dprintf("Got ptrans handle %d\n", f_ptrans_handle);
		}
	}
	return status;
}

int my_check_for_dot_handle()
{
	int status = 0;
	int handle = 0;
	if (render_check_for_handle(&handle))
	{
		f_fixpt_handle = handle;
		status = 1;
		dprintf("Got dot handle %d\n", f_fixpt_handle);
	}
	return status;
}


/* 
 * my_exp_init()
 * 
 * Initializations for overall experiment.
 * Determine the trial types, and create/populate the array of ExptCondition structs. Initialize the 
 * trial generator object. 
 * 
 */
 
int my_exp_init()
{
	dprintf("my_exp_init()\n");
	f_ms_per_frame = 1000/f_frames_per_second;
	f_all_done = 0;
	f_exp_abort = 0;
	f_nbpshlist = 0;
	switch (f_trialtype)
	{
		case TRIALTYPE_PREFERRED_HEADING:
		{
			if (generate_heading_trials())
			{
				dprintf("Error generating heading trials.\n");
				f_exp_abort = 1;
				return ERRORCD;
			}
			else
			{
				dprintf("Preferred heading trials, %d generated.\n", f_nbpshlist);
			}
			break;
		}
		case TRIALTYPE_PREFERRED_PURSUIT:
		{
			if (generate_pursuit_trials())
			{
				dprintf("Error generating pursuit trials.\n");
				f_exp_abort = 1;
				return ERRORCD;
			}
			else
			{
				dprintf("Preferred pursuit trials, %d generated.\n", f_nbpshlist);
			}
			break;
		}
		case TRIALTYPE_TEST:
		{
			if (generate_test_trials())
			{
				dprintf("Error generating test trials.\n");
				f_exp_abort = 1;
				return ERRORCD;
			}
			else
			{
				dprintf("Test trials, %d generated.\n", f_nbpshlist);
			}
			break;
		}
		case TRIALTYPE_FIXED_ELEVATION:
		{
			if (generate_fixed_elevation_trials())
			{
				dprintf("Error generating fixed elevation test trials.\n");
				f_exp_abort = 1;
				return ERRORCD;
			}
			else
			{
				dprintf("Test trials, %d generated.\n", f_nbpshlist);
			}
			break;
		}
		default:
		{
			dprintf("Trialtype must be 1, 2, or 3.\n");
			f_exp_abort = 1;
			return ERRORCD;
			break;
		}
	}


	/* 
	 * If no trials were generated its an error. 
	 */
	 
	if (f_nbpshlist == 0)
	{
		dprintf("No trials generated.\n");
		f_exp_abort = 1;
		return ERRORCD;
	}


	/* 
	 * Initialize trial generator. The trial generator will generate an index
	 * into the array f_exptcond[] which we just created and initialized.
	 */

	if (f_prtgen)
	{
		rtg_destroy(f_prtgen);
	}
	dprintf("Create random trial generator: %d trialspecs, %d trials each, blocksize %d\n", f_nbpshlist, f_ntrials, f_nblocksize);
	f_prtgen = rtg_create(f_nbpshlist, f_ntrials, f_nblocksize);

	/*
	 * Drop experimental settings into efile.
	 */

	render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_ospace_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_frame(0);
	 
	return 0;
}


/*
 * generate_fixed_elevation_trials()
 * 
 * Creates list of BPSHStruct within f_bpshlist for trials with a fixed elevation, 
 * a single pursuit direction (its reverse direction is implied as well), and a range
 * of azimuth values.
 * 
 * create_bpsh_struct() set blink frames based on settings from timing menu. For test trials, however, 
 * we do not want blinks. After calling create_bpsh_struct(), set bpsh.n_blink = 0. 
 * 
 * Returns 0 on success, nonzero if there was an error -- abort expt.
 * 
 */

int generate_fixed_elevation_trials()
{
	int status = 0;
	int i;
	int index;
	float step = 0;

	if (status = check_fixed_elevation_parameters()) return status;
	
	/* 
	 * Step size for azimuth angles.
	 */
	 
	if (f_fixed_elevation_azimuth_nangles > 1) step = (f_fixed_elevation_azimuth_max_deg - f_fixed_elevation_azimuth_min_deg)/(f_fixed_elevation_azimuth_nangles-1);
	dprintf("generate_fixed_elevation_trials: naz %d min %d max %d step %d\n", f_fixed_elevation_azimuth_nangles, (int)(f_fixed_elevation_azimuth_min_deg * 100), (int)(f_fixed_elevation_azimuth_max_deg * 100), step);

	/* 
	 * Loop over azimuth angles. At each step generate 3 x 2 + 1 bpsh structs (3 types, 2 pursuit angles, and a single non-pursuit case)
	 */
	 	
	for (i=0; i<f_fixed_elevation_azimuth_nangles; i++)
	{
		/*
		 * Heading+Pursuit
		 */		 

		if (f_nbpshlist < MAX_BPSHLIST)
		{
			if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_PURSUIT, 
									f_fixed_elevation_azimuth_min_deg + i*step, 
									f_fixed_elevation_deg, 
									f_fixed_elevation_heading_speed, 
									f_fixed_elevation_pursuit_deg, 
									f_fixed_elevation_pursuit_speed, 1, 0) < 0)
			{
				dprintf("generate_fixed_elevation_trials: error generating HP trials\n");
				status = 1;
				break;
			}				
		}
		
		if (f_nbpshlist < MAX_BPSHLIST)
		{
			if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_PURSUIT, 
									f_fixed_elevation_azimuth_min_deg + i*step, 
									f_fixed_elevation_deg, 
									f_fixed_elevation_heading_speed, 
									f_fixed_elevation_pursuit_deg + 180, 
									f_fixed_elevation_pursuit_speed, 1, 0) < 0)
			{
				dprintf("generate_fixed_elevation_trials: error generating HP trials\n");
				status = 1;
				break;
			}
		}
		
		/*
		 * Heading + simulated Pursuit
		 */

		if (f_nbpshlist < MAX_BPSHLIST)
		{
			if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_SIMULATED,
									f_fixed_elevation_azimuth_min_deg + i*step, 
									f_fixed_elevation_deg, 
									f_fixed_elevation_heading_speed, 
									f_fixed_elevation_pursuit_deg, 
									f_fixed_elevation_pursuit_speed, 1, 0) < 0)
			{
				dprintf("generate_fixed_elevation_trials: error generating HsimP trials\n");
				status = 1;
				break;
			}
		}
		
		if (f_nbpshlist < MAX_BPSHLIST)
		{
			if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_SIMULATED,
									f_fixed_elevation_azimuth_min_deg + i*step, 
									f_fixed_elevation_deg, 
									f_fixed_elevation_heading_speed, 
									f_fixed_elevation_pursuit_deg + 180, 
									f_fixed_elevation_pursuit_speed, 1, 0) < 0)
			{
				dprintf("generate_fixed_elevation_trials: error generating HsimP trials\n");
				status = 1;
				break;
			}
		}
		
		/*
		 * Heading+Pursuit with retinal stabilization
		 */

		if (f_nbpshlist < MAX_BPSHLIST)
		{
			if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_RETSTAB,
									f_fixed_elevation_azimuth_min_deg + i*step, 
									f_fixed_elevation_deg, 
									f_fixed_elevation_heading_speed, 
									f_fixed_elevation_pursuit_deg, 
									f_fixed_elevation_pursuit_speed, 1, 0) < 0)
			{
				dprintf("generate_fixed_elevation_trials: error generating HP+rs trials\n");
				status = 1;
				break;
			}				
		}
		
		if (f_nbpshlist < MAX_BPSHLIST)
		{
			if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_RETSTAB,
									f_fixed_elevation_azimuth_min_deg + i*step, 
									f_fixed_elevation_deg, 
									f_fixed_elevation_heading_speed, 
									f_fixed_elevation_pursuit_deg + 180, 
									f_fixed_elevation_pursuit_speed, 1, 0) < 0)
			{
				dprintf("generate_fixed_elevation_trials: error generating HP+rs trials\n");
				status = 1;
				break;
			}	
		}
		
		 
		/*
		 * Heading+noPursuit
		 */

		if (f_nbpshlist < MAX_BPSHLIST)
		{
			if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE,
									f_fixed_elevation_azimuth_min_deg + i*step, 
									f_fixed_elevation_deg, 
									f_fixed_elevation_heading_speed, 
									0, 
									0, 1, 0) < 0)
			{
				dprintf("generate_fixed_elevation_trials: error generating HnoP trial\n");
				status = 1;
				break;
			}				
		}
	}

	return status;
}


/*
 * generate_test_trials()
 * 
 * Creates list of BPSHStruct within f_pbpshlist for test trials.
 * The parameters in the test menu are used to determine the trial types. BPSHStruct are created
 * in f_pbpshstruct, and each element of that list represents the parameters for a unique trial type. 
 * 
 * create_bpsh_struct() set blink frames based on settings from timing menu. For test trials, however, 
 * we do not want blinks. After calling create_bpsh_struct(), set bpsh.n_blink = 0. 
 * 
 * Returns 0 on success, nonzero if there was an error -- abort expt.
 * 
 */



int generate_test_trials()
{
	int status = 0;
	int i;
	int index;
	BPSHStruct bpsh;
		
	/*
	 * There are 4 lines where users specify trial types to run. 
	 * If the lines are anything other than "-", 
	 */

	if (f_conditions[0].nconditions > 0)
	{
		/*
		 * Heading + pursuit trials
		 */
		for (i=0; i<f_conditions[0].nconditions; i++)
		{
			index = f_conditions[0].conditions[i];
			if (f_spec[index].not_specified)
			{
				dprintf("generate_test_trials(): HP condition %d is undefined.\n", index);
				status = 1;
			}
			else
			{
				dprintf("generate_test_trials(): HP condition %d is defined.\n", index);
				if (f_nbpshlist < MAX_BPSHLIST)
				{
					if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_PURSUIT, f_spec[index].az, f_spec[index].el, f_spec[index].vh, f_spec[index].rho, f_spec[index].vp, 1, 0) < 0)
					{
						status = 1;
					}
				}
			}
		}
	}

	if (f_conditions[1].nconditions > 0)
	{
		/*
		 * Heading + simulated pursuit trials
		 */
		for (i=0; i<f_conditions[1].nconditions; i++)
		{
			index = f_conditions[1].conditions[i];
			if (f_spec[index].not_specified)
			{
				dprintf("generate_test_trials(): HSimP condition %d is undefined.\n", index);
				status = 1;
			}
			else
			{
				dprintf("generate_test_trials(): HSimP condition %d is defined.\n", index);
				if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_SIMULATED, f_spec[index].az, f_spec[index].el, f_spec[index].vh, f_spec[index].rho, f_spec[index].vp, 1, 0) < 0)
				{
					status = 1;
				}
			}
		}
	}

	if (f_conditions[2].nconditions > 0)
	{
		/*
		 * Heading + pursuit with retinal stabilization trials
		 */
		 
		for (i=0; i<f_conditions[2].nconditions; i++)
		{
			index = f_conditions[2].conditions[i];
			if (f_spec[index].not_specified)
			{
				dprintf("generate_test_trials(): HP+RS condition %d is undefined.\n", index);
				status = 1;
			}
			else
			{
				dprintf("generate_test_trials(): HP+RS condition %d is defined.\n", index);
				if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_RETSTAB, f_spec[index].az, f_spec[index].el, f_spec[index].vh, f_spec[index].rho, f_spec[index].vp, 1, 0) < 0)
				{
					status = 1;
				}
			}
		}
	}

	if (f_conditions[3].nconditions > 0)
	{
		/*
		 * Frozen dots (no heading, or H0) + pursuit with retinal stabilization trials
		 */
		 
		for (i=0; i<f_conditions[3].nconditions; i++)
		{
			index = f_conditions[3].conditions[i];
			if (f_spec[index].not_specified)
			{
				dprintf("generate_test_trials(): H0P+RS condition %d is undefined.\n", index);
				status = 1;
			}
			else
			{
				dprintf("generate_test_trials(): H0P+RS condition %d is defined.\n", index);
				if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_RETSTAB, f_spec[index].az, f_spec[index].el, 0, f_spec[index].rho, f_spec[index].vp, 1, 0) < 0)
				{
					status = 1;
				}
			}
		}
	}

	return status;
}	



/*
 * generate_heading_trials()
 * 
 * Creates list of BPSHStruct within f_pbpshlist. Each element of that list represents
 * the parameters for a unique trial type. 
 * 
 * Returns 0 on success, nonzero if there was an error -- abort expt.
 * 
 */

int generate_heading_trials()
{
	int status = 0;
	int iaz;
	float az, el;
	float elstep, azstep;
	BPSHStruct bpsh;

	/*
	 * check input parameters
	 */
	 
	if (check_heading_parameters())
	{
		status = 1;
	}
	else
	{
		if (ANG_WITHIN_EPS(f_azimuth_min_deg, 0.0) && ANG_WITHIN_EPS(f_azimuth_max_deg, 360.0) &&
			f_azimuth_nangles == 8 &&
			ANG_WITHIN_EPS(f_elevation_max_deg, 90.0) && ANG_WITHIN_EPS(f_elevation_min_deg, -90.0) &&
			f_elevation_nangles == 5)
		{
			/*
			 * Default conditions lead to a specific set of headings. 
			 * We set the corresponding ecode for use when plotting rasters for this data. 
			 */

			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 0, 90, f_heading_speed, 0, 0, 0, 3000);

			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 0, 45, f_heading_speed, 0, 0, 0, 3010);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 45, 45, f_heading_speed, 0, 0, 0, 3011);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 90, 45, f_heading_speed, 0, 0, 0, 3012);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 135, 45, f_heading_speed, 0, 0, 0, 3013);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 180, 45, f_heading_speed, 0, 0, 0, 3014);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 225, 45, f_heading_speed, 0, 0, 0, 3015);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 270, 45, f_heading_speed, 0, 0, 0, 3016);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 315, 45, f_heading_speed, 0, 0, 0, 3017);

			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 0, 0, f_heading_speed, 0, 0, 0, 3020);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 45, 0, f_heading_speed, 0, 0, 0, 3021);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 90, 0, f_heading_speed, 0, 0, 0, 3022);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 135, 0, f_heading_speed, 0, 0, 0, 3023);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 180, 0, f_heading_speed, 0, 0, 0, 3024);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 225, 0, f_heading_speed, 0, 0, 0, 3025);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 270, 0, f_heading_speed, 0, 0, 0, 3026);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 315, 0, f_heading_speed, 0, 0, 0, 3027);

			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 0, -45, f_heading_speed, 0, 0, 0, 3030);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 45, -45, f_heading_speed, 0, 0, 0, 3031);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 90, -45, f_heading_speed, 0, 0, 0, 3032);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 135, -45, f_heading_speed, 0, 0, 0, 3033);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 180, -45, f_heading_speed, 0, 0, 0, 3034);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 225, -45, f_heading_speed, 0, 0, 0, 3035);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 270, -45, f_heading_speed, 0, 0, 0, 3036);
			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 315, -45, f_heading_speed, 0, 0, 0, 3037);

			create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, 0, -90, f_heading_speed, 0, 0, 0, 3040);
			
			dprintf("Using default heading set. Raster ecodes will be issued for each trial.\n");
		}
		else
		{
			
			/*
			 * Determine step sizes. Make sure we don't repeat 0, 360 degrees. Also make sure
			 * that if nangles=1 then we use just the min angle.
			 */
		
			if (f_azimuth_nangles==1)
			{
				azstep = 999999;		// this ensures the az loop below has just one step
			}
			else
			{
				if (ANG_WITHIN_EPS(f_azimuth_min_deg, 0.0) && ANG_WITHIN_EPS(f_azimuth_max_deg, 360.0))
				{
					azstep = (f_azimuth_max_deg - f_azimuth_min_deg)/f_azimuth_nangles;
				}
				else
				{
					azstep = (f_azimuth_max_deg - f_azimuth_min_deg)/(f_azimuth_nangles-1);
				}
			}
			
			if (f_elevation_nangles == 1)
			{
				elstep = 999999;
			}
			else
			{
				elstep = (f_elevation_max_deg - f_elevation_min_deg)/(f_elevation_nangles-1);
			}
		
			/*
			 * Generate trials for plain heading, no pursuit. 
			 */
		
			for (az = f_azimuth_min_deg, iaz = 0; az <= f_azimuth_max_deg; az += azstep, iaz++)
			{
				for (el = f_elevation_max_deg; el <= f_elevation_min_deg; el-= elstep)
				{
					
					/* 
					 * In elevation watch for the extremes +/- 90. 
					 * At those elevations we only do the first/last azimuth step.
					 * In that case the camera is pointing directly up/down. 
					 */
					 
					if ((ANG_WITHIN_EPS(el, -90) || ANG_WITHIN_EPS(el, 90.0)) && iaz)
					{
						continue;
					}
					else 
					{ 
						memset(&bpsh, 0, sizeof(BPSHStruct));
						if (create_bpsh_struct(BPSH_TTYPE_TRANSLATION, BPSH_PTYPE_NONE, az, el, f_heading_speed, 0, 0, 0, 0) < 0)
						{
							status = 1;
							break;
						}
					}
				}
			}
		}
	}
	return status;
}



/*
 * generate_pursuit_trials()
 * 
 * Creates list of BPSHStruct within f_pbpshlist for preferred pursuit trials.
 * The parameters in the pursuit menu are used to determine the trial types. BPSHStruct are created
 * in f_pbpshstruct, and each element of that list represents the parameters for a unique trial type. 
 * 
 * Returns 0 on success, nonzero if there was an error -- abort expt.
 * 
 */


int generate_pursuit_trials()
{
	int status = 0;
	float p, pstep;
	BPSHStruct bpsh;
	
	/* 
	 * Check input values. 
	 */

	if (check_pursuit_parameters())
	{
		status = 1;
	}
	else
	{

		if (ANG_WITHIN_EPS(f_pursuit_min_deg, 0.0) && ANG_WITHIN_EPS(f_pursuit_max_deg, 180.0) &&
			f_pursuit_nangles == 5)
		{
			/*
			 * Default conditions lead to a specific set of pursuit angles. 
			 * We set the corresponding ecode for use when plotting rasters for this data. 
			 */
			create_bpsh_struct(BPSH_TTYPE_NONE, BPSH_PTYPE_PURSUIT, 0, 0, 0,   0.0, f_pursuit_speed_degpersec, 0, 3050);
			create_bpsh_struct(BPSH_TTYPE_NONE, BPSH_PTYPE_PURSUIT, 0, 0, 0,  45.0, f_pursuit_speed_degpersec, 0, 3051);
			create_bpsh_struct(BPSH_TTYPE_NONE, BPSH_PTYPE_PURSUIT, 0, 0, 0,  90.0, f_pursuit_speed_degpersec, 0, 3052);
			create_bpsh_struct(BPSH_TTYPE_NONE, BPSH_PTYPE_PURSUIT, 0, 0, 0, 135.0, f_pursuit_speed_degpersec, 0, 3053);
			create_bpsh_struct(BPSH_TTYPE_NONE, BPSH_PTYPE_PURSUIT, 0, 0, 0, 180.0, f_pursuit_speed_degpersec, 0, 3054);
			create_bpsh_struct(BPSH_TTYPE_NONE, BPSH_PTYPE_PURSUIT, 0, 0, 0, 225.0, f_pursuit_speed_degpersec, 0, 3055);
			create_bpsh_struct(BPSH_TTYPE_NONE, BPSH_PTYPE_PURSUIT, 0, 0, 0, 270.0, f_pursuit_speed_degpersec, 0, 3056);
			create_bpsh_struct(BPSH_TTYPE_NONE, BPSH_PTYPE_PURSUIT, 0, 0, 0, 315.0, f_pursuit_speed_degpersec, 0, 3057);
			
			dprintf("Using default pursuit set. Raster ecodes will be issued for each trial.\n");
			
		}
		else
		{
			/*
			 * Determine step size
			 */
		
			if (f_pursuit_nangles == 1)
			{
				pstep = 999999;
			}
			else
			{
				pstep = (f_pursuit_max_deg - f_pursuit_min_deg)/(f_pursuit_nangles - 1);
			}
		
		
			/*
			 * Generate trials. For each pursuit angle, we create a trial type, and then we create
			 * another trial type for the angle+180. There is special handling for 180 degrees: If 0
			 * degrees was an option, then 180 is already covered, so skip it. 
			 */
			 
			for (p = f_pursuit_min_deg; p <= f_pursuit_max_deg; p += pstep)
			{
				if (ANG_WITHIN_EPS(p, 180) && ANG_WITHIN_EPS(f_pursuit_min_deg, 0))
				{
					dprintf("Skipping duplicate pursuit angle at 180.\n");
				}
				else
				{
					memset(&bpsh, 0, sizeof(BPSHStruct));
					bpsh.ttype = BPSH_TTYPE_NONE;
					if (create_bpsh_struct(BPSH_TTYPE_NONE, BPSH_PTYPE_PURSUIT, 0, 0, 0, p, f_pursuit_speed_degpersec, 0, 0) < 0 ||
					    create_bpsh_struct(BPSH_TTYPE_NONE, BPSH_PTYPE_PURSUIT, 0, 0, 0, p+180, f_pursuit_speed_degpersec, 0, 0) < 0)
					{
						return 1;
					}
				}
			}
		}
	}
	return 0;
}




/*
 * Using the currently set menu values, fill the array angles[] with pursuit angles and speeds[] with speeds. 
 * The value of *pnangles will be set to the number filled. The value max_angles is the maximum number of 
 * angles the arrays can hold. 
 * 
 * Returns 0 on success, or 1 if an error was found (bad angle input value, or too many values for arrays).
 */


int check_pursuit_parameters()
{
	int status = 0;	
	if (f_pursuit_min_deg < 0  || f_pursuit_min_deg > 180)
	{
		dprintf("Pursuit min angle out of range.\n");
		status = 1;
	}
	if (f_pursuit_max_deg < 0  || f_pursuit_max_deg > 180)
	{
		dprintf("Pursuit max angle out of range.\n");
		status = 1;
	}
	if (f_pursuit_min_deg >= f_pursuit_max_deg)
	{
		dprintf("Minimum pursuit angle is >= maximum pursuit angle.\n");
		status = 1;
	} 
	if (f_pursuit_nangles < 1)
	{
		dprintf("Num pursuit angles must be > 0\n");
		status = 1;
	}
	return status;
}


/* 
 * check_heading_parameters()
 * 
 * Check the user-entered parameters from the heading menu.
 * Return 0 if parameters are OK, 1 if there is an error in the parameters. 
 * 
 */

int check_heading_parameters()
{
	int status = 0;
	if (f_azimuth_min_deg < 0  || f_azimuth_min_deg > 720)
	{
		dprintf("Azimuth min angle out of range.\n");
		status = 1;
	}
	if (f_azimuth_max_deg < 0  || f_azimuth_max_deg > 720)
	{
		dprintf("Azimuth max angle out of range.\n");
		status = 1;
	}
	if (f_azimuth_min_deg >= f_azimuth_max_deg)
	{
		dprintf("Minimum azimuth angle is >= maximum azimuth angle.\n");
		status = 1;
	} 
	if (f_elevation_min_deg < -90  || f_elevation_min_deg > 90)
	{
		dprintf("Elevation min angle out of range.\n");
		status = 1;
	}
	if (f_elevation_max_deg < -90  || f_elevation_max_deg > 90)
	{
		dprintf("Elevation max angle out of range.\n");
		status = 1;
	}
	if (f_elevation_min_deg >= f_elevation_max_deg)
	{
		dprintf("Minimum elevation angle is >= maximum elevation angle.\n");
		status = 1;
	} 
	if (f_azimuth_nangles < 1)
	{
		dprintf("Num azimuth angles must be > 0.\n");
		status = 1;
	}
	if (f_elevation_nangles < 1)
	{
		dprintf("Num elevation angles must be > 0.\n");
		status = 1;
	}
			
	return status;
}


int check_fixed_elevation_parameters()
{
	int status = 0;

	/* 
	 * Fixed elevation angle
	 */

	if (f_fixed_elevation_deg < -90  || f_fixed_elevation_deg > 90)
	{
		dprintf("Elevation max angle out of range.\n");
		status = 1;
	}
	
	/*
	 * Azimuth angles
	 */
	if (f_fixed_elevation_azimuth_min_deg < 0  || f_fixed_elevation_azimuth_min_deg > 720)
	{
		dprintf("Azimuth min angle out of range.\n");
		status = 1;
	}
	if (f_fixed_elevation_azimuth_max_deg < 0  || f_fixed_elevation_azimuth_max_deg > 720)
	{
		dprintf("Azimuth max angle out of range.\n");
		status = 1;
	}
	if (f_fixed_elevation_azimuth_min_deg >= f_fixed_elevation_azimuth_max_deg)
	{
		dprintf("Minimum azimuth angle is >= maximum azimuth angle.\n");
		status = 1;
	} 
	if (f_fixed_elevation_azimuth_nangles < 1)
	{
		dprintf("Num azimuth angles must be > 0.\n");
		status = 1;
	}
	
	return status;
}



/*
 * Create BPSHStruct with given parameters. Malloc space and store in f_pbpshlist. Increment
 * f_nbpshlist if successful, and then return f_nbpshlist. If error, return -1, and leave
 * f_pbpshlist unchanged. 
 * 
 * az - azimuth angle degrees
 * el - elevation angle degrees
 * p - pursuit angle degrees
 * 
 * The is_test var indicates whether this is a test trial, or if its a preferred heading/pursuit trial.
 * In test trials we ignore blinks. Perhaps there will be other special handling, but for now thats it. 
 */

int create_bpsh_struct(int ttype, int ptype, float az, float el, float hspeed, float p, float pspeed, int is_test, int ecode)
{
	int status = 0;
	BPSHStruct bpsh;
	memset(&bpsh, 0, sizeof(BPSHStruct));
	bpsh.hptrans = f_ptrans_handle;
	bpsh.hospace = f_ospace_handle;
	bpsh.hdot = f_fixpt_handle;
	bpsh.evfactor = 25.0/6.0/f_frames_per_second;
	bpsh.ecode = ecode;
	bpsh.ttype = ttype;
	bpsh.ptype = ptype;
	bpsh.x = f_fixpt_refpos[0];
	bpsh.y = f_fixpt_refpos[1];
	bpsh.n_all_done = (int)floor(f_trial_end_time/1000.0*f_frames_per_second);
	switch (ttype)
	{
		case BPSH_TTYPE_TRANSLATION:
		{
			bpsh.alpha = az;
			bpsh.theta = el;
			bpsh.vh = f_ospace_block_size * hspeed/f_frames_per_second;
			bpsh.n_translation_start = (int)floor(f_translation_start_time/1000.0*f_frames_per_second);
			break;
		}
		case BPSH_TTYPE_NONE:
		{
			bpsh.vh = 0;
			break;
		}
		default:
		{
			dprintf("ERROR: create_bpsh_struct() - unknown translation type (%d)\n", ttype);
			status = 1;
			break;
		}
	}
	switch (ptype)
	{
		case BPSH_PTYPE_PURSUIT:
		case BPSH_PTYPE_SIMULATED:
		case BPSH_PTYPE_RETSTAB:
		{
			bpsh.rho = p;
			bpsh.vp = pspeed/f_frames_per_second;
			bpsh.n_pre_pursuit_start = (int)floor(f_pre_pursuit_start_time/1000.0*f_frames_per_second);
			bpsh.n_pursuit_start = (int)floor(f_pursuit_start_time/1000.0*f_frames_per_second);
			if (is_test)
			{
				bpsh.n_blink = 0;
			}
			else
			{
				bpsh.n_blink = (int)floor(f_pursuit_blink_duration/1000.0*f_frames_per_second);
			}
			break;
		}
		case BPSH_PTYPE_NONE:
		{
			bpsh.vp = 0;
			break;
		}	
		default:
		{
			dprintf("ERROR: create_bpsh_struct() - unknown pursuit type (%d)\n", ptype);
			status = 1;
			break;
		}
	}
	
	if (!status)
	{
		if (f_nbpshlist < MAX_BPSHLIST)
		{
			f_pbpshlist[f_nbpshlist] = (BPSHStruct *)malloc(sizeof(BPSHStruct));
			memcpy(f_pbpshlist[f_nbpshlist], &bpsh, sizeof(BPSHStruct));
			f_nbpshlist++;
			return f_nbpshlist;
		}
	}

	return -1;
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

	/* Initialize counters et al */
	/* Counts frames presented after fixation reached. See my_animate() and my_check_for_went(). 
	 * Below you will see that we increment f_step_counter aftercalling step(0). That's because
	 * the call to my_check_for_went is only made during the animation, not here.. */
	f_step_counter = 0;	
	f_animation_complete_flag = 0;
	f_pursuit_flag = 0;

	/*
	 * Now get next trial index. If the value is negative then all trials are done.
	 * If value is not negative, then create animation helper and take init step(0).
	 */
	 	
	f_trial_condition_index = f_prtgen->next(f_prtgen);
	if (f_trial_condition_index >= 0)
	{
		f_pbpsh = f_pbpshlist[f_trial_condition_index];
		initialize_bpsh_struct(f_pbpsh);
		// EV TESTING
		f_pbpsh->evType = f_evTest;
		
		if (f_verbose & DEBUG_INFO_BIT)
		{
			dprintf("=======Trial index %d =============\n", f_trial_condition_index);
			print_bpsh(f_pbpsh);
		}
		bpsh_step(f_pbpsh, 0);
		f_step_counter = 1;				
		
		// Set state times
		set_times("trial_init_pause", (long)f_trial_init_pause_time, -1);
		set_times("fixpt_acq", (long)f_acq_time, -1);
		set_times("fixpt_acq_fail_pause", (long)f_acq_fail_time, -1);
		set_times("fixpt_acq_noise_pause", (long)f_acq_noise_time, -1);
		set_times("fixpt_hold", (long)f_fixation_time, -1);
		set_times("fixation_fail_wait", (long)f_fixation_fail_time, -1);
		set_times("intertrial_pause", (long)f_intertrial_time, -1);
		
		/* 
		 * Set reward size for this trial 
		 */
		 
		if (f_reward_max > f_reward_preset && f_reward_step_size > 0)
		{ 
			long r = min(f_reward_preset+f_reward_step_size*f_trialsInARow, f_reward_max);
			set_times("reward_on", r, (long)f_reward_random);			
			dprintf("reward size for this trial will be %d\n", r);
		}
		else
		{
			set_times("reward_on", (long)f_reward_preset, (long)f_reward_random);
		}

		/*
		 * Initialize eye window
		 */
		 
		my_eye_window(EYE_WINDOW_INIT);
		
		/*
		 * Open the analog window. 
		 */
	
		awind(OPEN_W);
		
		
		/*
		 * bcodes for this trials conditions:
		 * f_pbpshlist[f_trial_condition_index]
		 * 
		 * trial index
		 * azimuth
		 * elevation
		 * heading speed
		 * pursuit angle
		 * pursuit speed
		 * 
		 */
		bcode_int(CHI_TRIAL_CONDITION, f_trial_condition_index);
		bcode_float(CHF_AZIMUTH, f_pbpshlist[f_trial_condition_index]->alpha);
		bcode_float(CHF_ELEVATION, f_pbpshlist[f_trial_condition_index]->theta);
		bcode_float(CHF_HEADING_SPEED, f_pbpshlist[f_trial_condition_index]->vh);
		bcode_float(CHF_RHO, f_pbpshlist[f_trial_condition_index]->rho);
		bcode_float(CHF_PURSUIT_SPEED, f_pbpshlist[f_trial_condition_index]->vp);
		bcode_int(CHI_TTYPE, f_pbpshlist[f_trial_condition_index]->ttype);
		bcode_int(CHI_PTYPE, f_pbpshlist[f_trial_condition_index]->ptype);
		
	}
	else
	{
		f_all_done = 1;
	}
	
	render_frame(0);


	return status;
}


/*
 * my_raster_ecode()
 * 
 * This is an action func that drops an ecode if the BPSHStruct has one set for this trial type. 
 * If no such code is set in BPSHStruct.ecode, then return 0 and no ecode is dropped. 
 */
 
int my_raster_ecode()
{
	return (f_pbpsh->ecode ? f_pbpsh->ecode : 0);
}



int my_animate()
{
	int iret = 0;
	int step_status = 0;

	step_status = bpsh_step(f_pbpsh, f_step_counter);
	
	if (step_status == -1)
	{
		f_animation_complete_flag = 1;
		f_pursuit_flag = 0;
	}
	else
	{
		if (step_status != 0)
		{
			int pstatus = (step_status & 0xff00) >> 8;
			int hstatus = (step_status & 0xff);
			if (pstatus & 0x1) ecode(PURSUIT_PAUSE);
			if (pstatus & 0x2) 
			{
				ecode(PURSUIT_PRE_START);
				f_pursuit_flag = 1;
			}
			if (pstatus & 0x4) ecode(PURSUIT_START);
			if (pstatus & 0x8) ecode(PURSUIT_BLINK_START);
			if (pstatus & 0x10) ecode(PURSUIT_BLINK_END);
			if (hstatus & 0x1) ecode(DOTS_ON);
			if (hstatus & 0x2) ecode(TRANS_START);
		}
		
		// bcodes 
		bcode_float(CHF_EYEPOS_X, f_pbpsh->cam.ex);
		bcode_float(CHF_EYEPOS_Y, f_pbpsh->cam.ey);
		bcode_float(CHF_EYEPOS_Z, f_pbpsh->cam.ez);
		bcode_float(CHF_EYEDIR_A0, f_pbpsh->cam.a0);
		bcode_float(CHF_EYEDIR_A1, f_pbpsh->cam.a1);
		bcode_float(CHF_EYEDIR_A2, f_pbpsh->cam.a2);
		bcode_int(CHI_EYEDIR_FLAG, f_pbpsh->cam.flag);
		bcode_float(CHF_PTRANS_PHI, f_pbpsh->ptrans.phi);
		bcode_float(CHF_PTRANS_BETA, f_pbpsh->ptrans.beta);
		bcode_float(CHF_PTRANS_RHO, f_pbpsh->ptrans.rho);
		bcode_int(CHI_EVOK, f_pbpsh->evOK);
		bcode_int(CHI_EVX, f_pbpsh->evx);
		bcode_int(CHI_EVY, f_pbpsh->evy);
		bcode_float(CHF_AZCORR, f_pbpsh->azcorrection);
		bcode_float(CHF_ELCORR, f_pbpsh->elcorrection);

	}

	render_frame_tagged(f_last_frametag);
	return 0;
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
	unsigned int framenumber, frametag;
	int itime;
//	f_wstatus = render_check_for_went(&frames);

	f_wstatus = render_check_for_went_extended(&frames, &framenumber, &frametag);
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
		
		// if frames were missed, dprint it and the cycles.
		// At step 1 we do not miss frames -- by design there is a pause after the initialization 
		// step (0), so the WENT will indicate missed frames. 

		itime = getClockTime();
		
		if (f_step_counter != 1)
		{
			if (frames != 1)
			{
				dprintf("**** Missed %d frames at step %d (%d check_went cycles) t %d dt %d num %d tag %d\n", frames, f_step_counter, f_went_cycles, itime, itime-f_last_time, framenumber, frametag);
				ecode(MISSEDCD);
			}

			if (itime-f_last_time < f_ms_per_frame-2)
			{
				dprintf("**** SHORT FRAME! step_counter %d dt=%d\n", f_step_counter, itime-f_last_time);
			}
			
			if (itime-f_last_time > f_ms_per_frame+2)
			{
				dprintf("**** Missed frame (%d check_went cycles)\n", f_went_cycles);
			}
			
			if (f_last_frametag != frametag)
			{
				dprintf("**** Frame tag error: last %d current %d framenumber %d\n", f_last_frametag, frametag, framenumber);
			}				
			/*
			else if ((itime - f_last_time) > 9)
			{
				dprintf("\n****\n\nMissed %d frames at step %d (%d check_went cycles) t %d *dt %d\n\n****\n", frames, f_step_counter, f_went_cycles, itime, itime-f_last_time);
				ecode(MISSEDCD);
			}
			*/
		}
		f_last_frametag = framenumber;	// the latest frame number will be
										// used as a tag for the next frame.
		f_last_time = itime;
		
		// reset went cycle counter, and increment step counter
		f_went_cycles = 0;
		f_step_counter++;
		
		
		if (f_animation_complete_flag) f_wstatus = 2;
				
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
	 
	my_eye_window(EYE_WINDOW_FIXPT_OFF);
	
	/* 
	 * Close analog window.
	 */
	
	awind(CLOSE_W);

	/*
	 * Tally trial if successful
	 */
	if (icorrect)
	{
		f_prtgen->mark(f_prtgen, f_trial_condition_index);
		f_trialsCompleted++;
		f_trialsInARow++;
	}
	else
	{
		f_trialsFailed++;
		f_trialsInARow = 0;
	}
	dprintf("\nTrials successful/failed/in_a_row = %d / %d / %d\n\n", f_trialsCompleted, f_trialsFailed, f_trialsInARow);
	
	/* 
	 * Clear screen. Sending FRAME - be sure to catch WENT!
	 */
	
	alloff();
	render_frame(0);

	/* Show status of random trial counts. */
	//show_trials_status();

	
	return 0;
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
		wd_src_pos(0, WD_DIRPOS, 0, WD_DIRPOS, 0);
		wd_pos(0, (long)(f_pbpsh->eyewx*10.0), (long)(f_pbpsh->eyewy*10.0));
		wd_siz(0, (long)(f_fixpt_window*10.0), (long)(f_fixpt_window*10.0));
		wd_cntrl(0, WD_OFF);
		wd_src_check(0, WD_SIGNAL, 0, WD_SIGNAL, 1);
	}
	
	if (iflag & EYE_WINDOW_FIXPT_UPDATE)
	{
		wd_pos(0, (long)(f_pbpsh->eyewx*10.0), (long)(f_pbpsh->eyewy*10.0));
		//dprintf("eyew update x=%d y=%d\n",  (long)(f_pbpsh->eyewx*10.0), (long)(f_pbpsh->eyewy*10.0));
	}

	if (iflag & EYE_WINDOW_FIXPT_ON)
	{
		//dprintf("eyew on\n");
		wd_cntrl(0, WD_ON);
	}

	if (iflag & EYE_WINDOW_FIXPT_OFF)
	{
		//dprintf("eyew off\n");
		wd_cntrl(0, WD_OFF);
	}


	return status;
}


int my_reward(int ireward)
{
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
	return 0;
}

void show_trials_status()
{
	int i, j;
	char snum[6];
	dprintf("===================Trial counts=================\n");
	dprintf("ind\ttally\n");
	dprintf("---\t-------------------------------\n");
	for (i=0; i<f_nbpshlist; i++)
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
 * Turns off fixpt and dots. Does not issue a render or frame. 
 * 
 */
 
int alloff()
{
	render_onoff(&f_ospace_handle, HANDLE_OFF, ONOFF_NO_FRAME);
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


// Turns fixpt on or off, and issues a FRAME.
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


int trial_didnot_start()
{
	fixpt_onoff(0);
	f_trialsInARow = 0;	/* Hack to force ramp back to beginning */
	if (f_reward_max > f_reward_preset && f_reward_step_size > 0)
	{ 
		set_times("reward_on", (long)f_reward_preset, (long)f_reward_random);			
		dprintf("reward size for this trial will be %d\n", f_reward_preset);
	}
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



#if 0
/*  
 * ev_init()
 * 
 * Called when starting eye velocity state loop. 
 * Create sliding windows of the size specified.
 * DO NOT attempt to access the BPSH struct f_pbpsh - it will not be 
 * defined when this state set starts up!
 */
 
int ev_init()
{
	f_psliderXPos = slider_create(f_evWindowSize);
	f_psliderYPos = slider_create(f_evWindowSize);
	f_psliderTime = slider_create(f_evWindowSize);
	return 0;
}


int ev_velocity(PSlider p)
{
	// Assume simple REX-type velocity calculation, Also assuming window size = 5!
	return (p->at(p, 4) + p->at(p, 3) - p->at(p, 1) - p->at(p, 0));
}


/*
 * ev_fill()
 * 
 * Called as an exit condition when in states ev_notfull and ev_full.
 * In both cases the current eye positions are added to the sliding windows, 
 * and velocity is computed (if windows are full). 
 * Returns 1 when window is full, 0 otherwise. 
 */

int ev_fill()
{
	int status = 0;
	f_psliderXPos->wadd(f_psliderXPos, eyeh);
	f_psliderYPos->wadd(f_psliderYPos, eyev);
	f_psliderTime->wadd(f_psliderTime, getClockTime());
	switch (f_evTest)
	{
		case 0:
		{
			if (f_psliderXPos->isFull(f_psliderXPos))
			{
				f_pbpsh->evx = ev_velocity(f_psliderXPos);
				f_pbpsh->evy = ev_velocity(f_psliderYPos);
				f_pbpsh->evOK = 1;
				status = 1;
			}
			break;
		}
		case 1:
		{
			if (f_psliderXPos->isFull(f_psliderXPos))
			{
				f_pbpsh->evx = (long)((float)f_psliderXPos->wsum(f_psliderXPos)/f_psliderXPos->size(f_psliderXPos));
				f_pbpsh->evy = (long)((float)f_psliderYPos->wsum(f_psliderYPos)/f_psliderYPos->size(f_psliderYPos));
				f_pbpsh->evOK = 1;
				status = 1;
			}
			break;
		}
		case 2:
		{
			f_pbpsh->evx = f_evWxTest;
			f_pbpsh->evy = f_evWyTest;
			f_pbpsh->evOK = 1;
			status = 1;
			break;
		}
	}
	return status;
}


/*
 * ev_wait()
 * 
 * Called when waiting for pursuit condition. 
 * Clear sliders and set vOK flag to 0.
 */

int ev_wait()
{
	f_psliderXPos->clear(f_psliderXPos);
	f_psliderYPos->clear(f_psliderYPos);
	f_psliderTime->clear(f_psliderTime);
	if (f_pbpsh) f_pbpsh->evOK = 0;
	return 0;
}


/* 
 * ev_saccade()
 * 
 * Called when a saccade has been detected during pursuit. 
 * Clear the sliders, but do not reset the vOK flag - we'll 
 * continue to use the latest estimated velocity.
 */

int ev_saccade()
{
	f_psliderXPos->clear(f_psliderXPos);
	f_psliderYPos->clear(f_psliderYPos);
	f_psliderTime->clear(f_psliderTime);
	return 0;
}
#endif

int evl_init()
{
	return evloop_init(f_evWindowSize);
}

int evl_clear()
{
	return evloop_clear();
}

int evl_fill()
{
	return evloop_fill();
}


/* 
 * REX state set starts here 
 */

%%

id 503
restart init_bighead
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
		to render_init_pause
	render_init_pause:
		time 100
		to render_init	
	render_init:
		do my_render_init()
		to dot_init on 1 % my_check_for_handle    /* get handle for outerspace and pursuit trans */
	dot_init:
		do my_dot_init()
		to exp_init on 1 % my_check_for_dot_handle	/* get handle for dot - need pursuit trans handle */
	exp_init:
		do my_exp_init()	/* initialize trial counters */
		to exp_abort on 1 = f_exp_abort
		to trial_init on 1 % render_check_for_went
	trial_init:
		code TRLSTART
		to my_trial_init
	my_trial_init:
		do my_trial_init()	/* initialization for a single trial */
		to all_done_wait on 1 = f_all_done
		to raster_ecode on 1 % render_check_for_went
	raster_ecode:
		do my_raster_ecode()
		to trial_init_pause
	trial_init_pause:
		time 500			/* This time will be updated based on menu entry - f_trial_init_pase - see my_trial_init() */
		to pause_detected on +PSTOP & softswitch
		to fixpt_on
	fixpt_on:
		do fixpt_onoff(1);
		to fixpt_window on 1 % render_check_for_went
	fixpt_window:
		time 10
		do my_eye_window(EYE_WINDOW_FIXPT_ON_UPDATE)	/* |EYE_WINDOW_FIXPT_ON)*/
		to fixpt_acq
	fixpt_acq:
		time 4000
		to pause_detected on +PSTOP & softswitch
		to fixpt_hold on -WD0_XY & eyeflag
		to fixpt_acq_fail
	fixpt_acq_fail:
		do trial_didnot_start()
		/*do fixpt_onoff(0)*/
		to fixpt_acq_fail_pause on 1 % render_check_for_went
	fixpt_acq_fail_pause:
		time 500			/* f_acq_fail_pause_time */
		do my_eye_window(EYE_WINDOW_FIXPT_OFF)
		to fixpt_on
	fixpt_hold:
		time 150			/* f_fixation_time */
		to pause_detected on +PSTOP & softswitch
		to fixpt_acq_noise on +WD0_XY & eyeflag
		to fixation
	fixpt_acq_noise:
		do my_eye_window(EYE_WINDOW_FIXPT_OFF)
		to fixpt_acq_noise_off
	fixpt_acq_noise_off:
		do trial_didnot_start()
		/*do fixpt_onoff(0)*/
		to fixpt_acq_noise_pause on 1 % render_check_for_went
	fixpt_acq_noise_pause:
		time 500			/* f_acq_noise_pause_time */
		to fixpt_on
	fixation:
		code FIXCD
		to animate
	animate:
		do my_animate()
		to animate_wait
	animate_wait:
		do my_eye_window(EYE_WINDOW_FIXPT_UPDATE)
		to fixation_fail_wait on +WD0_XY & eyeflag
		to pause_detected_wait on +PSTOP & softswitch
		to animate on 1 % my_check_for_went
		to trial_success on 2 = f_wstatus
	trial_success:
		code CORRECTCD
		do my_trial_done(1)
		to reward_on on 1 % render_check_for_went
	reward_on:
		code REWCD
		do my_reward(1)
		to reward_off
	reward_off:
		do my_reward(0)
		to intertrial_pause		
	fixation_fail_wait:
		time 500
		do my_fixation_fail_wait()
		to fixation_fail on 1 % render_check_for_went
	fixation_fail:
		code BREAKFIXCD
		do my_trial_done(0)
		to intertrial_pause on 1 % render_check_for_went
	intertrial_pause:
		time 500
		to trial_init
	pause_detected_wait:
		to pause_detected on 1 % render_check_for_went
	pause_detected:
		code PAUSECD
		do my_trial_done(0)
		to pause_wait on 1 % render_check_for_went
	pause_wait:
		to trial_init on -PSTOP & softswitch
	all_done_wait:
		to all_done on 1 % render_check_for_went
	all_done:
		do all_trials_done()
		to first on 1 = f_never
	exp_abort:
		do my_exp_abort()
		to first on 1 = f_never
/*abort	list:
		fixation_fail*/
}


evl_set {
	status ON
	begin	evl_begin:
		to evl_init
	evl_init:
		do evl_init()
		to evl_check
	evl_check:
		to evl_clear on -SF_OFF & sacflags
	evl_clear:
		do evl_clear()
		to evl_wait
	evl_wait:
		code EVL_WAIT
		to evl_pause on +PSTOP & softswitch
		to evl_fill on 1 = f_pursuit_flag
	evl_pause:
		do evl_clear()
		to evl_check on -PSTOP & softswitch
	evl_fill:
		code EVL_FILL
		to evl_delay on +SF_GOOD & sacflags				/* theoretically possible, if we start */
		to evl_saccade on +SF_ONSET & sacflags			/* _during_ or _just_after_ a saccade... */
		to evl_pause on +PSTOP & softswitch
		to evl_clear on 0 = f_pursuit_flag
		to evl_clear on 1 % evl_fill					/* evl_fill never returns 1 - it just keeps filling the slider */
	evl_saccade:
		code EVL_SACCADE
		do evl_clear()
		to evl_delay on +SF_GOOD & sacflags
		to evl_wait on - MY_SF_GOOD_OR_ONSET & sacflags
	evl_delay:
		code EVL_DELAY
		to evl_wait on -SF_GOOD & sacflags
}

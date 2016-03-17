#include <math.h>
#include "ldev.h"	/* hardware device defines */
#include "lcode.h"	/* Ecodes */
#include "lcalib.h"	/* local (per-rig) screen calibration */

#include "make_msg.h"			/* funcs to form command messages */
#include "transport.h"			/* funcs to send command messages to render */
#include "actions.h"			/* higher-level actions for render */
#include "pixels.h"				/* pixel conversion funcs */
#include "ivunif.h"				/* random number routines */

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


/* Really really big char for sprinting errors. */
char f_charbuf[2048];

/* the never var */
int f_never = -1;

/* counters handled by my_check_for_went */
int f_wstatus = 0;
int f_frame_counter = 0;
int f_went_counter = 0;
int f_jump_counter = 0;
int f_went_cycles = 0;

/* State list menu vars and associated vars*/
int f_seed = 0;
int f_seed_used = 0;
int f_jumps_per_trial = 10;
int f_ms_before_jump = 3000;
int f_ms_after_jump = 3000;
int f_frames_before_jump = 0;
int f_frames_after_jump = 0;
int f_ntrials = 5;
int f_trial_counter = 0;
int f_frames_per_second = 85;
char f_local_addr[32]="192.168.1.1";
char f_remote_addr[32]="192.168.1.2";
int f_remote_port=2000;
float f_steer_max_degrees = 3.0;
int f_steer_zero = 1024;
int f_steer_dead = 0;
int f_speed = 3;

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
int f_targ_offset_degrees[2] = { 10, -10};
int f_targ_offset_index = 1;

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
int f_blink_probability_per_jump = 0;
int f_blink_duration_ms=0;
int f_blink_guard_ms=0;
#define MAX_BLINK 5
typedef struct blink_struct
{
	int beginframe[MAX_BLINK];
	int endframe[MAX_BLINK];
	int index;
	int num;
	int state;
} BlinkStruct;
#define BLINK_OFF 996
#define BLINK_ON	997
#define BLINK_DO_NOTHING 998
BlinkStruct f_blink;
	

/* blip - target spot off/on */
int f_blip_debug=0;
int f_blip_probability_per_jump = 0;
int f_blip_duration_ms=0;
int f_blip_guard_ms=0;
int f_blip_amplitude_degrees=0;
#define MAX_BLIP 5
#define BLIP_BEGIN				149
#define BLIP_END				150
#define BLIP_IN_PROGRESS		151
#define BLIP_NOT_IN_PROGRESS	152
typedef struct blip_struct
{
	int beginframe[MAX_BLIP];
	int endframe[MAX_BLIP];
	int index;
	int num;
	int state;
} BlipStruct;
BlipStruct f_blip;

/* Reward stuff */
int f_going = 0;
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




/* REX menu declarations */
VLIST state_vl[] = {
"day_seed",		&f_seed, NP, NP, 0, ME_DEC,
"number_of_trials", &f_ntrials, NP, NP, 0, ME_DEC,
"jumps_per_trial",	&f_jumps_per_trial, NP, NP, 0, ME_DEC,
"ms_before_jump",	&f_ms_before_jump, NP, NP, 0, ME_DEC,
"ms_after_jump",	&f_ms_after_jump, NP, NP, 0, ME_DEC,
"Steering_max", &f_steer_max_degrees, NP, NP, 0, ME_FLOAT,
"Steering_zero_point", &f_steer_zero, NP, NP, 0, ME_DEC,
"Steering_dead_zone", &f_steer_dead, NP, NP, 0, ME_DEC,
"rew_win(deg)",	&f_reward_window_degrees, NP, NP, 0, ME_DEC,
"speed",	&f_speed, NP, NP, 0, ME_DEC,
"frame_rate(1/s)", &f_frames_per_second, NP, NP, 0, ME_DEC,
"local_ip", f_local_addr, NP, NP, 0, ME_STR,
"render_host_ip", f_remote_addr, NP, NP, 0, ME_STR,
"render_port", &f_remote_port, NP, NP, 0, ME_DEC,
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
"prob_per_jump[0,1000]", &f_blink_probability_per_jump, 0, NP, 0, ME_DEC,
"off_duration(ms)", &f_blink_duration_ms, 0, NP, 0, ME_DEC,
"guard(ms)", &f_blink_guard_ms, 0, NP, 0, ME_DEC,
"debug_msgs", &f_blink_debug, 0, NP, 0, ME_DEC,
NS,
};

char hm_blink[] = "";

VLIST blip_vl[] = {
"prob_per_jump[0,1000]", &f_blip_probability_per_jump, 0, NP, 0, ME_DEC,
"off_duration(ms)", &f_blip_duration_ms, 0, NP, 0, ME_DEC,
"guard(ms)", &f_blip_guard_ms, 0, NP, 0, ME_DEC,
"amplitude(degrees)", &f_blip_amplitude_degrees, 0, NP, 0, ME_DEC,
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
int next_target_offset();
void blink_prepare();
int blink_update(int frame);
void blip_prepare();
int blip_update(int frame, float *poffset);
int ecode(int code);
int ecode_multiplier(int basecd, float value, int factor);
int reward_check_window();
int reward_init();


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
	
	dprintf("init_steering paradigm\n");

	// initialize tcpip - must only do this OR gpib - NOT BOTH!!!!
	status = init_tcpip(f_local_addr, f_remote_addr, f_remote_port, 0);

	
//	// In case we did a Reset States
	f_going = 0;
	f_cam_trajectory = 0;
	f_targ_offset_index = 0;

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

	// seed random number generator if necessary
	if (!f_seed_used)
	{
		ivunif(f_seed, 1);
		f_seed_used = 1;
	}
	
	// zero out handles and initialize counters.
	f_gp_handle = f_targ_handle = 0;
	f_handle_count = 0;
	f_frames_before_jump = (int)((float)f_ms_before_jump/1000.0f*(float)f_frames_per_second);
	f_frames_after_jump = (int)((float)f_ms_after_jump/1000.0f*(float)f_frames_per_second);
	
	dprintf("before/after=%d/%d\n", f_frames_before_jump, f_frames_after_jump);
	
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
	
//	/* ecodes */
//	if (ecode(INITCD)) status = BADCD;
//	if (ecode(BASECD + (int)f_secondspertrial)) status = BADCD; 
//	if (ecode(BASECD + (int)f_secondsperdwell)) status = BADCD; 
//	if (ecode(BASECD + f_framespersecond)) status = BADCD; 
//	if (ecode(BASECD + targ_offset_degrees[0] + 360)) status = BADCD;
//	if (ecode(BASECD + targ_offset_degrees[1] + 360)) status = BADCD;

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
			dprintf("Ground plane handle is %d\n", f_gp_handle);
			f_handle_count = 1;
		}
		else if (f_handle_count == 1)
		{
			f_targ_handle = handle;
			dprintf("Target handle is %d\n", f_targ_handle);
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
	f_trial_counter = f_ntrials;
	alloff();
	render_frame(0);
	return 0;
}

int my_trial_init()
{
	f_frame_counter = 0;
	f_jump_counter = 0;
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

int my_trial_allon_offset()
{
	float targ_delta;
	targ_delta = next_target_offset() * M_PI / 180.;
	f_targ_dir = f_cam_trajectory + targ_delta;
	while (f_targ_dir < 0) f_targ_dir += (2*M_PI);
	while (f_targ_dir > (2*M_PI)) f_targ_dir -= (2*M_PI);

	f_target.xoffset = -sin(f_targ_dir);
	f_target.zoffset = cos(f_targ_dir);
	render_update(f_targ_handle, (void *)&f_target, sizeof(TargetStruct), HANDLE_ON);
	render_frame(0);
	return 0;
}

int my_trial_done()
{
	reward_pause(1);
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
	int update_ecode = UPDCD;	/* ecode dropped, may be changed below */
	
	
	/* Steering: update f_cam_trajectory based on joystick reading.  */
	
	ijoyh = joystick_value();		// joystick_value will return fakejoy value if fakejoy is enabled
	ival = ijoyh - f_steer_zero;		// subtract off zero point

	if (abs(ival) > f_steer_dead)
	{
		if (ival > 0)
			nval = (float)(ival - f_steer_dead)/(1024.f - (float)f_steer_dead);
		else 
			nval = (float)(ival + f_steer_dead)/(1024.f - (float)f_steer_dead);

		
		f_cam_trajectory += nval * f_steer_max_degrees * M_PI/180.f;		
		
		/* Make sure cam_trajectory is >=0 and <= 2*pi */
		while (f_cam_trajectory < 0) f_cam_trajectory += 2*M_PI;
		while (f_cam_trajectory > 2*M_PI) f_cam_trajectory -= 2*M_PI;
		
	}

	// beginning a jump period?
	// The 'update_ecode' may be modified here.
	if (f_went_counter == 0)
	{
		update_ecode = JBEGINCD;
	}
	else if (f_went_counter == f_frames_before_jump)
	{
		int offset = next_target_offset();
		if (offset < 0)
		{
			update_ecode = JLEFTCD;
			dprintf("JLEFT %d\n", offset);
		}
		else
		{
			update_ecode = JRIGHTCD;
			dprintf("JRIGHT %d\n", offset);
		}
		f_targ_dir = f_cam_trajectory + offset * M_PI / 180.0f;
		while (f_targ_dir < 0) f_targ_dir += (2*M_PI);
		while (f_targ_dir > (2*M_PI)) f_targ_dir -= (2*M_PI);
		targ_update_flag = 1;
	}

	// Handle blink. If the went counter is 0 we decide if there will be a blink and where it will be. 
	// We then decide whether we should take any action (i.e. turning target off or on). 
	if (f_went_counter == 0)
	{
		blink_prepare();
	}
	
	switch (blink_update(f_went_counter))
	{
		case BLINK_OFF:
			ecode(BLINKBEGINCD);
			render_onoff(&f_targ_handle, HANDLE_OFF, ONOFF_NO_FRAME);
			break;
		case BLINK_ON:
			ecode(BLINKENDCD);
			render_onoff(&f_targ_handle, HANDLE_ON, ONOFF_NO_FRAME);
			break;
	}
	
	
	// Handle blip. If the went counter is 0 we decide if there will be a blip and where it will be. 
	if (f_went_counter == 0)
	{
		blip_prepare();
	}

	switch (blip_update(f_went_counter, &blip_delta))
	{
		case BLIP_BEGIN:
			ecode(BLIPBEGINCD);
			ecode_multiplier(BASECD, f_blip_amplitude_degrees * 180.0f/M_PI, 10);
			ecode(BASECD + f_blip_duration_ms);
			// TODO: blip values here, amplitude, duration
			break;
		case BLIP_IN_PROGRESS:
			f_targ_dir += blip_delta;
			f_cam_trajectory += blip_delta;
			targ_update_flag = 1;
			break;
		case BLIP_END:
			ecode(BLIPENDCD);
			f_targ_dir += blip_delta;
			f_cam_trajectory += blip_delta;
			targ_update_flag = 1;
			break;
		default:
			break;
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
	if (ecode(update_ecode)) status = BADCD;
	if (ecode(BASECD + ijoyh)) status = BADCD;
	if (ecode_multiplier(BASECD, f_cam_trajectory * 180.0f/M_PI, 10)) status = BADCD;	/* cam_trajectory should be between 0 and 2PI */
	if (ecode_multiplier(BASECD, f_targ_dir * 180.0/M_PI, 10)) status = BADCD;
	
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


		if (f_went_counter == (f_frames_before_jump + f_frames_after_jump))
		{
			dprintf("jump %d completed\n", f_jump_counter);
			f_went_counter = 0;
			f_jump_counter++;
			ecode(JENDCD);

			if (f_jump_counter == f_jumps_per_trial)
			{
				f_jump_counter = 0;
				f_wstatus = 2;	// trial done
				ecode(TRIALENDCD);
			}
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
		value = f_fake_joy_value;
	}
	return value;
}

int next_target_offset()
{
	f_targ_offset_index = 1 - f_targ_offset_index;
	return f_targ_offset_degrees[f_targ_offset_index];
}


/* ecode
 * Drops an ecode value to efile. 
 */

int ecode(int icode)
{
	EVENT ev;
	int status=0;
	
	if (icode < MINIMUMCD || icode > 8192)
	{
//		dprintf("WARNING: ecode out of range %d\n", icode);
		status = -1;
	}
	
	ev.e_code= (short int)icode;
	ev.e_key= i_b->i_time;
	ldevent(&ev);
	return status;
}		

int ecode_multiplier(int basecd, float value, int factor)
{
	int icode = basecd + (int)(value * factor);
	return ecode(icode);
}


void blink_prepare()
{
	int i;
	int blink_duration_frames=0;
	int blink_guard_frames=0;

	/* Initialize stuff */
	f_blink.state = BLINK_ON;
	f_blink.index = -1;
	f_blink.num = 0;
	
	/* Convert blink values to frames */
	blink_duration_frames = (int)((float)f_blink_duration_ms / 1000.0f * f_frames_per_second);
	blink_guard_frames = (int)((float)f_blink_guard_ms / 1000.0f * f_frames_per_second);
	
	/* determine how many blinks there will be for this dwell. */ 
	if (f_blink_probability_per_jump >= 0 && 
		f_blink_probability_per_jump <= 1000)
	{
		if (ivunif(0, 999) < f_blink_probability_per_jump)
		{
			f_blink.num = 1;
		}
	}
	else
	{
		sprintf(f_charbuf, "ERROR! f_blink_probability_per_dwell must be in [0, 1000]!\n");
		rxerr(f_charbuf);
	}
	
	/* Now compute starting points for blink(s) */
	if ((2*blink_guard_frames) > f_frames_before_jump || (2*blink_guard_frames) > f_frames_after_jump) 
	{
		sprintf(f_charbuf, "ERROR! blink cannot fit between guards!");
		rxerr(f_charbuf);
	}
	else
	{
		for (i=0; i<f_blink.num; i++)	
		{
			int range_max = f_frames_before_jump + f_frames_after_jump - 4*blink_guard_frames;
			int rval = ivunif(0, range_max);
			
			/* Does blink fall before or after the jump? */
			if (rval < f_frames_before_jump-2*blink_guard_frames)
			{
				f_blink.beginframe[i] = blink_guard_frames + rval - blink_duration_frames/2;
				f_blink.endframe[i] = f_blink.beginframe[i] + blink_duration_frames;
			}
			else 
			{
				f_blink.beginframe[i] = 3*blink_guard_frames + rval - blink_duration_frames/2;
				f_blink.endframe[i] = f_blink.beginframe[i] + blink_duration_frames;
			}
		}
		if (f_blink.num > 0) f_blink.index = 0;
	}
	
	if (f_blink_debug)
	{
		dprintf("---------blink_prepare debug info ----------------\n");
		dprintf("blink frames(ms): guard|dur|before|after %d(%d)|%d(%d)|%d(%d)|%d(%d)\n", 
				blink_guard_frames, f_blink_guard_ms,
				blink_duration_frames, f_blink_duration_ms, 
				f_ms_before_jump, f_frames_before_jump, 
				f_ms_after_jump, f_frames_after_jump);
		for (i=0; i<f_blink.num; i++)
		{
			dprintf("%d beg|end frames %d|%d\n", i, f_blink.beginframe[i], f_blink.endframe[i]);
		}
	}
}

int blink_update(int frame)
{
	int status = BLINK_DO_NOTHING;
	if (f_blink.num > 0 && f_blink.index >= 0)
	{
		if (f_blink.state == BLINK_ON)
		{
			if (frame >= f_blink.beginframe[f_blink.index]) 
			{
				status = f_blink.state = BLINK_OFF;
				if (f_blink_debug)
				{
					dprintf("OFF ind %d frame %d\n", f_blink.index, frame);
				}
			}
		}
		else if (f_blink.state == BLINK_OFF)
		{
			if (frame >= f_blink.endframe[f_blink.index])
			{
				status = f_blink.state = BLINK_ON;
				if (f_blink_debug)
				{
					dprintf("ON ind %d frame %d\n", f_blink.index, frame);
				}
				if (++f_blink.index == f_blink.num) f_blink.index = -1;
			}
		}
	}
	return status;
} 

void blip_prepare()
{
	int i;
	int blip_duration_frames=0;
	int blip_guard_frames=0;

	/* Initialize stuff */
	f_blip.state = BLIP_NOT_IN_PROGRESS;
	f_blip.index = -1;
	f_blip.num = 0;
	
	/* Convert blip values to frames */
	blip_duration_frames = (int)((float)f_blip_duration_ms / 1000.0f * f_frames_per_second);
	blip_guard_frames = (int)((float)f_blip_guard_ms / 1000.0f * f_frames_per_second);
	
	/* determine how many blips there will be for this dwell. */ 
	if (f_blip_probability_per_jump >= 0 && 
		f_blip_probability_per_jump <= 1000)
	{
		if (ivunif(0, 999) < f_blip_probability_per_jump)
		{
			f_blip.num = 1;
		}
	}
	else
	{
		sprintf(f_charbuf, "ERROR! f_blip_probability_per_dwell must be in [0, 1000]!\n");
		rxerr(f_charbuf);
	}
	
	/* Now compute starting points for blip(s) */
	if ((2*blip_guard_frames) > f_frames_before_jump || (2*blip_guard_frames) > f_frames_after_jump) 
	{
		sprintf(f_charbuf, "ERROR! blip cannot fit between guards!");
		rxerr(f_charbuf);
	}
	else
	{
		for (i=0; i<f_blip.num; i++)	
		{
			int range_max = f_frames_before_jump + f_frames_after_jump - 4*blip_guard_frames;
			int rval = ivunif(0, range_max);
			
			/* Does blip fall before or after the jump? */
			if (rval < f_frames_before_jump-2*blip_guard_frames)
			{
				f_blip.beginframe[i] = blip_guard_frames + rval - blip_duration_frames/2;
				f_blip.endframe[i] = f_blip.beginframe[i] + blip_duration_frames;
			}
			else 
			{
				f_blip.beginframe[i] = 3*blip_guard_frames + rval - blip_duration_frames/2;
				f_blip.endframe[i] = f_blip.beginframe[i] + blip_duration_frames;
			}
		}
		if (f_blip.num > 0) f_blip.index = 0;
	}
	
	if (f_blip_debug)
	{
		dprintf("---------blip_prepare debug info ----------------\n");
		dprintf("blip frames(ms): guard|dur|before|after %d(%d)|%d(%d)|%d(%d)|%d(%d)\n", 
				f_blip_guard_ms, blip_guard_frames,
				f_blip_duration_ms, blip_duration_frames,  
				f_ms_before_jump, f_frames_before_jump, 
				f_ms_after_jump, f_frames_after_jump);
		for (i=0; i<f_blip.num; i++)
		{
			dprintf("%d beg|end frames %d|%d\n", i, f_blip.beginframe[i], f_blip.endframe[i]);
		}
	}
}


int blip_update(int frame, float *poffset)
{
	int status = BLIP_NOT_IN_PROGRESS;
	float offset = 0.0;
	if (f_blip.num > 0 && f_blip.index >= 0)
	{
		if (frame == f_blip.beginframe[f_blip.index])
		{
			status = BLIP_BEGIN;
		}
		else if (frame > f_blip.beginframe[f_blip.index] && frame <= f_blip.endframe[f_blip.index])
		{
			status = BLIP_IN_PROGRESS;
			if (frame == f_blip.endframe[f_blip.index]) status = BLIP_END;

			// compute displacement. 
			if (frame > f_blip.beginframe[f_blip.index])
			{
				float x = frame-f_blip.beginframe[f_blip.index];
				float xm1 = x-1;
				float A = f_blip_amplitude_degrees * M_PI / 180.0f;
				float F = f_blip.endframe[f_blip.index] - f_blip.beginframe[f_blip.index];

				// Use parabolic blip!
				offset = 4*A*((x/F)*(1.0f - x/F) - (xm1/F)*(1.0f - xm1/F));
			}
		}
	}
	*poffset = offset;
	return status;
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
	if (f_going)
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


int reward_init()
{
	f_reward_on_target = 0;
	f_reward_off_target = i_b->i_time;
	return 0;
}



int reward_pause(int isPaused)
{
	if (isPaused) f_going = 0;
	else f_going = 1;
	return 0;
}



/* REX state set starts here */

%%

id 400
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
		code TRIALCD
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
		to update on 1 % render_check_for_went
	update:
		do my_update()
		to p3
	p3:
		to p4 on +PSTOP & softswitch
		to update_wait on -PSTOP & softswitch
	p4:	
		code PAUSECD
		do reward_pause(1)
		to update_wait on -PSTOP & softswitch
	update_wait:
		do reward_pause(0)
		to update on 1 % my_check_for_went
		to trial_done on 2 = f_wstatus
	trial_done:
		do my_trial_done()
		to all_done on 1 ? f_trial_counter
		to trial_init
	all_done:
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

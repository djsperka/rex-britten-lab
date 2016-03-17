/*
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ldev.h"	/* hardware device defines */
#include "lcode.h"	/* Ecodes */
#include "lcalib.h"	/* local (per-rig) screen calibration */

#include "make_msg.h"	/* all messaging information */
#include "transport.h"	     /* defines for communication to render */
#include "actions.h"
#include "pixels.h"
#include "timerFunction.h"

#undef DEBUG
/*#define DEBUG 1*/

#define X 0
#define Y 1
#define TRUE 1
#define FALSE 0

#define PI 3.14159265
#define TARGD 100.0

/*
 * Ecodes local to the steering paradigm
 */

#define MINIMUMCD 1500
#define BASECD	1550
#define INITCD	1501 
#define TRIALCD	1502 
#define UPDCD	1503
#define TARGDIRCD 1510
#define MISSEDCD	1505	// missed frame code
#define TRIALENDCD 1506

#define ISICD	1900
#define GOCD	1901
#define BADCD	8192




/**********************************************************************
 * Global variables.
 **********************************************************************/
 
static int seedflg = 0;
static int flowflg = 1;

unsigned char going=0;					/* Flag that indicates camera is moving. Set to 0 during pauses. */
int handle_count=0;

float cam_position[3]; 							/* Current camera position */
float cam_start[3] = { 0, 0, 0 };		/* Initial eye position at start of trial */
int cam_height = 200;						/* Initial cam height - will be applied to cam_start (and used in updates) */
float cam_trajectory = 0;						/* trajectory - direction camera is moving - IN RADIANS*/
float cam_looking[3];
float cam_up[3] = { 0, 1, 0 };

/* globals for setting dot oscillation */
float targ_amplitude = 50;
float targ_period = 100;

/* target parameters. */	
float targ_size = 0.25;
int targ_color[3] = { 255, 0, 0 };
int targ_handle;
float targ_dir = 0;							/* radians */
float targ_h = 0;
float targ_dist = 1000;
int targ_offset_degrees[2] = { 10, -10};
int targ_offset_current = 0;

/* Perspective/camera parameters */
int screenDistanceMM = 290;
//int nearPlaneDistance = 1;
int farPlaneDistance = 5000;
//int fovy = 90;

/* groundpland parameters */
int gp_handle;
int gp_length=1000;
int gp_width=1000;
int gp_dots=200;
int gp_pool=500;
int gp_plane_color[3]= { 50, 50, 50 };
int gp_dot_color[3]= { 255, 255, 255 };
int gp_flags = 0;
float gp_dotsize=2.0;

/* background color */
int background_color[3] = { 0, 0, 0 };

/* Control variables */
int speed = 3;							/* units to advance per frame */
int f_trial_maxframes = 10000;		/* frames per trial */
int f_dwell_maxframes = 225;		/* frames between changes in target bearing */
int f_trial_framecounter;				/* run time frame counter. Trial ends when >= f_trial_maxframes */
int f_dwell_framecounter;				/* controls dot bearing alternation */
int f_framespersecond = 85;			/* frame rate */
int f_secondspertrial = 30;			/* f_trial_maxframes = f_secondspertrial * f_framespersecond */
int f_secondsperdwell = 10;			/* f_dwell_maxframes = f_secondsperdwell * f_framespersecond */
float steer_max_degrees=3.0;			/* trajectory bump at max steer in one direction */
int steer_zero = 1024;					/* joyh reading when going straight - i.e. joystick at rest */
int steer_dead=0;						/* dead zone on either size of steer_zero */
int f_wstatus;							/* Status code when checking for went */
int f_went_cycles=0;

/* Reward stuff */
int rew_window_degrees = 4;
int reward_flag = 0;

/* New reward stuff added 10-26-07 re: ramped reward wait time. ALL TIMES ARE IN MS */
int rew_on_target=0;			/* if 0, currently NOT on target, otherwise this is the last time on target */
int rew_off_target=0;			/* if 0, currently NOT off target, otherwise this is the time that target was lost */
int rew_on_target_sum=0;		/* Running total of time on target WITHOUT failing acquisition time test */
int rew_time=35;				/* time, in ms, that juice is open. Can be set via menu. */
int rew_acquisition_time=1000;	/* acquisition time, converted to ms in menu callback function */
int rew_ramp_time=5000;			/* time for reward wait to ramp from max to min */
int rew_wait_max = 1000;		/* max time for reward wait */
int rew_wait_min = 500;			/* min time for reward wait */
int rew_wait_counter=0;			/* counter used in rew wait state */

/* Addresses and port */
char *local_addr="192.168.1.1";
char *remote_addr="192.168.1.2";
int remote_port=2000;

/* Fake joystick */
int fake_joy_enable = 0;
int fake_joy_value = 1024;
int fake_joy_debug = 0;
#define FAKE_JOY_DEBUG_MAX 20
int fake_joy_debug_values[FAKE_JOY_DEBUG_MAX];
float fake_joy_traj_values[FAKE_JOY_DEBUG_MAX];
float fake_joy_nval_values[FAKE_JOY_DEBUG_MAX];
float fake_joy_delta_values[FAKE_JOY_DEBUG_MAX];
float fake_joy_x_values[FAKE_JOY_DEBUG_MAX];
float fake_joy_z_values[FAKE_JOY_DEBUG_MAX];
int fake_joy_debug_nvalues=0;
int fake_joy_timing = 0;
int fake_joy_timer = 0;
int fake_joy_jitter = 0;
int fake_joy_jitter_steps_low = 5;
int fake_joy_jitter_low = 1030;
int fake_joy_jitter_steps_high = 5;
int fake_joy_jitter_high = 1031;
int fake_joy_jitter_step = 0;

#define FAKE_JOY_DEBUG_FLAG 0x1
#define FAKE_JOY_NVAL_FLAG 0x2
#define FAKE_JOY_TRAJ_FLAG 0x4
#define FAKE_JOY_DELTA_FLAG 0x8
#define FAKE_JOY_POS_FLAG 0x10

/* messaging structures. */
TargetStruct f_target;
MessageStruct msg;

/************************************************************************
 * Declaration of statelist variables.
 ************************************************************************/
long seed = 1111;
	
int rew_crit = 5;


/***************************** rinitf() **********************************
 *                                                                       *
 * This function is the "reset" function in the state set. In other words, 
 * the state set specification contains the line:
 * 
 * restart rinitf
 * 
 * That line makes this function special, in that it will be called at the 
 * following times:
 * 
 * - the first time the clock is started
 * - whenever reset state happens
 *                                                                       *
 ************************************************************************/
void rinitf(void)
{
	int status=0;

	// initialize tcpip - must only do this OR gpib - NOT BOTH!!!!
	status = init_tcpip(local_addr, remote_addr, remote_port, 0);
	
	// In case we did a Reset States
	going = 0;
	cam_trajectory = 0;
	targ_offset_current = 0;

	// initialize pixel conversion
	if (initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, x_resolution, y_resolution, screenDistanceMM))
	{
		dprintf("ERROR in initialize_pixel_conversion: \n"
				"x,y dimensions=(%d, %d)\n"
				"x,y resolution=(%d, %d)\n"
				"screen distance=%d\n", (int)x_dimension_mm, (int)y_dimension_mm, (int)x_resolution, (int)y_resolution, (int)screenDistanceMM);
	}
	
}



int my_render_init()
{
	int status=0;
	float fovy;
	float size;
	float height;
	
	// zero out handles and counter
	gp_handle = targ_handle = 0;
	handle_count = 0;
	
	// background color
	render_bgcolor(background_color[0], background_color[1], background_color[2]);

	// Setup viewing volume -- this should be done prior to the groundplane init! */
	fovy = 2*atan2f(y_dimension_mm/2.0, screenDistanceMM); 	
	render_perspective(fovy, screenDistanceMM, farPlaneDistance);

	// Initialize groundplane	
	render_groundplane(gp_length, gp_width, gp_dots, gp_pool, gp_plane_color, gp_dot_color, gp_dotsize, gp_flags);

	/* Setup init camera position */
	cam_position[0] = cam_start[0];
	cam_position[1] = (float)cam_height;
	cam_position[2] = cam_start[2];
	cam_looking[0] = -sin(cam_trajectory);
	cam_looking[1] = 0;
	cam_looking[2] = cos(cam_trajectory);
	render_camera(cam_position, cam_looking, cam_up);

	/* Configure target.
	 * The target size is converted from angular degrees to appropriate units. */
	
	targ_dir = targ_offset_degrees[targ_offset_current] * PI/180.;
	f_target.xoffset = -sin(targ_dir);
	f_target.zoffset = cos(targ_dir);
	size = targ_dist * sin(targ_size * PI / 180.0);
	height = targ_dist * sin(targ_h * PI / 180.0);
	f_target.xsize = f_target.ysize = size;
	f_target.d = targ_dist;
	f_target.h = height;
	f_target.r = targ_color[0];
	f_target.g = targ_color[1];
	f_target.b = targ_color[2];
	render_target(&f_target);
	
	/* HANDLE message will be returned by render. We need the handle before we can turn the target off. */


	/* Set up timer if needed */
	if (fake_joy_timing)
	{
		timerFunction(1);
		initThisTimerHisto(fake_joy_timer, "send timing", 100, 100);
	}


	/* ecodes */
	if (ecode(INITCD)) status = BADCD;
	if (ecode(BASECD + (int)f_secondspertrial)) status = BADCD; 
	if (ecode(BASECD + (int)f_secondsperdwell)) status = BADCD; 
	if (ecode(BASECD + f_framespersecond)) status = BADCD; 
	if (ecode(BASECD + targ_offset_degrees[0] + 360)) status = BADCD;
	if (ecode(BASECD + targ_offset_degrees[1] + 360)) status = BADCD;

	return status;
}



/***************************** my_trial_init() ********************************
 * 
 * Initialization for start of trial. initialize counters (like frame counter), and drop ecodes to 
 * save parameters relevant to the trial. 
 * 
 * Note that framecounter is set to 0 here. We're not using rex's escape condition ' 1 ? count' to decrement
 * the frame counter. Instead, the action 'my_check_for_went' increments the counter based on the 
 * number of frames since the last call.
 * 
 * update 8-15-07 djs
 * Target is positioned in center of screen (same direction as cam_trajectory). This is so scene can be redrawn 
 * (in another state) for a pause, with a BEEP cue, before the new trial proceeds. This means an additional update
 * of the target (to change its direction) will be required before the trial proceeds. 
 */

int my_trial_init(void)
{
	int status = 0;
	f_trial_framecounter = 0;
	f_trial_maxframes = f_secondspertrial * f_framespersecond;
	f_dwell_maxframes = f_secondsperdwell * f_framespersecond;

	/* dwell counter counts backwards! */
	f_dwell_framecounter = f_dwell_maxframes;
	
	/* Position target at current target bearing value. 
	 * The current value of cam_trajectory was valid prior to the last update..... so when we're
	 * running (i.e. between trials) this value isn't strictly accurate, but its pretty darned close 
	 */	
	targ_dir = cam_trajectory;
	f_target.xoffset = -sin(targ_dir);
	f_target.zoffset = cos(targ_dir);
	render_update(targ_handle, (void *)&f_target, sizeof(TargetStruct), HANDLE_OFF);

	// 11-1-07 djs. Turn off gp AND target between trials. 
	render_onoff(&gp_handle, HANDLE_OFF, ONOFF_NO_FRAME);

	/* WORKAROUND for jitter?
	 * Jitter may be affected by the distance from the origin? Home the values between trials. 
	 */
	home();
	render_camera(cam_position, cam_looking, cam_up);

	/* render the now-black-screen */
	render_frame(1);
		
	/* ecodes */
	
	if (ecode(TRIALCD)) status = BADCD;
	if (ecode(BASECD + targ_offset_degrees[targ_offset_current] + 360)) status = BADCD;

   return 0;
}


/* target_update
 * 
 * update target position to the current offset. This is called once just prior to a trial starting. The trial_init sets target
 * dir (targ_dir) to dead-ahead (targ_dir = cam_trajectory). Here we set it to targ_dir + targ_offset_degrees[targ_offset_current] * PI/180
 * 
 */

int target_update()
{
	targ_dir = cam_trajectory + targ_offset_degrees[targ_offset_current] * PI / 180.;
	while (targ_dir < 0) targ_dir += (2*PI);
	while (targ_dir > (2*PI)) targ_dir -= (2*PI);
	f_target.xoffset = -sin(targ_dir);
	f_target.zoffset = cos(targ_dir);
	render_update(targ_handle, (void *)&f_target, sizeof(TargetStruct), 0);
	return 0;
}




/* my_trial_allon
 * 
 * Called once to turn on ground plane and target, and to send a FRAME. 
 * 
 */
int my_trial_allon()
{
	render_onoff(&gp_handle, HANDLE_ON, ONOFF_NO_FRAME);
	render_onoff(&targ_handle, HANDLE_ON, ONOFF_NO_FRAME);
	render_frame(0);
	return 0;
}


/* my_update
 * 
 * per-frame update while camera is in motion. 
 * Fetch joystick value and update cam trajectory.  Check the frame count to see if we're past the dwell count and its time
 * to change the target trajectory. 
 */
   
int my_update()
{

	/* TODO: first update trajectory 
	 * cam_trajectory is modified based on joystick reading. 
	 * After that, the camera looking vector (cam_looking) is set to be the same as the trajectory. 
	 * This trajectory is used to move the camera forward. 
	 * The result is that the motion is always in the same direction as the camera. 
	 */

	int ival;		/* input joystick value, shifted by zero value */
	float nval;		/* normalized joystick value */
	float delta = 0;
	int ijoyh;		/* in case we're interrupted, better save the value used. */
	int i;
	int status = 0;
	int targ_dir_changed = 0;	// flag value for ecode 
	
	ijoyh = joystick_value();
	ival = ijoyh - steer_zero;

	if (abs(ival) > steer_dead)
	{
		if (ival > 0)
			nval = (float)(ival - steer_dead)/(1024.f - (float)steer_dead);
		else 
			nval = (float)(ival + steer_dead)/(1024.f - (float)steer_dead);

		/* 1-23-07 djs Change steer_max_degrees to a float - allows for greater sensitivity. */
		delta = nval * steer_max_degrees * PI/180.f;
		
		cam_trajectory += delta;		
		
		/* Make sure cam_trajectory is >=0 and <= 2*pi */
		while (cam_trajectory < 0) cam_trajectory += 2*PI;
		while (cam_trajectory > 2*PI) cam_trajectory -= 2*PI;

		if (fake_joy_debug)
		{
			fake_joy_debug_values[fake_joy_debug_nvalues] = ijoyh;
			fake_joy_nval_values[fake_joy_debug_nvalues] = nval;
			fake_joy_traj_values[fake_joy_debug_nvalues] = cam_trajectory;
			fake_joy_delta_values[fake_joy_debug_nvalues] = delta;
			fake_joy_x_values[fake_joy_debug_nvalues] = cam_position[0];
			fake_joy_z_values[fake_joy_debug_nvalues] = cam_position[2];
			fake_joy_debug_nvalues++;
			if (fake_joy_debug_nvalues == FAKE_JOY_DEBUG_MAX)
			{
				if (fake_joy_debug & FAKE_JOY_DEBUG_FLAG)
				{
					dprintf("IJOYH ");
					for (i=0; i<FAKE_JOY_DEBUG_MAX ; i++)
					{
						dprintf("%d ", fake_joy_debug_values[i]);
					}
					dprintf("\n");
				}
				if (fake_joy_debug & FAKE_JOY_NVAL_FLAG)
				{
					dprintf("NVAL ");
					for (i=0; i<FAKE_JOY_DEBUG_MAX ; i++)
					{
						dprintf("%d ", (int)(fake_joy_nval_values[i]*10000));
					}
					dprintf("\n");
				}
				if (fake_joy_debug & FAKE_JOY_TRAJ_FLAG)
				{
					dprintf("TRAJ ");
					for (i=0; i<FAKE_JOY_DEBUG_MAX ; i++)
					{
						dprintf("%d ", (int)(fake_joy_traj_values[i]*1000 * 180.0f/PI));
					}
					dprintf("\n");
				}
				if (fake_joy_debug & FAKE_JOY_DELTA_FLAG)
				{
					dprintf("DELT ");
					for (i=0; i<FAKE_JOY_DEBUG_MAX ; i++)
					{
						dprintf("%d ", (int)(fake_joy_delta_values[i]*1000 * 180.0f/PI));
					}
					dprintf("\n");
				}
				if (fake_joy_debug & FAKE_JOY_POS_FLAG)
				{
					dprintf("X    ");
					for (i=0; i<FAKE_JOY_DEBUG_MAX ; i++)
					{
						dprintf("%d ", (int)(fake_joy_x_values[i]));
					}
					dprintf("\n");
					dprintf("Z    ");
					for (i=0; i<FAKE_JOY_DEBUG_MAX ; i++)
					{
						dprintf("%d ", (int)(fake_joy_z_values[i]));
					}
					dprintf("\n");
				}
				
				fake_joy_debug_nvalues = 0;
			}
		}
		else
		{
			fake_joy_debug_nvalues = 0;
		}
		
	}


	/* camera looking vector */
	cam_looking[0] = -sin(cam_trajectory);
	cam_looking[1] = 0;
	cam_looking[2] = cos(cam_trajectory);

	/* Move camera position */	
	cam_position[0] += cam_looking[0] * speed;
	cam_position[1] = (float)cam_height;
	cam_position[2] += cam_looking[2] * speed;
	render_camera(cam_position, cam_looking, cam_up);

	/* Check dwell counter and switch target bearing if its expired. Note that the dwell counter assumes
	 * no dropped frames....though we cannot assume its true, we do it anyways for the purposes of this 
	 * counter. 
	 */

	/* djs 12-01-2006
      * Change target direction based on current direction, not on an absolute bearing. 
	 */	
	if (f_dwell_framecounter <= 0)
	{
		float targ_delta;
		targ_offset_current = 1-targ_offset_current;
		targ_delta = targ_offset_degrees[targ_offset_current] * PI / 180.;
		targ_dir = cam_trajectory + targ_delta;
		while (targ_dir < 0) targ_dir += (2*PI);
		while (targ_dir > (2*PI)) targ_dir -= (2*PI);
		
		f_dwell_framecounter = f_dwell_maxframes;

		f_target.xoffset = -sin(targ_dir);
		f_target.zoffset = cos(targ_dir);
		render_update(targ_handle, (void *)&f_target, sizeof(TargetStruct), 0);
		targ_dir_changed = 1;
	}
	else f_dwell_framecounter--;

	if (fake_joy_timing)
	{
		startTimer(fake_joy_timer);
		render_frame(0);
		stopTimer(fake_joy_timer);
	}
	else
	{
		render_frame(0);
	}
	going = 1;	/* in case we come back from a pause */

	/* ecodes */
	/* 9-13-07 target direction code - use actual targ_dir, not offset! Also, convert to degrees and mult * 10 */
	if (ecode(UPDCD)) status = BADCD;
	if (ecode(BASECD + ijoyh)) status = BADCD;
	if (ecode(BASECD + (int)(cam_trajectory * 180.0f/PI *10.0f))) status = BADCD;	/* cam_trajectory should be between 0 and 2PI */
	if (ecode(BASECD + (int)(targ_dir * 180.0/PI * 10.0f))) status = BADCD;
	
	return status;	
}


int joystick_value()
{
	int value=0;
	if (!fake_joy_enable) 
	{
		value = joyh;
	}
	else 
	{
		if (fake_joy_jitter)
		{
			fake_joy_jitter_step++;
			if (fake_joy_jitter_step <= fake_joy_jitter_steps_low)
			{
				value = fake_joy_jitter_low;
			}
			else if (fake_joy_jitter_step <= (fake_joy_jitter_steps_low + fake_joy_jitter_steps_high))
			{
				value = fake_joy_jitter_high;
			}
			if (fake_joy_jitter_step == (fake_joy_jitter_steps_low + fake_joy_jitter_steps_high))
			{
				fake_joy_jitter_step = 0;
			}
		}
		else 
		{
			value = fake_joy_value;
		}
	}
	return value;
}



/* my_check_for_handle *********************************/

int my_check_for_handle()
{
	int status = 0;
	int handle = 0;
	if (render_check_for_handle(&handle))
	{
		if (handle_count == 0)
		{
			gp_handle = handle;
			dprintf("Ground plane handle is %d\n", gp_handle);
			handle_count = 1;
		}
		else if (handle_count == 1)
		{
			targ_handle = handle;
			dprintf("Target handle is %d\n", targ_handle);
			handle_count = 2;
			status = 1;
		}
	}
	return status;
}




/* my_check_for_went *********************************/

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
		// went was found. 'frames' has frame count
		f_trial_framecounter += frames;
		
		// TEST - if frames were missed, dprint it and the cycles...
		if (frames > 1)
		{
			dprintf("Missed %d frames (%d check_went cycles)\n", frames, f_went_cycles);
			ecode(MISSEDCD);
		}
		f_went_cycles = 0;

		// TODO: Generate ecode based on frame count
		
		// check if trial is done
		if (f_trial_framecounter >= f_trial_maxframes)
		{
			f_wstatus = 2;
		}
	}
	return f_wstatus;
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

		


/***************************** check_reward_window() ********************************
 *
 * Checks reward "window" and sets flag
 */

int check_reward_window()
{
	float dot;
	float v;
	
	// We want to compare the camera trajectory to the target bearing. If trajectory is within rew_window
	// then a reward is given. 

	reward_flag = 0;	
	if (going)
	{
		dot = cos(cam_trajectory)*cos(targ_dir) + sin(cam_trajectory)*sin(targ_dir);
		if (dot > cos(rew_window_degrees * PI/180))
		{
			reward_flag = 1;
		}
		if (reward_flag)
		{
			/* in reward window last time? If so, add elapsed time to total on target. If not, check acquisition time. */
			if (rew_on_target)
			{
				rew_on_target_sum += (i_b->i_time - rew_on_target);
			}
			else
			{
				if ((i_b->i_time - rew_off_target) > rew_acquisition_time)
				{
					rew_on_target_sum = 0;
				}
			}
			rew_off_target = 0;
			rew_on_target = i_b->i_time;
			v = (float)rew_on_target_sum/(float)rew_ramp_time;
			if (v>1) v=1;
			rew_wait_counter = rew_wait_max - v*(rew_wait_max - rew_wait_min);
		}
		else
		{
			/* did we just transition from being on-target? */
			if (rew_on_target)
			{
				rew_on_target = 0;
				rew_off_target = i_b->i_time;
			}
		}
	}
	else
	{
		/* During a pause we reset things. Sorry. */
		rewinit();
	}

	return reward_flag;
}


int rewinit()
{
	rew_on_target = 0;
	rew_off_target = i_b->i_time;
	return 0;
}

int apause(int isPaused)
{
	if (isPaused) going = 0;
	else going = 1;
	return 0;
}


void dump_values(void)
{
	float dot;
	float deg;
	dprintf("Cam Pos %d %d %d\n", (int)(cam_position[0]*100), (int)(cam_position[1]*100), (int)(cam_position[2]*100));
	dprintf("Cam Look %d %d %d\n", (int)(cam_looking[0]*100), (int)(cam_looking[1]*100), (int)(cam_looking[2]*100));
	printTimes();	

	deg = cam_trajectory * 180.0 / PI;
	dprintf("cam_trajectory*10=%d\n", (int)deg*10);
	deg = targ_dir * 180.0 / PI;
	dprintf("targ_dir*10=%d\n", (int)deg*10);
	
	return;
}	

void home(void)
{
	cam_position[0] = 0;
	cam_position[2] = 0;
}



/* user function menu */
USER_FUNC ufuncs[] = {
	{"dump", &dump_values, "void"},
	{"home", &home, "void"},
	{""},
};

	
/* Viewing volume menu */

VLIST vvol_vl[] = {
"screen_dist(mm)", &screenDistanceMM, NP, NP, 0, ME_DEC,
"far_plane_distance", &farPlaneDistance, NP, NP, 0, ME_DEC, 
"Camera_height", &cam_height, NP, NP, 0, ME_DEC, 
NS,
};

char hm_vvol[] = "";

VLIST groundplane_vl[] = {
"Grid_length", &gp_length, NP, NP, 0, ME_DEC,
"Grid_width", &gp_width, NP, NP, 0, ME_DEC,
"Dots_per_grid", &gp_dots, NP, NP, 0, ME_DEC,
"Grid_pool_size", &gp_pool, NP, NP, 0, ME_DEC,
"Plane_color(R)", &gp_plane_color[0], 0, NP, 0, ME_DEC,
"Plane_color(G)", &gp_plane_color[1], 0, NP, 0, ME_DEC,
"Plane_color(B)", &gp_plane_color[2], 0, NP, 0, ME_DEC,
"Dot_color(R)", &gp_dot_color[0], 0, NP, 0, ME_DEC,
"Dot_color(G)", &gp_dot_color[1], 0, NP, 0, ME_DEC,
"Dot_color(B)", &gp_dot_color[2], 0, NP, 0, ME_DEC,
"Flags", &gp_flags, 0, NP, 0, ME_DEC,
"Dot_size", &gp_dotsize, NP, NP, 0, ME_FLOAT,
NS,
};

char hm_groundplane[] = "";


/* Target parameters menu */

VLIST target_vl[] = {
"Target_diam(degrees)", &targ_size, NP, NP, 0, ME_FLOAT,
"Target_color(R)", &targ_color[0], 0, NP, 0, ME_DEC,
"Target_color(G)", &targ_color[1], 0, NP, 0, ME_DEC,
"Target_color(B)", &targ_color[2], 0, NP, 0, ME_DEC,
"Target_distance", &targ_dist, 0, NP, 0, ME_FLOAT,
"Target_height(degrees)", &targ_h, 0, NP, 0, ME_FLOAT,
NS,
};

char hm_target[] = "";

/* Background color menu  */

VLIST background_vl[] = {
"Background_color(R)", &background_color[0], 0, NP, 0, ME_DEC,
"Background_color(G)", &background_color[1], 0, NP, 0, ME_DEC,
"Background_color(B)", &background_color[2], 0, NP, 0, ME_DEC,
NS,
};

char hm_background[] = "";




/* Joystick parameters menu -- testing only!!!! */

VLIST fakejoy_vl[] = {
"Enable_fake_joy", &fake_joy_enable, 0, NP, 0, ME_DEC,
"Fake_joy_value", &fake_joy_value, 0, NP, 0, ME_DEC,
"Fake_joy_debug", &fake_joy_debug, 0, NP, 0, ME_DEC,
"Fake_joy_timing", &fake_joy_timing, 0, NP, 0, ME_DEC,
"Jitter", &fake_joy_jitter, 0, NP, 0, ME_DEC,
"Jitter_steps_low", &fake_joy_jitter_steps_low, 0, NP, 0, ME_DEC,
"Jitter_low", &fake_joy_jitter_low, 0, NP, 0, ME_DEC,
"Jitter_steps_high", &fake_joy_jitter_steps_high, 0, NP, 0, ME_DEC,
"Jitter_high", &fake_joy_jitter_high, 0, NP, 0, ME_DEC,
NS,
};

char hm_fakejoy[] = "";

/* Reward parameters menu! */

VLIST reward_vl[] = {
"acquisition_time(ms)", &rew_acquisition_time, NP, NP, 0, ME_DEC,
"reward_wait_max(ms)", &rew_wait_max, NP, NP, 0, ME_DEC,
"reward_wait_min(ms)", &rew_wait_min, NP, NP, 0, ME_DEC,
"reward_ramp_time(ms)", &rew_ramp_time, NP, NP, 0, ME_DEC,
NS,
};

char hm_reward[] = "";

VLIST state_vl[] = {
"day_seed",		&seed, NP, NP, 0, ME_DEC,
"seconds_per_trial",	&f_secondspertrial, NP, NP, 0, ME_DEC,
"Target_offset_1(deg)", &targ_offset_degrees[0], 0, NP, 0, ME_DEC,
"Target_offset_2(deg)", &targ_offset_degrees[1], 0, NP, 0, ME_DEC,
"dwell(seconds)",	&f_secondsperdwell, NP, NP, 0, ME_DEC,
"Steering_max", &steer_max_degrees, NP, NP, 0, ME_FLOAT,
"Steering_zero_point", &steer_zero, NP, NP, 0, ME_DEC,
"Steering_dead_zone", &steer_dead, NP, NP, 0, ME_DEC,
"rew_win(deg)",	&rew_window_degrees, NP, NP, 0, ME_DEC,
"speed",	&speed, NP, NP, 0, ME_DEC,
"flow_on",	&flowflg, NP, NP, 0, ME_DEC,
"frame_rate(1/s)", &f_framespersecond, NP, NP, 0, ME_DEC,
NS,
};

/*
 * Help message.
 */
char hm_sv_vl[] = "";

MENU umenus[] = {
{"state_vars", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{"separator", NP}, 
{"viewing volume", &vvol_vl, NP, NP, 0, NP, hm_vvol},
{"ground plane", &groundplane_vl, NP, NP, 0, NP, hm_groundplane},
{"target", &target_vl, NP, NP, 0, NP, hm_target},
{"background", &background_vl, NP, NP, 0, NP, hm_background},
{"fakejoy", &fakejoy_vl, NP, NP, 0, NP, hm_fakejoy},
{"reward", &reward_vl, NP, NP, 0, NP, hm_reward},
{NS},
};



%%

id 304
restart rinitf
main_set {
status ON
begin	first:
		code HEADCD
		rl 0
		to sendping
	sendping:
		rl 5
		do render_send_ping()
		to waitping
	waitping:
		rl 10
		time 100
		to checkping
	checkping:
		rl 15
		to reset on 1 % render_check_for_ping
		to pauseping
	pauseping:
		rl 20
		time 1000
		to sendping
	reset:
		rl 25
		do render_reset()
		to p1
	p1:
		to p2 on +PSTOP & softswitch
		to initscene on -PSTOP & softswitch
	p2:	
		code PAUSECD
		to initscene on -PSTOP & softswitch
	initscene:
		rl 25
		do my_render_init()
		to inithandle
	inithandle:
		rl 30
		to init_targoff on 1 % my_check_for_handle
	init_targoff:
		do render_onoff(&targ_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT)
		to waitscene
	waitscene:
		rl 35
		to loop on 1 % render_check_for_went
	loop:
		to go
	go:
		code GOCD
		rl 40
		do my_trial_init()
		to trial_pause
	trial_pause:
		time 2000
		to trial_allon
	trial_allon:
		do my_trial_allon()
		to trial_targwait on 1 % render_check_for_went
	trial_targwait:
		time 500
		to beep
	beep:
		dio_on(BEEP)
		time 100
		to beepoff
	beepoff:
		dio_off(BEEP)
		to target_update
	target_update:
		do target_update()
		to target_on
	target_on:
		do render_frame(0)
		to reward_on on 1 % render_check_for_went
	reward_on:
		do apause(0)
		to update
	update:
		do my_update()
		to p3
	p3:
		to p4 on +PSTOP & softswitch
		to update_wait on -PSTOP & softswitch
	p4:	
		code PAUSECD
		do apause(1)
		to update_wait on -PSTOP & softswitch
	update_wait:
		do apause(0)
		to update on 1 % my_check_for_went
		to 	trial_done on 2 = f_wstatus
	trial_done:
		code TRIALENDCD
		do apause(1)
		to target_off
	target_off:
		do render_onoff(&targ_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT)
		to target_off_wait_for_went
	target_off_wait_for_went:
		to isi on 1 % render_check_for_went
	isi:
		code ISICD
		to p5
	p5:
		to p6 on +PSTOP & softswitch
		to loop on -PSTOP & softswitch
	p6:	
		code PAUSECD
		do apause(1)
		to loop on -PSTOP & softswitch
abort	list:
		first
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
		to rewinit on -PSTOP & softswitch
	p8:
		code PAUSECD
		to rewinit on -PSTOP & softswitch
	rewinit:
		do rewinit()
		to check
	check:
		to reward on 1 % check_reward_window
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
		to check on 1 ? rew_wait_counter
}

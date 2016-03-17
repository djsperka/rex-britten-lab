/*
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ldev_tst.h"	/* hardware device defines */
#include "lcode_tst.h"	/* Ecodes */

#include "make_msg.h"	/* all messaging information */
#include "transport.h"	     /* defines for communication to render */
#include "actions.h"
/*#include "stercodes.h"*/

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
 
 /* DJS - WARNING: I MADE UP THE VALUE FOR HEADCD -- needed a def for it, but none was in this code?!? */
#define ISICD	1900
#define GOCD	1901
#define HEADCD 1902
#define FRAMECD 2000

#define TRIAL_CD 2000
#define PARAM_BEGIN_CD 2001
#define PARAM_END_CD 2002



/**********************************************************************
 * Global variables.
 **********************************************************************/
 
static int seedflg = 0;
static int flowflg = 1;

unsigned char going=0;					/* Flag that indicates camera is moving. Set to 0 during pauses. */

float cam_position[3]; 							/* Current camera position */
float cam_start[3] = { 0, 0, 0 };		/* Initial eye position at start of trial */
int cam_height = 50;						/* Initial cam height - will be applied to cam_start (and used in updates) */
float cam_trajectory = 0;						/* trajectory - direction camera is moving - IN RADIANS*/
float cam_looking[3];
float cam_up[3] = { 0, 1, 0 };

/* globals for setting dot oscillation */
float targ_amplitude = 50;
float targ_period = 100;

/* target parameters. */	
float targ_size = 1.0;
int targ_color[3] = { 255, 0, 0 };
int targ_handle;
float targ_dir = 0;							/* radians */
float targ_h = 3;
float targ_dist = 50;
int targ_dir_degrees[2] = { 20, -20};
int targ_dir_current = 0;

/* Perspective/camera parameters */
int nearPlaneDistance = 1;
int farPlaneDistance = 1000;
int fovy = 90;

/* groundpland parameters */
int gp_length=1000;
int gp_width=1000;
int gp_dots=1000;
int gp_pool=500;
int gp_plane_color[3]= { 50, 50, 50 };
int gp_dot_color[3]= { 255, 255, 255 };
float gp_dotsize=3.0;


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
int rew_window_degrees = 10;
int reward_flag = 0;

/* Addresses and port */
char *local_addr="192.168.1.1";
char *remote_addr="192.168.1.2";
int remote_port=2000;

/* Fake joystick */
int fake_joy_enable = 0;
int fake_joy_value = 1024;
int fake_joy_debug = 0;

/* messaging structures. */
TargetStruct f_target;
MessageStruct msg;

/************************************************************************
 * Declaration of statelist variables.
 ************************************************************************/
long seed = 1111;
	
int rew_crit = 5;


/* drop_ecode_value
 * Drops a sequence of 4 ecodes which hold the value of a 4-byte int or float. 
 * The first code is a "normal" ecode, and it holds the value in 'code'. This value should be
 * used to represent a particular experimental parameter. 
 * The next three codes hold the low 12 bits, the next 12 bits, and the high 8 bits, respectively. 
 * we do it in this funny way because each ecode can only hold a 13 bit code value. Thus, to store
 * an int or float, each of which has 4 bytes, we need at least three codes. I chose 12/12/8 division because
 * its a little nicer conceptually.....
 */
 
void drop_ecode_value(unsigned short code, void *value)
{
	int *pv;
	EVENT ev;

	ev.e_code= code;
	ev.e_key= i_b->i_time;
	ldevent(&ev);

	pv = (int *)value;

	// now drop low 12 bits....
	ev.e_code = (*pv) & 0xfff;
	ldevent(&ev);
	
	// and the next 12 bits...
	ev.e_code = (*pv >> 12) & 0xfff;
	ldevent(&ev);
	
	// and the high 8 bits...
	ev.e_code = (*pv >> 24) & 0xff;
	ldevent(&ev);
	
	return;	
}


/* drop_ecode_byte
 * Drops a sequence of 2 ecodes which hold the value of a byte variable.
 * The first code is a "normal" ecode, and it holds the value in 'code'. This value should be
 * used to represent a particular experimental parameter. The next code will contain the value
 * of the byte as the code. 
 */
 
void drop_ecode_byte(unsigned short code, void *value)
{
	char *pv;
	EVENT ev;

	ev.e_code= code;
	ev.e_key= i_b->i_time;
	ldevent(&ev);

	pv = (char *)value;

	// now drop the byte...
	ev.e_code = (*pv) & 0xf;
	ldevent(&ev);

	return;	
}

/* drop_ecode
 * Drops an ecode. No associated value. This can be called from within an action to drop an ecode
 */
 
void drop_ecode(unsigned short code)
{
	char *pv;
	EVENT ev;

	ev.e_code= code;
	ev.e_key= i_b->i_time;
	ldevent(&ev);

	return;	
}


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
	status = init_tcpip(local_addr, remote_addr, remote_port);
	
	// In case we did a Reset States
	going = 0;
	cam_trajectory = 0;
	targ_dir_current = 0;
}


int port_hack()
{
	int status;
	dprintf("Close port....\n");
	close_tcpip();
	dprintf("Open port....\n");
	status = init_tcpip(local_addr, remote_addr, remote_port);
	dprintf("Port open status=%d\n", status);
	return 0;
}


int my_render_init()
{
	int status;

	// issue reset command
	render_init_world("pursuit");

	// Setup viewing volume -- this should be done prior to the groundplane init! */
	render_perspective(fovy, nearPlaneDistance, farPlaneDistance);

	// Initialize groundplane	
	render_groundplane(gp_length, gp_width, gp_dots, gp_pool, gp_plane_color, gp_dot_color, gp_dotsize);

	/* Setup init camera position */
	cam_position[0] = cam_start[0];
	cam_position[1] = (float)cam_height;
	cam_position[2] = cam_start[2];
	cam_looking[0] = -sin(cam_trajectory);
	cam_looking[1] = 0;
	cam_looking[2] = cos(cam_trajectory);
	render_camera(cam_position, cam_looking, cam_up);

	/* Configure target. We use a struct for this because it makes updates simpler.  */
	targ_dir = targ_dir_degrees[targ_dir_current] * PI/180.;
	f_target.xoffset = -sin(targ_dir);
	f_target.zoffset = cos(targ_dir);
	f_target.xsize = f_target.ysize = targ_size;
	f_target.d = targ_dist;
	f_target.h = targ_h;
	f_target.r = targ_color[0];
	f_target.g = targ_color[1];
	f_target.b = targ_color[2];
	render_target(&f_target);
	
	/* HANDLE message will be returned by render. We need the handle before we can turn the target off. */

	return 0;
}

int joystick_value()
{
	if (!fake_joy_enable) return joyh;
	else return fake_joy_value;
}

   
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

		if (fake_joy_debug)
		{
			dprintf("%d %d  %d %d\n", ijoyh, (int)(nval*1000.0f), (int)(delta * 1000000.0f), (int)(cam_trajectory * 1000.0f));
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
		targ_dir_current = 1-targ_dir_current;
		targ_delta = targ_dir_degrees[targ_dir_current] * PI / 180.;
		targ_dir = cam_trajectory + targ_delta;
		f_dwell_framecounter = f_dwell_maxframes;

//		dprintf("delta=%d traj=%d dir=%d\n", (int)(targ_delta*180/PI*100), (int)(cam_trajectory*180/PI*100), (int)(targ_dir*180/PI*100));
		f_target.xoffset = -sin(targ_dir);
		f_target.zoffset = cos(targ_dir);
		render_update(targ_handle, (void *)&f_target, sizeof(TargetStruct), 0);
	}
	else f_dwell_framecounter--;

	render_frame(0);
	going = 1;	/* in case we come back from a pause */

#if 0
	/* ecodes */
	drop_ecode(PARAM_BEGIN_CD);

	drop_ecode_value(JOYH_CD, &ijoyh);
	drop_ecode_value(CAM_TRAJECTORY_CD, &cam_trajectory);
	drop_ecode_value(TARG_DIR_CD, &targ_dir);

	drop_ecode(PARAM_END_CD);
#endif
	
	return 0;	
}


/***************************** my_trial_init() ********************************
 * 
 * Initialization for start of trial. initialize counters (like frame counter), and drop ecodes to 
 * save parameters relevant to the trial. 
 * 
 * Note that framecounter is set to 0 here. We're not using rex's escape condition ' 1 ? count' to decrement
 * the frame counter. Instead, the action 'my_check_for_went' increments the counter based on the 
 * number of frames since the last call. 
 */
int my_trial_init(void)
{
	f_trial_framecounter = 0;
	f_trial_maxframes = f_secondspertrial * f_framespersecond;
	f_dwell_maxframes = f_secondsperdwell * f_framespersecond;

	/* dwell counter counts backwards! */
	f_dwell_framecounter = f_dwell_maxframes;
	
	/* Position target at current target bearing value. 
	 * The current value of cam_trajectory was valid prior to the last update..... so when we're
	 * running (i.e. between trials) this value isn't strictly accurate, but its pretty darned close 
	 */	
	targ_dir = cam_trajectory + targ_dir_degrees[targ_dir_current] * PI / 180.;
	f_target.xoffset = -sin(targ_dir);
	f_target.zoffset = cos(targ_dir);
	render_update(targ_handle, (void *)&f_target, sizeof(TargetStruct), 0);

	
#if 0   
	/* drop ecodes */   
	drop_ecode(TRIAL_CD);
	drop_ecode(PARAM_BEGIN_CD);

	drop_ecode_value(SPEED_CD, &speed);
	drop_ecode_value(TRIAL_MAXFRAMES_CD, &f_trial_maxframes);
	drop_ecode_value(DWELL_MAXFRAMES_CD, &f_dwell_maxframes);
	drop_ecode_value(STEER_MAX_DEGREES_CD, &steer_max_degrees);
	drop_ecode_value(STEER_ZERO_CD, &steer_zero);
	drop_ecode_value(STEER_DEAD_CD, &steer_dead);
	drop_ecode_value(REW_WINDOW_DEGREES_CD, &rew_window_degrees);
	drop_ecode_value(GP_LENGTH_CD, &gp_length);
	drop_ecode_value(GP_WIDTH_CD, &gp_width);
	drop_ecode_value(GP_DOTS_CD, &gp_dots);
	drop_ecode_value(GP_POOL_CD, &gp_pool);
	drop_ecode_value(GP_PLANE_R_CD, &gp_plane_color[0]);
	drop_ecode_value(GP_PLANE_G_CD, &gp_plane_color[1]);
	drop_ecode_value(GP_PLANE_B_CD, &gp_plane_color[2]);
	drop_ecode_value(GP_DOT_R_CD, &gp_dot_color[0]);
	drop_ecode_value(GP_DOT_G_CD, &gp_dot_color[1]);
	drop_ecode_value(GP_DOT_B_CD, &gp_dot_color[2]);
	drop_ecode_value(GP_DOTSIZE_CD, &gp_dotsize);
	drop_ecode_value(FOVY_CD, &fovy);
	drop_ecode_value(NEAR_PLANE_CD, &nearPlaneDistance);
	drop_ecode_value(FAR_PLANE_CD, &farPlaneDistance);
	drop_ecode_value(TARGET_SIZE_CD, &targ_size);
	drop_ecode_value(TARGET_DIST_CD, &targ_dist);
	drop_ecode_value(TARGET_H_CD, &targ_h);
	drop_ecode_value(TARGET_R_CD, &targ_color[0]);
	drop_ecode_value(TARGET_G_CD, &targ_color[1]);
	drop_ecode_value(TARGET_B_CD, &targ_color[2]);

	drop_ecode(PARAM_END_CD);
#endif
   
   return 0;
}



/* my_check_for_handle *********************************/

int my_check_for_handle()
{
	int status = 0;
	status = render_check_for_handle(&targ_handle);
	if (status == 1)
	{
		dprintf("my_check_for_handle: targ_handle=%d\n", targ_handle);
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
			dprintf("%d (%d)\n", frames, f_went_cycles);
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


int my_check_for_went2()
{
	int frames=0;
	f_wstatus = render_check_for_went(&frames);
	if (f_wstatus == 1 && frames!=1) dprintf("Frames=%d\n", frames);
	return f_wstatus;
}

/***************************** check_reward_window() ********************************
 *
 * Checks reward "window" and sets flag
 */
int check_reward_window()
{
	float dot;
	
	// We want to compare the camera trajectory to the target bearing. If trajectory is within rew_window
	// then a reward is given. 

	reward_flag = 0;	
	if (going)
	{
		dot = cos(cam_trajectory)*cos(targ_dir) + sin(cam_trajectory)*sin(targ_dir);
		if (dot > cos(rew_window_degrees * PI/180))
		{
			reward_flag = 1;
			/*dprintf("reward! dot=%d window=%d max=%d\n", (int)(dot*100), rew_window_degrees, (int)(cos(rew_window_degrees * PI/180)*100));*/
		}
	}

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
	dprintf("Cam Pos %d %d %d\n", (int)(cam_position[0]*100), (int)(cam_position[1]*100), (int)(cam_position[2]*100));
	dprintf("Cam Look %d %d %d\n", (int)(cam_looking[0]*100), (int)(cam_looking[1]*100), (int)(cam_looking[2]*100));
	return;
}	


/* user function menu */
USER_FUNC ufuncs[] = {
	{"dump", &dump_values, "void"},
	{""},
};

	
/* Viewing volume menu */

VLIST vvol_vl[] = {
"Y field of view(deg)", &fovy, NP, NP, 0, ME_DEC,
"near plane distance", &nearPlaneDistance, NP, NP, 0, ME_DEC, 
"far plane distance", &farPlaneDistance, NP, NP, 0, ME_DEC, 
"Camera height", &cam_height, NP, NP, 0, ME_DEC, 
NS,
};

char hm_vvol[] = "";

VLIST groundplane_vl[] = {
"Grid length", &gp_length, NP, NP, 0, ME_DEC,
"Grid width", &gp_width, NP, NP, 0, ME_DEC,
"Dots per grid", &gp_dots, NP, NP, 0, ME_DEC,
"Grid pool size", &gp_pool, NP, NP, 0, ME_DEC,
"Plane color(R)", &gp_plane_color[0], 0, NP, 0, ME_DEC,
"Plane color(G)", &gp_plane_color[1], 0, NP, 0, ME_DEC,
"Plane color(B)", &gp_plane_color[2], 0, NP, 0, ME_DEC,
"Dot color(R)", &gp_dot_color[0], 0, NP, 0, ME_DEC,
"Dot color(G)", &gp_dot_color[1], 0, NP, 0, ME_DEC,
"Dot color(B)", &gp_dot_color[2], 0, NP, 0, ME_DEC,
"Dot size", &gp_dotsize, NP, NP, 0, ME_FLOAT,
NS,
};

char hm_groundplane[] = "";


/* Target parameters menu */

VLIST target_vl[] = {
"Target diameter", &targ_size, NP, NP, 0, ME_FLOAT,
"Target color(R)", &targ_color[0], 0, NP, 0, ME_DEC,
"Target color(G)", &targ_color[1], 0, NP, 0, ME_DEC,
"Target color(B)", &targ_color[2], 0, NP, 0, ME_DEC,
"Target distance", &targ_dist, 0, NP, 0, ME_FLOAT,
"Target height", &targ_h, 0, NP, 0, ME_FLOAT,
NS,
};

char hm_target[] = "";


/* Joystick parameters menu -- testing only!!!! */

VLIST fakejoy_vl[] = {
"Enable fake joy", &fake_joy_enable, 0, NP, 0, ME_DEC,
"Fake joy value", &fake_joy_value, 0, NP, 0, ME_DEC,
"Fake joy debug", &fake_joy_debug, 0, NP, 0, ME_DEC,
NS,
};

char hm_fakejoy[] = "";



VLIST state_vl[] = {
"day_seed",		&seed, NP, NP, 0, ME_DEC,
"seconds per trial",	&f_secondspertrial, NP, NP, 0, ME_DEC,
"Target bearing 1 (deg)", &targ_dir_degrees[0], 0, NP, 0, ME_DEC,
"Target bearing 2 (deg)", &targ_dir_degrees[1], 0, NP, 0, ME_DEC,
"dwell(seconds)",	&f_secondsperdwell, NP, NP, 0, ME_DEC,
"Steering max", &steer_max_degrees, NP, NP, 0, ME_FLOAT,
"Steering zero point", &steer_zero, NP, NP, 0, ME_DEC,
"Steering dead zone", &steer_dead, NP, NP, 0, ME_DEC,
"rew_win(deg)",	&rew_window_degrees, NP, NP, 0, ME_DEC,
"speed (a.u.)",	&speed, NP, NP, 0, ME_DEC,
"flow_on",	&flowflg, NP, NP, 0, ME_DEC,
"frame rate (1/s)", &f_framespersecond, NP, NP, 0, ME_DEC,
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
{"fakejoy", &fakejoy_vl, NP, NP, 0, NP, hm_fakejoy},
{NS},
};



%%

id 300
restart rinitf
main_set {
status ON
begin	first:
		code STARTCD
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
		time 3000
		to target_on
	target_on:
		do render_onoff(&targ_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT)
		to target_wait
	target_wait:
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
		do apause(1)
		to update_wait on -PSTOP & softswitch
	update_wait:
		do apause(0)
		to update on 1 % my_check_for_went
		to 	trial_done on 2 = f_wstatus
	trial_done:
		do apause(1)
		to target_off
	target_off:
		do render_onoff(&targ_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT)
		to target_off_wait_for_went
	target_off_wait_for_went:
		to isi on 1 % render_check_for_went
	isi:
		code ISICD
		/*do port_hack()*/
		to p5
	p5:
		to p6 on +PSTOP & softswitch
		to loop on -PSTOP & softswitch
	p6:	
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
		to check on -PSTOP & softswitch
	p8:
		code PAUSECD
		to check on -PSTOP & softswitch
	check:
		do check_reward_window()
		to reward on 1 = reward_flag
		to wait
	reward:
		dio_on(REW)
		time 35
		to rewoff
	rewoff:
		dio_off(REW)
		to mark
	mark:
		do score(1)
		to wait
	wait:
		time 700
		to check
}

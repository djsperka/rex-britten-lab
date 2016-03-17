/* $Id: template.d,v 1.1 2008/10/24 23:02:03 devel Exp $  */


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
 * Counters and control vars
 */
int f_frame_counter = 0;		/* Counts number of frames */
int f_went_counter = 0;			/* Number of wents received. 
								 * May differ from frame count if 
								 * there are dropped frames. */
int f_going = 0;				/* indicates whether we are going or paused. */
int f_went_cycles = 0;			/* counts how many times we check for went before receiving one */		 
int f_update_countdown = 1;		/* downcounter that rex will decrement every update */
int f_never = 0;				/* never set this to 1 */
int f_reward_wait_counter = 500;	/* ms to wait after a reward is given before another 
                                     * can be had. Should set this in menu, probably */

/*
 * Experimental parameters.
 * These are set in the dialogs and used to control the expt. 
 */

int f_seed = 0;
int f_nframes = 400;
int f_frames_per_second = 85;
char f_local_addr[32]="192.168.1.1";		/* Local ip address */
char f_remote_addr[32]="192.168.1.2";		/* IP address of render machine */
int f_remote_port=2000;						/* port used for render */
int f_background_color[3] = { 0, 0, 0 };	/* background color, rgb, [0,255] */

/* 
 * 3D parameters. These aren't really needed if you're just displaying 
 * 2D stimuli. 
 */


int f_screen_distance_MM = 290;			/* dist from eye to screen, in mm */
int f_far_plane_distance = 5000;		/* dist to far plane of viewing volume, in mm */
float f_cam_position[3]; 				/* Current camera/eye position */
float f_cam_start[3] = { 0, 0, 0 };		/* Initial eye position at start of trial */
int f_cam_height = 200;					/* Initial cam height - will be applied to cam_start (and used in updates) */
float f_cam_trajectory = 0;				/* trajectory - direction camera is moving - IN RADIANS*/
float f_cam_looking[3];					/* direction camera/eye is aimed */
float f_cam_up[3] = { 0, 1, 0 };		/* up direction for camera */




/* REX menu declarations */
VLIST state_vl[] = {
"rand_seed",		&f_seed, NP, NP, 0, ME_DEC,
"number_of_frames", &f_nframes, NP, NP, 0, ME_DEC,
"frame_rate(1/s)", &f_frames_per_second, NP, NP, 0, ME_DEC,
"local_ip", f_local_addr, NP, NP, 0, ME_STR,
"render_host_ip", f_remote_addr, NP, NP, 0, ME_STR,
"render_port", &f_remote_port, NP, NP, 0, ME_DEC,
NS,
};

char hm_sv_vl[] = "";

/* Background color menu  */

VLIST background_vl[] = {
"Background_color(R)", &f_background_color[0], 0, NP, 0, ME_DEC,
"Background_color(G)", &f_background_color[1], 0, NP, 0, ME_DEC,
"Background_color(B)", &f_background_color[2], 0, NP, 0, ME_DEC,
NS,
};

char hm_background[] = "";

MENU umenus[] = {
{"Main", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{"separator", NP}, 
{"background", &background_vl, NP, NP, 0, NP, hm_background}, 
{NS},
};



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
 * 
 * Note also that the placement of certain things in this function (like
 * the seeding of the random number generator) assumes that the operator
 * does a 'reset states' after entering parameters. Its not an obvious 
 * workflow, because the paradigm must have a built-in pause state in order
 * for there to be an opportunity for entering things in the dialogs. REX
 * only has this hook function that we can be assured is called when we 
 * truly want initialization to happen. It will happen more than once, in 
 * that this function is called when the clock is started; just make sure that
 * any functions called here can be called more than once. 
 *                                                                       *
 ************************************************************************/
void init_steering(void)
{
	int status=0;

	// initialize tcpip - must only do this OR gpib - NOT BOTH!!!!
	status = init_tcpip(f_local_addr, f_remote_addr, f_remote_port, 0);

	f_going = 0;

	// initialize pixel conversion
	if (initialize_pixel_conversion(x_dimension_mm, y_dimension_mm, x_resolution, y_resolution, f_screen_distance_MM))
	{
		dprintf("ERROR in initialize_pixel_conversion: \n"
				"x,y dimensions=(%d, %d)\n"
				"x,y resolution=(%d, %d)\n"
				"screen distance=%d\n", (int)x_dimension_mm, (int)y_dimension_mm, (int)x_resolution, (int)y_resolution, (int)f_screen_distance_MM);
	}

	// seed random number generator
	ivunif(f_seed, 1);
	
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
	f_frame_counter = 0;
	f_went_counter = 0; 
	f_update_countdown = f_nframes;

	dprintf("my_render_init()\n");
	
	render_bgcolor(f_background_color[0], f_background_color[1], f_background_color[2]);

/*
 * This is setup for doing 3D work.
 */

//	// Setup viewing volume -- this should be done prior to the groundplane init! */
//
//	fovy = 2*atan2f(y_dimension_mm/2.0, f_screen_distance_MM); 	
//	render_perspective(fovy, f_screen_distance_MM, f_far_plane_distance);

//	/* Setup init camera position */
//
//	f_cam_position[0] = 0;
//	f_cam_position[1] = (float)f_cam_height;
//	f_cam_position[2] = 0;
//	f_cam_looking[0] = -sin(f_cam_trajectory);
//	f_cam_looking[1] = 0;
//	f_cam_looking[2] = cos(f_cam_trajectory);
//	render_camera(f_cam_position, f_cam_looking, f_cam_up);

	return status;
}


/* 
 * my_check_for_handle *********************************/




int my_check_for_handle()
{
	int status = 0;
	int handle = 0;
	if (render_check_for_handle(&handle))
	{
		/*
		 * The handle value is in the var 'handle'
		 */
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
	dprintf("my_exp_init()\n");

	render_frame(0);
	return 0;
}

int my_trial_init()
{
	f_frame_counter = 0;
	f_went_counter = 0;
	return 0;
}

int my_trial_done()
{
	reward_pause(1);
	return 0;
}
   
int my_update()
{
	int status=0;

	dprintf("f_update_countdown=%d\n", f_update_countdown);
	render_frame(0);

	f_update_countdown--;
	return status;	
}


/* my_check_for_went *********************************/

int my_check_for_went()
{
	int status;
	int frames = 0;
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
		f_frame_counter += frames;
		f_went_counter += 1;
				
		// TEST - if frames were missed, dprint it and the cycles...
		if (frames > 1)
		{
			dprintf("Missed %d frames (%d check_went cycles)\n", frames, f_went_cycles);
		}
		f_went_cycles = 0;

	}
	return status;
}


/*
 * reward_init - initialize any reward stuff here. 
 */

int reward_init()
{
	return 0;
}


/*
 * Checks reward "window". Should return 1 for reward, 0 otherwise. 
 */

int reward_check()
{
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
		to exp_init
		/*to exp_init on 1 % my_check_for_handle*/
	exp_init:
		do my_exp_init()
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
		to all_done on 0 = f_update_countdown
		to update on 1 % my_check_for_went
	all_done:
		do reward_pause(1)
		to first on 1 = f_never
abort	list:
		all_done
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
		to reward on 1 % reward_check
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

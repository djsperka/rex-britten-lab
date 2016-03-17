/* $Id: steertune.d,v 1.5 2015/05/12 20:53:48 devel Exp $  */


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
#include "randomtrialgenerator.h" /* generate random trials in blocks */

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
int f_reward_flag = 0;
int f_went_status = 0;
int f_trials_success = 0;
int f_trials_failed = 0;

/* Eye window, ecodes, bcode channels */
#define WIND0	    0

#define EXPTCD 1500
#define TRIALCD 1502
#define UPDCD	1503
#define FLOWCD 1520
#define SUCCESSCD 1521
#define FAILCD 1522
#define MISSEDCD	1505	// missed frame code
#define TRIALENDCD 1506
#define WENTCD		1509

/* per-trial channels */
#define CH_HEADING				1
#define CH_BLIPV				2
#define CH_INDEX				3

/* per-update channels */
#define CH_TRAJ					51
#define CH_LOOK 				52

/* per-experiment channels */
#define CH_NREPEAT				60
#define CH_NSTEPS				61
#define CH_DURATION				62
#define	CH_BLIP_DURATION		63
#define CH_BLIP_OFFSET			64
#define CH_BLIP_MAX_ANG_VELOCITY_DEG_PER_SEC		65
#define CH_SPEED				66
#define CH_SCREEN_DISTANCE_MM	67
#define CH_FAR_PLANE_DISTANCE	68
#define CH_CAMERA_HEIGHT		69
#define CH_GP_LENGTH			70
#define CH_GP_WIDTH				71
#define CH_DOTS_PER_GRID		72
#define CH_GRID_POOL_SIZE		73
#define CH_PLANE_R				74
#define CH_PLANE_G				75
#define CH_PLANE_B				76
#define CH_DOT_R				77
#define CH_DOT_G				78
#define CH_DOT_B				79
#define CH_GRID_FLAGS			80
#define CH_DOT_SIZE				81
#define CH_BACKGROUND_R			82
#define CH_BACKGROUND_G			83
#define CH_BACKGROUND_B			84
#define CH_FIXPT_X				85
#define CH_FIXPT_Y				86
#define CH_FIXPT_DIAMETER		87
#define CH_FIXPT_WINDOW			88
#define CH_FIXPT_ACQTIME		89
#define CH_FIXPT_ACQTIMEOUT		90
#define CH_FIXPT_FIXTIME		91
#define CH_BREAK_FIX_TIMEOUT	92
#define CH_FIXPT_R				93
#define CH_FIXPT_G				94
#define CH_FIXPT_B				95
#define CH_FRAMES_PER_SECOND	96


/*
 * Trial types
 */
 
typedef struct trial_type_struct 
{
	float heading;
	int duration_frames;
	int blip_start_frame;
	int blip_duration_frames;
	int speed;
	float blip_ang_velocity;	/* radians per frame */
} TrialType;

typedef struct trial_control
{
	int frame_count;
	TrialType *ptype;
	int index;
} TrialControl;
	
int f_ntrialtypes = 0;
TrialType *f_trialtypes = NULL;
TrialControl f_trialcontrol = {0, NULL};
RTGenerator *f_prtgen = NULL;
int f_all_trials_done = 0;

/* 
 * graphics structs and handles 
 */

MessageStruct f_msg;
DotStruct f_fixpt;
int f_handle_count = 0;
int f_gp_handle;
int f_fixpt_handle;


/*
 * Experimental parameters.
 * These are set in the dialogs and used to control the expt. 
 */

int f_seed = 0;
int f_speed = 10;
int f_nframes = 400;
int f_frames_per_second = 85;
char f_local_addr[32]="192.168.1.1";		/* Local ip address */
char f_remote_addr[32]="192.168.1.2";		/* IP address of render machine */
int f_remote_port=2000;						/* port used for render */
int f_background_color[3] = { 0, 0, 0 };	/* background color, rgb, [0,255] */
int f_trial_duration = 1000;				/* How long flow lasts in a trial */ 
int f_nrepeats = 6;						/* Number of complete trials needed of each type. */
int f_blocksize = 2;						/* block size for trial generator */
float f_max_heading = 10;					/* max heading in degrees. */
int f_nsteps = 5;							/* number of steps from 0-max */ 
int f_linear_steps=1;						/* 1 means steps are linear, 0 means steps are log */

/* 
 * 3D parameters. These aren't really needed if you're just displaying 
 * 2D stimuli. 
 */


int f_screen_distance_MM = 280;			/* dist from eye to screen, in mm */
int f_far_plane_distance = 5000;		/* dist to far plane of viewing volume, in mm */
float f_cam_position[3]; 				/* Current camera/eye position */
float f_cam_start[3] = { 0, 0, 0 };		/* Initial eye position at start of trial */
int f_cam_height = 200;					/* Initial cam height - will be applied to cam_start (and used in updates) */
float f_cam_trajectory = 0;				/* trajectory - direction camera is moving - IN RADIANS*/
float f_cam_looking[3];					/* direction camera/eye is aimed */
float f_cam_up[3] = { 0, 1, 0 };		/* up direction for camera */

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
int f_acq_time = 4000;
int f_fix_time = 150;
int f_acq_timeout = 500;
int f_break_fix_timeout = 500;
int f_fixpt_color[3] = { 0, 0, 255 };

/* blips */
int f_blip_duration_ms=0;
int f_blip_offset_ms=0;
float f_blip_max_ang_velocity_deg_per_sec = 0;



/* REX menu declarations */
VLIST state_vl[] = {
"rand_seed",		&f_seed, NP, NP, 0, ME_DEC,
"speed", &f_speed, NP, NP, 0, ME_DEC, 
"trial_duration(ms)", &f_trial_duration, NP, NP, 0, ME_DEC, 
"number_of_repeats", &f_nrepeats, NP, NP, 0, ME_DEC,
"blocksize", &f_blocksize, NP, NP, 0, ME_DEC, 
"max_heading(deg)", &f_max_heading, NP, NP, 0, ME_FLOAT,
"steps(not_incl_0)", &f_nsteps, NP, NP, 0, ME_DEC,
"linear=1,0=log", &f_linear_steps, NP, NP, 0, ME_DEC,  
"blip_duration(ms)", &f_blip_duration_ms, 0, NP, 0, ME_DEC,
"offset(ms)", &f_blip_offset_ms, 0, NP, 0, ME_DEC,
"max_vel(deg/sec)", &f_blip_max_ang_velocity_deg_per_sec, 0, NP, 0, ME_FLOAT,
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

/* Viewing volume menu */

VLIST vvol_vl[] = {
"screen_dist(mm)", &f_screen_distance_MM, NP, NP, 0, ME_DEC,
"far_plane_distance", &f_far_plane_distance, NP, NP, 0, ME_DEC, 
"Camera_height", &f_cam_height, NP, NP, 0, ME_DEC, 
NS,
};

char hm_vvol[] = "";


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
"brkfix_timeout(ms)", &f_break_fix_timeout, NP, NP, 0, ME_DEC,
NS,
};

char hm_fixpt[] = "";




MENU umenus[] = {
{"Main", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{"separator", NP}, 
{"background", &background_vl, NP, NP, 0, NP, hm_background}, 
{"fixation", &fixpt_vl, NP, NP, 0, NP, hm_fixpt}, 
{"viewing volume", &vvol_vl, NP, NP, 0, NP, hm_vvol},
{"separator", NP}, 
{"groundplane", &groundplane_vl, NP, NP, 0, NP, hm_groundplane},
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
	float fovy;

	// zero out handles and initialize counters.

	f_gp_handle = f_fixpt_handle = 0;
	f_handle_count = 0;
	f_frame_counter = 0;
	f_went_counter = 0; 
	f_update_countdown = f_nframes;

	dprintf("my_render_init()\n");
	
	render_bgcolor(f_background_color[0], f_background_color[1], f_background_color[2]);


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
		switch (f_handle_count)
		{
			case 0:
			f_gp_handle = handle;
			f_handle_count = 1;
			break;
			case 1:
			f_fixpt_handle = handle;
			f_handle_count = 2;
			status = 1;
			break;
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
	float stepsize;
	int i;
	int count = 0;
	int duration_frames;
	int blip_duration_frames;
	int blip_offset;
	
	/* init counters */
	f_trials_success = 0;
	f_trials_failed = 0;
	f_all_trials_done = 0;

	/* set state times */
	set_times("fixpt_acq", (long)f_acq_time, -1);
	set_times("fixpt_acq_fail_pause", (long)f_acq_timeout, -1);
	set_times("fixpt_hold", (long)f_fix_time, -1);
	set_times("break_fix_timeout", (long)f_break_fix_timeout, -1);

	/* compute trial duration etc. */	
	duration_frames = (int)((float)f_trial_duration / 1000.0f * f_frames_per_second);
	blip_duration_frames = (int)((float)f_blip_duration_ms / 1000.0f * f_frames_per_second);
	blip_offset = (int)((float)f_blip_offset_ms / 1000.0f * f_frames_per_second);

	/* 
	 * Prepare trial types
	 */
	if (f_trialtypes)
	{
		free(f_trialtypes);
		f_trialtypes = NULL;
	}
	f_ntrialtypes = 2*f_nsteps + 1;
	if (blip_duration_frames > 0)
	{
		f_ntrialtypes *= 3;
	}
	f_trialtypes = (TrialType *)calloc(f_ntrialtypes, sizeof(TrialType));
	memset(f_trialtypes, 0, f_ntrialtypes * sizeof(TrialType));

	dprintf("Preparing trial types.\nThere are %d headings.\nBlip duration %d frames.Total trial types %d\n", 2*f_nsteps + 1, blip_duration_frames, f_ntrialtypes);

	
	if (f_linear_steps == 0)
	{
		stepsize = log(f_max_heading)/(float)f_nsteps;
	}
	else
	{
		stepsize = f_max_heading/(float)f_nsteps;
	}

	for (i=-f_nsteps; i<=f_nsteps; i++)
	{
		float heading;
		int sgn;

		if (i==0) 
		{
			heading = 0;
		}
		else
		{
			if (0 == f_linear_steps)
			{
				heading = i/abs(i) * exp(abs(i)*stepsize);
			}
			else
			{
				heading = i * stepsize;
			}
		}

		/* non-blip type */
		f_trialtypes[count].heading = heading;
		f_trialtypes[count].duration_frames = duration_frames;
		f_trialtypes[count].speed = f_speed;
		f_trialtypes[count].blip_duration_frames = 0;
		f_trialtypes[count].blip_start_frame = 0;
		f_trialtypes[count].blip_ang_velocity = 0;
		count++;

		/* get the blip types here */		
		if (blip_duration_frames > 0)
		{
			/* positive blip */
			f_trialtypes[count].heading = heading;
			f_trialtypes[count].duration_frames = duration_frames;
			f_trialtypes[count].speed = f_speed;
			f_trialtypes[count].blip_duration_frames = blip_duration_frames;
			f_trialtypes[count].blip_start_frame = blip_offset;
			f_trialtypes[count].blip_ang_velocity = f_blip_max_ang_velocity_deg_per_sec;
			count++;

			/* negative blip */
			f_trialtypes[count].heading = heading;
			f_trialtypes[count].duration_frames = duration_frames;
			f_trialtypes[count].speed = f_speed;
			f_trialtypes[count].blip_duration_frames = blip_duration_frames;
			f_trialtypes[count].blip_start_frame = blip_offset;
			f_trialtypes[count].blip_ang_velocity = -f_blip_max_ang_velocity_deg_per_sec;
			count++;
		}
	}

	if (count != f_ntrialtypes)
	{
		dprintf("WARNING!!! COUNT!= F_NTRIALTYPES!!!\n");
	}
	
	/* 
	 * random trial generator
	 */
	 
	if (f_prtgen)
	{
		rtg_destroy(f_prtgen);
		f_prtgen = NULL;
	}
	f_prtgen = rtg_create(f_ntrialtypes, f_nrepeats, f_blocksize);
	
	
	/* ecodes and bcodes for parameters governing this expt */
	bcode_int(CH_FRAMES_PER_SECOND, f_frames_per_second);
	bcode_int(CH_SPEED, f_speed);
	bcode_int(CH_DURATION, f_trial_duration);
	bcode_int(CH_BLIP_DURATION, f_blip_duration_ms);
	bcode_float(CH_BLIP_MAX_ANG_VELOCITY_DEG_PER_SEC, f_blip_max_ang_velocity_deg_per_sec);
	bcode_int(CH_SCREEN_DISTANCE_MM, f_screen_distance_MM);
	bcode_int(CH_FAR_PLANE_DISTANCE, f_far_plane_distance);
	bcode_int(CH_CAMERA_HEIGHT, f_cam_height);
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
	bcode_float(CH_FIXPT_X, f_fixpt_x);
	bcode_float(CH_FIXPT_Y, f_fixpt_y);
	bcode_float(CH_FIXPT_DIAMETER, f_fixpt_diameter);
	bcode_float(CH_FIXPT_WINDOW, f_fixpt_window);
	bcode_int(CH_FIXPT_ACQTIME, f_acq_time);
	bcode_int(CH_FIXPT_ACQTIMEOUT, f_acq_timeout);
	bcode_int(CH_FIXPT_FIXTIME, f_fix_time);
	bcode_int(CH_BREAK_FIX_TIMEOUT, f_break_fix_timeout);
	bcode_int(CH_FIXPT_R, f_fixpt_color[0]);
	bcode_int(CH_FIXPT_G, f_fixpt_color[1]);
	bcode_int(CH_FIXPT_B, f_fixpt_color[2]);

	
	
	/*
	 * Turn everything off and render. 
	 */
	 
	alloff(1);
	return 0;
}

int my_trial_init()
{
	int itrial = f_prtgen->next(f_prtgen);
	f_frame_counter = 0;
	f_went_counter = 0;

	if (itrial < 0)
	{
		f_all_trials_done = 1;
	}
	else
	{	
		f_trialcontrol.frame_count = 0;
		f_trialcontrol.ptype = &f_trialtypes[itrial];
		f_trialcontrol.index = itrial;

		f_cam_position[0] = f_cam_position[2] = 0;
		f_cam_position[1] = (float)f_cam_height;
		f_cam_looking[0] = f_cam_looking[1] = 0;
		f_cam_looking[2] = 1;
		f_cam_trajectory = f_trialcontrol.ptype->heading;	/* in degrees */		
		render_camera(f_cam_position, f_cam_looking, f_cam_up);
		
		ecode(TRIALCD);
		bcode_float(CH_HEADING, f_trialcontrol.ptype->heading);
		bcode_float(CH_BLIPV, f_trialcontrol.ptype->blip_ang_velocity);
		bcode_int(CH_INDEX, f_trialcontrol.index);
		
		dprintf("Index\t%d\tHeading\t%d\tBlipV\t%d : ", f_trialcontrol.index, (int)(f_trialcontrol.ptype->heading*10), (int)(f_trialcontrol.ptype->blip_ang_velocity*10));
	}
	return 0;
}

int my_trial_done(int success)
{
	if (success)
	{
		f_reward_flag = 1;
		f_prtgen->mark(f_prtgen, f_trialcontrol.index);
		f_trials_success++;
		dprintf("SUCCESS (%d/%d)\n", f_trials_success, f_ntrialtypes*f_nrepeats);
		ecode(SUCCESSCD);
	}
	else
	{
		f_trials_failed++;
		f_went_cycles = 0;
		dprintf("FAILED (%d)\n", f_trials_failed);
		ecode(FAILCD);
	}
		
	
	alloff(1);
	
	return 0;
}
   
int my_update()
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
	 
	if (f_trialcontrol.ptype->blip_duration_frames > 0 &&
		f_went_counter >= f_trialcontrol.ptype->blip_start_frame &&
		f_went_counter <= f_trialcontrol.ptype->blip_start_frame+f_trialcontrol.ptype->blip_duration_frames)
	{
		float frac, angle;

		/*
		 * We're in a blip. Adjust camera trajectory. Remember that camera 
		 * trajectory is kept in degrees, so 'angle' below is computed in 
		 * degrees per frame. 
		 */

		frac = (float)(f_went_counter+1-f_trialcontrol.ptype->blip_start_frame)/(float)(f_trialcontrol.ptype->blip_duration_frames);
		angle = f_trialcontrol.ptype->blip_ang_velocity * sin(frac * M_PI) / f_frames_per_second;
		/*
		dprintf("traj before %d\n", (int)(f_cam_trajectory*1000));
		*/
		f_cam_trajectory += angle;
		/*
		dprintf("blip wc %d sf %d dur %d frac %d ang %d adj %d traj %d\n", 
			f_went_counter, 
			f_trialcontrol.ptype->blip_start_frame, 
			f_trialcontrol.ptype->blip_duration_frames,
			(int)(frac*1000), 
			(int)(f_trialcontrol.ptype->blip_ang_velocity*10), 
			(int)(angle*1000), 
			(int)(f_cam_trajectory*1000));
			*/
	}
	
	f_cam_looking[0] = sin((f_cam_trajectory - f_trialcontrol.ptype->heading) * M_PI/180.0f);
	f_cam_looking[1] = 0;
	f_cam_looking[2] = cos((f_cam_trajectory - f_trialcontrol.ptype->heading) * M_PI/180.0f);

	f_cam_position[0] += f_speed * sin(f_cam_trajectory * M_PI/180.0f);
	f_cam_position[2] += f_speed * cos(f_cam_trajectory * M_PI/180.0f);
	render_camera(f_cam_position, f_cam_looking, f_cam_up);
	render_frame(0);

	ecode(UPDCD);
	bcode_float(CH_TRAJ, f_cam_trajectory);

	return status;	
}

/* 
 * alloff()
 * 
 * Turns off groundplane and target. If arg is 1, issue a frame command as well. 
 * 
 */
 
int alloff(int iframe)
{
	render_onoff(&f_gp_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_NO_FRAME);
	if (iframe) render_frame(0);
	return 0;
}


/* 
 * my_check_for_went
 * 
 * Called as a state escape.function. Returns negative value on error, 0 if
 * no messages are available (note: makes a non-blocking call), or 1 if a went was 
 * received.   
 * 
 * There are three globals that are affected by this function:
 * f_went_counter : incremented when a WENT is actually received in this call
 * f_frame_counter: when a went is received, tallies the number of frames
 *                  since the last went. 
 * f_went_cycles  : count number of times this function called between wents. 
 */

int my_check_for_went()
{
	int status;
	int frames = 0;
	status = render_check_for_went(&frames);
	if (status < 0)
	{
		// ERROR! not clear what to do ...
		dprintf("ERROR in render_check_for_went!\n");
		status = -1;
	}
	else if (status == 0)
	{
		f_went_cycles++;
	}
	else if (status == 1)
	{
		// went was found. 'frames' has frame count. 
		f_frame_counter += frames;
		f_went_counter += 1;
				
		// TEST - if frames were missed, dprint it and the cycles...
		if (frames > 1 && f_went_counter > 1)
		{
			dprintf("Missed %d frames (%d check_went cycles, wc=%d)\n", frames, f_went_cycles, f_went_counter);
			ecode(MISSEDCD);
		}
		else
		{
			ecode(WENTCD);
		}
		f_went_cycles = 0;

		/* If f_went_counter is now equal to the trial duration, set the 
		 * return value to 2 -- state flow should use that to escape the 
		 * update loop and end the trial.
		 */ 
		 
		if (f_went_counter >= f_trialcontrol.ptype->duration_frames)
		{
			status = 2;
		}

	}
	f_went_status = status;
	return status;
}


/*
 * reward_init - initialize any reward stuff here. 
 */

int reward_init()
{
	f_reward_flag = 0;
	return 0;
}


/*
 * Checks reward "window". Should return 1 for reward, 0 otherwise. 
 */

int reward_check()
{
	return f_reward_flag;
}


int reward_pause(int isPaused)
{
	if (isPaused) f_going = 0;
	else f_going = 1;
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


int flow_init()
{
	f_went_counter = 0;
	return 0;
}


/* REX state set starts here */

%%

id 450
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
		do my_exp_init()
		to new_trial on 1 % render_check_for_went
	new_trial:
		to new_trial_pause on +PSTOP & softswitch
		to new_trial_init on -PSTOP & softswitch
	new_trial_pause:	
		code PAUSECD
		to new_trial_init on -PSTOP & softswitch
	new_trial_init:
		do my_trial_init()
		to all_done on 1 = f_all_trials_done
		to fixpt_on
	fixpt_on:
		do render_onoff(&f_fixpt_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT);
		to fixpt_window on 1 % render_check_for_went
	fixpt_window:
		do my_fixpt_window()
		to fixpt_acq
	fixpt_acq:
		time 4000
		to fixpt_hold on -WD0_XY & eyeflag
		to fixpt_acq_fail
	fixpt_acq_fail:
		do render_onoff(&f_fixpt_handle, HANDLE_OFF, ONOFF_FRAME_NOWAIT);
		to fixpt_acq_fail_pause on 1 % render_check_for_went
	fixpt_acq_fail_pause:
		time 500
		do wd_cntrl(WIND0, WD_OFF);
		to fixpt_on
	fixpt_hold:
		time 150
		to fixpt_noise on +WD0_XY & eyeflag
		to gp_on
	fixpt_noise:
		do render_onoff(&f_fixpt_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT);
		to fixpt_acq_fail_pause on 1 % render_check_for_went
	gp_on:
		code FIXCD
		do render_onoff(&f_gp_handle, HANDLE_ON, ONOFF_FRAME_NOWAIT);
		to gp_wait on 1 % render_check_for_went
	break_fix:
		code BREAKFIXCD
		do alloff(1)
		to break_fix_timeout on 1 % render_check_for_went
	break_fix_timeout:
		do wd_cntrl(WIND0, WD_OFF);
		time 500
		to fixpt_on
	gp_wait:
		time 500
		to break_fix on +WD0_XY & eyeflag
		to flow_on
	flow_on:
		code FLOWCD
		do flow_init()
		to update
	update:
		do my_update()
		to update_wait
	update_wait:
		do reward_pause(0)
		to trial_fail on +WD0_XY & eyeflag
		to update on 1 % my_check_for_went
		to success on 2 = f_went_status
	success:
		do my_trial_done(1)
		to trial_success_pause on 1 % render_check_for_went
	trial_success_pause:
		time 500
		to new_trial 
	trial_fail:
		to trial_fail_clear on 1 % render_check_for_went
	trial_fail_clear:
		time 500
		do my_trial_done(0)
		to new_trial on 1 % render_check_for_went
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
		time 50
		to rewoff
	rewoff:
		dio_off(REW)
		to reward_init
}

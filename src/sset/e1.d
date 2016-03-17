/* 
 * $Id: e1.d,v 1.10 2015/05/12 20:52:48 devel Exp $ 
 */

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
#include "randomtrialgenerator.h"
#include "eyepos.h"

/* 
 * channels for bcodes
 */
 
#define CH_FIXPT_X 1
#define CH_FIXPT_Y 2


/* 
 * bit masks for verbosity 
 */
 
#define DEBUG_BIT 0x1
#define DEBUG_REWARD_BIT 0x2

/* 
 * reward flag. This is set to deliver a reward. The actual 
 * reward size is set by the "reward_preset" and "reward_random"
 * values in the main menu. The define's are for the state action
 * reward() which delivers the reward itself. 
 */
 
int f_reward_flag=0;
#define REWARD_ON	7501
#define REWARD_OFF	7502

/*
 * Eye window def
 */
 
#define EYE_WINDOW_FIXPT 0

/* 
 * args to my_trial_done()
 */
 
#define TRIAL_FAIL_ACQ			7101
#define TRIAL_FAIL_NOISE		7102
#define TRIAL_SUCCESS			7103
#define TRIAL_FAIL_PAUSE		7104
#define TRIAL_FAIL_ABORT		7105

int f_handle_count = 0;			/* counter for my_check_for_handle */
int f_fixpt_handle = 0;			/* graphic handle for fixpt */
int f_all_trials_done = 0;		/* set to 1 in my_trial_init when all trials are done */
int f_abort_exp = 0;			/* set to 1 when my_exp_init decides things just won't work out */
RTGenerator *f_prtgen = NULL;
int f_trial_tallies[4] = {0};
int f_seed_used = 0;			/* has random number generator been seeded? */
DotStruct f_fixpt;
int f_never = 0;
float *f_pfixpt_x = NULL;
float *f_pfixpt_y = NULL;
int f_nfixpt = 0;
int f_calibration_current_index = 0;
int f_calibration_current_index_count = 0;
int f_current_condition_index = 0;

/* 
 * main menu
 */

int f_seed = 0;					/* random number seed (menu item) */
int f_ntrials = 4;				/* number of trials per condition */
int f_nblocksize=2;				/* block size for trials */
int f_screen_distance_MM = 280;			/* screen distance in mm */
int f_calibration = 1;
float f_calibration_offset = 10.0;
int f_nhorizontal_steps = 1;
float f_max_horizontal_offset = 20;
int f_nvertical_steps = 1;
float f_max_vertical_offset = 20;
int f_reward_preset = 20;
int f_reward_random = 20;
int f_verbose = 0;

VLIST state_vl[] = {
	"day_seed",		&f_seed, NP, NP, 0, ME_DEC,
	"#_of_trials/condition", &f_ntrials, NP, NP, 0, ME_DEC,
	"block_size", &f_nblocksize, NP, NP, 0, ME_DEC,
	"calib?(0,H=1,V=2,3)", &f_calibration, NP, NP, 0, ME_DEC,
	"calib_offset", &f_calibration_offset, NP, NP, 0, ME_FLOAT,
	"#_horiz_steps", &f_nhorizontal_steps, NP, NP, 0, ME_DEC,
	"max_horiz_offset", &f_max_horizontal_offset, NP, NP, 0, ME_FLOAT,
	"#_vertical_steps", &f_nvertical_steps, NP, NP, 0, ME_DEC,
	"max_vertical_offset", &f_max_vertical_offset, NP, NP, 0, ME_FLOAT,
	"screen_dist(mm)", &f_screen_distance_MM, NP, NP, 0, ME_DEC,
	"reward_preset", &f_reward_preset, NP, NP, 0, ME_DEC,
	"reward_random", &f_reward_random, NP, NP, 0, ME_DEC,
	"verbose", &f_verbose, NP, NP, 0, ME_DEC,
	NS,
};

char hm_sv_vl[] = "Calib?\n1: horizontal calibration, using calib_offset\n\n2:vertical calibration, using calib_offset\n\n"
					"0: Random trials, fix positions spaced using horiz/vert offset and #steps.\n\n"
					"3: Random trials, fix positions specified on fix_positions menu.\n\n"
					"0,3 use #trials/condition and blocksize.";

#define MAX_FIXPT 10
int f_nfixpt_xy = 0;	/* Not to be confused with f_nfixpt */
float f_fixpt_x_degrees[MAX_FIXPT] = {0};
float f_fixpt_y_degrees[MAX_FIXPT] = {0};
char f_fixpt_pos_file[256] = "";	/* Positions for fixation point found here */

/*RTGenerator *f_prtgen = NULL;*/

/* Fixation point positions menu */

VLIST fixpt_pos_vl[] = {
"Fixpt_positions_file", &f_fixpt_pos_file, NP, NP, 0, ME_STR,
"Number_of_fix_pts", &f_nfixpt_xy, NP, NP, 0, ME_DEC,
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

char hm_fixpt_pos[] ="If \"Fixpt_positions_file\" is empty, then we use \"Number_of_fix_pts\" and Fixpt_XY_#.\n\n If \"Fixpt_positions_file\" is NOT empty, then it should be a filename containing fixation point positions, and those will be used when calib = 3.";


/*
 * Fixation point/background menu
 */
 
float f_fixpt_diameter = 1;				/* diameter of fixpt (degrees) */
float f_fixpt_window = 2;				/* size of fixation point window */
int f_fixpt_color[3] = {255, 0, 0};		/* fixpt color [0,255] */
int f_background_color[3] = {0};		/* background color [0,255] */

VLIST fixpt_vl[] = {
	"fixpt_diameter(deg)", &f_fixpt_diameter, NP, NP, 0, ME_FLOAT,
	"fixpt_window(deg)", &f_fixpt_window, NP, NP, 0, ME_FLOAT,
	"fixpt_color(R)", &f_fixpt_color[0], 0, NP, 0, ME_DEC,
	"fixpt_color(G)", &f_fixpt_color[1], 0, NP, 0, ME_DEC,
	"fixpt_color(B)", &f_fixpt_color[2], 0, NP, 0, ME_DEC,
	"Background_color(R)", &f_background_color[0], 0, NP, 0, ME_DEC,
	"Background_color(G)", &f_background_color[1], 0, NP, 0, ME_DEC,
	"Background_color(B)", &f_background_color[2], 0, NP, 0, ME_DEC,
	NS,
};

char hm_fixpt[] = "";

/* 
 * Timing menu - all values in milliseconds
 */

int f_trial_init_pause_time = 200;	
int f_fixpt_acq_time = 4000;
int f_fixpt_hold_time = 1000;
int f_fixpt_acq_fail_pause_time = 2000;
int f_fixpt_acq_noise_pause_time = 200;
int f_intertrial_pause_time = 500;

VLIST timing_vl[] = {
	"trial_init_pause", &f_trial_init_pause_time, NP, NP, 0, ME_DEC,
	"fixpt_acq_time", &f_fixpt_acq_time, NP, NP, 0, ME_DEC,
	"fixation_time", &f_fixpt_hold_time, NP, NP, 0, ME_DEC,
	"acq_fail_timeout", &f_fixpt_acq_fail_pause_time, NP, NP, 0, ME_DEC,
	"acq_noise_timeout", &f_fixpt_acq_noise_pause_time, NP, NP, 0, ME_DEC,
	"intertrial_time", &f_intertrial_pause_time, NP, NP, 0, ME_DEC,
	NS
};

char hm_timing[] = "";


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
	{"timing", &timing_vl, NP, NP, 0, NP, hm_timing}, 
	{"fixpt/bkgd", &fixpt_vl, NP, NP, 0, NP, hm_fixpt}, 
	{"fix_positions", &fixpt_pos_vl, NP, NP, 0, NP, hm_fixpt_pos}, 
	{"communication", &comm_vl, NP, NP, 0, NP, hm_comm}, 
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

	dprintf("Initializing rex and render...\n");

	// seed random number generator if necessary
	// TODO: Make usage of random number generator (and seed) consistent. 

	if (!f_seed_used)
	{
		ivunif(f_seed, 1);
		f_seed_used = 1;
	}
	
	// zero out handles and initialize counters.

	f_fixpt_handle = 0;
	f_handle_count = 0;
	f_reward_flag = 0;
	f_all_trials_done = 0;
	f_abort_exp = 0;

	// background color

	render_bgcolor(f_background_color[0], f_background_color[1], f_background_color[2]);

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

	return status;
}





/*
 * my_exp_init()
 * 
 * Initialize fixpt object and send render_dot() (will catch handle in 
 * my_check_for_handle). 
 * 
 * Initialize fixation point positions. 
 * 
 * If the menu setting for calibration, f_calibration, is 1 
 * (horizontal calibration) or 2 (vertical calibration) then 
 * there will be just three fixation point positions: (0,0), 
 * and two more offset by f_calibration_offset degrees in the 
 * positive and negative directions (obviously when calibration 
 * is horizontal its right and left, and when calibration is 
 * vertical its down and up), f_vertical_offset. 
 * 
 * If the menu setting for calibration is 0 then the calibration
 * offset value is ignored. Instead, the fixation point positions
 * in use will be determined by 4 settings, two for horizontal
 * and two for vertical. For each direction there is a number
 * of steps (f_nhorizontal_steps, f_nvertical_steps) and a max
 * offset (f_max_horizontal_offset, f_max_vertical_offset). The 
 * number of steps determines the number of fixation points on 
 * each side of the axis. For example, if f_nhorizontal_steps is 
 * 3, then there will be 3 fixation points with x>0 and 3 with x<0, 
 * for a total of 7 (2*f_nhorizontal)steps+1) in that direction. 
 * The vertical positions are determined in the same way. Note that 
 * the point (0,0) is used once. The steps between 0 and the max 
 * offset in each direction are linearly spaced. 
 * 
 * When f_calibration is 0 the resulting fixation point positions
 * will form a cross centered at (0,0). The total number of 
 * fixation point positions (and hence trial types) will be
 * (2*f_nhorizontal_steps + 1) + (2*f_nvertical_steps + 1) - 1.
 * 
 * When f_calibration is 4 the resulting fixation point positions
 * fill an entire grid, so the number of fixation point positions 
 * will be
 * (2*f_nhorizontal_steps + 1) *  (2*f_nvertical_steps + 1)
 * 
 * The arrays f_pfixpt_x and f_pfixpt_y are allocated and filled
 * here. The random trial generator is created and its multiplicity
 * is the number of fixation point positions. The generator will 
 * produce indices that specify the fixation point position to be 
 * used on each trial. 
 * 
 * Note that the random trial generator is NOT used for calibration
 * runs. In that case the fixpt positions follow a set pattern and 
 * there is no limit to the number of trials that will be run (i.e. 
 * you better pay attention). 
 * 
 */
 
 

int my_exp_init()
{
	int i, j;
	int count = 0;
	
	dprintf("Initializing for this experiment...\n");
	
	/*
	 * Initializations and cleanup...
	 */
	 
	f_handle_count = 0;
	for (i=0; i<4; i++) f_trial_tallies[i]=0;

	if (f_prtgen)
	{
		rtg_destroy(f_prtgen);
	}
	if (f_pfixpt_x)
	{
		free(f_pfixpt_x);
		f_pfixpt_x = NULL;
	}
	if (f_pfixpt_y)
	{
		free(f_pfixpt_y);
		f_pfixpt_y = NULL;
	}


	switch (f_calibration)
	{
		case 1:
		{
			f_nfixpt = 3;
			f_pfixpt_x = (float *)malloc(f_nfixpt * sizeof(float));
			f_pfixpt_y = (float *)malloc(f_nfixpt * sizeof(float));
			for (i=0; i<f_nfixpt; i++)
			{
				f_pfixpt_y[i] = 0.0;
				f_pfixpt_x[i] = (i-1)*f_calibration_offset;
			}
			break;
		}
		case 2:
		{
			f_nfixpt = 3;
			f_pfixpt_x = (float *)calloc(f_nfixpt, sizeof(float));
			f_pfixpt_y = (float *)calloc(f_nfixpt, sizeof(float));
			for (i=0; i<3; i++)
			{
				f_pfixpt_x[i] = 0.0;
				f_pfixpt_y[i] = (i-1)*f_calibration_offset;
			}
			break;
		}
		case 0:
		{
			float stepsize;
			f_nfixpt = (2*f_nhorizontal_steps + 1) + (2*f_nvertical_steps + 1) - 1;
			f_pfixpt_x = (float *)calloc(f_nfixpt, sizeof(float));
			f_pfixpt_y = (float *)calloc(f_nfixpt, sizeof(float));
			/* 
			 * Set up origin, then horizontal steps, then vertical steps.
			 */
			 
			f_pfixpt_x[0] = f_pfixpt_y[0] = 0;
			count = 1;

			if (f_nhorizontal_steps > 0) stepsize = f_max_horizontal_offset/(float)f_nhorizontal_steps;
			else stepsize = 0;
			for (i=1; i<=f_nhorizontal_steps; i++)
			{
				f_pfixpt_x[count] = i*stepsize;
				f_pfixpt_y[count] = 0;
				f_pfixpt_x[count+1] = -i*stepsize;
				f_pfixpt_y[count+1] = 0;
				count += 2;
			}

			if (f_nvertical_steps > 0) stepsize = f_max_vertical_offset/(float)f_nvertical_steps;
			else stepsize = 0;

			for (i=1; i<=f_nvertical_steps; i++)
			{
				f_pfixpt_x[count] = 0;
				f_pfixpt_y[count] = i*stepsize;
				f_pfixpt_x[count+1] = 0;
				f_pfixpt_y[count+1] = -i*stepsize;
				count += 2;
			}
			
			break;			
		}
		case 3:
		{
			if (strlen(f_fixpt_pos_file)==0)
			{
				f_nfixpt = f_nfixpt_xy;
				f_pfixpt_x = (float *)calloc(f_nfixpt, sizeof(float));
				f_pfixpt_y = (float *)calloc(f_nfixpt, sizeof(float));
	
				for (i=0; i<f_nfixpt; i++)
				{
					f_pfixpt_x[i] = f_fixpt_x_degrees[i];		 
					f_pfixpt_y[i] = f_fixpt_y_degrees[i];		 
				}
			}
			else
			{
				PEyepos ep = eyepos_load_fp(f_fixpt_pos_file);
				
				if (!ep)
				{
					f_abort_exp = 1;
					return ERRORCD;
				}
				
				f_nfixpt = ep->size(ep);
				f_pfixpt_x = (float *)calloc(f_nfixpt, sizeof(float));
				f_pfixpt_y = (float *)calloc(f_nfixpt, sizeof(float));
	
				for (i=0; i<f_nfixpt; i++)
				{
					float x, y;
					ep->getFP(ep, i, &x, &y);
					f_pfixpt_x[i] = x;		 
					f_pfixpt_y[i] = y;		 
				}
			}
			break;			
		}
		default:
		{
			dprintf("ERROR! Calibration setting %d not handled!!!\n", f_calibration);
			break;
		}
	}			


	/* 
	 * If this is NOT a calibration run (f_calibration==0), then 
	 * initialize trial generator. 
	 * 
	 * The trial generator will generate an index
	 * into the array f_fixpt_x[] and f_fixpt_y[] 
	 * which we just created and initialized.
	 */


	if (f_calibration== 0 || f_calibration==3)
	{
		dprintf("Create random trial generator with %d conditions, %d trials, blocksize %d.\n", f_nfixpt, f_ntrials, f_nblocksize);
		f_prtgen = rtg_create(f_nfixpt, f_ntrials, f_nblocksize);
	}
	else
	{
		f_calibration_current_index = 0;
		f_calibration_current_index_count = 0;
	}


	/*
	 * Set state times and reward size. 
	 */

	set_times("trial_init_pause", (long)f_trial_init_pause_time, -1);	
	set_times("fixpt_acq", (long)f_fixpt_acq_time, -1);
	set_times("fixpt_hold", (long)f_fixpt_hold_time, -1);
	set_times("fixpt_acq_fail_pause", (long)f_fixpt_acq_fail_pause_time, -1);
	set_times("fixpt_acq_noise_pause", (long)f_fixpt_acq_noise_pause_time, -1);
	set_times("intertrial_pause", (long)f_intertrial_pause_time, -1);
	set_times("reward_on", f_reward_preset, f_reward_random);

	render_frame(0);
	
	return 0;
}



/* 
 * my_check_for_handle 
 * 
 * Called as an escape function ... e.g. 
 * 
 * to wherever on 1 % my_check_for_handle
 * 
 * This function will fetch 1 handle from render. It is saved
 * as f_fixpt_handle, and the function returns 1 when the handle
 * message is received. Returns 0 when no msg is found.  
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
	}
	return (f_handle_count == 1);
}


/*
 * fixpt_onoff(int onoff)
 * 
 * Turn fixpt on or off depending on arg (1/0). 
 */
 

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
 * my_trial_init()
 * 
 * Initializes conditions for a trial. Called as a state action.
 * Selects fixation point location (depends on whether this is a 
 * calibration run or not)and updates graphics. Sets position of 
 * eye window and opens it. Opens analog window as well (these 
 * windows are closed in my_trial_done). 
 * 
 * Drop ecode TRLSTART, then 
 * bcode channel 1 - fixpt x,
 * bcode channel 2 - fixpt y 
 * 
 */

int my_trial_init()
{
	int status = 0;

	if (f_prtgen)
	{
		f_current_condition_index = f_prtgen->next(f_prtgen);
		if (f_current_condition_index < 0)
		{
			f_all_trials_done = 1;
			return status;
		}
	}
	else
	{
		if (f_calibration_current_index_count == 2)
		{
			f_calibration_current_index_count = 0;
			f_calibration_current_index += 1;
			if (f_calibration_current_index == f_nfixpt)
			{
				f_calibration_current_index = 0;
			}
		}
		f_calibration_current_index_count++;
		f_current_condition_index = f_calibration_current_index;
	}

	/* 
	 * Update fixpt position
	 */
	 
	f_fixpt.xorigin = to_pixels(f_pfixpt_x[f_current_condition_index]);
	f_fixpt.yorigin = to_pixels(f_pfixpt_y[f_current_condition_index]);
	render_update(f_fixpt_handle, &f_fixpt, sizeof(f_fixpt), 0);
	dprintf("Fixpt position (%d, %d)\n", (int)(f_pfixpt_x[f_current_condition_index]*10), (int)(f_pfixpt_y[f_current_condition_index]*10));

	/* 
	 * Open analog window, eye window. 
	 */
	 
	awind(OPEN_W);
 	wd_src_pos(EYE_WINDOW_FIXPT, WD_DIRPOS, 0, WD_DIRPOS, 0);
	wd_pos(EYE_WINDOW_FIXPT, (long)(f_pfixpt_x[f_current_condition_index]*10.0), (long)(f_pfixpt_y[f_current_condition_index]*10.0));
	wd_siz(EYE_WINDOW_FIXPT, (long)(f_fixpt_window*10.0), (long)(f_fixpt_window*10.0));
	wd_cntrl(EYE_WINDOW_FIXPT, WD_ON);
	wd_src_check(EYE_WINDOW_FIXPT, WD_SIGNAL, 0, WD_SIGNAL, 1);


	/*
	 * ecodes, bcodes
	 */
	 
	ecode(TRLSTART);
	bcode_float(CH_FIXPT_X, f_pfixpt_x[f_current_condition_index]);
	bcode_float(CH_FIXPT_Y, f_pfixpt_y[f_current_condition_index]);

	return status;
}

/* 
 * my_trial_done(arg)
 * 
 * Wraps up a trial: tallies counters (depending on arg) and 
 * marks a trial as completed if the trial was successful. 
 */
 
int my_trial_done(int itrialstatus)
{
	/*
	 * Close analog, eye windows
	 */
	 
	awind(CLOSE_W);
	wd_cntrl(EYE_WINDOW_FIXPT, WD_OFF);

	
	switch (itrialstatus)
	{
		case TRIAL_SUCCESS:
			dprintf("Success!\n");
			ecode(CORRECTCD);
			if (f_prtgen)
			{
				f_prtgen->mark(f_prtgen, f_current_condition_index);
				show_trials_status();
			}
			f_trial_tallies[0]++;
			break;
		case TRIAL_FAIL_ACQ:
			dprintf("Acquisition failed.\n");
			ecode(NOCHCD);
			f_trial_tallies[1]++;
			break;
		case TRIAL_FAIL_NOISE:
			dprintf("Fixation failed - noise.\n");
			ecode(BREAKFIXCD);
			f_trial_tallies[2]++;
			break;
		case TRIAL_FAIL_PAUSE:
			dprintf("Trial paused.\n");
			ecode(WRONGCD);
			f_trial_tallies[3]++;
			break;
		case TRIAL_FAIL_ABORT:
			break;
		default:
			dprintf("my_trial_done: ERROR unhandled arg (%d)\n", itrialstatus);
			break;
	}
	
	ecode(TRLEND);
	
	return 0;
}

void show_trials_status()
{
	int i, j;
	dprintf("===================Trial counts=================\n");
	dprintf("ind\ttally\n");
	dprintf("---\t-------------------------------\n");
	for (i=0; i<f_nfixpt; i++)
	{
		dprintf("%d\t", i);
		for (j=0; j<f_prtgen->count(f_prtgen, i); j++) dprintf("X");
		dprintf("\n");
	}
	dprintf("\n");  
	return;		
}



/*
 * my_reward()
 * 
 * Sets reward flag. Reward loop will take care of the actual reward. 
 */
 
int my_reward()
{
	f_reward_flag = 1;
	return 0;
}


/*
 * reward(arg)
 * 
 * Turns reward on (REWARD_ON) or off (REWARD_OFF). Called as an 
 * action from reward loop. 
 */

int reward(int ireward)
{
	int status = 0;
	switch (ireward)
	{
		case REWARD_ON:
		{
			dio_on(REW);
			if (f_verbose & DEBUG_REWARD_BIT) dprintf("reward on\tnow=%d\n", getClockTime());
			f_reward_flag = 0;
			status = REWCD;
			break;
		}
		case REWARD_OFF:
		{
			dio_off(REW);
			if (f_verbose & DEBUG_REWARD_BIT) dprintf("reward off\tnow=%d\n", getClockTime());
			break;
		}
		default:
		{
			dprintf("ERROR: unhandled state (%d) in reward()!\n", ireward);
			status = ERRORCD;
			break;
		}
	}
	return status; 
}
			
		
int all_trials_done()
{
	dprintf("All trials done.\n");
	return 0;
}
		
int my_exp_abort()
{
	dprintf("Expt aborted. Check config and Controls>Process Controls>Print Debug\n");
	return 0;
}

/* REX state set starts here 
 * djs 5-13-09 id=501 
 */

%%

id 501
restart init_steering
main_set {
status ON
begin	first:
		code HEADCD
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
		do render_reset()
		to render_init
	render_init:
		do my_render_init()
		to exp_init on 1 % my_check_for_handle
	exp_init:
		do my_exp_init()	/* initialize trial counters */
		to exp_abort on 1 = f_abort_exp;
		to trial_init on 1 % render_check_for_went
	trial_init:
		do my_trial_init()	/* initialization for a single trial */
		to all_done on 1 = f_all_trials_done
		to trial_init_pause
	trial_init_pause:
		time 500			/* f_trial_init_pause_time */
		to pause_detected on +PSTOP & softswitch
		to fixpt_on
	fixpt_on:
		do fixpt_onoff(1)
		to pause_detected_wait on +PSTOP & softswitch
		to fixpt_acq on 1 % render_check_for_went
	fixpt_acq:
		time 4000			/* f_fixpt_acq_time */
		to pause_detected on +PSTOP & softswitch
		to fixpt_hold on -WD0_XY & eyeflag
		to fixpt_acq_fail
	fixpt_acq_fail:
		do fixpt_onoff(0)
		to fixpt_acq_fail_pause on 1 % render_check_for_went
	fixpt_acq_fail_pause:
		time 500			/* f_fixpt_acq_fail_pause_time */
		do my_trial_done(TRIAL_FAIL_ACQ)
		to trial_init
	fixpt_hold:
		code ACQCD
		time 150			/* f_fixpt_hold_time */
		to fixpt_acq_noise on +WD0_XY & eyeflag
		to fixpt_off
	fixpt_acq_noise:
		do fixpt_onoff(0)
		to fixpt_acq_noise_pause on 1 % render_check_for_went
	fixpt_acq_noise_pause:
		time 500			/* f_fixpt_acq_noise_pause_time */
		do my_trial_done(TRIAL_FAIL_NOISE)
		to trial_init
	fixpt_off:
		code FIXCD
		do fixpt_onoff(0)
		to reward
	reward:
		do my_reward(1)
		to trial_success
	trial_success:
		do my_trial_done(TRIAL_SUCCESS)
		to intertrial_pause
	intertrial_pause:
		time 1000			/* f_intertrial_pause_time */
		to trial_init
	pause_detected_wait:
		to pause_detected on 1 % render_check_for_went
	pause_detected:
		code PAUSECD
		do my_trial_done(TRIAL_FAIL_PAUSE)
		to pause_wait on 1 % render_check_for_went
	pause_wait:
		to trial_init on -PSTOP & softswitch
	all_done:
		do all_trials_done()
		to first on 1 = f_never
	exp_abort:
		do my_exp_abort()
		to first on 1 = f_never
	abort_state:
		do my_trial_done(TRIAL_FAIL_ABORT)
		to trial_init
abort list:
		abort_state
}


/*
 * Reward state set. The paradigm must set the flag f_reward_flag=REWARD_ON
 * to initiate a reward. The size of the reward is specified by the
 * time/rand settings in the reward_on state -- these values are
 * actually taken from the dialogs and set during exp_init state. 
 */

 
reward_set {
status ON
begin	reward_begin:
		to reward_check
	reward_check:
		to reward_on on 1 = f_reward_flag
	reward_on:
		do reward(REWARD_ON)
		time 5
		rand 15
		to reward_off
	reward_off:
		do reward(REWARD_OFF)
		to reward_check
}

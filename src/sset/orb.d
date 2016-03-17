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
#include "timerFunction.h"

#undef DEBUG
/*#define DEBUG 1*/

#define X 0
#define Y 1
#define TRUE 1
#define FALSE 0

#define PI 3.14159265
#define RAD_PER_DEGREE (PI/180.0)
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
 
static int f_framespersecond = 85;

/* Perspective/camera parameters */
int nearPlaneDistance = 1;
int farPlaneDistance = 1000;
int fovy = 90;

/* dotfield handle */
int dotfield_handles[5] = { 0, 0, 0, 0, 0 };
static int f_handle_count = 0;

/* background color */
int background_color[3] = { 0, 0, 0 };

/* Addresses and port */
char *local_addr="192.168.1.1";
char *remote_addr="192.168.1.2";
int remote_port=2000;

/* messaging structures. */
DotFieldStruct f_dotfield;
DotFieldStruct f_dotfield_extras[4];
CircularOrbitStruct f_circularorbit;
MessageStruct msg;


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
	
}



int my_render_init()
{
	int status;

	// background color
	render_bgcolor(background_color[0], background_color[1], background_color[2]);

	// dotfield
	f_dotfield.	xwidth = f_dotfield.ywidth = f_dotfield.zwidth = 1.0;
	f_dotfield.xorigin = f_dotfield.yorigin = f_dotfield.zorigin = 0.0;
	f_dotfield.ndots = 1000;
	f_dotfield.pointsize = 2.0;
	f_dotfield.r = 255;
	f_dotfield.g = 0;
	f_dotfield.b = 255;
	f_dotfield.a = 0;
	render_dotfield(&f_dotfield);

	// Now send 4 additional dotfield commands 
	f_dotfield.	xwidth = f_dotfield.ywidth = f_dotfield.zwidth = 5.0;
	f_dotfield.r = 255;
	f_dotfield.g = 255;
	f_dotfield.b = 0;
	f_dotfield.a = 0;
	render_dotfield(&f_dotfield);

	f_dotfield.	xwidth = f_dotfield.ywidth = f_dotfield.zwidth = 1.0;
	f_dotfield.xorigin = 3;
	f_dotfield.yorigin = f_dotfield.zorigin = 0.0;
	f_dotfield.r = 0;
	f_dotfield.g = 255;
	f_dotfield.b = 0;
	f_dotfield.a = 0;
	render_dotfield(&f_dotfield);

	f_dotfield.	xwidth = f_dotfield.ywidth = f_dotfield.zwidth = 1.0;
	f_dotfield.xorigin = -3;
	f_dotfield.yorigin = f_dotfield.zorigin = 0.0;
	f_dotfield.r = 0;
	f_dotfield.g = 255;
	f_dotfield.b = 0;
	f_dotfield.a = 0;
	render_dotfield(&f_dotfield);

	f_dotfield.	xwidth = f_dotfield.ywidth = f_dotfield.zwidth = 3.0;
	f_dotfield.xorigin = f_dotfield.yorigin = f_dotfield.zorigin = 0.0;
	f_dotfield.ndots = 100;
	f_dotfield.pointsize = 3.0;
	f_dotfield.r = 0;
	f_dotfield.g = 0;
	f_dotfield.b = 255;
	f_dotfield.a = 0;
	render_dotfield(&f_dotfield);


	f_handle_count = 0;
	
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
	f_circularorbit.r = 5;										// radius
	f_circularorbit.theta0 = 0;									// initial theta
	f_circularorbit.rtheta = 1 * RAD_PER_DEGREE;			// radians/step
	f_circularorbit.phi0 = 90 * RAD_PER_DEGREE;			// initial phi
	f_circularorbit.rphi = 0;										// radians/step
	render_circularorbit(&f_circularorbit);   
   return 0;
}


int my_trial_start()
{
	int i;
	// turn on dotfields
	for (i=0; i<5; i++)		
		render_onoff(&dotfield_handles[i], HANDLE_ON, ONOFF_NO_FRAME);

	// send start command	
	render_start();
	
	return 0;
}


int my_trial_stop()
{
	int i;
	
	// send stop command	
	render_stop();

	// turn off dotfield
	for (i=0; i<5; i++)		
		render_onoff(&dotfield_handles[i], HANDLE_ON, ONOFF_NO_FRAME);
	
	return 0;
}


/* my_check_for_handle *********************************/

int my_check_for_handle()
{
	int status = 0;
	status = render_check_for_handle(&dotfield_handles[f_handle_count]);
	if (status == 1)
	{
		dprintf("my_check_for_handle: dotfield_handle[%d]=%d\n", f_handle_count, dotfield_handles[f_handle_count]);
		f_handle_count++;
	}
	return status;
}


int my_targoff()
{
	int i;
	for (i=0; i<5; i++)
		render_onoff(&dotfield_handles[i], HANDLE_OFF, ONOFF_NO_FRAME);
	return 0;
}
	
/* Viewing volume menu */

VLIST vvol_vl[] = {
"Y field of view(deg)", &fovy, NP, NP, 0, ME_DEC,
"near plane distance", &nearPlaneDistance, NP, NP, 0, ME_DEC, 
"far plane distance", &farPlaneDistance, NP, NP, 0, ME_DEC, 
NS,
};

char hm_vvol[] = "";


/* Background color menu  */

VLIST background_vl[] = {
"Background color(R)", &background_color[0], 0, NP, 0, ME_DEC,
"Background color(G)", &background_color[1], 0, NP, 0, ME_DEC,
"Background color(B)", &background_color[2], 0, NP, 0, ME_DEC,
NS,
};

char hm_background[] = "";



VLIST state_vl[] = {
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
{"background", &background_vl, NP, NP, 0, NP, hm_background},
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
		to count_handle on 1 % my_check_for_handle
	count_handle:
		to all_targoff on 5 = f_handle_count
		to inithandle
	all_targoff:
		do my_targoff()
		to waitscene
	waitscene:
		time 100
		to loop
	loop:
		do my_trial_init()
		to trial_pause
	trial_pause:
		time 3000
		to trial_start
	trial_start:
		do my_trial_start()
		to trial_running
	trial_running:
		time 60000
		to trial_stop
	trial_stop:
		do my_trial_stop()
		to p3
	p3:
		to p4 on +PSTOP & softswitch
		to loop on -PSTOP & softswitch
	p4:	
		to loop on -PSTOP & softswitch
abort	list:
		first
}



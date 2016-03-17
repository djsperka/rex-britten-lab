/* $Id: ff1.d,v 1.2 2007/06/28 22:25:13 dan Exp $ */

/*  
 * Paradigm for fixation (and eventually saccade) training. Allows the
 * experimenter to control background luminance and fixation spot size
 * and color.
 *
 * This version updated to be compatible with Rex version 5.4
 * KHB, 12/21/95
 * 
 * Saccade.d generated from fix.d (and bits of the old saccade.d) 2/16/96
 *
 * ecal.d copied from saccade.d, updated to use new render. 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


#include "ldev_tst.h"	/* hardware device defines */
#include "lcode_tst.h"	/* Ecodes */

#include "actions.h"	/* includes all files needed for use with new render. */
#include "make_msg.h"

#define PI 3.14159265

#define WIND0	    0
#define EYEH_SIG    0
#define EYEV_SIG    1

// use these as the first arg to targ_ask
// ASK_ON means turn it on, ASK_OFF means turn it off. 
#define ASK_ON HANDLE_ON
#define ASK_OFF HANDLE_OFF

// use these as the SECOND arg to targ_ask
// ASK_FP means we're talking about the fixpt, ASK_TARG means we're talking about the target. 
#define ASK_FP 2
#define ASK_TARG 3


/**********************************************************************
 * Global variables.
 **********************************************************************/
 
int errcnt;
char outbuf[512];									/* used in pr_info */
static unsigned char fphandle, fprh, trh;      /* handles for fixation point */  
static int isfpon = 0, iston = 0;

/************************************************************************
 * Declaration of statelist variables.
 ************************************************************************/
 
int	fixx = 0,
	fixy = 0,
	targx = NULLI,
	targy = NULLI,
	/* eyeh = targx, */
	/* eyev = targy, */
	red = 255,
	green = 0,
	blue = 0,
	stimz = 57, 
	fpsiz = 8,
	axis = 0,
	dist = 100,
	ntrials = 100,
	msec = 1000,
	remain,
	bg = 0;		/* background grayscale value */

float cam_position[3]; 							/* Current camera position */
float cam_looking[3];
float cam_up[3] = { 0, 1, 0 };

char local_addr[16] = "192.168.1.1";
char remote_addr[16]="192.168.1.2";
int remote_port = 2000;


/* For fixpt and dotfield */

int f_fpHandle = 0;
int f_dotsHandle = 0;
int f_handleCount = 0;
DotStruct f_fpStruct;
DotFieldStruct f_dotsStruct;
PathStruct f_path;
MoveObjectStruct f_moveobject;



/************************************************************************
 * Control functions to make things happen.
 ************************************************************************/

int my_render_init()
{
	int status;

	// background color
	render_bgcolor(bg, bg, bg);

	// Setup viewing volume
	render_perspective(90, 1, 1000);

	// position camera
	cam_position[0] = 0;
	cam_position[1] = 0;
	cam_position[2] = 5;
	cam_looking[0] = 0;
	cam_looking[1] = 0;
	cam_looking[2] = -1;
	cam_up[0] = 0;
	cam_up[1] = 1;
	cam_up[2] = 0;
	render_camera(cam_position, cam_looking, cam_up);

	/* Configure fp.  */
	f_fpStruct.xorigin = 0;
	f_fpStruct.yorigin = 0;
	f_fpStruct.xsize = 10.0;
	f_fpStruct.ysize = 10.0;
	f_fpStruct.depth = 10;
	f_fpStruct.r = 255;
	f_fpStruct.g = 255;
	f_fpStruct.b = 255;
	f_fpStruct.a = 0;
	render_dot(&f_fpStruct);		// will need to get handle return

	// configure dotfield
	f_dotsStruct.xwidth = 1.0;
	f_dotsStruct.ywidth = 1.0;
	f_dotsStruct.zwidth = 1.0;
	f_dotsStruct.xorigin = 0.0;
	f_dotsStruct.yorigin = 0.0;
	f_dotsStruct.zorigin = 0.0;
	f_dotsStruct.ndots = 100;
	f_dotsStruct.pointsize = 2.0;
	f_dotsStruct.r = 255;
	f_dotsStruct.g = 255;
	f_dotsStruct.b = 255;
	f_dotsStruct.a = 0;
	render_dotfield(&f_dotsStruct);		// will need to get handle return

	return 0;
}



/* 
 * my_check_for_handles 
 * 
 * Escape function that returns 1 when two handles have been received. 
 * The first handle received will be the fp handle. 
 * The second handle received will be the dotfield handle. 
 * 
 * Uses a global int, f_handleCount. When that count reaches 2 we're done. 
 * 
 */

int my_check_for_handles()
{
	int h=0;
	while (render_check_for_handle(&h))
	{
		if (f_handleCount == 0)
		{
			f_fpHandle = h;
			f_handleCount = 1;
			dprintf("fp handle %d\n", f_fpHandle);
//			render_onoff(&f_fpHandle, HANDLE_ON, ONOFF_NO_FRAME);
		}
		else if (f_handleCount == 1)
		{
			f_dotsHandle = h;
			f_handleCount = 2;
			dprintf("dots handle %d\n", f_dotsHandle);
//			render_onoff(&f_dotsHandle, HANDLE_OFF, ONOFF_NO_FRAME);
		}
		else
		{
			dprintf("ERROR: Handle count is %d\n", f_handleCount);
		}
	}
	return (f_handleCount == 2);
}


/* my_path_setup
 * 
 * Send path commands. 
 * 
 */
int my_path_setup()
{
	// configure path parameters
	f_path.ep[0] = 0;
	f_path.ep[1] = 0;
	f_path.ep[2] = 10;

	f_path.epf[0] = 4;
	f_path.epf[1] = 0;
	f_path.epf[2] = 0;
	
	f_path.vp[0] = 0;
	f_path.vp[1] = 0;
	f_path.vp[2] = 0;

	f_path.vpf[0] = 0;
	f_path.vpf[1] = 0;
	f_path.vpf[2] = 0;

	f_path.nframes = 120;	// 2 seconds at 85Hz
	f_path.ndelay = 50;		// no delay before starting motion

	// send the PATH command
	render_path(&f_path);

	// configure moveobject parameters
	f_moveobject.handle = f_fpHandle;

	f_moveobject.initial_pos[0] = 0;
	f_moveobject.initial_pos[1] = 0;

	f_moveobject.final_pos[0] = 200;
	f_moveobject.final_pos[1] = 200;

	f_moveobject.nframes = 170;

	// send the MOVEOBJECT command
	render_moveobject(&f_moveobject);

	return 0;
}


/* my_path_start
 * 
 * Send start command.This starts the animations which were configured in my_path_setup. 
 * 
 */
int my_path_start()
{
	render_start();
	return 0;
}

/* my_path_stop
 * 
 * Send stop command. This stops the animation. Note that this command must be sent even if the animation has
 * already stopped (that is, it appears to have stopped).  
 * 
 */
int my_path_stop()
{
	render_stop();
	return 0;
}

/* my_clear
 * 
 * Clear screen after animation has stopped by turning off all graphic handles.
 * 
 */
int my_clear()
{
	render_onoff(&f_fpHandle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_onoff(&f_dotsHandle, HANDLE_OFF, ONOFF_NO_FRAME);
	render_frame(1);
	return 0;
}

/* my_show
 * 
 * Turn on fp and dots, issue RENDER command.  
 * 
 */
int my_show()
{
	render_update(f_dotsHandle, &f_dotsStruct, sizeof(f_dotsStruct), HANDLE_ON);
	render_update(f_fpHandle, &f_fpStruct, sizeof(f_fpStruct), HANDLE_ON);
	render_camera(cam_position, cam_looking, cam_up);
//	render_onoff(&f_fpHandle, HANDLE_ON, ONOFF_NO_FRAME);
//	render_onoff(&f_dotsHandle, HANDLE_ON, ONOFF_NO_FRAME);
	render_frame(1);
	return 0;
}
	


/********************************* setbg() *******************************
 *
 * sets the background luminance
 */
int setbg()
{
	/* TODO */
	render_bgcolor(bg, bg, bg);
	return 0;
}


void rinitf(void)
{
	int status=0;

	// initialize tcpip - The last arg (0) turns autoflush off. 
	status = init_tcpip(local_addr, remote_addr, remote_port, 0);

}




VLIST state_vl[] = {
"fp_x",		&fixx,	NP, NP,	0, ME_DEC,
"fp_y",		&fixy,	NP, NP,	0, ME_DEC,
"duration",	&msec, NP, NP, 0, ME_DEC, 
"bg(0-255)", 	&bg, NP, NP, 0, ME_DEC, 
"axis(0,1)",	&axis, NP, NP, 0, ME_DEC,
"fp_R(0-255)", 	&red, NP, NP, 0, ME_DEC, 
"fp_G(0-255)", 	&green, NP, NP, 0, ME_DEC, 
"fp_B(0-255)", 	&blue, NP, NP, 0, ME_DEC, 
"targ_dist", 	&dist, NP, NP, 0, ME_DEC,
"targ_x_ovrd",	&targx, NP, NP, 0, ME_NVALD,
"targ_y_ovrd",	&targy, NP, NP, 0, ME_NVALD,
"CRT_dist(cm)",		&stimz, NP, NP, 0, ME_DEC,
"fp_size(pix)",		&fpsiz, NP, NP, 0, ME_DEC,
"trials",		&ntrials, NP, NP, 0, ME_DEC,
"local ip addr", local_addr, NP, NP, 0, ME_STR,
"remote ip addr", remote_addr, NP, NP, 0, ME_STR,
"remote port", &remote_port, NP, NP, 0, ME_DEC,
NS,
};

/*
 * Help message.
 */
char hm_sv_vl[] = "";



%%

id 10
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
		to pause1
	pause1:
		to pause2 on +PSTOP & softswitch
		to renderinit on -PSTOP & softswitch
	pause2:	
		to renderinit on -PSTOP & softswitch
	renderinit:
		do my_render_init()
		to targhandle
	targhandle:
		to loop on 1 % my_check_for_handles
	loop:
		do my_show()
		to initpath
	initpath:
		do my_path_setup()
		to startpath
	startpath:
		do my_path_start()
		to waitpath
	waitpath:
		time 2000
		to stoppath
	stoppath:
		do my_path_stop()
		to clear
	clear:
		do my_clear()
		to isi
	isi:
		time 500
		to loop
	pause3:
		to pause4 on +PSTOP & softswitch
		to nothing on -PSTOP & softswitch
	pause4:
		code PAUSECD
		to nothing on -PSTOP & softswitch
	nothing:
		to exit on 1 = f_fpHandle
	exit:
		to pause1

abort list:
		exit
}

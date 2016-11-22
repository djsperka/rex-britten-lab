#ifndef BH_ANIMHELPER_H_
#define BH_ANIMHELPER_H_


#include "actions.h"

typedef struct bpsh_struct BPSHStruct;

#define BPSH_PTYPE_NONE 0			/* fixation */
#define BPSH_PTYPE_PURSUIT 1		/* fixpt moves */
#define BPSH_PTYPE_SIMULATED 2		/* fixpt stationary on screen, dots move to simulate pursuit */
#define BPSH_PTYPE_RETSTAB 3		/* pursuit, but with retinal stabilization */

#define BPSH_TTYPE_NONE 0			/* this implies no dots. frozen dots is translation with vh=0 */
#define BPSH_TTYPE_TRANSLATION 1

struct bpsh_struct
{
	/* Graphic handles and structs */
	
	GHandle hdot;
	GHandle hptrans;
	GHandle hospace;
	
	/* Ecode for raster plots */
	int ecode;
	
	/* 
	 * For retinal stabilization, current eye velocity, cumulative corrections in 
	 * azimuth and elevation, and OK flag (indicates if current velocity is usable). 
	 * Set in eye velocity state set.
	 */
	 	 
	int evx, evy;
	float azcorrection;
	float elcorrection;
	float evfactor;			// conversion factor: computed velocity -> degrees/frame
	int evOK;
	int evType;				// 0 = rex eye vel; 1 = window average position; same as f_evTest;

	/*
	 * Experimental changes to retinal stabilization.
	 * Keep state of eye averages here. 
	 */
	 
	float last_good_x, last_good_y;
	int last_good_is_valid;
	float last_delta_x, last_delta_y;
	int last_delta_is_valid;
	 
	
	/* pursuit parameters. */

	float x, y;			/* pos (in degrees) of middle of pursuit path */
	float vp;			/* pursuit velocity, in degrees per frame */
	float rho;			/* pursuit angle, in degrees */
	int ptype;			/* BPSH_TYPE_xxxxx */
	
	/* Heading parameters. Set n_wait = n_travel = 0 when no dots */
	
	int ttype;			/* BOSH_TTYPE_xxxxx */
	float vh;			/* pursuit velocity, in dots-units per frame. */
	float alpha;		/* azimuth angle, in degrees */
	float theta;		/* elevation angle, in degrees */
	
	/* timing parameters */

	int n_translation_start;	/* frames until translation starts; if <0, then no translation (frozen dots) */
	int n_all_done;				/* frames until trial is complete */
	int n_pre_pursuit_start;	/* frames until eps pursuit motion starts */
	int n_pursuit_start;		/* frames until pursuit motion starts */
	int n_blink;				/* number of frames a blink lasts. if 0, no blink. THIS IS A DURATION, NOT A START TIME */
	int do_pursuit_jump_start;  /* when != 0, fixation is at pursuit start pos, dot jumps at start of 
								 * pursuit to the phi0 position, but eye window remains at pursuit
								 * start pos until pursuit period begins. During pursuit period, eye window
								 * tracks dot */
	
	/* DO NOT INITIALIZE BELOW THIS LINE! */
	
	PursuitTransformStruct ptrans;
	CameraStruct cam;
	float beta;			/* computed - does not change during animation, in degrees */
	float phi0;			/* starting value of phi at step 0 (takes into account epsilon motion) */
	float phiPS;        /* value of phi at start of pursuit period (after epsilon period) */
	float dx, dy, dz;	/* translation step size */	
	int n_blink_start;	/* frame where blink starts; 0 if no blink */
	int n_blink_end;	/* frame where blink stops; 0 if no blink */
	float eyewx;
	float eyewy;
	int set_eye_window_size;  /* When set, tells set_eye_window to set size to eyesx, eyesy */
	float eyesx;
	float eyesy;
	
	
};


/* 
 * Compute values in preparation for animation. Set values in struct 
 * up to "DO NOT INITIALIZE BELOW THIS LINE", then call this. After calling
 * you can start calling bpsh_step().
 */
 
int initialize_bpsh_struct(BPSHStruct *pbpsh);

int bpsh_step(BPSHStruct *pbpsh, int istep);

void print_bpsh(BPSHStruct *pbpsh);

#endif /*BH_ANIMHELPER_H_*/

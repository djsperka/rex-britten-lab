#ifndef R3P_H_
#define R3P_H_

#include <stdio.h>

typedef struct r3p_header
{
	long time;
	float fixpt_x;
	float fixpt_y;
	float cam_height;
	int speed;
	float cam_position[3];
	float cam_trajectory;
	float ttrans_dir_offset;
} R3PHeader;

typedef struct went_data
{
	int ijoyh;
	float cam_trajectory;
	float ttrans_dir;
	float jitter_trajectory_delta;
	float jitter_target_delta;
	float meander_target_delta;
	float diffusion_target_delta;
} R3PWent;

typedef struct r3p_trial
{
	R3PHeader header;
	int nwent;
	R3PWent *pwent;
} R3PTrial;

typedef struct r3p_trial *R3PTrialP;

int r3p_get_trials_min(FILE *fp, int min_trial_length_frames);
R3PTrialP r3p_get_trial(int i);
int r3p_get_n_trials();
void r3p_clear();

#endif /*R3P_H_*/

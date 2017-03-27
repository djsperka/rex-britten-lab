#ifndef _BH_REPLAY_H
#define _BH_REPLAY_H

#include "bh_animhelper.h"

int bh_replay_prepare_output_file(char *filename, int nframes_per_trial);
void bh_replay_new_trial(BPSHStruct *pbpsh, int timestamp);
int bh_replay_save_frame(int istep, int cam_update, int ptrans_update, int dot_update, int dot_onoff, int ospace_update, int ospace_onoff, BPSHStruct *pbpsh);
int bh_replay_write_trial();
void bh_replay_close_output_file();
int bh_replay_load(char *filename, BPSHStruct **pbpshlist, int maxlist);

#endif

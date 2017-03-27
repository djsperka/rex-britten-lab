#include "bh_replay.h"
#include "bh_animhelper.h"

BPSHStruct f_bpshCurrent;
BPSHSave *f_array = NULL;
int f_arrayCount = 0;
FILE *f_fp = NULL;
int f_timestampCurrent = 0;

#define IPRT(x, y) (int)(x*y)


void bh_replay_new_trial(BPSHStruct *pbpsh, int timestamp)
{
	f_bpshCurrent = *pbpsh;
	f_arrayCount = 0;
	f_timestampCurrent = timestamp;
}


/* 
 * create (and empty) output filename for saving replay info.
 * Will not clobber existing files! 
 * Returns 0 on success, -1 on failure (because file exists or
 * memory allocation failed)
 */

int bh_replay_create_output_file(char *filename, int nframes_per_trial)
{
	int status = 0;	
	if (f_array) free(f_array);
	f_array = NULL;
	f_arrayCount = 0;
	f_array = (BPSHSave *)calloc(nframes_per_trial, sizeof(BPSHSave));
	if (!f_array)
	{
		dprintf("Cannot allocate space for saving %d frames per trial!\n", nframes_per_trial);
		return -1;
	}

	// check if file exists
	if (f_fp = fopen(filename, "rb"))
	{
		fclose(f_fp);
		dprintf("Cannot open file %s: already exists.\n", filename);
		f_fp = NULL;
		return -1;
	}
	
	
	// OK file doesn't exist. Open it up and get out of here.
	f_fp = fopen(filename, "wb");
	if (!f_fp)
	{
		dprintf("Cannot open save commands file %s\n", filename);
		return -1;
	}
	else
	{
		dprintf("Allocated space for %d frames.\n", nframes_per_trial);
		dprintf("Saving replay info to file %s\n", filename);
	}
	
	return status;
}

/*
 * Save info for a single frame.
 * bh_replay_save must have been called successfully prior to calling. 
 * Return the current count of saves.
 */

int bh_replay_save_frame(int istep, int cam_update, int ptrans_update, int dot_update, int dot_onoff, int ospace_update, int ospace_onoff, BPSHStruct *pbpsh)
{
	int status = 0;
	BPSHSave *psave = f_array + f_arrayCount;
	if (!f_fp) return 0;

	psave->istep = istep;
	psave->cam_update = cam_update;
	psave->ptrans_update = ptrans_update;
	psave->dot_update = dot_update;
	psave->dot_onoff = dot_onoff;
	psave->ospace_update = ospace_update;
	psave->ospace_onoff = ospace_onoff;
	psave->ptrans = pbpsh->ptrans;
	psave->cam = pbpsh->cam;
	psave->azcorrection = pbpsh->azcorrection;
	psave->elcorrection = pbpsh->elcorrection;
	
	f_arrayCount++;
	return f_arrayCount;
}


/* 
 * Write info for current trial to file. 
 * Clear array so its ready for another trial.
 */

int bh_replay_write_trial()
{
	
	int status = 0;
	int magic = -2;
		
	/*
	 * Write magic number, timestamp, count, array
	 */
	fwrite(&magic, sizeof(int), 1, f_fp);
	fwrite(&f_bpshCurrent, sizeof(BPSHStruct), 1, f_fp);
	fwrite(&f_timestampCurrent, sizeof(int), 1, f_fp);
	fwrite(&f_arrayCount, sizeof(int), 1, f_fp);
	fwrite(f_array, sizeof(BPSHSave), f_arrayCount, f_fp);
	dprintf("Saved %d commands to file, timestamp %d\n", f_arrayCount, f_timestampCurrent);

	bh_replay_clear_trial();
	
	return status;
}

void bh_replay_clear_trial()
{
	f_arrayCount = 0;
}


void bh_replay_close_output_file()
{
	if (f_fp)
	  fclose(f_fp);
	if (f_array)
	  free(f_array);
	f_array = NULL;
	f_fp = NULL;
}



int bh_replay_load(char *filename, BPSHStruct **pbpshlist, int maxlist)
{
	int count = 0;
	if (f_fp) bh_replay_close_output_file();
	
	f_fp = fopen(filename, "rb");
	if (!f_fp)
	{
		dprintf("Cannot open bpsh command file %s\n", filename);
		return 0;
	}
	else
	{
		int magic, ts, nframes;
		dprintf("Opened bpsh command file %s\n", filename);
		dprintf("sizeof(BPSHStruct): %d\n", sizeof(BPSHStruct));
		dprintf("sizeof(BPSHSave): %d\n", sizeof(BPSHSave));
	
		// read magic number. If this fails, we've reached the end of the file.
		// Note no error checking, I assume that if the magic number is correct
		// then the rest of the record will be there. 
		while (1 == fread(&magic, sizeof(int), 1, f_fp))
		{
			if (magic == -2)
			{
				int i1, i2, i3, i4, j;
				BPSHStruct bpsh;
				pbpshlist[count] = (BPSHStruct *)malloc(sizeof(BPSHStruct));
				memset(pbpshlist[count], 0, sizeof(BPSHStruct));
				i1 = fread(pbpshlist[count], sizeof(BPSHStruct), 1, f_fp);
				i2 = fread(&(pbpshlist[count]->timestamp), sizeof(int), 1, f_fp);
				i3 = fread(&nframes, sizeof(int), 1, f_fp);
				pbpshlist[count]->psaved = (BPSHSave *)calloc(nframes, sizeof(BPSHSave));
				pbpshlist[count]->nsaved = nframes;
				i4 = fread(pbpshlist[count]->psaved, sizeof(BPSHSave), nframes, f_fp);				 
				dprintf("%d: t=%d n=%d frames %d %d %d %d\n", count, pbpshlist[count]->timestamp, nframes, i1, i2, i3, i4);
				
				print_bpsh(pbpshlist[count]);
#if 0
				for (j=0; j<nframes; j++)
				{
					dprintf("frame %d: %d %d %d\n", 
							pbpshlist[count]->psaved[j].istep,
							IPRT(pbpshlist[count]->psaved[j].cam.ex, 100),
							IPRT(pbpshlist[count]->psaved[j].cam.ey, 100),
							IPRT(pbpshlist[count]->psaved[j].cam.ez, 100));
				}
#endif
				count++;
			}
			else
			{
//				dprintf("ERROR - got bad magif number %d\n", magic);
			}
			
		}
		dprintf("Read %d trials\n", count);
		fclose(f_fp);
	}
	
	return count;
}


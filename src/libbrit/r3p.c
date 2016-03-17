#include "r3p.h"
#include <stdlib.h>

#define MAX_R3P_TRIALS 100
R3PTrial f_r3pTrials[MAX_R3P_TRIALS];
int f_r3pNTrials = 0;

void r3p_clear()
{
	int i;
	for (i=0; i<f_r3pNTrials; i++)
	{
		free(f_r3pTrials[i].pwent);
		f_r3pTrials[i].pwent = NULL;
		f_r3pTrials[i].nwent = 0;
	}
	f_r3pNTrials = 0;
	return;
}

int r3p_get_n_trials()
{
	return f_r3pNTrials;
}


int r3p_get_next_trial_min(FILE *fp, R3PTrialP ptrial,int min_trial_length_frames)
{
	int size;
	int bytes_read;
	int nframes;
	int nframes_read = 0;
	int status = 0;
	while (1)
	{
		if (fread(&size, sizeof(int), 1, fp) != 1) 
		{
			printf("End of file\n");
			break;	/* end of file */
		}
	
		/* 
		 * Size tells us how many frames are in the trial. 
		 * First, subtract off the size of the header. All that's left after that
		 * are went structs. 
		 */
		 
		nframes = (size - (int)sizeof(R3PHeader))/(int)sizeof(R3PWent);
		printf("Trial has %d frames\n", nframes);
		if (min_trial_length_frames < 1 || nframes >= min_trial_length_frames)
		{
			/*
			 * Allocate space for went data 
			 */
			ptrial->pwent = (R3PWent *)calloc(nframes, sizeof(R3PWent));
			
			/* 
			 * Read header then wents
			 */
			
			fread(&ptrial->header, sizeof(struct r3p_header), 1, fp);
			bytes_read = sizeof(struct r3p_header);
			nframes_read = 0;
			while (nframes_read < nframes)
			{
				if (fread(ptrial->pwent+nframes_read, sizeof(struct went_data), 1, fp) == 1)
				{
					nframes_read++;
				}
			}
			ptrial->nwent = nframes_read;
			printf("Read %d wents\n", nframes_read);
			status = 1;
			break;
		}
		else
		{
			fseek(fp, size, SEEK_CUR);
			printf("Skipped %d bytes\n", size);
		}
	}
	return status;
}	

int r3p_get_trials_min(FILE *pfile, int min_trial_length_frames)
{
	int n = 0;	/* Number loaded this call */
	while (f_r3pNTrials < MAX_R3P_TRIALS &&  r3p_get_next_trial_min(pfile, f_r3pTrials+f_r3pNTrials, min_trial_length_frames))
	{
		f_r3pNTrials++;
		n++;
	}
	return f_r3pNTrials;
}

R3PTrialP r3p_get_trial(int i)
{
	if (i > -1 && i < f_r3pNTrials)
	{
		return f_r3pTrials+i;
	}
	else
	{
		return (R3PTrialP)NULL;
	}
}

#ifndef ST20AF_RTVARS_H_
#define ST20AF_RTVARS_H_

/*
 * This file TODO: is automatically generated.
 * Include this file in your paradigm. It doesn't matter much where you place it
 * in the source file, just use this line:
 *
 * #include "TODO: thisfile.h"
 *
 * and make sure that thisfile.h is included in your build (having it in the same
 * folder as your source (*.d) file should do it).
 *
 * You must make one function call (the function, int rtvarex_init() is
 * defined in this file) somewhere in your paradigm's initialization.
 * Its OK to call the function multiple times, so don't worry about
 * guard logic - its taken care of.
 *
 * The function rtvarex_init() returns 0 if its successful, and a nonzero
 * value if it fails. That would mean a problem in the generation of this
 * file.
 *
 * rtvarex_init() creates the storage and linkages necessary for you to maintain
 * counters and recorders in your paradigm. If they've been created already, they
 * are not re-created. Note that they are NOT CLEARED - you must do that explicitly
 * if needed. The macro to clear a recorder is
 *
 * CLEAR_RECORDER(RecorderIDHere);
 *
 * Use the same ID you used in the rtvarex input text file. Similarly, to clear
 * a counter:
 *
 * CLEAR_COUNTER(CounterIDHere);
 *
 * Incrementing a counter:
 *
 * INCREMENT(CounterIDHere);
 *
 * Adding a value "B" to a counter:
 *
 * ADDTO(CounterIDHere, B);
 *
 * Putting a value in a recorder:
 *
 * RECORD(RecorderIDHere, value);
 *
 *
 * The macros for getting values out of counters and recorders also use the same ID
 * you used in the rtvarex input text file. To get the current value of a counter:
 *
 * intVar = GETCOUNT(CounterIDHere);
 *
 * To get avg or std from a recorder,
 *
 * GETREC(RecorderIDHere, &floatAvg, &floatStd) fcounter_get_recorder(f_pfcounter, f_fcind_ ##, ##, ##);
 *
 * Here you must include the ampersand - the args are pointers. You can, however, use NULL for either
 * if you don't need that particular value. (Computing the standard deviation requires a second pass
 * through the values after computing the average and so is computationally more expensive - how much
 * depends on how many values are in your recorder).
 */


#include "fcounter.h"

/* Counter stuff */
PFCounter f_pfcounter = NULL;

/* Usage macros */

#define INCREMENT(A) fcounter_increment_counter(f_pfcounter, f_ifcind_ ## A)
#define ADDTO(A, B) fcounter_addto_counter(f_pfcounter, f_ifcind_ ## A, B)

#define GETCOUNT(A) fcounter_get_counter(f_pfcounter, f_ifcind_ ## A)
#define GETREC(A, B, C) fcounter_get_recorder(f_pfcounter, f_fcind_ ##, ##, ##);

#define RECORD(A, B) fcounter_record(f_pfcounter, f_ifcind_ ## A, B)
#define CLEAR_RECORDER(A) fcounter_clear_recorder(f_pfcounter, f_ifcind_ ## A);


/*
 * Macro to trigger computation of rtvars -- including averages. This forces averages to be computed and
 * their corresponding RTVARs to be updated. Call this to update the RTVARS associated with "avg" and
 * "recorder" type counters.
 *
 * Counter type RTVARS are updated automatically every time you increment them. If you only use counter
 * type RTVARS then there is no need to use this macro. If, on the other hand, you use recorder or avg
 * type RTVARS, the REX "Real Time Variables" dialog values associated with them will NOT be updated
 * unless you call this macro. Since computation of averages and stdev's may be costly, you may defer
 * their computation to down time between trials, for example.
 */

#define RTVARS_COMPUTE() fcounter_rtvar_trigger(f_pfcounter, 1)
#define RTVARS_TRIGGER() fcounter_rtvar_trigger(f_pfcounter, 1)



/*
 * AllTrials counter
 */
int	f_ifcind_AllTrials = 0;
char *f_sident_AllTrials = "AllTrials";

/*
 * CompleteTrials counter
 */
int	f_ifcind_CompleteTrials = 0;
char *f_sident_CompleteTrials = "CompleteTrials";

/*
 * BreakFix counter
 */
int	f_ifcind_BreakFix = 0;
char *f_sident_BreakFix = "BreakFix";

/*
 * Pause counter
 */
int	f_ifcind_Pause = 0;
char *f_sident_Pause = "Pause";

/*
 * GoodJumps counter
 */
int	f_ifcind_GoodJumps = 0;
char *f_sident_GoodJumps = "GoodJumps";

/*
 * AllJumps counter
 */
int	f_ifcind_AllJumps = 0;
char *f_sident_AllJumps = "AllJumps";

/*
 * LeftJumps counter
 */
int	f_ifcind_LeftJumps = 0;
char *f_sident_LeftJumps = "LeftJumps";

/*
 * RightJumps counter
 */
int	f_ifcind_RightJumps = 0;
char *f_sident_RightJumps = "RightJumps";

/*
 * TrialLength recorder
 */
int	f_ifcind_TrialLength = 0;
int	f_imxrec_TrialLength = 1000;
char *f_sident_TrialLength = "TrialLength";

/*
 * Reacquisition recorder
 */
int	f_ifcind_Reacquisition = 0;
int	f_imxrec_Reacquisition = 1000;
char *f_sident_Reacquisition = "Reacquisition";

/*
 * AbsSteerError recorder
 */
int	f_ifcind_AbsSteerError = 0;
int	f_imxrec_AbsSteerError = 5000;
char *f_sident_AbsSteerError = "AbsSteerError";

/*
 * AbsSteerErrorCumulativeTime counter
 */
int	f_ifcind_AbsSteerErrorCumulativeTime = 0;
char *f_sident_AbsSteerErrorCumulativeTime = "AbsSteerErrorCumulativeTime";

/*
 * TotalRewardCounter counter
 */
int	f_ifcind_TotalRewardCounter = 0;
char *f_sident_TotalRewardCounter = "TotalRewardCounter";


/*
 * RTVARS variable declarations
 */
int	f_irtcnt_AllTrials = 0;
int	f_irtavg_TrialLength = 0;
float	f_favgfc_TrialLength = 1.000000;
int	f_irtpct_CompleteTrialsAllTrials = 0;
int	f_irtcnt_AllJumps = 0;
int	f_irtcnt_RightJumps = 0;
int	f_irtcnt_LeftJumps = 0;
int	f_irtcnt_GoodJumps = 0;
int	f_irtavg_Reacquisition = 0;
float	f_favgfc_Reacquisition = 1.000000;
int	f_irtavg_AbsSteerError = 0;
float	f_favgfc_AbsSteerError = 100.000000;
int	f_irtcnt_AbsSteerErrorCumulativeTime = 0;
int	f_irtcnt_TotalRewardCounter = 0;
RTVAR rtvars[] = {
	{ "All trials", &f_irtcnt_AllTrials },
	{ "avg Trial Length", &f_irtavg_TrialLength },
	{ "% Complete", &f_irtpct_CompleteTrialsAllTrials },
	{ "All jump counter", &f_irtcnt_AllJumps },
	{ "Right jump counter", &f_irtcnt_RightJumps },
	{ "Left jump counter", &f_irtcnt_LeftJumps },
	{ "Good jump counter", &f_irtcnt_GoodJumps },
	{ "Reacq. time", &f_irtavg_Reacquisition },
	{ "AbsSteerErr", &f_irtavg_AbsSteerError },
	{ "SteerErrSumTime", &f_irtcnt_AbsSteerErrorCumulativeTime },
	{ "Reward Counter", &f_irtcnt_TotalRewardCounter },
	{"", 0},
};

/*
 * rtvarex_init()
 *
 * Call this function to initialize rtvars and counters. 
 * You may safely call this function multiple times.
 */

int rtvarex_init()
{
	int status=0;

	if (!f_pfcounter)
	{
		status = -1;
		f_pfcounter = fcounter_create(2, 1);
		if (f_pfcounter)
		{

			/*
			 * "AllTrials" counter
			 */

			f_ifcind_AllTrials = fcounter_init_counter(f_pfcounter);
			if (f_ifcind_AllTrials < 0) return -1;

			/*
			 * "CompleteTrials" counter
			 */

			f_ifcind_CompleteTrials = fcounter_init_counter(f_pfcounter);
			if (f_ifcind_CompleteTrials < 0) return -1;

			/*
			 * "BreakFix" counter
			 */

			f_ifcind_BreakFix = fcounter_init_counter(f_pfcounter);
			if (f_ifcind_BreakFix < 0) return -1;

			/*
			 * "Pause" counter
			 */

			f_ifcind_Pause = fcounter_init_counter(f_pfcounter);
			if (f_ifcind_Pause < 0) return -1;

			/*
			 * "GoodJumps" counter
			 */

			f_ifcind_GoodJumps = fcounter_init_counter(f_pfcounter);
			if (f_ifcind_GoodJumps < 0) return -1;

			/*
			 * "AllJumps" counter
			 */

			f_ifcind_AllJumps = fcounter_init_counter(f_pfcounter);
			if (f_ifcind_AllJumps < 0) return -1;

			/*
			 * "LeftJumps" counter
			 */

			f_ifcind_LeftJumps = fcounter_init_counter(f_pfcounter);
			if (f_ifcind_LeftJumps < 0) return -1;

			/*
			 * "RightJumps" counter
			 */

			f_ifcind_RightJumps = fcounter_init_counter(f_pfcounter);
			if (f_ifcind_RightJumps < 0) return -1;

			/*
			 * "TrialLength" recorder
			 */

			f_ifcind_TrialLength = fcounter_init_recorder(f_pfcounter, f_imxrec_TrialLength);
			if (f_ifcind_TrialLength < 0) return -1;

			/*
			 * "Reacquisition" recorder
			 */

			f_ifcind_Reacquisition = fcounter_init_recorder(f_pfcounter, f_imxrec_Reacquisition);
			if (f_ifcind_Reacquisition < 0) return -1;

			/*
			 * "AbsSteerError" recorder
			 */

			f_ifcind_AbsSteerError = fcounter_init_recorder(f_pfcounter, f_imxrec_AbsSteerError);
			if (f_ifcind_AbsSteerError < 0) return -1;

			/*
			 * "AbsSteerErrorCumulativeTime" counter
			 */

			f_ifcind_AbsSteerErrorCumulativeTime = fcounter_init_counter(f_pfcounter);
			if (f_ifcind_AbsSteerErrorCumulativeTime < 0) return -1;

			/*
			 * "TotalRewardCounter" counter
			 */

			f_ifcind_TotalRewardCounter = fcounter_init_counter(f_pfcounter);
			if (f_ifcind_TotalRewardCounter < 0) return -1;

			/*
			 * rtvar "All trials" (counter)
			 */

			fcounter_rtvar_counter(f_pfcounter, &f_irtcnt_AllTrials, f_ifcind_AllTrials);

			/*
			 * rtvar "avg Trial Length" (avg)
			 */

			fcounter_rtvar_avg(f_pfcounter, &f_irtavg_TrialLength, f_favgfc_TrialLength, NULL, 0.0f, f_ifcind_TrialLength);

			/*
			 * rtvar "% Complete" (pct)
			 */

			fcounter_rtvar_pct(f_pfcounter, &f_irtpct_CompleteTrialsAllTrials, f_ifcind_CompleteTrials, f_ifcind_AllTrials);

			/*
			 * rtvar "All jump counter" (counter)
			 */

			fcounter_rtvar_counter(f_pfcounter, &f_irtcnt_AllJumps, f_ifcind_AllJumps);

			/*
			 * rtvar "Right jump counter" (counter)
			 */

			fcounter_rtvar_counter(f_pfcounter, &f_irtcnt_RightJumps, f_ifcind_RightJumps);

			/*
			 * rtvar "Left jump counter" (counter)
			 */

			fcounter_rtvar_counter(f_pfcounter, &f_irtcnt_LeftJumps, f_ifcind_LeftJumps);

			/*
			 * rtvar "Good jump counter" (counter)
			 */

			fcounter_rtvar_counter(f_pfcounter, &f_irtcnt_GoodJumps, f_ifcind_GoodJumps);

			/*
			 * rtvar "Reacq. time" (avg)
			 */

			fcounter_rtvar_avg(f_pfcounter, &f_irtavg_Reacquisition, f_favgfc_Reacquisition, NULL, 0.0f, f_ifcind_Reacquisition);

			/*
			 * rtvar "AbsSteerErr" (avg)
			 */

			fcounter_rtvar_avg(f_pfcounter, &f_irtavg_AbsSteerError, f_favgfc_AbsSteerError, NULL, 0.0f, f_ifcind_AbsSteerError);

			/*
			 * rtvar "SteerErrSumTime" (counter)
			 */

			fcounter_rtvar_counter(f_pfcounter, &f_irtcnt_AbsSteerErrorCumulativeTime, f_ifcind_AbsSteerErrorCumulativeTime);

			/*
			 * rtvar "Reward Counter" (counter)
			 */

			fcounter_rtvar_counter(f_pfcounter, &f_irtcnt_TotalRewardCounter, f_ifcind_TotalRewardCounter);

			status = 0;
		}
	}
	return status;
}
#endif /* ST20AF_RTVARS_H_ */


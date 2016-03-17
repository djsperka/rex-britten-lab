/* $Id: */

/*
 * Reward calibration paradigm. See wiki for docs. 
 */



#include <math.h>
#include "bcode.h"
#include "ldev.h"	/* hardware device defines */
#include "lcode.h"	/* Ecodes */


/*
 * Counters and control vars
 */
int f_trials_remain = 0;			/* number of trials remaining */
int f_ntrials = 0;				/* Number of trials requested */
int f_reward_preset = 50;		/* preset value for rewward */
int f_reward_random = 50;		/* random value for rewward */
int f_intertrial_time = 500;
int f_reward_counter = 0;
int f_never = 0;

/* REX menu declarations */
VLIST state_vl[] = {
"number_of_rewards", &f_ntrials, NP, NP, 0, ME_DEC,
"reward_preset", &f_reward_preset, NP, NP, 0, ME_DEC,
"reward_random", &f_reward_random, NP, NP, 0, ME_DEC,
"time_between_rewards", &f_intertrial_time, NP, NP, 0, ME_DEC,
NS,
};

char hm_sv_vl[] = 	"Choose number of rewards, reward preset and random\n\n"
					"Each reward is equal to reward_preset plus a random value\n"
					"between 0 and reward_random.\n"
					"A new random value is chosen each time a reward is given.\n"
					"Reward_random value may be 0.\n\n"
					"When running multiple times, be sure to RESET.\n";


MENU umenus[] = {
{"Main", &state_vl, NP, NP, 0, NP, hm_sv_vl}, 
{NS},
};




/* my_exp_init *********************************************
 * 
 * Initializations for overall experiment. Trial counters...
 * 
 */
 
void my_exp_init()
{
	dprintf("\n============================\n\nrewardcal\nInitializing, clearing counters...\n");
	f_trials_remain = f_ntrials;
	f_reward_counter = 0;
	return;
}

int my_trial_init()
{
	set_times("reward_on", (long)f_reward_preset, (long)f_reward_random);
	set_times("trial_pause", (long)f_intertrial_time, (long)NULL);
	return 0;
}

int my_trial_done()
{
	print_status();
	return 0;
}
   
int reward_on()
{
	dio_on(REW);
	f_reward_counter++;
	return 0;
}


void print_status()
{
	dprintf("rewardcal status: preset/random %d/%d    rewards delivered %d\n", f_reward_preset, f_reward_random, f_reward_counter);
	return;
}


int all_done()
{
	dprintf("\nAll done!\n");
	print_status();
	dprintf("============================\n");
	return 0;
}

int paused()
{
	dprintf("\nPaused!\n");
	print_status();
	dprintf("============================\n");
	return 0;
}


/* REX state set */

%%
id 999
restart my_exp_init
main_set {
status ON
begin	first:
		to p1 
	p1:
		to p2 on +PSTOP & softswitch
		to trial_init on -PSTOP & softswitch
	p2:	
		to trial_init on -PSTOP & softswitch
	trial_init:
		do my_trial_init()
		to reward_on
	reward_on:
		do reward_on()
		time 50
		rand 50
		to reward_off
	reward_off:
		dio_off(REW)
		to trial_done
	trial_done:
		do my_trial_done()
		to paused on +PSTOP & softswitch
		to trial_pause
	trial_pause:
		time 500
		to trial_check
	trial_check:
		to all_done on 1 ? f_trials_remain
		to trial_init
	paused:
		do paused()
		to trial_pause on -PSTOP & softswitch
	all_done:
		do all_done()
		to first on 1 = f_never
abort	list:
		all_done
}


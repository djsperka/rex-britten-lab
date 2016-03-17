#include "ldev.h"	/* hardware device defines */


int rew_wait_counter = 0;
int rew_counter = 100;
int rew_menu = 100;

VLIST state_vl[] = {
"number of rewards",		&rew_menu, NP, NP, 0, ME_DEC,
NS,
};

/*
 * Help message.
 */
char hm_sv_vl[] = "";

void rinitf()
{
}

int rewinit()
{
	rew_counter = rew_menu;
	return 0;
}
	

int rewstat()
{
	rew_counter--;
	dprintf("rew_counter = %d\n", rew_counter);
	return 0;
}


// rew has been given. 
// decrement counter, check if its 0. 
int rew_update()
{
	rew_wait_counter = 100;
	rew_counter --;
	dprintf("rew_counter=%d\n", rew_counter);
	return 0;
}


%%

/*
 * Auxiliary state set for processing reward contingency.
 */
id 300
restart rinitf
main_set {
status ON
begin	first:
		to p7
	p7:
		to p8 on +PSTOP & softswitch
		to rewinit on -PSTOP & softswitch
	p8:
		code PAUSECD
		to rewinit on -PSTOP & softswitch
	rewinit:
		do rewinit()
		to rewupdate
	rewupdate:
		do rew_update()
		to reward
	reward:
		dio_on(REW)
		time 25
		to rewoff
	rewoff:
		dio_off(REW)
		to rewwait
	rewwait:
		time 500
		to rewdone
	rewdone:
		to rewupdate on 0<rew_counter
}

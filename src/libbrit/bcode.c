#include <stdio.h>
#include <sys/types.h>
#include "rexhdrs.h"
#include "bcode.h"

/* this value used internally to indicate whether the time marker has been
 * recorded.
 */

static long f_time=0;

/* 
 * bcode
 * 
 * pushes an event into the ecode buffer. The "key" is what is ordinarily 
 * the time. The pointer pkey should point to a 4-byte value. 
 */

static void bcode(short int ecode, void *pkey)
{
	EVENT ev;
	ev.e_code = ecode;
	ev.e_key = *(long *)pkey;
	ldevent(&ev);
	return;
}
	

/* 
 * bcode_internal
 * 
 * Checks that a bcode marker has been recorded for the current time. Called
 * internally for each of the public bcode_* functions to ensure a marker is
 * recorded each time a bcode is recorded. 
 */
static void bcode_internal()
{
	if (f_time != i_b->i_time)
	{
		bcode(BCODE_MARK, (void *)&i_b->i_time);
		f_time = i_b->i_time;
	}
}

/*
 * bcode_float
 * 
 * Records a floating point value. 
 */
void bcode_float(unsigned char channel, float value)
{
	bcode_internal();
	bcode(BCODE_FLOAT | channel, (void *)&value);
}

/*
 * bcode_int
 * 
 * Records an int value. 
 */
void bcode_int(unsigned char channel, int value)
{
	bcode_internal();
	bcode(BCODE_INT | channel, (void *)&value);
}

/*
 * bcode_int
 * 
 * Records an unsigned int value. 
 */
void bcode_uint(unsigned char channel, unsigned int value)
{
	bcode_internal();
	bcode(BCODE_UINT | channel, (void *)&value);
}





/* ecode
 * Drops an ecode value to efile. This is the "ordinary" use of ecodes - uses
 * the current time. Returns 0 on success, or -1 on failure (if the ecode is
 * outside the range allowed). 
 */

int ecode(int icode)
{
	EVENT ev;
	int status=0;
	
	if (icode < 1501 || icode > 8192)
	{
//		dprintf("WARNING: ecode out of range %d\n", icode);
		status = -1;
	}
	
	ev.e_code= (short int)icode;
	ev.e_key= i_b->i_time;
	ldevent(&ev);
	return status;
}		

/* ecode
 * Drops an ecode value to efile. The floating point value is scaled (i.e.
 * multiplied by) the factor, then cast to an int before it is recorded. 
 * This is the "ordinary" use of ecodes - uses the current time.
 */
int ecode_multiplier(int basecd, float value, int factor)
{
	int icode = basecd + (int)(value * factor);
	return ecode(icode);
}




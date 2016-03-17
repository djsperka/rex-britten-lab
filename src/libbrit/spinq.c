#include <stdlib.h>
#include <string.h>
#include "spinq.h"

static int f_debug = 0;

struct spinq_struct
{
	int *pj;
	int jsize;
	int *pq;
	int qsize;
	int qt;		/* top of the queue. this is the next value to go */
	int qb;		/* bottom of the queue - new value added after this */
	int qw;		/* window for spinning */
};


void spinq_dump(PSpinQ p)
{
	dprintf("jsize %d\n", p->jsize);
	dprintf("qsize %d\n", p->qsize);
	dprintf("qt=%d qb=%d\n", p->qt, p->qb);
}

void spinq_debug(int d)
{
	f_debug = d;
	if (f_debug) dprintf("spinq_debug ON\n");
}

/* 
 * Allocate spin queue struct and histogram. Queue is not allocated! 
 * Call spinq_init for that!!!
 */
 
PSpinQ spinq_create(int jsize)
{
	int i;
	PSpinQ p = (PSpinQ)malloc(sizeof(struct spinq_struct));
	if (!p)
	{
		if (f_debug) dprintf("spinq_create: cannot malloc!\n");
		return (PSpinQ)NULL;
	}
	memset(p, 0, sizeof(struct spinq_struct));

	p->pj = (int *)calloc(jsize, sizeof(int));
	if (!p->pj)
	{
		free(p);
		if (f_debug) dprintf("spinq_create: cannot calloc %d ints\n", jsize);
		return (PSpinQ)NULL;
	}
	for (i=0; i<jsize; i++)
	{
		p->pj[i] = 0;
	}
	p->jsize = jsize;
	if (f_debug) dprintf("spinq_create: created spinq size %d\n", p->jsize);
	return p;
}

void spinq_destroy(PSpinQ p)
{
	if (p)
	{
		if (p->pq) free(p->pq);
		if (p->pj) free(p->pj);
		free(p);
	}
	return;
}


/*
 * Inelegant resize. If size as passed is not the same, the old q is cleared 
 * and a new one is allocated. No attempt is made to preserve the previous q's
 * info. Sorry. This should be called on trial_init, e.g., not during a trial.
 */
int spinq_init(PSpinQ p, int qsize, int window)
{
	if (f_debug) dprintf("spinq_init qsize=%d\n", qsize);
	if (qsize <= 0) return -1;
	if (p->pq && qsize != p->qsize)
	{
		if (f_debug) dprintf("spinq_init: new size %d old size %d free old queue\n", qsize, p->qsize);
		free(p->pq);
		p->pq = NULL;
	}
	if (qsize != p->qsize)
	{
		p->pq = (int *)calloc(qsize, sizeof(int));
		if (!p->pq) return -2;
		p->qsize = qsize;
		if (f_debug) dprintf("spinq_init: allocated new queue size %d\n", p->qsize);
	}
	p->qw = window;
	spinq_clear(p);
	
	if (f_debug)
	{
		dprintf("spinq_init: pj is %s\n", (p->pj ? "NOT NULL" : "NULL"));
		dprintf("spinq_init: pq is %s\n", (p->pq ? "NOT NULL" : "NULL"));
	}
	return 0;
}


/*
 * Clear the queue and zero histogram. 
 */

int spinq_clear(PSpinQ p)
{
	int i;
	for (i=0; i<p->jsize; i++)
	{
		p->pj[i] = 0;
	}
	for (i=0; i<p->qsize; i++)
	{
		p->pq[i] = -1;
	}
	p->qt = p->qb = 0;
	return 0;
}

/*
 * spinq_push
 * 
 * Push a value onto queue. After push, tests whether spinning is happening. 
 * Returns 1 if in a spin, 0 if not, -1 on error.
 */
 
int spinq_push(PSpinQ p, int joy)
{
	int status = -1;
 	
 	/*
 	 * Is the queue ready?
 	 */ 
 	
	if (p && p->pj && p->pq)
	{
		/* 
	 	 * Is the joy value in the allowed range?
	 	 */
	 	status = -2;
	 	if (joy >= 0 && joy < p->jsize)
	 	{
			int next;
			int range_min_index;
			int range_max_index;
			int sum = 0;
			int i;
	
			/* 
			 * Where does new value go? 
			 */
			next = p->qb+1;
			if (next == p->qsize) next = 0;
	
			/*
			 * Will the new value force out an old value?
			 * If so, remove that value from the histogram.
			 */
			if (next == p->qt)
			{
				p->pj[p->pq[p->qt]]--;
			}
			
			/*
			 * Assign the new value and increment the histogram. 
			 */
			p->pq[next] = joy;
			p->pj[joy]++;
			
			/* 
			 * Update top and bottom pointers.
			 */
			if (next == p->qt)
			{
				p->qt++;
				if (p->qt == p->qsize) p->qt = 0;
			}
			p->qb = next;
			
			/*
			 * Now test for spinning.Figure out the range to test, then we 
			 * just add up the histogram entries in that range. If that number
			 * equals the size of the queue we are spinning. 
			 */

			range_min_index = p->pq[p->qt] - p->qw;
			if (range_min_index < 0) range_min_index = 0;
			range_max_index = p->pq[p->qt] + p->qw;
			if (range_max_index >= p->jsize) range_max_index = p->jsize-1;
			for (i=range_min_index; i<= range_max_index; i++)
			{
				sum += p->pj[i];
			}
			if (f_debug) 
			{
				dprintf("spinq_push: range %d-%d sum %d qsize %d\n", range_min_index, range_max_index, sum, p->qsize);
			}
			if (sum == p->qsize) status = 1;
			else status = 0;
	 	}
	}
	else
	{
		if (f_debug)
		{
			if (!p) dprintf("spinq_push: p is NULL\n");
			if (!p->pj) dprintf("spinq_push: p->pj is NULL\n");
			if (!p->pq) dprintf("spinq_push: p->pq is NULL\n");
		}
	}
	return status;
}		 

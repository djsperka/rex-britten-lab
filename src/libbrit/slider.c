#include <stdlib.h>
#include <limits.h>

#include "slider.h"

/* 
 * Lets define the opaque struct now. 
 * 
 * The 'head' of the sliding window is the oldest value in it. 
 * The 'next' index is where we will put the next value added. 
 * On initialization 'next' is -1 to indicate there's nothing in the array.
 * Once the sliding window is full, head == next, and those two values 
 * increase as elements are added to the window, but they always remain
 * the same. 
 * 
 */
struct sliding_window_priv
{
	long *v;
	long head, next, size;
	long sum;
};

/* static functions - these will be pointed to by the Slider struct. */
static long size(Slider *pslider);
static long wmax(Slider *pslider);
static long wmin(Slider *pslider);
static void clear(Slider *pslider);
static void wadd(Slider *pslider, long value);
static long isFull(Slider *pslider);
static long wat(Slider *pslider, int index);
static long wsum(Slider *pslider);

Slider *slider_create(long wsize)
{
	Slider *p = (Slider *)malloc(sizeof(Slider));
	
	if (!p) return NULL;
	memset(p, 0, sizeof(Slider));
	p->priv = (struct sliding_window_priv *)malloc(sizeof(struct sliding_window_priv));
	if (!p->priv) 
	{
		free(p);
		return NULL;
	}
	
	if (wsize > 0)
	{
		p->priv->v = (long *)calloc(wsize, sizeof(long));
		if (!p->priv->v)
		{
			free(p->priv);
			free(p);
			return NULL;
		}
		else
		{
			memset(p->priv->v, 0, wsize * sizeof(long));
		}	
	}
	else
	{
		p->priv->v = NULL;
		wsize = 0;
	}

	p->priv->head = 0;
	p->priv->next = -1;
	p->priv->size = wsize;
	clear(p);
	p->size = size;
	p->wmax = wmax;
	p->wmin = wmin;
	p->isFull = isFull;
	p->wadd = wadd;
	p->clear = clear;
	p->at = wat;
	p->wsum = wsum;

	return p;
}

void slider_destroy(Slider *pslider)
{
	if (pslider->priv->v) free(pslider->priv->v);
	free(pslider->priv);
	free(pslider);
	return;
}

void slider_dump(Slider *pslider)
{
	int i;
	int m = (int)pslider->size(pslider);
	printf("Slider size %ld\n", pslider->size(pslider));
	printf("Head=%ld next=%ld\n", pslider->priv->head, pslider->priv->next);
	for (i=0; i<m; i++)
	{
		printf("%3d\t%10ld\n", i, pslider->priv->v[i]);
	}
	return;
}	


/* static methods */
static long size(Slider *pslider)
{
	return pslider->priv->size;
}

static void wadd(Slider *pslider, long value)
{
	if (pslider->size(pslider)==0) return;
	
	/* 
	 * If next<0, the slider is empty.
	 * If not, then check whether next==size. If so, the next value goes in [0] and the new next value is 1. 
	 */
	if (pslider->priv->next < 0)
	{
		pslider->priv->v[0] = value;
	 	pslider->priv->head = 0;
	 	pslider->priv->next = 1;
	 	pslider->priv->sum = value;
	}
	else
	{ 
		long save_old_value = pslider->priv->v[pslider->priv->next];
		pslider->priv->sum += value;
		pslider->priv->v[pslider->priv->next] = value;

		/* Update head and next. If they were equal, then they both increment. If not, only next increments. */
		if (pslider->priv->head == pslider->priv->next)
		{
			pslider->priv->head++;
			if (pslider->priv->head == pslider->priv->size) pslider->priv->head = 0;
			pslider->priv->next = pslider->priv->head;
			pslider->priv->sum -= save_old_value;
		}
		else
		{
			pslider->priv->next++;
			if (pslider->priv->next == pslider->priv->size) pslider->priv->next = 0;
		}
	}
	return;
}

static long isFull(Slider *pslider)
{
	long status=0;
	if (pslider->size(pslider)>0 && pslider->priv->next == pslider->priv->head)
	{
		status = 1;
	}
	return status;
}

static void clear(Slider *pslider)
{
	pslider->priv->head = 0;
	pslider->priv->next = -1;
	pslider->priv->sum = 0;
	return;
}

static long wmax(Slider *pslider)
{
	long i;
	long itemp = LONG_MIN;
	for (i=0; i< (isFull(pslider) ? pslider->priv->size : pslider->priv->next); i++)
		if (pslider->priv->v[i] > itemp) itemp = pslider->priv->v[i];
	return itemp;
}		
	
static long wmin(Slider *pslider)
{
	long i;
	long itemp = LONG_MAX;
	for (i=0; i< (isFull(pslider) ? pslider->priv->size : pslider->priv->next); i++)
		if (pslider->priv->v[i] < itemp) itemp = pslider->priv->v[i];
	return itemp;
}		
		
static long wat(Slider *pslider, int index)
{
	long value = LONG_MIN;
	if (index < pslider->priv->size && index >= 0)
	{
		int actual = (pslider->priv->head + index) % pslider->priv->size;
		if (isFull(pslider) || actual < pslider->priv->head)
		{
			value = pslider->priv->v[actual];
		}
	}
	return value;
}

static long wsum(Slider *pslider)
{
	return pslider->priv->sum;
}

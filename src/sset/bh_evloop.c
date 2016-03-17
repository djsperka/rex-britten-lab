#include <stdio.h>
#include <sys/types.h>
#include "../hdr/sys.h"
#include "../hdr/cnf.h"
#include "../hdr/proc.h"
#include "../hdr/cdsp.h"
#include "../hdr/idsp.h"
#include "../hdr/buf.h"
#include "../hdr/menu.h"
#include "../hdr/state.h"
#include "../hdr/ecode.h"
#include "../hdr/device.h"
#include "../hdr/int.h"

#include "bh_evloop.h"
#include "slider.h"

static int f_last_good_x = 0;
static int f_last_good_y = 0;
static int f_last_good_is_valid = 0;
static int f_last_delta_x = 0;
static int f_last_delta_y = 0;
static int f_last_delta_is_ok = 0;
static float f_cum_pos_x = 0;
static float f_cum_pos_y = 0;
static float f_cum_pos_is_initialized = 0;
static PSlider f_psliderXPos = (PSlider)NULL;
static PSlider f_psliderYPos = (PSlider)NULL;
static PSlider f_psliderTime = (PSlider)NULL;


int evloop_init(int wsize)
{
	f_psliderXPos = slider_create(wsize);
	f_psliderYPos = slider_create(wsize);
	f_psliderTime = slider_create(wsize);
	return 0;
}

int evloop_fill()
{
	int status = 0;
	if (f_psliderXPos)
	{
		f_psliderXPos->wadd(f_psliderXPos, eyeh);
		f_psliderYPos->wadd(f_psliderYPos, eyev);
		f_psliderTime->wadd(f_psliderTime, getClockTime());
	}
	return status;
}

int evloop_clear()
{
	int status = 0;
	dprintf("CLEAR\n");
	if (f_psliderXPos)
	{
		f_psliderXPos->clear(f_psliderXPos);
		f_psliderYPos->clear(f_psliderYPos);
		f_psliderTime->clear(f_psliderTime);
	}
	return status;
}	

int evloop_xy(float *px, float *py)
{
	int status = 0;
	if (f_psliderXPos->isFull(f_psliderXPos))
	{
		*px = (float)f_psliderXPos->wsum(f_psliderXPos) / f_psliderXPos->size(f_psliderXPos);
		*py = (float)f_psliderYPos->wsum(f_psliderYPos) / f_psliderYPos->size(f_psliderYPos);
	}
	else
		status = 1;
	return status;
}		
		
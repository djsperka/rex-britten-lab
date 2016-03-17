#include "bh_animhelper.h"
#include "bh_evloop.h"
#include "math.h"
#include "sac.h"

#define TO_RAD(x) x*M_PI/180.0
#define TO_DEG(x) x*180.0/M_PI
#define IPRT(x, y) (int)(x*y)


void set_eyew(PursuitTransformStruct *ptrans, float *eyewx, float *eyewy);



int initialize_bpsh_struct(BPSHStruct* pbpsh)
{
	int status = 0;

	pbpsh->beta = -pbpsh->x * sin(pbpsh->rho*M_PI/180.0) + pbpsh->y * cos(pbpsh->rho*M_PI/180.0);
	
	/* 
	 * See notes : phi0 = a - (n_eps + n_rho/2)vp 
	 * Also (7/11/13) Was using -a, not a. a===(fixpt) DOT uhat. 
	 */	
	 
	/* If this is simulated pursuit, then use vp=0 in setting phi0! */
	
	
	if (pbpsh->ptype == BPSH_PTYPE_SIMULATED)
	{
		pbpsh->phi0 = (pbpsh->x * cos(pbpsh->rho*M_PI/180.0) + pbpsh->y * sin(pbpsh->rho*M_PI/180.0));
	}
	else
	{		
		pbpsh->phi0 = (pbpsh->x * cos(pbpsh->rho*M_PI/180.0) + pbpsh->y * sin(pbpsh->rho*M_PI/180.0)) - 
						((pbpsh->n_all_done + pbpsh->n_pursuit_start)/2 - pbpsh->n_pre_pursuit_start)*pbpsh->vp;
	}
		
	/* heading step sizes depend on azimuth and elevation angles */
	pbpsh->dx = pbpsh->vh * sin(pbpsh->alpha * M_PI/180.0);
	pbpsh->dy = pbpsh->vh * sin(pbpsh->theta * M_PI/180.0) * cos(pbpsh->alpha * M_PI/180.0);
	pbpsh->dz = -pbpsh->vh * cos(pbpsh->theta * M_PI/180.0) * cos(pbpsh->alpha * M_PI/180.0);
	
	/* blink start/stop */
	if (pbpsh->n_blink > 0)
	{
		pbpsh->n_blink_start = (pbpsh->n_pursuit_start + pbpsh->n_all_done - pbpsh->n_blink)/2;
		pbpsh->n_blink_end = (pbpsh->n_pursuit_start + pbpsh->n_all_done + pbpsh->n_blink)/2;
	}
	
	/* retinal stabilization values */
	pbpsh->azcorrection = 0;
	pbpsh->elcorrection = 0;
	pbpsh->evOK = 0;
	pbpsh->evx = 0;
	pbpsh->evy = 0;
	pbpsh->last_good_is_valid = 0;
	pbpsh->last_delta_is_valid = 0;

	return status;
}


void print_bpsh(BPSHStruct *pbpsh)
{
	if (pbpsh->ttype == BPSH_TTYPE_NONE)
	{
		dprintf("Dots: NONE\n");
	}
	else
	{
		dprintf("Dots: Translation az=%d el=%d vh=%d/frame\n", IPRT(pbpsh->alpha, 100), IPRT(pbpsh->theta, 100), IPRT(pbpsh->vh, 100));
	}
	if (pbpsh->ptype == BPSH_TTYPE_NONE)
	{
		dprintf("Pursuit: None\n");
	}
	else 
	{
		switch (pbpsh->ptype)
		{
			case BPSH_PTYPE_PURSUIT:
			{
				dprintf("Pursuit: Yes : ");
				break;
			}
			case BPSH_PTYPE_SIMULATED:
			{
				dprintf("Pursuit: Yes, simulated : ");
				break;
			}
			case BPSH_PTYPE_RETSTAB:
			{
				dprintf("Pursuit: Yes, with retinal stabilization : ");
				break;
			}
			default:
			{
				dprintf("ERROR: unknown pursuit type %d\n", pbpsh->ptype);
				break;
			}
		}
		dprintf("rho=%d vp=%d\n", IPRT(pbpsh->rho, 100), IPRT(pbpsh->vp, 10000));
	}		
	dprintf("t1=%d p1=%d p2=%d blink=%d done=%d\n", pbpsh->n_translation_start, pbpsh->n_pre_pursuit_start, pbpsh->n_pursuit_start, pbpsh->n_blink, pbpsh->n_all_done);
	return;
}

int bpsh_step(BPSHStruct *pbpsh, int istep)
{
	int status = 0;
	int hstatus = 0;
	int pstatus = 0;
	float phi;
	int cam_update = 0;
	int ptrans_update = 0;
	int dot_update = 0;			/* for blinks - call render_onoff with dot_onoff value */
	int dot_onoff = 0;
	int ospace_onoff = 0;
	int ospace_update = 0;

	/* 
	 * Heading. Set camera position and look/up directions. The look/up directions may be
	 * overridden in the case of simulated pursuit.If ttype is none (no dots) do not turn dots on, 
	 * otherwise turn dots on.
	 */
	if (istep == 0)
	{
		pbpsh->cam.ex = pbpsh->cam.ey = pbpsh->cam.ez = 0;
		pbpsh->cam.dx = pbpsh->cam.dy = 0;
		pbpsh->cam.dz = -1;
		pbpsh->cam.ux = pbpsh->cam.uz = 0;
		pbpsh->cam.uy = 1;
		pbpsh->cam.flag = CAMERA_DEFAULT;
		cam_update = 1;		
	}
	else if (istep < pbpsh->n_translation_start)
	{
		/*
		 * Delay phase: no movement, no commands issued.
		 * On first frame/step of the delay (frame 1) return "1" status. This seems kinda dumb, I think,
		 * but its consistent with the motion case.....
		 */
		cam_update = 0; 
		if (istep == 1) 
		{
			if (pbpsh->ttype == BPSH_TTYPE_TRANSLATION)
			{
				ospace_onoff = HANDLE_ON;
				ospace_update = 1;
			}
			hstatus = 1;
		}
	}
	else if (istep < pbpsh->n_all_done)
	{
		int hstep = istep-pbpsh->n_translation_start;
		if (istep == pbpsh->n_translation_start) hstatus = 2;
		pbpsh->cam.ex = hstep * pbpsh->dx;
		pbpsh->cam.ey = hstep * pbpsh->dy;
		pbpsh->cam.ez = hstep * pbpsh->dz;
		pbpsh->cam.dx = pbpsh->cam.dy = 0; pbpsh->cam.dz = -1;
		pbpsh->cam.ux = pbpsh->cam.uz = 0; pbpsh->cam.uy = 1;
		pbpsh->cam.flag = CAMERA_DEFAULT;			/* This may be changed if simulated pursuit */
		cam_update = 1;
	}
	else
	{
		hstatus = -1;
	}

	/* 
	 * For plain heading, set ptrans on first pass to get dot in right position but don't 
	 * update after that. 
	 * For regular pursuit and simulated pursuit do updates all the way through.
	 * For pursuit with retinal stabilization do updates also - the dot moves as in pursuit.
	 */
	 
	if (pbpsh->ptype == BPSH_PTYPE_NONE)
	{
		if (istep == 0)
		{
			pbpsh->ptrans.phi = pbpsh->phi0;
			pbpsh->ptrans.beta = pbpsh->beta;
			pbpsh->ptrans.rho = pbpsh->rho;
			ptrans_update = 1;
		}
	}
	else
	{
		if (istep == 0)
		{
			/*
			 * Initialization step
			 */
			phi = pbpsh->phi0;
			switch (pbpsh->ptype)
			{
				case BPSH_PTYPE_SIMULATED:
				{
					pbpsh->cam.a0 = phi;		
					pbpsh->cam.a1 = pbpsh->beta;
					pbpsh->cam.a2 = pbpsh->rho;
					pbpsh->cam.flag = CAMERA_PURSUIT;
					cam_update = 1;
	
					/*
					 * In simulated pursuit, vp=0, but we still set the trans angles to place the 
					 * dot at the correct offset in the camera's frame. In simulated pursuit we
					 * only do this once - dot remains attached to camera at that offset. 
					 */
					 				
					pbpsh->ptrans.phi = phi;
					pbpsh->ptrans.beta = pbpsh->beta;
					pbpsh->ptrans.rho = pbpsh->rho;
					ptrans_update = 1;
					break;
				}
				case BPSH_PTYPE_PURSUIT:
				case BPSH_PTYPE_RETSTAB:
				{
					/* Place dot at fixpt ref position. See above. */
					pbpsh->ptrans.phi = phi;
					pbpsh->ptrans.beta = pbpsh->beta;
					pbpsh->ptrans.rho = pbpsh->rho;
					ptrans_update = 1;
					break;
				}
				default:
				{
					dprintf("ERROR - unknown ptype (%d) in bpsh_step.\n", pbpsh->ptype);
					break;
				}
			}
		}
		else if (istep < pbpsh->n_pre_pursuit_start)
		{
			/*
			 * Delay phase: no movement, no commands issued.
			 * On first frame/step of the delay (frame 1) return "1" status. This seems kinda dumb, I think,
			 * but its consistent with the motion case.....
			 */
			if (istep == 1) pstatus = 1;
		}
		else if (istep < pbpsh->n_all_done)
		{
			int pstep = istep - pbpsh->n_pre_pursuit_start;
			
			/*
			 * Movement phase. On first frame/step where motion occurs send "2" return status.
			 */

			if (istep == pbpsh->n_pre_pursuit_start) pstatus |= 2;
			else if (istep == pbpsh->n_pursuit_start) pstatus |= 4;
			phi = pbpsh->phi0 + pstep * pbpsh->vp;
			switch (pbpsh->ptype)
			{
				case BPSH_PTYPE_SIMULATED:
				{
					pbpsh->cam.a0 = phi;		
					pbpsh->cam.a1 = pbpsh->beta;
					pbpsh->cam.a2 = pbpsh->rho;
					pbpsh->cam.flag = CAMERA_PURSUIT;
					cam_update = 1;			
					break;
				}
				case BPSH_PTYPE_PURSUIT:
				{
					pbpsh->ptrans.phi = phi;
					pbpsh->ptrans.beta = pbpsh->beta;
					pbpsh->ptrans.rho = pbpsh->rho;
					ptrans_update = 1;
					break;
				}
				case BPSH_PTYPE_RETSTAB:
				{
					float x, y;
					int estatus;
					
					// Update dot position
					pbpsh->ptrans.phi = phi;
					pbpsh->ptrans.beta = pbpsh->beta;
					pbpsh->ptrans.rho = pbpsh->rho;
					ptrans_update = 1;

					// Get current x,y avg. If its 					
					if (!evloop_xy(&x, &y))
					{
						//dprintf("xy %d %d\n", IPRT(x, 10), IPRT(y, 10));
						if (pbpsh->last_good_is_valid)
						{
							//dprintf("using last good  %d %d\n", IPRT(pbpsh->last_good_x, 10), IPRT(pbpsh->last_good_y, 10));
							pbpsh->last_delta_x = x - pbpsh->last_good_x;
							pbpsh->last_delta_y = y - pbpsh->last_good_y;
							pbpsh->last_delta_is_valid = 1;
							pbpsh->last_good_x = x;
							pbpsh->last_good_y = y;
							pbpsh->last_good_is_valid = 1;
							pbpsh->azcorrection -= pbpsh->last_delta_x/40.0;
							pbpsh->elcorrection -= pbpsh->last_delta_y/40.0;
							//dprintf("ongoing delta %d %d\n", IPRT(pbpsh->last_delta_x, 10), IPRT(pbpsh->last_delta_y, 10));
						}
						else if (pbpsh->last_delta_is_valid)
						{
							pbpsh->azcorrection -= pbpsh->last_delta_x/40.0;
							pbpsh->elcorrection -= pbpsh->last_delta_y/40.0;
							pbpsh->last_good_x = x;
							pbpsh->last_good_y = y;
							pbpsh->last_good_is_valid = 1;
							//dprintf("xy ok last xy NOT delta %d %d\n", IPRT(pbpsh->last_delta_x, 10), IPRT(pbpsh->last_delta_y, 10));
						}
						else
						{
							//dprintf("xy ok last xy NOT delta NOT\n");
							pbpsh->last_good_x = x;
							pbpsh->last_good_y = y;
							pbpsh->last_good_is_valid = 1;
						}							
					}
					else
					{
						pbpsh->last_good_is_valid = 0;
						if (pbpsh->last_delta_is_valid)
						{
							//dprintf("XY BAD use last delta %d %d corr %d %d\n", IPRT(pbpsh->last_delta_x, 10), IPRT(pbpsh->last_delta_y, 10), IPRT(pbpsh->azcorrection, 10), IPRT(pbpsh->elcorrection, 10));
							pbpsh->azcorrection -= pbpsh->last_delta_x/40.0;
							pbpsh->elcorrection -= pbpsh->last_delta_y/40.0;
						}
						else
						{
							//dprintf("xy bad last delta bad NO CORR  %d %d\n", IPRT(pbpsh->azcorrection, 10), IPRT(pbpsh->elcorrection, 10));
						}	
					}	
					pbpsh->cam.a0 = pbpsh->azcorrection;
					pbpsh->cam.a1 = pbpsh->elcorrection;
					pbpsh->cam.flag = CAMERA_AZIMUTH;
					cam_update = 1;	
						
#if 0
					dprintf("evloop_xy  (%d,%d) %d(%d,%d) %d(%d,%d) (%d,%d)\n", 
					(int)(x*100), (int)(y*100), 
					pbpsh->last_good_is_valid,
					(int)(pbpsh->last_good_x * 100), (int)(pbpsh->last_good_y * 100),
					pbpsh->last_delta_is_valid,
					(int)(pbpsh->last_delta_x * 100), (int)(pbpsh->last_delta_y * 100),
					(int)(pbpsh->azcorrection * 100), (int)(pbpsh->elcorrection * 100));
#endif
#if 0
					// If there is a correction velocity, then apply it to the camera rotation.
					if (pbpsh->evType == 0)
					{
						if (pbpsh->evOK)
						{
							pbpsh->azcorrection += pbpsh->evx * pbpsh->evfactor;
							pbpsh->elcorrection += pbpsh->evy * pbpsh->evfactor;
						}
						pbpsh->cam.a0 = pbpsh->azcorrection;
						pbpsh->cam.a1 = pbpsh->elcorrection;
						pbpsh->cam.flag = CAMERA_AZIMUTH;
						cam_update = 1;	
					}
					else if (pbpsh->evType == 1)
					{
						// compute x and y directions using the values in evx and evy. 
						// those values are simply the sum of the last f_evWindow eye position values. 
						//dprintf("x %d y %d\n", pbpsh->evx, pbpsh->evy);
						pbpsh->cam.flag = CAMERA_DEFAULT;
						pbpsh->cam.dx = (float)pbpsh->evx/40.0;
						pbpsh->cam.dy = (float)pbpsh->evy/40.0;
						pbpsh->cam.dz = -1;
					}
#endif
					break;
				}
			}
			
			/* 
			 * Check for blink transitions. If n_blink is nonzero, it should sit in the middle of the pursuit
			 * period. At the first frame of the blink just turn off dot. At the end of the period, turn dot back on.
			 */
			 
			if (istep == pbpsh->n_blink_start)
			{
				pstatus |= 8;	/* blink begins */
				dot_onoff = HANDLE_OFF;
				dot_update = 1;
			}
			else if (istep == pbpsh->n_blink_end)
			{
				pstatus |= 16;	/* blink ends   */
				dot_onoff = HANDLE_ON;
				dot_update = 1;
			}
		}
		else
		{
			pstatus = -1;
		}
	}
	if (cam_update)
	{
		// update camera position and pointing direction
		render_camera_s(&pbpsh->cam);
	}
	
	
	if (ptrans_update)
	{
		// update pursuit trans - this affects position of dot
		render_update(pbpsh->hptrans, &pbpsh->ptrans, sizeof(PursuitTransformStruct), 0);

		// Compute position of dot on screen - eyewx, eyewy are used for eye window
		set_eyew(&pbpsh->ptrans, &pbpsh->eyewx, &pbpsh->eyewy);
	}
	if (dot_update)
	{
		// turn dot on or off for blink trials
		render_onoff(&pbpsh->hdot, dot_onoff, ONOFF_NO_FRAME);
	}
	if (ospace_update)
	{
		// turn ospace on when needed
		render_onoff(&pbpsh->hospace, ospace_onoff, ONOFF_NO_FRAME);
	}
	
	// return value holds bit values indicating key frames in the animation
	if (pstatus < 0 || hstatus < 0) return -1;
	else return ((pstatus&0xff) << 8) | (hstatus&0xff);
}

void set_eyew(PursuitTransformStruct *ptrans, float *eyewx, float *eyewy)
{
	float A, B, C;
	float c_phi, s_phi;
	float c_beta, s_beta;
	float c_rho, s_rho;

	c_phi = cos(TO_RAD(ptrans->phi));
	s_phi = sin(TO_RAD(ptrans->phi));
	c_beta = cos(TO_RAD(ptrans->beta));
	s_beta = sin(TO_RAD(ptrans->beta));
	c_rho = cos(TO_RAD(ptrans->rho));
	s_rho = sin(TO_RAD(ptrans->rho));
	
	/*
	 * See Notes. These values A, B , C are the z-components of the 
	 * pursuit rotation (i.e. the third column - which controls how the 
	 * zhat vector is rotated). 
	 */
	 
	A = -c_rho * s_phi + s_rho * s_beta * c_phi;
	B = -s_rho * s_phi - c_rho * s_beta * c_phi;
	C = c_beta * c_phi;

	/*
	 * The angles formed by the rotation of the zhat vector are atan2(A, C) and atan2(B, C).
	 * These are NOT the right screen coords, however! We have to use the inverse of those, 
	 * because the camera convention is to point along the -z axis. Remember to think of the 
	 * pursuit rotation as happening along the +z axis, but that we're rotating objects that 
	 * lay along the -z axis (i.e. the z position of the fixpt dot is negative initially).
	 */

	*eyewx = -1 * TO_DEG(atan2(A, C));	
	*eyewy = -1 * TO_DEG(atan2(B, C));	

#if 0
	dprintf("set_eyew rho=%d beta=%d phi=%d\n", IPRT(ptrans->rho, 100), IPRT(ptrans->beta, 100), IPRT(ptrans->phi, 100));
	dprintf("set_eyew A=%d B=%d C=%d  x=%d y=%d\n", IPRT(A, 100), IPRT(B, 100), IPRT(C, 100), IPRT(*eyewx, 100), IPRT(*eyewy, 100));
#endif
	
}
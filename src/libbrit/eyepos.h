#ifndef EYEPOS_H_
#define EYEPOS_H_

typedef struct eyepos_struct *PEyepos;
typedef struct eyepos_struct
{
	int (*size)(PEyepos ep);
	int (*getFP)(PEyepos ep, int iwhich, float *pfpx, float *pfpy);
	int (*getWD)(PEyepos ep, int iwhich, long *pwdx, long *pwdy);
	void *priv;
} Eyepos;

PEyepos eyepos_load_fp(char *filename);	
PEyepos eyepos_load_fp_and_wd(char *filename);
void eyepos_destroy(PEyepos ep);


#endif /*EYEPOS_H_*/

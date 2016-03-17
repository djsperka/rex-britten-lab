#include <stdlib.h>
#include <stdio.h>

#include "eyepos.h"

#define USE_RXERR 1

char f_str[1024];


struct eyepos_priv
{
	int num;
	float *pfpx;
	float *pfpy;
	long *pwdx;
	long *pwdy;
};


static int _eyepos_size(PEyepos ep);
static int _eyepos_get_fp(PEyepos ep, int iwhich, float *pfpx, float *pfpy);
static int _eyepos_get_wd(PEyepos ep, int iwhich, long *pwdx, long *pwdy);
static int _eyepos_load_file(FILE *pfile, struct eyepos_priv *ppriv, int numperline);


PEyepos eyepos_load_fp(char *filename)
{
	FILE *pfile=NULL;
	PEyepos pep = NULL;
	struct eyepos_priv *ppriv=(struct eyepos_priv *)NULL;

	pep = (PEyepos)malloc(sizeof(Eyepos));
	if (!pep)
	{
		sprintf(f_str, "eyepos: Cannot malloc.\n");
#if USE_RXERR
			rxerr(f_str);
#else
			printf("%s\n", f_str);
#endif
		return (PEyepos)NULL;
	}

	ppriv = (struct eyepos_priv *)malloc(sizeof(struct eyepos_priv));
	if (!ppriv)
	{
		sprintf(f_str, "eyepos: Cannot malloc.\n");
#if USE_RXERR
			rxerr(f_str);
#else
			printf("%s\n", f_str);
#endif
		free(pep);
		return (PEyepos)NULL;
	}
	pep->priv = ppriv;
		
	/*
	 * Open file. 
	 */

	pfile = fopen(filename, "r");
	if (!pfile)
	{
		sprintf(f_str, "eyepos: Cannot open eyepos config file %s. Check filename.\n", filename);
#if USE_RXERR
		rxerr(f_str);
#else
		printf("%s\n", f_str);
#endif
		free(ppriv);
		free(pep);
		return (PEyepos)NULL;
	}

	/*
	 * Now load file
	 */

	if (_eyepos_load_file(pfile, ppriv, 2))
	{
		free(ppriv);
		free(pep);
		fclose(pfile);
		return (PEyepos)NULL;
	}
	
	pep->size = _eyepos_size;
	pep->getFP = _eyepos_get_fp;
	pep->getWD = _eyepos_get_wd;
	
	return pep;
}
			

PEyepos eyepos_load_fp_and_wd(char *filename)
{
	FILE *pfile=NULL;
	PEyepos pep = NULL;
	struct eyepos_priv *ppriv=(struct eyepos_priv *)NULL;

	pep = (PEyepos)malloc(sizeof(Eyepos));
	if (!pep)
	{
		sprintf(f_str, "eyepos: Cannot malloc.\n");
#if USE_RXERR
			rxerr(f_str);
#else
			printf("%s\n", f_str);
#endif
		return (PEyepos)NULL;
	}

	ppriv = (struct eyepos_priv *)malloc(sizeof(struct eyepos_priv));
	if (!ppriv)
	{
		sprintf(f_str, "eyepos: Cannot malloc.\n");
#if USE_RXERR
			rxerr(f_str);
#else
			printf("%s\n", f_str);
#endif
		free(pep);
		return (PEyepos)NULL;
	}
	pep->priv = ppriv;
		
	/*
	 * Open file. 
	 */

	pfile = fopen(filename, "r");
	if (!pfile)
	{
		sprintf(f_str, "eyepos: Cannot open eyepos config file %s. Check filename.\n", filename);
#if USE_RXERR
		rxerr(f_str);
#else
		printf("%s\n", f_str);
#endif
		free(ppriv);
		free(pep);
		return (PEyepos)NULL;
	}

	/*
	 * Now load file
	 */

	if (_eyepos_load_file(pfile, ppriv, 4))
	{
		free(ppriv);
		free(pep);
		fclose(pfile);
		return (PEyepos)NULL;
	}
	
	pep->size = _eyepos_size;
	pep->getFP = _eyepos_get_fp;
	pep->getWD = _eyepos_get_wd;
	
	return pep;
}


			
static int _eyepos_load_file(FILE *pfile, struct eyepos_priv *ppriv, int numperline)
{
	int num, dum;
	int i;
	char line[256];

	/*
	 * Check input: numperline must be 2 or 4. 
	 */
	 
	if (numperline != 2 && numperline != 4)
	{
		sprintf(f_str, "eyepos: Numperline must be 2 or 4! (its %d)\n", numperline);
#if USE_RXERR
		rxerr(f_str);
#else
		printf("%s\n", f_str);
#endif
		return -1;
	}
			
	/*
	 * Read first line of file - should be a single number - the number of subesequent lines in file. 
	 */

	if (!fgets(line, 256, pfile))
	{
		sprintf(f_str, "eyepos: Cannot read first line from file\n", i);
#if USE_RXERR
		rxerr(f_str);
#else
		printf("%s\n", f_str);
#endif
		return -1;
	}
		
		 
	if ((i=sscanf(line, "%d %d", &num, &dum)) != 1)
	{
		sprintf(f_str, "eyepos: First line of config file should have # of positions in file (and nothing else!)\n", i);
#if USE_RXERR
		rxerr(f_str);
#else
		printf("%s\n", f_str);
#endif
		return -1;
	}
	
	
	ppriv->pfpx = (float *)calloc(num, sizeof(float));
	ppriv->pfpy = (float *)calloc(num, sizeof(float));
	ppriv->pwdx = (long *)calloc(num, sizeof(long));
	ppriv->pwdy = (long *)calloc(num, sizeof(long));
	ppriv->num = num;
	for (i=0; i<num; i++)
	{
		if (numperline == 2)
		{
			if (fscanf(pfile, "%f %f", ppriv->pfpx+i, ppriv->pfpy+i) != 2)
			{
				sprintf(f_str, "eyepos: Format error at line %d.\n", i+2);
#if USE_RXERR
				rxerr(f_str);
#else
				printf("%s\n", f_str);
#endif
				free(ppriv->pfpx);
				free(ppriv->pfpy);
				free(ppriv->pwdx);
				free(ppriv->pwdy);
				return -1;
			}
			else
			{
				ppriv->pwdx[i] = (int)(ppriv->pfpx[i]*10);
				ppriv->pwdy[i] = (int)(ppriv->pfpy[i]*10);
			}
		}
		else
		{
			if (fscanf(pfile, "%f %f %d %d", ppriv->pfpx+i, ppriv->pfpy+i, ppriv->pwdx+i, ppriv->pwdy+i) != 4)
			{
				sprintf(f_str, "eyepos: Format error at line %d.\n", i+2);
#if USE_RXERR
				rxerr(f_str);
#else
				printf("%s\n", f_str);
#endif
				free(ppriv->pfpx);
				free(ppriv->pfpy);
				free(ppriv->pwdx);
				free(ppriv->pwdy);
				return -1;
			}
		}			
	}
	return 0;
}


static int _eyepos_size(PEyepos ep)
{
	return ((struct eyepos_priv *)(ep->priv))->num;
}


static int _eyepos_get_fp(PEyepos ep, int iwhich, float *pfpx, float *pfpy)
{
	struct eyepos_priv *p = (struct eyepos_priv *)ep->priv;
	if (iwhich < 0 || iwhich >= p->num)
	{
		sprintf(f_str, "eyepos: requested fp (%d) out of range (0<=value<%d).\n", iwhich, p->num);
#if USE_RXERR
		rxerr(f_str);
#else
		printf("%s\n", f_str);
#endif
		return -1;
	} 
	*pfpx = p->pfpx[iwhich];
	*pfpy = p->pfpy[iwhich];
	return 0;
}

static int _eyepos_get_wd(PEyepos ep, int iwhich, long *pwdx, long *pwdy)
{
	struct eyepos_priv *p = (struct eyepos_priv *)ep->priv;
	if (iwhich < 0 || iwhich >= p->num)
	{
		sprintf(f_str, "eyepos: requested fp (%d) out of range (0<=value<%d).\n", iwhich, p->num);
#if USE_RXERR
		rxerr(f_str);
#else
		printf("%s\n", f_str);
#endif
		return -1;
	} 
	*pwdx = p->pwdx[iwhich];
	*pwdy = p->pwdy[iwhich];
	return 0;
}

void eyepos_destroy(PEyepos ep)
{
	struct eyepos_priv *p = (struct eyepos_priv *)ep->priv;
	free(p->pfpx);
	free(p->pfpy);
	free(p->pwdx);
	free(p->pwdy);
	free(ep->priv);
	free(ep);
	return;
}

	

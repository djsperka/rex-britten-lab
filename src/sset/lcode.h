/*
 *-----------------------------------------------------------------------*
 * NOTICE:  This code was developed by the US Government.  The original
 * versions, REX 1.0-3.12, were developed for the pdp11 architecture and
 * distributed without restrictions.  This version, REX 4.0, is a port of
 * the original version to the Intel 80x86 architecture.  This version is
 * distributed only under license agreement from the National Institutes 
 * of Health, Laboratory of Sensorimotor Research, Bldg 10 Rm 10C101, 
 * 9000 Rockville Pike, Bethesda, MD, 20892, (301) 496-9375.
 *-----------------------------------------------------------------------*
 */

/********************** local codes and defines ************************
 *
 * Some of this is derived from Newsome lab standards, some is new to
 * Britten lab at UC Davis. Generated 7/24/94. In the process, I got
 * rid of most of the old NIH-derived stuff to clear some Ecode room.
 *
 ***********************************************************************/

/*Condition used in branches in paradigms*/

#define MET	1

/*
 *	Event codes specific per laboratory.
 */
#define FPONCD		1010
#define FPOFFCD		1025
#define REWCD		1030

/*
 * if we record how long it takes to configure new stimuli
 */
#define CONFIG		1150	/* enter config states */
#define CONF_DONE	1151	/* slave is done */

/*
 * stimulus control timing
 */
#define STIMASK		1160	/* have sent request */
#define STIMON		1161	/* stimulus is now on */
#define STIMOFF		1162
#define FIXASK		1164
#define FIXMOVE		1165
#define TRLSTART	1166
#define TRLEND		1167

#define ERRORCD		1175	/* hardware, software errors */
#define OVRDCD		1176	/* some keyboard override is on */
#define CRCTNCD		1177	/* critter's in a correction trial */

#define BREAKFIXCD	1180	/* the next are monkey-response codes */
#define CORRECTCD	1181
#define WRONGCD		1182
#define NOCHCD		1183
#define FIXCD		1184
#define ACQCD		1185
/*
 * for the headers at the tops of Efiles 
 *
 * backwards compatibility note: this was modified 6/14/95, to allow negative
 * values to work (want to keep clear of 1200 by about 200). This means the
 * Gabor data collected up to this point will need special treatment. Yuck.
 */
#define HEADCD		1499 
#define HEADBASE	1500

/*
 * codes to identify target onset and target offset
 *
 * added 1/23/98 hwh
 *
 */

#define TARGONCD 1040
#define TARGOFFCD 1041

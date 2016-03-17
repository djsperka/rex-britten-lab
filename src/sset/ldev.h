/*
 *-----------------------------------------------------------------------*
 * NOTICE:  This code was developed by the US Government.  The original
 * versions, REX 1.0-3.12, were developed for the pdp11 architecture and
 * are in the public domain.  This version, REX 4.0, is a port of the
 * original version to the Intel 80x86 architecture.  This version is
 * copyright (C) 1992 by the National Institutes of Health, Laboratory
 * of Sensorimotor Research, Bldg 10 Rm 10C101, 9000 Rockville Pike,
 * Bethesda, MD, 20892, (301) 496-9375.  All rights reserved.
 *-----------------------------------------------------------------------*
 */

/*
 * This header contains the device definitions that are specific
 * to each laboratory. 
 */

/*
 *-----------------------------------------------------------------------*
 *		    Laboratory Specific Device Definitions
 *-----------------------------------------------------------------------*
 *
 * This version is unique to the front rig room, which is outfitted with
 * the Harvey box. KHB 7/11/02.
 */



/*
 * for tstramp.d only
 */
#define LED1	    Dio_id(PCDIO_DIO, 0, 0x10)
#define BACKLT	    Dio_id(PCDIO_DIO, 0, 0x20)


#ifdef BRITTENLAB

/* 
 * These are the joystick buttons for the dotmap joystick
 * These should be AND-ed with 'dina'
 */
 
#define BUTTON1 0x1
#define BUTTON2 0x2
#define BUTTON3 0x4

#ifdef DIOTEST


#define BEEP	    Dio_id(PCDIO_DIO, 0, 0x01)
#define REW	    	Dio_id(PCDIO_DIO, 0, 0x02)
#define MUSTIM	    Dio_id(PCDIO_DIO, 0, 0x04)
#define HB_TRIG	    Dio_id(PCDIO_DIO, 0, 0x08)
#define FIX_LED	    Dio_id(PCDIO_DIO, 0, 0x10)

#else

#define BEEP	    Dio_id(PCDIO_DIO, 0, 0x01)
#define REW	    	Dio_id(PCDIO_DIO, 0, 0x02)
#define MUSTIM	    Dio_id(PCDIO_DIO, 0, 0x04)
#define HB_TRIG	    Dio_id(PCDIO_DIO, 0, 0x08)
#define FIX_LED	    Dio_id(PCDIO_DIO, 1, 0x01)

#endif   /* ifdef DIOTEST */


#endif   /* ifdef BRITTENLAB */

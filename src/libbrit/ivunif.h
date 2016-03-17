#ifndef IVUNIF_H_
#define IVUNIF_H_

/* 
 * Random number routines used in Rex paradigms.
 */ 
 
 /* IFUNIF
 *	fast interval uniform random number generator.
 * This generates integers between 0 and n, but uses a a less
 * precise technique than ivuniv(), and hence can
 * lead to under-representation of numbers near n.  However,
 * it is suitable for some uses, such as shuffling.
 * INPUT:  n = max integer of interval
 *
 * OUTPUT: integer in range [0, n], inclusive
 */

long ifuniv(long int n);


/* IVUNIF
 *	interval uniform random number generator.  From Bratley, et al.,
 * pg. 209.  Inversion algorithm
 * This uses an infinite precision technique to generate integers
 * between 0 and n without suffering from truncation errors, which can
 * lead to under-representation of numbers near n.
 *
 * INPUT:  seed > 0 ==> reseed
 *	   n	max integer of interval
 *
 * OUTPUT: integer in range [0, n], inclusive
 */

int ivunif(int seed, int n);

 

/* ISHUFFLE
 *	shuffles a list of (int) indices
 *
 * ARGS:
 *	p	= pointer to array of ints
 *	n	= number of elements in list
 */

void ishuffle(int* p, int n);


/* ISSHUFFLE
 *	shuffles a list of (short) indices
 *
 * ARGS:
 *	p	= pointer to array of ints
 *	n	= number of elements in list
 */

void isshuffle(short int* p, int n);




#endif /*IVUNIF_H_*/

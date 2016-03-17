/* 
 * $Id: randomtrialgenerator.h,v 1.3 2010/06/07 23:55:20 devel Exp $
 * 
 * C-style object for generating random sets of trials arranged in blocks.
 * 
 * This object will produce indices (you must interpret the index 
 * appropriately - e.g. by having an array of structures which hold your
 * trial variables - and you must indicate to the generator that a given
 * trial was "successful" and should be counted as such. 
 * 
 * Here's how it works. Suppose you have two experimental variables for 
 * which you want to vary across some number of values (lets say m and 
 * n values). Each combination of values consitutes a trial type, 
 * and you want at least N trials of each type.Thus, you will need N*m*n
 * total trials, and you want them randomly selected. Furthermore, you would
 * like the sets of m*n trial types to come in "blocks", where some number
 * of blocks B (B<N) are completed before moving on to additional blocks. 
 * 
 * Here's a concrete example for clarity. Lets say you have two independent
 * conditions: the position of a fixation point (5 of these) and a rate
 * which is relevant to your stimulus display (4 of these). A "trial type"
 * is one combination of fixation point position and rate. In all you have
 * 20 different combinations (5*4), or trial tyles, and you would like 
 * to collect data for 10 trials of each type.
 * 
 * You also know that there are times when you don't have time, or the 
 * cooperation of the subject, and that you can't always collect 10 trials
 * of each type in a session. In those cases you want to stop the data 
 * collection early, but you'd like to ensure some balance in the distribution
 * of trial types that have been collected at any point in the experiment. 
 * So, you choose a "block size" of 2, which means that the trial types will 
 * be randomly selected from the 20 types, but all trial types must have been
 * selected (and successfully completed) at least 2 times before any trial
 * type can be selected the third time. The same is true at each multiple of 
 * the block size (2, 4, 6, 8) up to the total trials required for each type. 
 * 
 * Thus, if you quit in the middle of data collection, before the required 
 * 10 trials of each type is collected, you can be assured that the count of
 * any trial type will be within "block size" of any other trial type. In other
 * words, you'll never have the situation where you have a large number of 
 * trials of some type, and a small number of some other trial type. The 
 * max difference between the types is guaranteed to be less than or equal
 * to the block size.  
 * 
 * Usage:
 * 
 * In your source file you must include this header file:
 * 
 * #include "randomtrialgenerator.h"
 * 
 * The trial generator must be "created" and "destroyed" by you - this takes
 * care of any memory allocation and cleanup. For performance reasons,  
 * no memory allocation is performed after the initial creation of the 
 * object. The creation function returns a pointer which is used in all calls
 * to the generator.   
 * 
 * RTGenerator* prtgen;
 * int nTrialTypes = 20;
 * int nTotal = 10;
 * int nBlockSize = 2;
 * int index;
 * 
 * prtgen = rtg_create(nTrialTypes, nTotal, nBlockSize);
 * 
 * 
 * To get a random trial selection, call 'next'. It returns an index
 * value between 0 and nTrialTypes-1 inclusive. In the example here, that's
 * between 0 and 19.When all trial types have been selected nTotal times
 * (or if there's an error) then next() will return -1.  
 * 
 * 
 * index = prtgen->next(prtgen);
 * if (index<0)
 * {
 *    ***all done!!!***
 * }
 * else
 * {
 *    ***do what you need to for trial type 'index'. 
 * }
 * 
 * When a trial is "successful", then you must tell the generator that's 
 * the case - by calling mark(). This function "marks" a tally for that
 * trial type in the current block. It returns the number of trials counted
 * in the current block for that type (or -1 on error - meaning a bad input
 * index value of invalid prtgen pointer). 
 * 
 * if (trial successful)
 * {
 *    prtgen->mark(prtgen, index);
 * }
 * else
 * {
 *    *** do not call mark(). The current trial type will be re-selected.
 * }
 * 
 * 
 * You can get a count of how many times a given index has been mark()'ed
 * by calling count():
 * 
 * numTimesCounted = prtgen->count(prtgen, index);
 * 
 * Furthermore, you can clear all counts by calling reset() - this sets all
 * tallies to 0 (but the generator remembers its total and block size). 
 * 
 * prtgen->reset(prtgen);
 * 
 * 
 */

#ifndef RANDOMTRIALGENERATOR_H_
#define RANDOMTRIALGENERATOR_H_





/* Forward declaration of opaque struct */
struct rtg_priv;
typedef struct rtg_struct RTGenerator;

struct rtg_struct
{
	struct rtg_priv *priv;
	int (*next)(RTGenerator *prtgen);
	int (*mark)(RTGenerator *prtgen, int index);
	int (*marklist)(RTGenerator *prtgen, int nlist, int *plist);
	void (*reset)(RTGenerator *prtgen);
	int (*count)(RTGenerator *prtgen, int index);
	int (*list)(RTGenerator *prtgen, int **plist);
};

RTGenerator *rtg_create(int ntypes, int ntotal, int blocksize);
RTGenerator *rtlistg_create(int ntypes, int ntotal, int blocksize, int listsize);
void rtg_destroy(RTGenerator *prtgen);
void rtg_status(RTGenerator *prtgen);

#endif /*RANDOMTRIALGENERATOR_H_*/

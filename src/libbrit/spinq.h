#ifndef SPINQ_H_
#define SPINQ_H_



/*
 * Forward declaration for opaque struct. See spinq.c for definition.
 */
struct spinq_struct;	
typedef struct spinq_struct *PSpinQ;


/*
 * spinq_create
 * 
 * Allocate a spinq struct and initialize a histogram with jsize elements.
 * Must call spinq_init prior to use. 
 */

PSpinQ spinq_create(int jsize);

/* 
 * spinq_destroy
 * 
 * free memory associated with spinq struct. The pointer is not defined
 * after this call.
 */
 
void spinq_destroy(PSpinQ p);

/* 
 * spinq_init(PSpinQ p, int qsize, int window)
 * 
 * Initialize spinq for use. Allocates a queue of size qsize (just clears
 * existing queue if previously initialized with same size). Assigns
 * spin window of 'window'. 
 * 
 * Returns 0 on success, negative on error (-1 of qsize<=0, -2 if can't alloc queue). 
 */
 
int spinq_init(PSpinQ p, int qsize, int window);

/* 
 * spinq_clear()
 * 
 * Empties queue but does not free any memory. Ready to use again. 
 * 
 */
 
int spinq_clear(PSpinQ p);

/* 
 * spinq_push(PSpinQ p, int joy)
 * 
 * Push a value on to the queue. If the queue is filled, a test is performed
 * to see if all values in queue are within 'window' of the value at the 
 * top of the queue. 
 * 
 * Returns 1 if spinning (all values within window of value at top of queue), 
 * or 0 if not spinning, or if queue not yet full. 
 */

int spinq_push(PSpinQ p, int joy);

/*
 * spinq_debug(PSpinQ p)
 * 
 * Sets a flag so debugging information is dprinted on function calls. 
 */
  
void spinq_debug(int d);

/*
 * spinq_dump(PSpinQ p)
 * 
 * Dumps info on the queue. 
 */
 
void spinq_dump(PSpinQ p);

#endif /*SPINQ_H_*/

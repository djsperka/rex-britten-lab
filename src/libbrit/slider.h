#ifndef SLIDING_H_
#define SLIDING_H_


/*
 * Implementation of a sliding window. Adding values works like a FIFO circular array - values
 * are added to the end of the array until it is full. After that the first element of the 
 * array is replaced, then the second, etc. 
 * 
 * The window keeps track of the current max and min values in the window. 
 * 
 * Individual values can be accessed with the at() function. The index works like an array: 
 * the oldest value in the sliding window is at index 0. This is not necessarily the first element
 * of the underlying array! 
 * 
 */




/* Forward declaration of opaque struct */
struct sliding_window_priv;
typedef struct sliding_window_struct Slider;
typedef struct sliding_window_struct* PSlider;

struct sliding_window_struct
{
	struct sliding_window_priv *priv;
	
	/*
	 * size(Slider*)
	 * 
	 * Size of the slider - i.e. how many values can the sliding window hold? 
	 */

 	long (*size)(Slider *);
	
	/*
	 * wmax(Slider*)
	 * 
	 * Max value currently held in sliding window
	 */
	 
	long (*wmax)(Slider *);
	
	/*
	 * wmax(Slider*)
	 * 
	 * Max value currently held in sliding window
	 */
	
	long (*wmin)(Slider *);
	
	/*
	 * isFull(Slider*)
	 * 
	 * Return 1 if the sliding window has all slots filled, otherwise return 0;
	 */
	
	long (*isFull)(Slider *);
	
	/*
	 * wadd(Slider *, long value)
	 * 
	 * Add the value 'value' to the front of the sliding window.
	 */

	long (*wadd)(Slider *, long);
	
	/*
	 * clear(Slider *)
	 * 
	 * Remove all values from sliding window.
	 */
	 
	void (*clear)(Slider *);
	
	/*
	 * at(Slider *, int at, long *value)
	 * 
	 * Returns the value of the element in the 'at' position of the window. Treats the 
	 * sliding window like an array - the oldest value is [0]. Asking for values at index
	 * greater than or equal to the window size is an error and the return value is LONG_MIN
	 */

	long (*at)(Slider *, int);
	
	/*
	 * sum(Slider*)
	 * 
	 * Returns the sum of elements in the slider. Not bothering with the number of values - 
	 * just use it when full for now. 
	 * 
	 */
	 
	 long (*wsum)(Slider *);
};

Slider *slider_create(long size);
void slider_destroy(Slider *);
void slider_dump(Slider *pslider);

#endif /*SLIDING_H_*/

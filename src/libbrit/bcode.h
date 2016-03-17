#ifndef BCODE_H_
#define BCODE_H_


/* 
 * BCODE bit flags
 */

#define	BCODE_FLAG		0x1000
#define BCODE_FLOAT		BCODE_FLAG | 0x800
#define BCODE_INT		BCODE_FLAG | 0x400
#define BCODE_UINT		BCODE_FLAG | 0x200
#define BCODE_MARK		BCODE_FLAG | 0x100

void bcode_float(unsigned char channel, float value);
void bcode_int(unsigned char channel, int value);
void bcode_uint(unsigned char channel, unsigned int value);

int ecode(int code);
int ecode_multiplier(int basecd, float value, int factor);

#endif /*BCODE_H_*/

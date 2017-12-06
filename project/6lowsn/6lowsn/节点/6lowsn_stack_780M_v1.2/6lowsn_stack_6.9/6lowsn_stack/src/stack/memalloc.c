

#include "compiler.h"               //compiler specific
#include <string.h>                 // has memset function
#include "6lowsn_config.h"
#include "hal.h"        //for global interrupt enable/disable
#include "halStack.h"
#include "console.h"
#include "debug.h"
#include "memalloc.h"




//LOWSN_HEAPSIZE must be < 64K, can allocate blocks up to this size
// format of block is
// free bit/15 bit size block_UINT8s | free bit/15 bit size block_UINT8s | etc..
// heap merging done on FREE, free adjacent blocks are merged.



//static UINT8 mem_heap[LOWSN_HEAPSIZE];
#include "halHeapSpace.h"

typedef UINT16 MEMHDR;

#define MEMHDR_FREE_MASK 0x80
#define MEMHDR_SIZE_MASK 0x7FFF

#define MEMHDR_GET_FREE(x) (*(x+1)&MEMHDR_FREE_MASK)
#define MEMHDR_CLR_FREE(x)  *(x+1) = *(x+1)&(~MEMHDR_FREE_MASK)
#define MEMHDR_SET_FREE(x)  *(x+1) = *(x+1)|MEMHDR_FREE_MASK


UINT16 memhdr_get_size (UINT8 *ptr) {
	UINT16 x;

	x = (UINT8) *ptr;
	x += ((UINT16) *(ptr+1)<< 8);
	x = x & 0x7FFF;
	return(x);
}

void memhdr_set_size (UINT8 *ptr, UINT16 size) {

	*ptr = (UINT8) size;
	ptr++;
	*ptr = *ptr & 0x80;  //clear size field
	*(ptr) += (size >> 8);  //add in size.
}


#if 0
typedef struct _MEMHDR {
    UINT16 val;

    unsigned int free:1;
	unsigned int size:15;	
}MEMHDR;
#endif


#define MINSIZE 16+sizeof(MEMHDR)


void MemInit (void) {
	memset(mem_heap,0,LOWSN_HEAPSIZE);
	MEMHDR_SET_FREE(((UINT8 *)&mem_heap[0]));
    memhdr_set_size(((UINT8 *)&mem_heap[0]),(LOWSN_HEAPSIZE - sizeof(MEMHDR)));
}

UINT8 * MemAlloc (UINT16 size) {

	UINT8 *free_blk, *next_blk;
	UINT16 offset;
	UINT16 remainder;
        BOOL  gie_status;



    if (!size) return(NULL);      //illegal size
	if (size < MINSIZE) size = MINSIZE;

        SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);
	free_blk = mem_heap;
	offset = 0;
	while (1) {
		if (MEMHDR_GET_FREE(((UINT8 *)free_blk)) &&
			(memhdr_get_size((UINT8 *)free_blk) >= size)) break; //found block
		//advance to next block
		offset = offset + memhdr_get_size((UINT8 *)free_blk) + sizeof(MEMHDR);
                if (offset >= LOWSN_HEAPSIZE) {
                  DEBUG_CHAR( DBG_ERR,DBG_CHAR_MEMFULL  );
                  RESTORE_GLOBAL_INTERRUPT(gie_status);
                  return(NULL); // no free blocks
                }
		free_blk = mem_heap + offset;
	}
	remainder =  memhdr_get_size((UINT8 *)free_blk) - size;
	if (remainder < MINSIZE) {
		//found block, mark as not-free
		MEMHDR_CLR_FREE((UINT8 *)free_blk);
        RESTORE_GLOBAL_INTERRUPT(gie_status);
		return(free_blk + sizeof(MEMHDR));
	}
	//remainder is large enough to support a new block
	//adjust allocated block to requested size
	memhdr_set_size(((UINT8 *)free_blk),size);
	//format next blk
	next_blk = free_blk+size+sizeof(MEMHDR);
	MEMHDR_SET_FREE((UINT8 *) next_blk);
	memhdr_set_size(((UINT8 *) next_blk), (remainder - sizeof(MEMHDR)));

	MEMHDR_CLR_FREE((UINT8 *)free_blk); //mark allocated block as non-free
    RESTORE_GLOBAL_INTERRUPT(gie_status);
	return(free_blk + sizeof(MEMHDR));      //return new block
}

void MemFree(UINT8 *ptr) {
	UINT8 *hdr;
	UINT16 offset, tmp;
        BOOL  gie_status;


    if (ptr == NULL) return;
    SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);
	hdr = ptr - sizeof(MEMHDR);
	//free this block
	MEMHDR_SET_FREE((UINT8 *)hdr);
	//now merge
	offset = 0;
	hdr = mem_heap;
	//loop until blocks that can be merged are merged
	while (1) {
		if (MEMHDR_GET_FREE((UINT8 *)hdr)) {
			//found a free block, see if we can merge with next block
			tmp = offset +  memhdr_get_size((UINT8 *)hdr) + sizeof(MEMHDR);
			if (tmp >= LOWSN_HEAPSIZE) break; //at end of heap, exit loop
			ptr = mem_heap + tmp; //point at next block
			if (MEMHDR_GET_FREE((UINT8 *)ptr)) {
				//next block is free, do merge by adding size of next block
	            memhdr_set_size(((UINT8 *)hdr),(memhdr_get_size((UINT8 *)hdr)+ memhdr_get_size((UINT8 *)ptr)
					                            + sizeof(MEMHDR)));
				// after merge, do not change offset, try to merge again
				//next time through loop
				continue; //back to top of loop
			}			
		}
		// next block
		offset = offset + memhdr_get_size((UINT8 *)hdr) + sizeof(MEMHDR);
		if (offset >= LOWSN_HEAPSIZE) break;  //at end of heap, exit loop
		hdr = mem_heap + offset;
	}
	 RESTORE_GLOBAL_INTERRUPT(gie_status);
}

#ifdef LOWSN_COMPILER_NO_RECURSION

//this supports the HI-TECH compiler, which does not support recursion

UINT16 ISR_memhdr_get_size (UINT8 *ptr) {
	UINT16 x;

	x = (UINT8) *ptr;
	x += ((UINT16) *(ptr+1)<< 8);
	x = x & 0x7FFF;
	return(x);
}

void ISR_memhdr_set_size (UINT8 *ptr, UINT16 size) {

	*ptr = (UINT8) size;
	ptr++;
	*ptr = *ptr & 0x80;  //clear size field
	*(ptr) += (size >> 8);  //add in size.
}



UINT8 * ISRMemAlloc (UINT16 size) {

	UINT8 *free_blk, *next_blk;
	UINT16 offset;
	UINT16 remainder;
        BOOL  gie_status;



        if (!size) return(NULL);      //illegal size
	if (size < MINSIZE) size = MINSIZE;

        SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);
	free_blk = mem_heap;
	offset = 0;
	while (1) {
		if (MEMHDR_GET_FREE(((UINT8 *)free_blk)) &&
			(ISR_memhdr_get_size((UINT8 *)free_blk) >= size)) break; //found block
		//advance to next block
		offset = offset + ISR_memhdr_get_size((UINT8 *)free_blk) + sizeof(MEMHDR);
                if (offset >= LOWSN_HEAPSIZE) {
                  DEBUG_CHAR( DBG_ERR,DBG_CHAR_MEMFULL  );
                  RESTORE_GLOBAL_INTERRUPT(gie_status);
                  return(NULL); // no free blocks
                }
		free_blk = mem_heap + offset;
	}
	remainder =  ISR_memhdr_get_size((UINT8 *)free_blk) - size;
	if (remainder < MINSIZE) {
		//found block, mark as not-free
		MEMHDR_CLR_FREE((UINT8 *)free_blk);
        RESTORE_GLOBAL_INTERRUPT(gie_status);
		return(free_blk + sizeof(MEMHDR));
	}
	//remainder is large enough to support a new block
	//adjust allocated block to requested size
	ISR_memhdr_set_size(((UINT8 *)free_blk),size);
	//format next blk
	next_blk = free_blk+size+sizeof(MEMHDR);
	MEMHDR_SET_FREE((UINT8 *) next_blk);
	ISR_memhdr_set_size(((UINT8 *) next_blk), (remainder - sizeof(MEMHDR)));

	MEMHDR_CLR_FREE((UINT8 *)free_blk); //mark allocated block as non-free
    RESTORE_GLOBAL_INTERRUPT(gie_status);
	return(free_blk + sizeof(MEMHDR));      //return new block
}

void ISRMemFree(UINT8 *ptr) {
	UINT8 *hdr;
	UINT16 offset, tmp;
        BOOL  gie_status;


        SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);
	hdr = ptr - sizeof(MEMHDR);
	//free this block
	MEMHDR_SET_FREE((UINT8 *)hdr);
	//now merge
	offset = 0;
	hdr = mem_heap;
	//loop until blocks that can be merged are merged
	while (1) {
		if (MEMHDR_GET_FREE((UINT8 *)hdr)) {
			//found a free block, see if we can merge with next block
			tmp = offset +  ISR_memhdr_get_size((UINT8 *)hdr) + sizeof(MEMHDR);
			if (tmp >= LOWSN_HEAPSIZE) break; //at end of heap, exit loop
			ptr = mem_heap + tmp; //point at next block
			if (MEMHDR_GET_FREE((UINT8 *)ptr)) {
				//next block is free, do merge by adding size of next block
	            ISR_memhdr_set_size(((UINT8 *)hdr),(ISR_memhdr_get_size((UINT8 *)hdr)+ ISR_memhdr_get_size((UINT8 *)ptr)
					                            + sizeof(MEMHDR)));
				// after merge, do not change offset, try to merge again
				//next time through loop
				continue; //back to top of loop
			}			
		}
		// next block
		offset = offset + ISR_memhdr_get_size((UINT8 *)hdr) + sizeof(MEMHDR);
		if (offset >= LOWSN_HEAPSIZE) break;  //at end of heap, exit loop
		hdr = mem_heap + offset;
	}
	 RESTORE_GLOBAL_INTERRUPT(gie_status);
}




#endif





#ifdef LOWSN_DEBUG
//Dump Memory to see what it looks like
void MemDump(void) {
	UINT8 *hdr;
	UINT16 offset,j;

	offset = 0;
	j = 1;
	while (offset < LOWSN_HEAPSIZE) {
		hdr = mem_heap+offset;
                conPrintROMString("Blk: ");
                conPrintUINT16(j);
                conPrintROMString("; offset: ");
                conPrintUINT16(offset);
                conPrintROMString(", Free:");
				if (MEMHDR_GET_FREE((UINT8 *)hdr)) {
                 conPrintROMString("Y ");
				}
				else {conPrintROMString("N ");
				}
                conPrintROMString(", Size: ");
                conPrintUINT16(memhdr_get_size((UINT8 *)hdr));
                conPrintROMString("\n");
		offset = offset + memhdr_get_size((UINT8 *)hdr)+ sizeof(MEMHDR);
		j++;

	}
}

#endif






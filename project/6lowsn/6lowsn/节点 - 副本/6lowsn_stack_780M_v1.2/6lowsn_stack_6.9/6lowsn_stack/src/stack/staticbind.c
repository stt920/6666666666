

#include "compiler.h"               //compiler specific
#include "hal.h"
#include "halStack.h"
#include "lowsn_config.h"
#include "evboard.h"
#include "neighbor.h"


#ifdef LRWPAN_COORDINATOR
#ifdef LRWPAN_USE_DEMO_STATIC_BIND
#include "staticbind.h"

//the prototypes for these functions are defined in evboard.h
//since binding other than these default static bind functions
//is assumed to be platform dependent.

static BYTE bind_table_index;
static BYTE src_map_index;
static BYTE bindSrcEP, bindCluster;


//this initializes the bindTable interator
BOOL evbBindTableIterInit(BYTE srcEP, SADDR srcSADDR, BYTE cluster){
	bind_table_index = 0;
	bindSrcEP = srcEP;
	bindCluster = cluster;
	//look up this srcSADDR in the address map table
	//if SADDR is not in the address table, this function returns FALSE, indicating failure
    return(ntFindAddressBySADDR(srcSADDR, &src_map_index));	
}

//find the next binding that matches the current bindSrcEP, bindSrcSADDR, bindCluster
BOOL evbResolveBind(BYTE *dstEP, SADDR *dstSADDR)
{
	BYTE i, dst_map_index;
	BINDENTRY *bptr;

    //check if at end, if yes, exit.
	if (bind_table_index == BINDTABLE_ENTRIES) return(FALSE);

	

	do {
		bptr = &(bindtable[bind_table_index]);
		if ( (bptr->srcEP == bindSrcEP) &&
			(bptr->cluster == bindCluster)) {
				//match on EP and cluster
				//now check the SRC long address
				for (i=0;i<8;i++) {
					if (bptr->src.bytes[i] != mac_addr_tbl[src_map_index].laddr[i]) break;
				}
				if (i == 8) {
					if (ntFindAddressByLADDR(&bptr->dst, &dst_map_index)) {
						//successfully found the dstSADDR
					  *dstEP = bptr->dstEP;
					  *dstSADDR = mac_addr_tbl[dst_map_index].saddr;
					  //before leaving, increment BPTR to point to next map entry
					  bind_table_index++;
					  return(TRUE);  //exit with match
					}

				}
			}
     //no match, continue looking
	 bind_table_index++;
	}while(bind_table_index < BINDTABLE_ENTRIES);

//if reach here, no match in entire table, so halt search
return(FALSE);
}

#endif
#endif


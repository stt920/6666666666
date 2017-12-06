

#include "compiler.h"
#include "6lowsn_config.h"         //user configurations
#include "6lowsn_common_types.h"   //types common acrosss most files
#include "ieee_lrwpan_defs.h"
#include "console.h"
#include "debug.h"
#include "memalloc.h"
#include "hal.h"
#include "halStack.h"
#include "phy.h"
#include "mac.h"


#include "neighbor.h"

//holds pairs of long/short addresses


#ifdef LOWSN_FFD
NAYBORENTRY mac_nbr_tbl[NTENTRIES];
#endif


//initializes the Address map
//called in halInit()
//also called anytime the Coord forms a network
//or a router joins a network.
//the first entry in the address map is always our own
//that maps long address to short address
void ntInitAddressMap(void){
  BYTE j;
  for (j=0;j<LOWSN_MAX_ADDRESS_MAP_ENTRIES;j++) {
		mac_addr_tbl[j].saddr = LOWSN_BCAST_SADDR;
	}
}
	

//address map functions
//finds a match in the address map table using a LADDR
BOOL ntFindAddressByLADDR(LADDR *ptr, BYTE *index){
   BYTE j,i;
   BYTE *src,*dst;

   for (j=0;j<LOWSN_MAX_ADDRESS_MAP_ENTRIES;j++) {
		if (mac_addr_tbl[j].saddr == LOWSN_BCAST_SADDR) continue;
		src = &ptr->bytes[0];
		dst = &mac_addr_tbl[j].laddr[0];
		for (i=0;i<8;i++, src++, dst++) {
			if (*src != *dst) break;
		}
		if (i== 8) {
			//have a match
		    *index = j;
		    break;
		}		
   }

   if (j != LOWSN_MAX_ADDRESS_MAP_ENTRIES) return(TRUE);
	else return(FALSE);	

}

void ntAddOurselvesToAddressTable(SADDR saddr){
	BYTE laddr[8];

	halGetProcessorIEEEAddress(&laddr[0]);
	ntNewAddressMapEntry(&laddr[0], saddr);
}


//finds a match in the address map table using a SADDR
BOOL ntFindAddressBySADDR(SADDR saddr, BYTE *index)
{
   BYTE j;

   for (j=0;j<LOWSN_MAX_ADDRESS_MAP_ENTRIES;j++) {
		if (mac_addr_tbl[j].saddr == LOWSN_BCAST_SADDR) continue;
		if (mac_addr_tbl[j].saddr != saddr) continue;
		*index = j;
		break;
		
	}

   if (j != LOWSN_MAX_ADDRESS_MAP_ENTRIES) return(TRUE);
	else return(FALSE);

}

//checks if  laddr, saddr is already in the address map table
//if it is, return map index in 'index'
BOOL ntCheckAddressMapEntry(BYTE *laddr, SADDR saddr, BYTE *index) {
   BYTE j,i;
   BYTE *src,*dst;

   for (j=0;j<LOWSN_MAX_ADDRESS_MAP_ENTRIES;j++) {
		if (mac_addr_tbl[j].saddr == LOWSN_BCAST_SADDR) continue;
		if (mac_addr_tbl[j].saddr != saddr) continue;
		src = laddr;
		dst = &mac_addr_tbl[j].laddr[0];
		for (i=0;i<8;i++) {
			if (*src != *dst) break;
			src++; dst++;
		}
		if (i == 8) {
			// we have a match
			*index = j;
			return(TRUE);
		}
	}

	return(FALSE);
}

//enters new laddr, saddr into address map entry
SADDR ntNewAddressMapEntry(BYTE *laddr, SADDR saddr) {
    BYTE j;

	if (ntCheckAddressMapEntry(laddr, saddr, &j)) {
		//entry is already in the table.
		return(mac_addr_tbl[j].saddr);
	}
	//now find free entry in address map table
	for (j=0;j<LOWSN_MAX_ADDRESS_MAP_ENTRIES;j++) {
		if (mac_addr_tbl[j].saddr == LOWSN_BCAST_SADDR) break;
	}
	if (j== LOWSN_MAX_ADDRESS_MAP_ENTRIES) return(LOWSN_BCAST_SADDR);//error, no room
    halUtilMemCopy(&mac_addr_tbl[j].laddr[0], laddr, 8);
	mac_addr_tbl[j].saddr = saddr;
	return(0);
}





UINT16 ntGetCskip(BYTE depth) {
	switch(depth){
		case 1: return(LOWSN_CSKIP_1);
		case 2: return(LOWSN_CSKIP_2);
		case 3: return(LOWSN_CSKIP_3);
		case 4: return(LOWSN_CSKIP_4);
		case 5: return(LOWSN_CSKIP_5);
		case 6: return(LOWSN_CSKIP_6);
		case 7: return(LOWSN_CSKIP_7);
		case 8: return(LOWSN_CSKIP_8);
		case 9: return(LOWSN_CSKIP_9);
		case 10: return(LOWSN_CSKIP_10);
	}
return(0);
}

SADDR ntGetMaxSADDR(SADDR router_saddr,BYTE depth){
	//compute the maximum SADDR given the router_saddr and depth

   return(router_saddr + (ntGetCskip(depth)*(LOWSN_MAX_ROUTERS_PER_PARENT))
	      + LOWSN_MAX_NON_ROUTER_CHILDREN);
}


//rest of this are neighbor table functions, only needed by FFDs

#ifdef LOWSN_FFD


NAYBORENTRY *ntFindBySADDR (UINT16 saddr){

	NAYBORENTRY *nt_ptr;
	BYTE j;

	nt_ptr = &mac_nbr_tbl[0];
	for (j=0;j<NTENTRIES;j++,nt_ptr++) {
		if ( nt_ptr->flags.bits.used &&
			mac_addr_tbl[nt_ptr->map_index].saddr == saddr) return(nt_ptr);
		//nt_ptr++;
    }
	return(NULL);
}

NAYBORENTRY *ntFindByLADDR (LADDR *ptr){
	NAYBORENTRY *nt_ptr;
	BYTE j,i;

    nt_ptr = &mac_nbr_tbl[0];
	for (j=0;j<NTENTRIES;j++,nt_ptr++) {
		if (!nt_ptr->flags.bits.used) continue;
		for (i=0;i<8;i++) {
			if (mac_addr_tbl[nt_ptr->map_index].laddr[i] != ptr->bytes[i]) break;
		}
		if (i == 8)	return(nt_ptr);
	}
	return(NULL);
}



//Init neighbor table. Called when Network is formed by
//coordinator or when a Router successfully joins a network.
//this also initializes the address table map
void ntInitTable(void) {
	NAYBORENTRY *nt_ptr;
	BYTE j;
	
	nt_ptr = &mac_nbr_tbl[0];
	for (j=0;j<NTENTRIES;j++,nt_ptr++) {
		nt_ptr->flags.val = 0;
	}
	ntInitAddressMap();	
}

void ntInitAddressAssignment(void){
    //also initialize ADDRESS assignment
	mac_pib.ChildRFDs = 0;
	mac_pib.ChildRouters = 0;
	mac_pib.nextChildRFD = macGetShortAddr() + 1+ ntGetCskip(mac_pib.depth+1)*(LOWSN_MAX_ROUTERS_PER_PARENT);
	mac_pib.nextChildRouter = macGetShortAddr() + 1;
}



//adds a neighbor, and assigns a new SADDR
SADDR ntAddNeighbor(BYTE *ptr, BYTE capinfo) {
	NAYBORENTRY *nt_ptr;
	BYTE j;
	BYTE *tmpptr;

	//First, find free entry in neighbor table
    nt_ptr = &mac_nbr_tbl[0];
	for (j=0;j<NTENTRIES;j++,nt_ptr++) {
		if (!nt_ptr->flags.bits.used) break;
	}
	if (j== NTENTRIES) return(LOWSN_BCAST_SADDR);//error, no room

	//now find free entry in address map table
	for (j=0;j<LOWSN_MAX_ADDRESS_MAP_ENTRIES;j++) {
		if (mac_addr_tbl[j].saddr == LOWSN_BCAST_SADDR) break;
	}
	if (j== LOWSN_MAX_ADDRESS_MAP_ENTRIES) return(LOWSN_BCAST_SADDR);//error, no room
    nt_ptr->map_index = j;
	nt_ptr->flags.bits.used = 1;
	nt_ptr->flags.bits.lqi = 0;
	nt_ptr->capinfo = capinfo;

	//now new saddr
	if (LOWSN_GET_CAPINFO_DEVTYPE(capinfo)) {
		mac_addr_tbl[nt_ptr->map_index].saddr = mac_pib.nextChildRouter;
		mac_pib.nextChildRouter += ntGetCskip(mac_pib.depth+1);
		mac_pib.ChildRouters++;
	}else {
		mac_addr_tbl[nt_ptr->map_index].saddr = mac_pib.nextChildRFD;
        mac_pib.nextChildRFD++;
		mac_pib.ChildRFDs++;
	}
	//now copy long addr
	tmpptr = &mac_addr_tbl[nt_ptr->map_index].laddr[0];
	for(j=0;j<8;j++) {
		*tmpptr = *ptr;
		tmpptr++; ptr++;
	}
	return(mac_addr_tbl[nt_ptr->map_index].saddr);
}




//determine where this packet is going based on its short address
SADDR ntFindNewDst(SADDR dstSADDR){
	SADDR tmpSADDR;
	NAYBORENTRY *nt_ptr;
	BYTE j;

  if (dstSADDR == macGetShortAddr()) {
	  //trying to send to myself, this is an error
	  return(0xFFFF);
	}
  //if destination is coordinator, has to go to our parent
  if (dstSADDR == 0) return(mac_pib.macCoordShortAddress);
  // See if this destination is within my routing range
  // if not, then have to send it to my parent
#ifndef LOWSN_COORDINATOR
  //do not check this for coordinator, as all nodes in range of coordinator.
  tmpSADDR = ntGetMaxSADDR(macGetShortAddr(),mac_pib.depth+1);
  if (!((dstSADDR > macGetShortAddr()) &&
	  (dstSADDR <= tmpSADDR))) {
		  //not in my range, must go to parent.
		  return(mac_pib.macCoordShortAddress);
	  }
#endif

  //goes to one of my children, check out each one.	  	
  nt_ptr = &mac_nbr_tbl[0];

  for (j=0;j<NTENTRIES;j++,nt_ptr++) {
		if (!nt_ptr->flags.bits.used) continue;
		if (LOWSN_GET_CAPINFO_DEVTYPE(nt_ptr->capinfo)) {
			//router. check its range, the range is mac_pib.depth+2 because we need
			//the depth of the my child's child (grandchild).
			tmpSADDR = ntGetMaxSADDR(mac_addr_tbl[nt_ptr->map_index].saddr,mac_pib.depth+2);
			if ((dstSADDR >= mac_addr_tbl[nt_ptr->map_index].saddr) && (dstSADDR <= tmpSADDR)) {
				//either for my child router or one of its children.
				return(mac_addr_tbl[nt_ptr->map_index].saddr);
			}
		}else {
			//if for a non-router child, return
			if (dstSADDR == mac_addr_tbl[nt_ptr->map_index].saddr) return(mac_addr_tbl[nt_ptr->map_index].saddr);
		}
  }
  //if get here, then packet is undeliverable
  return(0xFFFF);
}


#endif

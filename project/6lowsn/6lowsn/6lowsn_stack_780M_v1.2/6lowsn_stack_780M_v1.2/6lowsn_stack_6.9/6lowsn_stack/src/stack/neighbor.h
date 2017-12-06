
#ifndef NEIGHBOR_H
#define NEIGHBOR_H



typedef struct _MAPENTRY {
	UINT16 saddr;   //an address of 0xFFFF means this is unused
	//an address of 0xFFFE means this is unknown or not assigned
	BYTE laddr[8];	
}MAPENTRY;
//holds pairs of long/short addresses, first entry is always our own.
extern MAPENTRY mac_addr_tbl[LOWSN_MAX_ADDRESS_MAP_ENTRIES ];
SADDR ntNewAddressMapEntry(BYTE *laddr, SADDR saddr);
BOOL ntFindAddressBySADDR(SADDR saddr, BYTE *index);
BOOL ntFindAddressByLADDR(LADDR *ptr, BYTE *index);
void ntAddOurselvesToAddressTable(SADDR saddr);
void ntInitAddressMap(void);
SADDR ntGetMaxSADDR(SADDR router_saddr,BYTE depth);
UINT16 ntGetCskip(BYTE depth);

//rest of this only needed for FFDs
#ifdef LOWSN_FFD
//only needed for FFDs

#define NTENTRIES (LOWSN_MAX_CHILDREN_PER_PARENT)

//Howdy NAYBOR!!!!  I dislike spelling.
typedef struct _NAYBORENTRY {

	BYTE    map_index;   //index of corresponding laddr/saddr pair in address table
	BYTE    capinfo;     //node capability info
	union {
		BYTE val;
		struct {
			unsigned used: 1;  //true if used
			unsigned lqi: 7;   //link quality indictor
		}bits;
	}flags;
}NAYBORENTRY;


//holds the neighbor table entries.
extern NAYBORENTRY mac_nbr_tbl[NTENTRIES];


NAYBORENTRY *ntFindBySADDR (UINT16 saddr);
NAYBORENTRY *ntFindByLADDR (LADDR *ptr);
void ntInitTable(void);
SADDR ntAddNeighbor(BYTE *ptr, BYTE capinfo);
void ntInitAddressAssignment(void);
SADDR ntFindNewDst(SADDR dstSADDR);


#endif

#endif



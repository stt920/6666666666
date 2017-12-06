

#ifndef MP_H
#define MP_H

// 管理对象(  管理信息库)

// 能量管理对象
#define MP_OBJECT_ID_POWER    6
#define MP_ATTRIBUTE_ID_CURRENT_VOLTAGE    1



// 系统管理专用命令ID

#define MP_COMMAND_DEVICE_INFORMATION  100
#define MP_COMMAND_TREE_NEIGHBOR_INFORMATION  101

typedef enum _MP_SENSOR_TYPE {
  MP_SENSOR_NONE=0, 
  MP_SENSOR_TEMPERATURE=1,
  MP_SENSOR_HUMIDITY,
  MP_SENSOR_PRESSURE,
  MP_SENSOR_DUST,
  MP_SENSOR_CO,
  MP_SENSOR_SO2,
  MP_SENSOR_FROG,
  MP_SENSOR_FLOW,
  MP_SENSOR_METHANE,
  MP_SENSOR_ACCELERATION
} MP_SENSOR_TYPE;

// some information about sensor itself
#ifndef LOWSN_SENSOR_TYPE
#define LOWSN_SENSOR_TYPE  0
#endif

typedef enum _MP_EVENT_TYPE  {
  MP_EVENT_NONE, 
  MP_EVENT_RECEIVE_REQUEST, 
  MP_EVENT_SEND_DEVICE_INFO, 
  MP_EVENT_SEND_TREE_TOPO_INFO
} MP_EVENT_TYPE;


typedef struct _MP_PIB {
  union _MP_DATA_flags {
    BYTE val;
    struct {
     	unsigned eventPending:1;   

    }bits;
  }flags;
  MP_EVENT_TYPE eventType;
  MP_SENSOR_TYPE sensorType; 
  IPADDR dstAddr;

}MP_PIB;

typedef struct _MP_RESPONSE_PARAM {
	IPADDR dstIPADDR;
	BYTE dstObjID;
	BYTE srcObjID; 
	BYTE serviceID; 
	BYTE actDirection; 
	BYTE actType;
	BYTE *pload;
	BYTE plen;
}MP_RESPONSE_PARAM;

extern MP_RESPONSE_PARAM mp_rsp_param;

extern MP_PIB mp_pib;

#define mpSetEventFlag()          (mp_pib.flags.bits.eventPending = 1)
#define mpClearEventFlag()       (mp_pib.flags.bits.eventPending = 0)
#define mpIsEventPending()    (mp_pib.flags.bits.eventPending)

#define mpSetEventType(x)          (mp_pib.eventType = x)


void mpInit(void);
LOWSN_STATUS_ENUM mpHandleRxReport(void);
LOWSN_STATUS_ENUM mpHandleTreeNeighborInfo(void); 
LOWSN_STATUS_ENUM mpHandleDeviceInfo(void);
void mpFmtDeviceInfo(IPADDR dst_ipaddr);
void mpFmtTreeNeighborInfo(IPADDR dst_ipaddr);
void mpFmtReadResponse(void);
void mpFmtWriteResponse(void);
void mpExcuteMethod(void);
BOOL mpReadCheck(void);
BOOL mpWriteCheck(void);
BOOL mpExecuteCheck(void);


#if 0

#define ZEP_END_DEVICE_ANNOUNCE    0x13
#define ZEP_EXTENDED_CMD          0xFF
#define ZEP_EXT_PING              0x01
#define ZEP_EXT_NODE_INFO_RSP     0x02
#define ZEP_EXT_SEND_ALARM        0x03
#define ZEP_PLEN_NODE_INFO_RSP     16

LOWSN_STATUS_ENUM zepHandleRxPacket(void);
LOWSN_STATUS_ENUM usrZepRxCallback(void);
void zepFSM(void);
void zepFmtEndDeviceAnnounce(SADDR dst);
void zepFmtNodeInfoRsp(SADDR dst);
void zepFmtPing(SADDR dst);
void zepFmtAlarm(SADDR dst_saddr, BYTE mode);
#endif


#endif



/**
\brief General OpenWSN definitions

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2010
\author Ankur Mehta <mehtank@eecs.berkeley.edu>, September 2010
*/

#ifndef __OPENDEFS_H
#define __OPENDEFS_H

// general
#include <stdint.h>               // needed for uin8_t, uint16_t
#include "toolchain_defs.h"
#include "board_info.h"

//=========================== define ==========================================

static const uint8_t infoStackName[] = "OpenWSN ";
#define OPENWSN_VERSION_MAJOR     1
#define OPENWSN_VERSION_MINOR     9
#define OPENWSN_VERSION_PATCH     0

//to delimit the implementation of draft-thubert-6man-flow-label-for-rpl-03
#define FLOW_LABEL_RPL_DOMAIN 1

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define LENGTH_ADDR16b  2
#define LENGTH_ADDR64b  8
#define LENGTH_ADDR128b 16


enum {
   E_SUCCESS                           = 0,
   E_FAIL                              = 1,
};

// types of addresses
enum {
   ADDR_NONE                           = 0,
   ADDR_16B                            = 1,
   ADDR_64B                            = 2,
   ADDR_128B                           = 3,
   ADDR_PANID                          = 4,
   ADDR_PREFIX                         = 5,
   ADDR_ANYCAST                        = 6,
};

enum {
   OW_LITTLE_ENDIAN                    = TRUE,
   OW_BIG_ENDIAN                       = FALSE,
};

// protocol numbers, as defined by the IANA
enum {
   IANA_IPv6HOPOPT                     = 0x00,
   IANA_TCP                            = 0x06,
   IANA_UDP                            = 0x11,
   IANA_IPv6ROUTING                    = 0x03,
   IANA_IPv6ROUTE                      = 0x2b,//used for source routing
   IANA_ICMPv6                         = 0x3a,
   IANA_ICMPv6_ECHO_REQUEST            =  128,
   IANA_ICMPv6_ECHO_REPLY              =  129,
   IANA_ICMPv6_RS                      =  133,
   IANA_ICMPv6_RA                      =  134,
   IANA_ICMPv6_RA_PREFIX_INFORMATION   =    3,
   IANA_ICMPv6_RPL                     =  155,
   IANA_ICMPv6_RPL_DIO                 = 0x01,
   IANA_ICMPv6_RPL_DAO                 = 0x02,
   IANA_RSVP                           =   46,
   IANA_UNDEFINED                      =  250, //use an unassigned
};

// well known ports (which we define)
// warning: first 4 MSB of 2° octect may coincide with previous protocol number
enum {
   //TCP
   WKP_TCP_HTTP                        =    80,
   WKP_TCP_ECHO                        =     7,
   //UDP
   WKP_UDP_COAP                        =  5683,
   WKP_UDP_ECHO                        =     7,
   WKP_UDP_INJECT                      =  2000,
   WKP_UDP_RINGMASTER                  = 15000,
};

//status elements
enum {
   STATUS_ISSYNC                       =  0,
   STATUS_ID                           =  1,
   STATUS_DAGRANK                      =  2,
   STATUS_OUTBUFFERINDEXES             =  3,
   STATUS_ASN                          =  4,
   STATUS_MACSTATS                     =  5,
   STATUS_SCHEDULE                     =  6,
   STATUS_BACKOFF                      =  7,
   STATUS_QUEUE                        =  8,
   STATUS_NEIGHBORS                    =  9,
   STATUS_KAPERIOD                     = 10,
   STATUS_MAX                          = 11,
};

//component identifiers
//the order is important because
enum {
   COMPONENT_NULL                      = 0x00,
   COMPONENT_OPENWSN                   = 0x01,
   //cross-layers
   COMPONENT_IDMANAGER                 = 0x02,
   COMPONENT_OPENQUEUE                 = 0x03,
   COMPONENT_OPENSERIAL                = 0x04,
   COMPONENT_PACKETFUNCTIONS           = 0x05,
   COMPONENT_RANDOM                    = 0x06,
   //PHY
   COMPONENT_RADIO                     = 0x07,
   //MAClow
   COMPONENT_IEEE802154                = 0x08,
   COMPONENT_IEEE802154E               = 0x09,
   
   // all components with higher component id than COMPONENT_IEEE802154E
   // won't be able to get free packets from the queue 
   // when the mote is not synch
   
   //MAClow<->MAChigh ("virtual components")
   COMPONENT_SIXTOP_TO_IEEE802154E     = 0x0a,
   COMPONENT_IEEE802154E_TO_SIXTOP     = 0x0b,
   //MAChigh
   COMPONENT_SIXTOP                    = 0x0c,
   COMPONENT_NEIGHBORS                 = 0x0d,
   COMPONENT_SCHEDULE                  = 0x0e,
   COMPONENT_SIXTOP_RES                = 0x0f,
   //IPHC
   COMPONENT_OPENBRIDGE                = 0x10,
   COMPONENT_IPHC                      = 0x11,
   //IPv6
   COMPONENT_FORWARDING                = 0x12,
   COMPONENT_ICMPv6                    = 0x13,
   COMPONENT_ICMPv6ECHO                = 0x14,
   COMPONENT_ICMPv6ROUTER              = 0x15,
   COMPONENT_ICMPv6RPL                 = 0x16,
   //TRAN
   COMPONENT_OPENTCP                   = 0x17,
   COMPONENT_OPENUDP                   = 0x18,
   COMPONENT_OPENCOAP                  = 0x19,
   // applications
   COMPONENT_C6T                       = 0x1a,
   COMPONENT_CEXAMPLE                  = 0x1b,
   COMPONENT_CINFO                     = 0x1c,
   COMPONENT_CLEDS                     = 0x1d,
   COMPONENT_CSENSORS                  = 0x1e,
   COMPONENT_CSTORM                    = 0x1f,
   COMPONENT_CWELLKNOWN                = 0x20,
   COMPONENT_TECHO                     = 0x21,
   COMPONENT_TOHLONE                   = 0x22,
   COMPONENT_UECHO                     = 0x23,
   COMPONENT_UINJECT                   = 0x24,
   COMPONENT_RRT                       = 0x25,
};



enum {
   //Applications
   //Transport
   PROPERTY_L4_PROTOCOL                      = 0x40,
   PROPERTY_L4_PROTOCOL_COMPRESSED           = 0x41,
   PROPERTY_L4_SOURCE_PORT_OR_ICMPV6_TYPE    = 0x42,
   PROPERTY_L4_DESTINATION_PORT              = 0x43,
   //Routing
   PROPERTY_L3_FRAME_TYPE                    = 0x30,
   PROPERTY_L3_FRAME_TYPE_DIO                = 0x31,
   PROPERTY_L3_FRAME_TYPE_DAO                = 0x32,
   PROPERTY_L3_FRAME_DESTINATION_ADDRESS     = 0x33,
   PROPERTY_L3_FRAME_SOURCE_ADDRESS          = 0x34,

   PROPERTY_L3_NODE_ADDRESS                  = 0x35,
   PROPERTY_L3_NODE_DAGRANK                  = 0x36,

   PROPERTY_L3_LINK_PARENT_PREFERENCE        = 0x37,
   PROPERTY_L3_LINK_NEIGHBOR_DAGRANK         = 0x38,
   PROPERTY_L3_LINK_PARENT_JOIN_PRIORITY     = 0x39,
   // 6TOP
   PROPERTY_L25_MAINTENANCE_PERIOD           = 0x20,
   PROPERTY_L25_KA_PERIOD                    = 0x21,

   PROPERTY_L25_FRAME_ID                     = 0x23,
   PROPERTY_L25_FRAME_TYPE                   = 0x24,
   PROPERTY_L25_NUM_CELLS                    = 0x25,

   //Mac
   PROPERTY_L2_FRAME_TYPE                    = 0x20,
   PROPERTY_L2_FRAME_DSN                     = 0x21,
   PROPERTY_L2_FRAME_RETRIES_LEFT            = 0x22,
   PROPERTY_L2_FRAME_TX_ATTEMPTS             = 0x23,
   PROPERTY_L2_FRAME_ASN                     = 0x24,
   PROPERTY_L2_FRAME_JOIN_PRIORITY           = 0x25,

   PROPERTY_L2_NODE_ADDRESS_64B              = 0x26,
   PROPERTY_L2_NODE_ADDRESS_16B              = 0x27,

   PROPERTY_L2_LINK_NUM_TX                   = 0x28,
   PROPERTY_L2_LINK_NUM_RX                   = 0x29,
   PROPERTY_L2_LINK_NUM_TX_ACK               = 0x2A,
   PROPERTY_L2_LINK_IS_STABLE                = 0x2B,
   PROPERTY_L2_LINK_NUM_STABILITY            = 0x2C,
   //Radio
   PROPERTY_L1_TX_POWER                      = 0x10,
   PROPERTY_L1_RSSI                          = 0x11,
   PROPERTY_L1_LQI                           = 0x12,
   PROPERTY_L1_CRC                           = 0x13,
   PROPERTY_L1_ADDRESS                       = 0x14,
   //Generic
   PROPERTY_ENTITY_LEVEL                     = 0x0e,
   PROPERTY_FRAME_DISSECTOR                  = 0x0f,
};

#define COMPONENT_NAME_RADIO                              "IEEE 802.15.4 radio"
#define COMPONENT_NAME_IEEE802154E                        "IEEE 802.15.4e"
#define COMPONENT_NAME_SIXTOP                             "6TOP"
#define COMPONENT_NAME_IPHC                               "IPHC"
#define COMPONENT_NAME_ICMPv6RPL                          "IETF RPL"
#define COMPONENT_NAME_OPENTCP                            "TCP"
#define COMPONENT_NAME_OPENUDP                            "UDP"
#define COMPONENT_NAME_OPENCOAP                           "COAP"
// applications
#define COMPONENT_NAME_C6T                                "C6T"
#define COMPONENT_NAME_CEXAMPLE                           "CEXAMPLE"
#define COMPONENT_NAME_CINFO                              "CINFO"
#define COMPONENT_NAME_CLEDS                              "CLEDS"
#define COMPONENT_NAME_CSTORM                             "CSTORM"
#define COMPONENT_NAME_CWELLKNOWN                         "CWELLKNOWN"
#define COMPONENT_NAME_TECHO                              "TCP ECHO"
#define COMPONENT_NAME_TOHLONE                            "TCP OHLONE"
#define COMPONENT_NAME_UECHO                              "UDP ECHO"
#define COMPONENT_NAME_RRT                                "RRT"

#define PROPERTY_NAME_L4_PROTOCOL                         "transport protocol"
#define PROPERTY_NAME_L4_PROTOCOL_COMPRESSED              "compressed"
#define PROPERTY_NAME_L4_SOURCE_PORT_OR_ICMPV6_TYPE       "source port/ICMPv6 type"
#define PROPERTY_NAME_L4_DESTINATION_PORT                 "destination port"
//Routing
#define PROPERTY_NAME_L3_FRAME_TYPE                       "type"
#define PROPERTY_NAME_L3_FRAME_TYPE_DIO                   "DIO"
#define PROPERTY_NAME_L3_FRAME_TYPE_DAO                   "DAO"
#define PROPERTY_NAME_L3_FRAME_DESTINATION_ADDRESS        "destination address"
#define PROPERTY_NAME_L3_FRAME_SOURCE_ADDRESS             "source address"
#define PROPERTY_NAME_L3_NODE_ADDRESS                     "address (128bits)"
#define PROPERTY_NAME_L3_NODE_DAGRANK                     "DAG rank"
#define PROPERTY_NAME_L3_LINK_PARENT_PREFERENCE           "parent preference"
#define PROPERTY_NAME_L3_LINK_NEIGHBOR_DAGRANK            "neighbor DAG rank"
#define PROPERTY_NAME_L3_LINK_PARENT_JOIN_PRIORITY        "parent join priority"
//6TOP
#define PROPERTY_NAME_L25_MAINTENANCE_PERIOD             "maintenance period"
#define PROPERTY_NAME_L25_KA_PERIOD                      "keep alive period"
#define PROPERTY_NAME_L25_FRAME_ID                       "frame id"
#define PROPERTY_NAME_L25_FRAME_TYPE                     "type"
#define PROPERTY_NAME_L25_FRAME_TYPE_KEEP_ALIVE          "keep alive"
#define PROPERTY_NAME_L25_FRAME_TYPE_ENHANCED_BEACON     "enhanced beacon"
#define PROPERTY_NAME_L25_NUM_CELLS                      "cells count"
//Mac
#define PROPERTY_NAME_L2_FRAME_TYPE                       "type"
#define PROPERTY_NAME_L2_FRAME_TYPE_BEACON                "beacon"
#define PROPERTY_NAME_L2_FRAME_TYPE_DATA                  "data"
#define PROPERTY_NAME_L2_FRAME_TYPE_ACK                   "ack"
#define PROPERTY_NAME_L2_FRAME_DSN                        "data sequence number"
#define PROPERTY_NAME_L2_FRAME_RETRIES_LEFT               "retries left"
#define PROPERTY_NAME_L2_FRAME_TX_ATTEMPTS                "tx attempts"
#define PROPERTY_NAME_L2_FRAME_ASN                        "absolute slot number"
#define PROPERTY_NAME_L2_FRAME_JOIN_PRIORITY              "advertised join priority"
#define PROPERTY_NAME_L2_NODE_ADDRESS_64B                 "address (64bits)"
#define PROPERTY_NAME_L2_NODE_ADDRESS_16B                 "address (16bits)"
#define PROPERTY_NAME_L2_LINK_NUM_TX                      "TX count"
#define PROPERTY_NAME_L2_LINK_NUM_RX                      "RX count"
#define PROPERTY_NAME_L2_LINK_NUM_TX_ACK                  "TX ACK count"
#define PROPERTY_NAME_L2_LINK_IS_STABLE                   "link stable?"
#define PROPERTY_NAME_L2_LINK_NUM_STABILITY               "link stability counter"

//Radio
#define PROPERTY_NAME_L1_TX_POWER                         "TX power"
#define PROPERTY_NAME_L1_RSSI                             "RSSI"
#define PROPERTY_NAME_L1_LQI                              "LQI"
#define PROPERTY_NAME_L1_CRC                              "CRC verified?"
#define PROPERTY_NAME_L1_ADDRESS                          "address (64bits)"

//dissector names
#define DISSECTOR_NAME_COAP                              "coap"
#define DISSECTOR_NAME_UDP                               "udp"
#define DISSECTOR_NAME_TCP                               "tcp"
#define DISSECTOR_NAME_ICMPv6RPL                         "ipv6"
#define DISSECTOR_NAME_IEEE80215                         "wpan"

#define ENTITY_APPLICATION_LEVEL           7
#define ENTITY_TRANSPORT_LEVEL             4
#define ENTITY_NETWORK_LEVEL               3
#define ENTITY_IPHC_LEVEL                  2.75
#define ENTITY_CROSS_NET_LINK_LEVEL        2.5
#define ENTITY_LINK_LEVEL                  2
#define ENTITY_RADIO_LEVEL                 1


/**
\brief error codes used throughout the OpenWSN stack

\note The comments are used in the Python parsing tool:
   - {0} refers to the value of the first argument,
   - {1} refers to the value of the second argument,
*/
enum {
   // l7
   ERR_RCVD_ECHO_REQUEST               = 0x01, // received an echo request
   ERR_RCVD_ECHO_REPLY                 = 0x02, // received an echo reply
   ERR_GETDATA_ASKS_TOO_FEW_BYTES      = 0x03, // getData asks for too few bytes, maxNumBytes={0}, fill level={1}
   ERR_INPUT_BUFFER_OVERFLOW           = 0x04, // the input buffer has overflown
   ERR_COMMAND_NOT_ALLOWED             = 0x05, // the command is not allowerd, command = {0} 
   // l4
   ERR_WRONG_TRAN_PROTOCOL             = 0x06, // unknown transport protocol {0} (code location {1})
   ERR_WRONG_TCP_STATE                 = 0x07, // wrong TCP state {0} (code location {1})
   ERR_TCP_RESET                       = 0x08, // TCP reset while in state {0} (code location {1})
   ERR_UNSUPPORTED_PORT_NUMBER         = 0x09, // unsupported port number {0} (code location {1})
   // l3
   ERR_UNEXPECTED_DAO                  = 0x0a, // unexpected DAO (code location {0})
   ERR_UNSUPPORTED_ICMPV6_TYPE         = 0x0b, // unsupported ICMPv6 type {0} (code location {1})
   ERR_6LOWPAN_UNSUPPORTED             = 0x0c, // unsupported 6LoWPAN parameter {1} at location {0}
   ERR_NO_NEXTHOP                      = 0x0d, // no next hop
   ERR_INVALID_PARAM                   = 0x0e, // invalid parameter
   ERR_INVALID_FWDMODE                 = 0x0f, // invalid forward mode
   ERR_LARGE_DAGRANK                   = 0x10, // large DAGrank {0}, set to {1}
   ERR_HOP_LIMIT_REACHED               = 0x11, // packet discarded hop limit reached
   ERR_LOOP_DETECTED                   = 0x12, // loop detected due to previous rank {0} lower than current node rank {1}
   ERR_WRONG_DIRECTION                 = 0x13, // upstream packet set to be downstream, possible loop.
   // l2b
   ERR_NEIGHBORS_FULL                  = 0x14, // neighbors table is full (max number of neighbor is {0})
   ERR_NO_SENT_PACKET                  = 0x15, // there is no sent packet in queue
   ERR_NO_RECEIVED_PACKET              = 0x16, // there is no received packet in queue
   ERR_SCHEDULE_OVERFLOWN              = 0x17, // schedule overflown
   // l2a
   ERR_WRONG_CELLTYPE                  = 0x18, // wrong celltype {0} at slotOffset {1}
   ERR_IEEE154_UNSUPPORTED             = 0x19, // unsupported IEEE802.15.4 parameter {1} at location {0}
   ERR_DESYNCHRONIZED                  = 0x1a, // got desynchronized at slotOffset {0}
   ERR_SYNCHRONIZED                    = 0x1b, // synchronized at slotOffset {0}
   ERR_LARGE_TIMECORRECTION            = 0x1c, // large timeCorr.: {0} ticks (code loc. {1})
   ERR_WRONG_STATE_IN_ENDFRAME_SYNC    = 0x1d, // wrong state {0} in end of frame+sync
   ERR_WRONG_STATE_IN_STARTSLOT        = 0x1e, // wrong state {0} in startSlot, at slotOffset {1}
   ERR_WRONG_STATE_IN_TIMERFIRES       = 0x1f, // wrong state {0} in timer fires, at slotOffset {1}
   ERR_WRONG_STATE_IN_NEWSLOT          = 0x20, // wrong state {0} in start of frame, at slotOffset {1}
   ERR_WRONG_STATE_IN_ENDOFFRAME       = 0x21, // wrong state {0} in end of frame, at slotOffset {1}
   ERR_MAXTXDATAPREPARE_OVERFLOW       = 0x22, // maxTxDataPrepare overflows while at state {0} in slotOffset {1}
   ERR_MAXRXACKPREPARE_OVERFLOWS       = 0x23, // maxRxAckPrepapare overflows while at state {0} in slotOffset {1}
   ERR_MAXRXDATAPREPARE_OVERFLOWS      = 0x24, // maxRxDataPrepapre overflows while at state {0} in slotOffset {1}
   ERR_MAXTXACKPREPARE_OVERFLOWS       = 0x25, // maxTxAckPrepapre overflows while at state {0} in slotOffset {1}
   ERR_WDDATADURATION_OVERFLOWS        = 0x26, // wdDataDuration overflows while at state {0} in slotOffset {1}
   ERR_WDRADIO_OVERFLOWS               = 0x27, // wdRadio overflows while at state {0} in slotOffset {1}
   ERR_WDRADIOTX_OVERFLOWS             = 0x28, // wdRadioTx overflows while at state {0} in slotOffset {1}
   ERR_WDACKDURATION_OVERFLOWS         = 0x29, // wdAckDuration overflows while at state {0} in slotOffset {1}
   // general
   ERR_BUSY_SENDING                    = 0x2a, // busy sending
   ERR_UNEXPECTED_SENDDONE             = 0x2b, // sendDone for packet I didn't send
   ERR_NO_FREE_PACKET_BUFFER           = 0x2c, // no free packet buffer (code location {0})
   ERR_FREEING_UNUSED                  = 0x2d, // freeing unused memory
   ERR_FREEING_ERROR                   = 0x2e, // freeing memory unsupported memory
   ERR_UNSUPPORTED_COMMAND             = 0x2f, // unsupported command {0}
   ERR_MSG_UNKNOWN_TYPE                = 0x30, // unknown message type {0}
   ERR_WRONG_ADDR_TYPE                 = 0x31, // wrong address type {0} (code location {1})
   ERR_BRIDGE_MISMATCH                 = 0x32, // bridge mismatch (code location {0})
   ERR_HEADER_TOO_LONG                 = 0x33, // header too long, length {1} (code location {0})
   ERR_INPUTBUFFER_LENGTH              = 0x34, // input length problem, length={0}
   ERR_BOOTED                          = 0x35, // booted
   ERR_INVALIDSERIALFRAME              = 0x36, // invalid serial frame
   ERR_INVALIDPACKETFROMRADIO          = 0x37, // invalid packet frome radio, length {1} (code location {0})
   ERR_BUSY_RECEIVING                  = 0x38, // busy receiving when stop of serial activity, buffer input length {1} (code location {0})
   ERR_WRONG_CRC_INPUT                 = 0x39, // wrong CRC in input Buffer (input length {0})
};

//=========================== typedef =========================================


typedef uint16_t  errorparameter_t;
typedef uint16_t  dagrank_t;
typedef uint8_t   owerror_t;

BEGIN_PACK
typedef struct {
   uint8_t  byte4;
   uint16_t bytes2and3;
   uint16_t bytes0and1;
} asn_t;
END_PACK

BEGIN_PACK
typedef struct {                                 // always written big endian, i.e. MSB in addr[0]
   uint8_t type;
   union {
      uint8_t addr_16b[2];
      uint8_t addr_64b[8];
      uint8_t addr_128b[16];
      uint8_t panid[2];
      uint8_t prefix[8];
   };
} open_addr_t;
END_PACK

typedef struct {
   //admin
   uint8_t       creator;                        // the component which called getFreePacketBuffer()

   uint8_t       id;                             // unique ID, used by the observer library

   uint8_t       owner;                          // the component which currently owns the entry
   uint8_t*      payload;                        // pointer to the start of the payload within 'packet'
   uint8_t       length;                         // length in bytes of the payload
   //l4
   uint8_t       l4_protocol;                    // l4 protocol to be used
   bool          l4_protocol_compressed;         // is the l4 protocol header compressed?
   uint16_t      l4_sourcePortORicmpv6Type;      // l4 source port
   uint16_t      l4_destination_port;            // l4 destination port
   uint8_t*      l4_payload;                     // pointer to the start of the payload of l4 (used for retransmits)
   uint8_t       l4_length;                      // length of the payload of l4 (used for retransmits)
   //l3
   open_addr_t   l3_destinationAdd;              // 128b IPv6 destination (down stack) 
   open_addr_t   l3_sourceAdd;                   // 128b IPv6 source address 
   //l2
   owerror_t     l2_sendDoneError;               // outcome of trying to send this packet
   open_addr_t   l2_nextORpreviousHop;           // 64b IEEE802.15.4 next (down stack) or previous (up) hop address
   uint8_t       l2_frameType;                   // beacon, data, ack, cmd
   uint8_t       l2_dsn;                         // sequence number of the received frame
   uint8_t       l2_retriesLeft;                 // number Tx retries left before packet dropped (dropped when hits 0)
   uint8_t       l2_numTxAttempts;               // number Tx attempts
   asn_t         l2_asn;                         // at what ASN the packet was Tx'ed or Rx'ed
   uint8_t*      l2_payload;                     // pointer to the start of the payload of l2 (used for MAC to fill in ASN in ADV)
   uint8_t*      l2_scheduleIE_cellObjects;      // pointer to the start of cell Objects in scheduleIE
   uint8_t       l2_scheduleIE_numOfCells;       // number of cells were going to be scheduled or removed.
   uint8_t       l2_scheduleIE_frameID;          // frameID in scheduleIE
   uint8_t*      l2_ASNpayload;                  // pointer to the ASN in EB
   uint8_t       l2_joinPriority;                // the join priority received in EB
   bool          l2_IEListPresent;               //did have IE field?
   bool          l2_payloadIEpresent;            // did I have payload IE field
   bool          l2_joinPriorityPresent;
   //l1 (drivers)
   uint8_t       l1_txPower;                     // power for packet to Tx at
   int8_t        l1_rssi;                        // RSSI of received packet
   uint8_t       l1_lqi;                         // LQI of received packet
   bool          l1_crc;                         // did received packet pass CRC check?
   //the packet
   uint8_t       packet[1+1+125+2+1];            // 1B spi address, 1B length, 125B data, 2B CRC, 1B LQI
} OpenQueueEntry_t;


BEGIN_PACK
typedef struct {

   uint8_t          id;

   bool             used;
   uint8_t          parentPreference;
   bool             stableNeighbor;
   uint8_t          switchStabilityCounter;
   open_addr_t      addr_64b;
   dagrank_t        DAGrank;
   int8_t           rssi;
   uint8_t          numRx;
   uint8_t          numTx;
   uint8_t          numTxACK;
   uint8_t          numWraps;//number of times the tx counter wraps. can be removed if memory is a restriction. also check openvisualizer then.
   asn_t            asn;
   uint8_t          joinPrio;
} neighborRow_t;
END_PACK


//=========================== variables =======================================

//=========================== prototypes ======================================

#endif

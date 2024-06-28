#ifndef _RNDIS_PROTOCOL_H
#define _RNDIS_PROTOCOL_H

#include <cstdint>

#define RNDIS_MSG_PACKET              0x00000001
#define RNDIS_MSG_INITIALIZE          0x00000002
#define RNDIS_MSG_INITIALIZE_CMPLT    0x80000002
#define RNDIS_MSG_HALT                0x00000003
#define RNDIS_MSG_QUERY               0x00000004
#define RNDIS_MSG_QUERY_CMPLT         0x80000004
#define RNDIS_MSG_SET                 0x00000005
#define RNDIS_MSG_SET_CMPLT           0x80000005
#define RNDIS_MSG_RESET               0x00000006
#define RNDIS_MSG_RESET_CMPLT         0x80000006
#define RNDIS_MSG_INDICATE_STATUS     0x00000007
#define RNDIS_MSG_KEEPALIVE           0x00000008
#define RNDIS_MSG_KEEPALIVE_CMPLT     0x80000008

#define RNDIS_STATUS_SUCCESS          0x00000000
#define RNDIS_STATUS_FAILURE          0xC0000001
#define RNDIS_STATUS_INVALID_DATA     0xC0010015
#define RNDIS_STATUS_UNSUPPORTED      0xC00000BB
#define RNDIS_STATUS_MEDIA_CONNECT    0x4001000B
#define RNDIS_STATUS_MEDIA_DISCONNECT 0x4001000C

#define OID_GEN_SUPPORTED_LIST        0x00010101
#define OID_GEN_HARDWARE_STATUS       0x00010102
#define OID_GEN_MEDIA_SUPPORTED       0x00010103
#define OID_GEN_MEDIA_IN_USE          0x00010104
#define OID_GEN_MAXIMUM_FRAME_SIZE    0x00010106   // MTU
#define OID_GEN_LINK_SPEED            0x00010107
#define OID_GEN_TRANSMIT_BLOCK_SIZE   0x0001010A
#define OID_GEN_RECEIVE_BLOCK_SIZE    0x0001010B
#define OID_GEN_VENDOR_ID             0x0001010C
#define OID_GEN_VENDOR_DESCRIPTION    0x0001010D
#define OID_GEN_CURRENT_PACKET_FILTER 0x0001010E
#define OID_GEN_MAXIMUM_TOTAL_SIZE    0x00010111   // MAX_FRAME_LEN
#define OID_GEN_MEDIA_CONNECT_STATUS  0x00010114
#define OID_GEN_VENDOR_DRIVER_VERSION 0x00010116
#define OID_GEN_MEDIA_CAPABILITIES    0x00010201
#define OID_GEN_PHYSICAL_MEDIUM       0x00010202

#define OID_802_3_PERMANENT_ADDRESS   0x01010101
#define OID_802_3_CURRENT_ADDRESS     0x01010102
#define OID_802_3_MULTICAST_LIST      0x01010103
#define OID_802_3_MAXIMUM_LIST_SIZE   0x01010104

#define NDIS_MEDIUM_802_3             0x00000000

#define NDIS_PACKET_TYPE_DIRECTED       (1<<0)  // unicast (to this mac)
#define NDIS_PACKET_TYPE_MULTICAST      (1<<1)  // multicast for addresses in the multicast list
#define NDIS_PACKET_TYPE_ALL_MULTICAST  (1<<2)  // all multicast traffic (disregards subscription)
#define NDIS_PACKET_TYPE_BROADCAST      (1<<3)  // broadcast (to FF:FF:FF:FF:FF:FF)
#define NDIS_PACKET_TYPE_SOURCE_ROUTING (1<<4)
#define NDIS_PACKET_TYPE_PROMISCUOUS    (1<<5)
#define NDIS_PACKET_TYPE_SMT            (1<<6)
#define NDIS_PACKET_TYPE_ALL_LOCAL      (1<<7)
#define NDIS_PACKET_TYPE_GROUP          (1<<8)
#define NDIS_PACKET_TYPE_ALL_FUNCTIONAL (1<<9)
#define NDIS_PACKET_TYPE_FUNCTIONAL     (1<<10)
#define NDIS_PACKET_TYPE_MAC_FRAME      (1<<11)

#define NDIS_MEDIA_STATE_CONNECTED    0x00000000
#define NDIS_MEDIA_STATE_DISCONNECTED 0x00000001

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t RequestID;
} rndis_command;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t RequestID;
  uint32_t Status;
} rndis_msg_keepalive_cmplt;
typedef rndis_msg_keepalive_cmplt rndis_msg_set_cmplt;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t RequestID;
  uint32_t Status;
  uint32_t MajorVersion;
  uint32_t MinorVersion;
  uint32_t DeviceFlags;
  uint32_t Medium;
  uint32_t MaxPacketsPerTransfer;
  uint32_t MaxTransferSize;
  uint32_t PacketAlignmentFactor;
  uint8_t Reserved[8];
} rndis_msg_initialize_cmplt;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t RequestID;
  uint32_t Status;
  uint32_t InformationBufferLength;
  uint32_t InformationBufferOffset;
  uint8_t OIDInputBuffer[];
} rndis_msg_query_cmplt;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t Status;
  uint32_t AddressingReset;
} rndis_msg_reset_cmplt;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t RequestID;
  uint32_t MajorVersion;
  uint32_t MinorVersion;
  uint32_t MaxTransferSize;
  typedef rndis_msg_initialize_cmplt response_t;
} rndis_msg_initialize;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t RequestID;
} rndis_msg_halt;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t RequestID;
  uint32_t Oid;
  uint32_t InformationBufferLength;
  uint32_t InformationBufferOffset;
  uint32_t Reserved;
  uint8_t OIDInputBuffer[];
  typedef rndis_msg_query_cmplt response_t;
} rndis_msg_query;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t RequestID;
  uint32_t Oid;
  uint32_t InformationBufferLength;
  uint32_t InformationBufferOffset;
  uint32_t Reserved;
  uint8_t OIDInputBuffer[];
  typedef rndis_msg_set_cmplt response_t;
} rndis_msg_set;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t Reserved;
  typedef rndis_msg_reset_cmplt response_t;
} rndis_msg_reset;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t RequestID;
  typedef rndis_msg_keepalive_cmplt response_t;
} rndis_msg_keepalive;

typedef struct rndis_msg_indicate_status {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t Status;
  uint32_t StatusBufferLength;
  uint32_t StatusBufferOffset;
  uint8_t StatusBuffer[];
} rndis_msg_indicate_status;

typedef struct {
  uint32_t MessageType;
  uint32_t MessageLength;
  uint32_t DataOffset;
  uint32_t DataLength;
  uint32_t OutOfBandDataOffset;
  uint32_t OutOfBandDataLength;
  uint32_t NumOutOfBandDataElements;
  uint32_t PerPacketInfoOffset;
  uint32_t PerPacketInfoLength;
  uint8_t Reserved[8];
  uint8_t Payload[];
} rndis_packet;

#endif // _RNDIS_PROTOCOL_H

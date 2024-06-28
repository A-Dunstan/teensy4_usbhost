/*
  Copyright (C) 2024 Andrew Dunstan
  This file is part of teensy4_usbhost.

  teensy4_usbhost is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <alloca.h>
#include <cstring>
#include <cstdio>
#include "rndis.h"
#include "rndis_protocol.h"
#include "../usbhost_utility.h"

#define KEEPALIVE_TIMEOUT             (5*SYSTEM_TICKS_PER_SEC)
#define CONTROL_TIMEOUT               (10*SYSTEM_TICKS_PER_SEC)

#define CDC_CMD_SEND_ENCAPSULATED     0x0000
#define CDC_CMD_GET_ENCAPSULATED      0x0001

void USB_RNDIS::verifyCommandSent(int r, uint32_t MessageLength) {
  if (r < (int)MessageLength) {
    printf("bad command transfer length: %lu %d\n", MessageLength, r);
    auto s = atomMutexGet(&lock, 10);
    resp_buf[0] = resp_buf[1] = 0;
    atomCondSignal(&cmd_signal);
    if (s == ATOM_OK) atomMutexPut(&lock);
  }
}

template<class ccommand>
const typename ccommand::response_t* USB_RNDIS::sendCommand(const ccommand& cmd, const rndis_msg_indicate_status** err) {
  const typename ccommand::response_t* resp = NULL;
  if (err) *err = NULL;

  // if sending the command fails the response signal must still be set to avoid deadlock
  USBCallback send_cb = [=,MessageLength=cmd.MessageLength](int r) { verifyCommandSent(r,MessageLength); };

  /* this has to be an asynchronous call because we currently own the lock; if an interrupt arrives the
   * USB thread will try to take it and will deadlock / never complete this control transfer. So
   * instead the command gets queued here, then the lock gets released while we wait for the response
   * signal.
   */
  if (ControlMessage(USB_REQTYPE_INTERFACE_SET|USB_CTRLTYPE_TYPE_CLASS, CDC_CMD_SEND_ENCAPSULATED, 0, control_interface, cmd.MessageLength, const_cast<ccommand*>(&cmd), send_cb) < 0) {
    puts("Failed to queue command for sending\n");
    return NULL;
  }
  if (atomCondWait(&cmd_signal, &lock, CONTROL_TIMEOUT) != ATOM_OK) {
    puts("Failed to get command response signal");
    return NULL;
  }

  // response is now in the buffer
  auto r = (const typename ccommand::response_t*)resp_buf;
  if (r->MessageType != (cmd.MessageType | 0x80000000))
    puts("Response had incorrect type");
  else if (r->RequestID != cmd.RequestID)
    puts("Response had incorrect request ID");
  else if (r->MessageLength < sizeof(*r))
    printf("Response had incorrect length: %lu instead of %u\n", r->MessageLength, sizeof(*r));
  else
    return r;

  // TODO: check the status buffer to make sure this indicate_status refers to the command that was just sent
  if (err && r->MessageType==RNDIS_MSG_INDICATE_STATUS) {
    *err = (const rndis_msg_indicate_status*)resp_buf;
  }

  return resp;
}

void USB_RNDIS::control_in(int r) {
  uint32_t MessageType = resp_buf[0];
  uint32_t MessageLength = resp_buf[1];
  printf("control response: %d %08lX\n", r, MessageType);
  if (r >= 8 && r >= (int)MessageLength) {
    switch(MessageType) {
      case RNDIS_MSG_INITIALIZE_CMPLT:
      case RNDIS_MSG_QUERY_CMPLT:
      case RNDIS_MSG_SET_CMPLT:
      case RNDIS_MSG_RESET_CMPLT:
      case RNDIS_MSG_KEEPALIVE_CMPLT:
        // response to request has arrived, let main thread know
        atomCondSignal(&cmd_signal);
        break;
      case RNDIS_MSG_KEEPALIVE:
        // device wants to know if we're alive...
        // let the main thread handle it
        if (MessageLength >= sizeof(rndis_msg_keepalive)) {
          driver_msg keepalive_request = {
            .type = DRIVER_MSG_KEEPALIVE_REQUEST,
            .RequestID = resp_buf[2] // RequestID
          };
          atomQueuePut(&queue, -1, &keepalive_request);
        } // else send indicate status to signal error?
        break;
      case RNDIS_MSG_INDICATE_STATUS:
        if (MessageLength >= sizeof(rndis_msg_indicate_status)) {
          auto s = (const rndis_msg_indicate_status*)resp_buf;
          if (s->Status == RNDIS_STATUS_MEDIA_CONNECT)
            puts("RNDIS Indicate_Status: MEDIA_CONNECT");
          else if (s->Status == RNDIS_STATUS_MEDIA_DISCONNECT)
            puts("RNDIS Indicate_Status: MEDIA_DISCONNECT");
          else {
            // return it to the waiting handler
            printf("RNDIS Indicate_Status: %08lx\n", s->Status);
            atomCondSignal(&cmd_signal);
          }
          break;
        }
      case RNDIS_MSG_HALT:
      default:
        printf("UNHANDLED RESPONSE FROM DEVICE: %lu\n", MessageType);
    }
    InterruptMessage(status_endpoint, 8, status, &status_cb);
  }
  else printf("Unexpected control response: %d %lu %lu\n", r, MessageType, MessageLength);
  atomMutexPut(&lock);
}

void USB_RNDIS::status_in(int r) {
  if (r==8 && status[0]==1) {
    if (atomMutexGet(&lock, 0) == ATOM_OK) {
      if (ControlMessage(USB_REQTYPE_INTERFACE_GET|USB_CTRLTYPE_TYPE_CLASS, CDC_CMD_GET_ENCAPSULATED, 0, control_interface, sizeof(resp_buf), resp_buf, &control_cb) < 0) {
        puts("Failed to retrieve RNDIS control response");
        atomMutexPut(&lock);
      }
    }
    else puts("Status in failed to get lock");
  }
  else printf("Unexpected RNDIS status response: %d %lu\n", r, status[0]);
}

uint32_t USB_RNDIS::rndis_set(uint32_t OID, const void* setIn, uint32_t lengthIn) {
  uint32_t status = RNDIS_STATUS_FAILURE;
  uint32_t request = rID++;
  rndis_msg_set *set = (rndis_msg_set*)alloca(sizeof(rndis_msg_set)+lengthIn);
  set->MessageType = RNDIS_MSG_SET;
  set->MessageLength = sizeof(rndis_msg_set) + lengthIn;
  set->RequestID = request;
  set->Oid = OID;
  set->InformationBufferLength = lengthIn;
  set->InformationBufferOffset = (lengthIn==0) ? 0 : (sizeof(rndis_msg_set) - 8);
  set->Reserved = 0;
  if (lengthIn) memcpy(set->OIDInputBuffer, setIn, lengthIn);

  if (atomMutexGet(&lock, 0) == ATOM_OK) {
    auto sc = sendCommand(*set);
    if (sc) status = sc->Status;
    atomMutexPut(&lock);
  }

  return status;
}

uint32_t USB_RNDIS::rndis_query(uint32_t OID, void* queryOut, uint32_t lengthOut, const void* queryIn, uint32_t lengthIn) {
  uint32_t r = 0xFFFFFFFF;
  uint32_t request = rID++;
  rndis_msg_query *query = (rndis_msg_query*)alloca(sizeof(rndis_msg_query)+lengthIn);
  query->MessageType = RNDIS_MSG_QUERY;
  query->MessageLength = sizeof(*query) + lengthIn;
  query->RequestID = request;
  query->Oid = OID;
  query->InformationBufferLength = lengthIn;
  query->InformationBufferOffset = (lengthIn==0) ? 0 : (sizeof(rndis_msg_query) - 8);
  query->Reserved = 0;
  if (lengthIn) memcpy(query->OIDInputBuffer, queryIn, lengthIn);

  if (atomMutexGet(&lock, 0) == ATOM_OK) {
    auto qc = sendCommand(*query);
    if (qc && qc->Status == RNDIS_STATUS_SUCCESS) {
      r = qc->InformationBufferLength;
      if (lengthOut>0 && queryOut!=NULL) {
        if (lengthOut > r) lengthOut = r;
        uint32_t offset = qc->InformationBufferOffset + 8 - offsetof(rndis_msg_query_cmplt, OIDInputBuffer);
        memcpy(queryOut, qc->OIDInputBuffer+offset, lengthOut);
      }
    }
    atomMutexPut(&lock);
  }
  return r;
}

void USB_RNDIS::init_oids(void) {
  if (rndis_query(OID_802_3_CURRENT_ADDRESS, mac_address, sizeof(mac_address)) != sizeof(mac_address))
    return;
  printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
  if (rndis_query(OID_GEN_MAXIMUM_FRAME_SIZE, &MTU, sizeof(MTU)) != sizeof(MTU))
    return;
  printf("MTU: %lu\n", MTU);
  if (rndis_query(OID_GEN_MAXIMUM_TOTAL_SIZE, &MAX_FRAME_LEN, sizeof(MAX_FRAME_LEN)) != sizeof(MAX_FRAME_LEN))
    return;
  printf("MAX_FRAME_LEN: %lu\n", MAX_FRAME_LEN);
  uint32_t packet_filter = 0xF;
  uint32_t status = rndis_set(OID_GEN_CURRENT_PACKET_FILTER, &packet_filter, sizeof(packet_filter));
  if (status != RNDIS_STATUS_SUCCESS)
    return;
  if (rndis_query(OID_GEN_CURRENT_PACKET_FILTER, &packet_filter, sizeof(packet_filter)) != sizeof(packet_filter))
    return;
  printf("Packet filter was set to 0x%02lX\n", packet_filter);
  if (rndis_query(OID_GEN_LINK_SPEED, &link_speed, sizeof(link_speed)) != sizeof(link_speed))
    return;
  printf("Link Speed: %lu\n", link_speed);
  data_receive();
}

void USB_RNDIS::data_in(int r) {
  data_receive_active = false;
  if (r < 0) {
    printf("Error from data_in: %d\n", r);
    return;
  }

  if (r > 0) {
    data_end = data_buf + r;
    return;
  }
  data_receive();
}

size_t USB_RNDIS::data_unpack(void* p) {
  if (data_end == NULL)
    return 0;

  size_t r = 0;
  do {
    size_t len = data_end - data_buf;
    if (len < sizeof(rndis_packet)) break;
    rndis_packet* packet = (rndis_packet*)data_buf;
    if (packet->MessageType != RNDIS_MSG_PACKET) {
      puts("input packet did not have RNDIS_MSG_PACKET type");
      break;
    }
    if (packet->MessageLength > len) {
      printf("packet MessageLength was too long: %lu vs %u\n", packet->MessageLength, len);
      break;
    }
    if (packet->DataOffset+8+packet->DataLength > packet->MessageLength) {
      puts("packet data length was too long");
      break;
    }

    if (packet->OutOfBandDataLength != 0) {
      puts("packet contains out-of-band data, skipping");
    } else if (packet->PerPacketInfoLength != 0) {
      puts("packet contains PerPacketInfo, skipping");
    } else {
      r = packet->DataLength;
    }
    const uint8_t* data_next = data_buf + packet->MessageLength;

    if (r) {
      if (p != NULL) {
        memcpy(p, data_buf+packet->DataOffset+8, r);
        if (data_next >= data_end) break;
        memmove(data_buf, data_next, data_end - data_next);
        data_end -= packet->MessageLength;
      }
      return r;
    }

    memmove(data_buf, data_next, data_end - data_next);
    data_end -= packet->MessageLength;
  } while (data_end > data_buf);

  data_receive();
  return r;
}

void USB_RNDIS::data_receive(void) {
  if (!data_receive_active) {
    data_end = NULL;
    data_receive_active = true;
    if (BulkMessage(data_in_endpoint, sizeof(data_buf), data_buf, &dataIn_cb) < 0) {
      puts("Failed to queue BulkMessage (receive)");
      data_receive_active = false;
    }
  }
}

void USB_RNDIS::rndis_initialize(void) {
  uint32_t request = rID++;
  const rndis_msg_initialize initmsg = {
    .MessageType = RNDIS_MSG_INITIALIZE,
    .MessageLength = sizeof(rndis_msg_initialize),
    .RequestID = request,
    .MajorVersion = 1,
    .MinorVersion = 0,
    .MaxTransferSize = sizeof(resp_buf)
  };
  if (atomMutexGet(&lock, 0) == ATOM_OK) {
    auto init_cmplt = sendCommand(initmsg);
    if (init_cmplt && init_cmplt->Status==RNDIS_STATUS_SUCCESS) {
      printf("Initialization complete:\n");
      printf("\tMajorVersion: %08lX\n", init_cmplt->MajorVersion);
      printf("\tMinorVersion: %08lX\n", init_cmplt->MinorVersion);
      printf("\tDeviceFlags: %08lX\n", init_cmplt->DeviceFlags);
      printf("\tMedium: %08lX\n", init_cmplt->Medium);
      printf("\tMaxPacketsPerTransfer: %08lX\n", init_cmplt->MaxPacketsPerTransfer);
      printf("\tMaxTransferSize: %08lX\n", init_cmplt->MaxTransferSize);
      printf("\tPacketAlignmentFactor: %08lX\n", init_cmplt->PacketAlignmentFactor);
      max_transfer_packets = init_cmplt->MaxPacketsPerTransfer;
      max_transfer_size = init_cmplt->MaxTransferSize;
      init_oids();
    }
    atomMutexPut(&lock);
  }
}

void USB_RNDIS::threadproc(void) {
  driver_msg msg;
  while (atomQueueGet(&queue, 0, &msg) == ATOM_OK) {
    if (msg.type != DRIVER_MSG_RECV)
      printf("RNDIS thread drivermsg: %d\n", msg.type);
    switch (msg.type) {
      case DRIVER_MSG_RECV:
        *(msg.len) = data_unpack(msg.buf);
        atomSemPut(msg.signal);
        break;
      case DRIVER_MSG_GET_MAC_ADDR:
        if (link_speed > 0) {
          memcpy(msg.buf, mac_address, 6);
        } else {
          memset(msg.buf, 0, 6);
        }
        atomSemPut(msg.signal);
        break;
      case DRIVER_MSG_KEEPALIVE_REQUEST:
        {
          rndis_msg_keepalive_cmplt kc = {
            .MessageType = RNDIS_MSG_KEEPALIVE_CMPLT,
            .MessageLength = sizeof(rndis_msg_keepalive_cmplt),
            .RequestID = msg.RequestID,
            .Status = RNDIS_STATUS_SUCCESS
          };
          ControlMessage(USB_REQTYPE_INTERFACE_SET|USB_CTRLTYPE_TYPE_CLASS, CDC_CMD_SEND_ENCAPSULATED, 0, control_interface, kc.MessageLength, &kc);
        }
        break;
      case DRIVER_MSG_DETACH:
        printf("RNDIS detached\n");
        link_speed = 0;
        break;
      case DRIVER_MSG_ATTACH:
        // begin polling for status
        InterruptMessage(status_endpoint, 8, status, &status_cb);
        rndis_initialize();
        break;
    }
  }
}

bool USB_RNDIS::parse_config(const usb_configuration_descriptor* c, USB_RNDIS *p) {
  const uint8_t* b = (const uint8_t*)(c+1);
  const uint8_t* end = (const uint8_t*)c + c->wTotalLength;

  while (b < end) {
    auto iad = get_desc_type<usb_interface_association_descriptor>(b, USB_DT_INTERFACE_ASSOCIATION, end);
    if (iad) {
      // class: miscellaneous(239), subclass: 4(RNDIS), protocol: 1(RNDIS over Ethernet)
      if ((iad->bInterfaceCount==2 && iad->bFunctionClass==239 && iad->bFunctionSubClass==4 && iad->bFunctionProtocol==1) ||
          (iad->bInterfaceCount==2 && iad->bFunctionClass==224 && iad->bFunctionSubClass==1 && iad->bFunctionProtocol==3)) {
        const usb_interface_descriptor *control = NULL, *data = NULL;
        const uint8_t* bb = (const uint8_t*)(iad+1);
        for (int i=0; bb < end && i < iad->bInterfaceCount;) {
          auto iface = get_desc_type<usb_interface_descriptor>(bb, USB_DT_INTERFACE, end);
          if (iface && iface->bAlternateSetting==0) {
            ++i;
            if (iface->bNumEndpoints>=1 && iface->bInterfaceClass==iad->bFunctionClass && iface->bInterfaceSubClass==iad->bFunctionSubClass && iface->bInterfaceProtocol==iad->bFunctionProtocol && control==NULL) {
              // looks like a control interface... find an IN interrupt endpoint with wMaxPacketSize==8
              const uint8_t* bbb = (const uint8_t*)(iface+1);
              for (int j=0; bbb < end && j < iface->bNumEndpoints;) {
                auto ep = get_desc_type<usb_endpoint_descriptor>(bbb, USB_DT_ENDPOINT, end);
                if (ep) {
                  ++j;
                  if (ep->bEndpointAddress & 0x80 && (ep->bmAttributes&3)==USB_ENDPOINT_INTERRUPT && ep->wMaxPacketSize==8) {
                    control = iface;
                    if (p) {
                      p->control_interface = iface->bInterfaceNumber;
                      p->status_endpoint = ep->bEndpointAddress;
                    }
                    break;
                  }
                }
                bbb += bbb[0];
              }
            }
            // data interface uses CDC data interface class
            else if (iface->bNumEndpoints>=2 && iface->bInterfaceClass==10 && iface->bInterfaceSubClass==0 && iface->bInterfaceProtocol==0 && data==NULL) {
              // looks like a data interface... find IN and OUT bulk endpoints
              const uint8_t* bbb = (const uint8_t*)(iface+1);
              const usb_endpoint_descriptor *in = NULL, *out = NULL;
              for (int j=0; bbb < end && j < iface->bNumEndpoints;) {
                auto ep = get_desc_type<usb_endpoint_descriptor>(bbb, USB_DT_ENDPOINT, end);
                if (ep) {
                  ++j;
                  if ((ep->bmAttributes & 3) == USB_ENDPOINT_BULK) {
                    if (ep->bEndpointAddress & 0x80) {
                      if (in == NULL) in = ep;
                    } else {
                      if (out == NULL) out = ep;
                    }
                    if (in!=NULL && out!=NULL) {
                      data = iface;
                      if (p) {
                        p->data_in_endpoint = in->bEndpointAddress;
                        p->data_in_maxpacket = in->wMaxPacketSize;
                        p->data_out_endpoint = out->bEndpointAddress;
                        p->data_out_maxpacket = out->wMaxPacketSize;
                      }
                      break;
                    }
                  }
                }
                bbb += bbb[0];
              }
            }
            if (control!=NULL && data!=NULL)
              return true;
          }
          bb += bb[0];
        }
      }
    }
    b += b[0];
  }

  return false;
}

bool USB_RNDIS::offer(const usb_device_descriptor *d, const usb_configuration_descriptor *c) {
  if (getDevice() != NULL) return false;
  return parse_config(c);
}

USB_Driver* USB_RNDIS::attach(const usb_device_descriptor*,const usb_configuration_descriptor* c, USB_Device *dev) {
  if (getDevice() == NULL) {
    if (parse_config(c, this)) {
      setDevice(dev);

      driver_msg msg = {
        .type = DRIVER_MSG_ATTACH
      };
      atomQueuePut(&queue, 1, &msg);
      return this;
    }
  }
  return NULL;
}

void USB_RNDIS::detach(void) {
  driver_msg msg = {
    .type = DRIVER_MSG_DETACH
  };
  atomQueuePut(&queue, 1, &msg);
}

USB_RNDIS::USB_RNDIS(void) {
  atomQueueCreate(&queue, &q_msgs, sizeof(q_msgs[0]), sizeof(q_msgs)/sizeof(q_msgs[0]));
  atomThreadCreate(&thread, 96, thread_start, (uint32_t)this, stack, sizeof(stack), 0);
  atomMutexCreate(&lock);
  atomCondCreate(&cmd_signal);
}

void USB_RNDIS::get_mac_address(uint8_t mac[6]) {
  ATOM_SEM signal;
  driver_msg msg = {
    .type = DRIVER_MSG_GET_MAC_ADDR,
    .buf = mac,
    .signal = &signal
  };
  if (atomSemCreate(&signal, 0) == ATOM_OK) {
    if (atomQueuePut(&queue, 0, &msg) == ATOM_OK) {
      atomSemGet(&signal, 0);
    }
    atomSemDelete(&signal);
  }
}

size_t USB_RNDIS::recv(void* p) {
  size_t r = 0;
  ATOM_SEM signal;
  driver_msg msg = {
    .type = DRIVER_MSG_RECV,
    .buf = p,
    .len = &r,
    .signal = &signal
  };
  if (atomSemCreate(&signal, 0) == ATOM_OK) {
    if (atomQueuePut(&queue, 0, &msg) == ATOM_OK) {
      atomSemGet(&signal, 0);
    }
    atomSemDelete(&signal);
  }
  return r;
}

size_t USB_RNDIS::packet_header_length(void) const {
  return (sizeof(rndis_packet) + 3) & ~3;
}

int USB_RNDIS::send(void* p, size_t len) {
  rndis_packet* packet = (rndis_packet*)p;
  memset(packet, 0, sizeof(*packet));
  packet->MessageType = RNDIS_MSG_PACKET;
  packet->MessageLength = packet_header_length() + len;
  packet->DataOffset = packet_header_length()-8;
  packet->DataLength = len;

  uint32_t send_len = packet->MessageLength;
  if ((send_len % data_out_maxpacket)==0)
    send_len++;

  return BulkMessage(data_out_endpoint, send_len, p);
}

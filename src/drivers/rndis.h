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

#ifndef _RNDIS_DRIVER_H
#define _RNDIS_DRIVER_H

#include "../teensy4_usbhost.h"

typedef enum {
  DRIVER_MSG_ATTACH,
  DRIVER_MSG_DETACH,
  DRIVER_MSG_KEEPALIVE_REQUEST,
  DRIVER_MSG_KEEPALIVE_TIMER,
  DRIVER_MSG_GET_MAC_ADDR,
  DRIVER_MSG_RECV,
} driver_msg_type;

typedef struct {
  int type;
  union {
    uint32_t RequestID;
    struct {
      void *buf;
      size_t *len;
    };
  };
  ATOM_SEM *signal;
} driver_msg;

struct rndis_msg_indicate_status;

class USB_RNDIS : public USB_Driver, public USB_Driver::Factory {
  uint32_t resp_buf[0x1000] __attribute__((aligned(32)));
  uint8_t data_buf[0x4000] __attribute__((aligned(32)));
  uint32_t status[8] __attribute__((aligned(32)));
  uint32_t stack[0x400] __attribute__((aligned(8)));
  driver_msg q_msgs[8];
  ATOM_QUEUE queue;
  ATOM_TCB thread;
  ATOM_MUTEX lock;
  ATOM_COND cmd_signal;

  uint16_t data_in_maxpacket;
  uint16_t data_out_maxpacket;
  uint8_t control_interface;
  uint8_t status_endpoint;
  uint8_t data_in_endpoint;
  uint8_t data_out_endpoint;

  uint32_t rID = 0;
  uint32_t max_transfer_packets;
  uint32_t max_transfer_size;
  uint32_t link_speed = 0;
  uint32_t MTU;
  uint32_t MAX_FRAME_LEN;
  uint32_t maximum_frame_size;
  uint8_t mac_address[6];

  // lock should be taken before calling this function
  template<class ccommand>
  const typename ccommand::response_t* sendCommand(const ccommand& cmd, const rndis_msg_indicate_status** err=NULL);
  void verifyCommandSent(int r, uint32_t MessageLength);

  void control_in(int r);
  void status_in(int r);
  void data_in(int r);

  const USBCallback control_cb = [=](int r) { control_in(r); };
  const USBCallback status_cb = [=](int r) { status_in(r); };
  const USBCallback dataIn_cb = [=](int r) { data_in(r); };

  uint32_t rndis_set(uint32_t OID, const void* setIn, uint32_t lengthIn);
  uint32_t rndis_query(uint32_t OID, void* queryOut = NULL, uint32_t lengthOut = 0, const void* queryIn = NULL, uint32_t lengthIn = 0);
  void init_oids(void);
  void rndis_initialize(void);

  volatile bool data_receive_active = false;
  const uint8_t* data_end = NULL;
  void data_receive(void);
  size_t data_unpack(void*);

  void threadproc(void);
  static void thread_start(uint32_t p) { ((USB_RNDIS*)p)->threadproc(); }

  static bool parse_config(const usb_configuration_descriptor* c, USB_RNDIS *p=NULL);
  bool offer(const usb_device_descriptor *d, const usb_configuration_descriptor *c) override;
  USB_Driver* attach(const usb_device_descriptor*,const usb_configuration_descriptor* c, USB_Device *dev) override;
  void detach(void) override;

public:
  USB_RNDIS();
  bool isUp(void) { return link_speed > 0; }
  void get_mac_address(uint8_t mac[6]);
  size_t packet_header_length(void) const;
  int send(void*,size_t);
  size_t recv(void*);
};

#endif // _RNDIS_DRIVER_H

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

#ifndef _USB_ENDPOINT_QH_H
#define _USB_ENDPOINT_QH_H

#include "endpoint.h"
#include "portab.h"

class usb_transfer;

// QUEUE ELEMENT TRANSFER DESCRIPTOR: FIXED LAYOUT (32-BYTE ALIGNED EXCEPT IN QUEUE HEAD)
typedef struct usb_qTD_t {
  struct usb_qTD_t* next;
  struct usb_qTD_t* alt;
  union {
    struct {
      uint32_t status:8;
      uint32_t PID:2;
      uint32_t CERR:2;
      uint32_t c_Page:3;
      uint32_t IOC:1;
      uint32_t total:15;
      uint32_t dt:1;
    };
    uint32_t val;
  } token;
  uint32_t qtd_page[5];
} usb_qTD_t;

// QUEUE HEAD: FIXED LAYOUT, MUST BE 32-BYTE ALIGNED + NOT CROSS 4096-BYTE BOUNDARY (64-BYTE ALIGNMENT)
typedef struct __attribute__((aligned(64))) usb_queue_head_t {
  uint32_t horizontal_link;
  struct {
    uint32_t address:7;
    uint32_t I:1;
    uint32_t Endpt:4;
    uint32_t speed:2;
    uint32_t DTC:1;
    uint32_t H:1;
    uint32_t wMaxPacketSize:11;
    uint32_t C:1;
    uint32_t RL:4;

    uint8_t s_mask;
    uint8_t c_mask;
    uint16_t hub:7;
    uint16_t port:7;
    uint16_t Mult:2;
  } capabilities;
  usb_qTD_t* current;
  usb_qTD_t overlay;

  uint32_t pad[4]; // pad size to 64 bytes
} usb_queue_head_t;

class QH_Base : public usb_queue_head_t {
private:
  usb_transfer* pending;
  usb_transfer* dummy;
  bool active;

protected:
  void update(void);
  void flush(void);
  bool enqueue_transfer(usb_transfer* head);
  int BulkIntrTransfer(bool dir_in, uint32_t Length, void *buffer, const USBCallback* cb);
  QH_Base(uint8_t endpoint, uint16_t max_packet_size, uint8_t hub, uint8_t port, uint8_t address, uint8_t speed);
  ~QH_Base();
  uint32_t link_to() const { return ((uint32_t)&horizontal_link)+2; }
};

static inline void __attribute__((unused)) cache_flush(const QH_Base* p) {
  cache_flush(p, sizeof(usb_queue_head_t), alignof(usb_queue_head_t));
}

static inline void __attribute__((unused)) cache_invalidate(QH_Base* p) {
  cache_invalidate(p, sizeof(usb_queue_head_t), alignof(usb_queue_head_t));
}

static inline void __attribute__((unused)) cache_flush_invalidate(const QH_Base* p) {
  cache_flush_invalidate(p, sizeof(usb_queue_head_t), alignof(usb_queue_head_t));
}

class USB_Async_Endpoint : public USB_Endpoint, public QH_Base {
protected:
  USB_Async_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t hub, uint8_t port, uint8_t address, uint8_t speed, uint8_t ep_type) :
  USB_Endpoint(ep_type),
  QH_Base(endpoint, max_packet_size, hub, port, address, speed) {}
public:
  void set_link(const USB_Async_Endpoint* p);
  USB_Async_Endpoint* get_link(void) {
    USB_Async_Endpoint& endpoint = static_cast<USB_Async_Endpoint&>(*(usb_queue_head_t*)(horizontal_link & ~0x1F));
    return &endpoint;
  }
  void update(void) override { QH_Base::update(); }
  void flush(void) override { QH_Base::flush(); }
};

class USB_Control_Endpoint : public USB_Async_Endpoint {
public:
  USB_Control_Endpoint(uint8_t max_packet_size, uint8_t address, uint8_t hub, uint8_t port, uint8_t speed);
  int Transfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *buffer, CCallback<usb_control_transfer>* cb);
};



#endif // _USB_ENDPOINT_QH_H

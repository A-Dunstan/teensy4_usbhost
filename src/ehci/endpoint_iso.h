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

#ifndef _USB_ENDPOINT_ISO_H
#define _USB_ENDPOINT_ISO_H

#include "endpoint.h"
#include "config.h"
#include "log.h"
#include <bitset>

struct sitd_transfer;
struct itd_transfer;

template <class Transfer>
class USB_ISO_Endpoint : public USB_Periodic_Endpoint {
protected:
  const bool dir_in;
  const uint16_t wMaxPacketSize;
  uint8_t s_mask = 0;
  uint8_t c_mask = 0;

  Transfer* pending = NULL;
  std::bitset<PERIODIC_LIST_SIZE> frames{0};

  int Schedule(Transfer*);

  USB_ISO_Endpoint(bool _dir_in, uint16_t max_packet_size, PeriodicScheduler& scheduler) :
  USB_Periodic_Endpoint(USB_ENDPOINT_ISOCHRONOUS, scheduler),
  dir_in(_dir_in),
  wMaxPacketSize(max_packet_size) {}

public:
  void update(void) override;
  void new_offset(void) override;
  bool set_inactive(void) override;
  void get_masks(uint8_t& smask, uint8_t& cmask) const override {smask = s_mask, cmask = c_mask;}
  uint16_t get_max_packet_size(void) const override { return wMaxPacketSize; }
};

class USB_ISO_Full_Endpoint : public USB_ISO_Endpoint<sitd_transfer> {
private:
  uint32_t state;

public:
  USB_ISO_Full_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint32_t interval, PeriodicScheduler&);
  int IsochronousTransfer(isolength& Lengths, void *buffer, const USBCallback* cb) override;
};

class USB_ISO_High_Endpoint : public USB_ISO_Endpoint<itd_transfer> {
private:
  const uint32_t mult;
  const uint8_t address;
  const uint8_t endpoint;

public:
  USB_ISO_High_Endpoint(uint8_t endpoint, uint16_t wMaxPacketSize, uint8_t address, uint32_t interval, PeriodicScheduler&);
  int IsochronousTransfer(isolength& Lengths, void *buffer, const USBCallback* cb) override;
};

// SPLIT-TRANSACTION ISOCHRONOUS TRANSACTION DESCRIPTOR: FIXED LAYOUT (32-BYTE ALIGNED)
typedef struct __attribute__((aligned(32))) usb_siTD_t {
  uint32_t horizontal_link;

  typedef union {
    uint32_t val;
    struct {
      uint32_t address:7;
      uint32_t :1;
      uint32_t endpt:4;
      uint32_t :4;
      uint32_t hub:7;
      uint32_t :1;
      uint32_t port:7;
      uint32_t dir:1;
    };
  } const_sitd;
  const_sitd state;

  uint8_t s_mask;
  uint8_t c_mask;
  uint16_t :16;

  uint8_t status;
  uint8_t c_prog_mask;
  uint16_t total:10;
  uint16_t :4;
  uint16_t P:1;
  uint16_t IOC:1;

  void* page0;

  uint32_t t_count:3;
  uint32_t TP:2;
  uint32_t :7;
  uint32_t page1:20;

  uint32_t back_pointer;
  // pad up to 32 bytes
  uint32_t :32;
} usb_siTD_t;

// ISOCHRONOUS (HIGH-SPEED) TRANSFER DESCRIPTOR: FIXED LAYOUT (64-BYTE ALIGNED)
typedef struct __attribute__((aligned(64))) usb_iTD_t {
  uint32_t horizontal_link;
  struct {
    uint32_t offset:12;
    uint32_t PG:3;
    uint32_t IOC:1;
    uint32_t total:12;
    uint32_t status:4;
  } transfers[8];

  uint32_t address:7;
  uint32_t :1;
  uint32_t endpt:4;
  uint32_t page0:20;
  uint32_t wMaxPacketSize:11;
  uint32_t dir:1;
  uint32_t page1:20;
  uint32_t Mult:2;
  uint32_t :10;
  uint32_t page2:20;
  uint32_t :12;
  uint32_t page3:20;
  uint32_t :12;
  uint32_t page4:20;
  uint32_t :12;
  uint32_t page5:20;
  uint32_t :12;
  uint32_t page6:20;
} usb_iTD_t;

typedef struct sitd_transfer : usb_siTD_t {
  struct sitd_transfer *next;
  CCallback<sitd_transfer> *cb;
  uint32_t frame;
  int16_t length;
  uint32_t link_to() const { return ((uint32_t)&horizontal_link)+4; }
  void inactivate(void) { cb->callback(this, -ENODEV); }
  void complete(void) { cb->callback(this, length - total); }
  bool ready(void) {
    if (status & 0x80) return false;
    if (status & 0x7C)
      dprintf("Possible ISO transfer error: %02X\n", status);
    return true;
  }
} sitd_transfer;

typedef struct itd_transfer : usb_iTD_t {
  struct itd_transfer *next;
  CCallback<itd_transfer> *cb;
  uint32_t frame;
  uint8_t s_mask;
  uint32_t link_to() const { return (uint32_t)&horizontal_link; }
  void inactivate(void) {
    int remaining = __builtin_popcount(s_mask);
    while (remaining-->0) {
      cb->callback(this, -ENODEV);
    }
  }
  void complete(void) {
    unsigned int tmask = s_mask;
    while (tmask) {
      unsigned int pos = __builtin_ctz(tmask);
      if (transfers[pos].status & 7)
        dprintf("Possible ISO transfer error: %02X\n", transfers[pos].status);
      cb->callback(this, transfers[pos].total);
      tmask &= ~(1 << pos);
    }
  }
  bool ready(void) {
    // process complete iTD only
    for (unsigned int i=8; i > 0; i--) {
      if (transfers[i-1].status & 8)
        return false;
    }
    return true;
  }
} itd_transfer;

#endif // _USB_ENDOINT_ISO_H

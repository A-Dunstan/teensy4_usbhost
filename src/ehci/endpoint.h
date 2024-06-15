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

#ifndef _USB_ENDPOINT_H
#define _USB_ENDPOINT_H

#include "periodicscheduler.h"
#include "types.h"
#include <cstdint>
#include <cerrno>

class USB_Endpoint {
protected:
  USB_Endpoint(uint8_t _ep_type) :
  ep_type(_ep_type) {}
public:
  class USB_Endpoint* host_next = NULL;
  class USB_Device* device = NULL;
  const uint8_t ep_type;

  virtual void update(void) = 0;
  virtual void flush(void) {}

  virtual int BulkTransfer(uint32_t Length, void *buffer, const USBCallback* cb) {
    return -EOPNOTSUPP;
  }
  virtual int InterruptTransfer(uint32_t Length, void *buffer, const USBCallback* cb) {
    return -EOPNOTSUPP;
  }
  virtual int IsochronousTransfer(isolength& Lengths, void *buffer, const USBCallback* cb) {
    return -EOPNOTSUPP;
  }

  virtual ~USB_Endpoint() = default;
};

class USB_Periodic_Endpoint : public USB_Endpoint {
private:
  PeriodicScheduler &scheduler;
protected:
  USB_Periodic_Endpoint(uint8_t ep_type, PeriodicScheduler& _scheduler) :
  USB_Endpoint(ep_type),
  scheduler(_scheduler) {}

  virtual void new_offset(void) = 0;
  bool add_node(uint32_t frame, uint32_t link_to, uint32_t interval) const { return scheduler.add_node(frame, link_to, interval); }
  bool remove_node(uint32_t frame, uint32_t link_to, uint32_t next) const { return scheduler.remove_node(frame, link_to, next); }
  uint32_t current_uframe(void) const { return scheduler.current_uframe(); }

public:
  uint32_t interval = 0; // calculated interval in uframes, mapped to PERIODIC_LIST_SIZE*8
  uint32_t offset = 0;
  uint8_t stime = 0;
  uint8_t ctime = 0;

  void activate(uint32_t _offset) { offset = _offset; new_offset(); }
  virtual void get_masks(uint8_t& s_mask, uint8_t& c_mask) const = 0;
  virtual bool set_inactive(void) = 0;
};

USB_Endpoint* createIsoEndpoint(const usb_endpoint_descriptor*, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed, PeriodicScheduler&);
USB_Endpoint* createBulkEndpoint(uint8_t endpoint, uint16_t wMaxPacketSize, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed);
USB_Endpoint* createInterruptEndpoint(uint8_t endpoint, uint16_t wMaxPacketSize, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed, uint32_t interval, PeriodicScheduler&);

#endif // _USB_ENDPOINT_H

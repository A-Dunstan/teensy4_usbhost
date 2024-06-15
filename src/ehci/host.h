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

#ifndef _USB_HOST_H
#define _USB_HOST_H

#include "config.h"
#include "log.h"
#include "hub.h"
#include "endpoint_qh.h"
#include "device.h"

class USB_Host : protected USB_Hub, private CCallback<usb_control_transfer>, public PeriodicScheduler {
private:
  usb_ehci_cmd_t* const EHCI;
  uint32_t addresses[128/32];
  const uint8_t nPorts;
  uint8_t uframe_bandwidth[PERIODIC_LIST_SIZE*8];

  USB_Endpoint *endpoints;
  USB_Endpoint *async_cleanup;
  USB_Endpoint *periodic_cleanup;

  class Enum_Control : public USB_Control_Endpoint {
  public:
    USB_Hub *hub;
    uint8_t port;
    uint8_t speed;
    uint8_t bMaxPacketSize;
    uint8_t address_retry;
    bool in_progress;

    Enum_Control();

    void set(USB_Hub& h, uint8_t p, uint8_t s);
    void clean(void);
  } Enum;

private:
  void callback(const usb_control_transfer* t, int result);

  uint8_t get_address(void);
  void release_address(uint8_t i);

  bool set_port_timeout(USB_Hub& hub, uint8_t port, uint32_t ticks);
  void port_status(USB_Hub& hub, uint8_t port, port_status_t status);
  void port_timeout(usb_msg_t& msg);

  void notify_endpoint_removed(USB_Endpoint *ep);
  void update_transfers(void);

  void add_async_queue(USB_Async_Endpoint *ep);
  bool remove_async_queue(USB_Async_Endpoint *ep);

  void add_periodic_queue(USB_Periodic_Endpoint *ep);
  bool remove_periodic_queue(USB_Periodic_Endpoint *ep);
  void unschedule_periodic(void);
  bool calculate_offset(const USB_Periodic_Endpoint *ep, uint32_t &offset);

  void port_power(uint8_t port, bool set) override;
  void port_reset(uint8_t port, bool set) override;
  void port_enable(uint8_t port, bool set) override;
  uint8_t HS_port(uint8_t port) const override {return port;}

protected:
  USB_Host(usb_ehci_base_t* const base);
  void usb_process(void);
  static const USB_Host& deviceToHost(const USB_Device* dev) { return dev->host; }

  // functions to be provided by derived class
public:
  virtual bool putMessage(usb_msg_t&) = 0;
  virtual bool timerMsg(usb_msg_t &msg, uint32_t ms) = 0;
private:
  virtual bool getMessage(usb_msg_t&) = 0;
  virtual void nextIRQ(void) = 0;
  virtual void setHostMode(void) = 0;
  virtual uint32_t getMillis(void) = 0;

public:
  bool activate_endpoint(USB_Endpoint *ep, USB_Device *device);
  bool deactivate_endpoint(USB_Endpoint *ep);
};

#endif // _USB_HOST_H

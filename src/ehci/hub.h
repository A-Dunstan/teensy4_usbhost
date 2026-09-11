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

#ifndef _USB_HUB_H
#define _USB_HUB_H

#include <cstdint>

#define USB_PORT_FEATURE_CONNECTION     0
#define USB_PORT_FEATURE_ENABLE         1
#define USB_PORT_FEATURE_SUSPEND        2
#define USB_PORT_FEATURE_OVER_CURRENT   3
#define USB_PORT_FEATURE_RESET          4
#define USB_PORT_FEATURE_POWER          8
#define USB_PORT_FEATURE_LOW_SPEED      9
#define USB_PORT_FEATURE_C_CONNECTION   16
#define USB_PORT_FEATURE_C_ENABLE       17
#define USB_PORT_FEATURE_C_PORT_SUSPEND 18
#define USB_PORT_FEATURE_C_PORT_OVER_CURRENT 19
#define USB_PORT_FEATURE_C_RESET        20

class USB_Hub {
  friend class USB_Host;
private:
  struct USB_Port {
    uint32_t state;
    class USB_Device *device;
    uint32_t timeout_start;
  } port[8] = {0};

  virtual void port_power(uint8_t port, bool set) = 0;
  virtual void port_reset(uint8_t port, bool set) = 0;
  virtual void port_enable(uint8_t port, bool set) = 0;
  virtual uint8_t HS_port(uint8_t port) const = 0;
  virtual void phySetHighSpeed(uint8_t port, bool on) {}

  virtual void addref(void) {}
  virtual void deref(void) {}

protected:
  uint8_t const hub_addr;
  USB_Hub(uint8_t addr) : hub_addr(addr) {}
};

#endif // _USB_HUB_H

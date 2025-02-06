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

class USB_Hub {
  friend class USB_Host;
private:
  struct USB_Port {
    uint32_t state;
    class USB_Device *device;
    uint32_t timeout_start;
  } port[8];

  virtual void port_power(uint8_t port, bool set) = 0;
  virtual void port_reset(uint8_t port, bool set) = 0;
  virtual void port_enable(uint8_t port, bool set) = 0;
  virtual uint8_t HS_port(uint8_t port) const = 0;
  virtual void phySetHighSpeed(uint8_t port, bool on) {}

  virtual void addref(void) {}
  virtual void deref(void) {}

protected:
  uint8_t const hub_addr;
  USB_Hub(uint8_t addr);
};

#endif // _USB_HUB_H

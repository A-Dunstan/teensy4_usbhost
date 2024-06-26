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

#ifndef _USB_MOUSE_H
#define _USB_MOUSE_H

#include "../teensy4_usbhost.h"

/* mouse driver sends this to event queue.
 * The boot protocol is very basic, it only defines the
 * first three bytes but some mice will return the scroll wheel
 * as 1 or -1 in the fourth byte, or maybe even more data;
 * this is returned in extra[] if it's present
 */
typedef struct {
  uint8_t buttons;
  int8_t xdelta;
  int8_t ydelta;
  int8_t zdelta;
  uint8_t extra[4];
  uint8_t len;
} mouse_event;

class USBMouse : public USB_Driver, public USB_Driver::Factory {
private:
  uint8_t report[64] __attribute__((aligned(32)));
  uint16_t report_len;
  uint8_t ep_in;
  uint8_t interface;
  volatile bool attached = false;
  ATOM_QUEUE *queue = NULL;

  const USBCallback poll_cb = [=](int r) { poll(r); };
  void poll(int result);
  void startPolling();
  const usb_endpoint_descriptor* find_endpoint(const usb_interface_descriptor*,size_t);

  // Factory overrides
  bool offer(const usb_interface_descriptor*, size_t) override;
  USB_Driver* attach(const usb_interface_descriptor*,size_t,USB_Device*) override;
  void detach(void) override;

public:
  USBMouse() = default;

  operator bool() const { return attached==true; }
  bool begin(ATOM_QUEUE *q);
};

#endif // _USB_MOUSE_H

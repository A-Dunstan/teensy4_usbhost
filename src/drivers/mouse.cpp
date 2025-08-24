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

#include "mouse.h"
#include <cstring>

bool USBMouse::begin(ATOM_QUEUE *q) {
  if (q == NULL) return false;
  /* this odd pattern is to avoid a race condition
   * that could possibly call startPolling twice
   */
  if (attached && queue == NULL) {
    queue = q;
    startPolling();
  } else {
    queue = q;
  }
  return attached;
}

const usb_endpoint_descriptor* USBMouse::find_endpoint(const usb_interface_descriptor* i, size_t len) {
  // find IN interrupt endpoint
  const uint8_t *b = (const uint8_t*)i;
  const uint8_t *end = b + len - 2;
  for(b += b[0];b < end; b += b[0]) {
    // if we hit an alternate interface, abort
    if (b[1] == USB_DT_INTERFACE) break;
    if (b[1] != USB_DT_ENDPOINT) continue;

    const usb_endpoint_descriptor *ep = (const usb_endpoint_descriptor*)b;
    if (ep->bmAttributes != USB_ENDPOINT_INTERRUPT)
      continue;
    if ((ep->bEndpointAddress & 0x80) == 0)
      continue;
    if (ep->wMaxPacketSize > 64)
      continue;
    return ep;
  }

  return NULL;
}

bool USBMouse::offer(const usb_interface_descriptor* i, size_t len) {
  if (attached) return false;
  // want class==USB_CLASS_HID
  if (i->bInterfaceClass != 3) return false;
  // want subclass==HID_SUBCLASS_BOOT
  if (i->bInterfaceSubClass != 1) return false;
  // want protocol==HID_PROTOCOL_MOUSE
  if (i->bInterfaceProtocol != 2) return false;
  // test for endpoint
  if (find_endpoint(i, len) == NULL) return false;
  return true;
}

USB_Driver* USBMouse::attach(const usb_interface_descriptor* i, size_t len, USB_Device* dev) {
  if (!attached) {
    const usb_endpoint_descriptor* ep = find_endpoint(i, len);
    if (ep != NULL) {
      report_len = ep->wMaxPacketSize;
      ep_in = ep->bEndpointAddress;
      interface = i->bInterfaceNumber;
      setDevice(dev);
      if (queue != NULL) {
        attached = true;
        startPolling();
      } else {
        attached = true;
      }
      dprintf("Mouse was attached\n");
      return this;
    }
  }
  return NULL;
}

void USBMouse::detach(void) {
  dprintf("Mouse detached\n");
  attached = false;
}

void USBMouse::poll(int result) {
  if (result >= 3) {
    if (queue) {
      mouse_event event;
      if (result > 8) result = 8;
      memcpy(&event, report, result);
      event.len = (uint8_t)result;
      atomQueuePut(queue, -1, &event);
    }
    if (attached) InterruptMessage(ep_in, report_len, report, &poll_cb);
  }
}

void USBMouse::startPolling() {
  // set the boot protocol (instead of HID reports)
  ControlMessage(USB_REQTYPE_INTERFACE_SET|USB_CTRLTYPE_TYPE_CLASS, USB_REQ_SETPROTOCOL, 0, interface, 0, NULL, [=](int r) {
    if (r != -ENODEV) {
      // boot protocol was (hopefully) set, starting polling
      InterruptMessage(ep_in, report_len, report, &poll_cb);
    }
  });
}

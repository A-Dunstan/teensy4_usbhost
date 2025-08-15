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

#include "xbox360pad.h"

#define FLAG_INPROGRESS       (1<<31)
#define FLAG_SETLED           (1<<0)
#define FLAG_SETRUMBLE        (1<<1)

void XBOX360Pad::interrupt_in(int r) {
  if (r >= 2) {
    ready = true;
    switch (rep_in[0]) {
      case 0: // report 0: this is the button/stick/trigger data
        if (r >= 20) {
          state.buttons = rep_in[2] | (rep_in[3]<<8);
          state.trigger[0] = rep_in[4];
          state.trigger[1] = rep_in[5];
          state.stick[0][0] = rep_in[6] | (rep_in[7]<<8);
          state.stick[0][1] = rep_in[8] | (rep_in[9]<<8);
          state.stick[1][0] = rep_in[10] | (rep_in[11]<<8);
          state.stick[1][1] = rep_in[12] | (rep_in[13]<<8);
        }
        break;
      case 1: // response to output report 1, returns LED state
        if (r >= 3) {
          dprintf("XBOX LED state: %02X\n", rep_in[2]);
        }
        break;
      // these have all been observed to have a length of 3
      // Don't know what any of them mean, and third-party controllers may not send them...
//      case 2:
//      case 3:
//      case 8:
    }
  }
  if (r != -ENODEV)
    InterruptMessage(ep_in, sizeof(rep_in), rep_in, &in_cb);
}

void XBOX360Pad::interrupt_out(int r) {
  if (r >= 0) {
    atomMutexGet(&lock, 10);
    flags &= ~FLAG_INPROGRESS;

    if (flags & FLAG_SETLED) {
      flags &= ~FLAG_SETLED;
      setLED(led);
    } else if (flags & FLAG_SETRUMBLE) {
      flags &= ~FLAG_SETRUMBLE;
      setRumble(motor_heavy, motor_light);
    }
    atomMutexPut(&lock);
  }
}

FLASHMEM void XBOX360Pad::setLED(uint8_t new_led) {
  atomMutexGet(&lock, 10);

  if (flags & FLAG_INPROGRESS) {
    flags |= FLAG_SETLED;
    led = new_led;
  } else {
    flags |= FLAG_INPROGRESS;
    rep_out[0] = 1;
    rep_out[1] = 3;
    rep_out[2] = led;
    InterruptMessage(ep_out, 3, rep_out, &out_cb);
  }
  atomMutexPut(&lock);
}

void XBOX360Pad::setRumble(uint8_t heavy, uint8_t light) {
  atomMutexGet(&lock, 10);

  if (flags & FLAG_INPROGRESS) {
    flags |= FLAG_SETRUMBLE;
    motor_heavy = heavy;
    motor_light = light;
  } else {
    flags |= FLAG_INPROGRESS;
    memset(rep_out, 0, 8);
    rep_out[0] = 0;
    rep_out[1] = 8;
    rep_out[3] = heavy;
    rep_out[4] = light;
    InterruptMessage(ep_out, 8, rep_out, &out_cb);
  }
  atomMutexPut(&lock);
}

FLASHMEM const usb_endpoint_descriptor* XBOX360Pad::find_endpoint(const void* p, size_t& length) {
  if (p == NULL) return NULL;
  // find IN/OUT interrupt endpoints that are exactly 32 bytes long
  const uint8_t *b = (const uint8_t*)p;
  const uint8_t *end = b + length - 2;
  for (b += b[0]; b < end && b[0]; b += b[0]) {
    // if we hit an alternate interface, abort
    if (b[1] == USB_DT_INTERFACE) break;
    if (b[1] != USB_DT_ENDPOINT) continue;

    const usb_endpoint_descriptor *ep = (const usb_endpoint_descriptor*)b;
    if (ep->bmAttributes != USB_ENDPOINT_INTERRUPT)
      continue;
    if (ep->wMaxPacketSize != REP_SIZE)
      continue;

    length -= b - (const uint8_t*)p;
    return ep;
  }
  return NULL;
}

FLASHMEM bool XBOX360Pad::offer(const usb_interface_descriptor* id, size_t length) {
  if (getDevice() != NULL) return false;
  if (id->bInterfaceClass != 255) return false;
  if (id->bInterfaceSubClass != 93) return false;
  if (id->bInterfaceProtocol != 1) return false;
  if (id->bNumEndpoints < 1) return false;
  return true;
}

FLASHMEM USB_Driver* XBOX360Pad::attach(const usb_interface_descriptor* id, size_t length, USB_Device *dev) {
  if (getDevice() == NULL) {
    const usb_endpoint_descriptor *ep1 = find_endpoint(id, length);
    const usb_endpoint_descriptor *ep2 = find_endpoint(ep1, length);
    if (ep1 != NULL) {
      if (ep1->bEndpointAddress & 0x80) {
        ep_in = ep1->bEndpointAddress;
        if (ep2 && (ep2->bEndpointAddress & 0x80)==0)
          ep_out = ep2->bEndpointAddress;
        else
          ep_out = 255;
      } else {
        ep_out = ep1->bEndpointAddress;
        if (ep2 && (ep2->bEndpointAddress & 0x80))
          ep_in = ep2->bEndpointAddress;
        else
          ep_in = 255;
      }
    }
    if (ep_in != 255 && ep_out != 255) {
      dprintf("Found XBOX360 compatible controller, ep_in = %02X, ep_out = %02X\n", ep_in, ep_out);
      setDevice(dev);
      flags = 0;
      setLED(6);
      InterruptMessage(ep_in, sizeof(rep_in), rep_in, &in_cb);
      return this;
    }
  }
  return NULL;
}

FLASHMEM void XBOX360Pad::detach(void) {
  dprintf("XBOX360 detached");
  memset(&state, 0, sizeof(state));
  cur_buttons = 0;
  ready = false;
}

FLASHMEM XBOX360Pad::XBOX360Pad() {
  atomMutexCreate(&lock);
}

FLASHMEM XBOX360Pad::~XBOX360Pad() {
  atomMutexDelete(&lock);
}

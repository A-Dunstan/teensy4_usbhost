/*
  Copyright (C) 2026 Andrew Dunstan
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

#include "../teensy4_usbhost.h"

void PCS_PSPadBase::report_in(int r) {
  if (r >= 8 && rep_in[0]==1) {
    ready = true;
    uint8_t dpad[4] = {}; // up, down, left, right


    auto convert_stick = [=](uint8_t d) -> int16_t {
      int16_t stick = d * 0x0101;
      return stick ^ 0x8000;
    };

    update_padstick(Gamepad::Stick::LEFT_X,  convert_stick(rep_in[3]));
    update_padstick(Gamepad::Stick::LEFT_Y,  -1 - convert_stick(rep_in[4]));
    update_padstick(Gamepad::Stick::RIGHT_X, convert_stick(rep_in[2]));
    update_padstick(Gamepad::Stick::RIGHT_Y, -1 - convert_stick(rep_in[1]));
    switch (rep_in[5] & 0xF) {
      case 0: // up
        dpad[0] = 255;
        break;
      case 1: // up+right
        dpad[0] = 255;
        dpad[3] = 255;
        break;
      case 2: // right
        dpad[3] = 255;
        break;
      case 3: // right+down
        dpad[3] = 255;
        dpad[1] = 255;
        break;
      case 4: // down
        dpad[1] = 255;
        break;
      case 5: // left+down
        dpad[2] = 255;
        dpad[1] = 255;
        break;
      case 6: // left
        dpad[2] = 255;
        break;
      case 7: // left+up
        dpad[0] = 255;
        dpad[2] = 255;
        break;
    }
    update_padbutton(Gamepad::Button::DPAD_UP,       dpad[0]);
    update_padbutton(Gamepad::Button::DPAD_DOWN,     dpad[1]);
    update_padbutton(Gamepad::Button::DPAD_LEFT,     dpad[2]);
    update_padbutton(Gamepad::Button::DPAD_RIGHT,    dpad[3]);
    update_padbutton(Gamepad::Button::FACE_TOP,      rep_in[5] & 0x10 ? 255 : 0);
    update_padbutton(Gamepad::Button::FACE_RIGHT,    rep_in[5] & 0x20 ? 255 : 0);
    update_padbutton(Gamepad::Button::FACE_BOTTOM,   rep_in[5] & 0x40 ? 255 : 0);
    update_padbutton(Gamepad::Button::FACE_LEFT,     rep_in[5] & 0x80 ? 255 : 0);
    update_padbutton(Gamepad::Button::LEFT_TRIGGER,  rep_in[6] & 0x01 ? 255 : 0);
    update_padbutton(Gamepad::Button::RIGHT_TRIGGER, rep_in[6] & 0x02 ? 255 : 0);
    update_padbutton(Gamepad::Button::LEFT_BUMPER,   rep_in[6] & 0x04 ? 255 : 0);
    update_padbutton(Gamepad::Button::RIGHT_BUMPER,  rep_in[6] & 0x08 ? 255 : 0);
    update_padbutton(Gamepad::Button::SELECT,        rep_in[6] & 0x10 ? 255 : 0);
    update_padbutton(Gamepad::Button::START,         rep_in[6] & 0x20 ? 255 : 0);
    update_padbutton(Gamepad::Button::LEFT_STICK,    rep_in[6] & 0x40 ? 255 : 0);
    update_padbutton(Gamepad::Button::RIGHT_STICK,   rep_in[6] & 0x80 ? 255 : 0);
  }

  if (r >= 0 || errno != ENODEV) {
    InterruptMessage(ep_in, sizeof(rep_in), rep_in, &in_cb);
  }
}

FLASHMEM bool PCS_PSPadBase::driver_match(const usb_device_descriptor* dd, const usb_configuration_descriptor* cd) {
  if (dd->idVendor != 0x0810) return false;
  if (dd->idProduct != 0x0003) return false;
  if (cd->bNumInterfaces != 1) return false;

  return true;
}

FLASHMEM USB_Driver* PCS_PSPad::offer(const usb_device_descriptor* dd, const usb_configuration_descriptor* cd, const USB_Device*) {
  if (getDevice() != NULL) return NULL;
  if (!driver_match(dd, cd)) return NULL;
  return this;
}

FLASHMEM bool PCS_PSPadBase::attach(const usb_device_descriptor* dd, const usb_configuration_descriptor* cd) {
  const usb_descriptor* d = cd;
  auto end = &cd->bLength + cd->wTotalLength;
  while (d->bDescriptorType != usb_interface_descriptor::DescriptorType) {
    d = d->next();
    if (&d->bLength >= end) return false;
  }
  auto id = static_cast<const usb_interface_descriptor*>(d);

  ep_in = 0;
  for (uint8_t i=0; i < id->bNumEndpoints; i++) {
    auto ep = get_interface_endpoint(id, i);
    if (ep->bmAttributes != USB_ENDPOINT_INTERRUPT) continue;
    if (ep->wMaxPacketSize > sizeof(rep_in)) continue;
    if (ep->bEndpointAddress & 0x80) {
      ep_in = ep->bEndpointAddress;
      reset_padstate();
      report_in(0);
      return true;
    }
  }

  return false;
}

FLASHMEM void PCS_PSPadBase::detach(void) {
//  dprintf("PCS_PSPad detached");
  reset_padstate();
  ready = false;
}

const char* PCS_PSPadBase::getDeviceType() const {
  return PSTR("Playstation1/2 Controller via PCS USB adapter");
}

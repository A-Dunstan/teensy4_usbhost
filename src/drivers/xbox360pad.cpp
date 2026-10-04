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

#include "../teensy4_usbhost.h"

#define FLAG_INPROGRESS       (1<<31)
#define FLAG_SETLED           (1<<0)
#define FLAG_SETRUMBLE        (1<<1)

void XBOX360PadBase::report_in(int r) {
  const uint8_t* in = rep_in;

  if (wireless) {
    int len = r;
    if (len >= 2) {
      r = 0;
      if (in[0] & 0x08) { // notification that a controller connected/disconnected
        ready = in[1] & 0x80;
      }
      else if (in[1]==1 && len > 4) {
        in = rep_in + 4;
        r = len-4;
      }
    }
  }

  if (r >= 2) {
    switch (in[0]) {
      case 0: // report 0: this is the button/stick/trigger data
        if (r >= 20) {
          ready = true;
          update_padbutton(Gamepad::Button::DPAD_UP,      in[2]&0x01 ? 255 : 0);
          update_padbutton(Gamepad::Button::DPAD_DOWN,    in[2]&0x02 ? 255 : 0);
          update_padbutton(Gamepad::Button::DPAD_LEFT,    in[2]&0x04 ? 255 : 0);
          update_padbutton(Gamepad::Button::DPAD_RIGHT,   in[2]&0x08 ? 255 : 0);
          update_padbutton(Gamepad::Button::START,        in[2]&0x10 ? 255 : 0);
          update_padbutton(Gamepad::Button::SELECT,       in[2]&0x20 ? 255 : 0);
          update_padbutton(Gamepad::Button::LEFT_STICK,   in[2]&0x40 ? 255 : 0);
          update_padbutton(Gamepad::Button::RIGHT_STICK,  in[2]&0x80 ? 255 : 0);
          update_padbutton(Gamepad::Button::LEFT_BUMPER,  in[3]&0x01 ? 255 : 0);
          update_padbutton(Gamepad::Button::RIGHT_BUMPER, in[3]&0x02 ? 255 : 0);
          update_padbutton(Gamepad::Button::SYSTEM,       in[3]&0x04 ? 255 : 0);
          update_padbutton(Gamepad::Button::FACE_BOTTOM,  in[3]&0x10 ? 255 : 0);
          update_padbutton(Gamepad::Button::FACE_RIGHT,   in[3]&0x20 ? 255 : 0);
          update_padbutton(Gamepad::Button::FACE_LEFT,    in[3]&0x40 ? 255 : 0);
          update_padbutton(Gamepad::Button::FACE_TOP,     in[3]&0x80 ? 255 : 0);
          update_padbutton(Gamepad::Button::LEFT_TRIGGER, in[4]);
          update_padbutton(Gamepad::Button::RIGHT_TRIGGER, in[5]);
          update_padstick(Gamepad::Stick::LEFT_X, in[6]|(in[7]<<8));
          update_padstick(Gamepad::Stick::LEFT_Y, in[8]|(in[9]<<8));
          update_padstick(Gamepad::Stick::RIGHT_X, in[10]|(in[11]<<8));
          update_padstick(Gamepad::Stick::RIGHT_Y, in[12]|(in[13]<<8));
        }
        break;
      case 1: // response to output report 1, returns LED state
        if (r >= 3) {
          if (in[2] != led) {
            // set it again because it didn't listen
            setLED(led);
          }
        }
        break;
      // these have all been observed to have a length of 3
      // Don't know what any of them mean, and third-party controllers may not send them...
//      case 2:
//      case 3:
//      case 8:
    }
  }

  if (r != -ENODEV && r != -ENXIO)
    InterruptMessage(ep_in, sizeof(rep_in), rep_in, &in_cb);
}

void XBOX360PadBase::report_out(int r) {
  if (r >= 0) {
    auto lock = mutex.Lock(10);
    flags &= ~FLAG_INPROGRESS;

    if (flags & FLAG_SETLED) {
      flags &= ~FLAG_SETLED;
      setLED(led);
    } else if (flags & FLAG_SETRUMBLE) {
      flags &= ~FLAG_SETRUMBLE;
      setRumble(motor_heavy, motor_light);
    }
  }
}

FLASHMEM void XBOX360PadBase::setPlayerLED(uint8_t new_led) {
  setLED(6+(new_led&3));
}

FLASHMEM void XBOX360PadBase::setLED(uint32_t new_led) {
  auto lock = mutex.Lock(10);

  led = (uint8_t)new_led;
  if (flags & FLAG_INPROGRESS) {
    flags |= FLAG_SETLED;
  } else {
    flags |= FLAG_INPROGRESS;
    if (!wireless) {
      rep_out[0] = 1;
      rep_out[1] = 3;
      rep_out[2] = led;
    } else {
      memset(rep_out, 0, 12);
      rep_out[2] = 8;
      rep_out[3] = 0x40 + led;
    }
    InterruptMessage(ep_out, wireless ? 12 : 3, rep_out, &out_cb);
  }
}

void XBOX360PadBase::setRumble(uint8_t heavy, uint8_t light) {
  auto lock = mutex.Lock(10);

  if (flags & FLAG_INPROGRESS) {
    flags |= FLAG_SETRUMBLE;
    motor_heavy = heavy;
    motor_light = light;
  } else {
    flags |= FLAG_INPROGRESS;
    memset(rep_out, 0, 12);
    if (!wireless) {
      rep_out[1] = 8;
      rep_out[3] = heavy;
      rep_out[4] = light;
    } else {
      rep_out[1] = 1;
      rep_out[2] = 0xF;
      rep_out[3] = 0xC0;
      rep_out[5] = heavy;
      rep_out[6] = light;
    }
    InterruptMessage(ep_out, wireless ? 12: 8, rep_out, &out_cb);
  }
}

FLASHMEM bool XBOX360PadBase::driver_match(const usb_interface_descriptor* id, size_t length) {
  if (id->bInterfaceClass != 255) return false;
  if (id->bInterfaceSubClass != 93) return false;
  if (id->bInterfaceProtocol != 1 && \
      id->bInterfaceProtocol != 0x81) return false;
  if (id->bNumEndpoints < 2) return false;
  return true;
}

FLASHMEM USB_Driver* XBOX360Pad::offer(const usb_interface_descriptor* id, size_t length, const USB_Device*) {
  if (getDevice() != NULL) return NULL;
  if (!driver_match(id, length)) return NULL;
  return this;
}

FLASHMEM bool XBOX360PadBase::attach(const usb_interface_descriptor* id, size_t length) {
  ep_in = ep_out = 0;
  for (uint8_t i=0; i < id->bNumEndpoints; i++) {
    auto ep = get_interface_endpoint(id, i);
    if (ep->bmAttributes != USB_ENDPOINT_INTERRUPT)
      continue;
    if (ep->wMaxPacketSize != REP_SIZE)
      continue;

    if (ep->bEndpointAddress & 0x80) {
      if (ep_in == 0) ep_in = ep->bEndpointAddress;
    } else if (ep_out == 0)
      ep_out = ep->bEndpointAddress;
    }

    if (ep_in && ep_out) {
      wireless = (id->bInterfaceProtocol == 0x81);
//      dprintf("XBOX360 attached (%s)\n", wireless ? "wireless" : "wired");
      flags = 0;
      reset_padstate();
      setLED(0);
      return InterruptMessage(ep_in, sizeof(rep_in), rep_in, &in_cb) >= 0;
  }

  return false;
}

FLASHMEM void XBOX360PadBase::detach(void) {
//  dprintf("XBOX360 detached");
  reset_padstate();
  ready = false;
}

const char* XBOX360PadBase::getXBOXButtonName(uint8_t btn) {
  switch (btn) {
    case Gamepad::Button::SELECT:
      return PSTR("BACK");
    case Gamepad::Button::LEFT_TRIGGER:
      return PSTR("LT");
    case Gamepad::Button::RIGHT_TRIGGER:
      return PSTR("RT");
    case Gamepad::Button::SYSTEM:
      return PSTR("XBOX_BUTTON");
    case Gamepad::Button::FACE_TOP:
      return PSTR("Y");
    case Gamepad::Button::FACE_RIGHT:
      return PSTR("B");
    case Gamepad::Button::FACE_BOTTOM:
      return PSTR("A");
    case Gamepad::Button::FACE_LEFT:
      return PSTR("X");
  }
  return NULL;
}

const char* XBOX360PadBase::getDeviceType() const {
  if (wireless)
    return PSTR("XBOX360 Wireless Controller");
  return PSTR("XBOX360 Wired Controller");
}

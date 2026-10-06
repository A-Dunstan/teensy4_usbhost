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

#define USB_REQTYPE_HID_SET   (USB_CTRLTYPE_DIR_HOST2DEVICE|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_INTERFACE)
#define USB_REQTYPE_HID_GET   (USB_CTRLTYPE_DIR_DEVICE2HOST|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_INTERFACE)

#define FLAG_INPROGRESS       (1<<31)
#define FLAG_SETLED           (1<<0)
#define FLAG_SETRUMBLE        (1<<1)

static const uint8_t poll_start[4] PROGMEM = {0x42, 0x0C};

void PS3PadBase::report_in(int r) {
  if (r < 1) return;

  auto convert_stick = [](uint8_t s) -> int16_t {
    int x = (s << 8) | s;
    x -= 32768;
    if (x <= -32768) return -32768;
    if (x >= 32767) return 32767;
    return (int16_t)x;
  };

  switch (rep_in[0]) {
    case 0x01:
      if (r < 49) return;

      update_padbutton(Gamepad::Button::SELECT,        rep_in[2] & 0x01 ? 255 : 0);
      update_padbutton(Gamepad::Button::LEFT_STICK,    rep_in[2] & 0x02 ? 255 : 0);
      update_padbutton(Gamepad::Button::RIGHT_STICK,   rep_in[2] & 0x04 ? 255 : 0);
      update_padbutton(Gamepad::Button::START,         rep_in[2] & 0x08 ? 255 : 0);
      update_padbutton(Gamepad::Button::SYSTEM,        rep_in[4] & 0x01 ? 255 : 0);
      update_padbutton(Gamepad::Button::DPAD_UP,       rep_in[14]);
      update_padbutton(Gamepad::Button::DPAD_RIGHT,    rep_in[15]);
      update_padbutton(Gamepad::Button::DPAD_DOWN,     rep_in[16]);
      update_padbutton(Gamepad::Button::DPAD_LEFT,     rep_in[17]);
      update_padbutton(Gamepad::Button::LEFT_TRIGGER,  rep_in[18]);
      update_padbutton(Gamepad::Button::RIGHT_TRIGGER, rep_in[19]);
      update_padbutton(Gamepad::Button::LEFT_BUMPER,   rep_in[20]);
      update_padbutton(Gamepad::Button::RIGHT_BUMPER,  rep_in[21]);
      update_padbutton(Gamepad::Button::FACE_TOP,      rep_in[22]);
      update_padbutton(Gamepad::Button::FACE_RIGHT,    rep_in[23]);
      update_padbutton(Gamepad::Button::FACE_BOTTOM,   rep_in[24]);
      update_padbutton(Gamepad::Button::FACE_LEFT,     rep_in[25]);
      update_padstick(Gamepad::Stick::LEFT_X, convert_stick(rep_in[6]));
      update_padstick(Gamepad::Stick::LEFT_Y, -1 - convert_stick(rep_in[7]));
      update_padstick(Gamepad::Stick::RIGHT_X, convert_stick(rep_in[8]));
      update_padstick(Gamepad::Stick::RIGHT_Y, -1 - convert_stick(rep_in[9]));

      update_padstick(ACC_X, (rep_in[41]<<8)|rep_in[42]);
      update_padstick(ACC_Y, (rep_in[43]<<8)|rep_in[44]);
      update_padstick(ACC_Z, (rep_in[45]<<8)|rep_in[46]);
      update_padstick(GYRO,  (rep_in[47]<<8)|rep_in[48]);

      break;
    case 0xF2:
      if (r < 17) return;
      memcpy(mac_addr, rep_in+4, 6);
      dprintf("DualShock3 BT MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
      break;
  }
}

void PS3PadBase::interrupt_in(int r) {
  if (r > 0) report_in(r);
  if (r != -ENODEV)
    InterruptMessage(ep_in, sizeof(rep_in), rep_in, &in_cb);
}

void PS3PadBase::report_out(int r) {
  if (r >= 0) {
    auto lock = mutex.Lock(10);
    flags &= ~FLAG_INPROGRESS;

    if (flags & (FLAG_SETLED|FLAG_SETRUMBLE)) {
      flags |= FLAG_INPROGRESS;
      static const uint8_t outreport_01[49] = {
        0x01,
        0x00,
        0x00, 0x00, 0x00, 0x00, // light_duration, light_on (0/1), heavy_duration, heavy_force
        0x00, 0x00, 0x00, 0x00,
        0x00, // leds: 0x02, 0x04, 0x08, 0x10
        0xFF, 0x27, 0x10, 0x00, 0x32,
        0xFF, 0x27, 0x10, 0x00, 0x32,
        0xFF, 0x27, 0x10, 0x00, 0x32,
        0xFF, 0x27, 0x10, 0x00, 0x32
      };

      memcpy(rep_out, outreport_01, sizeof(outreport_01));
      rep_out[10] = 2 << (led&3);
      if (motor_heavy) {
        rep_out[4] = 0xFF;
        rep_out[5] = motor_heavy;
      }
      if (motor_light) {
        rep_out[2] = 0xFF;
        rep_out[3] = 1;
      }
      flags &= ~(FLAG_SETLED|FLAG_SETRUMBLE);
      InterruptMessage(ep_out, sizeof(outreport_01), rep_out, &out_cb);
    }
  }
}

FLASHMEM void PS3PadBase::setPlayerLED(uint8_t new_led) {
  auto lock = mutex.Lock(10);

  if (led == new_led) return;

  led = new_led;
  flags |= FLAG_SETLED;
  if ((flags & FLAG_INPROGRESS) == 0)
    report_out(0);
}

void PS3PadBase::setRumble(uint8_t heavy, uint8_t light) {
  auto lock = mutex.Lock(10);

  if (heavy==motor_heavy && light==motor_light) return;

  motor_heavy = heavy;
  motor_light = light;
  flags |= FLAG_SETRUMBLE;
  if ((flags & FLAG_INPROGRESS) == 0)
    report_out(0);
}

FLASHMEM bool PS3PadBase::driver_match(const usb_device_descriptor* dd, const usb_configuration_descriptor* cd) {
  if (dd->idVendor != 0x054C) return false;
  if (dd->idProduct != 0x0268) return false;
  if (cd->bNumInterfaces != 1) return false;
  return true;
}

FLASHMEM USB_Driver* PS3Pad::offer(const usb_device_descriptor* dd, const usb_configuration_descriptor* cd, const USB_Device*) {
  if (getDevice() != NULL) return NULL;
  if (!driver_match(dd, cd)) return NULL;
  return this;
}

FLASHMEM bool PS3PadBase::attach(const usb_device_descriptor* dd, const usb_configuration_descriptor* cd) {
  const usb_descriptor* d = cd;
  auto end = &cd->bLength + cd->wTotalLength;
  while (d->bDescriptorType != usb_interface_descriptor::DescriptorType) {
    d = d->next();
    if (&d->bLength >= end) return false;
  }
  auto id = static_cast<const usb_interface_descriptor*>(d);

  ep_in = ep_out = 0;
  for (uint8_t i=0; i < id->bNumEndpoints; i++) {
    auto ep = get_interface_endpoint(id, i);
    if (ep->bmAttributes != USB_ENDPOINT_INTERRUPT) continue;
    if (ep->wMaxPacketSize > 64) continue;
    if (ep->bEndpointAddress & 0x80) {
      if (ep_in == 0) ep_in = ep->bEndpointAddress;
    } else if (ep_out == 0)
      ep_out = ep->bEndpointAddress;

    if (ep_in && ep_out) {
      flags = 0;
      reset_padstate();
      iface = id->bInterfaceNumber;
      ControlMessage(USB_REQTYPE_HID_GET, USB_REQ_GETREPORT, (USB_REPTYPE_FEATURE<<8)|0xF2, iface, 17, rep_in, [=](int r) {report_in(r); });
      // start polling
      ControlMessage(USB_REQTYPE_HID_SET, USB_REQ_SETREPORT, (USB_REPTYPE_FEATURE<<8)|0xF4, iface, sizeof(poll_start), poll_start, [=](int r) {
        if (r >= 4) {
          ready = true;
          led = 0xFF;
          setPlayerLED(0);
          interrupt_in(0);
        }
      });
      return true;
    }
  }

  return false;
}

FLASHMEM void PS3PadBase::detach(void) {
//  dprintf("Dualshock3 detached");
  reset_padstate();
  ready = false;
}

FLASHMEM bool PS3PadBase::getBluetoothMAC(uint8_t* dst) const {
  if (ready) {
    memcpy(dst, mac_addr, 6);
    return true;
  }
  return false;
}

const char* PS3PadBase::getPSButtonName(uint8_t btn) {
  switch (btn) {
    case Gamepad::Button::LEFT_BUMPER:
      return PSTR("L1");
    case Gamepad::Button::RIGHT_BUMPER:
      return PSTR("R1");
    case Gamepad::Button::LEFT_TRIGGER:
      return PSTR("L2");
    case Gamepad::Button::RIGHT_TRIGGER:
      return PSTR("R2");
    case Gamepad::Button::SYSTEM:
      return PSTR("PS_BUTTON");
    case Gamepad::Button::FACE_TOP:
      return PSTR("TRIANGLE");
    case Gamepad::Button::FACE_RIGHT:
      return PSTR("CIRCLE");
    case Gamepad::Button::FACE_BOTTOM:
      return PSTR("CROSS");
    case Gamepad::Button::FACE_LEFT:
      return PSTR("SQUARE");
  }
  return NULL;
}

const char* PS3PadBase::getDeviceType() const {
  return PSTR("Playstation Dualshock3 Sixaxis Controller");
}

const char* PS3PadBase::getStickName(uint8_t stk) const {
  switch (stk) {
    case ACC_X:
      return PSTR("ACCELEROMETER_X");
    case ACC_Y:
      return PSTR("ACCELEROMETER_Y");
    case ACC_Z:
      return PSTR("ACCELEROMETER_Z");
    case GYRO:
      return PSTR("GYROSCOPE");
  }
  return NULL;
}

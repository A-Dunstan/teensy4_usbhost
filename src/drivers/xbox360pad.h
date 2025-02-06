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

#ifndef _USB_XBOX360PAD_H
#define _USB_XBOX360PAD_H

#include "../teensy4_usbhost.h"

#define XBOX_BUTTON_UP           0x0001
#define XBOX_BUTTON_DOWN         0x0002
#define XBOX_BUTTON_LEFT         0x0004
#define XBOX_BUTTON_RIGHT        0x0008
#define XBOX_BUTTON_START        0x0010
#define XBOX_BUTTON_BACK         0x0020
#define XBOX_BUTTON_SELECT       XBOX_BUTTON_BACK
#define XBOX_BUTTON_L3           0x0040
#define XBOX_BUTTON_R3           0x0080
#define XBOX_BUTTON_LB           0x0100
#define XBOX_BUTTON_RB           0x0200
#define XBOX_BUTTON_SYSTEM       0x0400
#define XBOX_BUTTON_A            0x1000
#define XBOX_BUTTON_B            0x2000
#define XBOX_BUTTON_X            0x4000
#define XBOX_BUTTON_Y            0x8000

class XBOX360Pad : public USB_Driver, public USB_Driver::Factory {

  enum { REP_SIZE = 32 };
  volatile bool ready = false;
  uint8_t ep_in;
  uint8_t ep_out;

  ATOM_MUTEX lock;
  uint8_t led;
  uint8_t motor_heavy, motor_light;
  uint32_t flags;

  struct {
    uint16_t buttons;
    uint8_t trigger[2]; // left,right
    int16_t stick[2][2]; // left X/Y, right X/Y
  } state = {0};
  uint16_t cur_buttons = 0;
  uint16_t old_buttons = 0;

  const USBCallback in_cb = [=](int r) { interrupt_in(r); };
  const USBCallback out_cb = [=](int r) { interrupt_out(r); };

  uint8_t rep_in[REP_SIZE] __attribute__((aligned(32)));
  uint8_t rep_out[REP_SIZE] __attribute__((aligned(32)));

  static const usb_endpoint_descriptor* find_endpoint(const void*,size_t&);
  void interrupt_in(int);
  void interrupt_out(int);
  void SetLED(uint8_t);
  void SetRumble(uint8_t,uint8_t);

  // Factory overrides
  bool offer(const usb_interface_descriptor*, size_t) override;
  USB_Driver* attach(const usb_interface_descriptor*,size_t,USB_Device*) override;
  void detach(void) override;

public:
  XBOX360Pad();
  ~XBOX360Pad();
  operator bool() const { return ready; }

  // values 6-9 are the typical player 1-4 indicators
  void setLed(uint8_t led_value) { SetLED(led_value); }
  // there's two rumble motors: one is smaller and produces lighter vibration
  void setRumble(uint8_t heavy, uint8_t light) { SetRumble(heavy, light); }

  // capture the current button state
  void update(void) { old_buttons = cur_buttons, cur_buttons = state.buttons; }
  // buttons that stayed down since last update call
  uint16_t held(void) const { return old_buttons & cur_buttons; }
  // buttons that went up or down since last update
  uint16_t changed(void) const {return old_buttons ^ cur_buttons; }
  // buttons that went down since last update
  uint16_t pressed(void) const { return ~old_buttons & cur_buttons; }
  // buttons that went up since last update
  uint16_t released(void) const { return old_buttons & ~cur_buttons; }

  uint16_t buttons(void) const { return cur_buttons; }
  uint8_t triggerL(void) const { return state.trigger[0]; }
  uint8_t triggerR(void) const { return state.trigger[1]; }
  int stickLX(void) const { return state.stick[0][0]; }
  int stickLY(void) const { return state.stick[0][1]; }
  int stickRX(void) const { return state.stick[1][0]; }
  int stickRY(void) const { return state.stick[1][1]; }
};


#endif

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

#include "gamepad.h"

class XBOX360PadBase : public USB_Driver, protected Gamepad::Impl  {
  enum { REP_SIZE = 32 };
  uint8_t rep_in[REP_SIZE] __attribute__((aligned(32)));
  uint8_t rep_out[REP_SIZE] __attribute__((aligned(32)));

  volatile bool ready = false;
  uint8_t ep_in;
  uint8_t ep_out;

  AtomMutex mutex;
  uint8_t led;
  uint8_t motor_heavy, motor_light;
  std::atomic<uint32_t> flags;
  bool wireless;

  const USBCallback in_cb = [=](int r) { report_in(r); };
  const USBCallback out_cb = [=](int r) { report_out(r); };

  void report_in(int);
  void report_out(int);

  // USB_Driver
  bool attach(const usb_interface_descriptor*,size_t) override;
protected:
  void detach(void) override;

  // GamePad::Impl
  bool isReady() { return ready; }
  void setPlayerLED(uint8_t id) override;
  void setLED(uint32_t led_value) override;
  void setRumble(uint8_t heavy, uint8_t light) override;
  const char* getButtonName(uint8_t btn) const override { return getXBOXButtonName(btn); }
  const char* getDeviceType() const override;

public:
  static const char* getXBOXButtonName(uint8_t);
  static bool driver_match(const usb_device_descriptor*, const usb_configuration_descriptor*) { return false; }
  static bool driver_match(const usb_interface_descriptor*, size_t);
  XBOX360PadBase(Gamepad& p) : Gamepad::Impl(p) {}
};

class XBOX360Pad : public Gamepad, public XBOX360PadBase, private USB_Driver::Factory {
  // USB_Driver::Factory
  USB_Driver* offer(const usb_interface_descriptor*, size_t, const USB_Device*) override;
public:
  // solve ambiguities caused by inheriting both Gamepad and Gamepad::Impl
  using XBOX360PadBase::setPlayerLED;
  using XBOX360PadBase::setLED;
  using XBOX360PadBase::setRumble;
  using XBOX360PadBase::getDeviceType;
  using Gamepad::getButtonName;
  using Gamepad::getStickName;
  XBOX360Pad() : XBOX360PadBase(*static_cast<Gamepad*>(this)) {}
};

#endif

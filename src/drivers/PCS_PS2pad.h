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

#ifndef _USB_PCS_PSPAD_H
#define _USB_PCS_PSPAD_H

// driver for Person Communication Systems, Inc. PS2 gamepad to USB adapter
// has a standard HID descriptor but we do not use it

#include "gamepad.h"

const char* getPlaystationButtonName(uint8_t);

class PCS_PSPadBase : public USB_Driver, protected Gamepad::Impl {
  uint8_t rep_in[32] __attribute__((aligned(32)));

  volatile bool ready = false;
  uint8_t ep_in;

  const USBCallback in_cb = [=](int r) { report_in(r); };

  void report_in(int);

  // USB_Driver
  bool attach(const usb_device_descriptor*,const usb_configuration_descriptor*) override;
protected:
  void detach(void) override;

  // Gamepad::Impl
  bool isReady() override { return ready; }
  const char* getButtonName(uint8_t btn) const override { return getPlaystationButtonName(btn); }
  const char* getDeviceType() const override;

public:
  static bool driver_match(const usb_device_descriptor*, const usb_configuration_descriptor*);
  static bool driver_match(const usb_interface_descriptor*, size_t) { return false; }
  PCS_PSPadBase(Gamepad& p) : Gamepad::Impl(p) {}
};

class PCS_PSPad : public Gamepad, public PCS_PSPadBase, private USB_Driver::Factory {
  // USB_Driver::Factory
  USB_Driver* offer(const usb_device_descriptor*,const usb_configuration_descriptor*,const USB_Device*) override;
public:
  // solve ambiguities caused by inheriting both Gamepad and Gamepad::Impl
  using PCS_PSPadBase::setPlayerLED;
  using PCS_PSPadBase::setLED;
  using PCS_PSPadBase::setRumble;
  using Gamepad::getButtonName;
  using PCS_PSPadBase::getDeviceType;
  using Gamepad::getStickName;

  PCS_PSPad() : PCS_PSPadBase(*static_cast<Gamepad*>(this)) {}
};


#endif

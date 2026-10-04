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

#ifndef _USB_DUALSHOCK3_H
#define _USB_DUALSHOCK3_H

#include "gamepad.h"

class PS3PadBase : public USB_Driver, protected Gamepad::Impl {
  enum {
    ACC_X = Gamepad::Stick::STICK_CUSTOM,
    ACC_Y,
    ACC_Z,
    GYRO
  };

  uint8_t rep_in[64] __attribute__((aligned(32)));
  uint8_t rep_out[64] __attribute__((aligned(32)));

  volatile bool ready = false;
  uint8_t ep_in;
  uint8_t ep_out;
  uint8_t iface;

  AtomMutex mutex;
  uint8_t led;
  uint8_t motor_heavy, motor_light;
  std::atomic<uint32_t> flags;
  uint8_t mac_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  const USBCallback in_cb = [=](int r) { interrupt_in(r); };
  const USBCallback out_cb = [=](int r) { report_out(r); };

  void interrupt_in(int);
  void report_in(int);
  void report_out(int);

  // USB_Driver
  bool attach(const usb_device_descriptor*,const usb_configuration_descriptor*) override;
protected:
  void detach(void) override;

  // Gamepad::Impl
  bool isReady() override { return ready; }
  void setPlayerLED(uint8_t) override;
  void setRumble(uint8_t, uint8_t) override;
  const char* getButtonName(uint8_t btn) const override { return getPSButtonName(btn); }
  const char* getStickName(uint8_t stk) const override;
  const char* getDeviceType() const override;

public:
  static const char* getPSButtonName(uint8_t);
  static bool driver_match(const usb_device_descriptor*, const usb_configuration_descriptor*);
  static bool driver_match(const usb_interface_descriptor*, size_t) { return false; }
  PS3PadBase(Gamepad& p) : Gamepad::Impl(p) {}

  bool getBluetoothMAC(uint8_t* dst) const;
};

class PS3Pad : public Gamepad, public PS3PadBase, private USB_Driver::Factory {
  // USB_Driver::Factory
  USB_Driver* offer(const usb_device_descriptor*,const usb_configuration_descriptor*,const USB_Device*) override;
public:
  // solve ambiguities caused by inheriting both Gamepad and Gamepad::Impl
  using PS3PadBase::setPlayerLED;
  using PS3PadBase::setRumble;
  using PS3PadBase::getDeviceType;
  using Gamepad::Impl::setLED;
  using Gamepad::getButtonName;
  using Gamepad::getStickName;
  PS3Pad() : PS3PadBase(*static_cast<Gamepad*>(this)) {}
};


#endif

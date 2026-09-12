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

#ifndef _USB_CH341_SERIAL_H
#define _USB_CH341_SERIAL_H

#include "serial.h"

#define CH341_MIN_BPS      50
#define CH341_MAX_BPS      2000000

#define CH341_DEFAULT_BAUD 9600

namespace ch341 {

class serial : public USB_Driver::Factory, public usbserial_base {
private:

  uint8_t status_in[8] __attribute__((aligned(32)));

  uint16_t version;
  uint8_t ep_status;

  uint8_t status;

  bool hw_flow;
  uint8_t lcr;
  uint8_t factor;
  uint8_t divisor;

  void calculate_baud(uint32_t baud);
  void uart_mode(uint32_t mode);

  USBCallback status_cb = [=](int r) { status_callback(r); };
  void status_callback(int result);
  void get_status(void);

  void init(int result, unsigned int stage);
  void start(void);

  uint8_t out_status;
  void dtr_rts_callback(int,uint8_t,uint8_t);
  void set_dtr_rts(uint8_t); // should be called under rx_lock

  void detach(void) override;
  USB_Driver* offer(const usb_device_descriptor*,const usb_configuration_descriptor*,const USB_Device*) override;
  bool attach(const usb_device_descriptor*,const usb_configuration_descriptor*) override;

public:
  serial();
  ~serial();

  void set_dtr_rts(bool dtr, bool rts);
  void set_dtr(bool set);
  void set_rts(bool set);
  virtual void status_change(bool cts, bool dsr, bool ring, bool connect) {}
  bool cts(void) const;
  bool dsr(void) const;
  bool ri(void) const;
  bool cd(void) const;

  operator bool() override;
  void begin(uint32_t baud, uint16_t format, bool rts_cts);
  void begin(uint32_t baud, uint16_t format=SERIAL_8N1) override {begin(baud,format,true);}
  void end() override;
};

} // namespace ch341

#endif // _USB_CH341_SERIAL_H

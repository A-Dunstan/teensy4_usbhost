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

#ifndef _USB_CDC_ACM_H
#define _USB_CDC_ACM_H

#include "serial.h"

class cdc_acm : public USB_Driver::Factory, public usbserial_base {
  struct line_coding_t {
    uint8_t dwDTERate[4];
    uint8_t bCharFormat;
    uint8_t bParityType;
    uint8_t bDataBits;
    line_coding_t(uint32_t rate, uint16_t format);
    uint32_t baud(void) const;
  };

  uint8_t status_in[64] __attribute__((aligned(CACHE_LINE_SIZE)));
  line_coding_t line_coding __attribute__((aligned(CACHE_LINE_SIZE))) = line_coding_t(115200, SERIAL_8N1);

  uint8_t ep_status;
  uint8_t iface_control;
  uint8_t iface_data;
  uint8_t capabilities;
  uint16_t line_state;

  const uint8_t bmrtACM = USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_INTERFACE;

  const USBCallback status_cb = [=](int r) { status_callback(r); };
  /* Don't have a "fully working" CDC-ACM device to test functionality of the status
   * endpoint, so I leave this unfinished. If anyone needs it, derive from this
   * class and override the method below...*/
  virtual void status_callback(int result);

  void init(int result, unsigned int stage);

  void detach(void) override;
  USB_Driver* offer(const usb_interface_descriptor* id, size_t length, const USB_Device* d) override;
  bool attach(const usb_interface_descriptor* id, size_t length) override;

public:
  operator bool() override;

  int set_dtr_rts(bool dtr, bool rts);
  int set_dtr(bool set);
  int set_rts(bool set);
  // override in derived class for status changes
  virtual void status_change(bool cts, bool dsr, bool ring, bool connect) {}

  // sends a break of $length milliseconds
  int send_break(uint16_t length);

  void begin(uint32_t baud, uint16_t format=SERIAL_8N1) override;
};

#endif // _USB_CDC_ACM_H

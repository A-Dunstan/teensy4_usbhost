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

#include "../../teensy4_usbhost.h"

namespace AIC8800 {

class ejector : public USB_Driver {
  uint8_t csw[32] __attribute__((aligned(32)));
  const uint8_t bulk_out = 0x02;
  const uint8_t bulk_in = 0x82;

  void receive(int r);
  bool attach(const usb_device_descriptor*, const usb_configuration_descriptor*) override;
  void detach(void);
public:
  ejector() = default;
};

void ejector::receive(int r) {
  if (r >= 0) BulkMessage(bulk_in, sizeof(csw), csw, [=](int r) { receive(r); });
}

bool ejector::attach(const usb_device_descriptor*, const usb_configuration_descriptor*) {
  struct __attribute__((packed)) ms_cbw {
    uint32_t dCBWSignature;
    uint32_t dCBWTag;
    uint32_t dCBWDataTransferLength;
    uint8_t bmCBWFlags;
    uint8_t bCBWLUN;
    uint8_t bCBWCBLength;
    uint8_t CBWCB[16];
  };
  // SCSI START_STOP cmd with load/eject bit set, start(load) unset
  static const struct ms_cbw eject PROGMEM = {/*'USBC'*/0x43425355, 0, 0, 0, 0, 6, {0x1B, 0, 0, 0, 2}};

  // start receive loop
  receive(0);

  dprintf("Sending eject command to AIC mass storage\n");
  int r = BulkMessage(bulk_out, sizeof(eject), &eject, [=](int r){ dprintf("eject send result: %d(%s)\n", r, r>=31 ? "ok":"bad"); });
  if (r < 0) {
    dprintf("Failed to queue eject command: %d\n", r);
    // don't claim the device
    return false;
  }
  return true;
}

void ejector::detach(void) {
  delete this;
}

}; // AIC8800

USB_Driver* AIC8800D80::create_ejector() {
  return new AIC8800::ejector;
}

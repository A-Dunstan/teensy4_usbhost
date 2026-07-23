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

USB_Driver* AIC8800D80::offer(const usb_device_descriptor* d, const usb_configuration_descriptor*, const USB_Device*) {
  // mass storage configuration "Aic MSC"
  if (d->idVendor==0xA69C && d->idProduct==0x5721) return create_ejector();

  // ready-for-firmware configuration "AIC Wlan"
  if (d->idVendor==0xA69C && d->idProduct==0x8D80) return create_fwuploader();

  // Wifi/Bluetooth configuration "AIC 8800D80"
  // if (d->idVendor==0x368B && d->idProduct==0x8D81) claim the wifi interface only because bluetooth uses standard HCI

  return NULL;
}

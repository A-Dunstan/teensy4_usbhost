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

#ifndef _USB_AIC_H
#define _USB_AIC_H

class AIC8800D80 : USB_Driver::Factory {
  // this class just sends a SCSI mass storage eject request to kick the device out of mass storage mode
  static USB_Driver* create_ejector();
  // performs initial bluetooth/wifi firmware upload
  static USB_Driver* create_fwuploader();

  USB_Driver* offer(const usb_device_descriptor*, const usb_configuration_descriptor*, const USB_Device*) override;
};

#endif // _USB_AIC_H

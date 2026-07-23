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

#ifndef _USB_DRIVER_H
#define _USB_DRIVER_H

#include "types.h"

class USB_Driver {
  friend class USB_Device;
private:
  USB_Device* device = NULL;
  virtual void detach(void) = 0;
  virtual bool attach(const usb_device_descriptor*,const usb_configuration_descriptor*) { return false; }
  virtual bool attach(const usb_interface_descriptor*,size_t) { return false; }
public:
  virtual ~USB_Driver() = default;

  class Factory {
  private:
    static class Factory* gList;
    class Factory* next = NULL;

  protected:
    Factory() { add(); }
    ~Factory() { remove(); }
    // alternative constructor which doesn't auto-register
    constexpr Factory(int) {};
    void add(void);
    void remove(void);

    virtual USB_Driver* offer(const usb_device_descriptor*,const usb_configuration_descriptor*,const USB_Device*) {return NULL;}
    virtual USB_Driver* offer(const usb_interface_descriptor*,size_t,const USB_Device*) {return NULL;}
  public:
    static USB_Driver* find_driver(const usb_device_descriptor*,const usb_configuration_descriptor*,const USB_Device*);
    static USB_Driver* find_driver(const usb_interface_descriptor*,size_t,const USB_Device*);
  };


protected:
  // return a const pointer so it can only be compared, not accessed
  const USB_Device* getDevice(void) const { return device; }
  /* Bulk messages can be either scatter/gather or regular:
   * Scatter/gather messages contain an array of (possibly non-consecutive) buffers and lengths, one transfer per buffer is performed. Each buffer can be a maximum of
   * 16384 - 20480 bytes, depending on its starting offset within a 4096 byte page.
   * The end of a message is indicated by a NULL buffer - this means zero-length transfers must have a non-NULL data pointer!
   * Regular messages take a single consecutive buffer which will be broken into multiple transfers if neccessary.
   */
  // asynchronous, temporary callback object
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, const USBCallback& cb) {
    return ControlMessage(bmRequestType, bmRequest, wValue, wIndex, 0, (void*)NULL, cb);
  }
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback&);
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, const void *data, const USBCallback&);
  int BulkMessageZLP(uint8_t bEndpoint, USBCallback cb) { return BulkMessage(bEndpoint, 0, (const void*)NULL, cb); }
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, const void *data, USBCallback);
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, USBCallback);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, const void *data, USBCallback);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, USBCallback);
  int IsochronousMessage(uint8_t bEndpoint, isolength&, const void *data, USBCallback);
  int IsochronousMessage(uint8_t bEndpoint, isolength&, void *data, USBCallback);
  int BulkMessage(uint8_t bEndpoint, const usb_bulkintr_sg *sg, USBCallback);
  // asynchronous, callback pointer
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, const USBCallback* cb) {
    return ControlMessage(bmRequestType, bmRequest, wValue, wIndex, 0, (void*)NULL, cb);
  }
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, const void *data, const USBCallback*);
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback*);
  int BulkMessageZLP(uint8_t bEndpoint, const USBCallback* cb) { return BulkMessage(bEndpoint, 0, (const void*)NULL, cb); }
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, const void *data, const USBCallback*);
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, const USBCallback*);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, const void *data, const USBCallback*);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, const USBCallback*);
  int IsochronousMessage(uint8_t bEndpoint, isolength&, void *data, const USBCallback*);
  int IsochronousMessage(uint8_t bEndpoint, isolength&, const void *data, const USBCallback*);
  int BulkMessage(uint8_t bEndpoint, const usb_bulkintr_sg *sg, const USBCallback*);
  // synchronous
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex) {
    return ControlMessage(bmRequestType, bmRequest, wValue, wIndex, 0, (void*)NULL);
  }
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, const void *data);
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data);
  int BulkMessageZLP(uint8_t bEndpoint) { return BulkMessage(bEndpoint, 0, (const void*)NULL); }
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, const void *data);
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, const void *data);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data);
  int IsochronousMessage(uint8_t bEndpoint, isolength&, const void *data);
  int IsochronousMessage(uint8_t bEndpoint, isolength&, void *data);
};

const usb_endpoint_descriptor* get_interface_endpoint(const usb_interface_descriptor*, uint8_t index);

#endif // _USB_DRIVER_H

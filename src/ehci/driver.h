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
private:
  USB_Device* device;
  virtual void detach(void) = 0;
public:
  void disconnect(void) {device = NULL; detach();}

  class Factory {
  private:
    static class Factory* gList;
    class Factory* next;

  protected:
    Factory();
    ~Factory();
    virtual bool offer(const usb_device_descriptor*,const usb_configuration_descriptor*) {return false;}
    virtual bool offer(const usb_interface_descriptor*,size_t) {return false;}
  public:
    static class USB_Driver::Factory* find_driver(const usb_device_descriptor*,const usb_configuration_descriptor*);
    static class USB_Driver::Factory* find_driver(const usb_interface_descriptor*,size_t);
    virtual class USB_Driver* attach(const usb_device_descriptor*,const usb_configuration_descriptor*,USB_Device*) {return NULL;}
    virtual class USB_Driver* attach(const usb_interface_descriptor*,size_t,USB_Device*) {return NULL;}
  };

  virtual ~USB_Driver() = default;

protected:
  USB_Driver();
  void setDevice(USB_Device *d) {device = d;}
  // return a const pointer so it can only be compared, not accessed
  const USB_Device* getDevice(void) { return device; }
  /* Bulk messages can be either scatter/gather or regular:
   * Scatter/gather messages contain an array of (possibly non-consecutive) buffers and lengths, one transfer per buffer is performed. Each buffer can be a maximum of
   * 16384 - 20480 bytes, depending on its starting offset within a 4096 byte page.
   * The end of a message is indicated by a NULL buffer - this means zero-length transfers must have a non-NULL data pointer!
   * Regular messages take a single consecutive buffer which will be broken into multiple transfers if neccessary.
   */
  // asynchronous, temporary callback object
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback&);
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, USBCallback);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, USBCallback);
  int IsochronousMessage(uint8_t bEndpoint, isolength&, void *data, USBCallback);
  int BulkMessage(uint8_t bEndpoint, const usb_bulkintr_sg *sg, USBCallback);
  // asynchronous, callback pointer
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback*);
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, const USBCallback*);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, const USBCallback*);
  int IsochronousMessage(uint8_t bEndpoint, isolength&, void *data, const USBCallback*);
  int BulkMessage(uint8_t bEndpoint, const usb_bulkintr_sg *sg, const USBCallback*);
  // synchronous
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data);
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data);
  int IsochronousMessage(uint8_t bEndpoint, isolength&, void *data);
};

// base class for a driver that can be instantiated multiple times as needed
template <class Driver>
class USB_Driver_FactoryGlue : public USB_Driver {
  class Register : USB_Driver::Factory {
    bool offer(const usb_device_descriptor *dd, const usb_configuration_descriptor *cd) {
      return Driver::offer_config(dd,cd);
    }
    bool offer(const usb_interface_descriptor *id, size_t l) {
      return Driver::offer_interface(id, l);
    }
    USB_Driver* attach(const usb_device_descriptor *dd, const usb_configuration_descriptor *cd, USB_Device *d) {
      return Driver::attach_config(dd,cd,d);
    }
    USB_Driver* attach(const usb_interface_descriptor *id, size_t l, USB_Device *d) {
      return Driver::attach_interface(id,l,d);
    }
  };
  template<Register&> struct inst {};

  // derived class should implement these as needed
  static bool offer_config(const usb_device_descriptor*,const usb_configuration_descriptor*) {
    return false;
  }
  static bool offer_interface(const usb_interface_descriptor*,size_t) {
    return false;
  }
  static USB_Driver* attach_config(const usb_device_descriptor*,const usb_configuration_descriptor*,USB_Device*) {
    return NULL;
  }
  static USB_Driver* attach_interface(const usb_interface_descriptor*,size_t,USB_Device*) {
    return NULL;
  }

  static Register auto_reg;
  static inst<auto_reg> reg_it;
protected:
  USB_Driver_FactoryGlue(USB_Device* p) {setDevice(p);}
};

template <class Driver> typename USB_Driver_FactoryGlue<Driver>::Register USB_Driver_FactoryGlue<Driver>::auto_reg;


#endif // _USB_DRIVER_H

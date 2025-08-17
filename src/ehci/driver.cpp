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

#include "driver.h"
#include "log.h"
#include "spec.h"
#include "device.h"
#include <cerrno>

USB_Driver::Factory::Factory() {
  next = gList;
  gList = this;
}

USB_Driver::Factory::~Factory() {
  USB_Driver::Factory** p = &gList;
  while (*p != NULL) {
    if (*p == this) {
      *p = next;
      return;
    }
    p = &((*p)->next);
  }
}

USB_Driver::Factory* USB_Driver::Factory::find_driver(const usb_device_descriptor *dd, const usb_configuration_descriptor *cd) {
  USB_Driver::Factory *f = gList;
  while (f) {
    if (f->offer(dd, cd) == true)
      return f;
    f = f->next;
  }
  return NULL;
}

USB_Driver::Factory* USB_Driver::Factory::find_driver(const usb_interface_descriptor *id, size_t length) {
  USB_Driver::Factory *f = gList;
  while (f) {
    if (f->offer(id, length) == true)
      return f;
    f = f->next;
  }
  return NULL;
}

USB_Driver::Factory* USB_Driver::Factory::gList;

USB_Driver::USB_Driver() {
  device = NULL;
  dprintf("new USB_Driver %p\n", this);
}

int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback* cb_func) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_CONTROL_TRANSFER,
    .device = {
    .cb = cb_func,
      .control = {
        .bmRequestType = bmRequestType,
        .bmRequest = bmRequest,
        .wValue = wValue,
        .wIndex = wIndex,
        .wLength = wLength,
        .data = data
    }
  }
  };
  if (device == NULL) errno = ENOENT;
  else {
    if (device->pushMessage(msg))
      return 0;
    errno = ENOMEM;
  }
  return -1;
}

int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback &cb_func) {
  // control callback is always copied (see pushMessage) so don't need to clone it here
  return ControlMessage(bmRequestType,bmRequest,wValue,wIndex,wLength,data,&cb_func);
}

int USB_Driver::BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, const USBCallback* cb_func) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_BULK_TRANSFER,
    .device = {
      .cb = cb_func,
      .bulkintr = {
        .bEndpoint = bEndpoint,
        .dLength = dLength,
        .data = data
      }
    }
  };
  if (device == NULL) errno = ENOENT;
  else {
    if (device->pushMessage(msg))
      return 0;
    errno = ENOMEM;
  }
  return -1;
}

int USB_Driver::BulkMessage(uint8_t bEndpoint, const usb_bulkintr_sg* sg, const USBCallback* cb_func) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_BULK_SG_TRANSFER,
    .device = {
      .cb = cb_func,
      .bulkintr = {
        .bEndpoint = bEndpoint,
        .sg = sg
      }
    }
  };
  if (device == NULL) errno = ENOENT;
  else {
    if (device->pushMessage(msg))
      return 0;
    errno = ENOMEM;
  }
  return -1;
}

int USB_Driver::InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, const USBCallback* cb_func) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_INTERRUPT_TRANSFER,
    .device = {
    .cb = cb_func,
      .bulkintr = {
        .bEndpoint = bEndpoint,
        .dLength = wLength,
        .data = data
      }
    }
  };
  if (device == NULL) errno = ENOENT;
  else {
    if (device->pushMessage(msg))
      return 0;
    errno = ENOMEM;
  }
  return -1;
}

int USB_Driver::IsochronousMessage(uint8_t bEndpoint, isolength& Lengths, void *data, const USBCallback* cb_func) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_ISOCHRONOUS_TRANSFER,
    .device = {
      .cb = cb_func,
      .iso = {
        .bEndpoint = bEndpoint,
        .lengths = &Lengths,
        .data = data
      }
    }
  };
  if (device == NULL) errno = ENOENT;
  else if (Lengths[0] == 0) errno = EINVAL;
  else {
    if (device->pushMessage(msg))
      return 0;
    errno = ENOMEM;
  }
  return -1;
}

class FuncWrapper {
private:
  const USBCallback base;
public:
  const USBCallback wrap = [=](int r) {
    base(r);
    delete this;
  };
  FuncWrapper(USBCallback& b) : base(std::move(b)) {}
};

int USB_Driver::BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, USBCallback cb_func) {
  FuncWrapper* cb = new (std::nothrow) FuncWrapper(cb_func);
  if (cb == NULL) {
    errno = ENOMEM;
  } else {
    if (BulkMessage(bEndpoint, dLength, data, &cb->wrap) >= 0)
      return 0;
    delete cb;
  }
  return -1;
}

int USB_Driver::InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, USBCallback cb_func) {
  FuncWrapper* cb = new (std::nothrow) FuncWrapper(cb_func);
  if (cb == NULL) {
    errno = ENOMEM;
  } else {
    if (InterruptMessage(bEndpoint, wLength, data, &cb->wrap) >= 0)
      return 0;
    delete cb;
  }
  return -1;
}

int USB_Driver::IsochronousMessage(uint8_t bEndpoint, isolength& Lengths, void *data, USBCallback cb_func) {
  FuncWrapper* cb = new (std::nothrow) FuncWrapper(cb_func);
  if (cb == NULL) {
    errno = ENOMEM;
  } else {
    if (IsochronousMessage(bEndpoint, Lengths, data, &cb->wrap) >= 0)
      return 0;
    delete cb;
  }
  return -1;
}

int USB_Driver::BulkMessage(uint8_t bEndpoint, const usb_bulkintr_sg* sg, USBCallback cb_func) {
  FuncWrapper* cb = new (std::nothrow) FuncWrapper(cb_func);
  if (cb == NULL) {
    errno = ENOMEM;
  } else {
    if (BulkMessage(bEndpoint, sg, &cb->wrap) >= 0)
      return 0;
    delete cb;
  }
  return -1;
}

// synchronous functions - not implemented here, these use weak symbols so they can be overridden using OS specific code
__attribute__((weak)) int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::BulkMessage(uint8_t,uint32_t,void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::InterruptMessage(uint8_t,uint16_t,void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::IsochronousMessage(uint8_t,isolength&,void*) {
  errno = ENOSYS;
  return -1;
}

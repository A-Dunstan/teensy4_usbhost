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

void USB_Driver::Factory::add(void) {
  if (next != NULL) return;

  next = gList;
  gList = this;
}

void USB_Driver::Factory::remove(void) {
  if (next == NULL) return;

  USB_Driver::Factory** p = &gList;
  while (*p != NULL) {
    if (*p == this) {
      *p = next;
      next = NULL;
      return;
    }
    p = &((*p)->next);
  }
}

USB_Driver* USB_Driver::Factory::find_driver(const usb_device_descriptor *dd, const usb_configuration_descriptor *cd, const USB_Device *d) {
  USB_Driver::Factory *f = gList;
  while (f) {
    auto driver = f->offer(dd, cd, d);
    if (driver) return driver;
    f = f->next;
  }
  return NULL;
}

USB_Driver* USB_Driver::Factory::find_driver(const usb_interface_descriptor *id, size_t length, const USB_Device *d) {
  USB_Driver::Factory *f = gList;
  while (f) {
    auto driver = f->offer(id, length, d);
    if (driver) return driver;
    f = f->next;
  }
  return NULL;
}

int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, const void *data, const USBCallback* cb_func) {
  if (bmRequestType & USB_CTRLTYPE_DIR_DEVICE2HOST) {
    errno = EFAULT;
    return -1;
  }
  return ControlMessage(bmRequestType, bmRequest, wValue, wIndex, wLength, const_cast<void*>(data), cb_func);
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

int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, const void *data, const USBCallback &cb_func) {
  return ControlMessage(bmRequestType,bmRequest,wValue,wIndex,wLength,data,&cb_func);
}

int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback &cb_func) {
  // control callback is always copied (see pushMessage) so don't need to clone it here
  return ControlMessage(bmRequestType,bmRequest,wValue,wIndex,wLength,data,&cb_func);
}

int USB_Driver::BulkMessage(uint8_t bEndpoint, uint32_t dLength, const void *data, const USBCallback* cb_func) {
  if (bEndpoint & 0x80) {
    errno = EFAULT;
    return -1;
  }
  return BulkMessage(bEndpoint, dLength, const_cast<void*>(data), cb_func);
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
  else if (device->pushMessage(msg) == false) errno = ENOMEM;
  else return 0;

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
  else if (device->pushMessage(msg) == false) errno = ENOMEM;
  else return 0;

  return -1;
}

int USB_Driver::InterruptMessage(uint8_t bEndpoint, uint16_t wLength, const void *data, const USBCallback* cb_func) {
  if (bEndpoint & 0x80) {
    errno = EFAULT;
    return -1;
  }
  return InterruptMessage(bEndpoint, wLength, const_cast<void*>(data), cb_func);
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
  else if (device->pushMessage(msg) == false) errno = ENOMEM;
  else return 0;

  return -1;
}

int USB_Driver::IsochronousMessage(uint8_t bEndpoint, isolength& Lengths, const void *data, const USBCallback* cb_func) {
  if (bEndpoint & 0x80) {
    errno = EFAULT;
    return -1;
  }
  return IsochronousMessage(bEndpoint, Lengths, const_cast<void*>(data), cb_func);
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
  else if (device->pushMessage(msg) == false) errno = ENOMEM;
  else return 0;

  return -1;
}

int USB_Driver::Timer(uint32_t ms, const std::function<void()>* timer_cb) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_TIMER,
    .device = {
      .timer_cb = timer_cb
    }
  };
  if (device == NULL) errno = ENOENT;
  else if (timer_cb == NULL) errno = EINVAL;
  else if (device->pushMessage(msg, ms) == false) errno = ENOMEM;
  else return 0;

  return -1;
}

template <class R, class... Args, class req_fn>
static int MessageWrapper(std::function<R(Args...)>& user_cb, const req_fn& req) {
  int ret = -1;
  auto cb = new(std::nothrow) std::function<R(Args...)>;
  if (cb == NULL) errno = ENOMEM;
  else {
    *cb = [=,orig_cb(std::move(user_cb))](Args... r) {
      orig_cb(r...);
      delete cb;
    };
    ret = req(cb);
    if (ret < 0) delete cb;
  }
  return ret;
}

int USB_Driver::BulkMessage(uint8_t bEndpoint, uint32_t dLength, const void *data, USBCallback cb_func) {
  if (bEndpoint & 0x80) {
    errno = EFAULT;
    return -1;
  }
  return MessageWrapper(cb_func, [&](const USBCallback* cb)->int {
    return BulkMessage(bEndpoint, dLength, const_cast<void*>(data), cb);
  });
}

int USB_Driver::BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, USBCallback cb_func) {
  return MessageWrapper(cb_func, [&](const USBCallback* cb)->int {
    return BulkMessage(bEndpoint, dLength, data, cb);
  });
}

int USB_Driver::InterruptMessage(uint8_t bEndpoint, uint16_t wLength, const void *data, USBCallback cb_func) {
  if (bEndpoint & 0x80) {
    errno = EFAULT;
    return -1;
  }
  return MessageWrapper(cb_func, [&](const USBCallback* cb)->int {
    return InterruptMessage(bEndpoint, wLength, const_cast<void*>(data), cb);
  });
}

int USB_Driver::InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, USBCallback cb_func) {
  return MessageWrapper(cb_func, [&](const USBCallback* cb)->int {
    return InterruptMessage(bEndpoint, wLength, data, cb);
  });
}

int USB_Driver::IsochronousMessage(uint8_t bEndpoint, isolength& Lengths, const void *data, USBCallback cb_func) {
  if (bEndpoint & 0x80) {
    errno = EFAULT;
    return -1;
  }
  return MessageWrapper(cb_func, [&](const USBCallback* cb)->int {
    return IsochronousMessage(bEndpoint, Lengths, const_cast<void*>(data), cb);
  });
}

int USB_Driver::IsochronousMessage(uint8_t bEndpoint, isolength& Lengths, void *data, USBCallback cb_func) {
  return MessageWrapper(cb_func, [&](const USBCallback* cb)->int {
    return IsochronousMessage(bEndpoint, Lengths, data, cb);
  });
}

int USB_Driver::BulkMessage(uint8_t bEndpoint, const usb_bulkintr_sg* sg, USBCallback cb_func) {
  return MessageWrapper(cb_func, [&](const USBCallback* cb)->int {
    return BulkMessage(bEndpoint, sg, cb);
  });
}

int USB_Driver::Timer(uint32_t ms, std::function<void()> cb_func) {
  return MessageWrapper(cb_func, [&](std::function<void()>* cb)->int {
    return Timer(ms, cb);
  });
}

// synchronous functions - not implemented here, these use weak symbols so they can be overridden using OS specific code
__attribute__((weak)) int USB_Driver::ControlMessage(uint8_t, uint8_t, uint16_t, uint16_t, uint16_t, const void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::ControlMessage(uint8_t, uint8_t, uint16_t, uint16_t, uint16_t, void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::BulkMessage(uint8_t,uint32_t,const void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::BulkMessage(uint8_t,uint32_t,void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::InterruptMessage(uint8_t,uint16_t,const void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::InterruptMessage(uint8_t,uint16_t,void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::IsochronousMessage(uint8_t,isolength&,const void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::IsochronousMessage(uint8_t,isolength&,void*) {
  errno = ENOSYS;
  return -1;
}

const usb_endpoint_descriptor* get_interface_endpoint(const usb_interface_descriptor* desc, uint8_t index) {
  auto src = (const uint8_t*)desc;

  for (uint8_t i=0; i < desc->bNumEndpoints;) {
    if (src[1] == USB_DT_ENDPOINT) {
      if (i++ == index) return (const usb_endpoint_descriptor*)src;
    }
    src += src[0];
  }

  return NULL;
}

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

#include "hub.h"
#include "device.h"
#include "driver.h"
#include "portab.h"
#include "log.h"
#include <cstring>

#define MK_BE16(a, b) (((a)<<8)|(b))
#define HUB_CLASS 9
#define HUB_MAX_PROTOCOL 2

class USB_Hub_Driver : public USB_Driver_FactoryGlue<USB_Hub_Driver>, private CCallback<usb_control_transfer>, private USB_Hub {
private:
  USB_Device* const dev;
  const uint8_t status_ep;
  const uint8_t hs_port;
  // atomic refcount not needed, all access comes from the USB_Host thread
  unsigned int refcount;
  uint8_t bNbrPorts;
  uint8_t __attribute__((aligned(CACHE_LINE_SIZE))) port_change[32];

  USB_Hub_Driver(USB_Device*,uint8_t status);

  const USBCallback interrupt_cb = [=](int r) {int_callback(r);};
  void detach(void);
  void int_callback(int);
  void callback(const usb_control_transfer*,int);

  void port_power(uint8_t,bool) override;
  void port_reset(uint8_t,bool) override;
  void port_enable(uint8_t,bool) override;
  uint8_t HS_port(uint8_t) const override;
  void addref(void);
  void deref(void);

  void do_poll(void);
  void request_status(uint8_t port);
  bool putMessage(usb_msg_t &msg);
public:
  static bool offer_config(const usb_device_descriptor*,const usb_configuration_descriptor*);
  static USB_Driver* attach_config(const usb_device_descriptor*,const usb_configuration_descriptor*,USB_Device*);
};

USB_Hub_Driver::USB_Hub_Driver(USB_Device *d, uint8_t status) :
USB_Driver_FactoryGlue<USB_Hub_Driver>(d),
USB_Hub(d->speed==2 ? d->address : d->hub_addr),dev(d),status_ep(status),hs_port(d->speed==2 ? 16 : d->port) {
  refcount = 1;
  // get the hub descriptor
  bNbrPorts = 0;
  dprintf("Attempting to get HUB descriptor...\n");
  dev->ControlTransfer(USB_CTRLTYPE_DIR_DEVICE2HOST|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_DEVICE, \
    USB_REQ_GET_DESCRIPTOR, USB_DT_HUB<<8, 0, 255, NULL, this);
}

void USB_Hub_Driver::detach(void) {
  dprintf("USB_Hub_Driver<%p> detached\n", this);
  usb_msg_t msg = {
    .type = USB_MSG_PORT_STATUS,
    .port = {
      .hub = this,
      .status = {.POWER=1}
    }
  };
  // send status for each port saying it is powered only (disconnect all devices)
  for (uint8_t i=1; i <= bNbrPorts; i++) {
    msg.port.port = i;
    putMessage(msg);
  }
  deref();
}

bool USB_Hub_Driver::offer_config(const usb_device_descriptor *d, const usb_configuration_descriptor *c) {
  if (d->bDeviceClass != HUB_CLASS)
    return false;
  if (d->bDeviceProtocol > HUB_MAX_PROTOCOL)
    return false;
  if (d->bDeviceSubClass == 0)
    return true;
  // some hubs have bDeviceSubClass==1, check for one interface with bInterfaceClass==HUB_CLASSCODE(9)
  if (c->bNumInterfaces != 1)
    return false;
  // find first interface descriptor
  const uint8_t *b = (const uint8_t*)(c+1);
  while (b[1] != USB_DT_INTERFACE) b += b[0];

  const usb_interface_descriptor *i = (const usb_interface_descriptor*)b;
  if (i->bInterfaceClass!=HUB_CLASS || i->bInterfaceProtocol>HUB_MAX_PROTOCOL || i->bInterfaceSubClass>1)
    return false;
  if (i->bNumEndpoints < 1)
    return false;
  return true;
}

USB_Driver* USB_Hub_Driver::attach_config(const usb_device_descriptor *d, const usb_configuration_descriptor *c, USB_Device *dev) {
  // find status endpoint
  const uint8_t *b = (const uint8_t*)(c+1);
  const uint8_t *end = b + c->wTotalLength-2;
  for (int i=0; i < c->bNumInterfaces && b < end;) {
    const uint8_t* bnext = b + b[0];
    if (b[1] == USB_DT_INTERFACE) {
      auto iface = (const usb_interface_descriptor*)b;
      if (iface->bAlternateSetting==0 &&
          iface->bInterfaceClass==HUB_CLASS &&
          iface->bInterfaceProtocol<=HUB_MAX_PROTOCOL &&
          iface->bInterfaceSubClass<=1 &&
          iface->bNumEndpoints > 0) {
        // this looks like a hub interface, find an IN interrupt endpoint with packetsize==1
        ++i;
        b = bnext;
        for (int j=0; j < iface->bNumEndpoints && b < end; b += b[0]) {
          if (b[1] == USB_DT_ENDPOINT) {
            auto ep = (const usb_endpoint_descriptor*)b;
            if (ep->bEndpointAddress & USB_CTRLTYPE_DIR_DEVICE2HOST &&
                (ep->bmAttributes & 3) == USB_ENDPOINT_INTERRUPT &&
                ep->wMaxPacketSize==1) {
              return new(std::nothrow) USB_Hub_Driver(dev, ep->bEndpointAddress);
            }
            ++j;
          }
        }
        continue;
      }
    }
    b = bnext;
  }
  return NULL;
}

void USB_Hub_Driver::int_callback(int result) {
  if (result >= 1) {
    dprintf("Hub<%p>: hub interrupt %02X (%d)\n", this, port_change[0], result);
    for (uint8_t i=1; i <= bNbrPorts; i++) {
      if (port_change[0] & (1<<i)) {
        dprintf("Requesting status for port %u\n", i);
        request_status(i);
      }
    }
    if (port_change[0]==0)
      do_poll();
    return;
  }
  dprintf("Hub<%p>: polling error (%d)\n", this, result);
}

void USB_Hub_Driver::callback(const usb_control_transfer *t, int result) {
  dprintf("Hub<%p> control callback: %d\n", this, result);
  if (result < 0)
    return;

  switch(MK_BE16(t->getbmRequestType(), t->getbmRequest())) {
    case MK_BE16(USB_CTRLTYPE_DIR_DEVICE2HOST|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_OTHER,USB_REQ_GET_STATUS):
      if (result >= 4) {
        uint8_t port = t->getwIndex();
        uint32_t status_change;
        memcpy(&status_change, t->getBuffer(), sizeof(status_change));
        dprintf("Hub<%p>: port %u %08lX\n", this, port, status_change);
        usb_msg_t msg = {
          .type = USB_MSG_PORT_STATUS,
          .port = {
            .hub = this,
            .port = port,
            .status = {
              .val = (uint16_t)status_change
            }
          }
        };
        putMessage(msg);
        // clear changed features
        for (uint32_t i=USB_PORT_FEATURE_C_CONNECTION; i <= USB_PORT_FEATURE_C_RESET; i++) {
          if (status_change & (1<<i)){
            dev->ControlTransfer(USB_CTRLTYPE_DIR_HOST2DEVICE|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_OTHER, USB_REQ_CLEAR_FEATURE, \
            i, port, 0, NULL, this);
          }
        }
        port_change[0] &= ~(1<<port);
        if (port_change[0]==0)
          do_poll();
        return;
      }
      break;
    case MK_BE16(USB_CTRLTYPE_DIR_DEVICE2HOST|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_DEVICE,USB_REQ_GET_DESCRIPTOR):
      const usb_hub_descriptor *desc = (const usb_hub_descriptor*)t->getBuffer();
      if (validate_descriptor((const uint8_t*)desc, result) == USB_DT_HUB) {
        bNbrPorts = desc->bNbrPorts;
        dprintf("Hub<%p> Hub Descriptor:\n", this);
        dprintf("   bNbrPorts %d\n   wHubCharacteristics %04X\n   bPwrOn2PwrGood %d\n", desc->bNbrPorts, desc->wHubCharacteristics, desc->bPwrOn2PwrGood);
        dprintf("   bHubContrCurrent %d\n   DeviceRemovable %02X\n", desc->bHubContrCurrent, desc->DeviceRemovable);
        port_change[0] = 0;
        // get status of each port
        for (uint8_t i=1; i <= bNbrPorts; i++) {
          port_change[0] |= 1<<i;
          request_status(i);
        }
      }
      break;
  }
}

void USB_Hub_Driver::port_power(uint8_t port, bool set) {
  uint8_t bRequest = set ? USB_REQ_SET_FEATURE : USB_REQ_CLEAR_FEATURE;
  dprintf("Hub<%p>: port_power %d %d\n", this, port, set);
  dev->ControlTransfer(USB_CTRLTYPE_DIR_HOST2DEVICE|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_OTHER, bRequest, \
    USB_PORT_FEATURE_POWER, port, 0, NULL, this);
}

void USB_Hub_Driver::port_reset(uint8_t port, bool set) {
  dprintf("Hub<%p>: port_reset %d %d\n", this, port, set);
  if (set) {
    dev->ControlTransfer(USB_CTRLTYPE_DIR_HOST2DEVICE|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_OTHER, USB_REQ_SET_FEATURE, \
      USB_PORT_FEATURE_RESET, port, 0, NULL, this);
  }
}

void USB_Hub_Driver::port_enable(uint8_t port, bool set) {
  uint8_t bRequest = set ? USB_REQ_SET_FEATURE : USB_REQ_CLEAR_FEATURE;
  dprintf("Hub<%p>: port_enable %d %d\n", this, port, set);
  dev->ControlTransfer(USB_CTRLTYPE_DIR_HOST2DEVICE|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_OTHER, bRequest, \
    USB_PORT_FEATURE_ENABLE, port, 0, NULL, this);
}

uint8_t USB_Hub_Driver::HS_port(uint8_t port) const {
  if (hs_port >= 16) return port;
  return hs_port;
}

void USB_Hub_Driver::addref(void) {
  ++refcount;
}

void USB_Hub_Driver::deref(void) {
  if (refcount==0) {
    dprintf("USB_Hub_Driver<%p> deref while refcount==0\n", this);
    *((uint32_t*)0) = 1;
  }
  if (--refcount == 0) {
    dprintf("USB_Hub_Driver<%p> deleted\n", this);
    delete this;
  }
}

void USB_Hub_Driver::do_poll(void) {
  dev->InterruptTransfer(status_ep, 1, port_change, &interrupt_cb);
}

void USB_Hub_Driver::request_status(uint8_t port) {
  dev->ControlTransfer(USB_CTRLTYPE_DIR_DEVICE2HOST|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_OTHER, USB_REQ_GET_STATUS, \
    0, port, 4, NULL, this);
}

bool USB_Hub_Driver::putMessage(usb_msg_t &msg) {
  addref();
  if (dev->pushMessage(msg) == true) return true;

  deref();
  return false;
}

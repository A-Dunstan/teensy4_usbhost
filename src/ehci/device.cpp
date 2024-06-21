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

#include "device.h"
#include "host.h"
#include "driver.h"
#include <cstring>
#include <locale>
#include <codecvt>
#include <memory>

#define MK_BE16(a, b) (((a)<<8)|(b))

class dev_interface {
  class setting {
  public:
    const usb_interface_descriptor* const interface_desc;
    size_t length;
    std::vector<const usb_endpoint_descriptor*> endpoints;

    setting(const uint8_t *i, const uint8_t *end) :
    interface_desc((const usb_interface_descriptor*)i) {
      length = end-i;
      for (uint8_t e=0; e < interface_desc->bNumEndpoints;) {
        if (i[0]>=sizeof(usb_endpoint_descriptor) && i[1]==USB_DT_ENDPOINT) {
          endpoints.push_back((const usb_endpoint_descriptor*)i);
          e++;
        }
        i += i[0];
      }
    }
  };

  std::vector<setting> altSettings;
public:
  int active;
  dev_interface(const uint8_t *i, const uint8_t *end) {
    altSettings.emplace_back(i, end);
    active = -1;
  }
  void add_alternate(const uint8_t *i, const uint8_t *end) {
    altSettings.emplace_back(i, end);
    altSettings[0].length += end-i;
  }
  const usb_interface_descriptor* getInterface(size_t &l, uint8_t index) const {
    if (index < altSettings.size()) {
      l = altSettings[index].length;
      return altSettings[index].interface_desc;
    }
    l = 0;
    return NULL;
  }
  const usb_interface_descriptor* getInterface(uint8_t index=0) const {
    if (index < altSettings.size())
      return altSettings[index].interface_desc;
    return NULL;
  }
  const usb_endpoint_descriptor* getEndpoint(uint8_t eindex, uint8_t iindex=0) const {
    if (iindex < altSettings.size()) {
      if (eindex < altSettings[iindex].endpoints.size())
        return altSettings[iindex].endpoints[eindex];
    }
    return NULL;
  }
};

class dev_config {
  /* Holds the data returned by USB_REQ_GETDESCRIPTOR for this configuration.
   * Interfaces and endpoints all return pointers into this memory.
   */
  std::unique_ptr<uint8_t[]> raw;
  const size_t length;
  // array of interfaces, which is also an array of altSettings
  std::vector<dev_interface> interfaces;

public:
  dev_config(const usb_configuration_descriptor *desc, size_t len) :
  length(len) {
    std::unique_ptr<uint8_t[]> r = std::make_unique<uint8_t[]>(len);
    memcpy(&r[0], desc, len);
    const uint8_t *d = &r[sizeof(usb_configuration_descriptor)];
    const uint8_t *end = &r[len-1]+1;

    const uint8_t *interface = NULL;
    while (d < end-4) {
      if (d[0] == sizeof(usb_interface_descriptor) && d[1] == USB_DT_INTERFACE) {
        if (interface) {
          if (interface[3] == 0) { // bAlternateSetting
            // default interface, create node
            interfaces.emplace_back(interface, d);
          } else {
            // alternate interface, add to existing node
            interfaces.back().add_alternate(interface, d);
          }
        }
        interface = d;
      }
      d += d[0];
    }
    // add final
    if (interface != NULL) {
      if (interface[3] == 0) {
        interfaces.emplace_back(interface, end);
      } else {
        interfaces.back().add_alternate(interface, end);
      }
    }
    raw = std::move(r);
  }

  const usb_configuration_descriptor* getConfiguration(void) const { return (const usb_configuration_descriptor*)&raw[0]; }
  const usb_configuration_descriptor* getConfiguration(size_t &l) const {l = length; return getConfiguration(); }
  dev_interface* interface(uint8_t index) {
    if (index < interfaces.size()) {
      return &(interfaces[index]);
    }
    return NULL;
  }
};

USB_Device::Endpoint_Elem& USB_Device::Endpoint_Array::operator[] (size_t ep_addr) {
  size_t index = ep_addr & 0x7F;
  if (index != 0) {
    if (index >= 16) index = 0;
    else if (ep_addr & 0x80) index += 15;
  }
  return eps[index];
}

USB_Device::Endpoint_Array::Endpoint_Array() {
  for (size_t i=0; i < sizeof(eps)/sizeof(eps[0]); i++) {
    eps[i].ep = NULL;
    eps[i].type = -1;
  }
};


void USB_Device::callback(const usb_control_transfer *t, int result) {
  dprintf("Device<%p> control callback %p result %d\n", this, t, result);
  uint16_t rt_rq = MK_BE16(t->getbmRequestType(), t->getbmRequest());

  switch (rt_rq) {
    case MK_BE16(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR): {
        uint8_t dt = validate_descriptor((const uint8_t*)t->getBuffer(), result);
        switch (t->getwValue() >> 8) {
          case USB_DT_DEVICE:
            if (dt == USB_DT_DEVICE) {
              memcpy(&ddesc,t->getBuffer(),sizeof(ddesc));
              dprintf("Device<%p> Device Descriptor %p:\n", this, t->getBuffer());
              dprintf("   bcdUSB %04X\n   bDeviceClass %02X\n   bDeviceSubClass %02X\n   bDeviceProtocol %02X\n", ddesc.bcdUSB, ddesc.bDeviceClass, ddesc.bDeviceSubClass, ddesc.bDeviceProtocol);
              dprintf("   idVendor %04X\n   idProduct %04X\n   bcdDevice %04X\n", ddesc.idVendor, ddesc.idProduct, ddesc.bcdDevice);
              dprintf("   iManufacturer %d\n   iProduct %d\n   iSerialNumber %d\n   bNumConfigurations %d\n", ddesc.iManufacturer, ddesc.iProduct, ddesc.iSerialNumber, ddesc.bNumConfigurations);
              if (ddesc.iManufacturer) request_string(ddesc.iManufacturer);
              if (ddesc.iProduct) request_string(ddesc.iProduct);
              if (ddesc.iSerialNumber) request_string(ddesc.iSerialNumber);
              // request all configurations (config descriptor only, full size will be retrieved afterwards)
              for (uint8_t c=0; c < ddesc.bNumConfigurations; c++) {
                control.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, (USB_DT_CONFIGURATION<<8)|c, 0, sizeof(usb_configuration_descriptor), NULL, this);
              }
              return;
            }
            break;
          case USB_DT_CONFIGURATION:
            if (dt == USB_DT_CONFIGURATION) {
              const usb_configuration_descriptor *cdesc = (const usb_configuration_descriptor*)t->getBuffer();
              if (t->getwLength() == sizeof(usb_configuration_descriptor)) {
                // got the length, now get the entire thing
                if (cdesc->iConfiguration) request_string(cdesc->iConfiguration);
                control.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, t->getwValue(), 0, cdesc->wTotalLength, NULL, this);
                return;
              } else {
                configs.try_emplace(cdesc->bConfigurationValue, std::make_unique<dev_config>(cdesc, result));
                cdesc = configs.at(cdesc->bConfigurationValue)->getConfiguration();
                dprintf("Device<%p> Configuration %d:\n", this, cdesc->bConfigurationValue);
                dprintf("\twTotalLength %d\n\tbNumInterfaces %d\n\tiConfiguration %d\n", cdesc->wTotalLength, cdesc->bNumInterfaces, cdesc->iConfiguration);
                dprintf("\tbmAttributes %02X\n\tbMaxPower %d\n", cdesc->bmAttributes, cdesc->bMaxPower);
                for (uint8_t i=0; i < cdesc->bNumInterfaces;) {
                  uint8_t alt=0;
                  const dev_interface* di = configs.at(cdesc->bConfigurationValue)->interface(i);
                  while(1) {
                    size_t l;
                    const usb_interface_descriptor *idesc = di->getInterface(l, alt);
                    if (idesc == NULL) break;
                    dprintf("Interface %d, Length %u:\n", idesc->bInterfaceNumber, l);
                    dprintf("\tbAlternateSetting %d\n\tbNumEndpoints %d\n\tbInterfaceClass %02X\n", idesc->bAlternateSetting, idesc->bNumEndpoints, idesc->bInterfaceClass);
                    dprintf("\tbInterfaceSubClass %02X\n\tbInterfaceProtocol %02X\n\tiInterface %d\n", idesc->bInterfaceSubClass, idesc->bInterfaceProtocol, idesc->iInterface);
                    if (idesc->iInterface) request_string(idesc->iInterface);
                    for (uint8_t e=0; e < idesc->bNumEndpoints; e++) {
                      const usb_endpoint_descriptor *edesc = di->getEndpoint(e, alt);
                      dprintf("\t Endpoint %02X:\n", edesc->bEndpointAddress);
                      dprintf("\t\tbmAttributes %02X\n\t\twMaxPacketSize %04X\n\t\tbInterval %d\n", edesc->bmAttributes, edesc->wMaxPacketSize, edesc->bInterval);
                    }
                    ++alt;
                  }
                  ++i;
                }
              }

              if (configs.size() == ddesc.bNumConfigurations) {
                /* There may still be string requests pending and
                * it would be cleaner to look-up drivers in a
                * different function than this one, so punt this
                * action.
                */
                usb_msg_t msg = {
                  .type = USB_MSG_DEVICE_FIND_DRIVER,
                  .device = {
                   .dev = this
                  }
                };
                /* delayed message is the "safe" way, but if it
                * fails just put the message directly on the host queue.
                */
                refcount.fetch_add(1);
                if (host.timerMsg(msg, 10) == false) {
                  if (host.putMessage(msg) == false) {
                    // have to give up, driver won't be hooked and
                    // configuration/interfaces won't be activated
                    deref();
                  }
                }
              }
              return;
            }
            else
              dprintf("Device<%p> had a bad configuration descriptor\n", this);
            break;
          case USB_DT_STRING:
            // this device has a bad bDescriptorType in some of its strings
            if (ddesc.idVendor==0x0BDA && ddesc.idProduct==0x8179 && dt==0x43)
              dt = USB_DT_STRING;

            if (dt == USB_DT_STRING) {
              const usb_string_descriptor *s = (const usb_string_descriptor*)t->getBuffer();
              int len = (result-2) / 2;
              int index = t->getwValue() & 0xFF;
              if (index == 0) { // supported languages
                dprintf("Device<%p> number of supported languages: %d\n", this, len);
                for (int i=0; i < len; i++) {
                  if (s->wLANGID[i] == USB_LANG) {
                    string_lang = USB_LANG;
                    dprintf("Device<%p> using language %04X for strings\n", this, string_lang);
                    break;
                  }
                }
              } else if (t->getwIndex() == 0x0409) {
                std::wstring_convert<std::codecvt_utf8<char16_t>, char16_t> converter;
                strings[index] = converter.to_bytes(std::u16string(s->bString,len));
                dprintf("Device<%p> string index %d: %s\n", this, index, strings.at(index).c_str());
              }
              return;
            }
            break;
        }
      }
      break;
    case MK_BE16(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_CONFIGURATION):
      if (result >= 0) {
        dprintf("Device<%p> new configuration set: %02X\n", this, t->getwValue());
        activate_configuration(t->getwValue());
      } else {
        dprintf("Device<%p> setting new configuration failed %d\n", this, result);
      }
      return;
    case MK_BE16(USB_REQTYPE_INTERFACE_GET, USB_REQ_GET_INTERFACE):
      if (result >= 1) {
        const uint8_t* d = (const uint8_t*)t->getBuffer();
        dprintf("Device<%p> Interface setting retrieved: %u %u\n", this, t->getwIndex(), *d);
        activate_interface(t->getwIndex(), *d);
        return;
      }
      break;
    case MK_BE16(USB_REQTYPE_INTERFACE_SET, USB_REQ_SET_INTERFACE):
      if (result >= 0) {
        dprintf("New interface set: interface %u, altSetting %u\n", t->getwIndex(), t->getwValue());
        activate_interface(t->getwIndex(), t->getwValue());
      } else {
        dprintf("Setting new interface failed (%d) for interface %u, altSetting %u\n", result, t->getwIndex(), t->getwValue());
        // don't know which altSetting is active now - so ask
        control.Transfer(USB_REQTYPE_INTERFACE_GET, USB_REQ_GET_INTERFACE, 0, t->getwIndex(), 1, NULL, this);
      }
      return;
    case MK_BE16(USB_REQTYPE_ENDPOINT_SET, USB_REQ_CLEAR_FEATURE):
      if (t->getwValue() == USB_FEATURE_ENDPOINT_HALT) {
        if (result >= 0) {
          uint8_t endpoint = t->getwIndex();
          dprintf("HALT was cleared for endpoint %02X\n", endpoint);
          // must reset data toggle in queue head
          int ep_type = Endpoints[endpoint].type;
          if (ep_type == USB_ENDPOINT_INTERRUPT || ep_type == USB_ENDPOINT_BULK) {
            Endpoints[endpoint].ep->flush();
          }
        }
        else dprintf("Clearing endpoint halt failed\n");
        return;
      }
      break;
  }

  dprintf("USB_Device<%p>: unknown control request or failure (%d), bmRequestType %02X bmRequest %02X wValue %04X\n", this, result, t->getbmRequestType(), t->getbmRequest(), t->getwValue());
}

void USB_Device::deref(void) {
//  dprintf("USB_Device<%p> deref: %d\n", this, refcount.load()-1);
  // this performs a post-decrement so compare against 1
  if (refcount.fetch_sub(1) == 1) {
    // detach all drivers
    for (auto drv=drivers.begin(); drv != drivers.end(); drv++) {
      (*drv)->disconnect();
    }
    usb_msg_t msg = {
      .type = USB_MSG_ADDRESS_RELEASED,
      .address = address
    };
    host.putMessage(msg);
    dprintf("USB_Device<%p> deleted\n", this);
    delete this;
  }
}

USB_Device::USB_Device(USB_Host& _host, uint8_t _hub_addr, uint8_t _port, uint8_t _speed, uint8_t _address, uint8_t control_packet_size) :
control(control_packet_size, _address, _hub_addr, _port, _speed),
host(_host),
hub_addr(_hub_addr),
port(_port),
speed(_speed),
address(_address),
refcount(2) // hub and control endpoint have a reference
{
  dprintf("New Device<%p>: address %d, port %d, hub %d, speed %d, control_packet_size %d\n", this, address, port, hub_addr, speed, control_packet_size);
  memset(&ddesc, 0, sizeof(ddesc));
  string_lang = 0;
  active_config = -1;
  pending_config = -1;
  host.activate_endpoint(&control, this);
}

void USB_Device::disconnect(void) {
  // control endpoint last
  for (size_t i=1; i < 16; i++) {
    deactivate_endpoint(i);
    deactivate_endpoint(0x80|i);
  }
  host.deactivate_endpoint(&control);
  deref();
}

void USB_Device::USBMessage(const usb_msg_t& msg) {
  switch (msg.type) {
    case USB_MSG_DEVICE_INIT:
      // request string descriptor zero (supported languages)
      control.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, USB_DT_STRING << 8, 0, 255, NULL, this);
      // request device descriptor
      control.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8, 0, sizeof(ddesc), NULL, this);
      return;
    case USB_MSG_DEVICE_ENDPOINT_REMOVED:
      dprintf("USB_Device<%p> Endpoint %p removed\n", this, msg.device.endpoint);
      if (msg.device.endpoint != &control)
        delete msg.device.endpoint;
      deref();
      break;
    case USB_MSG_DEVICE_FIND_DRIVER:
      search_for_drivers();
      deref();
      break;
    case USB_MSG_DEVICE_BULK_TRANSFER:
      BulkTransfer(msg.device.bulkintr.bEndpoint, msg.device.bulkintr.dLength, msg.device.bulkintr.data, msg.device.cb);
      deref();
      break;
    case USB_MSG_DEVICE_INTERRUPT_TRANSFER:
      InterruptTransfer(msg.device.bulkintr.bEndpoint, msg.device.bulkintr.dLength, msg.device.bulkintr.data, msg.device.cb);
      deref();
      break;
    case USB_MSG_DEVICE_CONTROL_TRANSFER:
      ControlTransfer(msg.device.control.bmRequestType, msg.device.control.bmRequest, msg.device.control.wValue, msg.device.control.wIndex, msg.device.control.wLength, msg.device.control.data, msg.device.control_cb);
      deref();
      break;
    case USB_MSG_DEVICE_ISOCHRONOUS_TRANSFER:
      IsochronousTransfer(msg.device.iso.bEndpoint, *msg.device.iso.lengths, msg.device.iso.data, msg.device.cb);
      deref();
      break;
    default:
      dprintf("Unknown USB Device message: %d\n", msg.type);
  }
}

void USB_Device::request_string(uint8_t string_id) {
  if (string_lang) {
    if (strings.count(string_id)==0) {
      // add empty string placeholder in case it is requested again before this request completes
      strings[string_id] = "";
      control.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING<<8)|string_id, string_lang, 255, NULL, this);
    }
    else dprintf("String %d was already present (\"%s\")\n", string_id, strings.at(string_id).c_str());
  }
}

void USB_Device::search_for_drivers(void) {
  // try each Configuration
  for (auto c=configs.begin(); c != configs.end(); c++) {
    const usb_configuration_descriptor *config = (*c).second->getConfiguration();
    USB_Driver::Factory *f = USB_Driver::Factory::find_driver(&ddesc, config);
    if (f != NULL) {
      activate_configuration((*c).first);
      USB_Driver *d = f->attach(&ddesc, config, this);
      if (d) {
        drivers.push_back(d);
        return;
      }
    }
  }
  // else offer each individual interface from first configuration
  bool activated = false;
  dev_config* c = (*(configs.begin())).second.get();
  for (unsigned int i=0; i < 256; i++) {
    size_t l;
    const dev_interface *di = c->interface(i);
    if (di == NULL) break;
    const usb_interface_descriptor *iface = di->getInterface(l, 0);
    USB_Driver::Factory *f = USB_Driver::Factory::find_driver(iface, l);
    if (f != NULL) {
      if (!activated) {
        activate_configuration((*(configs.begin())).first);
        activated = true;
      }
      USB_Driver *d = f->attach(iface, l, this);
      if (d) {
        drivers.push_back(d);
      }
    }
  }
}

void USB_Device::deactivate_endpoint(size_t i) {
  if (Endpoints[i].type >= 0) {
    host.deactivate_endpoint(Endpoints[i].ep);
    Endpoints[i].type = -1;
    Endpoints[i].ep = NULL;
  }
}

void USB_Device::activate_endpoint(const usb_endpoint_descriptor* p) {
  size_t i = p->bEndpointAddress;
  if (Endpoints[i].type >= 0) {
    dprintf("Device<%p> activate_endpoint error: endpoint already exists for address %u\n", this, p->bEndpointAddress);
    return;
  }

  switch (p->bmAttributes & 0x3) {
    case USB_ENDPOINT_BULK:
      refcount.fetch_add(1);
      Endpoints[i].ep = createBulkEndpoint(p->bEndpointAddress, p->wMaxPacketSize, address, hub_addr, port, speed);
      if (Endpoints[i].ep != NULL) {
        Endpoints[i].type = USB_ENDPOINT_BULK;
        if (host.activate_endpoint(Endpoints[i].ep, this))
          return;
      }
      break;

    case USB_ENDPOINT_INTERRUPT:
      refcount.fetch_add(1);
      Endpoints[i].ep = createInterruptEndpoint(p->bEndpointAddress, p->wMaxPacketSize, address, hub_addr, port, speed, p->bInterval, host);
      if (Endpoints[i].ep != NULL) {
        Endpoints[i].type = USB_ENDPOINT_INTERRUPT;
        if (host.activate_endpoint(Endpoints[i].ep, this))
          return;
      }
      break;

    case USB_ENDPOINT_ISOCHRONOUS:
      if ((p->wMaxPacketSize & 0x7FF) == 0) {
        dprintf("activate_endpoint: ISO endpoint has wMaxPacketSize==0, skipping\n");
        return;
      }
      refcount.fetch_add(1);
      Endpoints[i].ep = createIsoEndpoint(p, address, hub_addr, port, speed, host);
      if (Endpoints[i].ep != NULL) {
        Endpoints[i].type = USB_ENDPOINT_ISOCHRONOUS;
        if (host.activate_endpoint(Endpoints[i].ep, this))
          return;
      }
      break;

    default:
      dprintf("Unsupported endpoint type: %02X\n", p->bmAttributes);
      return;
  }
  dprintf("Failed to activate endpoint %02X for device %p\n", p->bEndpointAddress, this);
  deref();
}

void USB_Device::activate_interface(uint8_t interface, int altsetting) {
  dprintf("Device<%p> activate interface %02X altSetting %02X\n", this, interface, altsetting);
  std::vector<const usb_endpoint_descriptor*> to_remove;
  std::vector<const usb_endpoint_descriptor*> to_add;
  if (active_config < 0)
    return;

  dev_interface *di = configs.at(active_config)->interface(interface);
  if (di == NULL)
    return;

  if (altsetting == di->active)
    return;

  if (di->active >= 0) {
    // gather all the old interface's endpoints
    uint8_t old = (uint8_t)di->active;
    for (uint8_t i=0; i < 30; i++) {
      const usb_endpoint_descriptor* ep = di->getEndpoint(i, old);
      if (ep == NULL)
        break;
      to_remove.push_back(ep);
    }
  }

  if (altsetting >= 0) {
    for (uint8_t i=0; i < 30; i++) {
      const usb_endpoint_descriptor* ep = di->getEndpoint(i,altsetting);
      if (ep == NULL)
        break;
      for (auto ep_it = to_remove.begin(); ep_it != to_remove.end(); ep_it++) {
        if (ep->bLength == (*ep_it)->bLength && memcmp(ep, *ep_it, ep->bLength)==0) {
          // endpoint is the same in old and new interface, don't need to do anything with it
          to_remove.erase(ep_it, ep_it+1);
          ep = NULL;
          break;
        }
      }
      if (ep)
        to_add.push_back(ep);
    }
  }

  // deactivate old endpoints
  for (auto ep_it = to_remove.begin(); ep_it != to_remove.end(); ep_it++) {
    const usb_endpoint_descriptor &ep = **ep_it;
    size_t epi = (ep.bEndpointAddress & 0xF) + ((ep.bEndpointAddress & 0x80) ? 15 : 0);
    deactivate_endpoint(epi-1);
  }

  di->active = altsetting;
  if (altsetting < 0)
    return;

  // set the new interface
  control.Transfer(USB_REQTYPE_INTERFACE_SET, USB_REQ_SET_INTERFACE, altsetting, interface, 0, NULL, this);
  // activate new endpoints
  for (auto ep_it = to_add.begin(); ep_it != to_add.end(); ep_it++) {
    activate_endpoint(*ep_it);
  }
}

void USB_Device::activate_configuration(int configuration) {
  dprintf("Device<%p> activate configuration %d(%d)\n", this, configuration, active_config);
  if (active_config == configuration) return;

  if (active_config >= 0) {
    // disable all currently active interfaces
    const usb_configuration_descriptor *c = configs.at(active_config)->getConfiguration();
    for (uint8_t i=0; i < c->bNumInterfaces; i++)
      activate_interface(i, -1);
    active_config = -1;
  }
  if (configs.count(configuration)==0) return;

  control.Transfer(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_CONFIGURATION, configuration, 0, 0, NULL, this);
  active_config = configuration;
  const usb_configuration_descriptor *c = configs.at(active_config)->getConfiguration();
  for (uint8_t i=0; i < c->bNumInterfaces; i++)
    activate_interface(i, 0);
}

/* Some control transfers from drivers need to have their return value checked by USB_Device
 * e.g. if the driver changes an interface's altSetting the USB_Device needs to reconfigure
 * the endpoints. So use this callback class to pass the result to USB_Device::callback and
 * then the original callback.
 */
class ControlCallback : public CCallback<usb_control_transfer> {
private:
  const USBCallback driver_cb;
  CCallback<usb_control_transfer>* const device_cb;

  void callback(const usb_control_transfer *t, int result) override {
    if (device_cb)
      device_cb->callback(t, result);
    driver_cb(result);
    delete this;
  }
public:
  ControlCallback(const USBCallback* drv_cb, CCallback<usb_control_transfer>* dev_cb) :
  driver_cb(*drv_cb),
  device_cb(dev_cb) {}
};

bool USB_Device::prepare_control_transfer(usb_msg_t &msg) {
  bool proxy = false;
  const uint8_t bmRequestType = msg.device.control.bmRequestType;
  const uint8_t bmRequest = msg.device.control.bmRequest;
  const uint16_t wValue = msg.device.control.wValue;
  const uint16_t wIndex = msg.device.control.wIndex;

  uint16_t rt_rq = MK_BE16(bmRequestType, bmRequest);
  switch (rt_rq) {
    // change device address is not allowed
    case MK_BE16(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS):
      errno = EINVAL;
      return false;
    case MK_BE16(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR):
      if ((wValue >> 8) == USB_DT_STRING)
        proxy = true;
      break;
    case MK_BE16(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_CONFIGURATION):
      // TODO: allow this from device drivers but not interface drivers
      errno = EINVAL;
      return false;
    case MK_BE16(USB_REQTYPE_INTERFACE_GET, USB_REQ_GET_INTERFACE):
    case MK_BE16(USB_REQTYPE_INTERFACE_SET, USB_REQ_SET_INTERFACE):
      proxy = true;
      break;
    case MK_BE16(USB_REQTYPE_ENDPOINT_SET, USB_REQ_CLEAR_FEATURE):
      if (wValue == USB_FEATURE_ENDPOINT_HALT) {
        if (wIndex == 0) { // CLEAR_HALT is invalid for endpoint 0
          errno = EINVAL;
          return false;
        }
        proxy = true;
      }
      break;
  }

  msg.device.control_cb = new ControlCallback(msg.device.cb, proxy ? this : NULL);
  return true;
}

void USB_Device::BulkTransfer(uint8_t bEndpoint, uint32_t dLength, void *data, const USBCallback* cb) {
  if (Endpoints[bEndpoint].type == USB_ENDPOINT_BULK) {
    if (Endpoints[bEndpoint].ep->BulkTransfer(dLength, data, cb) == 0)
      return;
  }
  else errno = ENXIO;
  (*cb)(-errno);
}

void USB_Device::InterruptTransfer(uint8_t bEndpoint, uint32_t dLength, void *data, const USBCallback* cb) {
  if (Endpoints[bEndpoint].type == USB_ENDPOINT_INTERRUPT) {
    if (Endpoints[bEndpoint].ep->InterruptTransfer(dLength, data, cb) == 0)
      return;
  }
  else errno = ENXIO;
  (*cb)(-errno);
}

void USB_Device::IsochronousTransfer(uint8_t bEndpoint, isolength& Lengths, void *data, const USBCallback* cb) {
  if (Endpoints[bEndpoint].type == USB_ENDPOINT_ISOCHRONOUS) {
    if (Endpoints[bEndpoint].ep->IsochronousTransfer(Lengths, data, cb) == 0)
      return;
  }
  else errno = ENXIO;
  (*cb)(-errno);
}

void USB_Device::ControlTransfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, CCallback<usb_control_transfer>* cb) {
  if (control.Transfer(bmRequestType, bmRequest, wValue, wIndex, wLength, data, cb) < 0) {
    cb->callback(NULL, -errno);
  }
}

bool USB_Device::pushMessage(usb_msg_t& msg) {
  switch (msg.type) {
    case USB_MSG_DEVICE_CONTROL_TRANSFER:
      if (prepare_control_transfer(msg) == false)
        return false;
    case USB_MSG_DEVICE_BULK_TRANSFER:
    case USB_MSG_DEVICE_INTERRUPT_TRANSFER:
    case USB_MSG_DEVICE_ISOCHRONOUS_TRANSFER:
      msg.device.dev = this;
      refcount.fetch_add(1);
    default:
      break;
  }
  return host.putMessage(msg);
}

uint8_t validate_descriptor(const uint8_t* &desc, const uint8_t* const end) {
  int length = end - desc;
  if (length < 2) return 0;
  if (length < desc[0]) {
//    dprintf("validate descriptor %p length %d less than desc_len %u\n", desc, length, desc[0]);
    return 0;
  }
  if (desc[0] == 0) {
//    dprintf("validate_descriptor %p: descriptor length is 0\n", desc);
    return 0;
  }

//  dprintf("validate_descriptor: %p %02X %02X\n", desc, desc[0], desc[1]);

  switch (desc[1]) {
    case USB_DT_DEVICE:
      if (desc[0] == sizeof(usb_device_descriptor)) {
        desc += sizeof(usb_device_descriptor);
        return USB_DT_DEVICE;
      }
      break;
    case USB_DT_CONFIGURATION:
      if (desc[0] == sizeof(usb_configuration_descriptor)) {
        const usb_configuration_descriptor* c = (const usb_configuration_descriptor*)desc;
        // a configuration must have one or more interfaces
        if (c->bNumInterfaces == 0)
          return 0;
        desc += sizeof(usb_configuration_descriptor);
//        dprintf("validate_descriptor %p DT_CONFIGURATION #%d, %d interfaces\n", c, c->bConfigurationValue, c->bNumInterfaces);
        if (length == sizeof(usb_configuration_descriptor)) {
          return USB_DT_CONFIGURATION;
        }
        uint8_t i=0;
        uint8_t alt=0;
        while (desc < end) {
          const usb_interface_descriptor *a = (const usb_interface_descriptor*)desc;
          uint8_t dt = validate_descriptor(desc, end);
          if (dt == 0) break;
          // interfaces must be sorted by bInterfaceNumber then bAlternateSetting
          if (dt == USB_DT_INTERFACE) {
            if (a->bAlternateSetting == 0) {
              if (i++ != a->bInterfaceNumber) break;
              alt = 1;
            } else {
              if (i-1 != a->bInterfaceNumber) break;
              if (alt++ != a->bAlternateSetting) break;
            }
          }
          // endpoint should not occur outside of an interface
          else if (dt == USB_DT_ENDPOINT)
            break;
        }
        if (i == c->bNumInterfaces && desc == end)
          return USB_DT_CONFIGURATION;
        // rewind on failure
        desc = (const uint8_t*)c;
      }
      break;
    case USB_DT_STRING:
      // strings must be an even number of bytes long
      if ((desc[0] & 1)==0) {
        desc += desc[0];
        return USB_DT_STRING;
      }
      break;
    case USB_DT_INTERFACE:
      if (desc[0] == sizeof(usb_interface_descriptor)) {
        const usb_interface_descriptor* c = (const usb_interface_descriptor*)desc;
        desc += sizeof(usb_interface_descriptor);
//        dprintf("validate_descriptor %p DT_INTERFACE #%d(%d), %d endpoints\n", c, c->bInterfaceNumber, c->bAlternateSetting, c->bNumEndpoints);
        if (length == sizeof(usb_interface_descriptor) || c->bNumEndpoints==0) {
          return USB_DT_INTERFACE;
        }
        uint8_t e=0;
        while (1) {
          uint8_t dt = validate_descriptor(desc, end);
          if (dt == 0) break;
          if (dt == USB_DT_ENDPOINT && (++e == c->bNumEndpoints))
            return USB_DT_INTERFACE;
        }
        // rewind on failure
        desc = (const uint8_t*)c;
      }
      break;
    case USB_DT_ENDPOINT:
      // audio endpoints can be 9 bytes long instead of 7
      if (desc[0] >= sizeof(usb_endpoint_descriptor)) {
        const usb_endpoint_descriptor* e = (const usb_endpoint_descriptor*)desc;
//        dprintf("validate_descriptor %p DT_ENDPOINT %02X length %u\n", desc, e->bEndpointAddress, desc[0]);
        // Endpoint address 0 and control endpoint types are not valid
        if ((e->bEndpointAddress & 0xF) != 0 && (e->bmAttributes & 0x3) != 0) {
          desc += desc[0];
          return USB_DT_ENDPOINT;
        }
      }
      break;
    case USB_DT_HUB:
      if (desc[0] >= sizeof(usb_hub_descriptor)) {
        const usb_hub_descriptor* h = (const usb_hub_descriptor*)desc;
        if (h->bNbrPorts <= 7) {
          desc += desc[0];
          return USB_DT_HUB;
        }
      }
      break;
    default:
      uint8_t dt = desc[1];
      desc += desc[0];
      return dt;
  }
  return 0;
}

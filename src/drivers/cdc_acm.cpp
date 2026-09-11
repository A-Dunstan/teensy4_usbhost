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

#include "../teensy4_usbhost.h"

#define USB_INTERFACE_CLASS_CDC          2
#define USB_INTERFACE_CLASS_DATA         10
#define USB_INTERFACE_SUBCLASS_CDCACM    2

#define USB_INTERFACE_CDC_SUBTYPE_ACM    2
#define USB_INTERFACE_CDC_SUBTYPE_UNION  6

#define ACM_CAPABILITY_COMM_FEATURE      (1<<0)
#define ACM_CAPABILITY_LINE_CODING       (1<<1)
#define ACM_CAPABILITY_SEND_BREAK        (1<<2)
#define ACM_CAPABILITY_NET_CONNECT       (1<<3)

#define SET_LINE_CODING                  0x20
#define GET_LINE_CODING                  0x21
#define SET_LINE_CONTROL_STATE           0x22
#define SEND_BREAK                       0x23

struct usb_class_interface_descriptor : usb_descriptor {
  enum { DescriptorType = USB_DT_INTERFACE|USB_CTRLTYPE_TYPE_CLASS };
  uint8_t bDescriptorSubType;
};

struct usb_union_functional_descriptor : usb_class_interface_descriptor {
  enum { DescriptorSubType = USB_INTERFACE_CDC_SUBTYPE_UNION };
  uint8_t bControlInterface;
  uint8_t bSubordinateInterface0;
};

struct usb_acm_functional_descriptor : usb_class_interface_descriptor {
  enum { DescriptorSubType = USB_INTERFACE_CDC_SUBTYPE_ACM };
  uint8_t bmCapabilities;
};

cdc_acm::line_coding_t::line_coding_t(uint32_t rate, uint16_t format) {
  bDataBits = 8;
  dwDTERate[0] = rate >> 0;
  dwDTERate[1] = rate >> 8;
  dwDTERate[2] = rate >> 16;
  dwDTERate[3] = rate >> 24;
  bCharFormat = format & SERIAL_2STOP_BITS ? 2 : 0;
  if (format & 2) {
    if ((format & 4) == 0) --bDataBits;
    bParityType = format & 1 ? 1 : 2;
  }
  else bParityType = 0;
}

uint32_t cdc_acm::line_coding_t::baud(void) const {
  return (dwDTERate[3] << 24) | (dwDTERate[2] << 16) | (dwDTERate[1] << 8) | dwDTERate[0];
}

void cdc_acm::status_callback(int result) {
  if (result >= 0) {
    InterruptMessage(ep_status, sizeof(status_in), status_in, &status_cb);
  }
}

void cdc_acm::init(int result, unsigned int stage) {
//  dprintf("cdc_asm::init stage %u result %d\n", stage, result);
  USBCallback fn( [=,nextstage=stage+1](int r) { init(r, nextstage);} );

  if (result < 0) return;

  switch (stage) {
    case 0:
      // set line coding
      ControlMessage(bmrtACM, SET_LINE_CODING, 0, iface_control, sizeof(line_coding), &line_coding, fn);
      break;
    case 1:
      // set DTR and RTS
      set_dtr_rts(true, true);
      start();
      break;
  }
}

void cdc_acm::detach(void) {
  capabilities = 0;
  end();
}

USB_Driver* cdc_acm::offer(const usb_interface_descriptor* id, size_t length, const USB_Device* d) {
  if (getDevice()==NULL) {
    if (id->bInterfaceClass==USB_INTERFACE_CLASS_CDC && id->bInterfaceSubClass==USB_INTERFACE_SUBCLASS_CDCACM)
      return this;
  } else if (getDevice()==d && id->bInterfaceNumber==iface_data) {
    if (id->bInterfaceClass==USB_INTERFACE_CLASS_DATA && id->bInterfaceSubClass==0 && id->bInterfaceProtocol==0 && id->bNumEndpoints>=2)
      return this;
  }

  return NULL;
}

bool cdc_acm::attach(const usb_interface_descriptor* id, size_t length) {
  if (id->bInterfaceClass == USB_INTERFACE_CLASS_CDC) { // control interface
    iface_control = iface_data = 255;
    ep_status = 0;
    capabilities = 0;
    auto end = &id->bLength + length - 2;
    for (const usb_descriptor* desc = id->next(); &desc->bLength < end; desc = desc->next()) {
      if (desc->bDescriptorType == usb_interface_descriptor::DescriptorType) {
        // if we've hit another interface stop parsing
        break;
      } else if (desc->bDescriptorType == usb_endpoint_descriptor::DescriptorType && desc->bLength >= sizeof(usb_endpoint_descriptor)) {
        auto ep = static_cast<const usb_endpoint_descriptor*>(desc);
        if (ep_status==0 && ep->bEndpointAddress&0x80 && ep->bmAttributes==USB_ENDPOINT_INTERRUPT && ep->wMaxPacketSize <= sizeof(status_in))
          ep_status = ep->bEndpointAddress;
      } else if (desc->bDescriptorType == usb_class_interface_descriptor::DescriptorType && desc->bLength >= sizeof(usb_class_interface_descriptor)) {
        auto cd = static_cast<const usb_class_interface_descriptor*>(desc);
        if (cd->bDescriptorSubType == usb_union_functional_descriptor::DescriptorSubType && desc->bLength >= sizeof(usb_union_functional_descriptor)) {
          auto ufd = static_cast<const usb_union_functional_descriptor*>(desc);
          iface_control = ufd->bControlInterface;
          iface_data = ufd->bSubordinateInterface0;
        } else if (cd->bDescriptorSubType == usb_acm_functional_descriptor::DescriptorSubType && desc->bLength >= sizeof(usb_acm_functional_descriptor)) {
          auto afd = static_cast<const usb_acm_functional_descriptor*>(desc);
          capabilities = afd->bmCapabilities;
        }
      }
    }
    if (id->bInterfaceNumber==iface_control) {
      // TODO: begin polling serial status if supported
      if (ep_status && capabilities & ACM_CAPABILITY_LINE_CODING)
        InterruptMessage(ep_status, sizeof(status_in), status_in, &status_cb);
      return true;
    }
  } else { // data interface
    ep_out = ep_in = 0;
    for (uint8_t i=0; i < id->bNumEndpoints; i++) {
      auto ep = get_interface_endpoint(id, i);
      if (ep->bmAttributes != USB_ENDPOINT_BULK) continue;
      if (ep->bEndpointAddress & 0x80) {
        if (ep_in == 0) {
          ep_in = ep->bEndpointAddress;
          ep_in_len = ep->wMaxPacketSize;
        }
      } else if (ep_out == 0) {
        ep_out = ep->bEndpointAddress;
        ep_out_len = ep->wMaxPacketSize;
        tx_wmark = ep_out_len;
      }
    }
    if (ep_out && ep_in) {
      line_state = 0;
      init(0,0);
      return true;
    }
  }

  return false;
}

cdc_acm::operator bool() {
  return state & STATE_STARTED;
}

int cdc_acm::set_dtr_rts(bool dtr, bool rts) {
  if (getDevice() == NULL) errno = ENODEV;
  else if ((capabilities & ACM_CAPABILITY_LINE_CODING)==0) errno = EOPNOTSUPP;
  else {
    uint16_t new_line_state = (dtr ? 1:0) | (rts ? 2:0);
    if (new_line_state == line_state) return 0;
    return ControlMessage(bmrtACM, SET_LINE_CONTROL_STATE, new_line_state, iface_control, [=](int r) {
        if (r >= 0) line_state = new_line_state;
    });
  }

  return -1;
}

int cdc_acm::set_dtr(bool set) { return set_dtr_rts(set, line_state & 2); }
int cdc_acm::set_rts(bool set) { return set_dtr_rts(line_state & 1, set); }

int cdc_acm::send_break(uint16_t length) {
  if (getDevice() == NULL) errno = ENODEV;
  else if ((capabilities & ACM_CAPABILITY_SEND_BREAK)==0) errno = EOPNOTSUPP;
  else return ControlMessage(bmrtACM, SEND_BREAK, length, iface_control);
  return -1;
}

void cdc_acm::begin(uint32_t baud, uint16_t format) {
  line_coding = line_coding_t(baud, format);

  if (state & STATE_STARTED && capabilities & ACM_CAPABILITY_LINE_CODING) {
    ControlMessage(bmrtACM, SET_LINE_CODING, 0, iface_control, sizeof(line_coding), &line_coding);
  }
}

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

#ifndef _USB_TYPES_H
#define _USB_TYPES_H

#include "spec.h"
#include <cstdint>
#include <functional>

typedef std::function<void(int)> USBCallback;
typedef int16_t isolength[8];

class USB_Hub;
class USB_Endpoint;
class USB_Device;
class USB_Driver;
class USB_Host;

template <class CTransfer>
class CCallback {
public:
  virtual void callback(const CTransfer*,int) = 0;
  virtual ~CCallback() {}
};

class usb_control_transfer {
protected:
  usb_control_setup req;
  void* buffer;
public:
  uint8_t getbmRequestType(void) const { return req.bmRequestType; }
  uint8_t getbmRequest(void) const { return req.bmRequest;}
  uint16_t getwValue(void) const { return req.wValue; }
  uint16_t getwIndex(void) const { return req.wIndex; }
  uint16_t getwLength(void) const { return req.wLength; }
  const usb_control_setup& getSetup(void) const { return req; }
  const void* getBuffer(void) const { return buffer; }
  bool compare(const usb_control_setup& s) const {
    if (s.bmRequestType != req.bmRequestType) return false;
    if (s.bmRequest != req.bmRequest) return false;
    if (s.wValue != req.wValue) return false;
    if (s.wIndex != req.wIndex) return false;
    if (s.wLength != req.wLength) return false;
    return true;
  }
};

typedef struct {
  const uint8_t CAPLENGTH;
  const uint16_t HCIVERSION;
  struct {
    const uint32_t N_PORTS:4;
    const uint32_t PPC:1;
    const uint32_t :3;
    const uint32_t N_PCC:4;
    const uint32_t N_CC:4;
    const uint32_t P_INDICATOR:1;
    const uint32_t :3;
    const uint32_t N_PTT:4;
    const uint32_t N_TT:4;
    const uint32_t :4;
  } HCSPARAMS;
  struct {
    const uint32_t ADC:1;
    const uint32_t PFL:1;
    const uint32_t ASP:1;
    const uint32_t :1;
    const uint32_t IST:4;
    const uint32_t EECP:8;
    const uint32_t :16;
  } HCCPARAMS;
} usb_ehci_base_t;

typedef struct {
  volatile uint32_t USBCMD;
  volatile uint32_t USBSTS;
  volatile uint32_t USBINTR;
  volatile uint32_t FRINDEX;
  volatile uint32_t CTRLDSSEGMENT;
  void* volatile PERIODICLISTBASE;
  volatile uint32_t ASYNCLISTADDR;
  const uint32_t pad[9];
  volatile uint32_t CONFIGFLAG;
  volatile uint32_t PORTSC[0];
} usb_ehci_cmd_t;

typedef union {
  struct {
    uint16_t CONNECTION:1;
    uint16_t ENABLE:1;
    uint16_t SUSPEND:1;
    uint16_t OVER_CURRENT:1;
    uint16_t RESET:1;
    uint16_t :3;
    uint16_t POWER:1;
    uint16_t LOW_SPEED:1;
    uint16_t HIGH_SPEED:1;
    uint16_t TEST:1;
    uint16_t INDICATOR:1;
    uint16_t :3;
  };
  uint16_t val;
} port_status_t;

typedef enum {
  USB_MSG_INTERRUPT = 0,
  USB_MSG_ADDRESS_RELEASED,
  USB_MSG_PORT_STATUS = 0x100,
  USB_MSG_PORT_TIMEOUT,
  USB_MSG_ENDPOINT_ACTIVATE = 0x200,
  USB_MSG_ENDPOINT_DEACTIVATE,
  USB_MSG_DEVICE_INIT = 0x300,
  USB_MSG_DEVICE_ENDPOINT_REMOVED,
  USB_MSG_DEVICE_FIND_DRIVER,
  USB_MSG_DEVICE_CONTROL_TRANSFER,
  USB_MSG_DEVICE_BULK_TRANSFER,
  USB_MSG_DEVICE_INTERRUPT_TRANSFER,
  USB_MSG_DEVICE_ISOCHRONOUS_TRANSFER,
} usb_msg_type_t;

typedef struct {
  usb_msg_type_t type;
  union {
    struct {
      USB_Hub* hub;
      uint8_t port;
      port_status_t status;
    } port;
    struct {
      USB_Endpoint *ep;
      USB_Device *device;
    } endpoint;
    struct {
      class USB_Device *dev;
      union {
        const USBCallback* cb;
        CCallback<usb_control_transfer>* control_cb;
      };
      union {
        class USB_Endpoint *endpoint;
        struct {
          uint8_t bmRequestType;
          uint8_t bmRequest;
          uint16_t wValue;
          uint16_t wIndex;
          uint16_t wLength;
          void* data;
        } control;
        struct {
          uint8_t bEndpoint;
          uint32_t dLength;
          void* data;
        } bulkintr;
        struct {
          uint8_t bEndpoint;
          isolength* lengths;
          void *data;
        } iso;
      };
    } device;
    uint8_t address;
  };
} usb_msg_t;

#endif // _USB_TYPES_H

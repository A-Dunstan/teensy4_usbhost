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

#ifndef _USB_DEVICE_H
#define _USB_DEVICE_H

#include "spec.h"
#include "types.h"
#include "endpoint_qh.h"
#include <atomic>
#include <vector>
#include <map>
#include <string>
#include <memory>

class USB_Device : private CCallback<usb_control_transfer> {
  friend class USB_Host;
  friend class USB_Hub_Driver;
private:
  USB_Control_Endpoint control;
  class USB_Host& host;
  const uint8_t hub_addr;
  const uint8_t port;
  const uint8_t speed;
  const uint8_t address;
  std::atomic_uint refcount;

  struct Endpoint_Elem {
    USB_Endpoint *ep;
    int type;
  };
  struct Endpoint_Array {
    // 1-15=OUT,16-30=IN
    Endpoint_Elem eps[31];
    Endpoint_Elem& operator[] (size_t ep_addr);
    Endpoint_Array();
  } Endpoints;

  usb_device_descriptor ddesc;
  uint16_t string_lang;
  std::map<uint8_t,std::string> strings;
  std::map<uint8_t,std::unique_ptr<class dev_config>> configs;
  // array of attached drivers for this device
  std::vector<class USB_Driver*> drivers;
  int active_config, pending_config;

  ~USB_Device() = default;

  void callback(const usb_control_transfer*,int) override;
  void deref(void);

  USB_Device(USB_Host& _host, uint8_t _hub_addr, uint8_t _port, uint8_t _speed, uint8_t _address, uint8_t control_packet_size);
  void disconnect(void);

  void USBMessage(const usb_msg_t&);
  void request_string(uint8_t string_id);
  void search_for_drivers(void);

  void deactivate_endpoint(size_t i);
  void activate_endpoint(const usb_endpoint_descriptor*);
  void activate_interface(uint8_t interface, int altsetting);
  void activate_configuration(int configuration);

  bool prepare_control_transfer(usb_msg_t& msg);
  int bulk_intr_transfer(uint32_t dLength, void* data, uint16_t packet_align, const std::function<int(const usb_bulkintr_sg*)> &Transfer);
  void BulkTransfer(uint8_t bEndpoint, uint32_t dLength, void* data, const USBCallback* cb);
  void InterruptTransfer(uint8_t bEndpoint, uint16_t dLength, void* data, const USBCallback* cb);
  void IsochronousTransfer(uint8_t bEndpoint, isolength& Lengths, void* data, const USBCallback* cb);
  void ControlTransfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, CCallback<usb_control_transfer>*);
  void BulkTransfer(uint8_t bEndpoint, const usb_bulkintr_sg* sg, const USBCallback* cb);

public:
  bool pushMessage(usb_msg_t&);
};


uint8_t validate_descriptor(const uint8_t* &desc, const uint8_t* const end);
static inline uint8_t validate_descriptor(const uint8_t* desc, int length) {
  if (length <= 0) return 0;
  return validate_descriptor(desc, desc+length);
}

#endif // _USB_DEVICE_H

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

#ifndef _USB_SPEC_H
#define _USB_SPEC_H

#include <cstdint>

#define USB_CTRLTYPE_DIR_HOST2DEVICE     (0<<7)
#define USB_CTRLTYPE_DIR_DEVICE2HOST     (1<<7)
#define USB_CTRLTYPE_TYPE_STANDARD       (0<<5)
#define USB_CTRLTYPE_TYPE_CLASS          (1<<5)
#define USB_CTRLTYPE_TYPE_VENDOR         (2<<5)
#define USB_CTRLTYPE_TYPE_RESERVED       (3<<5)
#define USB_CTRLTYPE_REC_DEVICE          0
#define USB_CTRLTYPE_REC_INTERFACE       1
#define USB_CTRLTYPE_REC_ENDPOINT        2
#define USB_CTRLTYPE_REC_OTHER           3

#define USB_REQTYPE_DEVICE_GET           (USB_CTRLTYPE_DIR_DEVICE2HOST|USB_CTRLTYPE_TYPE_STANDARD|USB_CTRLTYPE_REC_DEVICE)
#define USB_REQTYPE_DEVICE_SET           (USB_CTRLTYPE_DIR_HOST2DEVICE|USB_CTRLTYPE_TYPE_STANDARD|USB_CTRLTYPE_REC_DEVICE)
#define USB_REQTYPE_INTERFACE_GET        (USB_CTRLTYPE_DIR_DEVICE2HOST|USB_CTRLTYPE_TYPE_STANDARD|USB_CTRLTYPE_REC_INTERFACE)
#define USB_REQTYPE_INTERFACE_SET        (USB_CTRLTYPE_DIR_HOST2DEVICE|USB_CTRLTYPE_TYPE_STANDARD|USB_CTRLTYPE_REC_INTERFACE)
#define USB_REQTYPE_ENDPOINT_GET         (USB_CTRLTYPE_DIR_DEVICE2HOST|USB_CTRLTYPE_TYPE_STANDARD|USB_CTRLTYPE_REC_ENDPOINT)
#define USB_REQTYPE_ENDPOINT_SET         (USB_CTRLTYPE_DIR_HOST2DEVICE|USB_CTRLTYPE_TYPE_STANDARD|USB_CTRLTYPE_REC_ENDPOINT)

/* standard requests */
#define USB_REQ_GET_STATUS               0
#define USB_REQ_CLEAR_FEATURE            1
#define USB_REQ_SET_FEATURE              3
#define USB_REQ_SET_ADDRESS              5
#define USB_REQ_GET_DESCRIPTOR           6
#define USB_REQ_SET_CONFIGURATION        9
#define USB_REQ_GET_INTERFACE            10
#define USB_REQ_SET_INTERFACE            11

/* non-standard (class) requests */
#define USB_REQ_SETPROTOCOL              11

#define USB_FEATURE_ENDPOINT_HALT        0

#define USB_DT_DEVICE                    1
#define USB_DT_CONFIGURATION             2
#define USB_DT_STRING                    3
#define USB_DT_INTERFACE                 4
#define USB_DT_ENDPOINT                  5
#define USB_DT_DEVICE_QUALIFIER          6
#define USB_DT_OTHER_SPEED_CONFIGURATION 7
#define USB_DT_INTERFACE_POWER           8
#define USB_DT_INTERFACE_ASSOCIATION     11
#define USB_DT_HID                       (1 | USB_CTRLTYPE_TYPE_CLASS)
#define USB_DT_HUB                       (9 | USB_CTRLTYPE_TYPE_CLASS)

#define USB_ENDPOINT_CONTROL             0
#define USB_ENDPOINT_ISOCHRONOUS         1
#define USB_ENDPOINT_BULK                2
#define USB_ENDPOINT_INTERRUPT           3

typedef struct {
  uint8_t bmRequestType;
  uint8_t bmRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} usb_control_setup;

typedef struct {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdUSB;
  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bMaxPacketSize;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t iManufacturer;
  uint8_t iProduct;
  uint8_t iSerialNumber;
  uint8_t bNumConfigurations;
} usb_device_descriptor;

typedef struct {
  uint8_t bLength;
  uint8_t bDescriptorType;
  union {
    uint16_t wLANGID[0];
    char16_t bString[0];
  };
} usb_string_descriptor;

// these descriptors may occur unaligned so are all declared as packed

typedef struct {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t wTotalLength;
  uint8_t bNumInterfaces;
  uint8_t bConfigurationValue;
  uint8_t iConfiguration;
  uint8_t bmAttributes;
  uint8_t bMaxPower;
} __attribute__((packed)) usb_configuration_descriptor;

typedef struct {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bInterfaceNumber;
  uint8_t bAlternateSetting;
  uint8_t bNumEndpoints;
  uint8_t bInterfaceClass;
  uint8_t bInterfaceSubClass;
  uint8_t bInterfaceProtocol;
  uint8_t iInterface;
} __attribute__((packed)) usb_interface_descriptor;

typedef struct {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bEndpointAddress;
  uint8_t bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t bInterval;
} __attribute__((packed)) usb_endpoint_descriptor;

#define USB_PORT_FEATURE_CONNECTION     0
#define USB_PORT_FEATURE_ENABLE         1
#define USB_PORT_FEATURE_SUSPEND        2
#define USB_PORT_FEATURE_OVER_CURRENT   3
#define USB_PORT_FEATURE_RESET          4
#define USB_PORT_FEATURE_POWER          8
#define USB_PORT_FEATURE_LOW_SPEED      9
#define USB_PORT_FEATURE_C_CONNECTION   16
#define USB_PORT_FEATURE_C_ENABLE       17
#define USB_PORT_FEATURE_C_PORT_SUSPEND 18
#define USB_PORT_FEATURE_C_PORT_OVER_CURRENT 19
#define USB_PORT_FEATURE_C_RESET        20

typedef struct {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bNbrPorts;
  uint16_t wHubCharacteristics;
  uint8_t bPwrOn2PwrGood;
  uint8_t bHubContrCurrent;
  // technically this is an array but in practice
  // nothing supports more than 7 ports
  uint8_t DeviceRemovable;
} __attribute__((packed)) usb_hub_descriptor;

#endif // _USB_SPEC_H

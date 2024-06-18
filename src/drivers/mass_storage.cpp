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

#include "mass_storage.h"

#include <Arduino.h>  // millis() / delay()
#include <cstring>
#include <cerrno>

/* NOTE: USB (CBW/CSW) uses little-endian multi-byes values.
 * SCSI (CDBs) uses big-endian multi-byte values.
 * Have fun with that!
 */

#define CBW_SIGNATURE 0x43425355   // 'USBC'
#define CSW_SIGNATURE 0x53425355   // 'USBS'
#define CBW_OUT   USB_CTRLTYPE_DIR_HOST2DEVICE
#define CBW_IN    USB_CTRLTYPE_DIR_DEVICE2HOST

#define SCSI_TEST_UNIT_READY                 0x00
#define SCSI_REQUEST_SENSE                   0x03
#define SCSI_INQUIRY                         0x12
#define SCSI_START_STOP_UNIT                 0x1B
#define SCSI_READ_CAPACITY_10                0x25
#define SCSI_READ_10                         0x28
#define SCSI_WRITE_10                        0x2A
#define SCSI_READ_16                         0x88
#define SCSI_WRITE_16                        0x8A
#define SCSI_SERVICE_16                      0x9E

#define SCSI_SERVICE_ACTION_READ_CAPACITY    0x10

#define SCSI_SENSE_RECOVERED                 0x01
#define SCSI_SENSE_NOT_READY                 0x02
#define SCSI_SENSE_MEDIUM_ERROR              0x03
#define SCSI_SENSE_HARDWARE_ERROR            0x04
#define SCSI_SENSE_UNIT_ATTENTION            0x06

#define USBMS_REQ_GET_MAX_LUN  254
#define USBMS_REQ_RESET        255

std::list<USB_Storage*> USB_Storage::devices;
USB_Storage::mutex_cxx USB_Storage::list_lock;

void USB_Storage::deref(void) {
  if (ref.fetch_sub(1, std::memory_order_release) == 1) {
    delete this;
  }
}

void USB_Storage::detach(void) {
  {
    auto lock = list_lock.autolock();
    devices.remove(this);
  }

  lun_count = 0;
  deref();
}

USB_Storage* USB_Storage::open(void) {
  if (get_lun_count() > 0) {
    addref();
    return this;
  }
  return NULL;
}

USB_Storage::USB_Storage(USB_Device *d, uint8_t iface, uint8_t ep_in, uint8_t ep_out) :
USB_Driver_FactoryGlue<USB_Storage>(d), interface(iface), bulk_in(ep_in), bulk_out(ep_out) {
  const uint8_t bmrtMaxLun = USB_CTRLTYPE_DIR_DEVICE2HOST|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_INTERFACE;

  dprintf("Created new USB_Storage %p, interface %d, bulk_in %02X, bulk_out %02X\n", this, interface, bulk_in, bulk_out);
  ref = 1;
  lun_count = 0;
  tag = millis()*0x01011111; // random-ish

  xfer.buf[0] = 255;
  ControlMessage(bmrtMaxLun, USBMS_REQ_GET_MAX_LUN, 0, interface, 1, xfer.buf, [this](int result) {
    // get_lun is allowed to fail, but do not accept the device being unplugged
    if (result != -ENODEV) {
      if (result >= 0 && xfer.buf[0] <= 15)
        lun_count = xfer.buf[0] + 1;
      else
        lun_count = 1;

      auto lock = list_lock.autolock();
      devices.push_back(this);
    }
  });
}

USB_Storage::~USB_Storage() {
  dprintf("USB_Storage<%p> was destroyed\n", this);
}

bool USB_Storage::offer_interface(const usb_interface_descriptor* id, size_t) {
  if (id->bNumEndpoints < 2) return false;
  if (id->bInterfaceClass != 8) return false;
  if (id->bInterfaceSubClass != 6) return false;
  if (id->bInterfaceProtocol != 80) return false;
  return true;
}

USB_Driver* USB_Storage::attach_interface(const usb_interface_descriptor* id, size_t s, USB_Device *dev) {
  uint8_t ep_out = 0, ep_in = 0;
  const uint8_t *p = (const uint8_t*)id;
  const uint8_t *desc_end = p + s;
  p += id->bLength;
  do {
    if (p[1] == USB_DT_ENDPOINT) {
      usb_endpoint_descriptor *ep = (usb_endpoint_descriptor*)p;
      if (ep->bmAttributes == USB_ENDPOINT_BULK) {
        if (ep->bEndpointAddress & 0x80) {
          if (ep_in == 0)
            ep_in = ep->bEndpointAddress;
        } else if (ep_out == 0) {
          ep_out = ep->bEndpointAddress;
        }
        if (ep_in != 0 && ep_out != 0)
          return new(std::nothrow) USB_Storage(dev, id->bInterfaceNumber, ep_in, ep_out);
      }
    }
    p += p[0];
  } while (p < desc_end);
  return NULL;
}

USB_Storage* USB_Storage::open_device(size_t index) {
  auto lock = list_lock.autolock();
  for (auto it = devices.begin(); it != devices.end(); it++) {
    if (index == 0) {
      USB_Storage *ms = *it;
      ms->addref();
      return ms;
    }
    --index;
  }
  return NULL;
}

void USB_Storage::reset(void) {
  const uint8_t bmrtReset = USB_CTRLTYPE_DIR_HOST2DEVICE|USB_CTRLTYPE_TYPE_CLASS|USB_CTRLTYPE_REC_INTERFACE;

  // bulk storage reset
  ControlMessage(bmrtReset, USBMS_REQ_RESET, 0, interface, 0, NULL);
  delay(100);
  // clear HALT on IN endpoint
  ControlMessage(USB_REQTYPE_ENDPOINT_SET, USB_REQ_CLEAR_FEATURE, USB_FEATURE_ENDPOINT_HALT, bulk_in, 0, NULL);
  delay(100);
  // clear HALT on OUT endpoint
  ControlMessage(USB_REQTYPE_ENDPOINT_SET, USB_REQ_CLEAR_FEATURE, USB_FEATURE_ENDPOINT_HALT, bulk_out, 0, NULL);
  delay(100);
}

int USB_Storage::scsi_cmd(uint8_t lun, void* data, size_t length, const uint8_t* command, bool write) {
  auto lck = cmd_lock.autolock();
  if (!lck) {
    errno = ETIMEDOUT;
    return -1;
  }
  size_t commandLength;
  int csw_tofetch = sizeof(ms_csw);

  if (lun >= lun_count) {
    errno = ENXIO;
    return -1;
  }

  switch (command[0] >> 5) {
    case 0:
      commandLength = 6;
      break;
    case 1:
    case 2:
      commandLength = 10;
      break;
    case 4:
      commandLength = 16;
      break;
    case 5:
      commandLength = 12;
      break;
    default:
      errno = EINVAL;
      return -1;
  }

  // send CBW
  uint32_t t = tag++;
  xfer.cbw.dCBWSignature = CBW_SIGNATURE;
  xfer.cbw.dCBWTag = t;
  xfer.cbw.dCBWDataTransferLength = length;
  xfer.cbw.bmCBWFlags = write ? CBW_OUT : CBW_IN;
  xfer.cbw.bCBWLUN = lun;
  xfer.cbw.bCBWCBLength = commandLength;
  memcpy(xfer.cbw.CBWCB, command, commandLength);

  int ret = BulkMessage(bulk_out, sizeof(xfer.cbw), &xfer.cbw);
  if (ret < (int)sizeof(ms_cbw)) {
    if (ret >= 0)
      errno = EPROTO;
    return -1;
  }

  // send/receive data (if any)
  size_t transferred = 0;
  while (transferred < length) {
    uint8_t *p = (uint8_t*)data + transferred;
    uint32_t to_send;
    if ((length - transferred) <= 65536)
      to_send = (uint32_t)(length - transferred);
    else
      to_send = 65536;

    ret = BulkMessage(write ? bulk_out : bulk_in, to_send, p);
    if (ret < 0)
      return -1;

    if (ret < (int)to_send) {
      dprintf("scsi_cmd: short data transfer %d vs %u\n", ret, to_send);
      // the device may have skipped all the data and sent the CSW instead...
      if (write == false && ret >= 4 && ret <= csw_tofetch && *(uint32_t*)p == CSW_SIGNATURE) {
        dprintf("Got CSW instead of data\n");
        memcpy(xfer.buf, p, ret);
        csw_tofetch -= ret;
      }
      else
        transferred += ret;
      break;
    }
    transferred += to_send;
  }

  // receive CSW
  if (csw_tofetch) {
    ret = BulkMessage(bulk_in, csw_tofetch, xfer.buf+sizeof(ms_csw)-csw_tofetch);
    if (ret<0 && errno==EPIPE) {
      // endpoint stalled, probably the device letting us know the data stage returned less than expected
      ControlMessage(USB_REQTYPE_ENDPOINT_SET, USB_REQ_CLEAR_FEATURE, USB_FEATURE_ENDPOINT_HALT, bulk_in, 0, NULL);
      delay(100);
      ret = BulkMessage(bulk_in, csw_tofetch, xfer.buf+sizeof(ms_csw)-csw_tofetch);
    }
    if (ret < csw_tofetch) {
      if (ret >= 0)
        errno = EPROTO;
      return -1;
    }
  }

  if (xfer.csw.dCSWSignature == CSW_SIGNATURE && xfer.csw.dCSWTag == t) {
    if (xfer.csw.bCSWStatus == 0) { // success
      uint32_t residue = xfer.csw.dCSWDataResidue;
      if (residue != (length - transferred)) {
        dprintf("SCSI: Residue mismatch, expected %u but received %u\n", length - transferred, residue);
      }
      else if (residue)
        dprintf("SCSI RESIDUE: %u\n", residue);
      return (int)transferred;
    }
    if (xfer.csw.bCSWStatus != 1) // phase error
      reset();

    errno = EBUSY;
  }
  else
    errno = EPROTO;
  return -1;
}

int USB_Storage::request_sense(uint8_t lun, void* sense_data, uint8_t length, bool desc) {
  const uint8_t cmd[6] = {SCSI_REQUEST_SENSE, (uint8_t)(desc ? 1:0), 0, 0, length};
  return scsi_cmd(lun, sense_data, length, cmd, false);
}

int USB_Storage::test_unit_ready(uint8_t lun) {
  const uint8_t cmd[6] = {SCSI_TEST_UNIT_READY};
  return scsi_cmd(lun, NULL, 0, cmd, false);
}

int USB_Storage::lun_ready(uint8_t lun, uint64_t& sector_count, uint32_t& sector_size) {
  basic_inquiry_response inq;

  sector_count = 0;
  sector_size = 0;

  if (inquiry(lun, &inq, sizeof(inq)) >= (int)sizeof(inq)) {
    if (inq.peripheral_qualifier != 0 || inq.peripheral_device_type == 0x31) {
      // peripheral_qualifier == 1: device is not connected e.g. USB->IDE adapter with no drive connected
      // peripheral_qualifier > 1: reserved, not supported or vendor specific
      // peripheral_device_type == 31: unknown or no device type
      errno = ENOTSUP;
      return -1;
    }
    // device is good (enough) - test if its ready
    if (test_unit_ready(lun) >= 0) {
      if (read_capacity(lun, sector_count, sector_size) == true)
        return 0;
    }
  }

  if (errno == EBUSY) {
    // SCSI command reported an error, request sense data otherwise device may continuously fail forever
    basic_sense_data sense;
    if (request_sense(lun,  &sense, sizeof(sense)) > (int)offsetof(basic_sense_data, additional_sense_code_qualifier)) {
      if (sense.response_code == 0x70) { // sense response is for current command
        switch(sense.sense_key) {
          // these can be immediately retried
          case SCSI_SENSE_RECOVERED:
          case SCSI_SENSE_UNIT_ATTENTION:
            errno = EAGAIN;
            break;
          case SCSI_SENSE_NOT_READY:
            if (sense.additional_sense_code == 0x3A) // drive is empty
              return 0;
            if (sense.additional_sense_code == 4) { // logical unit not ready
              switch (sense.additional_sense_code_qualifier) {
                case 2: // START UNIT required
                  {
                    const uint8_t cmd[6] = {SCSI_START_STOP_UNIT, 0, 0, 0, 1};
                    scsi_cmd(lun, NULL, 0, cmd, false);
                  }
                case 0: // not reportable / non-specific - treat this as retry
                case 1: // becoming ready
                case 9: // self-test in progress
                  errno = EAGAIN;
                // treat anything else as incompatible
              }
            }
            break;
        }
      }
    }
  }
  return -1;
}

bool USB_Storage::read_capacity(uint8_t lun, uint64_t& block_count, uint32_t& block_size) {
  block_count = 0;
  block_size = 0;
  uint8_t cmd[16] = {SCSI_READ_CAPACITY_10};
  uint32_t buf[2];

  if (scsi_cmd(lun, buf, sizeof(buf), cmd, false) < 8)
    return false;

  // if device has MAX_UINT32 LBAs, attempt READ_CAPACITY_16
  if (buf[0] == 0xFFFFFFFF) {
    struct {
      uint64_t lba;
      uint32_t size;
      uint8_t pad[20];
    } buf16;
    cmd[0] = SCSI_SERVICE_16;
    cmd[1] = SCSI_SERVICE_ACTION_READ_CAPACITY;
    cmd[13] = sizeof(buf16);
    if (scsi_cmd(lun, &buf16, cmd[13], cmd, false) >= 12) {
      block_count = 1 + __builtin_bswap64(buf16.lba);
      block_size = __builtin_bswap32(buf16.size);
      return true;
    }
  }

  block_count = 1 + __builtin_bswap32(buf[0]);
  block_size = __builtin_bswap32(buf[1]);
  return true;
}

int USB_Storage::inquiry(uint8_t lun, void *inq_data, uint16_t length) {
  uint8_t cmd[6] = {SCSI_INQUIRY, 0, 0, (uint8_t)(length>>8), (uint8_t)(length & 0xFF)};
  return scsi_cmd(lun, inq_data, length, cmd, false);
}

const char* USB_Storage::vendor_name(uint8_t lun) {
  basic_inquiry_response inq;

  if (inquiry(lun, &inq, sizeof(inq)) >= (int)sizeof(inq)) {
    xfer.buf[8] = '\0';
    unsigned int i;
    for (i=8; i>0; i--) {
      if (inq.T10_vendor[i-1] != ' ')
        break;
      xfer.buf[i-1] = '\0';
    }
    memcpy(xfer.buf, inq.T10_vendor, i);
    return (char*)xfer.buf;
  }

  return "";
}

const char* USB_Storage::product_name(uint8_t lun) {
  basic_inquiry_response inq;

  if (inquiry(lun, &inq, sizeof(inq)) >= (int)sizeof(inq)) {
    xfer.buf[16] = '\0';
    unsigned int i;
    for (i=16; i>0; i--) {
      if (inq.product_id[i-1] != ' ')
        break;
      xfer.buf[i-1] = '\0';
    }
    memcpy(xfer.buf, inq.product_id, i);
    return (char*)xfer.buf;
  }

  return "";
}


int USB_Storage::read(uint8_t lun, uint64_t lba, uint32_t count, void* data, size_t length) {
  uint8_t cmd[16] = {0};
  if (lba <= 0xFFFFFFFFu && count <= 0xFFFF) {
    cmd[0] = SCSI_READ_10;
    cmd[2] = (uint8_t)((lba >> 24) & 0xFF);
    cmd[3] = (uint8_t)((lba >> 16) & 0xFF);
    cmd[4] = (uint8_t)((lba >>  8) & 0xFF);
    cmd[5] = (uint8_t)((lba >>  0) & 0xFF);
    cmd[7] = (uint8_t)((count >> 8) & 0xFF);
    cmd[8] = (uint8_t)((count >> 0) & 0xFF);
  } else {
    cmd[0] = SCSI_READ_16;
    cmd[2] = (uint8_t)((lba >> 56) & 0xFF);
    cmd[3] = (uint8_t)((lba >> 48) & 0xFF);
    cmd[4] = (uint8_t)((lba >> 40) & 0xFF);
    cmd[5] = (uint8_t)((lba >> 32) & 0xFF);
    cmd[6] = (uint8_t)((lba >> 24) & 0xFF);
    cmd[7] = (uint8_t)((lba >> 16) & 0xFF);
    cmd[8] = (uint8_t)((lba >>  8) & 0xFF);
    cmd[9] = (uint8_t)((lba >>  0) & 0xFF);
    cmd[10] = (uint8_t)((count >> 24) & 0xFF);
    cmd[11] = (uint8_t)((count >> 16) & 0xFF);
    cmd[12] = (uint8_t)((count >>  8) & 0xFF);
    cmd[13] = (uint8_t)((count >>  0) & 0xFF);
  }

  return scsi_cmd(lun, data, length, cmd, false);
}

int USB_Storage::write(uint8_t lun, uint64_t lba, uint32_t count, const void* data, size_t length) {
  uint8_t cmd[16] = {0};
  if (lba <= 0xFFFFFFFFu && count <= 0xFFFF) {
    cmd[0] = SCSI_WRITE_10;
    cmd[2] = (uint8_t)((lba >> 24) & 0xFF);
    cmd[3] = (uint8_t)((lba >> 16) & 0xFF);
    cmd[4] = (uint8_t)((lba >>  8) & 0xFF);
    cmd[5] = (uint8_t)((lba >>  0) & 0xFF);
    cmd[7] = (uint8_t)((count >> 8) & 0xFF);
    cmd[8] = (uint8_t)((count >> 0) & 0xFF);
  } else {
    cmd[0] = SCSI_WRITE_16;
    cmd[2] = (uint8_t)((lba >> 56) & 0xFF);
    cmd[3] = (uint8_t)((lba >> 48) & 0xFF);
    cmd[4] = (uint8_t)((lba >> 40) & 0xFF);
    cmd[5] = (uint8_t)((lba >> 32) & 0xFF);
    cmd[6] = (uint8_t)((lba >> 24) & 0xFF);
    cmd[7] = (uint8_t)((lba >> 16) & 0xFF);
    cmd[8] = (uint8_t)((lba >>  8) & 0xFF);
    cmd[9] = (uint8_t)((lba >>  0) & 0xFF);
    cmd[10] = (uint8_t)((count >> 24) & 0xFF);
    cmd[11] = (uint8_t)((count >> 16) & 0xFF);
    cmd[12] = (uint8_t)((count >>  8) & 0xFF);
    cmd[13] = (uint8_t)((count >>  0) & 0xFF);
  }

  return scsi_cmd(lun, (void*)data, length, cmd, true);
}

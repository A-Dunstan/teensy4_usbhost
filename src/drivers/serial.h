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

#ifndef _USB_SERIAL_H
#define _USB_SERIAL_H

#include "../teensy4_usbhost.h"
#include <HardwareSerial.h>
#include <EventResponder.h>
#include <vector>

class usbserial_base : public USB_Driver, public HardwareSerial {
  class circ_buf {
  private:
    std::vector<uint8_t> buf;

    size_t head = 0;
    size_t tail = 0;
    size_t avail = 0; // space available for writing
  public:
    circ_buf() = default;
    size_t available(void) const {return buf.size()-avail;}
    size_t availableForWrite(void) const {return avail;}
    size_t read(uint8_t *dst, size_t len);
    size_t write(const uint8_t *src, size_t len);
    uint8_t peek(void) const {return buf[head]; }
    void set_size(size_t length);
  };

  circ_buf read_buf;
  circ_buf write_buf;

  struct aligned_buf_deleter {
    void operator()(uint8_t* p) const {
      if (p) operator delete[] (p, std::align_val_t(CACHE_LINE_SIZE));
    }
  };

  std::unique_ptr<uint8_t[], aligned_buf_deleter> usb_read_buffer;
  std::unique_ptr<uint8_t[], aligned_buf_deleter> usb_write_buffer;

  TAtomQueue<uint8_t*,2> tx_queue;
  TAtomQueue<uint8_t*,3> rx_queue;
  size_t rx_avail = 0;

  EventResponder flush_trigger;
  MillisTimer flush_timer;
  static void send_timer_expired(EventResponderRef);

  void rx_update(size_t add);
  void flush(int force);

protected:
  uint8_t ep_out;
  uint8_t ep_in;
  size_t ep_out_len;
  size_t ep_in_len;

  enum {
    STATE_CAN_RECEIVE = 1,
    STATE_CAN_TRANSMIT = 2,
    STATE_STARTED = 4,
  };
  uint8_t state = 0;

  AtomMutex rx_lock;
  AtomMutex tx_lock;

  // maintained by derived class: can buffer up to this amount of data in write_buf before calling usb_write
  size_t tx_wmark = 1;

  // len_read and len_write set the sizes used for the circular buffers
  usbserial_base();
  void start(void);

public:
  int available(void) override;
  int peek(void) override;
  int read(void) override;
  int availableForWrite(void) override;
  void flush() override { flush(1); }
  size_t write(uint8_t) override;
  void end() override;
};

#endif // _USB_SERIAL_H

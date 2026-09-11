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
#include "serial.h"
#include <cstdlib>

size_t usbserial_base::circ_buf::read(uint8_t *dst, size_t len) {
  len = std::min(len, available());
  size_t to_copy = len;
  if ((head + to_copy) >= buf.size()) {
    to_copy = buf.size() - head;
    memcpy(dst, &buf[head], to_copy);
    dst += to_copy;
    head = 0;
    if (to_copy == len)
      goto end;
    to_copy = len - to_copy;
  }
  memcpy(dst, &buf[head], to_copy);
  head += to_copy;
end:
  avail += len;
  return len;
}

size_t usbserial_base::circ_buf::write(const uint8_t *src, size_t len) {
  len = std::min(len, availableForWrite());
  size_t to_copy = len;
  if ((tail + to_copy) >= buf.size()) {
    to_copy = buf.size() - tail;
    memcpy(&buf[tail], src, to_copy);
    src += to_copy;
    tail = 0;
    if (to_copy == len)
      goto end;
    to_copy = len - to_copy;
  }
  memcpy(&buf[tail], src, to_copy);
  tail += to_copy;
end:
  avail -= len;
  return len;
}

void usbserial_base::circ_buf::set_size(size_t length) {
  buf.resize(length);
  head = tail = 0;
  avail = length;
}

void usbserial_base::send_timer_expired(EventResponder& e) {
  auto p = (usbserial_base*)e.getContext();
  p->flush();
}

usbserial_base::usbserial_base() {
  flush_trigger.setContext(this);
  flush_trigger.attach(send_timer_expired);
}

void usbserial_base::rx_update(size_t add) {
  uint8_t* dst;
  auto lock = rx_lock.Lock();
  rx_avail += add;
  while (rx_avail >= ep_in_len && rx_queue.Get(dst, -1) == ATOM_OK) {
    size_t length = std::min(rx_avail, ep_in_len*2) & -ep_in_len;
    int ret = BulkMessage(ep_in, length, dst, [=](int r) {
      if (r > 0) {
        auto lock = rx_lock.Lock();
        read_buf.write(dst, (size_t)r);
      }
      rx_queue.Put(dst);
      if (r >= 0)
        rx_update(length-r);
    });
    if (ret < 0) {
      rx_queue.Put(dst);
      return;
    }
    rx_avail -= length;
  }
}

void usbserial_base::start() {
  if (state & STATE_STARTED) return;

  auto txlock = tx_lock.Lock();
  auto rxlock = rx_lock.Lock();
  usb_read_buffer.reset( new(std::align_val_t(CACHE_LINE_SIZE), std::nothrow) uint8_t[(ep_in_len  * 2 * rx_queue.MAX_MSG + CACHE_LINE_SIZE - 1) & -CACHE_LINE_SIZE]);
  usb_write_buffer.reset(new(std::align_val_t(CACHE_LINE_SIZE), std::nothrow) uint8_t[(ep_out_len * 2 * tx_queue.MAX_MSG + CACHE_LINE_SIZE - 1) & -CACHE_LINE_SIZE]);
  uint8_t* p;
  // flush queues
  while (rx_queue.Get(p, -1) == ATOM_OK);
  while (tx_queue.Get(p, -1) == ATOM_OK);
  for (size_t i=0; i < rx_queue.MAX_MSG; i++) {
    rx_queue.Put(&usb_read_buffer[i * ep_in_len * 2]);
  }
  for (size_t i=0; i < tx_queue.MAX_MSG; i++) {
    tx_queue.Put(&usb_write_buffer[i * ep_out_len * 2]);
  }

  read_buf.set_size(ep_in_len*2*(rx_queue.MAX_MSG-1));
  write_buf.set_size(ep_out_len*2*(tx_queue.MAX_MSG-1));
  rx_avail = read_buf.availableForWrite();
  state |= STATE_STARTED|STATE_CAN_TRANSMIT|STATE_CAN_RECEIVE;
  rx_update(0);
}

void usbserial_base::end(void) {
  auto txlock = tx_lock.Lock();
  auto rxlock = rx_lock.Lock();
  flush_timer.end();
  state = 0;
  usb_read_buffer.reset(nullptr);
  usb_write_buffer.reset(nullptr);
}

int usbserial_base::available(void) {
  int ret = 0;
  auto lock = rx_lock.Lock(1);
  if (lock) {
    ret = (int)read_buf.available();
  }
  return ret;
}

int usbserial_base::peek(void) {
  int ret = -1;
  auto lock = rx_lock.Lock(1);
  if (lock) {
    if (available() > 0) {
      ret = read_buf.peek();
    }
  }
  return ret;
}

int usbserial_base::read(void) {
  int ret = -1;
  auto lock = rx_lock.Lock(1);
  if (lock) {
    uint8_t c;
    if (read_buf.read(&c, 1)) {
      ret = c;
      rx_update(1);
    }
  }
  return ret;
}

int usbserial_base::availableForWrite(void) {
  int ret = 0;
  auto lock = tx_lock.Lock(1);
  if (lock && state & STATE_CAN_TRANSMIT) {
      ret = (int)write_buf.availableForWrite();
  }
  return ret;
}

void usbserial_base::flush(int force) {
  auto lock = tx_lock.Lock(10);
  if ((state & STATE_STARTED) == 0) return;
  if ((state & STATE_CAN_TRANSMIT) && (force || write_buf.available() >= tx_wmark)) {
    flush_timer.end();

    while (write_buf.available()) {
      uint8_t* dst;
      if (tx_queue.Get(dst, 5) == ATOM_OK) {
        size_t len = write_buf.read(dst, ep_out_len*2);
        if (BulkMessage(ep_out, len, dst, [=](int r) { tx_queue.Put(dst); }) >= 0)
          continue;
      }
      break;
    }
  }

  // schedule any remaining data for later
  if (write_buf.available())
    flush_timer.begin(1, flush_trigger);
}

size_t usbserial_base::write(uint8_t c) {
  auto lock = tx_lock.Lock();
  if (lock) {
    if (state & STATE_STARTED) {
      while (availableForWrite()==0)
        yield();
      write_buf.write(&c, 1);
      flush(0);
      return 1;
    }
  }
  return 0;
}


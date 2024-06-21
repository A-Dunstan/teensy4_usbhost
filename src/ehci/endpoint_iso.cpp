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

#include "endpoint_iso.h"
#include "config.h"
#include "portab.h"
#include "log.h"
#include <cstdlib>
#include <cstring>

static void cache_invalidate(itd_transfer* t) {
  usb_iTD_t& itd = static_cast<usb_iTD_t&>(*t);
  cache_invalidate(&itd, sizeof(itd), alignof(itd));
}

static void cache_flush(const itd_transfer* t) {
  const usb_iTD_t& itd = static_cast<const usb_iTD_t&>(*t);
  cache_flush(&itd, sizeof(itd), alignof(itd));
}

template <class CTransfer>
class iso_transfer : public CCallback<CTransfer> {
private:
  const USBCallback *cb;
  int16_t* lengths;
  uint8_t* buffer;

  int16_t* last_length;
  int total_length;

  void callback(const CTransfer *t,int r) {
//    dprintf("T%d %p:%d %d\n", lengths-last_length, t, t->frame, r);
    if (r >= 0) {
      total_length += r;
      if (buffer) {
        cache_invalidate(buffer, r);
      }
    }

    if (buffer) buffer += *lengths;
    *lengths = (int16_t)r;

    if (++lengths >= last_length) {
      (*cb)(r < 0 ? r : total_length);
      delete(this);
    }
  }
public:
  void add_count(int amount=1) { last_length += amount; }
  size_t get_count(void) const { return last_length - lengths; }

  iso_transfer(const USBCallback *_cb, int16_t *l, uint8_t* _buffer) :
  cb(_cb),
  lengths(l),
  buffer(_buffer) {
    last_length = l;
    total_length = 0;
  }

  static void* operator new(size_t s, unsigned int t_count, CTransfer* &transfers) throw() {
    // siTD = 32 byte alignment
    // iTD = 64 byte alignment, to ensure they don't straddle a 4096 byte boundary
    size_t alignment = alignof(CTransfer);
    s = (s + alignment-1) & ~(alignment-1);
    uint8_t *p = (uint8_t*)aligned_alloc(alignment, s + t_count*sizeof(CTransfer));
    if (p != NULL)
      transfers = new (p+s) CTransfer[t_count];
    return p;
  }
};

template <class Transfer>
int USB_ISO_Endpoint<Transfer>::Schedule(Transfer *t) {
  uint32_t uframe_now = current_uframe();
  uint32_t frame_max = (uframe_now + PERIODIC_LIST_SIZE*8) >> 3;
  uint32_t frame_min = (uframe_now + 9) >> 3;
  uint32_t frame_cur;

  // subtract one interval in case there is an active back pointer (siTD only)
  // if (interval < PERIODIC_LIST_SIZE*8) frame_max -= (interval+7)>>3;

  for (frame_cur = frame_min; frame_cur < frame_max; frame_cur++) {
    if (frames[frame_cur % PERIODIC_LIST_SIZE] == false)
      break;
  }

  // round down to nearest interval and apply offset
  t->frame = ((frame_cur*8 & (~interval + 1)) + offset) >> 3;
  // may need correction applied depending on offset amount
  if (t->frame < frame_cur)
    t->frame = (t->frame*8 + interval) >> 3;

//  dprintf("ISO transfer<%p>: uframe_now %lu:%lu, scheduled %lu, frame_min %lu, frame_max %lu\n", t, uframe_now>>3, uframe_now&7, t->frame%PERIODIC_LIST_SIZE, frame_min, frame_max);

  // did we find a suitable time-slot?
  if (frame_min <= t->frame && t->frame < frame_max && frames[t->frame % PERIODIC_LIST_SIZE]==false) {
    t->frame = t->frame % PERIODIC_LIST_SIZE;

    if (add_node(t->frame, t->link_to(), 0xFFFFFFFF)) {
      Transfer** p = &pending;
      while (*p) p = &(*p)->next;
      *p = t;
      frames[t->frame] = true;
      return 0;
    }
    errno = ENOBUFS;
  }
  else errno = EBUSY;
  dprintf("Failed to add node: uframe_now %lu:%lu, scheduled %lu, frame_min %lu, frame_max %lu, frame_cur %lu\n", uframe_now>>3, uframe_now&7, t->frame, frame_min, frame_max, frame_cur);
  if (pending) {
    dprintf("PENDING:");
    Transfer *p = pending;
    while (p) {
      dprintf(" %lu", p->frame);
      p = p->next;
    }
    dprintf("\n");
  }
  return -1;
}

template <class Transfer>
void USB_ISO_Endpoint<Transfer>::update(void) {
  while (pending) {
    Transfer *t = pending;

    cache_invalidate(t);
    if (t->ready() == false) return;

    pending = t->next;
    remove_node(t->frame, t->link_to(), t->horizontal_link);
    frames[t->frame] = false;

    t->complete();
  }
}

template <class Transfer>
void USB_ISO_Endpoint<Transfer>::new_offset(void) {
  s_mask <<= (offset & 7);
  c_mask <<= (offset & 7);
}

template <class Transfer>
bool USB_ISO_Endpoint<Transfer>::set_inactive(void) {
  Transfer **p = &pending;
  while (*p) {
    Transfer *t = *p;
    if (remove_node(t->frame, t->link_to(), t->horizontal_link)) {
      frames[t->frame] = false;
      *p = t->next;
      t->inactivate();
    } else {
      p = &t->next;
    }
  }

  return pending != NULL;
}

USB_ISO_Full_Endpoint::USB_ISO_Full_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint32_t i, PeriodicScheduler& scheduler) :
USB_ISO_Endpoint(endpoint & 0x80, (max_packet_size > 1023) ? 1023 : max_packet_size, scheduler) {
  if (i > 16) i = 16;
  i = 1 << (i - 1);

  if (i > PERIODIC_LIST_SIZE)
    i = PERIODIC_LIST_SIZE;
  interval = i << 3;

  // worse case bit stuffing = 7/6ths. Includes CRC16 in bitstuffing plus packet structure
  uint32_t maxlen = (((wMaxPacketSize+2) * 298) >> 8) + 5+12;
  uint32_t num_transfers = (wMaxPacketSize*2 + 374) / 375;
  maxlen = (19 + 17 + maxlen + 31) >> 5;
  // SSPLIT and CSPLIT use 4 byte tokens, one byte larger than IN/OUT
  // Full/Low IN/OUT are 3 bytes smaller than High (16 vs 9)

  /* See usb_20.pdf page 375 onwards for start/complete split scheduling
   * restrictions
   */
  if (dir_in) {
    stime = (20 + 16 + 31) >> 5;              // S-SPLIT -> Full/Low IN
    ctime = maxlen;
    s_mask = 1;
    if (num_transfers > 4) num_transfers = 4;
    c_mask = (16 << num_transfers) - 4;
  } else {
    stime = maxlen;
    s_mask = (1 << num_transfers) - 1;
    c_mask = 0;
  }

  usb_siTD_t::const_sitd s = {0};

  s.address = address;
  s.endpt = endpoint;
  s.hub = hub_addr;
  s.port = port;
  s.dir = dir_in ? 1:0;
  state = s.val;
}

int USB_ISO_Full_Endpoint::IsochronousTransfer(isolength& Lengths, void *buffer, const USBCallback* cb) {
  iso_transfer<sitd_transfer> *sg;
  sitd_transfer* t;
  unsigned int t_count;
  uint8_t* dst = (uint8_t*)buffer;

  for (t_count = 0; t_count < 8; t_count++) {
    if (Lengths[t_count]==0) break;
  }

  sg = new(t_count, t) iso_transfer<sitd_transfer>(cb, Lengths, dir_in ? dst:NULL);
  if (sg == NULL) errno = ENOMEM;
  else {
    int16_t *pLength = Lengths;
    do {
      int16_t length = *pLength;

      memset(t, 0, sizeof(usb_siTD_t));
      t->horizontal_link = 1;
      t->state.val = state;
      t->s_mask = s_mask;
      t->c_mask = c_mask;
      t->status = 0x80;
      t->total = length;
      t->IOC = 1;

      t->page0 = dst;
      t->page1 = (((uint32_t)dst)>>12)+1;

      t->t_count = dir_in ? 1 : ((t->total+187) / 188);
      t->TP = (t->t_count==1) ? 0 : 1; // ALL or BEGIN
      t->back_pointer = 1;

      t->cb = sg;
      t->length = length;
      t->next = NULL;

      // this may take a while so do it before scheduling
      cache_flush_invalidate(dst, length);

      if (Schedule(t) < 0) {
        dprintf("Failed to schedule transfer\n");
        do {
          *pLength++ = 0;
        } while (--t_count);
        break;
      }
      sg->add_count();
      dst += length;
      ++t;
      ++pLength;
    } while (--t_count);

    if (sg->get_count() > 0)
      return 0;

    delete sg;
  }
  return -1;
}

USB_ISO_High_Endpoint::USB_ISO_High_Endpoint(uint8_t _endpoint, uint16_t _wMaxPacketSize, uint8_t _address, uint32_t i, PeriodicScheduler& scheduler) :
USB_ISO_Endpoint(_endpoint & 0x80, (_wMaxPacketSize & 0x400) ? 1024 : (_wMaxPacketSize & 0x3FF), scheduler),
mult(((_wMaxPacketSize + 2048) >> 11) & 3),
address(_address),
endpoint(_endpoint & 0x7F) {
  if (i > 16) i = 16;
  i = 1 << (i - 1);

  if (i > PERIODIC_LIST_SIZE*8)
    i = PERIODIC_LIST_SIZE*8;
  interval = i;

  uint32_t maxlen = (((wMaxPacketSize+2) * 298) >> 8) + 5+12;
  stime = mult * ((19 + maxlen + 31) >> 5);

  switch (interval) {
    case 1:
      s_mask = 0xFF;
      break;
    case 2:
      s_mask = 0x55;
      break;
    case 4:
      s_mask = 0x11;
      break;
    default:
      s_mask = 1;
  }
}

int USB_ISO_High_Endpoint::IsochronousTransfer(isolength& Lengths, void* buffer, const USBCallback* cb) {
  int transfers_per_itd = __builtin_popcount(s_mask);
  unsigned int t_count;

  // assume at least one transaction - this allows zero-length transfers
  for (t_count = 1; t_count < 8; t_count++) {
    if (Lengths[t_count] == 0)
      break;
  }

  const int16_t *pLength = Lengths;
  const int16_t *eLength = Lengths+t_count;
  t_count = (t_count + transfers_per_itd - 1) / transfers_per_itd;
  int16_t wMaxLength = mult * wMaxPacketSize;
  uint8_t *dst = (uint8_t*)buffer;
  itd_transfer* t;

  auto ig = new(t_count, t) iso_transfer<itd_transfer>(cb, Lengths, dir_in ? dst:NULL);
  if (ig == NULL) {
    dprintf("Failed to allocate high speed iso transfer\n");
    errno = ENOMEM;
  } else {
    itd_transfer *p = t-1;
    uint8_t mask = 0;
    uint32_t b;
    do {
      if (mask == 0) {
        // new iTD, initialize it
        ++p;
        memset(p, 0, sizeof(usb_iTD_t));
        p->horizontal_link = 1;
        p->address = address;
        p->endpt = endpoint;
        p->wMaxPacketSize = wMaxPacketSize;
        p->dir = dir_in ? 1:0;
        p->Mult = mult;
        b = (uint32_t)dst;
        p->page0 = (b + 0x0000) >> 12;
        p->page1 = (b + 0x1000) >> 12;
        p->page2 = (b + 0x2000) >> 12;
        p->page3 = (b + 0x3000) >> 12;
        p->page4 = (b + 0x4000) >> 12;
        p->page5 = (b + 0x5000) >> 12;
        p->page6 = (b + 0x6000) >> 12;
        b &= 0xFFF;

        p->next = NULL;
        p->cb = ig;
        p->s_mask = 0;

        mask = s_mask;
      }

      int slot = __builtin_ctz(mask);
      int16_t len = *pLength;
      int16_t actual_len = len > wMaxLength ? wMaxLength : len;

      p->transfers[slot].status = 8; // active
      p->transfers[slot].total = actual_len;
      p->transfers[slot].PG = b >> 12;
      p->transfers[slot].offset = b & 0xFFF;

      p->s_mask |= (1 << slot);
      mask &= ~(1 << slot);

      cache_flush_invalidate(dst, actual_len);
      dst += len;
      b += len;

    } while (++pLength != eLength);

    do {
      // set IOC on last transfer
      int last_slot = 31 - __builtin_clz(t->s_mask);
      t->transfers[last_slot].IOC = 1;
      cache_flush(t);
      if (Schedule(t) < 0)
        break;
      ig->add_count(__builtin_popcount(t->s_mask));
    } while (p != t++);

    if (ig->get_count())
      return 0;

    delete ig;
  }

  return -1;
}

USB_Endpoint* createIsoEndpoint(const usb_endpoint_descriptor* p, uint8_t address, uint8_t hub_addr,
uint8_t port, uint8_t speed, PeriodicScheduler& s) {
  if (speed == 0) // full speed
    return new(std::nothrow) USB_ISO_Full_Endpoint(p->bEndpointAddress, p->wMaxPacketSize, address, hub_addr, port, p->bInterval, s);
  if (speed == 2) // high speed
    return new(std::nothrow) USB_ISO_High_Endpoint(p->bEndpointAddress, p->wMaxPacketSize, address, p->bInterval, s);

  return NULL;
}

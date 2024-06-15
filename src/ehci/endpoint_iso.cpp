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

static void cache_invalidate(sitd_transfer* t) {
  usb_siTD_t& sitd = static_cast<usb_siTD_t&>(*t);
  cache_invalidate(&sitd, sizeof(sitd), alignof(sitd));
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
//    dprintf("T%d %p:%d %d\n", lengths-last_length, t, t->frame>>3, r);
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

  static void* operator new(size_t s, unsigned int t_count, CTransfer* &transfers) noexcept(true) {
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

int USB_ISO_Full_Endpoint::Transfer(sitd_transfer *t, int16_t Length, void *buffer, CCallback<sitd_transfer>* cb) {
  memset(t, 0, sizeof(usb_siTD_t));
  t->horizontal_link = 1;
  t->state.val = state;
  t->s_mask = s_mask;
  t->c_mask = c_mask;
  t->status = 0x80;
  t->total = Length;
  t->IOC = 1;

  t->page0 = buffer;
  t->page1 = (((uint32_t)buffer)>>12)+1;

  t->t_count = dir_in ? 1 : ((t->total+187) / 188);
  t->TP = (t->t_count==1) ? 0 : 1; // ALL or BEGIN
  t->back_pointer = 1;

  t->cb = cb;
  t->length = Length;
  t->next = NULL;

  // this may take a while so do it before fetching the current uframe index
  cache_flush_invalidate(buffer, Length);

  uint32_t uframe_now = current_uframe();
  // this is the cut-off point for scheduling: a full list period from now
  uint32_t uframe_max = (uframe_now + PERIODIC_LIST_SIZE*8 - interval) & ~7;
  // this is the earliest allowed: at least 1 full frame ahead
  uint32_t uframe_min = (uframe_now + 9) & ~7;
  uint32_t uframe_cur = uframe_min;
  if (pending) {
    // insert after all pending transfers
    sitd_transfer *p = pending;
    while (p->next) p = p->next;
    uint32_t f = p->frame;
    if (f < uframe_now) {
      f += PERIODIC_LIST_SIZE*8;
    }
    if (f < uframe_max) {
      f += interval;
      if (f > uframe_cur) uframe_cur = f;
    }
  }
  t->frame = (uframe_cur & (~interval + 1)) + offset;
  // 1x interval correction may be needed
  if (t->frame < uframe_cur)
    t->frame += interval;

//  dprintf("ISO transfer<%p>: uframe_now %lu:%lu, scheduled %lu:%lu, uframe_min %lu:%lu, uframe_max %lu:%lu\n", t, uframe_now>>3, uframe_now&7, t->frame>>3, t->frame&7, uframe_min>>3, uframe_min&7, uframe_max>>3, uframe_max&7);

  // did we find a suitable time-slot?
  if (uframe_min <= t->frame && t->frame < uframe_max) {
    t->frame = t->frame % (PERIODIC_LIST_SIZE*8);

    if (add_node(t->frame>>3, t->link_to(), 0xFFFFFFFF)) {
      sitd_transfer** p = &pending;
      while (*p) p = &(*p)->next;
      *p = t;
      return 0;
    }
    errno = ENOBUFS;
  }
  else errno = EBUSY;
  dprintf("Failed to add node: uframe_now %lu:%lu, scheduled %lu:%lu, uframe_min %lu:%lu, uframe_max %lu:%lu, uframe_cur %lu:%lu\n", uframe_now>>3, uframe_now&7, t->frame>>3, t->frame&7, uframe_min>>3, uframe_min&7, uframe_max>>3, uframe_max&7, uframe_cur>>3, uframe_cur&7);
  if (pending) {
    dprintf("PENDING:");
    sitd_transfer *p = pending;
    while (p) {
      dprintf(" %lu:%lu", p->frame>>3, p->frame&7);
      p = p->next;
    }
    dprintf("\n");
  }
  return -1;
}

void USB_ISO_Full_Endpoint::update(void) {
  while (pending) {
    sitd_transfer *t = pending;

    cache_invalidate(t);
    if (t->status & 0x80) return;
    if (t->status & 0x7C)
      dprintf("Possible ISO transfer error: %02X\n", t->status);
    // todo: handle errors

    pending = t->next;
    remove_node(t->frame>>3, t->link_to(), t->horizontal_link);

    int length = t->length - t->total;
    t->cb->callback(t,length);
  }
}

void USB_ISO_Full_Endpoint::new_offset(void) {
  dprintf("USB_ISO_Full_Endpoint<%p>: new_offset %d\n", this, offset);
  s_mask <<= (offset & 7);
  c_mask <<= (offset & 7);
}

bool USB_ISO_Full_Endpoint::set_inactive(void) {
  bool removed = false;
  while (pending) {
    sitd_transfer *t = pending;
    pending = t->next;
    removed |= remove_node(t->frame>>3, t->link_to(), t->horizontal_link);

    t->cb->callback(t, -ENODEV);
  }

  return removed;
}

USB_ISO_Full_Endpoint::USB_ISO_Full_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint32_t i, PeriodicScheduler& scheduler) :
USB_Periodic_Endpoint(USB_ENDPOINT_ISOCHRONOUS, scheduler),
dir_in(endpoint & 0x80) {
  wMaxPacketSize = max_packet_size & 0x7FF;
  if (wMaxPacketSize >= 1024) wMaxPacketSize = 1023;

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
      if (Transfer(t, length, dst, sg) < 0) {
        dprintf("Failed to schedule transfer\n");
        break;
      }
      sg->add_count();
      dst += length;
      t++;
    } while (--t_count);

    if (sg->get_count() > 0)
      return 0;

    delete sg;
  }
  return -1;
}

int USB_ISO_High_Endpoint::Schedule(itd_transfer *t) {
  uint32_t uframe_now = current_uframe();
  uint32_t uframe_max = (uframe_now + PERIODIC_LIST_SIZE*8 - interval) & ~7;
  uint32_t uframe_min = (uframe_now + 9) & ~7;
  uint32_t uframe_cur = uframe_min;
  if (pending) {
    itd_transfer *p = pending;
    while (p->next) p = p->next;
    uint32_t f = p->frame;
    if (f < uframe_now) {
      f += PERIODIC_LIST_SIZE*8;
    }
    if (f < uframe_max) {
      f += interval > 8 ? interval : 8;
      if (f > uframe_cur) uframe_cur = f;
    }
  }
  t->frame = (uframe_cur & (~interval + 1)) + offset;
  if (t->frame < uframe_cur)
    t->frame += interval;

//  dprintf("ISO transfer<%p>: uframe_now %lu:%lu, scheduled %lu:%lu, uframe_min %lu:%lu, uframe_max %lu:%lu\n", t, uframe_now>>3, uframe_now&7, (t->frame>>3)&31, t->frame&7, uframe_min>>3, uframe_min&7, uframe_max>>3, uframe_max&7);

  // did we find a suitable time-slot?
  if (uframe_min <= t->frame && t->frame < uframe_max) {
    t->frame = t->frame % (PERIODIC_LIST_SIZE*8);

    if (add_node(t->frame>>3, t->link_to(), 0xFFFFFFFF)) {
      itd_transfer** p = &pending;
      while (*p) p = &(*p)->next;
      *p = t;
      return 0;
    }
    errno = ENOBUFS;
  }
  else errno = EBUSY;
  dprintf("Failed to add node: uframe_now %lu:%lu, scheduled %lu:%lu, uframe_min %lu:%lu, uframe_max %lu:%lu, uframe_cur %lu:%lu\n", uframe_now>>3, uframe_now&7, t->frame>>3, t->frame&7, uframe_min>>3, uframe_min&7, uframe_max>>3, uframe_max&7, uframe_cur>>3, uframe_cur&7);
  if (pending) {
    dprintf("PENDING:");
    itd_transfer *p = pending;
    while (p) {
      dprintf(" %lu:%lu", p->frame>>3, p->frame&7);
      p = p->next;
    }
    dprintf("\n");
  }
  return -1;
}

void USB_ISO_High_Endpoint::update(void) {
  while (pending) {
    itd_transfer *t = pending;

    cache_invalidate(t);
    // process complete iTD only
    for (unsigned int i=8; i > 0; i--) {
      if (t->transfers[i-1].status & 8)
        return;
    }

    pending = t->next;
    remove_node(t->frame>>3, t->link_to(), t->horizontal_link);

    unsigned int tmask = t->s_mask;
    while (tmask) {
      unsigned int pos = __builtin_ctz(tmask);
      if (t->transfers[pos].status & 7)
        dprintf("Possible ISO transfer error: %02X\n", t->transfers[pos].status);
      t->cb->callback(t, t->transfers[pos].total);
      tmask &= ~(1 << pos);
    }
  }
}

void USB_ISO_High_Endpoint::new_offset(void) {
  s_mask <<= (offset & 7);
}

bool USB_ISO_High_Endpoint::set_inactive(void) {
  bool removed = false;
  while (pending) {
    itd_transfer *t = pending;
    pending = t->next;
    removed |= remove_node(t->frame>>3, t->link_to(), t->horizontal_link);


    int remaining = __builtin_popcount(t->s_mask);
    while (remaining-->0) {
      t->cb->callback(t, -ENODEV);
    }
  }

  return removed;
}

USB_ISO_High_Endpoint::USB_ISO_High_Endpoint(uint8_t _endpoint, uint16_t _wMaxPacketSize, uint8_t _address, uint32_t i, PeriodicScheduler& scheduler) :
USB_Periodic_Endpoint(USB_ENDPOINT_ISOCHRONOUS, scheduler),
dir_in(_endpoint & 0x80),
mult(((_wMaxPacketSize + 2048) >> 11) & 3),
wMaxPacketSize(((_wMaxPacketSize & 0x7FF) < 1024) ? (_wMaxPacketSize & 0x7FF) : 1024),
address(_address),
endpoint(_endpoint & 0x7F) {
  dprintf("New USB_ISO_High_Endpoint: %p\n", this);
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
        memset(p, 0, sizeof(*p));
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

        p->cb = ig;

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
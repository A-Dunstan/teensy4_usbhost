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

#include "endpoint_qh.h"
#include "log.h"
#include "config.h"
#include <cstring>

#define QTD_PTR_INVALID      ((usb_qTD_t*)1)
#define IS_QTD_PTR_VALID(p)  ((((uint32_t)p) & 0x1F)==0)

static void fill_qtd(usb_qTD_t& p, usb_qTD_t* next_qtd, usb_qTD_t* alt_qtd, bool dt, uint16_t total, bool ioc, uint8_t pid, const void* data) {
  p.next = next_qtd != NULL ? next_qtd : QTD_PTR_INVALID;
  p.alt = alt_qtd != NULL ? alt_qtd : QTD_PTR_INVALID;
  p.token.dt = dt ? 1:0;
  p.token.total = total;
  p.token.IOC = ioc ? 1:0;
  p.token.c_Page = 0;
  p.token.CERR = 3;
  p.token.PID = pid;
  p.token.status = 0x80;
  if (total) {
    uint32_t d = (uint32_t)data;
    p.qtd_page[0] = d;
    d &= 0xFFFFF000;
    p.qtd_page[1] = d + 0x1000;
    p.qtd_page[2] = d + 0x2000;
    p.qtd_page[3] = d + 0x3000;
    p.qtd_page[4] = d + 0x4000;
  }
}

typedef struct __attribute__((aligned(32))) : usb_qTD_t {} usb_qTD_aligned;

static void cache_flush(const usb_qTD_aligned*);
static void cache_invalidate(usb_qTD_aligned*);

class usb_transfer : public usb_qTD_aligned  {
  static void sync_chain(usb_qTD_aligned& src, usb_qTD_aligned& end) {
    usb_qTD_aligned& next = static_cast<usb_qTD_aligned&>(*src.next);
    if (&next != &end)
      sync_chain(next, end);
    if (IS_QTD_PTR_VALID(src.alt)) {
      usb_qTD_aligned& alt = static_cast<usb_qTD_aligned&>(*src.alt);
      if (&alt != &end)
        sync_chain(alt, end);
    }
    if (&src != &end)
      cache_flush(&src);
  }
public:
  CCallback<class usb_transfer>* cb = NULL;

  virtual ~usb_transfer() = default;

  void sync_chain(void) {sync_chain(*this,*this); }
};

class static_usb_transfer : public usb_transfer {
  // single unit allocation not supported
  void* operator new(size_t);
  /* single unit deallocation does nothing - should only be used as part of
   * another class or as an array
   */
  void operator delete(void*p,size_t) {}

public:
  /* GCC BUG: default new[] doesn't work properly for aligned objects with constructors/destructors
   * The compiler allocates extra space for array metadata at the beginning
   * then increments the aligned pointer forward to cover it - this misaligns the returned objects!
   */
  void* operator new[](size_t size, std::align_val_t align) {
    size_t a = (size_t)align;
    // round up to alignment size
    auto len = (size + a - 1) & -a;
    uint8_t* p = (uint8_t*)aligned_alloc(a, len);
    if (p) // increment over unused space
      p += len - size;

    return p;
  }
  void operator delete[](void* ptr, size_t, std::align_val_t align) {
    size_t a = (size_t)align;
    if (ptr == NULL) return;

    // round pointer down to alignment boundary
    size_t p = (size_t)ptr & -a;
    free((void*)p);
  }
};

class bulk_interrupt_transfer : public usb_transfer, private CCallback<usb_transfer> {
private:
   const USBCallback& cbi;
   int ilength = 0;
   std::unique_ptr<static_usb_transfer[]> more;

   void callback(const usb_transfer*,int result) override {
     if (result >= 0 && ilength) {
       result = ilength - result;
     }
     cbi(result);
   }

   bulk_interrupt_transfer(const USBCallback* _cb) : cbi(*_cb) {}

public:
  static usb_transfer* create_bulk_interrupt_transfer(bool dir_in, const usb_bulkintr_sg* sg, const USBCallback* cb) {
    bulk_interrupt_transfer* t = new(std::nothrow) bulk_interrupt_transfer(cb);
    if (t) {
      uint8_t PID = dir_in ? 1:0;

      size_t tcount = 0;
      usb_transfer *last;

      do {
        auto l = sg[tcount].wLength;
        if (l) {
          t->ilength += l;
          cache_flush_invalidate(sg[tcount].data, l);
        }
      } while (sg[++tcount].data != NULL);

      if (--tcount) {
        static_usb_transfer* more = new static_usb_transfer[tcount];
        if (more == NULL) {
          delete t;
          return NULL;
        }
        t->more.reset(more);
        last = &more[tcount - 1];

        // first transfer
        fill_qtd(*t, more, t, true, sg->wLength, false, PID, sg->data);
        ++sg;
        // middle transfer(s)
        for (auto p = more; p < last; p++) {
          fill_qtd(*p, p+1, t, true, sg->wLength, false, PID, sg->data);
          ++sg;
        }
      }
      else last = t;

      // final transfer
      fill_qtd(*last, t, t, true, sg->wLength, true, PID, sg->data);
      last->cb = t;
    }

    return t;
  }
};

class control_transfer : public usb_transfer, private CCallback<usb_transfer>, private usb_control_transfer {
private:
  static_usb_transfer data, ack;
  CCallback<usb_control_transfer>* const cb;
  bool do_release;

  void callback(const usb_transfer*, int result) override {
    if (result >= 0 && req.wLength) {
      result = req.wLength - data.token.total;
      if (req.bmRequestType & USB_CTRLTYPE_DIR_DEVICE2HOST)
        cache_invalidate(buffer, result);
    }
    cb->callback(this, result);
    if (do_release) free(buffer);
  }

public:
  control_transfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, \
  void *_buffer, CCallback<usb_control_transfer>* _cb, bool release_mem) :
  cb(_cb) {
    req.bmRequestType = bmRequestType;
    req.bmRequest = bmRequest;
    req.wValue = wValue;
    req.wIndex = wIndex;
    req.wLength = wLength;
    buffer = _buffer;
    do_release = release_mem;

    cache_flush(&req);

    ack.cb = this;
    fill_qtd(*this, &ack, NULL, false, sizeof(req), false, 2, &req);
    fill_qtd(ack, this, NULL, true, 0, true, 1, NULL);

    if (wLength) {
      next = &data;
      fill_qtd(data, &ack, NULL, true, wLength, false, bmRequestType & USB_CTRLTYPE_DIR_DEVICE2HOST ? 1:0, buffer);
      ack.token.PID = 1 - data.token.PID;
      cache_flush(buffer, wLength);
    }
  }
};

void cache_flush(const usb_qTD_aligned* qtd) {
  cache_flush(qtd, sizeof(*qtd), alignof(*qtd));
}

void cache_invalidate(usb_qTD_aligned* qtd) {
  cache_invalidate(qtd, sizeof(*qtd), alignof(*qtd));
}

void cache_flush_invalidate(usb_qTD_aligned* qtd) {
  cache_flush_invalidate(qtd, sizeof(*qtd), alignof(*qtd));
}

void QH_Base::update(void) {
  while (pending != dummy.get()) {
    int ret = 0;
    usb_transfer* t = pending;
    cache_invalidate(t);
    if (t->token.status & 0x80) // transfer is still active
      return;

//    dprintf("EP<%p> usb transfer: %p, token %08lX, cb %p, data %p\n", this, static_cast<usb_qTD_t*>(t), t->token.val, t->cb, t->qtd_page[0]);
    if (t->token.status & 0x40) {
      // there was an error, walk the list until we find the next callback
      dprintf("USB TRANSFER ERROR %p %08lX\n", t, t->token.val);
      ret = -EPIPE;
      uint32_t status = t->token.status;
      while (t->cb == NULL) {
        usb_transfer &n = static_cast<usb_transfer&>(*t->next);
        delete t;
        t = &n;
      }

      if (status & 0x20)
        ret = -EOVERFLOW;
      else if (status & 0x10)
        ret = -EPROTO;

      // unhalt the queue, resume processing
      cache_invalidate(this);
      overlay.next = t->next;
      cache_flush(this);
      overlay.token.status = 0; // clear halt, reset DT
      cache_flush_invalidate(this);
      pending = static_cast<usb_transfer*>(t->next);
    } else {
      ret = t->token.total;
      if (ret && IS_QTD_PTR_VALID(t->alt)) {
        pending = static_cast<usb_transfer*>(t->alt);
        while (t->cb == NULL) {
          usb_transfer &n = static_cast<usb_transfer&>(*t->next);
          delete t;
          ret += n.token.total;
          t = &n;
        }
      }
      else {
        pending = static_cast<usb_transfer*>(t->next);
      }
    }

    if (t->cb) {
      t->cb->callback(t, ret);
    }

    delete t;
//    dprintf("next: %p(%p)\n", pending, dummy.get());
  }
}

void QH_Base::flush(void) {
  // get rid of any pending transfers
  dprintf("Endpoint %p flush\n", this);
  cache_invalidate(this);
  overlay.alt = QTD_PTR_INVALID;
  overlay.next = dummy.get();
  overlay.token.val = 0; // reset dt to zero
  cache_flush_invalidate(this);

  usb_transfer *t = pending;
  while (t != dummy.get()) {
    cache_invalidate(t);
    usb_transfer *next = static_cast<usb_transfer*>(t->next);
    dprintf("FLUSH: %p %08lX\n", static_cast<usb_qTD_t*>(t), t->token.val);
    delete t;
    t = next;
  }
  pending = t;
}

bool QH_Base::enqueue_transfer(usb_transfer* head) {
  if (!active) {
    errno = ENODEV;
    return false;
  }
//  dprintf("Enqueue transfer<%p>: %p(%p), dummy: %p\n", this, static_cast<usb_qTD_t*>(head), head->qtd_page[0], static_cast<usb_qTD_t*>(dummy.get()));

  head->sync_chain();

  usb_transfer* p = dummy.release(); // copy dummy ptr
  dummy.reset(head);                 // head becomes new dummy
  head->token.status = 0x40;         // set inactive+halted
  *p = *head;                        // copy over dummy (becomes new head)
  cache_flush(head);
  cache_flush_invalidate(p);         // ensure new qTD data is updated _before_ activating
  p->token.status = 0x80;            // set old dummy/new head active
  cache_flush(p);                    // flush (triggers overlay if QH is idle)

#if 0
  cache_invalidate(this);
  dprintf("EP<%p> Current<%p>: token %08lX next %p alt %p ", this, current, overlay.token.val, overlay.next, overlay.alt);
  usb_transfer *h = pending;
  while (1) {
    cache_invalidate(h);
    dprintf(" -> %p (%08lX %s %u)", static_cast<usb_qTD_t*>(h), h->token.val, h->token.val & 0x40 ? "HALT" : (h->token.PID==2 ? "SETUP" : (h->token.PID==1 ? "IN" : "OUT")), h->token.total);
    if (h == dummy.get()) break;
    h = static_cast<usb_transfer*>(h->next);
  }
  dprintf("\n");
#endif
  return true;
}

int QH_Base::BulkIntrTransfer(bool dir_in, const usb_bulkintr_sg* sg, const USBCallback* cb) {
  usb_transfer *msg = bulk_interrupt_transfer::create_bulk_interrupt_transfer(dir_in, sg, cb);
  if (msg == NULL) {
    errno = ENOMEM;
  } else {
    if (enqueue_transfer(msg) == true)
      return 0;
    delete msg;
  }
  return -1;
}

QH_Base::QH_Base(uint8_t endpoint, uint16_t max_packet_size, uint8_t hub, uint8_t port, uint8_t address, uint8_t speed) {
  dummy = std::make_unique<usb_transfer>();
  dummy->token.status = 0x40; // halt
  pending = dummy.get();
  cache_flush(pending);

  memset(static_cast<usb_queue_head_t*>(this), 0, sizeof(usb_queue_head_t));
  // initialize generic queue head
  horizontal_link = 1;
  capabilities.RL = 15;
  capabilities.Mult = 1;
  capabilities.Endpt = endpoint;
  capabilities.address = address;
  if (speed <= 1) {
    capabilities.port = port;
    capabilities.hub = hub;
    if (speed == 1) {
      capabilities.speed = 1;
      capabilities.wMaxPacketSize = max_packet_size < 8 ? max_packet_size : 8;
    } else {
      capabilities.speed = 0;
      capabilities.wMaxPacketSize = max_packet_size < 64 ? max_packet_size : 64;
    }
  } else {
    capabilities.speed = 2;
    capabilities.wMaxPacketSize = max_packet_size < 1024 ? max_packet_size : 1024;
  }

  overlay.alt = QTD_PTR_INVALID;
  overlay.next = pending;

  active = true;
}

QH_Base::~QH_Base() {
  // this endpoint has been removed from the async or periodic schedule and won't process any more transfers
  active = false;
  usb_transfer *t = pending;
  while (t != dummy.get()) {
    usb_transfer *next = static_cast<usb_transfer*>(t->next);
    if (t->cb)
      t->cb->callback(t, -ENODEV);
    delete t;
    t = next;
  }
}

void USB_Async_Endpoint::set_link(const USB_Async_Endpoint* p) {
  uint32_t old_link = horizontal_link;
  if (old_link != 1) cache_invalidate(&horizontal_link);
  horizontal_link = p->link_to();
  if (old_link == 1) cache_flush_invalidate(this);
  else cache_flush_invalidate(&horizontal_link);
}

USB_Control_Endpoint::USB_Control_Endpoint(uint8_t max_packet_size, uint8_t address, uint8_t hub, uint8_t port, uint8_t speed) :
USB_Async_Endpoint(0, max_packet_size, hub, port, address, speed, USB_ENDPOINT_CONTROL) {
  if (speed < 2) capabilities.C = 1;
  capabilities.DTC = 1;
}

int USB_Control_Endpoint::Transfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *buffer, CCallback<usb_control_transfer>* cb) {
  bool dyn_mem = false;

  if (wLength) {
    if (buffer == NULL) {
      if (bmRequestType & USB_CTRLTYPE_DIR_DEVICE2HOST) {
        if (wLength > 5*4096) {
          dprintf("Control transfer too big\n");
          errno = E2BIG;
          return -1;
        }
        if (wLength > 4*4096) {
          buffer = aligned_alloc(4096, (wLength+CACHE_LINE_SIZE-1)&~(CACHE_LINE_SIZE-1));
        } else {
          buffer = aligned_alloc(CACHE_LINE_SIZE, (wLength+CACHE_LINE_SIZE-1)&~(CACHE_LINE_SIZE-1));
        }
        if (buffer == NULL) {
          dprintf("Failed to allocate buffer for control transfer\n");
          errno = ENOMEM;
          return -1;
        }
        dyn_mem = true;
      } else {
        dprintf("Error: Control transfer from host with no buffer provided\n");
        errno = EINVAL;
        return -1;
      }
    } else {
      uint32_t max_length = 5*4096 - ((uint32_t)buffer & 0xFFF);
      if (wLength > max_length) {
        errno = E2BIG;
        return -1;
      }
    }
  }

  control_transfer *msg = new(std::nothrow) control_transfer(bmRequestType, bmRequest, wValue, wIndex, wLength, buffer, cb, dyn_mem);
  if (msg == NULL) {
    errno = ENOMEM;
  } else {
    if (enqueue_transfer(msg) == true)
      return 0;

    if (dyn_mem) free(buffer);
    delete msg;
  }
  return -1;
}

class USB_Bulk_Endpoint : public USB_Async_Endpoint {
private:
  const bool dir_in;
public:
  USB_Bulk_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed) :
  USB_Async_Endpoint(endpoint & 0xF, max_packet_size & 0x7FF, hub_addr, port, address, speed, USB_ENDPOINT_BULK),
  dir_in(endpoint & 0x80) {}

  int BulkTransfer(const usb_bulkintr_sg* sg, const USBCallback* cb) override {
    return BulkIntrTransfer(dir_in, sg, cb);
  }
};

USB_Endpoint* createBulkEndpoint(uint8_t endpoint, uint16_t wMaxPacketSize, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed) {
  return new(std::nothrow) USB_Bulk_Endpoint(endpoint, wMaxPacketSize, address, hub_addr, port, speed);
}

class USB_Interrupt_Endpoint : public USB_Periodic_Endpoint, public QH_Base {
private:
  const bool dir_in;

  void new_offset(void) override {
    // update masks
    switch (interval) {
      case 1:
        capabilities.s_mask = 0xFF;
        break;
      case 2:
        capabilities.s_mask = 0x55 << (offset & 1);
        break;
      case 4:
        capabilities.s_mask = 0x11 << (offset & 3);
        break;
      default:
        capabilities.s_mask <<= (offset & 7);
        capabilities.c_mask <<= (offset & 7);
    }
    cache_flush_invalidate(this);

    uint32_t f_interval = (interval > 8) ? (interval >> 3) : 1;
    for (uint32_t i=offset>>3; i < PERIODIC_LIST_SIZE; i += f_interval) {
      add_node(i, link_to(), interval);
    }
  }
public:
  int InterruptTransfer(const usb_bulkintr_sg* sg, const USBCallback* cb) override {
    return BulkIntrTransfer(dir_in, sg, cb);
  }
  void update(void) override { QH_Base::update(); }
  void flush(void) override { QH_Base::flush(); }
  void get_masks(uint8_t& s_mask, uint8_t& c_mask) const override { s_mask = capabilities.s_mask, c_mask = capabilities.c_mask; }

  bool set_inactive(void) override;
  uint16_t get_max_packet_size(void) const override { return capabilities.wMaxPacketSize; }

  USB_Interrupt_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed, uint32_t i, PeriodicScheduler& scheduler) :
  USB_Periodic_Endpoint(USB_ENDPOINT_INTERRUPT, scheduler),
  QH_Base(endpoint & 0xF, max_packet_size, hub_addr, port, address, speed),
  dir_in(endpoint & 0x80) {
    // worse cast bit stuffing = 7/6ths. Includes CRC16 in bitstuffing plus packet structure
    uint32_t maxlen = (((capabilities.wMaxPacketSize+2) * 298) >> 8) + 5+12;
    uint32_t data_time = (19 + 17 + maxlen + 31) >> 5;
    if (speed == 2) {
      capabilities.Mult = 1 + ((max_packet_size & 0x1800) >> 11);
      if (capabilities.Mult > 3) capabilities.Mult = 3;
      if (i > 16) i = 16;
      i = 1 << (i - 1);
      if (i > PERIODIC_LIST_SIZE*8)
        i = PERIODIC_LIST_SIZE*8;
      interval = i;
      // IN/OUT, DATA, ACK
      stime = data_time * capabilities.Mult;
      capabilities.s_mask = 1;
    } else {
      /* full/low speed interval is in frames
       * round it down to a power of two (required to keep the schedule balanced)
       * and multiply by 8 to get uframes
       */
      for (uint32_t rdown = PERIODIC_LIST_SIZE; rdown > 1; rdown >>= 1) {
        if (i >= rdown) {
          i = rdown;
          break;
        }
      }
      interval = i << 3;
      // SSPLIT and CSPLIT use 4 bytes tokens, one byte larger than IN/OUT
      if (endpoint & 0x80) { // IN
        stime = (20 + 16 + 31) >> 5;       // SSPLIT, Full/Low IN
        ctime = data_time;                 // CSPLIT, Full/Low IN, DATA
      } else { // OUT
        stime = data_time;                 // SSPLIT, Full/Low OUT, DATA
        ctime = (20 + 16 + 17 + 31) >> 5;  // CSPLIT, Full/Low OUT, ACK
      }
      capabilities.s_mask = 1;
      capabilities.c_mask = 0x1C;
    }
  }
};

bool USB_Interrupt_Endpoint::set_inactive(void) {
  cache_invalidate(this);
  if (capabilities.speed != 2 && capabilities.I == 0) {
    dprintf("setting I on endpoint %p\n", this);
    capabilities.I = 1;
    cache_flush_invalidate(this);
    return true;
  }

  bool was_removed = false;
  uint32_t f_interval = (interval > 8) ? (interval >> 3) : 1;
  for (uint32_t i=offset>>3; i < PERIODIC_LIST_SIZE; i += f_interval) {
    if (remove_node(i, link_to(), horizontal_link))
      was_removed = true;
  }

  return was_removed;
}

USB_Endpoint* createInterruptEndpoint(uint8_t endpoint, uint16_t wMaxPacketSize, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed, uint32_t interval, PeriodicScheduler& s) {
  return new(std::nothrow) USB_Interrupt_Endpoint(endpoint, wMaxPacketSize, address, hub_addr, port, speed, interval, s);
}

uint32_t periodicnode_to_interval(uint32_t node) {
  if ((node & 0x1F) == 2) { // is a queue head pointer
    const usb_queue_head_t* qh = (const usb_queue_head_t*)(node-2);
    return static_cast<const USB_Interrupt_Endpoint&>(*qh).interval;
  }
  return 0xFFFFFFFF;
}


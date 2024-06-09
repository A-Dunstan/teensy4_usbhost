#include <atomic>
#include "usb_host.h"

#include <cerrno>
#include <cstring>
#include <string>
#include <locale>
#include <codecvt>
#include <unistd.h>

#include "../utility/portab.h"

#define MK_BE16(a, b) (((a)<<8)|(b))
#define MK_SETUP64(bmrt,bmr,wv,wi,wl) ((((uint64_t)(bmrt))<<56)|(((uint64_t)(bmr))<<48)|(((uint64_t)(wv))<<32)|(((uint64_t)(wi))<<16)|((uint64_t)(wl)))

#define QTD_PTR_INVALID ((usb_qTD_t*)1)
#define IS_QTD_PTR_VALID(p)  ((((uint32_t)p) & 0x1F)==0)

void usb_transfer::sync_chain(usb_qTD_t *end) {
  if (next != end)
    static_cast<usb_transfer*>(next)->sync_chain(end);
  if (IS_QTD_PTR_VALID(alt) && alt != end)
    static_cast<usb_transfer*>(alt)->sync_chain(end);
  if (this != end)
    sync_after_write();
}

void usb_transfer::sync_after_write(void) const {
  cache_flush(static_cast<const usb_qTD_t*>(this));
}

void usb_transfer::sync_before_read(void) {
  cache_invalidate(static_cast<usb_qTD_t*>(this));
}

void usb_qTD_t::fill_qtd(usb_qTD_t* next_qtd, usb_qTD_t* alt_qtd, bool dt, uint16_t total, bool ioc, uint8_t pid, const void* data) {
  next = next_qtd != NULL ? next_qtd : QTD_PTR_INVALID;
  alt = alt_qtd != NULL ? alt_qtd : QTD_PTR_INVALID;
  token.dt = dt ? 1:0;
  token.total = total;
  token.IOC = ioc ? 1:0;
  token.c_Page = 0;
  token.CERR = 3;
  token.PID = pid;
  token.status = 0x80;
  if (total) {
    uint32_t d = (uint32_t)data;
    qtd_page[0] = d;
    d &= 0xFFFFF000;
    qtd_page[1] = d + 0x1000;
    qtd_page[2] = d + 0x2000;
    qtd_page[3] = d + 0x3000;
    qtd_page[4] = d + 0x4000;
  }
}

void usb_queue_head_t::sync_after_write(void) const {
  cache_flush(this, sizeof(usb_queue_head_t));
}

void usb_queue_head_t::sync_before_read(void) {
  cache_invalidate(this, sizeof(usb_queue_head_t));
}

void usb_queue_head_t::clean_after_write(void) {
  cache_flush_invalidate(this, sizeof(usb_queue_head_t));
}

void QH_Base::update(void) {
  while (pending != dummy) {
    int ret = 0;
    usb_transfer* t = pending;
    t->sync_before_read();
    if (t->token.status & 0x80) // transfer is still active
      return;

//    dprintf("EP<%p> usb transfer: %p, token %08lX, cb %p, data %p\n", this, static_cast<usb_qTD_t*>(t), t->token.val, t->cb, t->qtd_page[0]);
    if (t->token.status & 0x40) {
      // there was an error, walk the list until we find the next callback
      dprintf("USB TRANSFER ERROR %p %08lX\n", t, t->token.val);
      ret = -EPIPE;
      uint32_t status = t->token.status;
      if (t->error_handler) {
        usb_transfer *n = t->error_handler;
        delete t;
        t = n;
      } else {
        while (t->cb == NULL) {
          usb_transfer *n = static_cast<usb_transfer*>(t->next);
          delete t;
          t = n;
        }
      }

      if (status & 0x20)
        ret = -EOVERFLOW;
      else if (status & 0x10)
        ret = -EPROTO;

      // unhalt the queue, resume processing
      sync_before_read();
      overlay.next = t->next;
      sync_after_write();
      overlay.token.status = 0; // clear halt, reset DT
      clean_after_write();
      pending = static_cast<usb_transfer*>(t->next);
    } else {
      ret = t->token.total;
      if (ret && IS_QTD_PTR_VALID(t->alt)) {
        pending = static_cast<usb_transfer*>(t->alt);
        while (t->cb == NULL) {
          usb_transfer *n = static_cast<usb_transfer*>(t->next);
          delete t;
          ret += n->token.total;
          t = n;
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

//    dprintf("next: %p(%p)\n", pending, dummy);
  }
}

QH_Base::~QH_Base() {
  // this endpoint has been removed from the async or periodic schedule and won't process any more transfers
  dprintf("Endpoint %p cleanup\n", this);
  active = false;
  usb_transfer *t = pending;
  while (t != dummy) {
    usb_transfer *next = static_cast<usb_transfer*>(t->next);
    if (t->cb)
      t->cb->callback(t, -ENODEV);
    delete t;
    t = next;
  }
  // release dummy too
  delete t;
}

void QH_Base::flush(void) {
  // get rid of any pending transfers
  dprintf("Endpoint %p flush\n", this);
  sync_before_read();
  overlay.alt = QTD_PTR_INVALID;
  overlay.next = dummy;
  overlay.token.val = 0; // reset dt to zero
  clean_after_write();

  usb_transfer *t = pending;
  while (t != dummy) {
    t->sync_before_read();
    usb_transfer *next = static_cast<usb_transfer*>(t->next);
    dprintf("FLUSH: %p %08X\n", static_cast<usb_qTD_t*>(t), t->token.val);
    delete t;
    t = next;
  }
  pending = t;

}

QH_Base::QH_Base(uint8_t endpoint, uint16_t max_packet_size, uint8_t hub, uint8_t port, uint8_t address, uint8_t speed) {
  dummy = new(std::nothrow) usb_transfer;
  dummy->token.status = 0x40; // halt
  dummy->sync_after_write();
  pending = dummy;

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
  overlay.next = dummy;

  active = true;
}

bool QH_Base::enqueue_transfer(usb_transfer* head) {
  if (!active) {
    errno = ENODEV;
    return false;
  }
//  dprintf("Enqueue transfer<%p>: %p(%p), dummy: %p\n", this, static_cast<usb_qTD_t*>(head), head->qtd_page[0], static_cast<usb_qTD_t*>(dummy));

  head->sync_chain();

  usb_transfer* p = dummy;    // local copy of dummy ptr
  head->token.status = 0x40;  // set inactive+halted
  *p = *head;                 // copy over dummy (becomes new head)
  p->sync_after_write();      // ensure new qTD data is updated _before_ activating
  head->sync_after_write();
  cache_sync();               // ensure cache flush has completed
  p->token.status = 0x80;     // set old dummy/new head active
  p->sync_after_write();      // flush (triggers overlay if QH is idle)
  dummy = head;               // old head is now dummy

#if 0
  sync_before_read();
  dprintf("EP<%p> Current<%p>: token %08lX next %p alt %p ", this, current, overlay.token.val, overlay.next, overlay.alt);
  usb_transfer *h = pending;
  while (1) {
    h->sync_before_read();
    dprintf(" -> %p (%08lX %s %u)", static_cast<usb_qTD_t*>(h), h->token.val, h->token.val & 0x40 ? "HALT" : (h->token.PID==2 ? "SETUP" : (h->token.PID==1 ? "IN" : "OUT")), h->token.total);
    if (h == dummy) break;
    h = static_cast<usb_transfer*>(h->next);
  }
  dprintf("\n");
#endif
  return true;
}

uint64_t usb_control_transfer::getSetupData(void) const {
  return MK_SETUP64(req.bmRequestType, req.bmRequest, req.wValue, req.wIndex, req.wLength);
}

void USB_Async_Endpoint::set_link(const USB_Async_Endpoint* p) {
  uint32_t old_link = horizontal_link;
  if (old_link != 1) cache_invalidate(&horizontal_link);
  horizontal_link = p->link_to();
  if (old_link == 1) clean_after_write();
  else cache_flush_invalidate(&horizontal_link);
}

void USB_Control_Endpoint::transfer::callback(const usb_transfer*, int result) {
  if (result >= 0 && req.wLength) {
    result = req.wLength - data.token.total;
    if (req.bmRequestType & USB_CTRLTYPE_DIR_DEVICE2HOST)
      cache_invalidate(buffer, result);
  }
  cb->callback(this, result);
  if (do_release) free(buffer);
}

USB_Control_Endpoint::transfer::transfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength,
void *_buffer, CCallback<usb_control_transfer>* _cb, bool release_mem) : cb(_cb) {
  req.bmRequestType = bmRequestType;
  req.bmRequest = bmRequest;
  req.wValue = wValue;
  req.wIndex = wIndex;
  req.wLength = wLength;
  buffer = _buffer;
  do_release = release_mem;

  cache_flush(&req);

  ack.cb = this;
  fill_qtd(&ack, NULL, false, sizeof(req), false, 2, &req);
  ack.fill_qtd(this, NULL, true, 0, true, 1, NULL);

  if (wLength) {
    next = &data;
    data.fill_qtd(&ack, NULL, true, wLength, false, bmRequestType & USB_CTRLTYPE_DIR_DEVICE2HOST ? 1:0, buffer);
    ack.token.PID = 1 - data.token.PID;
    cache_flush(buffer, wLength);
  }
//  dprintf("usb_control_transfer: msg %p data %p ack %p (%p)\n", static_cast<usb_qTD_t*>(this), static_cast<usb_qTD_t*>(&data), static_cast<usb_qTD_t*>(&ack), static_cast<usb_qTD_t*>(&data)->qtd_page[0]);
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
      if (bmRequestType & 0x80) {
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

  transfer *msg = new(std::nothrow) transfer(bmRequestType, bmRequest, wValue, wIndex, wLength, buffer, cb, dyn_mem);
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

void usb_bulk_interrupt_transfer::callback(const usb_transfer *t, int result) {
  if (result >= 0 && dlength) {
    result = dlength - result;
    if (dir_in)
      cache_invalidate(buffer, result);
  }
  (*cbi)(result);
}

usb_bulk_interrupt_transfer::usb_bulk_interrupt_transfer(bool _dir_in, uint32_t length, void *_buffer, const USBCallback* _cb, uint32_t max_packet) :
cbi(_cb) {
  buffer = _buffer;
  dlength = length;
  dir_in = _dir_in;

  if (length) {
    cache_flush(buffer, length);
  }

  uint8_t PID = dir_in ? 1:0;
  uint32_t length0 = 5*4096 - ((uint32_t)buffer & 0xFFF);

  cb = extra[0].cb = extra[1].cb = extra[2].cb = this;
  extra[0].error_handler = extra[1].error_handler = &extra[2];

  if (length0 >= length)
    fill_qtd(this, NULL, true, length, true, PID, buffer);
  else {
    /* cascaded transfers must ensure their lengths are multiples of wMaxPacketSize
     * else the device may trigger babble
     */
    length0 -= length0 % max_packet;
    const uint8_t* b = (const uint8_t*)buffer + length0;
    length -= length0;
    error_handler = &extra[2];
    fill_qtd(&extra[2], this, true, length0, false, PID, buffer);
    length0 = 5*4096 - ((uint32_t)b & 0xFFF);
    cb = extra[0].cb = extra[1].cb = NULL;
    if (length > length0) {
      next = &extra[0];
      size_t i=0;
      while (length > length0) {
        length0 -= length0 % max_packet;
        extra[i].fill_qtd(&extra[i+1], this, true, length0, false, PID, b);
        b += length0;
        length -= length0;
        length0 = 5*4096 - ((uint32_t)b & 0xFFF);
        i++;
      }
      extra[i-1].next = &extra[2];
    }
    extra[2].fill_qtd(this, NULL, true, (uint16_t)length, true, PID, b);
  }
}

int QH_Base::BulkIntrTransfer(bool dir_in, uint32_t Length, void *buffer, const USBCallback* cb) {
  uint32_t max_length = 4*5*4096 - ((uint32_t)buffer & 0xFFF);
  if (Length > max_length) {
    dprintf("bulk_interrupt transfer too big\n");
    errno = E2BIG;
  } else {
    usb_transfer *msg = new(std::nothrow) usb_bulk_interrupt_transfer(dir_in, Length, buffer, cb, capabilities.wMaxPacketSize);
    if (msg == NULL) {
      errno = ENOMEM;
    } else {
      if (enqueue_transfer(msg) == true) // why is the qualified name required here?
        return 0;
      delete msg;
    }
  }
  return -1;
}

PeriodicScheduler::PeriodicScheduler(volatile uint32_t& frindex) :
FRINDEX(frindex) {
  periodictable = (uint32_t*)aligned_alloc(4096, sizeof(uint32_t)*PERIODIC_LIST_SIZE);

  dprintf("PERIODIC_LIST_SIZE: %d (%p)\n", PERIODIC_LIST_SIZE, periodictable);
  for (int i=0; i < PERIODIC_LIST_SIZE; i++)
    periodictable[i] = 0x80000001;
  cache_flush(periodictable, sizeof(periodictable[0])*PERIODIC_LIST_SIZE);
}

PeriodicScheduler::~PeriodicScheduler() {
  free(periodictable);
}

USB_Interrupt_Endpoint::USB_Interrupt_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed, uint32_t i) :
USB_Bulk_Interrupt_Endpoint(endpoint, max_packet_size, address, hub_addr, port, speed, USB_ENDPOINT_INTERRUPT) {
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

void USB_Interrupt_Endpoint::new_offset(void) {
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
  clean_after_write();

  uint32_t f_interval = (interval > 8) ? (interval >> 3) : 1;
  for (uint32_t i=offset>>3; i < PERIODIC_LIST_SIZE; i += f_interval) {
    add_node(i, link_to(), interval);
  }
}

bool USB_Interrupt_Endpoint::set_inactive(void) {
  sync_before_read();
  if (capabilities.speed != 2 && capabilities.I == 0) {
    dprintf("setting I on endpoint %p\n", this);
    capabilities.I = 1;
    clean_after_write();
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

template <class CTransfer>
class iso_transfer : public CCallback<CTransfer> {
private:
  const USBCallback *cb;
  int16_t* const lengths;
  unsigned int xfer_done;
  int total_length;

  void callback(const CTransfer *t,int r) override {
//    dprintf("T%u %p %d\n", done, t, r);
    lengths[xfer_done] = (int16_t)r;
    if (r > 0) total_length += r;

    if (++xfer_done >= xfer_total) {
      (*cb)(r < 0 ? r : total_length);
      delete(this);
    }
  }
public:
  unsigned int xfer_total;

  iso_transfer(const USBCallback *_cb, int16_t *l) :
  cb(_cb),
  lengths(l) {
    xfer_done = 0;
    xfer_total = 0;
    total_length = 0;
  }

  static void* operator new(size_t s, unsigned int t_count, CTransfer* &transfers) noexcept(true) {
    s = (s + 31) & ~31;
    uint8_t *p = (uint8_t*)aligned_alloc(32, s + t_count*sizeof(CTransfer));
    if (p != NULL)
      transfers = new (p+s) CTransfer[t_count];
    return p;
  }
};

class USB_ISO_High_Endpoint : public USB_Periodic_Endpoint {
private:
  const bool dir_in;
  const uint32_t mult;
  const uint32_t wMaxPacketSize;
  const uint8_t address;
  const uint8_t endpoint;
  uint8_t s_mask;

  itd_transfer* pending = NULL;

  int Schedule(itd_transfer*);

public:
  void update(void);
  void flush(void) override {};

  void new_offset(void) override;
  bool set_inactive(void) override;

  USB_ISO_High_Endpoint(uint8_t endopint, uint16_t wMaxPacketSize, uint8_t address, uint32_t interval);
  ~USB_ISO_High_Endpoint();

  void get_masks(uint8_t& smask, uint8_t& cmask) const override {smask = s_mask, cmask = 0;}

  int IsochronousTransfer(isolength& Lengths, void *buffer, const USBCallback* cb) override;
};

USB_ISO_High_Endpoint::USB_ISO_High_Endpoint(uint8_t _endpoint, uint16_t _wMaxPacketSize, uint8_t _address, uint32_t i) :
USB_Periodic_Endpoint(USB_ENDPOINT_ISOCHRONOUS),
dir_in(_endpoint & 0x80),
mult(((_wMaxPacketSize + 2048) >> 11) & 3),
wMaxPacketSize(((_wMaxPacketSize & 0x7FF) < 1024) ? (_wMaxPacketSize & 0x7FF) : 1024),
address(_address),
endpoint(_endpoint & 0x7F)
{
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

USB_ISO_High_Endpoint::~USB_ISO_High_Endpoint() {
  dprintf("USB_ISO_High_Endpoint<%p> deleted\n", this);
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

    t->cb->callback(t, -ENODEV);
  }

  return removed;
}

void USB_ISO_High_Endpoint::update(void) {
  while (pending) {
    itd_transfer *t = pending;

    cache_invalidate(t, sizeof(usb_iTD_t));
    for (unsigned int i=8; i > 0; i--) {
      // process complete iTD only
      if (t->transfers[i-1].status & 0x8)
        return;
    }
    pending = t->next;
    remove_node(t->frame>>3, t->link_to(), t->horizontal_link);

    uint16_t transferred = 0;
    for (unsigned int i=0; i < 8; i++) {
      if (t->s_mask & (1<<i)) {
        if (t->transfers[i].status & 7)
          dprintf("Possible ISO transfer error: %02X\n", t->transfers[i].status);
        /* iTD is handled differently to siTD:
         * siTD total is updated to the number of bytes that _weren't_ transferred
         * iTD total is updated to the number of bytes that _were_ transferred
         */
        transferred += t->transfers[i].total;
      }
    }

    if (dir_in) {
      cache_invalidate(t->buffer, transferred);
    }

    t->cb->callback(t,transferred);
  }
}

USB_ISO_Full_Endpoint::USB_ISO_Full_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint32_t i) :
USB_Periodic_Endpoint(USB_ENDPOINT_ISOCHRONOUS),
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

  memset(&state, 0, sizeof(state));
  state.address = address;
  state.endpt = endpoint;
  state.hub = hub_addr;
  state.port = port;
  state.dir = dir_in ? 1:0;
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


USB_ISO_Full_Endpoint::~USB_ISO_Full_Endpoint() {
  dprintf("USB_ISO_Full_Endpoint<%p> deleted\n", this);
}

void USB_ISO_Full_Endpoint::new_offset(void) {
  dprintf("USB_ISO_Full_Endpoint<%p>: new_offset %d\n", this, offset);
  s_mask <<= (offset & 7);
  c_mask <<= (offset & 7);
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

    int16_t length = t->length - t->total;

    if (dir_in) {
      cache_invalidate(t->buffer, length);
    }

    t->cb->callback(t,length);
  }
}

int USB_ISO_High_Endpoint::IsochronousTransfer(isolength& Lengths, void* buffer, const USBCallback* cb) {
  iso_transfer<itd_transfer> *ig;
  itd_transfer* t;
  unsigned int t_count;
  uint8_t* dst = (uint8_t*)buffer;
  uint16_t length_per_t = mult * wMaxPacketSize;

  for (t_count = 0; t_count < 8; t_count++) {
    if (Lengths[t_count]==0) break;
  }

  ig = new(t_count, t) iso_transfer<itd_transfer>(cb, Lengths);
  if (ig == NULL) errno = ENOMEM;
  else {
    int16_t *pLength = Lengths;
    do {
      memset(t, 0, sizeof(*t));
      t->horizontal_link = 1;
      t->address = address;
      t->endpt = endpoint;
      t->wMaxPacketSize = wMaxPacketSize;
      t->dir = dir_in;
      t->Mult = mult;

      uint32_t len = *pLength;
      uint32_t PG = 0;
      uint32_t b = (uint32_t)dst;
      t->page0 = b >> 12;
      t->page1 = (b+0x1000) >> 12;
      t->page2 = (b+0x2000) >> 12;
      t->page3 = (b+0x3000) >> 12;
      t->page4 = (b+0x4000) >> 12;
      t->page5 = (b+0x5000) >> 12;
      t->page6 = (b+0x6000) >> 12;

      cache_flush_invalidate(dst, len);

      for (int i=0; i < 8 && len; i++) {
        if (s_mask & (1<<i)) {
          uint32_t this_len = len > length_per_t ? length_per_t : len;
          t->transfers[i].status = 8; // active
          t->transfers[i].total = this_len;
          t->transfers[i].IOC = 1;
          t->transfers[i].PG = PG;
          t->transfers[i].offset = b & 0xFFF;
          uint32_t nextb = b + this_len;
          if ((nextb ^ b) & 0x1000) ++PG;
          b = nextb;
          t->s_mask |= 1<<i;
          len -= this_len;
        }
      }

      t->cb = ig;
      t->buffer = dst;
      dst += *pLength;
      *pLength++ = 0;
      cache_flush(t, sizeof(usb_iTD_t));

      if (Schedule(t) < 0) {
        dprintf("Failed to schedule transfer\n");
        break;
      }
      ig->xfer_total++;
      t++;
    } while (--t_count);

    if (ig->xfer_total > 0)
      return 0;

    delete ig;
  }
  return -1;
}

int USB_ISO_Full_Endpoint::IsochronousTransfer(isolength& Lengths, void *buffer, const USBCallback* cb) {
  iso_transfer<sitd_transfer> *sg;
  sitd_transfer* t;
  unsigned int t_count;
  uint8_t* dst = (uint8_t*)buffer;

  for (t_count = 0; t_count < 8; t_count++) {
    if (Lengths[t_count]==0) break;
  }

  sg = new(t_count, t) iso_transfer<sitd_transfer>(cb, Lengths);
  if (sg == NULL) errno = ENOMEM;
  else {
    int16_t *pLength = Lengths;
    do {
      int16_t length = *pLength;
      *pLength++ = 0;
      if (Transfer(t, length, dst, sg) < 0) {
        dprintf("Failed to schedule transfer\n");
        break;
      }
      sg->xfer_total++;
      dst += length;
      t++;
    } while (--t_count);

    if (sg->xfer_total > 0)
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

//  dprintf("ISO transfer<%p>: uframe_now %lu:%lu, scheduled %lu:%lu, uframe_min %lu:%lu, uframe_max %lu:%lu\n", t, uframe_now>>3, uframe_now&7, t->frame>>3, t->frame&7, uframe_min>>3, uframe_min&7, uframe_max>>3, uframe_max&7);

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

int USB_ISO_Full_Endpoint::Transfer(sitd_transfer *t, int16_t Length, void *buffer, CCallback<sitd_transfer>* cb) {
  memset(t, 0, sizeof(usb_siTD_t));
  t->horizontal_link = 1;
  t->state = state;
  t->s_mask = s_mask;
  t->c_mask = c_mask;
  t->status = 0x80;
  t->total = Length;
  t->IOC = 1;

  t->buffer = t->page0 = buffer;
  t->page1 = (((uint32_t)buffer)>>12)+1;

  t->t_count = dir_in ? 1 : ((t->total+187) / 188); //__builtin_popcount(s_mask);
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

USB_Hub::USB_Hub(uint8_t addr) : hub_addr(addr) {
  memset(port, 0, sizeof(port));
}

USB_Device::dev_interface::setting::setting(const uint8_t *i, const uint8_t *end) : interface_desc((const usb_interface_descriptor*)i) {
  length = end-i;
  for (uint8_t e=0; e < interface_desc->bNumEndpoints;) {
    if (i[0]>=sizeof(usb_endpoint_descriptor) && i[1]==USB_DT_ENDPOINT){
      endpoints.push_back((const usb_endpoint_descriptor*)i);
      e++;
    }
    i += i[0];
  }
}

USB_Device::dev_interface::dev_interface(const uint8_t *i, const uint8_t *end) {
  altSettings.emplace_back(i, end);
  active = -1;
}

void USB_Device::dev_interface::add_alternate(const uint8_t *i, const uint8_t *end) {
  altSettings.emplace_back(i, end);
  altSettings[0].length += end-i;
}

const usb_interface_descriptor* USB_Device::dev_interface::getInterface(size_t &l, uint8_t index) const {
  if (index < altSettings.size()) {
    l = altSettings[index].length;
    return altSettings[index].interface_desc;
  }
  l = 0;
  return NULL;
}

const usb_interface_descriptor* USB_Device::dev_interface::getInterface(uint8_t index) const {
  if (index < altSettings.size())
    return altSettings[index].interface_desc;
  return NULL;
}

const usb_endpoint_descriptor* USB_Device::dev_interface::getEndpoint(uint8_t eindex, uint8_t iindex) const {
  if (iindex < altSettings.size()) {
    if (eindex < altSettings[iindex].endpoints.size())
      return altSettings[iindex].endpoints[eindex];
  }
  return NULL;
}

USB_Device::dev_config::dev_config(const usb_configuration_descriptor *desc, size_t len) : raw(new uint8_t[len]), length(len) {
  memcpy(raw, desc, len);
  const uint8_t *d = raw + sizeof(usb_configuration_descriptor);
  const uint8_t *end = raw + len;

  const uint8_t *interface = NULL;
  while (d < end-4) {
    if (d[0] == sizeof(usb_interface_descriptor) && d[1] == USB_DT_INTERFACE) {
      if (interface) {
        if (interface[3] == 0) { // bAlternateSetting
          // default interface, create node
          interfaces.emplace_back(interface, d);
        } else {
          // alternate interface, add to existing node
          interfaces.back().add_alternate(interface, d);
        }
      }
      interface = d;
    }
    d += d[0];
  }
  // add final
  if (interface != NULL) {
    if (interface[3] == 0) {
      interfaces.emplace_back(interface, end);
    } else {
      interfaces.back().add_alternate(interface, end);
    }
  }
}

USB_Device::dev_config::~dev_config() {
  delete[] raw;
}

USB_Device::dev_interface* USB_Device::dev_config::interface(uint8_t index) {
  if (index < interfaces.size()) {
    return &(interfaces[index]);
  }
  return NULL;
}

void USB_Device::callback(const usb_control_transfer *t, int result) {
  dprintf("Device<%p> control callback %p result %d\n", this, t, result);
  uint16_t rt_rq = MK_BE16(t->getbmRequestType(), t->getbmRequest());

  switch (rt_rq) {
    case MK_BE16(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR): {
        uint8_t dt = validate_descriptor((const uint8_t*)t->getBuffer(), result);
        switch (t->getwValue() >> 8) {
          case USB_DT_DEVICE:
            if (dt == USB_DT_DEVICE) {
              memcpy(&ddesc,t->getBuffer(),sizeof(ddesc));
              dprintf("Device<%p> Device Descriptor %p:\n", this, t->getBuffer());
              dprintf("   bcdUSB %04X\n   bDeviceClass %02X\n   bDeviceSubClass %02X\n   bDeviceProtocol %02X\n", ddesc.bcdUSB, ddesc.bDeviceClass, ddesc.bDeviceSubClass, ddesc.bDeviceProtocol);
              dprintf("   idVendor %04X\n   idProduct %04X\n   bcdDevice %04X\n", ddesc.idVendor, ddesc.idProduct, ddesc.bcdDevice);
              dprintf("   iManufacturer %d\n   iProduct %d\n   iSerialNumber %d\n   bNumConfigurations %d\n", ddesc.iManufacturer, ddesc.iProduct, ddesc.iSerialNumber, ddesc.bNumConfigurations);
              if (ddesc.iManufacturer) request_string(ddesc.iManufacturer);
              if (ddesc.iProduct) request_string(ddesc.iProduct);
              if (ddesc.iSerialNumber) request_string(ddesc.iSerialNumber);
              // request all configurations (config descriptor only, full size will be retrieved afterwards)
              for (uint8_t c=0; c < ddesc.bNumConfigurations; c++) {
                control.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, (USB_DT_CONFIGURATION<<8)|c, 0, sizeof(usb_configuration_descriptor), NULL, this);
              }
              return;
            }
            break;
          case USB_DT_CONFIGURATION:
            if (dt == USB_DT_CONFIGURATION) {
              const usb_configuration_descriptor *cdesc = (const usb_configuration_descriptor*)t->getBuffer();
              if (t->getwLength() == sizeof(usb_configuration_descriptor)) {
                // got the length, now get the entire thing
                if (cdesc->iConfiguration) request_string(cdesc->iConfiguration);
                control.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, t->getwValue(), 0, cdesc->wTotalLength, NULL, this);
                return;
              } else {
                configs.try_emplace(cdesc->bConfigurationValue, cdesc, result);
                cdesc = configs.at(cdesc->bConfigurationValue).getConfiguration();
                dprintf("Device<%p> Configuration %d:\n", this, cdesc->bConfigurationValue);
                dprintf("\twTotalLength %d\n\tbNumInterfaces %d\n\tiConfiguration %d\n", cdesc->wTotalLength, cdesc->bNumInterfaces, cdesc->iConfiguration);
                dprintf("\tbmAttributes %02X\n\tbMaxPower %d\n", cdesc->bmAttributes, cdesc->bMaxPower);
                for (uint8_t i=0; i < cdesc->bNumInterfaces;) {
                  uint8_t alt=0;
                  const dev_interface* di = configs.at(cdesc->bConfigurationValue).interface(i);
                  while(1) {
                    size_t l;
                    const usb_interface_descriptor *idesc = di->getInterface(l, alt);
                    if (idesc == NULL) break;
                    dprintf("Interface %d, Length %u:\n", idesc->bInterfaceNumber, l);
                    dprintf("\tbAlternateSetting %d\n\tbNumEndpoints %d\n\tbInterfaceClass %02X\n", idesc->bAlternateSetting, idesc->bNumEndpoints, idesc->bInterfaceClass);
                    dprintf("\tbInterfaceSubClass %02X\n\tbInterfaceProtocol %02X\n\tiInterface %d\n", idesc->bInterfaceSubClass, idesc->bInterfaceProtocol, idesc->iInterface);
                    if (idesc->iInterface) request_string(idesc->iInterface);
                    for (uint8_t e=0; e < idesc->bNumEndpoints; e++) {
                      const usb_endpoint_descriptor *edesc = di->getEndpoint(e, alt);
                      dprintf("\t Endpoint %02X:\n", edesc->bEndpointAddress);
                      dprintf("\t\tbmAttributes %02X\n\t\twMaxPacketSize %04X\n\t\tbInterval %d\n", edesc->bmAttributes, edesc->wMaxPacketSize, edesc->bInterval);
                    }
                    ++alt;
                  }
                  ++i;
                }
              }

              if (configs.size() == ddesc.bNumConfigurations) {
                /* There may still be string requests pending and
                * it would be cleaner to look-up drivers in a
                * different function than this one, so punt this
                * action.
                */
                usb_msg_t msg = {
                  .type = USB_MSG_DEVICE_FIND_DRIVER,
                  .device = {
                   .dev = this
                  }
                };
                /* delayed message is the "safe" way, but if it
                * fails just put the message directly on the host queue.
                */
                refcount.fetch_add(1);
                if (host->timerMsg(msg, 10) == false) {
                  if (host->putMessage(msg) == false) {
                    // have to give up, driver won't be hooked and
                    // configuration/interfaces won't be activated
                    deref();
                  }
                }
              }
              return;
            }
            else
              dprintf("Device<%p> had a bad configuration descriptor\n", this);
            break;
          case USB_DT_STRING:
            // this device has a bad bDescriptorType in some of its strings
            if (ddesc.idVendor==0x0BDA && ddesc.idProduct==0x8179 && dt==0x43)
              dt = USB_DT_STRING;
            if (dt == USB_DT_STRING) {
              const usb_string_descriptor *s = (const usb_string_descriptor*)t->getBuffer();
              int len = (result-2) / 2;
              int index = t->getwValue() & 0xFF;
              if (index == 0) { // supported languages
                dprintf("Device<%p> number of supported languages: %d\n", this, len);
                for (int i=0; i < len; i++) {
                  if (s->wLANGID[i] == USB_LANG) {
                    string_lang = USB_LANG;
                    dprintf("Device<%p> using language %04X for strings\n", this, string_lang);
                    break;
                  }
                }
              } else if (t->getwIndex() == 0x0409) {
                std::wstring_convert<std::codecvt_utf8<char16_t>, char16_t> converter;
                strings[index] = converter.to_bytes(std::u16string(s->bString,len));
                dprintf("Device<%p> string index %d: %s\n", this, index, strings.at(index).c_str());
              }
              return;
            }
            break;
        }
      }
      break;
    case MK_BE16(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_CONFIGURATION):
      if (result >= 0) {
        dprintf("Device<%p> new configuration set: %02X\n", this, t->getwValue());
        activate_configuration(t->getwValue());
      } else {
        dprintf("Device<%p> setting new configuration failed %d\n", this, result);
      }
      return;
    case MK_BE16(USB_REQTYPE_INTERFACE_GET, USB_REQ_GET_INTERFACE):
      if (result >= 1) {
        const uint8_t* d = (const uint8_t*)t->getBuffer();
        dprintf("Device<%p> Interface setting retrieved: %u %u\n", this, t->getwIndex(), *d);
        activate_interface(t->getwIndex(), *d);
        return;
      }
      break;
    case MK_BE16(USB_REQTYPE_INTERFACE_SET, USB_REQ_SET_INTERFACE):
      if (result >= 0) {
        dprintf("New interface set: interface %u, altSetting %u\n", t->getwIndex(), t->getwValue());
        activate_interface(t->getwIndex(), t->getwValue());
      } else {
        dprintf("Setting new interface failed (%d) for interface %u, altSetting %u\n", result, t->getwIndex(), t->getwValue());
        // don't know what altSetting is active now - so ask
        control.Transfer(USB_REQTYPE_INTERFACE_GET, USB_REQ_GET_INTERFACE, 0, t->getwIndex(), 1, NULL, this);
      }
      return;
    case MK_BE16(USB_REQTYPE_ENDPOINT_SET, USB_REQ_CLEAR_FEATURE):
      if (t->getwValue() == USB_FEATURE_ENDPOINT_HALT) {
        if (result >= 0) {
      uint8_t endpoint = t->getwIndex();
      dprintf("HALT was cleared for endpoint %02X\n", endpoint);
      // data toggle must be reset in queue head
      int ep_type = Endpoints[endpoint].type;
      if (ep_type == USB_ENDPOINT_INTERRUPT || ep_type == USB_ENDPOINT_BULK) {
        Endpoints[endpoint].ep->flush();
      }
    }
    else dprintf("Clearing endpoint halt failed\n");
    return;
    }
    break;
  }

  dprintf("USB_Device<%p>: unknown control request or failure (%d), bmRequestType %02X bmRequest %02X wValue %04X\n", this, result, t->getbmRequestType(), t->getbmRequest(), t->getwValue());
}

void USB_Device::deref(void) {
//  dprintf("USB_Device<%p> deref: %d\n", this, refcount.load()-1);
  // this performs a post-decrement so compare against 1
  if (refcount.fetch_sub(1) == 1) {
    // detach all drivers
    for (auto drv=drivers.begin(); drv != drivers.end(); drv++) {
      (*drv)->disconnect();
    }
    usb_msg_t msg = {
      .type = USB_MSG_ADDRESS_RELEASED,
      .address = address
    };
    host->putMessage(msg);
    dprintf("USB_Device<%p> deleted\n", this);
    delete this;
  }
}

USB_Device::USB_Device(USB_Host* _host, uint8_t _hub_addr, uint8_t _port, uint8_t _speed, uint8_t _address, uint8_t control_packet_size) :
control(control_packet_size, _address, _hub_addr, _port, _speed),
host(_host),
hub_addr(_hub_addr),
port(_port),
speed(_speed),
address(_address),
refcount(2) // hub and control endpoint have a reference
{
  dprintf("New Device<%p>: address %d, port %d, hub %d, speed %d, control_packet_size %d\n", this, address, port, hub_addr, speed, control_packet_size);
  memset(&ddesc, 0, sizeof(ddesc));
  string_lang = 0;
  active_config = -1;
  pending_config = -1;
  host->activate_endpoint(&control, this);
}

void USB_Device::disconnect(void) {
  // control endpoint last
  for (size_t i=1; i < 16; i++) {
    deactivate_endpoint(i);
    deactivate_endpoint(0x80|i);
  }
  host->deactivate_endpoint(&control);
  deref();
}

void USB_Device::USBMessage(const usb_msg_t& msg) {
  switch (msg.type) {
    case USB_MSG_DEVICE_INIT:
      // request string descriptor zero (supported languages)
      control.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, USB_DT_STRING << 8, 0, 255, NULL, this);
      // request device descriptor
      control.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8, 0, sizeof(ddesc), NULL, this);
      return;
    case USB_MSG_DEVICE_ENDPOINT_REMOVED:
      dprintf("USB_Device<%p> Endpoint %p removed\n", this, msg.device.endpoint);
      if (msg.device.endpoint != &control)
        delete msg.device.endpoint;
      deref();
      break;
    case USB_MSG_DEVICE_FIND_DRIVER:
      search_for_drivers();
      deref();
      break;
    case USB_MSG_DEVICE_BULK_TRANSFER:
      BulkTransfer(msg.device.bulkintr.bEndpoint, msg.device.bulkintr.dLength, msg.device.bulkintr.data, msg.device.cb);
      deref();
      break;
    case USB_MSG_DEVICE_INTERRUPT_TRANSFER:
      InterruptTransfer(msg.device.bulkintr.bEndpoint, msg.device.bulkintr.dLength, msg.device.bulkintr.data, msg.device.cb);
      deref();
      break;
    case USB_MSG_DEVICE_CONTROL_TRANSFER:
      ControlTransfer(msg.device.control.bmRequestType, msg.device.control.bmRequest, msg.device.control.wValue, msg.device.control.wIndex, msg.device.control.wLength, msg.device.control.data, msg.device.control_cb);
      deref();
      break;
    case USB_MSG_DEVICE_ISOCHRONOUS_TRANSFER:
      IsochronousTransfer(msg.device.iso.bEndpoint, *msg.device.iso.lengths, msg.device.iso.data, msg.device.cb);
      deref();
      break;
    default:
      dprintf("Unknown USB Device message: %d\n", msg.type);
  }
}

void USB_Device::request_string(uint8_t string_id) {
  if (string_lang) {
    if (strings.count(string_id)==0) {
      // add empty string placeholder in case it is requested again before this request completes
      strings[string_id] = "";
      control.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING<<8)|string_id, string_lang, 255, NULL, this);
    }
    else dprintf("String %d was already present (\"%s\")\n", string_id, strings.at(string_id).c_str());
  }
}

void USB_Device::search_for_drivers(void) {
  // try each Configuration
  for (auto c=configs.begin(); c != configs.end(); c++) {
    const usb_configuration_descriptor *config = (*c).second.getConfiguration();
    USB_Driver::Factory *f = USB_Driver::Factory::find_driver(&ddesc, config);
    if (f != NULL) {
      activate_configuration((*c).first);
      USB_Driver *d = f->attach(&ddesc, config, this);
      if (d) {
        drivers.push_back(d);
        return;
      }
    }
  }
  // else offer each individual interface from first configuration
  bool activated = false;
  dev_config& c = (*(configs.begin())).second;
  for (unsigned int i=0; i < 256; i++) {
    size_t l;
    const dev_interface *di = c.interface(i);
    if (di == NULL) break;
    const usb_interface_descriptor *iface = di->getInterface(l, 0);
    USB_Driver::Factory *f = USB_Driver::Factory::find_driver(iface, l);
    if (f != NULL) {
      if (!activated) {
        activate_configuration((*(configs.begin())).first);
        activated = true;
      }
      USB_Driver *d = f->attach(iface, l, this);
      if (d) {
        drivers.push_back(d);
      }
    }
  }
}

void USB_Device::deactivate_endpoint(size_t i) {
  if (Endpoints[i].type >= 0) {
    host->deactivate_endpoint(Endpoints[i].ep);
    Endpoints[i].type = -1;
    Endpoints[i].ep = NULL;
  }
}

void USB_Device::activate_endpoint(const usb_endpoint_descriptor* p) {
  size_t i = p->bEndpointAddress;
  if (Endpoints[i].type >= 0) {
    dprintf("Device<%p> activate_endpoint error: endpoint already exists for address %u\n", this, p->bEndpointAddress);
    return;
  }

  switch (p->bmAttributes & 0x3) {
    case USB_ENDPOINT_BULK:
      refcount.fetch_add(1);
      Endpoints[i].ep = new(std::nothrow) USB_Bulk_Endpoint(p->bEndpointAddress, p->wMaxPacketSize, address, hub_addr, port, speed);
      if (Endpoints[i].ep != NULL) {
        Endpoints[i].type = USB_ENDPOINT_BULK;
        if (host->activate_endpoint(Endpoints[i].ep, this))
          return;
      }
      break;

    case USB_ENDPOINT_INTERRUPT:
      refcount.fetch_add(1);
      Endpoints[i].ep = new(std::nothrow) USB_Interrupt_Endpoint(p->bEndpointAddress, p->wMaxPacketSize, address, hub_addr, port, speed, p->bInterval);
      if (Endpoints[i].ep != NULL) {
        Endpoints[i].type = USB_ENDPOINT_INTERRUPT;
        if (host->activate_endpoint(Endpoints[i].ep, this))
          return;
      }
      break;

    case USB_ENDPOINT_ISOCHRONOUS:
      if (p->wMaxPacketSize == 0) {
        dprintf("activate_endpoint: ISO endpoint has wMaxPacketSize==0, skipping\n");
        return;
      }
      refcount.fetch_add(1);
      switch (speed) {
        case 0: // full speed
          Endpoints[i].ep = new(std::nothrow) USB_ISO_Full_Endpoint(p->bEndpointAddress, p->wMaxPacketSize, address, hub_addr, port, p->bInterval);
          break;
        case 2: // high speed
           Endpoints[i].ep = new(std::nothrow) USB_ISO_High_Endpoint(p->bEndpointAddress, p->wMaxPacketSize, address, p->bInterval);
          break;
        default: // ???
          dprintf("Unsupported speed for Isochronous endpoint: %d\n", speed);
      }
      if (Endpoints[i].ep != NULL) {
        Endpoints[i].type = USB_ENDPOINT_ISOCHRONOUS;
        if (host->activate_endpoint(Endpoints[i].ep, this))
          return;
      }
      break;

    default:
      dprintf("Unsupported endpoint type: %02X\n", p->bmAttributes);
      return;
  }
  dprintf("Failed to activate endpoint %02X for device %p\n", p->bEndpointAddress, this);
  deref();
}

void USB_Device::activate_interface(uint8_t interface, int altsetting) {
  dprintf("Device<%p> activate interface %02X altSetting %02X\n", this, interface, altsetting);
  std::vector<const usb_endpoint_descriptor*> to_remove;
  std::vector<const usb_endpoint_descriptor*> to_add;
  if (active_config < 0)
    return;

  dev_interface *di = configs.at(active_config).interface(interface);
  if (di == NULL)
    return;

  if (altsetting == di->active)
    return;

  if (di->active >= 0) {
    // gather all the old interface's endpoints
    uint8_t old = (uint8_t)di->active;
    for (uint8_t i=0; i < 30; i++) {
      const usb_endpoint_descriptor* ep = di->getEndpoint(i, old);
      if (ep == NULL)
        break;
      to_remove.push_back(ep);
    }
  }

  if (altsetting >= 0) {
    for (uint8_t i=0; i < 30; i++) {
      const usb_endpoint_descriptor* ep = di->getEndpoint(i,altsetting);
      if (ep == NULL)
        break;
      for (auto ep_it = to_remove.begin(); ep_it != to_remove.end(); ep_it++) {
        if (ep->bLength == (*ep_it)->bLength && memcmp(ep, *ep_it, ep->bLength)==0) {
          // endpoint is the same in old and new interface, don't need to do anything with it
          to_remove.erase(ep_it, ep_it+1);
          ep = NULL;
          break;
        }
      }
      if (ep)
        to_add.push_back(ep);
    }
  }

  // deactivate old endpoints
  for (auto ep_it = to_remove.begin(); ep_it != to_remove.end(); ep_it++) {
    const usb_endpoint_descriptor &ep = **ep_it;
    size_t epi = (ep.bEndpointAddress & 0xF) + ((ep.bEndpointAddress & 0x80) ? 15 : 0);
    deactivate_endpoint(epi-1);
  }

  di->active = altsetting;
  if (altsetting < 0)
    return;

  // set the new interface
  control.Transfer(USB_REQTYPE_INTERFACE_SET, USB_REQ_SET_INTERFACE, altsetting, interface, 0, NULL, this);
  // activate new endpoints
  for (auto ep_it = to_add.begin(); ep_it != to_add.end(); ep_it++) {
    activate_endpoint(*ep_it);
  }
}

void USB_Device::activate_configuration(int configuration) {
  dprintf("Device<%p> activate configuration %d(%d)\n", this, configuration, active_config);
  if (active_config == configuration) return;

  if (active_config >= 0) {
    // disable all currently active interfaces
    const usb_configuration_descriptor *c = configs.at(active_config).getConfiguration();
    for (uint8_t i=0; i < c->bNumInterfaces; i++)
      activate_interface(i, -1);
    active_config = -1;
  }
  if (configs.count(configuration)==0) return;

  control.Transfer(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_CONFIGURATION, configuration, 0, 0, NULL, this);
  active_config = configuration;
  const usb_configuration_descriptor *c = configs.at(active_config).getConfiguration();
  for (uint8_t i=0; i < c->bNumInterfaces; i++)
    activate_interface(i, 0);
}

uint8_t USB_Device::validate_descriptor(const uint8_t* &desc, const uint8_t* const end) {
  int length = end - desc;
  if (length < 2) return 0;
  if (length < desc[0]) {
//    dprintf("validate descriptor %p length %d less than desc_len %u\n", desc, length, desc[0]);
    return 0;
  }
  if (desc[0] == 0) {
//    dprintf("validate_descriptor %p: descriptor length is 0\n", desc);
    return 0;
  }

//  dprintf("validate_descriptor: %p %02X %02X\n", desc, desc[0], desc[1]);

  switch (desc[1]) {
    case USB_DT_DEVICE:
      if (desc[0] == sizeof(usb_device_descriptor)) {
        desc += sizeof(usb_device_descriptor);
        return USB_DT_DEVICE;
      }
      break;
    case USB_DT_CONFIGURATION:
      if (desc[0] == sizeof(usb_configuration_descriptor)) {
        const usb_configuration_descriptor* c = (const usb_configuration_descriptor*)desc;
        // a configuration must have one or more interfaces
        if (c->bNumInterfaces == 0)
          return 0;
        desc += sizeof(usb_configuration_descriptor);
//        dprintf("validate_descriptor %p DT_CONFIGURATION #%d, %d interfaces\n", c, c->bConfigurationValue, c->bNumInterfaces);
        if (length == sizeof(usb_configuration_descriptor)) {
          return USB_DT_CONFIGURATION;
        }
        uint8_t i=0;
        uint8_t alt=0;
        while (desc < end) {
          const usb_interface_descriptor *a = (const usb_interface_descriptor*)desc;
          uint8_t dt = validate_descriptor(desc, end);
          if (dt == 0) break;
          // interfaces must be sorted by bInterfaceNumber then bAlternateSetting
          if (dt == USB_DT_INTERFACE) {
            if (a->bAlternateSetting == 0) {
              if (i++ != a->bInterfaceNumber) break;
              alt = 1;
            } else {
              if (i-1 != a->bInterfaceNumber) break;
              if (alt++ != a->bAlternateSetting) break;
            }
          }
          // endpoint should not occur outside of an interface
          else if (dt == USB_DT_ENDPOINT)
            break;
        }
        if (i == c->bNumInterfaces && desc == end)
          return USB_DT_CONFIGURATION;
        // rewind on failure
        desc = (const uint8_t*)c;
      }
      break;
    case USB_DT_STRING:
      // strings must be an even number of bytes long
      if ((desc[0] & 1)==0) {
        desc += desc[0];
        return USB_DT_STRING;
      }
      break;
    case USB_DT_INTERFACE:
      if (desc[0] == sizeof(usb_interface_descriptor)) {
        const usb_interface_descriptor* c = (const usb_interface_descriptor*)desc;
        desc += sizeof(usb_interface_descriptor);
//        dprintf("validate_descriptor %p DT_INTERFACE #%d(%d), %d endpoints\n", c, c->bInterfaceNumber, c->bAlternateSetting, c->bNumEndpoints);
        if (length == sizeof(usb_interface_descriptor) || c->bNumEndpoints==0) {
          return USB_DT_INTERFACE;
        }
        uint8_t e=0;
        while (1) {
          uint8_t dt = validate_descriptor(desc, end);
          if (dt == 0) break;
          if (dt == USB_DT_ENDPOINT && (++e == c->bNumEndpoints))
            return USB_DT_INTERFACE;
        }
        // rewind on failure
        desc = (const uint8_t*)c;
      }
      break;
    case USB_DT_ENDPOINT:
      // audio endpoints can be 9 bytes long instead of 7
      if (desc[0] >= sizeof(usb_endpoint_descriptor)) {
        const usb_endpoint_descriptor* e = (const usb_endpoint_descriptor*)desc;
//        dprintf("validate_descriptor %p DT_ENDPOINT %02X length %u\n", desc, e->bEndpointAddress, desc[0]);
        // Endpoint address 0 and control endpoint types are not valid
        if ((e->bEndpointAddress & 0xF) != 0 && (e->bmAttributes & 0x3) != 0) {
          desc += desc[0];
          return USB_DT_ENDPOINT;
        }
      }
      break;
    case USB_DT_HUB:
      if (desc[0] >= sizeof(usb_hub_descriptor)) {
        const usb_hub_descriptor* h = (const usb_hub_descriptor*)desc;
        if (h->bNbrPorts <= 7) {
          desc += desc[0];
          return USB_DT_HUB;
        }
      }
      break;
    default:
      uint8_t dt = desc[1];
      desc += desc[0];
      return dt;
  }
  return 0;
}

/* Some control transfers from drivers need to have their return value checked by USB_Device
 * e.g. if the driver changes an interface's altSetting the USB_Device needs to reconfigure
 * the endpoints. So use this callback class to pass the result to USB_Device::callback and
 * then the original callback.
 */
class ControlCallback : public CCallback<usb_control_transfer> {
private:
  const USBCallback driver_cb;
  CCallback<usb_control_transfer>* const device_cb;

  virtual void callback(const usb_control_transfer *t, int result) override {
    if (device_cb)
      device_cb->callback(t, result);
    driver_cb(result);
    delete this;
  }
public:
  ControlCallback(const USBCallback* drv_cb, CCallback<usb_control_transfer>* dev_cb) : driver_cb(*drv_cb), device_cb(dev_cb) {}
};

bool USB_Device::prepare_control_transfer(usb_msg_t &msg) {
  bool proxy = false;
  const uint8_t bmRequestType = msg.device.control.bmRequestType;
  const uint8_t bmRequest = msg.device.control.bmRequest;
  const uint16_t wValue = msg.device.control.wValue;
  const uint16_t wIndex = msg.device.control.wIndex;

  uint16_t rt_rq = MK_BE16(bmRequestType, bmRequest);
  switch (rt_rq) {
    // change device address is not allowed
    case MK_BE16(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS):
      errno = EINVAL;
      return false;
    case MK_BE16(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR):
      if ((wValue >> 8) == USB_DT_STRING)
        proxy = true;
      break;
    case MK_BE16(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_CONFIGURATION):
      // TODO: allow this from device drivers but not interface drivers
      errno = EINVAL;
      return false;
    case MK_BE16(USB_REQTYPE_INTERFACE_GET, USB_REQ_GET_INTERFACE):
    case MK_BE16(USB_REQTYPE_INTERFACE_SET, USB_REQ_SET_INTERFACE):
      proxy = true;
      break;
    case MK_BE16(USB_REQTYPE_ENDPOINT_SET, USB_REQ_CLEAR_FEATURE):
      if (wValue == USB_FEATURE_ENDPOINT_HALT) {
        if (wIndex == 0) { // CLEAR_HALT is invalid for endpoint 0
          errno = EINVAL;
          return false;
        }
        proxy = true;
      }
      break;
  }

  msg.device.control_cb = new ControlCallback(msg.device.cb, proxy ? this : NULL);
  return true;
}

void USB_Device::ControlTransfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, CCallback<usb_control_transfer>* cb) {
  if (control.Transfer(bmRequestType, bmRequest, wValue, wIndex, wLength, data, cb) < 0) {
    cb->callback(NULL, -errno);
  }
}

void USB_Device::BulkTransfer(uint8_t bEndpoint, uint32_t dLength, void *data, const USBCallback* cb) {
  if (Endpoints[bEndpoint].type == USB_ENDPOINT_BULK) {
    if (Endpoints[bEndpoint].ep->BulkTransfer(dLength, data, cb) == 0)
      return;
  }
  else errno = ENXIO;
  (*cb)(-errno);
}

void USB_Device::InterruptTransfer(uint8_t bEndpoint, uint32_t dLength, void *data, const USBCallback* cb) {
  if (Endpoints[bEndpoint].type == USB_ENDPOINT_INTERRUPT) {
    if (Endpoints[bEndpoint].ep->InterruptTransfer(dLength, data, cb) == 0)
      return;
  }
  else errno = ENXIO;
  (*cb)(-errno);
}

void USB_Device::IsochronousTransfer(uint8_t bEndpoint, isolength& Lengths, void *data, const USBCallback* cb) {
  if (Endpoints[bEndpoint].type == USB_ENDPOINT_ISOCHRONOUS) {
    if (Endpoints[bEndpoint].ep->IsochronousTransfer(Lengths, data, cb) == 0)
      return;
  }
  else errno = ENXIO;
  (*cb)(-errno);
}

bool USB_Device::pushMessage(usb_msg_t& msg) {
  switch (msg.type) {
    case USB_MSG_DEVICE_CONTROL_TRANSFER:
      if (prepare_control_transfer(msg) == false)
        return false;
    case USB_MSG_DEVICE_BULK_TRANSFER:
    case USB_MSG_DEVICE_INTERRUPT_TRANSFER:
    case USB_MSG_DEVICE_ISOCHRONOUS_TRANSFER:
      msg.device.dev = this;
      refcount.fetch_add(1);
    default:
      break;
  }
  return host->putMessage(msg);
}

USB_Driver::Factory::Factory() {
  next = gList;
  gList = this;
}

USB_Driver::Factory::~Factory() {
  USB_Driver::Factory** p = &gList;
  while (*p != NULL) {
    if (*p == this) {
      *p = next;
      return;
    }
    p = &((*p)->next);
  }
}

USB_Driver::Factory* USB_Driver::Factory::find_driver(const usb_device_descriptor *dd, const usb_configuration_descriptor *cd) {
  USB_Driver::Factory *f = gList;
  while (f) {
    if (f->offer(dd, cd) == true)
      return f;
    f = f->next;
  }
  return NULL;
}

USB_Driver::Factory* USB_Driver::Factory::find_driver(const usb_interface_descriptor *id, size_t length) {
  USB_Driver::Factory *f = gList;
  while (f) {
    if (f->offer(id, length) == true)
      return f;
    f = f->next;
  }
  return NULL;
}

USB_Driver::Factory* USB_Driver::Factory::gList;

USB_Driver::USB_Driver() {
  device = NULL;
  dprintf("new USB_Driver %p\n", this);
}

int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback* cb_func) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_CONTROL_TRANSFER,
    .device = {
    .cb = cb_func,
      .control = {
        .bmRequestType = bmRequestType,
        .bmRequest = bmRequest,
        .wValue = wValue,
        .wIndex = wIndex,
        .wLength = wLength,
        .data = data
    }
  }
  };
  if (device == NULL) errno = ENOENT;
  else {
    if (device->pushMessage(msg))
      return 0;
    errno = ENOMEM;
  }
  return -1;
}

int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback &cb_func) {
  // control callback is always copied (see pushMessage) so don't need to clone it
  return ControlMessage(bmRequestType,bmRequest,wValue,wIndex,wLength,data,&cb_func);
}

int USB_Driver::BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, const USBCallback* cb_func) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_BULK_TRANSFER,
    .device = {
      .cb = cb_func,
      .bulkintr = {
        .bEndpoint = bEndpoint,
        .dLength = dLength,
        .data = data
      }
    }
  };
  if (device == NULL) errno = ENOENT;
  else {
    if (device->pushMessage(msg))
      return 0;
    errno = ENOMEM;
  }
  return -1;
}

int USB_Driver::InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, const USBCallback* cb_func) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_INTERRUPT_TRANSFER,
    .device = {
    .cb = cb_func,
      .bulkintr = {
        .bEndpoint = bEndpoint,
        .dLength = wLength,
        .data = data
      }
    }
  };
  if (device == NULL) errno = ENOENT;
  else {
    if (device->pushMessage(msg))
      return 0;
    errno = ENOMEM;
  }
  return -1;
}

int USB_Driver::IsochronousMessage(uint8_t bEndpoint, isolength& Lengths, void *data, const USBCallback* cb_func) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_ISOCHRONOUS_TRANSFER,
    .device = {
      .cb = cb_func,
      .iso = {
        .bEndpoint = bEndpoint,
        .lengths = &Lengths,
        .data = data
      }
    }
  };
  if (device == NULL) errno = ENOENT;
  else if (Lengths[0] == 0) errno = EINVAL;
  else {
    if (device->pushMessage(msg))
      return 0;
    errno = ENOMEM;
  }
  return -1;
}

class FuncWrapper {
private:
  const USBCallback base;
public:

  const USBCallback wrap = [=](int r) {
    base(r);
    delete this;
  };
  FuncWrapper(USBCallback& b) : base(std::move(b)) {}
};

int USB_Driver::BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, USBCallback cb_func) {
  FuncWrapper* cb = new (std::nothrow) FuncWrapper(cb_func);
  if (cb == NULL) {
    errno = ENOMEM;
  } else {
    if (BulkMessage(bEndpoint, dLength, data, &cb->wrap) >= 0)
      return 0;
    delete cb;
  }
  return -1;
}

int USB_Driver::InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, USBCallback cb_func) {
  FuncWrapper* cb = new (std::nothrow) FuncWrapper(cb_func);
  if (cb == NULL) {
    errno = ENOMEM;
  } else {
    if (InterruptMessage(bEndpoint, wLength, data, &cb->wrap) >= 0)
      return 0;
    delete cb;
  }
  return -1;
}

int USB_Driver::IsochronousMessage(uint8_t bEndpoint, isolength& Lengths, void *data, USBCallback cb_func) {
  FuncWrapper* cb = new (std::nothrow) FuncWrapper(cb_func);
  if (cb == NULL) {
    errno = ENOMEM;
  } else {
    if (IsochronousMessage(bEndpoint, Lengths, data, &cb->wrap) >= 0)
      return 0;
    delete cb;
  }
  return -1;
}

// synchronous functions - not implemented here, these use weak symbols so they can be overridden using OS specific code
__attribute__((weak)) int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::BulkMessage(uint8_t,uint32_t,void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::InterruptMessage(uint8_t,uint16_t,void*) {
  errno = ENOSYS;
  return -1;
}

__attribute__((weak)) int USB_Driver::IsochronousMessage(uint8_t,isolength&,void*) {
  errno = ENOSYS;
  return -1;
}

USB_Host::Enum_Control::Enum_Control() : USB_Control_Endpoint(64, 0, 0, 0, 0) {
  // first pipe, link to self
  set_link(this);
  capabilities.H = 1;

  in_progress = false;
}

void USB_Host::Enum_Control::set(USB_Hub& h, uint8_t p, uint8_t s) {
  sync_before_read();
  capabilities.hub = h.hub_addr;
  capabilities.port = h.base_port(p);
  capabilities.speed = s;
  capabilities.C = (s < 2) ? 1:0;
  clean_after_write();

  hub = &h;
  port = p;
  speed = s;
  address_retry = 0;
  h.addref();
}

void USB_Host::Enum_Control::clean(void) {
  if (in_progress == false) return;

  dprintf("Enumeration failed, disabling port\n");
  in_progress = false;
  if (hub && hub->port[port].state != 3) {
    hub->port[port].state = 3;
    hub->port_enable(port, false);
    hub->deref();
  }
  flush();
}

void USB_Host::callback(const usb_control_transfer* t, int result) {
  if (Enum.in_progress == false) return;

  USB_Hub &hub = *Enum.hub;
  uint8_t port = Enum.port;

  uint32_t state = hub.port[port].state;
  if (state < 4) return;

  if (result >= 0) {
    if (t == NULL) {
      dprintf("!!! Enumeration begin !!!\n");
      Enum.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, (USB_DT_DEVICE << 8), 0, sizeof(usb_device_descriptor), NULL, this);
      return;
    }

    uint16_t rt_rq = MK_BE16(t->getbmRequestType(), t->getbmRequest());

    if (rt_rq == MK_BE16(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS) && t->getwValue()!=0) {
      // address has been set, basic enumeration is complete
      hub.port[port].state = 6;
      hub.deref();
      Enum.in_progress = false;
      dprintf("Port %d:%d successfully set address %d\n", hub.hub_addr, port, t->getwValue());
      /* schedule device to begin initializing after set_address timeout
        * (this should also give time for the control endpoint to be activated)
        */
      usb_msg_t msg = {
        .type = USB_MSG_DEVICE_INIT,
        .device = {
          .dev = hub.port[port].device
        }
      };
      timerMsg(msg,20);
      return;
    }

    if (rt_rq == MK_BE16(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR)) {
      const usb_device_descriptor *desc = (const usb_device_descriptor*)t->getBuffer();
      if (result>=8 && desc->bDescriptorType == USB_DT_DEVICE && desc->bLength == sizeof(usb_device_descriptor)) {
        dprintf("Port %d:%d bDeviceClass %02X bDeviceSubClass %02X bDeviceProtocol %02X bMaxPacketSize %d\n", hub.hub_addr, port, desc->bDeviceClass, desc->bDeviceSubClass, desc->bDeviceProtocol, desc->bMaxPacketSize);
        Enum.bMaxPacketSize = desc->bMaxPacketSize;
        /* this particular device doesn't respond properly to the first SET_ADDRESS request (doesn't return an error but doesn't switch addresses)
          * It has a 16-byte packet control endpoint, so VID/PID/bcdDevice will be filled (enumerator uses wMaxPacketSize==64)
          * Workaround: send a request to set address to 0, doesn't matter if it accepts it or not. Also safe for any other devices that end up here.
          */
        if (result>=16 && desc->idVendor==0x0A12 && desc->idProduct==0x0001 && desc->bcdDevice==0x0134) {
          Enum.Transfer(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS, 0, 0, 0, NULL, this);
          return;
        }
      } else {
        Enum.clean();
        return;
      }
    }

    uint8_t address = get_address();
    if (address > 0) {
      USB_Device *d = new(std::nothrow) USB_Device(this, hub.hub_addr, hub.base_port(port), Enum.speed, address, Enum.bMaxPacketSize);
      if (d != NULL) {
        hub.port[port].device = d;
        // set the address (possibly for the second time)
        Enum.Transfer(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS, address, 0, 0, NULL, this);
        return;
      }
      release_address(address);
    }
  } else {
    if (t && result==-EPIPE) {
      if (Enum.address_retry == 0) { // first failure - retry at timeout
        uint64_t s = MK_SETUP64(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS, 0, 0, 0);
        if ((t->getSetupData() & 0xFFFF0000FFFFFFFFllu) == s) {
          Enum.address_retry = t->getwValue();
          return;
        }
      } // second failure - give up
    } else if (result==-EBUSY && Enum.address_retry!=0) {
      // reset timeout
      set_port_timeout(*Enum.hub, Enum.port, 500);
      Enum.Transfer(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS, Enum.address_retry, 0, 0, NULL, this);
      return;
    }
  }

  Enum.clean();
}

uint8_t USB_Host::get_address(void) {
  for (uint8_t i=1; i < 128; i++)
  {
    if (addresses[i>>5] & (1 << (i&31)))
      continue;
    addresses[i>>5] |= 1 << (i&31);
    return i;
  }
  return 0;
}

void USB_Host::release_address(uint8_t i) {
  if (i != 0)
    addresses[i>>5] &= ~(1<<(i&31));
}

bool USB_Host::set_port_timeout(USB_Hub& hub, uint8_t port, uint32_t ms) {
  usb_msg_t msg = {
    .type = USB_MSG_PORT_TIMEOUT,
    .port = {
      .hub = &hub,
      .port = port
    }
  };
  uint32_t start = getMillis();
  if (start==0) start = 1;
  uint32_t oldstart = hub.port[port].timeout_start;
  hub.port[port].timeout_start = start;
  if (oldstart == 0) {
    if (timerMsg(msg, ms) == false)
      return false;
    hub.addref();
  } else {
    dprintf("Adjusted timer start time\n");
  }
  return true;
}

void USB_Host::port_status(USB_Hub& hub, uint8_t port, port_status_t status) {
  if (status.ENABLE == 0) {
    if (hub.port[port].device) {
      dprintf("Device %p disconnected\n", hub.port[port].device);
      hub.port[port].device->disconnect();
      hub.port[port].device = NULL;
    }
    /* if port was active it might have had an overcurrent event or otherwise
      * unintentionally disabled itself without a disconnection, set the state
      * as if it was disconnected to restart enumeration
      */
    if (hub.port[port].state == 6)
      hub.port[port].state = 1;
    // else don't change state, port was disabled deliberately (e.g. enumeration failure)
  }
  if (status.CONNECTION == 0) {
    if (hub.port[port].state >= 3 && hub.port[port].state < 6) {
      Enum.clean();
    }
    hub.port[port].state = 1;
  }
  if (status.POWER == 0) {
    hub.port[port].state = 0;
  }

  switch (hub.port[port].state) {
    case 0: // unpowered
      if (status.POWER) { // it's powered now, continue to stage 1
        hub.port[port].state = 1;
        dprintf("Port %d:%d Power ON\n", hub.hub_addr, port);
      } else { // else stay unpowered for 1 second before turning back on
        set_port_timeout(hub, port, 1000);
        break;
      }
    case 1: // powered, disconnected
      hub.phySetHighSpeed(port, false);
      if (status.CONNECTION) { // something has connected, start 100ms debounce
        dprintf("Port %d:%d starting debounce\n", hub.hub_addr, port);
        hub.port[port].state = 2;
        set_port_timeout(hub, port, 100);
      }
      break;
    case 3: // port has been reset
      if (status.ENABLE) { // begin 10ms recovery
        dprintf("Port %d:%d starting recovery\n", hub.hub_addr, port);
        hub.port[port].state = 4;
        Enum.set(hub, port, status.HIGH_SPEED ? 2 : status.LOW_SPEED);
        if (status.LOW_SPEED==0 && status.HIGH_SPEED==1)
          hub.phySetHighSpeed(port, true);
        set_port_timeout(hub, port, 10);
      }
      break;
  }
}

void USB_Host::port_timeout(usb_msg_t& msg) {
  USB_Hub& hub = *msg.port.hub;
  uint8_t port = msg.port.port;
  uint32_t now = getMillis();
  uint32_t start = hub.port[port].timeout_start;
  uint32_t ticked = now - start;
  hub.port[port].timeout_start = 0;

  switch (hub.port[port].state) {
    case 0: // port is unpowered: 1 second timer before repowering
      if (ticked >= 1000) {
        dprintf("Port %d:%d finished power timeout\n", hub.hub_addr, port);
        hub.port_power(port, true);
        hub.port[port].state = 1;
        return;
      } else {
        ticked = 1000-ticked;
      }
      break;
    case 2: // device is connected, 100ms debounce
      if (ticked >= 100) {
        if (Enum.in_progress == false) {
          // reset the port
          dprintf("Port %d:%d finished debounce, starting reset\n", hub.hub_addr, port);
          hub.port[port].state = 3;
          hub.port_reset(port, true);
          Enum.in_progress = true;
          return;
        } // else try again in 100ms
        dprintf("Port %d:%d finished debounce but Enumerator is busy, delaying...\n", hub.hub_addr, port);
        ticked = 100;
        start = now;
      } else {
        ticked = 100-ticked;
      }
      break;
    case 4: // 10ms recovery
      if (ticked >= 10) {
        dprintf("Port %d:%d finished recovery\n", hub.hub_addr, port);
        hub.port[port].state = 5;
        set_port_timeout(hub, port, 500);
        callback(NULL, 0);
        return;
      }
      ticked = 10-ticked;
      break;
    case 5: // enumeration stages, must complete in < 500ms
      if (ticked >= 500) {
        dprintf("Port %d:%d enumeration timeout\n", hub.hub_addr, port);
        callback(NULL,-EBUSY);
        return;
      }
      ticked = 500-ticked;
      break;
    default:
      return;
  }

  if (timerMsg(msg, ticked) == true) {
    hub.addref();
    // restore original start time
    hub.port[port].timeout_start = start;
  }
}

void USB_Host::update_transfers(void) {
  USB_Endpoint *ep = endpoints;
  while (ep) {
    ep->update();
    ep = ep->host_next;
  }
}

void USB_Host::add_async_queue(USB_Async_Endpoint *ep) {
  Enum.sync_before_read();
  ep->set_link(Enum.get_link());
  // this is typically where an endpoint is first activated so flush the entire queue head
  Enum.set_link(ep);
  Enum.clean_after_write();
  ep->host_next = endpoints;
  endpoints = ep;
}

bool USB_Host::remove_async_queue(USB_Async_Endpoint *ep) {
  dprintf("Removing async USB_Endpoint %p\n", ep);

  USB_Async_Endpoint* prev = &Enum;
  while (prev->get_link() != ep) {
    if (prev->get_link() == &Enum) {
      dprintf("Can't remove USB_Async_Endpoint %p, not found in async list\n", ep);
      return false;
    }
    prev = prev->get_link();
  }
  prev->set_link(ep->get_link());

  // remove from active endpoint list
  USB_Endpoint **p = &endpoints;
  while (*p != NULL) {
    if (*p == ep) {
      *p = ep->host_next;
      break;
    }
    p = &(*p)->host_next;
  }
  // put at end of cleanup list
  ep->host_next = NULL;
  p = &async_cleanup;
  while (*p != NULL)
    p = &(*p)->host_next;
  *p = ep;
  // ring doorbell
  EHCI->USBCMD |= USB_USBCMD_IAA;
  return true;
}

void USB_Host::notify_endpoint_removed(USB_Endpoint *ep) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_ENDPOINT_REMOVED,
    .device = {
      .dev = ep->device,
      .endpoint = ep
    }
  };
  putMessage(msg);
}

void USB_Host::unschedule_periodic(void) {
  if (periodic_cleanup == NULL) return;
  USB_Periodic_Endpoint *ep = static_cast<USB_Periodic_Endpoint*>(periodic_cleanup);

  // low/full speed endpoints must be inactive to ensure split transactions aren't interrupted
  if (ep->set_inactive() == true)
    return;

  uint8_t s_mask, c_mask;
  ep->get_masks(s_mask, c_mask);

  uint8_t stime = ep->stime;
  uint8_t ctime = ep->ctime;
  const uint32_t f_interval = (ep->interval > 8) ? (ep->interval>>3) : 1; // uframes -> frames

  for (uint32_t i=ep->offset >> 3; i < PERIODIC_LIST_SIZE; i += f_interval) {
    for (uint8_t j=0; j < 8; j++) {
      // assumes s_mask and c_mask don't overlap
      if (s_mask & (1<<j)) {
        uframe_bandwidth[i*8+j] -= stime;
        dprintf("bandwidth[%u:%u] = %u\n", i, j, uframe_bandwidth[i*8+j]);
      }
      if (c_mask & (1<<j)) {
        uframe_bandwidth[i*8+j] -= ctime;
        dprintf("bandwidth[%u:%u] = %u\n", i, j, uframe_bandwidth[i*8+j]);
      }
    }
  }

  // endpoint has been removed from schedule and the frame index has rolled over
  // so it should be completely safe to remove now
  dprintf("unschedule: safe to remove %p\n", ep);
  periodic_cleanup = ep->host_next;
  notify_endpoint_removed(ep);
}

uint32_t PeriodicScheduler::current_uframe(void) {
  return FRINDEX % (PERIODIC_LIST_SIZE*8);
}

bool PeriodicScheduler::add_node(uint32_t frame, uint32_t link_to, uint32_t interval) {
  uint32_t *hlink = (uint32_t*)(link_to & ~0x1F);
  uint32_t *h = periodictable+frame;
  uint32_t hnext = *hlink;
  uint32_t next;
  while (next = *h, (next & 1)==0) {
    if (next == link_to) {
      // found node already in this frame, nothing to insert
      return false;
    }
    if (next == hnext) {
      // h points to the same node as n so point h at n
      break;
    }
    // insert in front of queue heads with smaller intervals, otherwise keep traversing
    if ((next & 0x1F) == 2) { // is a queue head pointer
      auto qh = (const usb_queue_head_t*)(next & ~0x1F);
      uint32_t next_interval = static_cast<const USB_Interrupt_Endpoint*>(qh)->interval;
      if (next_interval < interval)
        break;
    }
    // follow horizontal link
    h = (uint32_t*)(next & ~0x1F);
  }

  // insert node into tree
  if (hnext == 1) {
    // update the endpoint's horizontal link only once - all following nodes repeat with the same interval
    *hlink = next;
    cache_flush_invalidate(hlink);
  } else if (next != hnext) {
    dprintf("ERROR: node %p links to %08X instead of %08X! (frame %u)", h, next, hnext, frame);
  }
  cache_invalidate(h);
  *h = link_to;
  cache_flush_invalidate(h);
  return true;
}

bool PeriodicScheduler::remove_node(uint32_t frame, uint32_t link_to, uint32_t hnext) {
  uint32_t *h = periodictable+frame;
  uint32_t next;
  while (next = *h, (next & 1)==0) {
    if (next == hnext) {
      // node was already removed
      break;
    }
    if (next == link_to) {
      // unlink node from this frame
      cache_invalidate(h);
      *h = hnext;
      cache_flush_invalidate(h);
      return true;
    }
    // follow horizontal link
    h = (uint32_t*)(next & ~0x1F);
  }
  return false;
}

bool USB_Host::calculate_offset(const USB_Periodic_Endpoint *ep, uint32_t &offset) {
  const uint32_t interval = ep->interval;
  const uint32_t stime = ep->stime;
  const uint32_t ctime = ep->ctime;
  uint8_t s_mask, c_mask;
  ep->get_masks(s_mask, c_mask);

  uint32_t best_bandwidth = 0xFFFFFFFF;
  // for each possible uframe offset, find the worst uframe_bandwidth
  for (uint32_t off=0; off < interval; off++) {
    /* TODO: host must not schedule a start-split for uframe 6+8X
     * so this function needs to know the endpoint speed to
     * know when start-splits are in use...
     */

    // low/full speed can't split across frames
    if ((s_mask << (off&7)) >= 256) {
//      dprintf("offset %u not allowed due to s_mask\n", off);
      continue;
    }
    if ((c_mask << (off&7)) >= 256) {
//      dprintf("offset %u not allowed due to c_mask\n", off);
      continue;
    }
    uint32_t max_bandwidth = 0;
    for (uint32_t i=off; i < PERIODIC_LIST_SIZE*8; i += (interval > 8 ? interval:8)) {
      uint32_t umax = max_bandwidth;
      for (uint8_t j=0; j < 8; j++) {
        if ((s_mask | c_mask) & (1<<j)) {
          if ((i+j) >= PERIODIC_LIST_SIZE*8) {
            // frame rollover not allowed
//            dprintf("offset %u not allowed due to frame rollover\n", i+j);
            umax = 0xFFFFFFFF;
            break;
          }
          uint32_t bw = uframe_bandwidth[i+j];
          if (s_mask & (1<<j))
            bw += stime;
          if (c_mask & (1<<j))
            bw += ctime;
          if (bw > umax) umax = bw;
        }
      }
      if (umax > max_bandwidth) max_bandwidth = umax;
    }
    // remember which uframe offset is the best
    // uses "<=" over "<" to favor last possible offset
    if (max_bandwidth <= best_bandwidth) {
      best_bandwidth = max_bandwidth;
      offset = off;
    }
  }
  // fail if the best found needs more than 80% (234 * 0.8) in any uframe
  if (best_bandwidth > 187) {
    dprintf("calculate_bandwidth for endpoint %p failed: best_bandwidth %u, offset %u, interval %u\n", ep, best_bandwidth, offset, ep->interval);
    return false;
  }
  return true;
}

void USB_Host::add_periodic_queue(USB_Periodic_Endpoint *ep) {
  uint32_t offset;
  if (calculate_offset(ep, offset) == false) {
    // no bandwidth available - tell the device it is inactive
    notify_endpoint_removed(ep);
    return;
  }

  ep->activate(offset, this);

  const uint32_t interval = ep->interval;
  const uint32_t f_interval = (interval > 8) ? (interval>>3) : 1;

  uint8_t stime = ep->stime;
  uint8_t ctime = ep->ctime;
  uint8_t s_mask, c_mask;
  ep->get_masks(s_mask, c_mask);
  dprintf("Endpoint<%p> interval %lu offset %lu stime %lu ctime %lu s_mask %02X c_mask %02X\n", ep, interval, offset, stime, ctime, s_mask, c_mask);

  /* use the interval and offset to insert the endpoint into the periodic schedule,
    * and s_mask+c_mask to update uframe_bandwidth
    * (interval and offset are milliseconds/frames)
    */
  for (uint32_t i=offset>>3; i < PERIODIC_LIST_SIZE; i += f_interval) {
    // update uframe_bandwidth
    for (uint32_t j=0; j < 8; j++) {
      uint32_t mask = 1<<j;
      if (mask & s_mask) {
        uframe_bandwidth[i*8+j] += stime;
        dprintf("Updated frame %lu/%lu bandwidth for stime %u: %u\n", i, j, stime, uframe_bandwidth[i*8+j]);
      }
      if (mask & c_mask) {
        uframe_bandwidth[i*8+j] += ctime;
        dprintf("Updated frame %lu/%lu bandwidth for ctime %u: %u\n", i, j, ctime, uframe_bandwidth[i*8+j]);
      }
    }
  }

  // put in host's active list of endpoints
  ep->host_next = endpoints;
  endpoints = ep;
}

bool USB_Host::remove_periodic_queue(USB_Periodic_Endpoint *ep) {
  dprintf("Removing periodic USB_Endpoint %p\n", ep);
  // first remove it from the host's active endpoint list
  USB_Endpoint **b = &endpoints;
  while (*b != ep) {
    if (*b == NULL) return false;
    b = &(*b)->host_next;
  }
  *b = ep->host_next;

  // add to end of cleanup list
  ep->host_next = NULL;
  b = &periodic_cleanup;
  while (*b != NULL) {
    b = &(*b)->host_next;
  }
  *b = ep;

  // begin cleanup if it's not already scheduled
  if ((EHCI->USBINTR & USB_USBINTR_FRE) == 0) {
    unschedule_periodic();
    EHCI->USBINTR |= USB_USBINTR_FRE;
  }
  return true;
}

void USB_Host::port_power(uint8_t port, bool set) {
  dprintf("Port %d:%d Power %s\n", hub_addr, port, set ? "on":"off");
  if (set)
    EHCI->PORTSC[port] |= USB_PORTSC1_PP;
  else
    EHCI->PORTSC[port] &= ~USB_PORTSC1_PP;
}

void USB_Host::port_reset(uint8_t port, bool set) {
  dprintf("Port %d:%d Reset %s\n", hub_addr, port, set ? "on" : "off");
  if (set)
    EHCI->PORTSC[port] = (EHCI->PORTSC[port] | USB_PORTSC1_PR) & ~USB_PORTSC1_PE;
  else
    EHCI->PORTSC[port] &= ~USB_PORTSC1_PR;
}

void USB_Host::port_enable(uint8_t port, bool set) {
  dprintf("Port %d:%d Enable %s\n", hub_addr, port, set ? "on" : "off");
  if (set)
    EHCI->PORTSC[port] |= USB_PORTSC1_PE;
  else
    EHCI->PORTSC[port] &= ~USB_PORTSC1_PE;
}

USB_Host::USB_Host(usb_ehci_base_t* const base) :
USB_Hub(0),
PeriodicScheduler(((usb_ehci_cmd_t*)((uint8_t*)base + base->CAPLENGTH))->FRINDEX),
EHCI((usb_ehci_cmd_t*)((uint8_t*)base + base->CAPLENGTH)),
nPorts(base->HCSPARAMS.N_PORTS)
{
  memset(addresses, 0, sizeof(addresses));
  // reserve address 0
  addresses[0] = 1;
  endpoints = &Enum;
  async_cleanup = NULL;
  periodic_cleanup = NULL;
}

void USB_Host::usb_process(void) {
  dprintf("begin ehci reset (%d root ports)", nPorts);
  EHCI->USBCMD |= USB_USBCMD_RST;
  while (EHCI->USBCMD & USB_USBCMD_RST) {
    dprintf(".");
  }
  dprintf(" done\n");

  memset(uframe_bandwidth, 0, sizeof(uframe_bandwidth));
  EHCI->USBINTR = 0;
  setHostMode();
  EHCI->PERIODICLISTBASE = periodictable;
  EHCI->FRINDEX = 0;
  dprintf("Enumeration QH: %p\n", static_cast<usb_queue_head_t*>(&Enum));
  Enum.clean_after_write();
  EHCI->ASYNCLISTADDR = (uint32_t)static_cast<usb_queue_head_t*>(&Enum);
  uint32_t usbcmdinit = 0;
#if (PERIODIC_LIST_SIZE >= 8 && PERIODIC_LIST_SIZE < 128)
    usbcmdinit |= USB_USBCMD_FS_2 | USB_USBCMD_FS_1((64 - PERIODIC_LIST_SIZE) / 17);
#elif (PERIODIC_LIST_SIZE >= 128 && PERIODIC_LIST_SIZE <= 1024)
    usbcmdinit |= USB_USBCMD_FS_1((1024 - PERIODIC_LIST_SIZE) / 257);
#else
#error "Unsupported PERIODIC_LIST_SIZE value"
#endif
  dprintf("USBCMD (FS): %08lX\n", usbcmdinit);
  // ensure other registers are written before CMD is updated
  mem_sync();
  EHCI->USBCMD = usbcmdinit | USB_USBCMD_ITC(0) | USB_USBCMD_RS | USB_USBCMD_ASP(3) | USB_USBCMD_ASPE | USB_USBCMD_PSE | USB_USBCMD_ASE;

  EHCI->CONFIGFLAG = 1;
  // turn on ports, set their stage to 1
  for (uint8_t i=0; i < nPorts; i++) {
    port_power(i, true);
    port[i].state = 1;
  }
  // wait 20ms for ports to power up
  usleep(20000);

  EHCI->USBINTR |= USB_USBINTR_AAE | USB_USBINTR_SEE | USB_USBINTR_PCE | USB_USBINTR_UEE | USB_USBINTR_UE;

  dprintf("USB: beginning main loop\n");

  while (1) {
    usb_msg_t msg;
    if (getMessage(msg)==false)
      break;

    switch (msg.type) {
      case USB_MSG_INTERRUPT:
        do {
          uint32_t state = EHCI->USBSTS;
          state &= EHCI->USBINTR;
          EHCI->USBSTS = state;
          if (0) {
            dprintf("ISR: %08lX", state);
            if (state & USB_USBSTS_FRI) dprintf(" Frame Rollover");
            if (state & USB_USBSTS_AAI) dprintf(" Async Advance");
            if (state & USB_USBSTS_SEI) dprintf(" System Error");
            if (state & USB_USBSTS_PCI) dprintf(" Port Change");
            if (state & USB_USBSTS_UEI) dprintf(" USB Error");
            if (state & USB_USBSTS_UI) dprintf(" USB Interrupt");
            dprintf("\n");
          }

          if (state & USB_USBSTS_SEI) {
            dprintf("USB SYSTEM ERROR\n");
            while (1);
          }

          if (state & (USB_USBSTS_UI|USB_USBSTS_UEI))
            update_transfers();

          if (state & USB_USBSTS_PCI) {
            // update all ports on root hub
            for (uint8_t i=0; i < nPorts; i++) {
              uint32_t status = EHCI->PORTSC[i];
              EHCI->PORTSC[i] = status | (USB_PORTSC1_OCC|USB_PORTSC1_PEC|USB_PORTSC1_CSC);
              dprintf("Root Port %d status: %08lX\n", i, status);
              port_status_t pstatus = {0};
              if (status & USB_PORTSC1_CCS)
                pstatus.CONNECTION = 1;
              if (status & USB_PORTSC1_PE)
                pstatus.ENABLE = 1;
              if (status & USB_PORTSC1_SUSP)
                pstatus.SUSPEND = 1;
              if (status & USB_PORTSC1_OCA)
                pstatus.OVER_CURRENT = 1;
              if (status & USB_PORTSC1_PR)
                pstatus.RESET = 1;
              if (status & USB_PORTSC1_PP)
                pstatus.POWER = 1;
              if (status & USB_PORTSC1_PSPD(1))
                pstatus.LOW_SPEED = 1;
              else if (status & USB_PORTSC1_PSPD(2))
                pstatus.HIGH_SPEED = 1;
              if (status & USB_PORTSC1_PTC(0xF))
                pstatus.TEST = 1;
              if (status & USB_PORTSC1_PIC(3))
                pstatus.INDICATOR = 1;
              port_status(*this, i, pstatus);
            }
          }

          if (state & USB_USBSTS_AAI) {
            /* cleanup ONE endpoint from the async cleanup list.
            * Multiple endpoints may have been queued for cleanup at the same time
            * meaning the doorbell may be rung multiple times before it gets answered,
            * we need to be sure the async schedule has advanced after every ring.
            */
            USB_Endpoint *p = async_cleanup;
            if (p->host_next) {
              // ring again for the next cleanup
              EHCI->USBCMD |= USB_USBCMD_IAA;
            }
            async_cleanup = p->host_next;
            notify_endpoint_removed(p);
          }

          if (state & USB_USBSTS_FRI) {
            EHCI->USBINTR &= ~USB_USBINTR_FRE;
            unschedule_periodic();
            // re-enable the interrupt if there's more to cleanup
            if (periodic_cleanup != NULL) {
              EHCI->USBINTR |= USB_USBINTR_FRE;
              mem_sync();
            }
          }
        } while(0);
        nextIRQ();
        continue;
      case USB_MSG_ADDRESS_RELEASED:
        release_address(msg.address);
        continue;
      case USB_MSG_PORT_STATUS:
        port_status(*msg.port.hub, msg.port.port, msg.port.status);
        msg.port.hub->deref();
        continue;
      case USB_MSG_PORT_TIMEOUT:
        port_timeout(msg);
        msg.port.hub->deref();
        continue;
      case USB_MSG_ENDPOINT_ACTIVATE:
        msg.endpoint.ep->device = msg.endpoint.device;
        switch (msg.endpoint.ep->ep_type) {
          case USB_ENDPOINT_INTERRUPT:
          case USB_ENDPOINT_ISOCHRONOUS:
            add_periodic_queue(static_cast<USB_Periodic_Endpoint*>(msg.endpoint.ep));
            break;
          case USB_ENDPOINT_BULK:
          case USB_ENDPOINT_CONTROL:
            add_async_queue(static_cast<USB_Async_Endpoint*>(msg.endpoint.ep));
            break;
          default:
            dprintf("Don't know how to activate endpoint %p, type is %d\n", msg.endpoint.ep, msg.endpoint.ep->ep_type);
            notify_endpoint_removed(msg.endpoint.ep);
            break;
        }
        continue;
      case USB_MSG_ENDPOINT_DEACTIVATE:
        switch (msg.endpoint.ep->ep_type) {
          case USB_ENDPOINT_INTERRUPT:
          case USB_ENDPOINT_ISOCHRONOUS:
            if (remove_periodic_queue(static_cast<USB_Periodic_Endpoint*>(msg.endpoint.ep)) == true)
              continue;
            break;
          case USB_ENDPOINT_CONTROL:
          case USB_ENDPOINT_BULK:
            if (remove_async_queue(static_cast<USB_Async_Endpoint*>(msg.endpoint.ep)) == true)
              continue;
            break;
          default:
            dprintf("Don't know how to deactivate endpoint %p, type is %d\n", msg.endpoint.ep, msg.endpoint.ep->ep_type);
        }
        // removal could not be queued, send notification now
        notify_endpoint_removed(msg.endpoint.ep);
        continue;
      case USB_MSG_DEVICE_INIT:
      case USB_MSG_DEVICE_ENDPOINT_REMOVED:
      case USB_MSG_DEVICE_FIND_DRIVER:
      case USB_MSG_DEVICE_CONTROL_TRANSFER:
      case USB_MSG_DEVICE_BULK_TRANSFER:
      case USB_MSG_DEVICE_INTERRUPT_TRANSFER:
      case USB_MSG_DEVICE_ISOCHRONOUS_TRANSFER:
        // forward to device
        msg.device.dev->USBMessage(msg);
        continue;
    }
    break;
  }
  dprintf("USB: shutting down\n");
}

bool USB_Host::activate_endpoint(USB_Endpoint *ep, USB_Device *dev) {
  usb_msg_t msg = {
    .type = USB_MSG_ENDPOINT_ACTIVATE,
    .endpoint = {
      .ep = ep,
      .device = dev,
    }
  };
  return putMessage(msg);
}

bool USB_Host::deactivate_endpoint(USB_Endpoint *ep) {
  usb_msg_t msg = {
    .type = USB_MSG_ENDPOINT_DEACTIVATE,
    .endpoint = {
      .ep = ep
    }
  };
  return putMessage(msg);
}

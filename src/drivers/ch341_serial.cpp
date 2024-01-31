#include "ch341_serial.h"
#include <cstdio>

using namespace ch341;

#define CH341_LCR_SERIAL5      (0<<0)
#define CH341_LCR_SERIAL6      (1<<0)
#define CH341_LCR_SERIAL7      (2<<0)
#define CH341_LCR_SERIAL8      (3<<0)
#define CH341_LCR_2STOP        (1<<2)
#define CH341_LCR_PARITY       (1<<3)
#define CH341_LCR_PARITY_ODD   ((0<<4) | CH341_LCR_PARITY)
#define CH341_LCR_PARITY_EVEN  ((1<<4) | CH341_LCR_PARITY)
#define CH341_LCR_PARITY_MARK  ((2<<4) | CH341_LCR_PARITY)
#define CH341_LCR_PARITY_SPACE ((3<<4) | CH341_LCR_PARITY)
#define CH341_LCR_ENABLE_TX    (1<<6)
#define CH341_LCR_ENABLE_RX    (1<<7)

// input (status)
#define CH341_STATUS_CTS       (1<<0)
#define CH341_STATUS_DSR       (1<<1)
#define CH341_STATUS_RI        (1<<2)
#define CH341_STATUS_DCD       (1<<3)
#define CH341_STATUS_IN (CH341_STATUS_CTS|CH341_STATUS_DSR|CH341_STATUS_RI|CH341_STATUS_DCD)

#define CH341_STATUSTYPE_OVERFLOW (1<<1)
#define CH341_STATUSTYPE_ERROR    (1<<2)
#define CH341_STATUSTYPE_LINE     (1<<3)
#define CH341_STATUSTYPE_FRAME    (1<<6)

// output
//#define CH3411_STATUS_OUT      (1<<4)
#define CH341_STATUS_DTR       (1<<5)
#define CH341_STATUS_RTS       (1<<6)

// commands

// 2 bytes in, wValue=wIndex=0, retrieves "vendor version" in byte 0
#define CH341_REQ_READ_VERSION  0x5F
// initializes CH341 for UART operation (also capable of SPI/I2C/Parallel/GPIO/etc)
// wValue=(LCR<<8)|0xC09C, wIndex = (factor<<8)|divisor|0x80, no data
#define CH341_REQ_SERIAL_INIT   0xA1
// writes two 8-bit values to two separate 8-bit registers
// wvalue = ((reg2<<8)|reg1), wIndex = ((val2<<8)|val1), no data
#define CH341_REQ_WRITE_2REG    0x9A
// as above but reads (2 bytes in, wIndex=0)
#define CH341_REQ_READ_2REG     0x95
// controls UART DTR/RTS output, possibly more (OUT#?)
// no data, wValue=inverted DTR/RTS bits, wIndex=0
#define CH341_REQ_MODEM_CTRL    0xA4

#define CH341_CONTROL_IN        (USB_CTRLTYPE_DIR_DEVICE2HOST | USB_CTRLTYPE_TYPE_VENDOR | USB_CTRLTYPE_REC_DEVICE)
#define CH341_CONTROL_OUT       (USB_CTRLTYPE_DIR_HOST2DEVICE | USB_CTRLTYPE_TYPE_VENDOR | USB_CTRLTYPE_REC_DEVICE)

// registers

// bit 0 = set/clear break (also set/clear enable TX in LCR at same time)
#define CH341_REG_BREAK         0x05
// contains CTS,DSR,RI,DCD, delivered via the interrupt endpoint rather than reading these registers
#define CH341_REG_STATUS1       0x06
#define CH341_REG_STATUS2       0x07
// baudrate divisor, bit 7 = send immediately instead of waiting for full buffers (32 bytes)
#define CH341_REG_BAUD_DIVISOR  0x12
// baudrate factor
#define CH341_REG_BAUD_FACTOR   0x13
// contains word length,stop bit,parity mode and enables rx/tx
#define CH341_REG_LCR           0x18
#define CH341_REG_LCR2          0x25
// set bit 0 when RTS/CTS is used
#define CH341_REG_FLOW_CONTROL  0x27
// unknown, written to during initialization ([F]=0,[2C]=7)
#define CH341_REG_F             0x0F
#define CH341_REG_2C            0x2C

serial::operator bool() {
  bool ret = false;
  if (started) {
    if (!hw_flow || (status & CH341_STATUS_DSR))
      ret = true;
  }
  return ret;
}

void serial::calculate_baud(uint32_t baud) {
  if (baud < CH341_MIN_BPS) baud = CH341_MIN_BPS;
  else if (baud > CH341_MAX_BPS) baud = CH341_MAX_BPS;

  // baudrates > 307200 have lower throughput than expected
  // this may be due to the stop bit using a fixed width that doesn't scale...

  unsigned char b;
  unsigned long a;
  float c;

  // pre is a bitmask of three bits which disable separate prescalers of 8, 64, and 2
  // (applied to the base USB clock of 12MHz)
  static const struct { unsigned char pre; float scale; } scalers[] = {
	  {7, 1.0f},
	  {3, 2.0f},
	  {6, 8.0f},
	  {2, 16.0f},
	  {5, 64.0f},
	  {1, 128.0f},
	  {4, 512.0f},
	  {0, 1024.0f}
  };

  size_t i=0;
  do {
	  b = scalers[i].pre;
	  c = 12000000.0f / scalers[i].scale;
	  i++;
	  a = floor(c / baud);
  } while (b && a > 256);

  // factors <= 8 without a prescaler need to be halved
  if (a <= 8 && b==7) {
	  c = 6000000.0f;
	  a >>= 1;
  }

  if ((c / a - baud) > (baud - c / (a + 1)))
	a++;
  a = 256 - a;

  factor = (unsigned char)a;
  divisor = b;

  uint32_t max_out = baud / 1000;
  if (max_out == 0) tx_max = 1;
  else if (max_out >= sizeof(data_out[0])) tx_max = sizeof(data_out[0]);
  else tx_max = max_out;
}

void serial::uart_mode(uint32_t mode) {
  lcr = CH341_LCR_ENABLE_TX|CH341_LCR_ENABLE_RX;

  uint8_t bits = 8;
  if (mode & (1<<1)) { // parity
    if (mode & (1<<0)) // odd
      lcr |= CH341_LCR_PARITY_ODD;
    else
      lcr |= CH341_LCR_PARITY_EVEN;
    // bit 2 set = 8 bits+parity, otherwise 7 bits+parity
    if ((mode & (1<<2)) == 0)
      bits = 7;
  }

  if (mode & (1<<8))
    lcr |= CH341_LCR_2STOP;

  if (bits == 7)
    lcr |= CH341_LCR_SERIAL7;
  else
    lcr |= CH341_LCR_SERIAL8;
}

void serial::status_callback(int result) {
  if (result < 4) return;

//  dprintf("Status: %02X:%02X:%02X:%02X\n", status_in[0], status_in[1], status_in[2], status_in[3]);

  if (status_in[0] & CH341_STATUSTYPE_LINE) {
    uint8_t new_status = (~status_in[2]) & CH341_STATUS_IN;

    // compare new vs. old status, act on changes...
//    dprintf("CH341 line status %02X (%02X)\n", new_status, status);
    status = new_status;
  }

  if (status_in[0] & CH341_STATUSTYPE_OVERFLOW) {
    dprintf("CH341: input overrun (%u %p %p)\n", read_buf.available(), rx_buf[0], rx_buf[1]);
  }

  if (status_in[0] & CH341_STATUSTYPE_ERROR) {
    if (status_in[0] & CH341_STATUSTYPE_FRAME) {
      dprintf("CH341: frame error\n");
    } else {
      dprintf("CH341: parity error\n");
    }
  }

  get_status();
}

void serial::get_status(void) {
  if (attached) {
    const auto fn = std::bind(&serial::status_callback, this, std::placeholders::_1);
    InterruptMessage(ep_status, sizeof(status_in), status_in, fn);
  }
}

void serial::read_callback(int result, uint8_t *buf) {
  if (atomMutexGet(&rx_lock, 10) == ATOM_OK) {
    if (result >= 0) {
      read_buf.write(buf, (size_t)result);
    }
    atomMutexPut(&rx_lock);
  }
  if (result != -ENXIO)
    queue_read(buf);
}

void serial::queue_read(uint8_t *buf) {
  if (atomMutexGet(&rx_lock, 1) == ATOM_OK) {
    do {
      uint32_t to_read = (uint32_t)read_buf.availableForWrite();
      if (started && to_read) {
        const auto fn = std::bind(&serial::read_callback, this, std::placeholders::_1, buf);
        if (to_read > sizeof(data_in[0])) to_read = sizeof(data_in[0]);
        if (BulkMessage(ep_in, to_read, buf, fn) >= 0) {
          break;
        }
      }
      if (rx_buf[0] == NULL) rx_buf[0] = buf;
      else rx_buf[1] = buf;
    } while(0);
    atomMutexPut(&rx_lock);
  }
}

void serial::dtr_rts_callback(int result, uint8_t old_status, uint8_t new_status) {
  if (atomMutexGet(&rx_lock, 10) == ATOM_OK) {
    if (result < 0) {
      dprintf("Setting new DTR/RTS failed %d\n", result);
      out_status = old_status;
    } else {
      dprintf("Setting new DTR/RTS succeeded\n");
      out_status = new_status;
    }
    atomMutexPut(&rx_lock);
  }
}

void serial::set_dtr_rts(uint8_t new_status) {
  new_status &= CH341_STATUS_DTR|CH341_STATUS_RTS;
  if (hw_flow) new_status |= CH341_STATUS_RTS;
  if (attached && new_status != out_status) {
    const auto fn = std::bind(&serial::dtr_rts_callback, this, std::placeholders::_1, out_status, new_status);
    if (ControlMessage(CH341_CONTROL_OUT, CH341_REQ_MODEM_CTRL, ~new_status, 0, 0, NULL, fn) >= 0)
      out_status = new_status;
  }
}

void serial::start(void) {
  uint8_t t = atomMutexGet(&tx_lock, 1);
  if (atomMutexGet(&rx_lock, 1) == ATOM_OK) {
    if (hw_flow) set_dtr_rts(CH341_STATUS_DTR|CH341_STATUS_RTS);
    if (!started) {
      started = true;
      read_buf.flush();
      if (rx_buf[1]) {
        uint8_t *b = rx_buf[1];
        rx_buf[1] = NULL;
        queue_read(b);
      }
      if (rx_buf[0]) {
        uint8_t *b = rx_buf[0];
        rx_buf[0] = NULL;
        queue_read(b);
      }
    }

    atomMutexPut(&rx_lock);
  }
  if (t == ATOM_OK) atomMutexPut(&tx_lock);
}

void serial::init(int result, unsigned int stage) {
  dprintf("ch341::serial::init stage %u result %d\n", stage, result);
  const auto fn = std::bind(&serial::init, this, std::placeholders::_1, stage+1);

  if (result < 0) return;

  switch (stage) {
    case 0:
      ControlMessage(CH341_CONTROL_IN, CH341_REQ_READ_VERSION, 0, 0, sizeof(version), &status_in, fn);
      break;
    case 1:
      if (result < (int)sizeof(version)) break;
      memcpy(&version, status_in, sizeof(version));
      dprintf("CH341 version: %04X\n", version);
      // initialize device to UART mode, set mode and baudrate
      ControlMessage(CH341_CONTROL_OUT, CH341_REQ_SERIAL_INIT, (lcr<<8)|0xC09C, (factor<<8)|divisor|0x80, 0, NULL, fn);
      break;
    case 2:
      // unknown register pokes - REG_F might be related to more accurate baud timing (see FreeBSD driver...)
      ControlMessage(CH341_CONTROL_OUT, CH341_REQ_WRITE_2REG, (CH341_REG_F<<8)|CH341_REG_2C, 0x0007, 0, NULL, fn);
      break;
    case 3:
      // the write command always writes to two registers so I think this is how you write to only one register
      // this supposedly enables hardware flow control (RTS/CTS), I have verified this at least disables tx whilst CTS is clear...
      ControlMessage(CH341_CONTROL_OUT, CH341_REQ_WRITE_2REG, (CH341_REG_FLOW_CONTROL<<8)|CH341_REG_FLOW_CONTROL, hw_flow ? 0x0101:0, 0, NULL, fn);
      break;
    case 4:
      // get current port status
      ControlMessage(CH341_CONTROL_IN, CH341_REQ_READ_2REG, (CH341_REG_STATUS2<<8)|CH341_REG_STATUS1, 0, 2, status_in, fn);
      break;
    case 5:
      if (result < 2) break;
      status = (~status_in[0]) & CH341_STATUS_IN;
      dprintf("CH341 Port status: %02X\n", status);

      get_status();
      start();
      break;
  }
}

void serial::send_timer_expired(EventResponder& e) {
  (static_cast<serial*>(&e))->flush();
}

serial::serial() {
  EventResponder::attach(send_timer_expired);
  atomMutexCreate(&tx_lock);
  atomMutexCreate(&rx_lock);
  atomCondCreate(&tx_signal);

  tx_buf[0] = data_out[0];
  tx_buf[1] = data_out[1];
  tx_length = 0;
  tx_max = sizeof(data_out[0]);

  rx_buf[0] = rx_buf[1] = NULL;

  attached = false;
  started = false;
  calculate_baud(CH341_DEFAULT_BAUD);
  uart_mode(SERIAL_8N1);
  hw_flow = true;
  status = 0;
}

serial::~serial() {
  if (attached) {
    detach();
    started = false;
    attached = false;
  }

  EventResponder::detach();

  atomCondDelete(&tx_signal);
  atomMutexDelete(&tx_lock);
  atomMutexDelete(&rx_lock);
}

void serial::detach(void) {
  uint8_t t = atomMutexGet(&tx_lock, 10);
  uint8_t r = atomMutexGet(&rx_lock, 10);
  sendTimer.end();
  started = false;
  attached = false;
  status = 0;

  if (t == ATOM_OK) atomMutexPut(&tx_lock);
  if (r == ATOM_OK) atomMutexPut(&rx_lock);
  dprintf("ch341::serial Detached\n");
}

bool serial::offer(const usb_device_descriptor* d,const usb_configuration_descriptor*) {
  if (d->idVendor==0x4348 && d->idProduct==0x5523) return true;
  if (d->idVendor==0x1A86 && d->idProduct==0x7523) return true; // CH340
  return false;
}

USB_Driver* serial::attach(const usb_device_descriptor*,const usb_configuration_descriptor* c, USB_Device* d) {
  if (!attached) {
    int endpoints=0;
    ep_out = ep_in = ep_status = 0;
    const uint8_t* b = (const uint8_t*)(c+1);
    // find first interface
    while (b[1] != USB_DT_INTERFACE) b += b[0];
    const usb_interface_descriptor *i = (const usb_interface_descriptor*)b;
    if (i->bNumEndpoints < 3) return NULL;
    for (uint8_t n=0; n < i->bNumEndpoints && endpoints < 3; n++) {
      // find next endpoint
      while (b[1] != USB_DT_ENDPOINT) b += b[0];
      const usb_endpoint_descriptor *e = (const usb_endpoint_descriptor*)b;
      b += b[0];
      if (e->bmAttributes == USB_ENDPOINT_BULK && e->wMaxPacketSize == 32) {
        if (e->bEndpointAddress & 0x80) {
          if (ep_in == 0) {
            ep_in = e->bEndpointAddress;
            endpoints++;
          }
        } else if (ep_out == 0) {
          ep_out = e->bEndpointAddress;
          endpoints++;
        }
      } else if (e->bmAttributes == USB_ENDPOINT_INTERRUPT && e->wMaxPacketSize >= 4) {
        if (ep_status == 0) {
          ep_status = e->bEndpointAddress;
          endpoints++;
        }
      }
    }
    if (endpoints != 3) {
      dprintf("ch341: failed to find 3 endpoints (%d)\n", endpoints);
      return NULL;
    }

    setDevice(d);

    if (atomMutexGet(&rx_lock, 10) == ATOM_OK) {
      out_status = 0;
      atomMutexPut(&rx_lock);
    }
    started = false;
    status = 0;
    init(0, 0);
    attached = true;
    queue_read(data_in[0]);
    queue_read(data_in[1]);

    dprintf("ch341::serial Attached (%p)\n", this);
    return this;
  }
  return NULL;
}

void serial::begin(uint32_t baud, uint16_t format, bool rts_cts) {
  uint8_t oldfactor = factor;
  uint8_t olddivisor = divisor;
  uint8_t oldlcr = lcr;
  bool flow = hw_flow;

  calculate_baud(baud);
  uart_mode(format);
  hw_flow = rts_cts;

  if (!attached) return;

  if (oldfactor != factor || olddivisor != divisor || oldlcr != lcr) {
    ControlMessage(CH341_CONTROL_OUT, CH341_REQ_SERIAL_INIT, (lcr<<8)|0xC09C, (factor<<8)|divisor|0x80, 0, NULL);
  }
  if (flow != hw_flow) {
    ControlMessage(CH341_CONTROL_OUT, CH341_REQ_WRITE_2REG, (CH341_REG_FLOW_CONTROL<<8)|CH341_REG_FLOW_CONTROL, hw_flow ? 0x0101:0, 0, NULL);
  }

  start();
}

void serial::end() {
  started = false;
  if (atomMutexGet(&rx_lock, 1) == ATOM_OK) {
    set_dtr_rts(0);
    atomMutexPut(&rx_lock);
  }
}

int serial::available() {
  int ret = 0;
  if (atomMutexGet(&rx_lock, 1) == ATOM_OK) {
    size_t s = read_buf.available();
    if (s) ret = (int)s;
    atomMutexPut(&rx_lock);
  }
  return ret;
}

int serial::peek() {
  int ret = -1;
  if (atomMutexGet(&rx_lock, 1) == ATOM_OK) {
    if (read_buf.available() > 0) {
      ret = *read_buf.peek();
    }
    atomMutexPut(&rx_lock);
  }
  return ret;
}

int serial::read() {
  int ret = -1;
  if (atomMutexGet(&rx_lock, 1) == ATOM_OK) {
    if (read_buf.available() > 0) {
      uint8_t c;
      read_buf.read(&c, 1);
      ret = c;
      // there is now at least some space in the circular buffer,
      // so queue pending rx transfers
      if (rx_buf[0]) {
        uint8_t *buf = rx_buf[0];
        rx_buf[0] = rx_buf[1];
        rx_buf[1] = NULL;
        queue_read(buf);
      }
    }
    atomMutexPut(&rx_lock);
  }
  return ret;
}

int serial::availableForWrite() {
  int ret=0;
  if (atomMutexGet(&tx_lock, 1) == ATOM_OK) {
    if (started && (!hw_flow || (status & CH341_STATUS_CTS))) {
      if (tx_buf[0] && tx_length < tx_max) {
        ret = (int)(tx_max - tx_length);
      }
    }
    atomMutexPut(&tx_lock);
  }
  return ret;
}

size_t serial::write(uint8_t c) {
  size_t ret = 0;
  if (atomMutexGet(&tx_lock, 1) == ATOM_OK) {
    if (started) {
      if (tx_buf[0] == NULL) {
        if (atomCondWait(&tx_signal, &tx_lock, 50) != ATOM_OK) {
          dprintf("Failed to get TX signal\n");
        }
      }
      if (tx_buf[0] && tx_length < sizeof(data_out[0])) {
        tx_buf[0][tx_length++] = c;
        ret = 1;
        // maybe start the timer to send later
        if (tx_length < tx_max)
          sendTimer.begin(1, *this);
        // else send now
        else
          flush();
      }
    }
    atomMutexPut(&tx_lock);
  }
  else dprintf("Failed to get tx_lock\n");
  return ret;
}

void serial::write_callback(int result, uint8_t *buf) {
  if (result < 0) dprintf("write failed: %d\n", result);
  // don't care about the result - buffer is now free, recycle it
  if (atomMutexGet(&tx_lock, 10) == ATOM_OK) {
    if (tx_buf[0] == NULL) { // use it immediately
      tx_buf[0] = buf;
      tx_length = 0;
    }
    else { // queue it for later use
      tx_buf[1] = buf;
    }
    atomCondSignal(&tx_signal);
    atomMutexPut(&tx_lock);
  }
}

void serial::flush(void) {
  if (atomMutexGet(&tx_lock, 10) == ATOM_OK) {
    if (started) {
      sendTimer.end();
      if (tx_buf[0] && tx_length>0) {
        const auto fn = std::bind(&serial::write_callback, this, std::placeholders::_1, tx_buf[0]);
        if (BulkMessage(ep_out, tx_length, tx_buf[0], fn) < 0) {
          // failed to send - nothing we can do now, buffer will be discarded
          dprintf("Failed to flush CH341 output buffer\n");
        } else {
          tx_buf[0] = tx_buf[1];
          tx_buf[1] = NULL;
        }
        tx_length = tx_buf[0] ? 0 : tx_max;
      }
    }
    atomMutexPut(&tx_lock);
  }
}

void serial::set_dtr_rts(bool dtr, bool rts) {
  if (atomMutexGet(&rx_lock, 1) == ATOM_OK) {
    if (attached) {
      uint8_t s = (rts ? CH341_STATUS_RTS : 0);
      s |= (dtr ? CH341_STATUS_DTR : 0);
      set_dtr_rts(s);
    }
    atomMutexPut(&rx_lock);
  }
}

void serial::set_dtr(bool set) {
  if (atomMutexGet(&rx_lock, 1) == ATOM_OK) {
    set_dtr_rts((out_status & ~CH341_STATUS_DTR) | (set ? CH341_STATUS_DTR : 0));
    atomMutexPut(&rx_lock);
  }
}

void serial::set_rts(bool set) {
  // don't allow controlling RTS if hardware flow control is in use
  if (hw_flow) return;
  if (atomMutexGet(&rx_lock, 1) == ATOM_OK) {
    set_dtr_rts((out_status & ~CH341_STATUS_RTS) | (set ? CH341_STATUS_RTS : 0));
    atomMutexPut(&rx_lock);
  }
}

template<size_t length>
circ_buf<length>::circ_buf() {
  flush();
}

template<size_t length>
size_t circ_buf<length>::read(uint8_t *dst, size_t len) {
  if (len > available()) len = available();
  size_t to_copy = len;
  if ((head + to_copy) >= (buf + length)) {
    to_copy = buf + length - head;
    memcpy(dst, head, to_copy);
    dst += to_copy;
    head = buf;
    if (to_copy == len)
      goto end;
    to_copy = len - to_copy;
  }
  memcpy(dst, head, to_copy);
  head += to_copy;
end:
  avail += len;
  return len;
}

template<size_t length>
void circ_buf<length>::write(const uint8_t *src, size_t len) {
  if (len > availableForWrite()) len = availableForWrite();
  size_t to_copy = len;
  if ((tail + to_copy) >= (buf + length)) {
    to_copy = buf + length - tail;
    memcpy(tail, src, to_copy);
    src += to_copy;
    tail = buf;
    if (to_copy == len)
      goto end;
    to_copy = len - to_copy;
  }
  memcpy(tail, src, to_copy);
  tail += to_copy;
end:
  avail -= len;
}

template <size_t length>
void circ_buf<length>::flush(void) {
  head = tail = buf;
  avail = length;
}

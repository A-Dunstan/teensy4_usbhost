#include "FL2000.h"
#include "FL2000_regs.h"

#include <usb_portab.h>
#include <Arduino.h>
#include <cstdarg>
#include <cerrno>

#define REQUEST_REG_READ        64
#define REQUEST_REG_WRITE       65

#define I2C_RETRY_MAX           10

#define I2C_ADDRESS_HDMI        0x4C
#define I2C_ADDRESS_DSUB        0x50

template <class Ex>
Ex* LoadExclusivePtr(Ex** ptr) {
  Ex* val;
  asm volatile("ldrex %0, %1" : "=r" (val) : "Q" (*ptr));
  return val;
}

// returns 0 if store succeeded, 1 on failure
template <class Ex>
int StoreExclusivePtr(Ex* val, Ex** ptr) {
  int r;
  asm volatile("strex %0, %2, %1" : "=&r"(r), "=Q"(*ptr) : "r"(val));
  return r;
}

static const struct mode_timing vid_modes[] PROGMEM = {
  // refresh rate is approximate, for lookup purposes only
  // WIDTH   HEIGHT   REFRESH  H_TOTAL  H_SYNC  H_BP  V_TOTAL  V_SYNC  V_BP  P    M    D   FLAGS
  {  256,    240,     60,      320,     38,     20,   525,     2,      33,   2,  127,  63, VIDMODE_FLAG_LINEDOUBLE},
  {  256,    288,     50,      324,     34,     24,   597,     3,      17,   1,   35,  36, VIDMODE_FLAG_VSYNC_POS|VIDMODE_FLAG_LINEDOUBLE},
  {  320,    200,     70,      400,     48,     24,   449,     2,      35,   1,   73,  58, VIDMODE_FLAG_VSYNC_POS|VIDMODE_FLAG_LINEDOUBLE},
  {  320,    240,     60,      400,     48,     24,   525,     2,      33,   1,   73,  58, VIDMODE_FLAG_LINEDOUBLE},
  {  320,    400,     70,      400,     48,     24,   449,     2,      35,   1,   73,  58, VIDMODE_FLAG_VSYNC_POS},
  {  320,    480,     60,      400,     48,     24,   525,     2,      33,   1,   73,  58, 0},
  {  640,    350,     70,      800,     96,     48,   449,     2,      60,   1,   73,  29, VIDMODE_FLAG_HSYNC_POS},
  {  640,    400,     70,      800,     96,     48,   449,     2,      35,   1,   73,  29, VIDMODE_FLAG_VSYNC_POS},
  {  640,    480,     60,      800,     96,     48,   525,     2,      33,   1,   73,  29, 0},
  {  768,    576,     50,      976,     104,    80,   597,     3,      17,   1,   35,  12, VIDMODE_FLAG_VSYNC_POS},
  {  768,    576,     60,      976,     104,    80,   597,     3,      17,   1,   7,   2,  VIDMODE_FLAG_VSYNC_POS},
  {  800,    600,     56,      1024,    128,    72,   625,     2,      22,   1,   18,  5,  VIDMODE_FLAG_HSYNC_POS|VIDMODE_FLAG_VSYNC_POS},
  {  800,    600,     60,      1056,    128,    88,   628,     4,      23,   1,   8,   2,  VIDMODE_FLAG_HSYNC_POS|VIDMODE_FLAG_VSYNC_POS},
};

struct mode_request {
  const struct mode_timing& mode;
  int32_t input_format;
  int32_t output_format;
  int result;
  ATOM_SEM sema;
};

struct frame_request {
  const void *src;
  size_t pitch;
  int result;
  ATOM_SEM sema;
};

struct pal_request {
  const uint32_t *src;
  uint8_t start;
  size_t count;
  int result;
  ATOM_SEM sema;
};

#define CMD_ATTACH              0
#define CMD_DETACH              1
#define CMD_INTERRUPT           2
#define CMD_SET_MODE            3
#define CMD_SLICE_DONE          4
#define CMD_FRAME_DONE          5
#define CMD_SET_NEXT_FRAME      6
#define CMD_SET_PALETTE         7
#define CMD_SEND_SLICE          8

struct threadMsg {
  uint32_t cmd;
  union {
    mode_request* mode_req;
    frame_request* frame_req;
    pal_request* pal_req;
    struct {
      uint8_t *block;
      void* dma;
      uint32_t id;
    } slice;
  };
};

void FL2000::thread(void) {
  threadMsg msgs[10];

  USBCallback intr_cb = [=](int r) {
    if (r >= 0) {
      dbg_log("Interrupt arrived");

      threadMsg msg = {CMD_INTERRUPT};
      if (atomQueuePut(&workQueue, -1, &msg) != ATOM_OK) {
        dbg_log("Failed to queue interrupt");
      }
    }
    else dbg_log("Interrupt returned error: %d", r);
  };

  if (atomQueueCreate(&workQueue, msgs, sizeof(msgs[0]), sizeof(msgs)/sizeof(msgs[0])) == ATOM_OK) {
    threadMsg msg;

    dbg_log("thread started");

    int status = 0;
    do {
      if (atomQueueGet(&workQueue, 0, &msg) != ATOM_OK)
        break;

      switch (msg.cmd) {
        case CMD_ATTACH:
          dbg_log("Attach msg");
          status = device_init();
          if (status < 0)
            break;
          // fallthrough
        case CMD_INTERRUPT:
          if (process_interrupt() < 0) {
            dbg_log("Failed to handle interrupt!");
          }
          status = InterruptMessage(0x83, sizeof(intr_data), intr_data, &intr_cb);
          break;
        case CMD_DETACH:
          dbg_log("Detach msg");
          break;
        case CMD_SET_MODE:
          dbg_log("set mode msg");
          if (msg.mode_req == NULL) {
            status = -1;
            break;
          }
          msg.mode_req->result = set_mode(msg.mode_req->mode, msg.mode_req->input_format, msg.mode_req->output_format);
          atomSemPut(&msg.mode_req->sema);
          break;
        case CMD_SEND_SLICE:
          send_slice(msg.slice.block, msg.slice.id, msg.slice.dma);
          break;
        case CMD_SLICE_DONE:
          // make sure render context is still current
          if (msg.slice.id == render_id) {
            if (sent_lines < max_lines) {
              begin_slice(msg.slice.block, msg.slice.dma);
            }
          } else {
            dbg_log("slice render_id mismatch");
          }
          break;
        case CMD_FRAME_DONE:
          if (msg.slice.id == render_id) {
            if (next_fb) {
              current_fb = next_fb;
              current_pitch = next_pitch;
              next_fb = NULL;
            }
            status = frame_begin();
          } else {
            dbg_log("frame render_id mismatch");
          }
          break;
        case CMD_SET_NEXT_FRAME:
          next_pitch = msg.frame_req->pitch;
          next_fb = (uint8_t*)msg.frame_req->src;
          msg.frame_req->result = 0;
          atomSemPut(&msg.frame_req->sema);
          break;
        case CMD_SET_PALETTE:
          msg.pal_req->result = 0;
          for (size_t i=0; i < msg.pal_req->count; i++) {
            uint8_t idx = i + msg.pal_req->start;
            reg_vga_plt_data p = {
              .palette_ram_wr_addr = idx,
              .palette_ram_data = msg.pal_req->src[i]
            };
            int r = reg_write(REG_VGA_PLT_DATA, p.val);
            if (r < 0) {
              msg.pal_req->result = r;
              break;
            }
          }
          atomSemPut(&msg.pal_req->sema);
          break;
        default:
          dbg_log("Unknown command: %u", msg.cmd);
          status = -1;
      }
    } while (status >= 0);

    atomQueueDelete(&workQueue);
  }
  else dbg_log("Failed to create workQueue");

}

FLASHMEM void FL2000::threadStart(uint32_t arg) {
  class FL2000* p = (class FL2000*)arg;
  p->thread();
  p->dbg_log("ERROR workThread exited!");
  while(1) atomTimerDelay(1000);
};

int FL2000::dbg_log(const char* fmt, ...) {
  va_list args;
  int i = fprintf(stderr, "FL2000<%p>: ", this);

  va_start(args, fmt);
  int j = vfprintf(stderr, fmt, args);
  va_end(args);
  putchar('\n');

  if (i >= 0) {
    if (j < 0)
      i = j;
    else i += j;
  }
  return i;
}

FLASHMEM int FL2000::reg_write(uint16_t offset, const uint32_t val) {
  uint8_t bmReqtype = USB_CTRLTYPE_DIR_HOST2DEVICE | USB_CTRLTYPE_TYPE_VENDOR | USB_CTRLTYPE_REC_DEVICE;
  uint8_t bmRequest = REQUEST_REG_WRITE;
  reg_data[0] = val;
  int r = ControlMessage(bmReqtype, bmRequest, 0, offset, sizeof(reg_data[0]), reg_data);
  if (r >= 0 && r < (int)sizeof(reg_data[0])) {
    dbg_log("reg_write failed for register %04X", offset);
    errno = EINVAL;
    r = -1;
  }
  return r;
}

FLASHMEM int FL2000::reg_read(uint16_t offset, uint32_t& val) {
  uint8_t bmReqtype = USB_CTRLTYPE_DIR_DEVICE2HOST | USB_CTRLTYPE_TYPE_VENDOR | USB_CTRLTYPE_REC_DEVICE;
  uint8_t bmRequest = REQUEST_REG_READ;
  int r = ControlMessage(bmReqtype, bmRequest, 0, offset, sizeof(reg_data[0]), reg_data);
  if (r >= 0 && r < (int)sizeof(reg_data[0])) {
    dbg_log("reg_read failed for register %04X", offset);
    errno = EINVAL;
    r = -1;
  }
  val = reg_data[0];
  return r;
}

FLASHMEM int FL2000::reg_set(uint16_t offset, const uint32_t val) {
  uint32_t data;
  int r = reg_read(offset, data);
  if (r >= 0) {
    r = reg_write(offset, data | val);
  }
  return r;
}

FLASHMEM int FL2000::reg_clear(uint16_t offset, const uint32_t val) {
  uint32_t data;
  int r = reg_read(offset, data);
  if (r >= 0) {
    r = reg_write(offset, data & ~val);
  }
  return r;
}

FLASHMEM int FL2000::i2c_transfer(uint8_t address, uint8_t offset, bool read) {
  reg_i2c_control ctrl;

  int ret = reg_read(REG_I2C_CONTROL, ctrl.val);
  if (ret < 0)
    return ret;

  ctrl.monitor_detect = 1;
  ctrl.address = address;
  ctrl.cmd = read ? 1:0;
  ctrl.offset = offset;
  ctrl.spi = 0;
  ctrl.spi_erase = 0;
  ctrl.complete = 0;
  ret = reg_write(REG_I2C_CONTROL, ctrl.val);
  if (ret < 0)
    return ret;

  delay(3);
  int retry;
  for (retry=0; retry < I2C_RETRY_MAX; retry++) {
    ret = reg_read(REG_I2C_CONTROL, ctrl.val);
    if (ret < 0)
      return ret;

    if (ctrl.complete)
      break;

    atomTimerDelay(1);
  }
  if (retry >= I2C_RETRY_MAX || ctrl.status!=0) {
    dbg_log("i2c_transfer failed for address/offset %02X:%02X (%s)", address, offset, (read ? "read" : "write"));
    errno = EIO;
    return -1;
  }

  return 0;
}

FLASHMEM int FL2000::i2c_read_dword(uint8_t address, uint8_t offset, uint32_t& val) {
  int ret = i2c_transfer(address, offset, true);
  if (ret < 0)
    return ret;

  return reg_read(REG_I2C_READ, val);
}

FLASHMEM int FL2000::i2c_write_dword(uint8_t address, uint8_t offset, const uint32_t val) {
  int ret = reg_write(REG_I2C_WRITE, val);
  if (ret < 0)
    return ret;

  return i2c_transfer(address, offset, false);
}

FLASHMEM int FL2000::i2c_read_byte(uint8_t address, uint8_t offset, uint8_t& val) {
  uint32_t data;
  uint32_t bit_offset = (offset & 3) << 3;
  offset &= ~3;

  int ret = i2c_read_dword(address, offset, data);
  if (ret < 0)
    return ret;

  val = (uint8_t)(data >> bit_offset);
  return 0;
}

FLASHMEM int FL2000::i2c_write_byte(uint8_t address, uint8_t offset, const uint8_t val) {
  uint32_t data;
  uint32_t bit_offset = (offset & 3) << 3;
  offset &= ~3;

  int ret = i2c_read_dword(address, offset, data);
  if (ret < 0)
    return ret;

  data &= ~(0x000000FF << bit_offset);
  data |= val << bit_offset;
  return i2c_write_dword(address, offset, data);
}

FLASHMEM int FL2000::i2c_write_byte_masked(uint8_t address, uint8_t offset, const uint8_t val, const uint8_t mask) {
  uint32_t data;
  uint32_t bit_offset = (offset & 3) << 3;
  offset &= ~3;

  int ret = i2c_read_dword(address, offset, data);
  if (ret < 0)
    return ret;

  data &= ~(mask << bit_offset);
  data |= (val & mask) << bit_offset;
  return i2c_write_dword(address, offset, data);
}

FLASHMEM int FL2000::device_init(void) {
  int ret = 0;
  monitor_plugged_in = 0;

  // USB3 LPM - disable U1/U2 states
  reg_usb_lpm lpm = {
    .u2_reject = 1,
    .u1_reject = 1
  };
  ret = reg_set(REG_USB_LPM, lpm.val);
  if (ret < 0) return ret;

  // trigger software reset
  reg_reset_ctrl rctrl = { .sw_reset = 1 };
  ret = reg_set(REG_RESET_CTRL, rctrl.val);
  if (ret < 0) return ret;

  // check for HDMI encoder chip
  uint32_t data;
  has_ITE66121 = false;
  ret = i2c_read_dword(I2C_ADDRESS_HDMI, 0, data);
  if (ret >= 0) {
    // vendor 0x4954, device 0x612, top 4 bits are revision (don't care)
    if ((data & 0x0FFFFFFF) == 0x06124954) {
      // HDMI encoder is ITE Tech IT66121
      dbg_log("found HDMI encoder");
      // reset HDMI
      ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_SW_RESET, ITE_REG_SW_RESET_REF_CLOCK, ITE_REG_SW_RESET_REF_CLOCK);
      if (ret < 0) return ret;
      // gate all except RCLK (I2C)
      ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_GATE_BANK_CTRL, ~ITE_REG_GATE_BANK_CTRL_RCLK, ITE_REG_GATE_BANK_CTRL_GATE_ALL);
      if (ret < 0) return ret;
      // turn on TxCLK
      ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_INT_CTRL, 0, ITE_REG_INT_CTRL_TXCLK);
      if (ret < 0) return ret;
      // power on DRV
      ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AFE_DRV, 0, ITE_REG_AFE_DRV_PWD);
      if (ret < 0) return ret;
      // power on XPLL
      ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AFE_XP, 0, ITE_REG_AFE_XP_PWDI | ITE_REG_AFE_XP_PWDPLL);
      if (ret < 0) return ret;
      // power on IPLL
      ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AFE_IP, 0, ITE_REG_AFE_IP_PWDPLL);
      if (ret < 0) return ret;
      // clear DRV_RST
      ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AFE_DRV, 0, ITE_REG_AFE_DRV_RST);
      if (ret < 0) return ret;
      // set XP_RESETB
      ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AFE_XP, ITE_REG_AFE_XP_RESETB, ITE_REG_AFE_XP_RESETB);
      if (ret < 0) return ret;
      // set IP_RESETB
      ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AFE_IP, ITE_REG_AFE_IP_RESETB, ITE_REG_AFE_IP_RESETB);
      if (ret < 0) return ret;
      // poke XP_TEST register?
      ret = i2c_write_byte(I2C_ADDRESS_HDMI, ITE_REG_AFE_XP_TEST, 0x70);
      if (ret < 0) return ret;
      // poke AFE_EN
      ret = i2c_write_byte(I2C_ADDRESS_HDMI, ITE_REG_AFE_EN, 0x1F);
      if (ret < 0) return ret;
      // set output current level
      ret = i2c_write_byte(I2C_ADDRESS_HDMI, ITE_REG_AFE_LEVEL, ITE_REG_AFE_LEVEL_DRV_ISW(7));
      if (ret < 0) return ret;
      // power on RCLK, IACLK, TXCLK
      ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_GATE_BANK_CTRL, ITE_REG_GATE_BANK_CTRL_CRCLK, ITE_REG_GATE_BANK_CTRL_GATE_ALL);
      if (ret < 0) return ret;
      has_ITE66121 = true;
    }
  }

  // enable interrupts, set EOF notification method and other misc. setup
  // these settings may persist as long as power is applied (e.g. across USB resets)
  reg_vga_control vga_c;
  ret = reg_read(REG_VGA_CONTROL, vga_c.val);
  if (ret < 0) return ret;
  dbg_log("Initial VGA CONTROL: %08X", vga_c.val);
  vga_c.edid_mon_int_en = 1;
  vga_c.ext_mon_int_en = 1;
  vga_c.vga_status_self_clear_dis = 0;
  vga_c.feedback_int_en = 0;
  vga_c.force_loopback = 0;
  vga_c.biac_en = 0;
  vga_c.vga_err_int_en = 0;
  vga_c.use_zero_td = 0;
  vga_c.use_zero_pkt_len = 1;
  vga_c.use_pkt_pending = 0;

  vga_c.lbuf_drop_frame_en = 0;
  vga_c.lbuf_err_int_en = 1;
  vga_c.vga_err_int_en = 1;

  ret = reg_write(REG_VGA_CONTROL, vga_c.val);
  if (ret < 0) return ret;

  // enable interrupt for I2C detection and external monitor
  reg_i2c_control i2c = {
    .monitor_detect = 1,
    .edid_detect = 1
  };
  ret = reg_set(REG_I2C_CONTROL, i2c.val);
  if (ret < 0) return ret;
  atomTimerDelay(SYSTEM_TICKS_PER_SEC*3/4);

  // some sort of reset?
  ret = reg_clear(0x8088, 1<<10);

  return ret;
}

FLASHMEM int FL2000::process_interrupt(void) {
  int ret;

  reg_vga_status status;
  ret = reg_read(REG_VGA_STATUS, status.val);
  if (ret < 0) return ret;
  dbg_log("vga_status register: %08X", status.val);
  if (status.vga_status) {
    if (!monitor_plugged_in) {
      reg_clear(0x0078, 1<<17);

      // retrieve EDID block 0
      if (has_ITE66121) {
        dbg_log("UNIMPLEMENTED: reading EDID from HDMI");
      } else { // read EDID from DSUB
        uint32_t d[2];
        if (i2c_read_dword(I2C_ADDRESS_DSUB, 0, d[0])>=0 &&
            i2c_read_dword(I2C_ADDRESS_DSUB, 4, d[1])>=0 &&
            d[0] == 0xFFFFFF00 && d[1] == 0x00FFFFFF) {
          memcpy(edid, d, 8);
          for (size_t i=8; i < sizeof(edid); i+=4) {
            if (i2c_read_dword(I2C_ADDRESS_DSUB, i, d[0]) < 0)
              break;
            memcpy(edid+i, d, 4);
          }
        }
      }
      // verify checksum
      uint8_t sum = 0;
      for (size_t i=0; i < 128; i++) {
//        printf("%02X ", edid[i]);
//        if ((i&0xF)==15) printf("\n");
        sum += edid[i];
      }
      if (sum != 0) {
        dbg_log("EDID checksum was incorrect, clearing");
        memset(edid, 0, sizeof(edid));
      } else {
        dbg_log("Retrieved EDID block 0 (checksum: %02X)", edid[127]);
      }

      monitor_plugged_in = true;

      if (monitor_notify)
        monitor_notify->triggerEvent(MONITOR_NOTIFY_CONNECTED, this);
    }
  } else {
    if (monitor_plugged_in) {
      monitor_plugged_in = false;
      reg_set(0x0078, 1<<17);

      if (monitor_notify)
        monitor_notify->triggerEvent(MONITOR_NOTIFY_DISCONNECTED, this);
    }
  }

  return ret;
}

FLASHMEM bool FL2000::offer(const usb_device_descriptor* d, const usb_configuration_descriptor *cd) {
  if (getDevice() != NULL) return false; // driver instance already attached to a device
  if (d->idVendor != 0x1D5C || d->idProduct != 0x2000) return false;
  return true;
}

FLASHMEM USB_Driver* FL2000::attach(const usb_device_descriptor *d, const usb_configuration_descriptor*, USB_Device *dev) {
  setDevice(dev);
  dbg_log("ATTACH");
  threadMsg msg = {CMD_ATTACH};
  atomQueuePut(&workQueue, 10, &msg);
  return this;
}

FLASHMEM void FL2000::detach(void) {
  dbg_log("DETACH");
  threadMsg msg = {CMD_DETACH};
  atomQueuePut(&workQueue, 10, &msg);
}

FLASHMEM FL2000::FL2000() {
  monitor_notify = NULL;
  render_id = 0;
  atomThreadCreate(&workThread, 80, threadStart, (uint32_t)this, workStack, sizeof(workStack), 0);
}

FLASHMEM FL2000::~FL2000() {
  if (monitor_notify)
    monitor_notify->triggerEvent(MONITOR_NOTIFY_ERROR);

  // atomThreadJoin(&workThread);
}

FLASHMEM EventResponder* FL2000::set_monitor_event(EventResponder* event_hook) {
  EventResponder* old = monitor_notify;
  monitor_notify = event_hook;
  return old;
}

template <class creq>
int FL2000::forwardMsg(creq& req, threadMsg& msg) {
  int ret = -1;

  if (atomSemCreate(&req.sema, 0) == ATOM_OK) {
    if (atomQueuePut(&workQueue, 0, &msg) != ATOM_OK) {
      errno = ENXIO;
    } else if (atomSemGet(&req.sema, 0) != ATOM_OK) {
      errno = EBUSY;
    } else if (req.result < 0) {
      errno = -req.result;
    } else {
      ret = 0;
    }

    atomSemDelete(&req.sema);
  }

  return ret;
}

int FL2000::setFrame(const void *fb, size_t pitch) {
  frame_request req = {
    .src = fb,
    .pitch = pitch,
  };
  threadMsg msg = {
    .cmd = CMD_SET_NEXT_FRAME,
    .frame_req = &req
  };

  return forwardMsg(req, msg);
}

FLASHMEM int FL2000::setFormat(const struct mode_timing& mode, int32_t input_format, int32_t output_format) {
  mode_request req = {
    .mode = mode,
    .input_format = input_format,
    .output_format = output_format,
  };
  threadMsg msg = {
    .cmd = CMD_SET_MODE,
    .mode_req = &req
  };

  return forwardMsg(req, msg);
}

int FL2000::setPalette(uint8_t index, size_t count, const uint32_t *colors) {
  pal_request req = {
    .src = colors,
    .start = index,
    .count = count
  };
  threadMsg msg = {
    .cmd = CMD_SET_PALETTE,
    .pal_req = &req
  };

  return forwardMsg(req, msg);
}

FLASHMEM int FL2000::setFormat(unsigned short width, unsigned short height, unsigned short refresh, int32_t input_format, int32_t output_format) {
  for (size_t i=0; i < sizeof(vid_modes)/sizeof(vid_modes[0]); i++) {
    if (vid_modes[i].active_width != width) continue;
    if (vid_modes[i].active_height != height) continue;
    if (vid_modes[i].refresh_rate != refresh) continue;
    return setFormat(vid_modes[i], input_format, output_format);
  }

  errno = EINVAL;
  return -1;
}

class FL2000DMA {
  using DMARequest = FL2000::DMARequest;
private:
  static DMARequest* queue;

  static void isr(void) {
    atomIntEnter();
    ch1.clearInterrupt();
    DMARequest *head, *next;
    do {
      head = LoadExclusivePtr(&queue);
      next = head->next;
    } while (StoreExclusivePtr(next, &queue));
    if (next) {
      ch1 = next->ch1;
      ch2 = next->ch2;
    }
    head->callback();
    atomIntExit(FALSE);
    asm volatile("dsb");
  }

  struct oneTimeInit {
    oneTimeInit() {
      queue = NULL;
      ch1.disable();
      ch2.disable();
      ch1.attachInterrupt(isr);
    }
  };

public:
  static DMAChannel ch1;
  static DMAChannel ch2;

  static void submit_request(DMARequest* req) {
    static oneTimeInit init;

    DMARequest** head;
    do {
      head = &queue;
      // this relies on LDREX/STREX not actually caring about a non-matching pointer i.e. a full range reservation
      DMARequest* p = LoadExclusivePtr(head);
      while (p != NULL) {
        head = &p->next;
        p = *head;
      }

      req->next = NULL;
    } while (StoreExclusivePtr(req, head));

    if (head == &queue) {
      ch1 = req->ch1;
      ch2 = req->ch2;
    }
  }
};

FL2000DMA::DMARequest* FL2000DMA::queue;
DMAChannel FL2000DMA::ch1;
DMAChannel FL2000DMA::ch2;

void FL2000::convert_dma(uint8_t* dst, uint32_t height, void* pdma) {
  uint32_t line_width = current_mode.active_width * output_bytes_per_pixel;
  uint32_t slice_size = height * line_width;

  if (current_src == NULL) {
    send_slice(dst, slice_size);
    return;
  }

  DMARequest *req = (DMARequest*)pdma;
  DMASetting& s1 = req->ch1;
  DMASetting& s2 = req->ch2;
  req->callback = [=](void) {
    threadMsg msg = {
      .cmd = CMD_SEND_SLICE,
      .slice = {
        .block  = dst,
        .dma = pdma,
        .id = slice_size
      }
    };
    if (atomQueuePut(&workQueue, -1, &msg) != ATOM_OK) {
      digitalWriteFast(13, 1);
    }
  };

  s2.TCD->SADDR = current_src;
  s2.TCD->SOFF = 8;
  s2.TCD->ATTR_SRC = DMA_TCD_ATTR_SIZE_64BIT;
  s2.TCD->ATTR_DST = DMA_TCD_ATTR_SIZE_32BIT;
  s2.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_DMLOE | DMA_TCD_NBYTES_MLOFFYES_MLOFF(16) | DMA_TCD_NBYTES_MLOFFYES_NBYTES(8);
  s2.TCD->DADDR = dst + 4;
  s2.TCD->DOFF = -4;
  s2.TCD->CSR = DMA_TCD_CSR_START;

  if (current_mode.flags & VIDMODE_FLAG_LINEDOUBLE) {
    // s1 = copy odd dst lines to even dst lines
    s1.TCD->SADDR = dst;
    s1.TCD->SOFF = 8;
    s1.TCD->ATTR_SRC = DMA_TCD_ATTR_SIZE_64BIT;
    s1.TCD->ATTR_DST = DMA_TCD_ATTR_SIZE_64BIT;
    s1.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_DMLOE | DMA_TCD_NBYTES_MLOFFYES_MLOFF(line_width) | DMA_TCD_NBYTES_MLOFFYES_NBYTES(line_width);
    s1.TCD->DADDR = dst+line_width;
    s1.TCD->DOFF = 8;
    s1.TCD->DLASTSGA = 0;
    s1.TCD->CITER = s1.TCD->BITER = height / 2;
    s1.TCD->CSR = 0;
    // s2 = reorder src dwords to dst odd lines
    s2.TCD->SLAST = current_pitch - line_width;
    s2.TCD->DLASTSGA = line_width + 16; // odd lines only
    s2.TCD->CITER = s2.TCD->BITER = line_width / 8;

    cache_flush(current_src, height * current_pitch/2);
    current_src += height * current_pitch/2;
  } else {
    // s1 = copy last 8 bytes of each row
    s1.TCD->SADDR = current_src + line_width - 8;
    s1.TCD->SOFF = current_pitch;
    s1.TCD->ATTR_SRC = DMA_TCD_ATTR_SIZE_64BIT;
    s1.TCD->ATTR_DST = DMA_TCD_ATTR_SIZE_32BIT;
    s1.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_DMLOE | DMA_TCD_NBYTES_MLOFFYES_MLOFF(line_width + 8) | DMA_TCD_NBYTES_MLOFFYES_NBYTES(8);
    s1.TCD->DADDR = dst + line_width - 4;
    s1.TCD->DOFF = -4;
    s1.TCD->DLASTSGA = 0;
    s1.TCD->CITER = s1.TCD->BITER = height;
    s1.TCD->CSR = 0;
    // s2 = copy src dwords to dst except last 8 bytes
    s2.TCD->SLAST = current_pitch - (line_width - 8);
    s2.TCD->DLASTSGA = 16+8;
    s2.TCD->CITER = s2.TCD->BITER = (line_width / 8) - 1;

    cache_flush(current_src, height * current_pitch);
    current_src += height * current_pitch;
  }

  // end of s2 minor loop triggers ch2
  FL2000DMA::ch2.triggerAtTransfersOf(s2);
  // end of s2 major loop triggers ch1
  FL2000DMA::ch1.triggerAtCompletionOf(s2);
  // end of s1 minor loop triggers ch2
  FL2000DMA::ch2.triggerAtTransfersOf(s1);
  // end of s1 major signals completion
  s1.interruptAtCompletion();

  FL2000DMA::submit_request(req);
}

void FL2000::convert_rgb24_to_rgb8(uint8_t* dst, uint32_t height,void*) {
  uint32_t width = current_mode.active_width;
  bool linedouble = current_mode.flags & VIDMODE_FLAG_LINEDOUBLE;
  const uint8_t* src = current_src;
  uint8_t* dst0 = dst;

  if (src) {
    for (uint32_t i=0; i < height; i++) {
      for (size_t j=0; j < width; j++) {
        uint8_t b = src[j*3+0] & 0xE0;
        uint8_t g = src[j*3+1] & 0xE0;
        uint8_t r = src[j*3+2] & 0xC0;
        dst[j ^ 4] = r | (g>>2) | (b>>5);
      }

      src += current_pitch;
      dst += width;
      if (linedouble) {
        memcpy(dst, dst-width, width);
        dst += width;
        i++;
      }
    }

    current_src = src;
  }

  send_slice(dst0, height*width);
}

void FL2000::convert_rgb565_to_rgb8(uint8_t* dst, uint32_t height,void*) {
  uint32_t width = current_mode.active_width;
  bool linedouble = current_mode.flags & VIDMODE_FLAG_LINEDOUBLE;
  const uint8_t* src = current_src;
  uint8_t* dst0 = dst;

  if (src) {
    for (size_t i=0; i < height; i++) {
      for (size_t j=0; j < width; j++) {
        uint8_t b = src[j*2+0] << 3;
        uint8_t g = src[j*2+1] << 5;
        uint8_t r = src[j*2+1] & 0xC0;
        dst[j ^ 4] = r | (g>>2) | (b>>5);
      }

      src += current_pitch;
      dst += width;
      if (linedouble) {
        memcpy(dst, dst-width, width);
        dst += width;
        i++;
      }
    }

    current_src = src;
  }

  send_slice(dst0, height*width);
}

void FL2000::convert_rgb555_to_rgb8(uint8_t* dst, uint32_t height,void*) {
  uint32_t width = current_mode.active_width;
  bool linedouble = current_mode.flags & VIDMODE_FLAG_LINEDOUBLE;
  const uint8_t* src = current_src;
  uint8_t* dst0 = dst;

  if (src) {
    for (size_t i=0; i < height; i++) {
      for (size_t j=0; j < width; j++) {
        uint8_t b = src[j*2+0] << 3;
        uint8_t g = (src[j*2+1] << 4) | (src[j*2+0] >> 4);
        uint8_t r = src[j*2+1] >> 5;
        dst[j ^ 4] = (r<<6) | (g&0x38) | (b>>5);
      }

      src += current_pitch;
      dst += width;
      if (linedouble) {
        memcpy(dst, dst-width, width);
        dst += width;
        i++;
      }
    }

    current_src = src;
  }

  send_slice(dst0, height*width);
}

void FL2000::convert_copy(uint8_t* dst, uint32_t height,void*) {
  uint32_t width = current_mode.active_width * output_bytes_per_pixel;
  bool linedouble = current_mode.flags & VIDMODE_FLAG_LINEDOUBLE;
  size_t pitch = current_pitch;

  uint32_t* d = (uint32_t*)dst;
  const uint32_t* s = (const uint32_t*)current_src;
  uint32_t w = width / 4;
  pitch /= 4;

  if (s) {
    for (size_t i=0; i < height; i++) {
      for (size_t j=0; j < w; j+=2) {
        *d++ = s[j+1];
        *d++ = s[j+0];
      }

      s += pitch;
      if (linedouble) {
        memcpy(d, d-w, width);
        d += w;
        i++;
      }
    }

    current_src = (const uint8_t*)s;
  }

  send_slice(dst, height*width);
}

struct conversion_entry {
  int32_t input_format;
  int32_t output_format;
  void (FL2000::*func)(uint8_t*,uint32_t,void*);
};

FLASHMEM int FL2000::set_mode(const struct mode_timing& mode, int32_t input_format, int32_t output_format) {
  static const struct conversion_entry convert_table[] PROGMEM = {
    {COLOR_FORMAT_RGB_24,         COLOR_FORMAT_RGB_8_332,     &FL2000::convert_rgb24_to_rgb8},
    {COLOR_FORMAT_RGB_16_565,     COLOR_FORMAT_RGB_8_332,     &FL2000::convert_rgb565_to_rgb8},
    {COLOR_FORMAT_RGB_16_555,     COLOR_FORMAT_RGB_8_332,     &FL2000::convert_rgb555_to_rgb8},
    {COLOR_FORMAT_RGB_8_INDEXED,  COLOR_FORMAT_RGB_8_INDEXED, &FL2000::convert_dma},
    {COLOR_FORMAT_RGB_8_332,      COLOR_FORMAT_RGB_8_332,     &FL2000::convert_dma},
    {COLOR_FORMAT_RGB_16_555,     COLOR_FORMAT_RGB_16_555,    &FL2000::convert_dma},
    {COLOR_FORMAT_RGB_16_565,     COLOR_FORMAT_RGB_16_565,    &FL2000::convert_dma},
  };

  if (!monitor_plugged_in)
    return -ENODEV;

  // check if timing parameters look valid
  switch (mode.prescaler) {
    case 1:
      if (mode.mult < 7 || mode.mult > 100)
        return -EINVAL;
      break;
    case 2:
      if (mode.mult < 13 || mode.mult > 128)
        return -EINVAL;
      break;
    default:
      return -EINVAL;
  }
  if (mode.divisor < 1 || mode.divisor > 128)
    return -EINVAL;

  // width must be a multiple of 8
  if (mode.active_width & 7)
    return -EINVAL;

  if (mode.active_width >= 1024)
    return -EINVAL;

  if (input_format == COLOR_FORMAT_AUTO) {
    // input format must be specified
    return -EINVAL;
  }

  if (output_format == COLOR_FORMAT_AUTO) {
    output_format = input_format;
  } else if (input_format & COLOR_FORMAT_COMPRESSED) {
    // if input is compressed make sure the output is too
    output_format |= COLOR_FORMAT_COMPRESSED;
  }

  int input = input_format & ~COLOR_FORMAT_COMPRESSED;
  int output = output_format & ~COLOR_FORMAT_COMPRESSED;

  if (input == COLOR_FORMAT_RGB_8_INDEXED) {
    if (output != COLOR_FORMAT_RGB_8_INDEXED)
      return -EINVAL;
  }

  // calculate bandwidth in pixels/sec
  uint32_t bps = mode.active_width * mode.active_height * mode.refresh_rate;
  if (mode.flags & VIDMODE_FLAG_LINEDOUBLE)
    bps *= 2;

  uint32_t bytes_per_pixel = 1;
  switch (output) {
    case COLOR_FORMAT_RGB_24:
      bytes_per_pixel = 3;
      break;
    case COLOR_FORMAT_RGB_16_565:
    case COLOR_FORMAT_RGB_16_555:
      bytes_per_pixel = 2;
      break;
  }

  // force compression for more than 30mbps
  if (bps * bytes_per_pixel > 30000000) {
    output_format |= COLOR_FORMAT_COMPRESSED;
  }

  convert_slice = NULL;
  for (size_t i=0; i < sizeof(convert_table)/sizeof(convert_table[0]); i++) {
    if (convert_table[i].input_format == input_format && convert_table[i].output_format == output_format) {
      convert_slice = convert_table[i].func;
      break;
    }
  }
  if (convert_slice == NULL) {
    dbg_log("Couldn't find a conversion function from %08X to %08X", input_format, output_format);
    return -EINVAL;
  }
  dbg_log("Using conversion function: %p", convert_slice);

  ++render_id;
  current_fb = NULL;
  next_fb = NULL;
  max_lines = mode.active_height * ((mode.flags & VIDMODE_FLAG_LINEDOUBLE) ? 2:1);
  sent_lines = 0;
  output_bytes_per_pixel = bytes_per_pixel;

  current_mode = mode;

  // initialize hardware...
  reg_vga_pll pll;
  int ret = reg_read(REG_VGA_PLL, pll.val);
  if (ret < 0) {
    dbg_log("Failed to read REG_VGA_PLL");
    return ret;
  }
  pll.divisor = mode.divisor;
  pll.prescaler = mode.prescaler;
  pll.multiplier = mode.mult;
  uint32_t mclk = 10 / mode.prescaler * mode.mult;
  if (mclk < 125) pll.function = 0;
  else if (mclk < 250) pll.function = 1;
  else if (mclk < 500) pll.function = 2;
  else pll.function = 3;
  ret = reg_write(REG_VGA_PLL, pll.val);
  if (ret < 0) {
    dbg_log("Failed to write REG_VGA_PLL");
    return ret;
  }

  reg_reset_ctrl sw_reset = { .sw_reset = 1 };
  ret = reg_set(REG_RESET_CTRL, sw_reset.val);
  if (ret < 0) {
    dbg_log("Failed to write REG_RESET_CTRL");
    return ret;
  }

  // confirm PLL
  reg_vga_pll new_pll;
  ret = reg_read(REG_VGA_PLL, new_pll.val);
  if (ret < 0) {
    dbg_log("Failed to read REG_VGA_PLL");
    return ret;
  }
  if (new_pll.val != pll.val) {
    dbg_log("PLL mismatch");
    return -EIO;
  }

  reg_vga_control vga_ctrl = { .lbuf_vde_rst_en = 1 };
  ret = reg_clear(REG_VGA_CONTROL, vga_ctrl.val);
  if (ret < 0) return ret;

  reg_vga_dac_control dac_ctrl;
  ret = reg_read(REG_VGA_DAC_CONTROL, dac_ctrl.val);
  if (ret < 0) return ret;

  dac_ctrl.hsync_polarity = (mode.flags & VIDMODE_FLAG_HSYNC_POS) ? 1:0;
  dac_ctrl.vsync_polarity = (mode.flags & VIDMODE_FLAG_VSYNC_POS) ? 1:0;
  dac_ctrl.clear_125us_cnt = 0;
  dac_ctrl.v565_mode = 0;
  dac_ctrl.v555_mode = 0;
  dac_ctrl.compress = (output_format & COLOR_FORMAT_COMPRESSED) ? 1 : 0;
  dac_ctrl.v332_mode = 0;
  dac_ctrl.color_palette_en = 0;
  dac_ctrl.first_bt_enc_en = 0;
  dac_ctrl.clear_watermark = 1;
  dac_ctrl.dac_output_en = 1;
  switch (output) {
    case COLOR_FORMAT_RGB_8_INDEXED:
      dac_ctrl.color_palette_en = 1;
    case COLOR_FORMAT_RGB_8_332:
      dac_ctrl.v332_mode = 1;
      break;
    case COLOR_FORMAT_RGB_16_565:
      dac_ctrl.v565_mode = 1;
      break;
    case COLOR_FORMAT_RGB_16_555:
      dac_ctrl.v555_mode = 1;
      break;
  }
  ret = reg_write(REG_VGA_DAC_CONTROL, dac_ctrl.val);
  if (ret < 0) return ret;

  reg_vga_hsync1 hsync1 = { .htotal = mode.h_total, .hactive = mode.active_width };
  ret = reg_write(REG_VGA_HSYNC1, hsync1.val);
  if (ret < 0) return ret;

  reg_vga_hsync2 hsync2 = {
    .hstart = (uint32_t)(mode.h_sync + mode.h_back_porch + 1),
    .hsync_width = mode.h_sync
  };
  ret = reg_write(REG_VGA_HSYNC2, hsync2.val);
  if (ret < 0) return ret;

  reg_vga_vsync1 vsync1 = {
    .vtotal = mode.v_total,
    .vactive = (uint32_t)(mode.active_height * ((mode.flags & VIDMODE_FLAG_LINEDOUBLE) ? 2:1))
  };
  ret = reg_write(REG_VGA_VSYNC1, vsync1.val);
  if (ret < 0) return 0;

  reg_vga_vsync2 vsync2 = {
    .vstart = (uint32_t)(mode.v_sync + mode.v_back_porch + 1),
    .vsync_width = mode.v_sync,
    .start_latency = (uint32_t)(mode.v_sync + mode.v_back_porch + 1)
  };
  ret = reg_write(REG_VGA_VSYNC2, vsync2.val);
  if (ret < 0) return ret;

  reg_vga_isoch isoch = { .mframe_cnt = 0x3FFF };
  ret = reg_clear(REG_VGA_ISOCH, isoch.val);
  if (ret < 0) return ret;

  reg_usb_lpm lpm = { .magic = 1 };
  ret = reg_set(REG_USB_LPM, lpm.val);
  if (ret < 0) return ret;

  if (has_ITE66121) {
    ret = hdmi_init();
    if (ret < 0) return ret;
    dbg_log("hdmi_init complete");
  }

  cache_invalidate(bulk_data[0], sizeof(bulk_data[0]));
  cache_invalidate(bulk_data[1], sizeof(bulk_data[1]));

  return frame_begin();
}

FLASHMEM int FL2000::hdmi_init(void) {
  reg_vga_control vga_control = {
    .force_vga_connect = 1
  };

  // force vga connected
  int ret = reg_set(REG_VGA_CONTROL, vga_control.val);
  if (ret < 0) return ret;

  // IT66121 programming guide, chap 6:
  // 1. Set regC1[0]=1 for AVMUTE the output
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AVMUTE, ITE_REG_AVMUTE_MUTE, ITE_REG_AVMUTE_MUTE);
  if (ret < 0) return ret;

  // 2. program input signal type
  // reset video clock
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_SW_RESET, ITE_REG_SW_RESET_VIDEO_CLOCK, ITE_REG_SW_RESET_VIDEO_CLOCK);
  if (ret < 0) return ret;
  // input is RGB, non-CCIR656, sync separate, SDR, PCLK delay=1
  ret = i2c_write_byte(I2C_ADDRESS_HDMI, ITE_REG_INCOLOR, ITE_REG_INCOLOR_MODE_RGB|1);
  if (ret < 0) return ret;
  // don't generate signals / use syncs from FL2000
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_SIGNALGEN, 0, ITE_REG_SIGNALGEN_GENSYNC|ITE_REG_SIGNALGEN_GENDE);
  if (ret < 0) return ret;

  // 3. set color space conversion
  // no conversion: input is RGB444, output is RGB444
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_CSC_CONTROL, 0,
    ITE_REG_CSC_CONTROL_DITHER | ITE_REG_CSC_CONTROL_UDFILTER | ITE_REG_CSC_CONTROL_DNFREEGO | ITE_REG_CSC_CONTROL_MODE);
  if (ret < 0) return ret;
  // set AVI Infoframe output mode to RGB444
  // needs register bank 1
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_GATE_BANK_CTRL, 1, ITE_REG_GATE_BANK_CTRL_BANK);
  if (ret >= 0) {
    ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_PKT_AVIINFO0, ITE_REG_PKT_AVIINFO0_Y_RGB, ITE_REG_PKT_AVIINFO0_Y);
    // back to register bank 0
    i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_GATE_BANK_CTRL, 0, ITE_REG_GATE_BANK_CTRL_BANK);
  }
  if (ret < 0) return ret;

  // 4. configure analog front end using the input video pixel clock
  // power off AFE
  ret = i2c_write_byte(I2C_ADDRESS_HDMI, ITE_REG_AFE_DRV, ITE_REG_AFE_DRV_RST);
  if (ret < 0) return ret;
  uint32_t pix_clk = 10000000 / current_mode.prescaler * current_mode.mult / current_mode.divisor;
  // use pixel repetition if clock is below HDMI minimum
  uint8_t pllpr = ITE_REG_CLOCK59_MANUALPLLPR_1;
  if (pix_clk < 12500000) {
    pllpr = ITE_REG_CLOCK59_MANUALPLLPR_4;
    pix_clk <<= 2;
  } else if (pix_clk < 25000000) {
    pllpr = ITE_REG_CLOCK59_MANUALPLLPR_2;
    pix_clk <<= 1;
  }
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_CLOCK59, ITE_REG_CLOCK59_DISLOCKPR|pllpr, ITE_REG_CLOCK59_DISLOCKPR|ITE_REG_CLOCK59_MANUALPLLPR);
  if (ret < 0) return ret;

  bool high_level = pix_clk > 80000000;
  dbg_log("hdmi_init: pix_clk is %u", pix_clk);
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AFE_XP, high_level ? ITE_REG_AFE_XP_GAINBIT : ITE_REG_AFE_XP_ER0, ITE_REG_AFE_XP_GAINBIT|ITE_REG_AFE_XP_ER0);
  if (ret < 0) return ret;
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AFE_IP, high_level ? ITE_REG_AFE_IP_GAINBIT : (ITE_REG_AFE_IP_ER0|ITE_REG_AFE_IP_EC1),
    ITE_REG_AFE_IP_GAINBIT|ITE_REG_AFE_IP_ER0|ITE_REG_AFE_IP_EC1);
  if (ret < 0) return ret;
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AFE_XPIP_MISC, high_level ? 0 : ITE_REG_AFE_XPIP_MISC_XP_EC1, ITE_REG_AFE_XPIP_MISC_XP_EC1);
  if (ret < 0) return ret;

  // 5. Set DVI mode (or HDMI but we don't use it)
  ret = i2c_write_byte(I2C_ADDRESS_HDMI, ITE_REG_HDMI_MODE, 0);
  if (ret < 0) return ret;

  // clear resets
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_SW_RESET, 0, ITE_REG_SW_RESET_REF_CLOCK | ITE_REG_SW_RESET_VIDEO_CLOCK);
  if (ret < 0) return ret;

  // 6-7. HDCP and audio - skip
  // 8. Clear AVMUTE by setting regC1[0]=0
  ret = i2c_write_byte_masked(I2C_ADDRESS_HDMI, ITE_REG_AVMUTE, 0, ITE_REG_AVMUTE_MUTE);
  if (ret < 0) return ret;

  // now fire AFE
  ret = i2c_write_byte(I2C_ADDRESS_HDMI, ITE_REG_AFE_DRV, 0);
  if (ret < 0) return ret;

  // clear vga connected
  return reg_clear(REG_VGA_CONTROL, vga_control.val);
}

int FL2000::frame_begin(void) {
  current_src = current_fb;
  sent_lines = 0;

  if (current_src == NULL) {
    memset(bulk_data[0], 0, sizeof(bulk_data[0]));
    memset(bulk_data[1], 0, sizeof(bulk_data[1]));
  }

  begin_slice(bulk_data[0], &dma_req[0]);
  if (sent_lines < max_lines)
    begin_slice(bulk_data[1], &dma_req[1]);

  return 0;
}

void FL2000::begin_slice(uint8_t* dst, void* dma) {
  if (sent_lines >= max_lines) {
    dbg_log("begin_slice sent_lines >= max_lines");
    while(1);
  }
  uint16_t lines_to_fill = max_lines - sent_lines;
  uint16_t max_slice_lines = FL2000_SLICE_SIZE / (current_mode.active_width * output_bytes_per_pixel);
  if (current_mode.flags & VIDMODE_FLAG_LINEDOUBLE) max_slice_lines &= ~1;

  if (lines_to_fill > max_slice_lines)
    lines_to_fill = max_slice_lines;

  if (lines_to_fill == 0) {
    dbg_log("lines_to_fill ==0");
    while(1);
  }

  sent_lines += lines_to_fill;
  // fill output
  (this->*convert_slice)(dst, lines_to_fill, dma);
}

void FL2000::send_slice(uint8_t* dst, size_t slice_len, void* dma) {
  if (BulkMessage(1, slice_len, dst, [=](int r) {
    if (r >= 0) {
      if (1 || sent_lines < max_lines) {
        threadMsg msg = {
          .cmd = CMD_SLICE_DONE,
          .slice = {
            .block = dst,
            .dma = dma,
            .id = render_id
          }
        };
        if (atomQueuePut(&workQueue, -1, &msg) != ATOM_OK) {
          dbg_log("Failed to send CMD_SLICE_DONE msg");
          digitalWriteFast(13, 1);
        }
      }
    } else {
      dbg_log("BulkMessage returned error: %d", r);
      digitalWriteFast(13, 1);
    }
  }) < 0) {
    dbg_log("Sending BulkMessage failed");
    while(1);
  }

  if (sent_lines == max_lines) {
    ++sent_lines;
    if (monitor_notify)
      monitor_notify->triggerEvent(MONITOR_NOTIFY_FRAMEDONE, this);

    if (BulkMessage(1, 0, NULL, [=](int r) {
      if (r >= 0) {
        threadMsg msg = {
          .cmd = CMD_FRAME_DONE,
          .slice = {
            .id = render_id
          }
        };
        if (atomQueuePut(&workQueue, -1, &msg) != ATOM_OK) {
          dbg_log("Failed to send CMD_FRAME_DONE msg");
          digitalWriteFast(13, 1);
        }
      } else {
        digitalWriteFast(13, 1);
      }
    }) < 0) {
      dbg_log("Sending ZLP BulkMessage failed");
      while(1);
    }
  }
}

FLASHMEM int FL2000::calcTiming(const uint32_t freq, uint8_t& prescaler, uint8_t& mult, uint8_t& divisor) {
  const uint32_t base = 10000000;

  uint32_t best_diff = 500000;

  for (uint8_t p = 1; p < 3; p++) {
    uint32_t prescale_base = base / p;
    for (uint8_t m = 1; m < 129; m++) {
      uint32_t mult_base = prescale_base * m;

      // minimum 62.5MHz
      if (mult_base < 62500000)
        continue;

      // maximum 800MHz (maybe? 10*81/80 does not work, 5*81/40 does...)
      if (mult_base > 800000000)
        break;

      for (uint8_t d = 1; d < 129; d++) {
        uint32_t clk = mult_base / d;
        uint32_t diff = freq < clk ? (clk - freq) : (freq - clk);
        if (diff < best_diff) {
          best_diff = diff;
          prescaler = p;
          mult = m;
          divisor = d;
          if (diff == 0)
            return 0;
        } else if (clk < freq) {
          break;
        }
      }
    }
  }


  if (best_diff < 500000)
    return 0;

  errno = ERANGE;
  return -1;
}

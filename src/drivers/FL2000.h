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

#ifndef _USB_FL2000_H
#define _USB_FL2000_H

#include "../teensy4_usbhost.h"
#include <EventResponder.h>
#include <DMAChannel.h>
#include <vector>

enum notify_status {
  /* something went wrong (e.g. device unplugged, communication error)
   * assume device is unusable after this. */
  MONITOR_NOTIFY_ERROR = -1,
  /* For VGA ports, the device seems to detect the presence of a connected
   * monitor by weak-pulling up a ground pin and detecting when it goes low.
   * This isn't 100% reliable since some cables connect all their ground pins
   * together in the plug, rather than passing them through to a connected monitor.
   * In other words this may only detect a connected cable rather than a VGA screen. */
  MONITOR_NOTIFY_DISCONNECTED,
  MONITOR_NOTIFY_CONNECTED,
  /* This notifies when the previous frame has been completely passed to the device.
   * The source address of the retired frame is the data for the event. */
  MONITOR_NOTIFY_FRAMEDONE,
  /* This notifies when the device detects a change in EDID information. This can be
   * a somewhat more reliable indicator of a VGA screen being (dis)connected, assuming
   * the monitor supports EDID/DDC in the first place.
   * Data for the event: a pointer to 128 bytes (page 0) of the new EDID, else NULL if
   * EDID is not available. */
  MONITOR_NOTIFY_EDID,
};

#define COLOR_FORMAT_AUTO            -1
#define COLOR_FORMAT_RGB_24          0
#define COLOR_FORMAT_RGB_16_565      1
#define COLOR_FORMAT_RGB_16_555      2
#define COLOR_FORMAT_RGB_8_332       3        // note this is not the "typical" 332 - it is RRGGGBBB
#define COLOR_FORMAT_RGB_8_INDEXED   4
#define COLOR_FORMAT_COMPRESSED      0x80000000
#define COLOR_FORMAT_NODMA           0x40000000   // this avoids using DMA to copy input framedata into USB slice buffers

#define VIDMODE_FLAG_HSYNC_POS     (1<<0)
#define VIDMODE_FLAG_VSYNC_POS     (1<<1)
#define VIDMODE_FLAG_LINEDOUBLE    (1<<2)

struct mode_timing {
  uint16_t active_width;
  uint16_t active_height;
  uint16_t refresh_rate;

  uint16_t h_total;
  uint16_t h_sync;
  uint16_t h_back_porch;
  uint16_t v_total;
  uint16_t v_sync;
  uint16_t v_back_porch;

  // based on 10MHz base clock
  // pixel clock = base / prescaler * mult / divisor
  uint8_t prescaler;   // 1-2
  uint8_t mult;        // prescaler=1: 7-100, prescaler=2: 13-128
  uint8_t divisor;     // 1-128

  uint32_t flags;
};

#define FL2000_SLICE_SIZE    (16*1024)

class FL2000 : public USB_Driver, public USB_Driver::Factory {
friend class FL2000DMA;
private:
  uint32_t reg_data[8] __attribute__((aligned(32)));
  uint8_t intr_data[32] __attribute__((aligned(32)));
  uint8_t bulk_data[2][FL2000_SLICE_SIZE] __attribute__((aligned(32)));

  struct threadMsg;

  ATOM_TCB workThread;
  ATOM_QUEUE workQueue;
  uint32_t workStack[512];
  void thread(void);
  static void threadStart(uint32_t arg);

  int dbg_log(const char* fmt, ...) const;

  EventResponder *monitor_notify = NULL;

  bool monitor_plugged_in;
  bool has_ITE66121;
  uint8_t edid[128];

  mode_timing current_mode;

  const uint8_t* current_fb;
  size_t current_pitch;
  uint8_t current_fixed_bits;
  const uint8_t* next_fb;
  size_t next_pitch;
  uint8_t next_fixed_bits;

  const uint8_t* current_src;
  uint16_t next_line;
  uint16_t max_lines;
  uint32_t render_id = 0;
  uint32_t output_bytes_per_pixel;

  struct DMARequest {
    DMASetting line_copy;
    DMASetting line_count;
    std::function<void(void)> callback;
    struct DMARequest *next;
  };

  struct slice_data {
    uint8_t* const data;
    DMARequest dma_req;
    std::vector<usb_bulkintr_sg> sg;
    bool last;
  } slices[2] = {
    {bulk_data[0]},
    {bulk_data[1]}
  };

  int reg_write(uint16_t offset, const uint32_t val);
  int reg_read(uint16_t offset, uint32_t& val);

  int reg_set(uint16_t offset, const uint32_t val);
  int reg_clear(uint16_t offset, const uint32_t val);

  int i2c_transfer(uint8_t address, uint8_t offset, bool read);
  int i2c_read_dword(uint8_t address, uint8_t offset, uint32_t& val);
  int i2c_write_dword(uint8_t address, uint8_t offset, const uint32_t val);
  int i2c_read_byte(uint8_t address, uint8_t offset, uint8_t& val);
  int i2c_write_byte(uint8_t address, uint8_t offset, const uint8_t val);
  int i2c_write_byte_masked(uint8_t address, uint8_t offset, const uint8_t val, const uint8_t mask);

  int hdmi_init(void);

  int process_interrupt(void);
  int set_mode(const struct mode_timing& mode, int32_t input_format, int32_t output_format);

  int frame_begin(void);
  void begin_slice(slice_data*);
  void send_slice(slice_data*,size_t slice_len);

  int device_init(void);

  // Factory overrides
  bool offer(const usb_device_descriptor* d, const usb_configuration_descriptor *cd) override;
  USB_Driver* attach(const usb_device_descriptor *d, const usb_configuration_descriptor*, USB_Device *dev) override;
  void detach(void) override;

  int forwardMsg(struct sync_request& req, threadMsg& msg);

  void convert_dma_init(slice_data *slice);

  void (FL2000::*convert_slice)(slice_data* slice, uint32_t height);
  void convert_rgb24_to_rgb8(slice_data*,uint32_t);
  void convert_rgb565_to_rgb8(slice_data*,uint32_t);
  void convert_rgb555_to_rgb8(slice_data*,uint32_t);
  void convert_copy(slice_data*,uint32_t);
  void convert_dma(slice_data*,uint32_t);

public:
  FL2000();
  ~FL2000();

  // returns previously set responder
  EventResponder* set_monitor_event(EventResponder*);

  // simple method to set resolution: searches an internal table for a matching timing mode
  // refresh may be 0 to match any available framerate
  int setFormat(unsigned short width, unsigned short height, unsigned short refresh, int32_t input_format, int32_t output_format=COLOR_FORMAT_AUTO);
  // advanced method: must supply timing mode information
  int setFormat(const struct mode_timing& mode, int32_t input_format, int32_t output_format=COLOR_FORMAT_AUTO);

  // sets the next framebuffer to be rendered, must match the specified format
  // fixedb specifies how many address bits (starting with the MSB) are fixed, for wraparound buffers
  int setFrame(const void *fb, size_t pitch, uint8_t fixb=0);

  // sets one or more palette entries, 32-bit value = 0x00RRGGBB
  int setPalette(uint8_t index, size_t count, const uint32_t* colors);

  // helper function to find PLL parameters for specific frequencies
  static int calcTiming(const uint32_t freq, uint8_t& prescaler, uint8_t& mult, uint8_t& divisor);
};

#endif // _USB_FL2000_H

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

#ifndef _FL2000_REGS_H
#define _FL2000_REGS_H

#include <stdint.h>

#define REG_USB_LPM            0x0070
typedef union {
  struct {
    uint32_t          :13;
    uint32_t magic    :1;
    uint32_t          :5;
    uint32_t u2_reject:1;
    uint32_t u1_reject:1;
    uint32_t          :11;
  };
  uint32_t val;
} reg_usb_lpm;

#define REG_VGA_STATUS         0x8000
typedef union {
  struct {
    uint32_t vga_status        :1;
    uint32_t vga_error_int     :1;  /* self-clearing */
    uint32_t linebuf_halt      :1;
    uint32_t iso_ack_int       :1;  /* self-clearing */
    uint32_t td_drop_int       :1;  /* self-clearing */
    uint32_t irq_pending       :1;  /* self-clearing */
    uint32_t pll_status        :1;
    uint32_t dac_status        :1;
    uint32_t linebuf_underflow :1;
    uint32_t linebuf_overflow  :1;
    uint32_t framecount        :16;
    uint32_t hdmi_int          :1;  /* self-clearing */
    uint32_t hdmi_status       :1;
    uint32_t edid_status       :1;
    uint32_t monitor_status    :1;
    uint32_t monitor_int       :1;  /* self-clearing */
    uint32_t edid_int          :1;  /* self-clearing */
  };
  uint32_t val;
} reg_vga_status;

#define REG_VGA_DAC_CONTROL    0x8004
typedef union {
  struct {
    uint32_t clear_watermark   :1;     // 0
    uint32_t frame_sync        :1;     // 1
    uint32_t hsync_polarity    :1;     // 2 assuming 1 = positive, 0 = negative
    uint32_t vsync_polarity    :1;     // 3
    uint32_t de_polarity       :1;     // 4
    uint32_t mirror_mode       :1;     // 5 what does this do???
    uint32_t v565_mode         :1;     // 6
    uint32_t dac_output_en     :1;     // 7
    uint32_t timing_en         :1;     // 8
    uint32_t use_new_pkt_retry :1;     // 9
    uint32_t ref_select        :1;     // 10
    uint32_t dac_px_clk_invert :1;     // 11
    uint32_t clear_lbuf_status :1;     // 12
    uint32_t drop_cnt          :1;     // 13
    uint32_t use_vdi_itp_cnt   :1;     // 14
    uint32_t                   :9;     // 15
    uint32_t compress          :1;     // 24
    uint32_t v332_mode         :1;     // 25
    uint32_t color_palette_en  :1;     // 26
    uint32_t first_bt_enc_en   :1;     // 27
    uint32_t clear_125us_cnt   :1;     // 28
    uint32_t disable_halt      :1;     // 29
    uint32_t force_de_en       :1;     // 30
    uint32_t v555_mode         :1;     // 31
  };
  uint32_t val;
} reg_vga_dac_control;

#define REG_VGA_HSYNC1         0x8008
typedef union {
  struct {
    uint32_t htotal     :12;
    uint32_t            :4;
    uint32_t hactive    :12;
    uint32_t            :4;
  };
  uint32_t val;
} reg_vga_hsync1;

#define REG_VGA_HSYNC2         0x800C
typedef union {
  struct {
    uint32_t hstart     :12;
    uint32_t            :4;
    uint32_t hsync_width:12;
    uint32_t            :4;
  };
  uint32_t val;
} reg_vga_hsync2;

#define REG_VGA_VSYNC1         0x8010
typedef union {
  struct {
    uint32_t vtotal     :12;
    uint32_t            :4;
    uint32_t vactive    :12;
    uint32_t            :4;
  };
  uint32_t val;
} reg_vga_vsync1;

#define REG_VGA_VSYNC2         0x8014
typedef union {
  struct {
    uint32_t vstart     :12;
    uint32_t            :4;
    uint32_t vsync_width:3;
    uint32_t            :1;
    uint32_t start_latency:10;
    uint32_t            :1;
    uint32_t buf_error_en:1;
  };
  uint32_t val;
} reg_vga_vsync2;

#define REG_VGA_ISOCH          0x801C
typedef union {
  struct {
    uint32_t start_mframe_cnt   :14;
    uint32_t use_mframe_match   :1;
    uint32_t use_zero_len_frame :1;
    uint32_t mframe_cnt         :14;
    uint32_t mframe_cnt_update  :1;
    uint32_t                    :1;
  };
  uint32_t val;
} reg_vga_isoch;

#define REG_I2C_CONTROL        0x8020
typedef union {
  struct {
    uint32_t address         :7;
    uint32_t read            :1;  // 1=READ, 0=WRITE
    uint32_t offset          :8;
    uint32_t spi             :1;  // 1=SPI, 0=EEPROM
    uint32_t spi_erase       :1;
    uint32_t                 :6;
    uint32_t status          :4;  // one bit for each byte transferred, 1=failed / 0=pass
    uint32_t monitor_detect  :1;
    uint32_t                 :1;
    uint32_t edid_detect     :1;
    uint32_t complete        :1;
  };
  uint32_t val;
} reg_i2c_control;

#define REG_I2C_READ           0x8024
#define REG_I2C_WRITE          0x8028

#define REG_VGA_PLL            0x802C
// PLL frequency = 10MHz / prescaler * multiplier / divisor
typedef union {
  struct {
    uint32_t divisor          :8;
    uint32_t prescaler        :2;
    uint32_t                  :3;
    uint32_t function         :2;
    uint32_t                  :1;
    uint32_t multiplier       :8;
    uint32_t test_io          :1;
    uint32_t cfg_dac_pwrdown  :1;
    uint32_t force_dac_pwrup  :1;
    uint32_t                  :5;
  };
  uint32_t val;
} reg_vga_pll;

#define REG_VGA_CONTROL        0x803C
typedef union {
  struct {
    uint32_t cfg_timing_reset_n    :1;      // 0
    uint32_t plh_block_en          :1;      // 1
    uint32_t edid_mon_int_en       :1;      // 2
    uint32_t ext_mon_int_en        :1;      // 3
    // setting this to one disables self-clear bits in REG_VGA_STATUS
    uint32_t vga_status_self_clear_dis :1;  // 4
    uint32_t pll_lock_time         :5;      // 5
    uint32_t pll_fast_timeout_en   :1;      // 10
    uint32_t ppe_block_en          :1;      // 11
    uint32_t pll_timer_en          :1;      // 12
    uint32_t feedback_int_en       :1;      // 13
    uint32_t clr_125us_counter     :1;      // 14
    uint32_t ccs_pd_dis            :1;      // 15
    uint32_t standby_en            :1;      // 16
    uint32_t force_loopback        :1;      // 17
    uint32_t lbuf_drop_frame_en    :1;      // 18
    uint32_t lbuf_vde_rst_en       :1;      // 19
    uint32_t lbuf_sw_rst           :1;      // 20
    uint32_t lbuf_err_int_en       :1;      // 21
    uint32_t biac_en               :1;      // 22
    uint32_t pxclk_in_en           :1;      // 23
    uint32_t vga_err_int_en        :1;      // 24
    uint32_t force_vga_connect     :1;      // 25
    uint32_t force_pll_up          :1;      // 26
    uint32_t use_zero_td           :1;      // 27
    uint32_t use_zero_pkt_len      :1;      // 28
    uint32_t use_pkt_pending       :1;      // 29
    uint32_t pll_dac_pd_usbp3_en   :1;      // 30
    uint32_t pll_dac_pd_novga_en   :1;      // 31
  };
  uint32_t val;
} reg_vga_control;

#define REG_RESET_CTRL         0x8048
typedef union {
  struct {
    uint32_t dis_hot_rst2_port     :1;
    uint32_t dis_warm_rst2_port    :1;
    uint32_t dis_hot_reset_pipe    :1;
    uint32_t dis_warm_reset_pipe   :1;
    uint32_t dis_hot_reset_pix     :1;
    uint32_t dis_warm_reset_pix    :1;
    uint32_t dis_usb2_reset_pix    :1;
    uint32_t dis_pll_reset_pix     :1;
    uint32_t dis_sw_reset_pix      :1;
    uint32_t dis_usb2_reset_buf    :1;
    uint32_t dis_sw_reset_buf      :1;
    uint32_t dis_lbuf_reset_pix    :1;
    uint32_t dis_hot_reset_port    :1;
    uint32_t dis_warm_reset_port   :1;
    uint32_t set_slow_clk_predft   :1;
    uint32_t sw_reset              :1;
    uint32_t frame_first_itp_wl    :16;
  };
  uint32_t val;
} reg_reset_ctrl;

#define REG_VGA_PLT_DATA       0x805C
typedef union {
  struct {
    uint32_t palette_ram_wr_addr :8;
    uint32_t palette_ram_data    :24;
  };
  uint32_t val;
} reg_vga_plt_data;

#define REG_VGA_PLT_RADDR      0x8060
typedef union {
  struct {
    uint32_t palette_ram_rd_addr        :8;
    uint32_t last_frame_lbuf_watermark  :16;
    uint32_t                            :8;
  };
  uint32_t val;
} reg_vga_plt_raddr;


#define ITE_REG_SW_RESET       0x04
#define ITE_REG_SW_RESET_HDCP            (1<<0)
#define ITE_REG_SW_RESET_AUDIO_FIFO      (1<<2)
#define ITE_REG_SW_RESET_VIDEO_CLOCK     (1<<3)
#define ITE_REG_SW_RESET_AUDIO_CLOCK     (1<<4)
#define ITE_REG_SW_RESET_REF_CLOCK       (1<<5)

#define ITE_REG_INT_CTRL       0x05
#define ITE_REG_INT_CTRL_TXCLK           (1<<0)

#define ITE_REG_GATE_BANK_CTRL 0x0F
#define ITE_REG_GATE_BANK_CTRL_BANK      (3<<0)
#define ITE_REG_GATE_BANK_CTRL_BANK_0    (0<<0)
#define ITE_REG_GATE_BANK_CTRL_BANK_1    (1<<0)
#define ITE_REG_GATE_BANK_CTRL_CRCLK     (1<<3)
#define ITE_REG_GATE_BANK_CTRL_TXCLK     (1<<4)
#define ITE_REG_GATE_BANK_CTRL_IACLK     (1<<5)
#define ITE_REG_GATE_BANK_CTRL_RCLK      (1<<6)
#define ITE_REG_GATE_BANK_CTRL_GATE_ALL  (ITE_REG_GATE_BANK_CTRL_CRCLK | ITE_REG_GATE_BANK_CTRL_TXCLK | ITE_REG_GATE_BANK_CTRL_IACLK | ITE_REG_GATE_BANK_CTRL_RCLK)

#define ITE_REG_CLOCK59        0x59
#define ITE_REG_CLOCK59_MANUALPLLPR      (3<<6)
#define ITE_REG_CLOCK59_MANUALPLLPR_1    (0<<6)
#define ITE_REG_CLOCK59_MANUALPLLPR_2    (1<<6)
#define ITE_REG_CLOCK59_MANUALPLLPR_4    (3<<6)
#define ITE_REG_CLOCK59_DISLOCKPR        (1<<4)

#define ITE_REG_AFE_DRV        0x61
#define ITE_REG_AFE_DRV_RST              (1<<4)
#define ITE_REG_AFE_DRV_PWD              (1<<5)

#define ITE_REG_AFE_XP         0x62
#define ITE_REG_AFE_XP_PWDI              (1<<2)
#define ITE_REG_AFE_XP_RESETB            (1<<3)
#define ITE_REG_AFE_XP_ER0               (1<<4)
#define ITE_REG_AFE_XP_PWDPLL            (1<<6)
#define ITE_REG_AFE_XP_GAINBIT           (1<<7)

#define ITE_REG_AFE_LEVEL      0x63
#define ITE_REG_AFE_LEVEL_DRV_ISW(a)     ((a&7)<<3)

#define ITE_REG_AFE_IP         0x64
#define ITE_REG_AFE_IP_EC1               (1<<0)
#define ITE_REG_AFE_IP_RESETB            (1<<2)
#define ITE_REG_AFE_IP_ER0               (1<<3)
#define ITE_REG_AFE_IP_PWDPLL            (1<<6)
#define ITE_REG_AFE_IP_GAINBIT           (1<<7)

#define ITE_REG_AFE_EN         0x66

#define ITE_REG_AFE_XPIP_MISC  0x68
#define ITE_REG_AFE_XPIP_MISC_XP_EC1     (1<<4)

#define ITE_REG_AFE_XP_TEST    0x6A

#define ITE_REG_INCOLOR        0x70
#define ITE_REG_INCOLOR_MODE             (3<<6)
#define ITE_REG_INCOLOR_MODE_RGB         (0<<6)
#define ITE_REG_INCOLOR_MODE_YUV422      (1<<6)
#define ITE_REG_INCOLOR_MODE_YUV444      (2<<6)
#define ITE_REG_INCOLOR_PCLKDIV2         (1<<5)
#define ITE_REG_INCOLOR_2X656CLK         (1<<4)
#define ITE_REG_INCOLOR_SYNCEMB          (1<<3)
#define ITE_REG_INCOLOR_DDR              (1<<2)
#define ITE_REG_INCOLOR_CLKDELAY         (3<<0)

#define ITE_REG_CSC_CONTROL    0x72
#define ITE_REG_CSC_CONTROL_DITHER       (1<<7)
#define ITE_REG_CSC_CONTROL_UDFILTER     (1<<6)
#define ITE_REG_CSC_CONTROL_DNFREEGO     (1<<5)
#define ITE_REG_CSC_CONTROL_MODE         (3<<0)
#define ITE_REG_CSC_CONTROL_RGB2YUV      (2<<0)
#define ITE_REG_CSC_CONTROL_YUV2RGB      (3<<0)

#define ITE_REG_SIGNALGEN      0x90
#define ITE_REG_SIGNALGEN_PGHTOTAL_LO(A) ((A)<<4)
#define ITE_REG_SIGNALGEN_GENSYNC        (1<<3)
#define ITE_REG_SIGNALGEN_VPOLARITY      (1<<2)
#define ITE_REG_SIGNALGEN_HPOLARITY      (1<<1)
#define ITE_REG_SIGNALGEN_GENDE          (1<<0)

#define ITE_REG_PGHTOTAL_HI    0x91
#define ITE_REG_PGHDES_LO      0x92
#define ITE_REG_PGHDEE_LO      0x93
#define ITE_REG_PGH_DHI        0x94
#define ITE_REG_PGH_DHI_DEE(A)           ((A)<<4)
#define ITE_REG_PGH_DHI_DES(A)           ((A)<<0)
#define ITE_REG_PGHRS_LO       0x95
#define ITE_REG_PGHRE_LO       0x96
#define ITE_REG_PGH_RHI        0x97
#define ITE_REG_PGH_RHI_HRE(A)           ((A)<<4)
#define ITE_REG_PGH_RHI_HRS(A)           ((A)<<0)
#define ITE_REG_PGV_TOTAL_LO   0x98
#define ITE_REG_PGV_TOTAL_HI   0x99
#define ITE_REG_PGV_DES_LO     0x9A
#define ITE_REG_PGV_DEE_LO     0x9B
#define ITE_REG_PGV_DE_HI      0x9C
#define ITE_REG_PGV_DE_HI_DEE(A)         ((A)<<4)
#define ITE_REG_PGV_DE_HI_DES(A)         ((A)<<0)


#define ITE_REG_HDMI_MODE      0xC0
#define ITE_REG_HDMI_MODE_HDMI           (1<<0)

#define ITE_REG_AVMUTE         0xC1
#define ITE_REG_AVMUTE_MUTE              (1<<0)
#define ITE_REG_AVMUTE_BLUE              (1<<1)

#define ITE_REG_PKT_AVIINFO0   ((0x158)&0xFF)
#define ITE_REG_PKT_AVIINFO0_Y           (3<<5)
#define ITE_REG_PKT_AVIINFO0_Y_RGB       (0<<5)
#define ITE_REG_PKT_AVIINFO0_Y_YUV422    (1<<5)
#define ITE_REG_PKT_AVIINFO0_Y_YUV444    (2<<5)

#endif

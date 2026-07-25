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

#include <memory>

#include "../../teensy4_usbhost.h"
#include "lmac_msg.h"

#include "fw_patch_table_8800d80_u02.h"
#include "fw_adid_8800d80_u02.h"
#include "fw_patch_8800d80_u02.h"
#include "fw_patch_8800d80_u02_ext.h"
#include "fmacfw_8800d80_u02.h"
#include "fmacfw_8800d80_h_u02.h"

#define RX_MSG_VALID_PATTERN 0xADDEDE2A

#define FW_RAM_ADID_BASE_ADDR         0x002017E0
#define FW_RAM_PATCH_BASE_ADDR        0x0020B2B0
#define RAM_FMAC_FW_ADDR              0x100000

#define FW_RAM_PATCH_BASE_ADDR_U02    0x0020B43C
#define FW_RAM_ADID_BASE_ADDR_U02     0x00201940
#define RAM_FMAC_FW_ADDR_U02          0x120000

#define CHIP_REV_U01      0x01
#define CHIP_REV_U02      0x03
#define CHIP_REV_U03      0x07
#define CHIP_REV_U04      0x0F
#define CHIP_REV_U05      0x1F
#define CHIP_SUB_REV_U04  0x20

#define CHIP_ID_H_MASK    0xC0
#define IS_CHIP_ID_H()    ((chip_id & CHIP_ID_H_MASK) == CHIP_ID_H_MASK)

#define AIC_PATCH_BLOCK_MAX      4

struct aic_patch_t {
  uint32_t magic_num;
  uint32_t pair_start;
  uint32_t magic_num_2;
  uint32_t pair_count;
  uint32_t block_dst[AIC_PATCH_BLOCK_MAX];
  uint32_t block_src[AIC_PATCH_BLOCK_MAX];
  uint32_t block_size[AIC_PATCH_BLOCK_MAX];
};

enum {
  AICBT_PT_INF = 0,
  AICBT_PT_TRAP,
  AICBT_PT_B4,
  AICBT_PT_BTMODE,
  AICBT_PT_PWRON,
  AICBT_PT_AF,
  AICBT_PT_VERSION
};

struct aicbt_patch_table {
  const char* name;
  uint32_t type;
  const uint32_t* data;
  uint32_t len;
  struct aicbt_patch_table* next = NULL;

  aicbt_patch_table(const uint8_t* &src) {
    name = (const char*)src;
    src += 16;

    type = *(uint32_t*)src;
    src += 4;

    len = *(uint32_t*)src;
    src += 4;

    if (len != 0) {
      if (type >= 1000)
        len = 0;
      else {
        data = (const uint32_t*)src;
        src += len*8;
      }
    }
  }

  ~aicbt_patch_table() {
    delete next;
  }
};

struct aicbt_patch_info_t {
  uint32_t info_len;
  uint32_t adid_addrinf;
  uint32_t addr_adid;
  uint32_t patch_addrinf;
  uint32_t addr_patch;
  uint32_t reset_addr;
  uint32_t reset_val;
  uint32_t adid_flag_addr;
  uint32_t adid_flag;
  uint32_t ext_patch_nb_addr;
  uint32_t ext_patch_nb;
  const uint32_t *ext_patch_param;
};

static struct aicbt_patch_table* aicbt_patch_table_alloc(const uint8_t* p, size_t size) {
  static const char AICBT_PT_TAG[12] PROGMEM = {
    'A', 'I', 'C', 'B', 'T', '_', 'P', 'T', '_', 'T', 'A', 'G'
  };

  struct aicbt_patch_table *head = NULL;
  struct aicbt_patch_table *cur = NULL;
  const uint8_t* end = p + size;

  dprintf("### Upload fw_patch_table, size=%u\n", size);

  if (memcmp(p, AICBT_PT_TAG, sizeof(AICBT_PT_TAG))) {
    dprintf("TAG err\n");
    return NULL;
  }
  p += 16;

  while (p < end) {
    auto next = new(std::nothrow) aicbt_patch_table(p);
    if (next == NULL) {
      delete head;
      return NULL;
    }
    if (head == NULL) {
      head = next;
    } else {
      cur->next = next;
    }
    cur = next;
  }

  return head;
}

static int aicbt_patch_info_unpack(struct aicbt_patch_info_t *patch_info, struct aicbt_patch_table *head) {
  if (head->type != AICBT_PT_INF)
    return 0;

  uint32_t base_len = (offsetof(struct aicbt_patch_info_t, ext_patch_nb_addr) - offsetof(struct aicbt_patch_info_t, adid_addrinf)) / (2*sizeof(uint32_t));
  dprintf("%s head->len:%lu base_len:%lu \n", __func__, head->len, base_len);
  uint32_t memcpy_len;

  if (head->len > base_len) {
    patch_info->info_len = base_len;
    memcpy_len = base_len + 1; // include ext patch nb
  } else {
    patch_info->info_len = memcpy_len = head->len;
  }
  head->len = patch_info->info_len;
  dprintf("%s memcpy_len:%lu\n", __func__, memcpy_len);

  if (patch_info->info_len) {
    switch (memcpy_len) {
      default:
        patch_info->ext_patch_nb_addr = head->data[8];
        patch_info->ext_patch_nb = head->data[9];
      case 4:
        patch_info->adid_flag_addr = head->data[6];
        patch_info->adid_flag = head->data[7];
      case 3:
        patch_info->reset_addr = head->data[4];
        patch_info->reset_val = head->data[5];
      case 2:
        patch_info->patch_addrinf = head->data[2];
        patch_info->addr_patch = head->data[3];
      case 1:
        patch_info->adid_addrinf = head->data[0];
        patch_info->addr_adid = head->data[1];
    }
    dprintf("%s adid_addrinf:%lx addr_adid:%lx\n", __func__, patch_info->adid_addrinf, patch_info->addr_adid);

    patch_info->ext_patch_param = head->data + memcpy_len*2;

    for (uint32_t i=0; i < patch_info->ext_patch_nb; i++) {
      dprintf("%s id:%lx addr:%lx\n", __func__, patch_info->ext_patch_param[i*2], patch_info->ext_patch_param[i*2 + 1]);
    }
  }

  return 0;
}

namespace AIC8800 {

class fwuploader : public USB_Driver {
  union {
    tx_msg txmsg;
    rx_msg rxmsg;
  } __attribute__((aligned(32)));

  uint8_t chip_id = 0;
  uint16_t tid = 0;

  const uint8_t bulk_out = 0x01;
  const uint8_t bulk_in = 0x82;

  EventResponder evt;
  static void handle_event(EventResponderRef);

  int ipc_send(void);
  int ipc_recv(void);
  int debug_mem_read(uint32_t, uint32_t&);
  int debug_mem_write(uint32_t, uint32_t, uint32_t* readback=NULL);
  int debug_mem_block_write(uint32_t, uint32_t, const void*);
  int debug_start_app(uint32_t, uint32_t);

  int bin_fw_upload(uint32_t, const uint8_t*, size_t);
  int ext_patch_data_load(const struct aicbt_patch_info_t*);
  int patch_table_load(const struct aicbt_patch_table*);
  int patch_config(void);

  int system_config(void);
  int download_fw(void);

  bool attach(const usb_device_descriptor*, const usb_configuration_descriptor*) override;
  void detach(void);
public:
  fwuploader();
};

int fwuploader::ipc_send(void) {
  txmsg.len = txmsg.lmac.len+12;
  txmsg.cmd = 0x11;
  txmsg.empty = 0;
  txmsg.lmac.dest_id = TASK_DEBUG;
  txmsg.lmac.src_id = tid++;

  int r = BulkMessage(bulk_out, txmsg.len+4, &txmsg);
  if (r < txmsg.len+4) return -1;
  return 0;
}

int fwuploader::ipc_recv(void) {
  int r = BulkMessage(bulk_in, sizeof(rxmsg), &rxmsg);
  if (r < 16) return -1;
  if (r < rxmsg.len+4) return -1;
  if (rxmsg.cmd != USB_TYPE_CFG_CMD_RSP) return -1;
  if (rxmsg.pattern != RX_MSG_VALID_PATTERN) return -1;
  if (rxmsg.lmac.dest_id+1 != tid) return -1;
  if (rxmsg.lmac.src_id != TASK_DEBUG) return -1;

  return rxmsg.len-12;
}

int fwuploader::debug_mem_read(uint32_t addr, uint32_t& data) {
  txmsg.lmac.id = DBG_MEM_READ_REQ;
  txmsg.lmac.len = sizeof(dbg_mem_read_req);
  txmsg.debug.mem_read_req.memaddr = addr;

  if (ipc_send() < 0) return -1;
  if (ipc_recv() < (int)sizeof(dbg_mem_read_cfm)) return -1;

  if (rxmsg.lmac.id != DBG_MEM_READ_CFM) return -1;
  if (rxmsg.lmac.len < sizeof(dbg_mem_read_cfm)) return -1;
  if (rxmsg.debug.mem_read_cfm.memaddr != addr) return -1;

  data = rxmsg.debug.mem_read_cfm.memdata;
  return 0;
}

int fwuploader::debug_mem_write(uint32_t addr, uint32_t data, uint32_t* readback) {
  txmsg.lmac.id = DBG_MEM_WRITE_REQ;
  txmsg.lmac.len = sizeof(dbg_mem_write_req);
  txmsg.debug.mem_write_req.memaddr = addr;
  txmsg.debug.mem_write_req.memdata = data;

  if (ipc_send() < 0) return -1;
  if (ipc_recv() < (int)sizeof(dbg_mem_write_cfm)) return -1;

  if (rxmsg.lmac.id != DBG_MEM_WRITE_CFM) return -1;
  if (rxmsg.lmac.len < sizeof(dbg_mem_write_cfm)) return -1;
  if (rxmsg.debug.mem_write_cfm.memaddr != addr) return -1;
  if (readback) *readback = rxmsg.debug.mem_write_cfm.memdata;

  return 0;
};

int fwuploader::debug_mem_block_write(uint32_t addr, uint32_t len, const void* src) {
  txmsg.lmac.id = DBG_MEM_BLOCK_WRITE_REQ;
  txmsg.lmac.len = len+8;
  txmsg.debug.mem_block_write_req.memaddr = addr;
  txmsg.debug.mem_block_write_req.memsize = len;
  memcpy(txmsg.debug.mem_block_write_req.memdata, src, len);

  if (ipc_send() < 0) return -1;
  if (ipc_recv() < (int)sizeof(dbg_mem_block_write_cfm)) return -1;

  if (rxmsg.lmac.id != DBG_MEM_BLOCK_WRITE_CFM) return -1;
  if (rxmsg.lmac.len < sizeof(dbg_mem_block_write_cfm)) return -1;

  return (rxmsg.debug.mem_block_write_cfm.wstatus == 0) ? 0 : -1;
}

int fwuploader::debug_start_app(uint32_t boot_addr, uint32_t boot_type) {
  txmsg.lmac.id = DBG_START_APP_REQ;
  txmsg.lmac.len = sizeof(dbg_start_app_req);
  txmsg.debug.start_app_req.bootaddr = boot_addr;
  txmsg.debug.start_app_req.boottype = boot_type;

  if (ipc_send() < 0) return -1;
  // don't expect a response to this because device will reset
  return 0;
}

bool fwuploader::attach(const usb_device_descriptor*, const usb_configuration_descriptor*) {
  evt.triggerEvent(0);
  return true;
}

void fwuploader::detach(void) {
  evt.triggerEvent(-1);
}

int fwuploader::system_config(void) {
  const uint32_t mem_addr = 0x40500000;
  const uint32_t cache_mem_addr = 0x40100020;
  uint32_t data;
  uint8_t chip_mcu_id = 0;

  int ret = debug_mem_read(mem_addr, data);
  if (ret < 0) {
    dprintf("%lx rd fail: %d\n", mem_addr, ret);
    return ret;
  }
  if (((data >> 25) & 1) == 0) {
    chip_mcu_id = 1;
  }
  chip_id = data >> 16;
  dprintf("chip_id=%x, chip_mcu_id = %d\n", chip_id, chip_mcu_id);

  if (chip_mcu_id) {
    ret = debug_mem_read(cache_mem_addr, data);
    if (ret < 0) {
      dprintf("%lx rd fail: %d\n", cache_mem_addr, ret);
      return ret;
    }
    ret = debug_mem_write(cache_mem_addr, data|1);
    if (ret < 0) {
      dprintf("%lx write fail: %d\n", cache_mem_addr, ret);
      return ret;
    }
  }

  return 0;
}

int fwuploader::bin_fw_upload(uint32_t fw_addr, const uint8_t* src, size_t size) {
  int err = 0;
  const size_t msg_max = sizeof(dbg_mem_block_write_req::memdata);
  const uint8_t* end = src + size;

  dprintf("### Upload firmware, @ = %lx  size=%u\n", fw_addr, size);

  while (src < end) {
    size_t i = end - src;
    if (i > msg_max) i = msg_max;
    err = debug_mem_block_write(fw_addr, i, src);
    if (err) {
      dprintf("bin upload fail: %lx, err:%d\n", fw_addr, err);
      break;
    }

    fw_addr += i;
    src += i;
  }

  dprintf("fw download complete\n\n");
  return err;
}

int fwuploader::ext_patch_data_load(const struct aicbt_patch_info_t* patch_info) {
  int ret = 0;

  for (uint32_t index = 0; index < patch_info->ext_patch_nb; index++) {
    uint32_t id = patch_info->ext_patch_param[index * 2];
    uint32_t addr = patch_info->ext_patch_param[index * 2 + 1];
    dprintf("%s ext_patch_id:%lx ext_patch_addr:%lx\n", __func__, id, addr);
    ret = bin_fw_upload(addr, fw_patch_u02_ext[id].src, fw_patch_u02_ext[id].size);
    if (ret) break;
  }

  return ret;
}

int fwuploader::patch_table_load(const struct aicbt_patch_table* p) {
  int ret = 0;

  for(; p; p=p->next) {
    const uint32_t *data = p->data;
    if (p->type == AICBT_PT_VERSION) {
      dprintf("patch version %s\n", (const char*)data);
      continue;
    }

    if (p->type == AICBT_PT_BTMODE) {
      dprintf("%s, bt btmode:%lu\n", __func__, data[7]);
      dprintf("%s, bt btport:%lu\n", __func__, data[9]);
      dprintf("%s, bt uart_baud:%lu\n", __func__, data[11]);
      dprintf("%s, bt uart_flowctrl:%lu\n", __func__, data[13]);
      dprintf("%s, bt lpm_enable:%lu\n", __func__, data[15]);
      dprintf("%s, bt tx_pwr:%4lX\n", __func__, data[17]);
    }

    for (uint32_t i=0; i < p->len; i++) {
      ret = debug_mem_write(data[0], data[1]);
      if (ret != 0) {
        dprintf("patch table load failed, patch %lu addr %lx data %lx\n", i, data[0], data[1]);
        dprintf("table tag: %s\n", p->name);
        return ret;
      }
      data += 2;
    }

    if (p->type == AICBT_PT_PWRON)
      atomTimerDelayms(100);
  }

  return ret;
}

#define USER_PWROFFST_COVER_CALIB_FLAG       (1 << 0)

#define AIC_PATCH_OFFSET(mem) ((size_t) &((aic_patch_t*)0)->mem)
#define AIC_PATCH_ADDR(mem) ((uint32_t) (aic_patch_str_base + AIC_PATCH_OFFSET(mem)))

int fwuploader::patch_config(void) {
  const uint32_t patch_magic_num  = 0x48435450; // 'PTCH'
  const uint32_t patch_magic_num2 = 0x50544348; // 'HCTP'

  static const uint32_t patch_tbl[][2] PROGMEM = {
    {0x000000B4, 0xF3010000},
    {0x00000170, 0x0001000A}, // rx aggr counter
    {0x00000188, USER_PWROFFST_COVER_CALIB_FLAG}, // user_ext_flags
  };
  const uint32_t patch_cnt = sizeof(patch_tbl) / sizeof(patch_tbl[0]);

  int ret;
  uint32_t data;

  uint32_t start_addr = 0x001D7000;
  uint32_t patch_addr = start_addr;

  const uint32_t rd_version_addr = 0x1C + (chip_id == CHIP_REV_U01 ? RAM_FMAC_FW_ADDR : RAM_FMAC_FW_ADDR_U02);
  const uint32_t rd_patch_addr = 0x17C + rd_version_addr;
  const uint32_t aic_patch_addr = 0x184 + rd_version_addr;

  dprintf("Read FW mem: %08lx\n", rd_patch_addr);
  if ((ret = debug_mem_read(rd_patch_addr, data))) {
    dprintf("setting base[0x%lx] rd fail: %d\n", rd_patch_addr, ret);
    return ret;
  }
  dprintf("%lx=%lx\n", rd_patch_addr, data);
  uint32_t config_base = data;

  if ((ret = debug_mem_read(aic_patch_addr, data))) {
    dprintf("patch_str_base[0x%lx] rd fail: %d\n", aic_patch_addr, ret);
    return ret;
  }
  dprintf("%lx=%lx\n", aic_patch_addr, data);
  uint32_t aic_patch_str_base = data;

  if ((ret = debug_mem_read(rd_version_addr, data))) {
    dprintf("version val[0x%lx] rd fail:% d\n", rd_version_addr, ret);
    return ret;
  }
  dprintf("rd_version_val=%08lX\n", data);
  if (data > 0x06090100) {
    const uint32_t patch_buff_addr = rd_patch_addr + 12;
    ret = debug_mem_read(patch_buff_addr, data);
    if (ret) {
      dprintf("patch buff rd fail\n");
      return ret;
    }
    dprintf("%lx=%lx\n", patch_buff_addr, data);
    patch_addr = start_addr = data;
  }

  if ((ret = debug_mem_write(AIC_PATCH_ADDR(magic_num), patch_magic_num))) {
    dprintf("magic_num[0x%lx] write fail: %d\n", AIC_PATCH_ADDR(magic_num), ret);
    return ret;
  }

  if ((ret = debug_mem_write(AIC_PATCH_ADDR(magic_num_2), patch_magic_num2))) {
    dprintf("magic_num[0x%lx] write fail: %d\n", AIC_PATCH_ADDR(magic_num_2), ret);
    return ret;
  }

  if ((ret = debug_mem_write(AIC_PATCH_ADDR(pair_start), patch_addr))) {
    dprintf("pair_start[0x%lx] write fail: %d\n", AIC_PATCH_ADDR(pair_start), ret);
    return ret;
  }

  if ((ret = debug_mem_write(AIC_PATCH_ADDR(pair_count), patch_cnt))) {
    dprintf("pair_count[0x%lx] write fail: %d\n", AIC_PATCH_ADDR(pair_count), ret);
    return ret;
  }

  for (uint32_t i=0; i < patch_cnt; i++,start_addr+=8) {
    // send address
    ret = debug_mem_write(start_addr, patch_tbl[i][0]+config_base);
    // send patch
    if (ret == 0) ret = debug_mem_write(start_addr+4, patch_tbl[i][1]);
    if (ret) {
      dprintf("write_fail patch_tbl %lu\n", i);
      return ret;
    }
  }

  for (int i=0; i < AIC_PATCH_BLOCK_MAX; i++) {
    if ((ret = debug_mem_write(AIC_PATCH_ADDR(block_size[i]), 0))) {
      dprintf("block_size[0x%lx] write fail: %d\n", AIC_PATCH_ADDR(block_size[i]), ret);
      return ret;
    }
  }

  return ret;
}

int fwuploader::download_fw(void) {
  std::unique_ptr<struct aicbt_patch_table> head;
  struct aicbt_patch_info_t patch_info = {0};

  if (system_config() < 0)
    return -1;

  // firmware file for U01 is missing
  if (chip_id == CHIP_REV_U01) {
    head = NULL; //aicbt_patch_table_alloc(fw_patch_table);
  } else {
    head.reset(aicbt_patch_table_alloc(fw_patch_table_u02, sizeof(fw_patch_table_u02)));
  }
  if (head == NULL) {
    dprintf("aicbt_patch_table_alloc fail\n");
    return -1;
  }

  if (chip_id == CHIP_REV_U01) {
    patch_info.addr_adid = FW_RAM_ADID_BASE_ADDR;
    patch_info.addr_patch = FW_RAM_PATCH_BASE_ADDR;
  } else if (chip_id == CHIP_REV_U02 || chip_id == CHIP_REV_U03) {
    patch_info.addr_adid = FW_RAM_ADID_BASE_ADDR_U02;
    patch_info.addr_patch = FW_RAM_PATCH_BASE_ADDR_U02;
  }
  aicbt_patch_info_unpack(&patch_info, head.get());
  if (patch_info.info_len == 0) {
    dprintf("%s, aicbt_pach_info_unpack fail\n", __func__);
    return -1;
  }

  dprintf("addr_adid 0x%lx, addr_patch 0x%lx\n", patch_info.addr_adid, patch_info.addr_patch);

  if (chip_id != CHIP_REV_U01) {
    if (bin_fw_upload(patch_info.addr_adid, fw_adid_u02, sizeof(fw_adid_u02))) {
      return -1;
    }
    if (bin_fw_upload(patch_info.addr_patch, fw_patch_u02, sizeof(fw_patch_u02))) {
      return -1;
    }
    if (ext_patch_data_load(&patch_info)) {
      return -1;
    }

    if (patch_table_load(head.get())) {
      return -1;
    }

    if (IS_CHIP_ID_H()) {
      if (bin_fw_upload(RAM_FMAC_FW_ADDR_U02, fmacfw_h_u02, sizeof(fmacfw_h_u02))) {
        return -1;
      }
    } else {
      if (bin_fw_upload(RAM_FMAC_FW_ADDR_U02, fmacfw_u02, sizeof(fmacfw_u02))) {
        return -1;
      }
    }

    if (patch_config()) {
      return -1;
    }

    if (debug_start_app(RAM_FMAC_FW_ADDR_U02, HOST_START_APP_AUTO)) {
      return -1;
    }
  } else {
  }

  return 0;
}

void fwuploader::handle_event(EventResponderRef ev) {
  auto p = (fwuploader*)ev.getContext();

  if (ev.getStatus() == -1) { // detach
    delete p;
    return;
  }
  // else attempt to upload the firmware

  if (p->download_fw() >= 0) {
    dprintf("Firmware uploaded successfully\n");
    return;
  }

  dprintf("Firmware upload failed\n");
}

fwuploader::fwuploader() {
  evt.setContext(this);
  evt.attach(handle_event);
}

}; // AIC8800

USB_Driver* AIC8800D80::create_fwuploader() {
  return new AIC8800::fwuploader;
}

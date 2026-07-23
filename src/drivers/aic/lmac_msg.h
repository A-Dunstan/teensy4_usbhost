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

#include <stdint.h>

enum {
  TASK_NONE = (uint8_t)-1,

  TASK_MM = 0, // MAC management
  TASK_DEBUG,
  TASK_SCAN,
  TASK_TDLS,
  TASK_SCANU,
  TASK_ME,
  TASK_SM,
  TASK_APM,
  TASK_BAM,
  TASK_MESH,
  TASK_RXU,
  TASK_RM
};

// Message Identifier. Lowest 10 bits are message index, remainder are task index.
typedef uint16_t lmac_msg_id_t;
typedef uint16_t lmac_task_id_t;

#define LMAC_FIRST_MSG(task) ((lmac_msg_id_t)((task) << 10))

// get the task from an id
#define MSG_T(msg) ((lmac_task_id_t)((msg) >> 10))
// get the index from an id
#define MSG_I(msg) ((msg) & ((1<<10)-1))

struct lmac_msg {
  lmac_msg_id_t     id;        // Message id
  lmac_task_id_t    dest_id;   // Destination kernel id
  lmac_task_id_t    src_id;    // Source kernel id
  uint16_t          len;       // length of payload
};

// Messages related to Debug Task
enum {
  // Memory read request
  DBG_MEM_READ_REQ = LMAC_FIRST_MSG(TASK_DEBUG),
  // Memory read confirma
  DBG_MEM_READ_CFM,
  // Memory write request
  DBG_MEM_WRITE_REQ,
  // Memory write confirm
  DBG_MEM_WRITE_CFM,
  // Module filter request
  DBG_SET_MOD_FILTER_REQ,
  // Module filter confirm
  DBG_SET_MOD_FILTER_CFM,
  // Severity filter request
  DBG_SET_SEV_FILTER_REQ,
  // Severity filter confirmation
  DBG_SET_SEV_FILTER_CFM,
  // LMAC/MAC HW fata error indication
  DBG_ERROR_IND,
  // Request to get system statistics
  DBG_GET_SYS_STAT_REQ,
  // Confirmation of system statistics
  DBG_GET_SYS_STAT_CFM,
  // Memory block write request
  DBG_MEM_BLOCK_WRITE_REQ,
  // Memory block write confirm
  DBG_MEM_BLOCK_WRITE_CFM,
  // Start app request
  DBG_START_APP_REQ,
  // Start app confirm
  DBG_START_APP_CFM,
  // Start npc request
  DBG_START_NPC_REQ,
  // Start npc confirm
  DBG_START_NPC_CFM,
  // Memory mask write request
  DBG_MEM_MASK_WRITE_REQ,
  // Memory mask write confirm
  DBG_MEM_MASK_WRITE_CFM,

  DBG_RFTEST_CMD_REQ,
  DBG_RFTEST_CMD_CFM,
  DBG_BINDING_REQ,
  DBG_BINDING_IND,

  DBG_CUSTOM_MSG_REQ,
  DBG_CUSTOM_MSG_CFM,
  DBG_CUSTOM_MSG_IND,

  DBG_GPIO_WRITE_REQ,
  DBG_GPIO_WRITE_CFM,
  DBG_GPIO_READ_REQ,
  DBG_GPIO_READ_CFM,
  DBG_GPIO_INIT_REQ,
  DBG_GPIO_INIT_CFM,

  // EF usrdata read request
  DBG_EF_USRDATA_READ_REQ,
  // EF usrdata read confirm
  DBG_EF_USRDATA_READ_CFM,
  // Memory block read request
  DBG_MEM_BLOCK_READ_REQ,
  // Memory block read confirm
  DBG_MEM_BLOCK_READ_CFM,

  DBG_PWM_INIT_REQ,
  DBG_PWM_INIT_CFM,
  DBG_PWM_DEINIT_REQ,
  DBG_PWM_DEINIT_CFM,

  // Max number of Debug messages
  DBG_MAX
};

// Structure containing the parameters of the @ref DBG_MEM_READ_REQ message.
struct dbg_mem_read_req
{
  uint32_t memaddr;
};

// Structure containing the parameters of the @ref DBG_MEM_READ_CFM message.
struct dbg_mem_read_cfm
{
  uint32_t memaddr;
  uint32_t memdata;
};

// Structure containing the parameters of the @ref DBG_MEM_WRITE_REQ message.
struct dbg_mem_write_req
{
  uint32_t memaddr;
  uint32_t memdata;
};

// Structure containing the parameters of the @ref DBG_MEM_WRITE_CFM message.
struct dbg_mem_write_cfm
{
  uint32_t memaddr;
  uint32_t memdata;
};

// Structure containing the parameters of the @ref DBG_MEM_MASK_WRITE_REQ message.
struct dbg_mem_mask_write_req
{
  uint32_t memaddr;
  uint32_t memmask;
  uint32_t memdata;
};

// Structure containing the parameters of the @ref DBG_MEM_MASK_WRITE_CFM message.
struct dbg_mem_mask_write_cfm
{
  uint32_t memaddr;
  uint32_t memdata;
};

struct dbg_rftest_cmd_req
{
  uint32_t cmd;
  uint32_t argc;
  uint8_t argv[30];
};

struct dbg_rftest_cmd_cfm
{
  uint32_t rftest_result[32];
};

struct dbg_gpio_write_req {
  uint8_t gpio_idx;
  uint8_t gpio_val;
};

struct dbg_gpio_read_req {
  uint8_t gpio_idx;
};

struct dbg_gpio_read_cfm {
  uint8_t gpio_idx;
  uint8_t gpio_val;
};

struct dbg_gpio_init_req {
  uint8_t gpio_idx;
  uint8_t gpio_dir; //1 output, 0 input;
  uint8_t gpio_val; //for output, 1 high, 0 low;
};

// Structure containing the parameters of the DBG_SET_MOD_FILTER_REQ message.
struct dbg_set_mod_filter_req
{
  // Bit field indicating for each module if the traces are enabled or not
  uint32_t mod_filter;
};

// Structure containing the parameters of the DBG_SEV_MOD_FILTER_REQ message.
struct dbg_set_sev_filter_req
{
  // Bit field indicating the severity threshold for the traces
  uint32_t sev_filter;
};

// Structure containing the parameters of the DBG_GET_SYS_STAT_CFM message.
struct dbg_get_sys_stat_cfm
{
  // Time spent in CPU sleep since last reset of the system statistics
  uint32_t cpu_sleep_time;
  // Time spent in DOZE since last reset of the system statistics
  uint32_t doze_time;
  // Total time spent since last reset of the system statistics
  uint32_t stats_time;
};

// Structure containing the parameters of the DBG_MEM_BLOCK_WRITE_REQ message.
struct dbg_mem_block_write_req
{
  uint32_t memaddr;
  uint32_t memsize;
  uint8_t memdata[1024];
};

// Structure containing the parameters of the DBG_MEM_BLOCK_WRITE_CFM message.
struct dbg_mem_block_write_cfm
{
  uint32_t wstatus;
};

// Structure containing the parameters of the DBG_MEM_BLOCK_READ_REQ message.
struct dbg_mem_block_read_req
{
  uint32_t memaddr;
  uint32_t memsize;
};

// Structure containing the parameters of the DBG_MEM_BLOCK_READ_CFM message.
struct dbg_mem_block_read_cfm
{
  uint32_t memaddr;
  uint32_t memsize;
  uint8_t memdata[1024];
};

// Structure containing the parameters of the DBG_START_APP_REQ message.
struct dbg_start_app_req
{
  uint32_t bootaddr;
  uint32_t boottype;
};

// Structure containing the parameters of the DBG_START_APP_CFM message.
struct dbg_start_app_cfm
{
  uint32_t bootstatus;
};

enum {
  HOST_START_APP_AUTO = 1,
  HOST_START_APP_CUSTOM,
  HOST_START_APP_REBOOT,
  HOST_START_APP_FNCALL = 4,
  HOST_START_APP_DUMMY  = 5,
};

union dbg_msg {
  struct dbg_mem_read_req            mem_read_req;
  struct dbg_mem_read_cfm            mem_read_cfm;
  struct dbg_mem_write_req           mem_write_req;
  struct dbg_mem_write_cfm           mem_write_cfm;
  struct dbg_mem_mask_write_req      mem_mask_write_req;
  struct dbg_mem_mask_write_cfm      mem_mask_write_cfm;
  struct dbg_rftest_cmd_req          rftest_cmd_req;
  struct dbg_rftest_cmd_cfm          rftest_cmd_cfm;
  struct dbg_gpio_write_req          gpio_write_req;
  struct dbg_gpio_read_req           gpio_read_req;
  struct dbg_gpio_read_cfm           gpio_read_cfm;
  struct dbg_gpio_init_req           gpio_init_req;
  struct dbg_set_mod_filter_req      set_mod_filter_req;
  struct dbg_set_sev_filter_req      set_sev_filter_req;
  struct dbg_get_sys_stat_cfm        get_sys_stat_cfm;
  struct dbg_mem_block_write_req     mem_block_write_req;
  struct dbg_mem_block_write_cfm     mem_block_write_cfm;
  struct dbg_mem_block_read_req      mem_block_req_req;
  struct dbg_mem_block_read_cfm      mem_block_read_cfm;
  struct dbg_start_app_req           start_app_req;
  struct dbg_start_app_cfm           start_app_cfm;
};

struct tx_msg {
  uint16_t len;
  uint16_t cmd;
  uint32_t empty;
  struct lmac_msg lmac;
  union {
    dbg_msg debug;
  };
};

struct rx_msg {
  uint16_t len;
  uint16_t cmd;
  struct lmac_msg lmac;
  uint32_t pattern;
  union {
    dbg_msg debug;
  };
};

enum {
  USB_TYPE_DATA        = 0,
  USB_TYPE_CFG         = 0x10,
  USB_TYPE_CFG_CMD_RSP,
  USB_TYPE_CFG_DATA_CFM,
};

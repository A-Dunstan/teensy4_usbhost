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

#include "../teensy4_usbhost.h"

enum {
  PHY_REG_BMCR = 0,
  PHY_REG_BMSR,
  PHY_REG_PHYIDR1,
  PHY_REG_PHYIDR2,
  PHY_REG_ANAR,
  PHY_REG_ANLPAR,
  PHY_REG_ANER,
};

#define BMCR_RESET             (1<<15)  // self-clearing
#define BMCR_LOOPBACK          (1<<14)
#define BMCR_SPEED_SELECTION   (1<<13)
#define BMCR_AUTO_NEGOTIATE    (1<<12)
#define BMCR_POWER_DOWN        (1<<11)
#define BMCR_ISOLATE           (1<<10)
#define BMCR_RESTART_AUTO_NEG  (1<<9)   // self-clearing
#define BMCR_DUPLEX_MODE       (1<<8)
#define BMCR_COLLISION_TEST    (1<<7)

#define BMSR_100BASE_T4        (1<<15)
#define BMSR_100BASE_TX_FULL   (1<<14)
#define BMSR_100BASE_TX_HALF   (1<<13)
#define BMSR_10BASE_T_FULL     (1<<12)
#define BMSR_10BASE_T_HALF     (1<<11)
#define BMSR_MF_PRE_SUPPRESS   (1<<6)
#define BMSR_AUTO_NEG_COMPLETE (1<<5)
#define BMSR_REMOTE_FAULT      (1<<4)
#define BMSR_AUTO_NEG_ABILITY  (1<<3)
#define BMSR_LINK_STATUS       (1<<2)
#define BMSR_JABBER_DETECT     (1<<1)
#define BMSR_EXTENDED_CAP      (1<<0)

// same definitions are used for ANLPAR
#define ANAR_NP                (1<<15)
#define ANAR_ACK               (1<<14)
#define ANAR_RF                (1<<13)
#define ANAR_PAUSE             (1<<10)
#define ANAR_T4                (1<<9)
#define ANAR_TX_FD             (1<<8)
#define ANAR_TX_HD             (1<<7)
#define ANAR_10_FD             (1<<6)
#define ANAR_10_HD             (1<<5)
#define ANAR_SELECTOR_MASK     0x1F

#define MEDIUM_MODE_SM         (1<<12)
#define MEDIUM_MODE_SBP        (1<<11)
#define MEDIUM_MODE_PS         (1<<9)
#define MEDIUM_MODE_RE         (1<<8)
#define MEDIUM_MODE_PF         (1<<7)
#define MEDIUM_MODE_TFC        (1<<5)
#define MEDIUM_MODE_RFC        (1<<4)
#define MEDIUM_MODE_FD         (1<<1)

enum {
  OP_INIT =                    1<<0,
  OP_SET_MAC =                 1<<1,
  OP_SET_MULTICAST =           1<<2,
  OP_UPDATE_MEDIUM =           1<<3,
  OP_UPDATE_BMCR =             1<<4,
  OP_UPDATE_BMSR =             1<<5,
  OP_UPDATE_ANAR =             1<<6,
  OP_CLEAR_FLE =               1<<7,

  OP_ALL =                     0xFFFFFFFF
};

void asix88772_eth::interrupt(int result) {
  if (result < 0) {
    //dprintf("eth status returned %d\n", result);
    return;
  }

  if (result > 2) {
    uint8_t flagsdiff = status[2] ^ last_int;
    if (flagsdiff & 1) {
      // link state changed, set BMSR update after 100ms
      Timer(100, &bmsr_cb);
    }
    if (flagsdiff & status[2] & 4) {
      // bad data sent to bulk out endpoint (frame length error)
      pending_ops |= OP_CLEAR_FLE;
    }
    last_int = status[2];
#if 0
    dprintf("eth interrupt: %d bytes ", result);
    for (int i=0; i < result; i++) {
      dprintf("%02X ", status[i]);
    }
    dprintf("\n");
#endif
  }

  InterruptMessage(ep_status, sizeof(status), status, &status_cb);
}

typedef std::tuple<uint16_t,uint16_t,uint16_t,void*> ReadControlArgs;
typedef std::tuple<uint16_t,uint16_t,uint16_t,const void*> WriteControlArgs;
template<uint8_t> struct make_cmd_args;

#define CMD_READ_SRAM                2
#define CMD_WRITE_SRAM               3
#define CMD_SOFTWARE_SERIAL_CONTROL  6
#define CMD_READ_PHY                 7
#define CMD_WRITE_PHY                8
#define CMD_READ_SERIAL_STATUS       9
#define CMD_HARDWARE_SERIAL_CONTROL  10
#define CMD_READ_SROM                11
#define CMD_WRITE_SROM               12
#define CMD_SROM_WRITE_ENABLE        13
#define CMD_SROM_WRITE_DISABLE       14
#define CMD_READ_RX_CONTROL          15
#define CMD_WRITE_RX_CONTROL         16
#define CMD_READ_IPG                 17
#define CMD_WRITE_IPG                18
#define CMD_READ_NODE_ID             19
#define CMD_WRITE_NODE_ID            20
#define CMD_READ_MULTICAST_FILTER    21
#define CMD_WRITE_MULTICAST_FILTER   22
#define CMD_WRITE_TEST               23
#define CMD_READ_PHY_ADDRESS         25
#define CMD_READ_MEDIUM              26
#define CMD_WRITE_MEDIUM             27
#define CMD_READ_MONITOR             28
#define CMD_WRITE_MONITOR            29
#define CMD_READ_GPIO                30
#define CMD_WRITE_GPIO               31
#define CMD_WRITE_SOFTWARE_RESET     32
#define CMD_READ_PHY_SELECT          33
#define CMD_WRITE_PHY_SELECT         34

// tx_sram==1 means access tx_sram, else access rx_sram
template<> struct make_cmd_args<CMD_READ_SRAM> {
  static auto pack(uint16_t address, bool tx_sram, void* dst) { return ReadControlArgs(address, tx_sram, 8, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_SRAM> {
  static auto pack(uint16_t address, bool tx_sram, const void* src) { return WriteControlArgs(address, tx_sram, 8, src); }
};
template<> struct make_cmd_args<CMD_SOFTWARE_SERIAL_CONTROL> {
  static auto pack(void) { return WriteControlArgs(0, 0, 0, NULL); }
};
template<> struct make_cmd_args<CMD_READ_PHY> {
  static auto pack(uint8_t phy_id, uint8_t reg_addr, uint16_t* dst) { return ReadControlArgs(phy_id, reg_addr, 2, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_PHY> {
  static auto pack(uint8_t phy_id, uint8_t reg_addr, const uint16_t* src) { return WriteControlArgs(phy_id, reg_addr, 2, src); }
};
template<> struct make_cmd_args<CMD_READ_SERIAL_STATUS> {
  static auto pack(uint8_t* dst) { return ReadControlArgs(0, 0, 1, dst); }
};
template<> struct make_cmd_args<CMD_HARDWARE_SERIAL_CONTROL> {
  static auto pack(void) { return WriteControlArgs(0, 0, 0, NULL); }
};
template<> struct make_cmd_args<CMD_READ_SROM> {
  static auto pack(uint8_t address, uint16_t* dst) { return ReadControlArgs(address, 0, 2, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_SROM> {
  static auto pack(uint8_t address, uint16_t val) { return WriteControlArgs(address, val, 0, NULL); }
};
template<> struct make_cmd_args<CMD_SROM_WRITE_ENABLE> {
  static auto pack(void) { return WriteControlArgs(0, 0, 0, NULL); }
};
template<> struct make_cmd_args<CMD_SROM_WRITE_DISABLE> {
  static auto pack(void) { return WriteControlArgs(0, 0, 0, NULL); }
};
template<> struct make_cmd_args<CMD_READ_RX_CONTROL> {
  static auto pack(uint16_t* dst) { return ReadControlArgs(0, 0, 2, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_RX_CONTROL> {
  static auto pack(uint16_t val) { return WriteControlArgs(val, 0, 0, NULL); }
};
template<> struct make_cmd_args<CMD_READ_IPG> {
  static auto pack(uint8_t* dst) { return ReadControlArgs(0, 0, 3, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_IPG> {
  static auto pack(uint8_t val1, uint8_t val2, uint8_t val3) { return WriteControlArgs((val2<<8)|val1, val3, 0, NULL); }
};
template<> struct make_cmd_args<CMD_READ_NODE_ID> {
  static auto pack(uint8_t* dst) { return ReadControlArgs(0, 0, 6, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_NODE_ID> {
  static auto pack(const uint8_t* src) { return WriteControlArgs(0, 0, 6, src); }
};
template<> struct make_cmd_args<CMD_READ_MULTICAST_FILTER> {
  static auto pack(uint8_t* dst) { return ReadControlArgs(0, 0, 8, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_MULTICAST_FILTER> {
  static auto pack(const uint8_t* src) { return WriteControlArgs(0, 0, 8, src); }
};
template<> struct make_cmd_args<CMD_WRITE_TEST> {
  static auto pack(uint16_t val) { return WriteControlArgs(val, 0, 0, NULL); }
};
template<> struct make_cmd_args<CMD_READ_PHY_ADDRESS> {
  static auto pack(uint16_t* dst) { return ReadControlArgs(0, 0, 2, dst); }
};
template<> struct make_cmd_args<CMD_READ_MEDIUM> {
  static auto pack(uint16_t* dst) { return ReadControlArgs(0, 0, 2, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_MEDIUM> {
  static auto pack(uint16_t val) { return WriteControlArgs(val, 0, 0, NULL); }
};
template<> struct make_cmd_args<CMD_READ_MONITOR> {
  static auto pack(uint8_t* dst) { return ReadControlArgs(0, 0, 1, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_MONITOR> {
  static auto pack(uint8_t val) { return WriteControlArgs(val, 0, 0, NULL); }
};
template<> struct make_cmd_args<CMD_READ_GPIO> {
  static auto pack(uint8_t* dst) { return ReadControlArgs(0, 0, 1, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_GPIO> {
  static auto pack(uint8_t val) { return WriteControlArgs(val, 0, 0, NULL); }
};
template<> struct make_cmd_args<CMD_WRITE_SOFTWARE_RESET> {
  static auto pack(uint8_t val) { return WriteControlArgs(val, 0, 0, NULL); }
};
template<> struct make_cmd_args<CMD_READ_PHY_SELECT> {
  static auto pack(uint8_t* dst) { return ReadControlArgs(0, 0, 1, dst); }
};
template<> struct make_cmd_args<CMD_WRITE_PHY_SELECT> {
  static auto pack(uint8_t val) { return WriteControlArgs(val, 0, 0, NULL); }
};

// vendor command INPUT
bool asix88772_eth::vendor_command(uint8_t bmr, uint16_t wValue, uint16_t wIndex ,uint16_t wLength, void* data) {
  uint8_t buf[32] __attribute__((aligned(32)));

  int ret = ControlMessage(USB_CTRLTYPE_DIR_DEVICE2HOST|USB_CTRLTYPE_TYPE_VENDOR|USB_CTRLTYPE_REC_DEVICE, bmr, wValue, wIndex, wLength, buf);
  if (ret > 0) memcpy(data, buf, ret);

  return ret >= wLength;
}
// vendor command OUTPUT
bool asix88772_eth::vendor_command(uint8_t bmr, uint16_t wValue, uint16_t wIndex, uint16_t wLength, const void* data) {
  return ControlMessage(USB_CTRLTYPE_DIR_HOST2DEVICE|USB_CTRLTYPE_TYPE_VENDOR|USB_CTRLTYPE_REC_DEVICE, bmr, wValue, wIndex, wLength, data) >= wLength;
}

template<uint8_t cmd, typename...Args>
bool asix88772_eth::vendor_command(Args...r) {
  auto fn = [&](auto...cmargs) { return vendor_command(cmd, cmargs...); };
  auto params = make_cmd_args<cmd>::pack(r...);

  return std::apply(fn, params);
}

bool asix88772_eth::write_PHY(uint8_t phy_reg, const uint16_t val, bool internal) {
  bool ret = false;
  uint8_t phy_address = internal ? PHY_id.internal : PHY_id.external;

  if (vendor_command<CMD_SOFTWARE_SERIAL_CONTROL>()) {
    ret = vendor_command<CMD_WRITE_PHY>(phy_address, phy_reg, &val);
    vendor_command<CMD_HARDWARE_SERIAL_CONTROL>();
  }

  return ret;
}

bool asix88772_eth::read_PHY(uint8_t phy_reg, uint16_t& val, bool internal) {
  bool ret = false;
  uint8_t phy_address = internal ? PHY_id.internal : PHY_id.external;

  if (vendor_command<CMD_SOFTWARE_SERIAL_CONTROL>()) {
    ret = vendor_command<CMD_READ_PHY>(phy_address, phy_reg, &val);
    vendor_command<CMD_HARDWARE_SERIAL_CONTROL>();
  }

  return ret;
}

bool asix88772_eth::update_medium_mode() {
  uint16_t m = MEDIUM_MODE_SM|MEDIUM_MODE_RE|(1<<2);
  // support pause
  if (anar & anlpar & ANAR_PAUSE) m |= MEDIUM_MODE_TFC|MEDIUM_MODE_RFC;
  // set 100mbps?
  if (get100mbps()) m |= MEDIUM_MODE_PS;
  // set full duplex?
  if (getFullDuplex()) m |= MEDIUM_MODE_FD;

  if (vendor_command<CMD_WRITE_MEDIUM>(m)) {
    pending_ops &= ~OP_UPDATE_MEDIUM;
    return true;
  }

  return false;
}

bool asix88772_eth::update_anar() {
  uint16_t old_anar;
  // update anar settings from bmcr
  anar |= ANAR_TX_FD|ANAR_TX_HD|ANAR_10_FD|ANAR_10_HD;
  if ((bmcr & BMCR_SPEED_SELECTION)==0) // disable 100TX?
    anar &= ~(ANAR_TX_FD|ANAR_TX_HD);
  if ((bmcr & BMCR_DUPLEX_MODE)==0)      // disable full duplex?
    anar &= ~(ANAR_TX_FD|ANAR_10_FD);

  bool ret = read_PHY(PHY_REG_ANAR, old_anar);
  if (ret) {
    uint16_t new_anar = (anar & 0x05FF) | (old_anar & ~0x05FF);
    ret = write_PHY(PHY_REG_ANAR, new_anar);
    if (ret) {
      pending_ops &= ~OP_UPDATE_ANAR;
      // if new settings were written restart auto-negotiation
      if ((anar ^ old_anar) & 0x05FF) {
        bmcr |= BMCR_RESTART_AUTO_NEG;
        update_bmcr();
      }
    }

    anar = new_anar;
  }

  return ret;
}

bool asix88772_eth::update_bmcr() {
  bool ret = write_PHY(PHY_REG_BMCR, bmcr);
  if (ret) {
    pending_ops &= ~OP_UPDATE_BMCR;
    if (bmcr & BMCR_RESET) {
      // will need to update BMCR again - it ignores all other bits when RESET is set
      bmcr &= ~BMCR_RESET;
      pending_ops |= OP_UPDATE_BMCR|OP_UPDATE_BMSR|OP_UPDATE_ANAR;
    }
    else {// else remove other self-clearing bits and update medium
      bmcr &= ~BMCR_RESTART_AUTO_NEG;
      update_bmsr();
    }
  }
  return ret;
}

bool asix88772_eth::update_bmsr() {
  auto old_bmsr = bmsr;
  bool ret = read_PHY(PHY_REG_BMSR, bmsr);
  if (ret) {
    pending_ops &= ~OP_UPDATE_BMSR;
    if (bmsr & BMSR_AUTO_NEG_COMPLETE) {
      read_PHY(PHY_REG_ANLPAR, anlpar);
    } else {
      anlpar = 0;
      // auto-negotiation may still be in progress
      if ((bmcr & BMCR_AUTO_NEGOTIATE) && (old_bmsr & BMSR_LINK_STATUS)==0 && (bmsr & BMSR_LINK_STATUS)) {
        // yes, read BMSR again after 200ms
        Timer(200, &bmsr_cb);
      }
    }
    update_medium_mode();
  }
  return ret;
}

bool asix88772_eth::set_node_ID() {
  if (!vendor_command<CMD_WRITE_NODE_ID>(asix_mac.addr))
    return false;

  pending_ops &= ~OP_SET_MAC;
  return true;
}

FLASHMEM bool asix88772_eth::init() {
  if (getDevice() == NULL)
    return false;

  // get chip type
  if (!vendor_command<CMD_READ_SERIAL_STATUS>(&chip_type))
    return false;
  chip_type = (chip_type >> 4) & 7;
  if (chip_type != 0) { // only accept AX88772 base model
    // 1 = AX88772A
    // 2 = AX88772B
    dprintf("AX88772: Bad chip_type(%u), not AX88772 chipset\n", chip_type);
    return false;
  }

  // setup GPIOs (GPIO0+1 in, GPIO2 out+on seems to be the default...)
  if (!vendor_command<CMD_WRITE_GPIO>(0xB0))
    return false;

  // select internal PHY
  if (!vendor_command<CMD_WRITE_PHY_SELECT>(1))
    return false;

  // set power down internal PHY
  if (!vendor_command<CMD_WRITE_SOFTWARE_RESET>(1<<6))
    return false;
  delay(20);
  // clear power down, leave in reset
  if (!vendor_command<CMD_WRITE_SOFTWARE_RESET>(0))
    return false;
  delay(70);
  // clear reset of internal PHY / hold external PHY in reset
  if (!vendor_command<CMD_WRITE_SOFTWARE_RESET>((1<<5)|(1<<3)))
    return false;
   delay(150);

  // write IPG/IPG1/IPG2
  if (!vendor_command<CMD_WRITE_IPG>(0x15,0x0C,0x12))
    return false;

  // set Node ID
  if (!set_node_ID())
    return false;

  // read primary/secondary PHY ids
  if (!vendor_command<CMD_READ_PHY_ADDRESS>(&PHY_id.val))
    return false;
  //dprintf("External PHY id %02X, Internal PHY id %02X\n", PHY_id.external, PHY_id.internal);

  // reset PHY
  if (!write_PHY(PHY_REG_BMCR, BMCR_RESET))
    return false;

  // update speed/duplex/auto-negotiation
  if (!update_bmcr())
    return false;

  // update auto-negotiation advertisement
  if (!update_anar())
    return false;

  // read PHY BMSR for status
  if (!update_bmsr())
    return false;

  // set medium mode (interrupt polling won't work if RX path is not enabled)
  if (!update_medium_mode())
    return false;

  // disable Wake-on-LAN monitor
  if (!vendor_command<CMD_WRITE_MONITOR>(0))
    return false;

  // update MAC filter
  if (!update_mac_filter())
    return false;

  interrupt(0);
  pending_ops &= ~OP_INIT;
  return true;
}

bool asix88772_eth::update_mac_filter() {
  uint16_t rx = 0x388; // default rx control: 16KB frame burst + start operation + receive broadcast frames
  uint8_t filter[8] = {0};

  auto calc_mask = [](mac_addr& mac)-> uint32_t {
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i : mac.addr) {
      crc ^= __builtin_arm_rbit(i);
      // compiler will completely unroll this loop if optimizations are on
      for (int j=0; j<8; j++) {
        crc = (crc<<1) ^ (crc & 0x80000000 ? 0x04C11DB7 : 0);
      }
    }

    return crc >> 26;
  };

  for (auto& mac : mac_filter) {
    if (mac == asix_mac) continue; // ignore our own mac
    auto ibit = calc_mask(mac);
    //dprintf("MULTICAST HASH %02X:%02X:%02X:%02X:%02X:%02X %02lX\n", mac.addr[0], mac.addr[1], mac.addr[2], mac.addr[3], mac.addr[4], mac.addr[5], ibit);
    filter[ibit>>3] |= (1 << (ibit & 7));
    if (mac.isMulticast()) rx |= 1<<4; // receive any multicast frames that match filter
    else rx |= 1<<5;                   // receive any unicast frames that match filter
  }

  if (!vendor_command<CMD_WRITE_MULTICAST_FILTER>(filter)) {
    // setting multicast filter failed for some reason so just accept everything?
    if (rx & (3<<4)) {
      //dprintf("Failed to set multicast filter, activating promiscuous mode\n");
      rx = (rx & ~(3<<4))|1;
    }
  }
  if (!vendor_command<CMD_WRITE_RX_CONTROL>(rx))
    return false;

  pending_ops &= ~OP_SET_MULTICAST;
  return true;
}

FLASHMEM USB_Driver* asix88772_eth::offer(const usb_device_descriptor* d, const usb_configuration_descriptor*, const USB_Device*) {
  if (getDevice() == NULL) {
    if (d->idVendor == 0x0B95 && d->idProduct == 0x7720)
      return this;
  }

  return NULL;
}

FLASHMEM bool asix88772_eth::attach(const usb_device_descriptor*, const usb_configuration_descriptor* cd) {
  ep_status = ep_out = ep_in = 0;

  const uint8_t* end = &cd->bLength + cd->wTotalLength;
  const usb_descriptor* desc = cd;
  // find the interface descriptor
  do {
    desc = desc->next();
    if (&desc->bLength >= end) return false;
  } while (desc->bDescriptorType != usb_interface_descriptor::DescriptorType);

  if (desc) {
    auto id = static_cast<const usb_interface_descriptor*>(desc);
    // parse endpoints to get status (INT IN), in (BULK IN) and out (BULK OUT)
    for (uint8_t i=0; i < id->bNumEndpoints; i++) {
      auto ep = get_interface_endpoint(id, i);
      if (ep == NULL) return false;
      switch (ep->bmAttributes) {
        case USB_ENDPOINT_BULK:
          if (ep->bEndpointAddress&0x80) {
            if (ep_in==0) ep_in = ep->bEndpointAddress;
          } else if (ep_out==0)
            ep_out = ep->bEndpointAddress;
          break;
        case USB_ENDPOINT_INTERRUPT:
          if (ep->bEndpointAddress&0x80 && ep->wMaxPacketSize==8 && ep_status==0)
            ep_status = ep->bEndpointAddress;
          break;
      }

      if (ep_status && ep_in && ep_out) {
        last_int = 0;
        pending_ops |= OP_ALL;
        evt.triggerEvent();
        return true;
      }
    }
  }

  return false;
}

FLASHMEM void asix88772_eth::detach(void) {
  pending_ops = OP_INIT;
}

FLASHMEM asix88772_eth::asix88772_eth(bool autoNegotiate, bool speed, bool duplex) :
bmsr_cb([=]() { pending_ops |= OP_UPDATE_BMSR; }) {
  auto mac1 = HW_OCOTP_MAC1;
  auto mac0 = HW_OCOTP_MAC0;
  asix_mac.addr[0] = mac1 >> 8;
  asix_mac.addr[1] = mac1 >> 0;
  asix_mac.addr[2] = mac0 >> 24;
  asix_mac.addr[3] = mac0 >> 16;
  asix_mac.addr[4] = mac0 >> 8;
  asix_mac.addr[5] = mac0 >> 0;

  bmcr = 0;
  bmsr = 0;
  anar = ANAR_PAUSE|(1 & ANAR_SELECTOR_MASK);
  pending_ops = OP_ALL;

  set100mbps(speed);
  setFullDuplex(duplex);
  setAutoNegotiation(autoNegotiate);

  evt.setContext(this);
  evt.attach(Event);
}

bool asix88772_eth::get_mac(uint8_t* mac) {
  memcpy(mac, asix_mac.addr, 6);
  return true;
}

bool asix88772_eth::set_mac(const mac_addr mac) {
  asix_mac = mac;
  pending_ops |= OP_SET_MAC;

  return true;
}

bool asix88772_eth::filter_address(const mac_addr mac, const bool allow) {
  bool removed = false;

  for (auto addr = mac_filter.begin(); addr != mac_filter.end(); addr++) {
    if (*addr == mac) {
      if (allow) {
        // mac is already in the list
        return true;
      }
      mac_filter.erase(addr);
      removed = true;
      break;
    }
  }

  if (allow) mac_filter.push_back(mac);
  if (allow || removed) pending_ops |= OP_SET_MULTICAST;

  return true;
}

bool asix88772_eth::clear_FLE() {
  if (!vendor_command<CMD_WRITE_SOFTWARE_RESET>(0x2B))
    return false;
  if (!vendor_command<CMD_WRITE_SOFTWARE_RESET>(0x28))
    return false;
  pending_ops &= ~OP_CLEAR_FLE;
  return true;
}

bool asix88772_eth::loop() {
  if (pending_ops & OP_INIT && !init())
    return false;
  if (pending_ops & OP_CLEAR_FLE && !clear_FLE())
    return false;
  if (pending_ops & OP_SET_MAC && !set_node_ID())
    return false;
  if (pending_ops & OP_SET_MULTICAST && !update_mac_filter())
    return false;
  if (pending_ops & OP_UPDATE_MEDIUM && !update_medium_mode())
    return false;
  if (pending_ops & OP_UPDATE_BMCR && !update_bmcr())
    return false;
  if (pending_ops & OP_UPDATE_ANAR && !update_anar())
    return false;
  if (pending_ops & OP_UPDATE_BMSR && !update_bmsr())
    return false;

  rx_pump();

  return bmsr & BMSR_LINK_STATUS;
}

void asix88772_eth::Event(EventResponderRef evt) {
  ((asix88772_eth*)evt.getContext())->loop();
}

void asix88772_eth::bulk_in(int r, usb_bulkintr_sg* sg) {
  auto p = sg;
  while (p->data) {
    auto buf = (read_buffer*)p->data;
    int len = r;
    if (len > 0) {
      if (len > p->wLength) len = p->wLength;
      input_filled.Put({buf, (size_t)len}, -1);
      r -= len;
    }
    else input_buffers.Put(buf, -1);
    ++p;
  }

  // recycle sg if possible
  rx_pump(sg);
}

void asix88772_eth::rx_pump(usb_bulkintr_sg* sg) {
  const size_t gather_len = 4;

  if (getDevice()) {
    while (input_buffers.Size() >= gather_len) {
      if (sg==NULL) sg = new(std::nothrow) usb_bulkintr_sg[gather_len+1];
      if (sg) {
        // fill scatter-gather list
        for (size_t i=0; i < gather_len; i++) {
          read_buffer *rd;
          if (input_buffers.Get(rd, -1) != ATOM_OK) {
            if (i) break; // continue with less than gather_len buffers
            // else something has gone horribly wrong, bail
            goto bail_out;
          }
          sg[i] = {rd->data, sizeof(rd->data)};
        }

        int ret = BulkMessage(ep_in, sg, [=](int r){bulk_in(r, sg);});
        if (ret < 0) {
          //dprintf("Failed to queue input: %d\n", errno);
          bulk_in(ret, sg);
          return;
        }

        sg = NULL;
      }
    }
  }

bail_out:
  delete[] sg;
}

void asix88772_eth::submit_read_buffer(read_buffer& buf) {
  input_buffers.Put(&buf, -1);
  rx_pump();
}

bool asix88772_eth::get_read(read_buffer*& buf, size_t& len) {
  filled_buf f;
  if (input_filled.Get(f, -1) == ATOM_OK) {
    buf = f.buf;
    len = f.length;
    return true;
  }
  return false;
}

bool asix88772_eth::output_frame(const void* frame, size_t len) {
  int ret = BulkMessage(ep_out, len, frame);
  return ret >= 0 && (size_t)ret >= len;
}

void asix88772_eth::restart_auto_negotiation() {
  bmcr |= BMCR_RESTART_AUTO_NEG;
  pending_ops |= OP_UPDATE_BMCR;
}

void asix88772_eth::reset_phy() {
  bmcr |= BMCR_RESET;
  pending_ops |= OP_UPDATE_BMCR;
}

bool asix88772_eth::getFullDuplex() const {
  if ((bmsr & BMSR_AUTO_NEG_COMPLETE)==0)
    return bmcr & BMCR_DUPLEX_MODE;
  return anar & anlpar & (ANAR_TX_FD|ANAR_10_FD);
}

void asix88772_eth::setFullDuplex(bool duplex) {
  bmcr = (bmcr & ~BMCR_DUPLEX_MODE) | (duplex ? BMCR_DUPLEX_MODE : 0);
  pending_ops |= OP_UPDATE_BMCR|OP_UPDATE_ANAR;
}

bool asix88772_eth::get100mbps() const {
  if ((bmsr & BMSR_AUTO_NEG_COMPLETE)==0)
    return bmcr & BMCR_SPEED_SELECTION;
  return anar & anlpar & (ANAR_TX_FD|ANAR_TX_HD);
}

void asix88772_eth::set100mbps(bool speed) {
  bmcr = (bmcr & ~BMCR_SPEED_SELECTION) | (speed ? BMCR_SPEED_SELECTION : 0);
  pending_ops |= OP_UPDATE_BMCR|OP_UPDATE_ANAR;
}

bool asix88772_eth::getAutoNegotiation() const {
  return bmcr & BMCR_AUTO_NEGOTIATE;
}

void asix88772_eth::setAutoNegotiation(bool auto_neg) {
  bmcr = (bmcr & ~BMCR_AUTO_NEGOTIATE) | (auto_neg ? BMCR_AUTO_NEGOTIATE : 0);
  pending_ops |= OP_UPDATE_BMCR;
}

void asix88772_eth::setPHYPower(bool on) {
  bmcr = (bmcr & ~BMCR_POWER_DOWN) | (on ? 0 : BMCR_POWER_DOWN);
  pending_ops |= OP_UPDATE_BMCR;
}

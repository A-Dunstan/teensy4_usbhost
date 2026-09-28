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

#ifndef _USB_ASIX_88772
#define _USB_ASIX_88772

class asix88772_eth : public USB_Driver, public USB_Driver::Factory {
  enum { MAX_INPUT_BUFFERS = 16 };
public:
  struct read_buffer {
    uint8_t data[512] __attribute__((aligned(32)));
  };
  constexpr static size_t max_input_buffers() { return MAX_INPUT_BUFFERS; }

private:
  uint8_t status[8] __attribute__((aligned(32)));

  struct filled_buf {
    read_buffer* buf;
    size_t length;
  };

  TAtomQueue<read_buffer*, MAX_INPUT_BUFFERS> input_buffers;
  TAtomQueue<filled_buf, MAX_INPUT_BUFFERS> input_filled;
  void bulk_in(int, usb_bulkintr_sg*);
  void rx_pump(usb_bulkintr_sg* sg=NULL);

  struct mac_addr {
    uint8_t addr[6];
    mac_addr() {}
    mac_addr(const void* m) {
      memcpy(addr, m, 6);
    }
    bool operator==(const mac_addr& src) const {
      return memcmp(addr, src.addr, 6)==0;
    }
    bool isMulticast() const { return addr[0] & 1; }
  };

  std::vector<mac_addr> mac_filter;
  mac_addr asix_mac;

  EventResponder evt;
  static void Event(EventResponderRef ref);
  uint8_t chip_type;
  uint16_t bmcr;
  uint16_t bmsr;
  uint16_t anar;
  uint16_t anlpar;

  union {
    struct {
      uint8_t external:5;
      uint8_t external_type:3;
      uint8_t internal:5;
      uint8_t internal_type:3;
    };
    uint16_t val;
  } PHY_id;

  uint8_t ep_status;
  uint8_t ep_out;
  uint8_t ep_in;
  volatile uint32_t pending_ops;
  uint8_t last_int;

  bool vendor_command(uint8_t,uint16_t,uint16_t,uint16_t,void*);
  bool vendor_command(uint8_t,uint16_t,uint16_t,uint16_t,const void*);

  // template to ensure amount/type of arguments matches the command type
  template <uint8_t cmd, typename...Args>
  bool vendor_command(Args...);

  void interrupt(int);
  const USBCallback status_cb = [=](int r) { interrupt(r); };

  bool init();
  bool update_mac_filter();
  bool write_PHY(uint8_t phy_reg, const uint16_t val, bool internal=true);
  bool read_PHY(uint8_t phy_reg, uint16_t& val, bool internal=true);
  bool set_node_ID();
  bool update_bmcr();
  bool update_bmsr();
  bool update_anar();
  bool update_medium_mode();
  bool clear_FLE();

protected:
  USB_Driver* offer(const usb_device_descriptor*, const usb_configuration_descriptor*, const USB_Device*) override;
  void detach(void) override;
  bool attach(const usb_device_descriptor*, const usb_configuration_descriptor*) override;
public:
  asix88772_eth(bool autoNegotiate=true, bool speed=true, bool duplex=true);
  bool get_mac(uint8_t* mac);
  bool set_mac(const mac_addr mac);
  bool filter_address(mac_addr mac, bool allow);
  void restart_auto_negotiation();
  void reset_phy();
  bool loop(); // returns state of the link
  void submit_read_buffer(read_buffer&);
  bool get_read(read_buffer*&, size_t&); // returns a filled buffer
  bool output_frame(const void* frame, size_t len);
  bool getFullDuplex() const;
  void setFullDuplex(bool);
  bool get100mbps() const;
  void set100mbps(bool);
  bool getAutoNegotiation() const;
  void setAutoNegotiation(bool);
  void setPHYPower(bool);
};

#endif // _USB_ASIX_88772

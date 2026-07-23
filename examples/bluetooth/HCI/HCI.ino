#include <teensy4_usbhost.h>

static DMAMEM TeensyUSBHost2 usb;

class BT_HCI_test : USB_Driver, USB_Driver::Factory {
  uint8_t event_in;
  uint8_t acl_in;
  uint8_t acl_out;

  uint8_t buf[64];

  bool find_endpoints(const usb_interface_descriptor* desc, uint8_t& int_in, uint8_t& bulk_in, uint8_t& bulk_out) {
    int_in = bulk_in = bulk_out = 255;

    for (uint8_t i=0; i < desc->bNumEndpoints; i++) {
      auto ep = get_interface_endpoint(desc, i);
      if (ep) {
        uint8_t ep_type = ep->bmAttributes & 3;
        if (ep->bEndpointAddress & 0x80) { // IN endpoint
          if (ep_type == USB_ENDPOINT_INTERRUPT)
            int_in = ep->bEndpointAddress;
          else if (ep_type == USB_ENDPOINT_BULK)
            bulk_in = ep->bEndpointAddress;
        } else if (ep_type == USB_ENDPOINT_BULK) { // OUT endpoint
          bulk_out = ep->bEndpointAddress;
        }
      }

      if (int_in!=255 && bulk_in!=255 && bulk_out!=255) return true;
    }

    return false;
  }

  USB_Driver* offer(const usb_interface_descriptor* id, size_t, const USB_Device* d) override {
    uint8_t ep[3];

    // ensure we're not already attached to something
    if (getDevice() != NULL)
      return NULL;

    // only accept bluetooth HCI interfaces
    if (id->bInterfaceClass!=0xE0 || id->bInterfaceSubClass!=0x01 || id->bInterfaceProtocol!=0x01)
      return NULL;

    if (id->bNumEndpoints != 3)
      return NULL;

    if (!find_endpoints(id, ep[0], ep[1], ep[2])) {
      dprintf("Failed to find endpoints\n");
      return NULL;
    }

    return this;
  }

  void detach(void) override {
    dprintf("HCI interface shutdown\n");
  }

  void event_receive(int r) {
    dprintf("HCI event response: %d\n", r);
    if (r >= 0) {
      if (r>=12) { // must receive at least 12 bytes
        if (buf[0]==0x0E) { // HCI_COMMAND_COMPLETE
          if (buf[1]>=0x0A) { // payload length >= 10 bytes
            if (buf[3]==0x09 && buf[4]==0x10 && buf[5]==0x00) { // ocf/ogf matches read_bd_addr command
              dprintf("BD ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n", buf[11], buf[10], buf[9], buf[8], buf[7], buf[6]);
            }
          }
        }
      }
      
      // receive next event
      InterruptMessage(event_in, sizeof(buf), buf, [=](int r) {event_receive(r);});
    }
  }

  bool attach(const usb_interface_descriptor* desc, size_t) override {
    static const uint8_t read_bd_addr_cmd[] = {0x09, 0x10, 0x00};

    if (!find_endpoints(desc, event_in, acl_in, acl_out))
      return false;

    dprintf("HCI interface startup %02X %02X %02X\n", event_in, acl_in, acl_out);

    // start receiving events
    event_receive(0);
    // send command to retrieve BD address
    if (ControlMessage(USB_CTRLTYPE_TYPE_CLASS, 0, 0, 0, sizeof(read_bd_addr_cmd), read_bd_addr_cmd, [=](int r) {
      dprintf("send read_bd_addr result: %d\n", r);
    }) < 0) {
      dprintf("Failed to send read_bd_addr command\n");
      return false;
    }

    return true;
  }

public:
  BT_HCI_test() = default;
};

AIC8800D80 wifi;
BT_HCI_test bt;

FLASHMEM void setup() {
  Serial.begin(0);
  while (!Serial);

  usb.begin();
}

void loop() {
}

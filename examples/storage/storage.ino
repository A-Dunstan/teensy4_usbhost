#include "teensy4_usb.h"

#define USE_MASS_STORAGE
#include "usb_drivers.h"

static DMAMEM USB_Host_Port1 usb;

class USB_Bulk_Storage : public USB_Driver_FactoryGlue<USB_Bulk_Storage> {
  void detach(void);
  USB_Bulk_Storage(USB_Device*);
public:
  static bool offer_interface(const usb_interface_descriptor*,size_t);
  static USB_Driver* attach_interface(const usb_interface_descriptor*,size_t,USB_Device*);
};

void USB_Bulk_Storage::detach(void) {
  dprintf("USB_Bulk_Storage<%p>: deleting\n", this);
  delete this;
}

USB_Bulk_Storage::USB_Bulk_Storage(USB_Device *d) : USB_Driver_FactoryGlue<USB_Bulk_Storage>(d) {
  dprintf("Created new USB_Bulk_Storage %p\n", this);
}

bool USB_Bulk_Storage::offer_interface(const usb_interface_descriptor* id, size_t) {
  dprintf("USB_Bulk_Storage::offer_interface\n");
  if (id->bNumEndpoints < 2) return false;
  if (id->bInterfaceClass != 8) return false;
  if (id->bInterfaceSubClass != 6) return false;
  if (id->bInterfaceProtocol != 80) return false;
  return true;
}

USB_Driver* USB_Bulk_Storage::attach_interface(const usb_interface_descriptor* id, size_t, USB_Device *dev) {
  return new(std::nothrow) USB_Bulk_Storage(dev);
}

void setup() {
  Serial.begin(0);
  if (CrashReport) CrashReport.printTo(Serial);

  pinMode(LED_BUILTIN,OUTPUT);
  usb.begin();
}

void loop() {
  atomTimerDelay(SYSTEM_TICKS_PER_SEC);
  digitalToggle(LED_BUILTIN);
}

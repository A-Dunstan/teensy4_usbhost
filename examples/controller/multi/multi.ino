#include <teensy4_usbhost.h>

DMAMEM static TeensyUSBHost2 usb;

class MultiController : public Gamepad, private USB_Driver::Factory {
  template <class C>
  class DynController : public C {
    void detach(void) override { C::detach(); static_cast<MultiController&>(C::pad).driver = NULL; delete this; }
  public:
    DynController(MultiController& _host) : C(_host) {}
  };

  // USB_Driver::Factory
  USB_Driver* offer(const usb_device_descriptor* dd, const usb_configuration_descriptor* cd, const USB_Device*) override {
    if (driver) return NULL;
    if (PS3PadBase::driver_match(dd, cd))
      driver = new DynController<PS3PadBase>(*this);
    else if (PCS_PSPadBase::driver_match(dd, cd))
      driver = new DynController<PCS_PSPadBase>(*this);

    return driver;
  }
  USB_Driver* offer(const usb_interface_descriptor* id, size_t length, const USB_Device*) override {
    if (driver) return NULL;
    if (XBOX360PadBase::driver_match(id, length))
      driver = new DynController<XBOX360PadBase>(*this);

    return driver;
  }
  USB_Driver *driver = NULL;
};

DMAMEM static MultiController pad1;

int stickX = -1;
int stickY = -1;

void setup() {
  Serial.begin(0);
  while (!Serial);

  analogWriteFrequency(LED_BUILTIN, 256);
  analogWrite(LED_BUILTIN, 128);

  usb.begin();
  Serial.println("Waiting for Gamepad...");
}

void loop() {
  static bool found = false;
  if (!pad1) { // pad not found
    if (found) {
      Serial.println("Gamepad was disconnected");
      found = false;
    }
    atomTimerDelay(SYSTEM_TICKS_PER_SEC/10);
    return;
  } else if (!found) {
    Serial.print("Found a gamepad: ");
    Serial.println(pad1.getDeviceType());
    found = true;
  }

  pad1.update();
  pad1.setRumble(pad1.triggerL(), pad1.triggerR());
  int x = pad1.stickLX();
  int y = pad1.stickLY();
  if (x != stickX || y != stickY) {
    stickX = x;
    stickY = y;
    if (x <= -10 && y >= 10)
      pad1.setPlayerLED(0);
    else if (y >= 10)
      pad1.setPlayerLED(1);
    else if (x <= -10)
      pad1.setPlayerLED(2);
    else
      pad1.setPlayerLED(3);
  }
  analogWrite(LED_BUILTIN, 128+(pad1.stickRY()/256));

  if (pad1.changed()) {
    auto down = pad1.pressed();
    auto up = pad1.released();
    if (down) {
      Serial.print("Pressed:");
      for (int i=0; i < 18; i++) {
        if (down & (1<<i)) {
          Serial.print(' ');
          Serial.print(pad1.getButtonName(i));
        }
      }
      Serial.println();
    }
    if (up) {
      Serial.print("Released:");
      for (int i=0; i < 18; i++) {
        if (up & (1<<i)) {
          Serial.print(' ');
          Serial.print(pad1.getButtonName(i));
        }
      }
      Serial.println();
    }
  }
}
#include <teensy4_usbhost.h>

#ifndef MOUSE_INTERFACE
#error Set USB type to a configuration that includes mouse
#endif

DMAMEM static TeensyUSBHost2 usb;

class serial_mouse : public ch341::serial {
public:
  void status_change(bool cts, bool dsr, bool ring, bool connect) override {
    dprintf("Serial status change: CTS %d DSR %d RI %d CD %d\n", cts, dsr, ring, connect);
  }
  serial_mouse() {
    /* this is meant to be 7 bits, no parity, 1 stop but there's no predefined mode for that,
     * plus CH340 seems to work fine set to 8 bits (just ignore the top bit)
     */
    begin(1200, SERIAL_8N1, false);
  }
};

DMAMEM static serial_mouse mouse;

FLASHMEM void setup() {
  Serial.begin(0);
  while (!Serial);

  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println("Press Enter to continue.");
    while (Serial.read() != '\n');
  }

  usb.begin();

  Serial.println("Waiting for USB Serial...");
  while (!mouse);
  Serial.println("Found USB Serial");
  mouse.set_dtr_rts(true, true);
}

void loop() {
  if (Serial.read() == 't') {
    Serial.println("Toggling DTR/RTS");
    mouse.set_dtr_rts(false, false);
    delay(200);
    mouse.set_dtr_rts(true, true);
  }

  while (mouse.available() >= 3) {
    // Microsoft Serial Mouse protocol
    uint8_t d1 = mouse.read();
    Serial.print("DATA: ");
    Serial.print(d1, HEX);
    if (d1 & 0x40) {
      uint8_t d2 = mouse.peek();
      if ((d2 & 0x40)==0) {
        mouse.read();
        uint8_t d3 = mouse.peek();
        if ((d3 & 0x40)==0) {
          mouse.read();
          Serial.print(' ');
          Serial.print(d2, HEX);
          Serial.print(' ');
          Serial.print(d3, HEX);
          uint8_t d4 = 0;
          // logitech extension for 3 button mouse
          if (mouse.available() && (mouse.peek() & 0x40)==0) {
            d4 = mouse.read();
            Serial.print(' ');
            Serial.print(d4, HEX);
          }
          int8_t x = (d1 << 6) | (d2 & 0x3F);
          int8_t y = ((d1 << 4) & 0xC0) | (d3 & 0x3F);
          Mouse.set_buttons(d1 & 0x20, d4 & 0x20, d1 & 0x10);
          Mouse.move(x, y);
        }
      }
    }
    Serial.println();
  }
}

#include <teensy4_usbhost.h>

DMAMEM static TeensyUSBHost2 usb;
DMAMEM static PS3Pad pad;

#ifndef MOUSE_INTERFACE
#error Set Teensy USB Type to a configuration that includes mouse
#endif

FLASHMEM void setup() {
  Serial.begin(0);
  while (!Serial);

  if (CrashReport) Serial.print(CrashReport);

  usb.begin();
  Serial.println("Waiting for PS3 Pad...");
}

void loop() {
  static bool found = false;
  if (!pad) {
    if (found) {
      Serial.println("PS3 pad was disconnected");
      found = false;
    }
    else atomTimerDelayms(10);
    return;
  } else if (!found) {
    Serial.println("Found PS3 pad");
    found = true;
  }

  pad.update();
  int x = pad.stickLX();
  int y = pad.stickLY();
  auto b = pad.buttons();
  x = (x < -16384) ? -1 : (x > 16384 ? 1:0);
  y = (y < -16384) ? 1 : (y > 16384 ? -1:0);
  Mouse.move(x, y);
  Mouse.set_buttons(b & PADBUTTON_A ?1:0, b & PADBUTTON_X ?1:0, b & PADBUTTON_B ?1:0);
}

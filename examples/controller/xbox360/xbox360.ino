#define USE_XBOX360 1
#include <teensy4_usbhost.h>

static DMAMEM TeensyUSBHost2 usb;
static XBOX360Pad pad1;

void setup() {
  Serial.begin(0);
  while (!Serial);

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWriteFast(LED_BUILTIN, LOW);

  usb.begin();
  Serial.println("Waiting for XBOX 360 gamepad...");
}

void loop() {
  if (!pad1) {
    digitalWriteFast(LED_BUILTIN, LOW);
    atomTimerDelay(SYSTEM_TICKS_PER_SEC/10);
    return;
  }

  pad1.update();
  if (pad1.changed())
    digitalToggleFast(LED_BUILTIN);
}
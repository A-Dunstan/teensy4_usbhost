#define USE_XBOX360 1
#include <teensy4_usbhost.h>

static TeensyUSBHost2 usb;
static XBOX360Pad pad1;
int stickX = -1;
int stickY = -1;

void setup() {
  Serial.begin(0);
  while (!Serial);

  analogWriteFrequency(LED_BUILTIN, 256);
  analogWrite(LED_BUILTIN, 128);

  usb.begin();
  Serial.println("Waiting for XBOX 360 gamepad...");
}

void loop() {
  if (!pad1) { // pad not found
    atomTimerDelay(SYSTEM_TICKS_PER_SEC/10);
    return;
  }

  pad1.update();
  pad1.setRumble(pad1.triggerL(), pad1.triggerR());
  int x = pad1.stickLX();
  int y = pad1.stickLY();
  if (x != stickX || y != stickY) {
    stickX = x;
    stickY = y;
    if (x <= -10 && y >= 10)
      pad1.setLED(6);
    else if (y >= 10)
      pad1.setLED(7);
    else if (x <= -10)
      pad1.setLED(8);
    else
      pad1.setLED(9);
  }
  analogWrite(LED_BUILTIN, 128+(pad1.stickRY()/256));

  if (pad1.changed()) {
    static const char* b_names[16] = {
      "DPAD_UP", "DPAD_DOWN", "DPAD_LEFT", "DPAD_RIGHT", "START", "SELECT", "L3", "R3",
      "LB",  "RB", "MENU", NULL /*no button 12*/, "A", "B", "X", "Y"
    };
    auto down = pad1.pressed();
    auto up = pad1.released();
    if (down) {
      Serial.print("Pressed:");
      for (int i=0; i < 16; i++) {
        if (i == 11) continue;
        if (down & (1<<i)) {
          Serial.print(' ');
          Serial.print(b_names[i]);
        }
      }
      Serial.println();
    }
    if (up) {
      Serial.print("Released:");
      for (int i=0; i < 16; i++) {
        if (i == 11) continue;
        if (up & (1<<i)) {
          Serial.print(' ');
          Serial.print(b_names[i]);
        }
      }
      Serial.println();
    }
  }
}
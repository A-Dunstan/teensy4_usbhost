#include <usb_mouse.h> // for having Teensy act as a mouse

#define USE_MOUSE 1
#include <teensy4_usbhost.h>

static DMAMEM TeensyUSBHost2 usb;
static DMAMEM USBMouse myMouse;

#define NUM_EVENTS 5
static mouse_event m_events[NUM_EVENTS];
static ATOM_QUEUE mouse_queue;

void setup() {
  Serial.begin(0);
  while (!Serial);

  pinMode(LED_BUILTIN,OUTPUT);

  usb.begin();
  atomQueueCreate(&mouse_queue, (uint8_t*)&m_events, sizeof(m_events[0]), NUM_EVENTS);
  myMouse.begin(&mouse_queue);
  Serial.println("Waiting for mouse...");
}

void loop() {
  // is mouse attached?
  if (myMouse) {
    digitalWriteFast(LED_BUILTIN, HIGH);
    mouse_event ev;
    // wait (10 systicks) for an event
    if (atomQueueGet(&mouse_queue, 10, (uint8_t*)&ev) == ATOM_OK) {
      Serial.print("Mouse Event(");
      Serial.print(ev.len);
      Serial.print("): Buttons ");
      Serial.print(ev.buttons, BIN);
      Serial.print("\txdelta ");
      Serial.print(ev.xdelta);
      Serial.print("\tydelta ");
      Serial.print(ev.ydelta);
      Serial.print("\tzdelta ");
      Serial.println((ev.len >= 4) ? ev.zdelta:0);
      // if Teensy USB Type includes Mouse, forward mouse events to host PC
#if defined(MOUSE_INTERFACE)
      usb_mouse_buttons_state = ev.buttons & MOUSE_ALL;
      Mouse.move(ev.xdelta, ev.ydelta, ev.zdelta);
#endif
    }
  }
  else digitalWriteFast(LED_BUILTIN, LOW);
}

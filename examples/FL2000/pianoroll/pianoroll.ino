#include <teensy4_usbhost.h>

#ifndef MIDI_INTERFACE
#error Set USB type to "MIDI" under the Tools menu
#endif

static DMAMEM TeensyUSBHost2 usb;
static FL2000 fl2000;
static EventResponder monitor_responder;
ATOM_SEM frame_done;

/* 640x200 = 128000
 * rounded up to nearest power of two = 131072 (2^17)
 * this is the size (and alignment) of our framebuffer
 */

#define WIDTH 640
#define HEIGHT 200
#define FREQ 70
#define COLOR_FORMAT (COLOR_FORMAT_RGB_8_INDEXED)
#define FB_SEG    17

static uint8_t fb[1 << FB_SEG] DMAMEM __attribute__((aligned(1 << FB_SEG)));
static size_t fb_offset;
static const size_t fb_mask = (1 << FB_SEG) - 1;

#define RGB(R, G, B)  ((R<<16)|(G<<8)|(B))
static void setPalette(FL2000* monitor) {
  uint32_t pal[] = {
  RGB(0, 0, 45),           // background: dark blue
  RGB(140, 112, 0),        // grid lines: dark orange
  RGB(255, 255, 255),      // channel 1: white
  RGB(226, 18, 18),        // channel 2: red
  RGB(255, 22, 85),        // channel 3: pale yellow
  RGB(44, 222, 20),        // channel 4: bright green
  RGB(11, 95, 0),          // channel 5: dark green
  RGB(14, 242, 189),       // channel 6: bright teal
  RGB(40, 130, 109),       // channel 7: dark teal
  RGB(33, 161, 236),       // channel 8: pale blue
  RGB(4, 28, 160),         // channel 9: dark blue
  RGB(77, 88, 95),         // channel 10: grey
  RGB(75, 1, 105),         // channel 11: purple
  RGB(175, 15, 238),       // channel 12: light purple
  RGB(208, 13, 114),       // channel 13: pink
  RGB(183, 44, 44),        // channel 14: dark red
  RGB(254, 135, 7),        // channel 15: orange
  RGB(164, 254, 7)         // channel 16: fluorescent yellow
  };
  monitor->setPalette(0, sizeof(pal)/sizeof(pal[0]), pal);
}

bool paused = false;
void monitor_event(EventResponder& ev) {
  int status = ev.getStatus();
  auto monitor = (FL2000*)ev.getContext();

  switch (status) {
    case MONITOR_NOTIFY_ERROR:
      Serial.printf("Monitor instance was deleted\n");
      break;
    case MONITOR_NOTIFY_DISCONNECTED:
      Serial.printf("Monitor %p was disconnected\n", monitor);
      break;
    case MONITOR_NOTIFY_CONNECTED:
      Serial.printf("Monitor %p was connected\n", monitor);
      Serial.printf("setFormat (%ux%u@%uHz) result: %d\n", WIDTH, HEIGHT, FREQ, monitor->setFormat(WIDTH, HEIGHT, FREQ, COLOR_FORMAT));
      setPalette(monitor);
      monitor->setFrame(fb, WIDTH);
      break;
    case MONITOR_NOTIFY_FRAMEDONE:
      if (!paused) {
        atomSemPut(&frame_done);
      }
      break;
    default:
      Serial.printf("Unknown monitor %p event: %d\n", monitor, status);
  }
}

static byte notes[16][128];

void setup() {
  Serial.begin(0);
  while (!Serial);

  if (CrashReport) {
    while(!Serial);
    Serial.print(CrashReport);
    Serial.println("Press enter to continue");
    while (Serial.read() != '\n');
  }

  memset(fb, 0, sizeof(fb));
  atomSemCreateLimit(&frame_done, 0, 1);
  monitor_responder.attach(monitor_event);
  fl2000.set_monitor_event(&monitor_responder);

  usb.begin();
}

static void render(void) {
  static int linecount;

  uint8_t bg;
  if (++linecount == 10) {
    linecount = 0;
    bg = 1;
  }
  else bg = 0;

  auto loffset = fb_offset + WIDTH*(HEIGHT-1);
  for (int i=0; i < WIDTH; i+=5) {
    uint8_t p = bg;
    int note = i/5;
    byte vel = 0;
    for (int i=0; i < 16; i++) {
      if (notes[i][note] > vel) {
        p = i+2;
        vel = notes[i][note];
      }
    }

    // separator
    fb[loffset++ & fb_mask] = 1;
    fb[loffset++ & fb_mask] = bg;
    fb[loffset++ & fb_mask] = p;
    fb[loffset++ & fb_mask] = p;
    fb[loffset++ & fb_mask] = bg;
  }

  fl2000.setFrame(fb+fb_offset, WIDTH, 32-FB_SEG);
  fb_offset = (fb_offset + WIDTH) & fb_mask;
}

void loop() {
  if (usbMIDI.read()) {
    auto type = usbMIDI.getType();
    auto channel = usbMIDI.getChannel();
    if (channel >= 1 && channel <= 16) {
      if (type == 0x80) { // note off
        auto note = usbMIDI.getData1();
        if (note < 128)
          notes[channel-1][note] = 0;
      } else if (type == 0x90) { // note on
        auto note = usbMIDI.getData1();
        auto vel = usbMIDI.getData2();
        if (note < 128)
          notes[channel-1][note] = vel;
      } else if (type == 0xB0) {
        auto cc = usbMIDI.getData1();
        if (cc == 123) { // all notes off
          memset(notes[channel-1], 0, sizeof(notes[0]));
        }
      } else if (type == 0xC0) { // program change
        memset(notes[channel-1], 0, sizeof(notes[0]));
      }
    }
  }

  if (atomSemGet(&frame_done, -1) == ATOM_OK)
    render();

  if (Serial.read() == 'p')
    paused = !paused;
}

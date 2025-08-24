#include <Audio.h>
#include <teensy4_usbhost.h>

#ifndef AUDIO_INTERFACE
#error Set USB type to "Audio" under the Tools menu
#endif

AudioInputUSB  audioIn;
AudioOutputSPDIF3 audioOut;
AudioMixer4    mixer;
AudioAnalyzeFFT1024 fft1024;
AudioConnection p1(audioIn, 0, mixer, 0);
AudioConnection p2(audioIn, 1, mixer, 1);
AudioConnection p3(mixer, fft1024);
AudioConnection p4(audioIn, 0, audioOut, 0);
AudioConnection p5(audioIn, 1, audioOut, 1);

static DMAMEM TeensyUSBHost2 usb;
static FL2000 fl2000;
static EventResponder monitor_responder;
ATOM_SEM frame_done;


// HEIGHT should be less or equal to WIDTH for this example
// 800x600x60 is possible but requires CPU to be at least 600MHz
#define WIDTH 800
#define HEIGHT 600
#define FREQ 56
#define COLOR_FORMAT (COLOR_FORMAT_RGB_8_INDEXED/*|COLOR_FORMAT_NODMA*/)

static uint8_t fb0[WIDTH*HEIGHT] DMAMEM __attribute__((aligned(32)));
// enable this if PSRAM/other external memory is available to reduce tearing
#if 0
static uint8_t fb1[WIDTH*HEIGHT] EXTMEM __attribute__((aligned(32)));
#else
uint8_t* fb1 = fb0;
#endif
static auto *fb = fb0;

static void setPalette(FL2000* monitor) {
  uint32_t pal[256];
  for (int i=0; i < 128; i++) {
    pal[i] = ((i*2) << 16) | (255 << 8);
  }
  for (int i=0; i < 127; i++) {
    pal[128+i] = (255 << 16) | ((254 - i*2) << 8);
  }
  pal[255] = 45; // dark blue background
  monitor->setPalette(0, 256, pal);
}


// only update every X frames
#define FRAME_INTERVAL 1
void monitor_event(EventResponder& ev) {
  int status = ev.getStatus();
  auto monitor = (FL2000*)ev.getContext();
  static int framecount;

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
      fb = fb0;
      memset(fb, 255, sizeof(fb0));
      monitor->setFrame(fb, WIDTH*sizeof(*fb));
      break;
    case MONITOR_NOTIFY_FRAMEDONE:
      if (++framecount >= FRAME_INTERVAL) {
        framecount = 0;
        atomSemPut(&frame_done);
      }
      break;
    default:
      Serial.printf("Unknown monitor %p event: %d\n", monitor, status);
  }
}

void setup() {
  AudioMemory(16);

  Serial.begin(0);
  while (!Serial);

  if (CrashReport) {
    while(!Serial);
    Serial.print(CrashReport);
    Serial.println("Press enter to continue");
    while (Serial.read() != '\n');
  }

  mixer.gain(0, 0.707);
  mixer.gain(1, 0.707);

  atomSemCreateLimit(&frame_done, 0, 1);
  monitor_responder.attach(monitor_event);
  fl2000.set_monitor_event(&monitor_responder);

  usb.begin();
}

// draw a line from srcx:srcy to dstx:dsty with linear color interpolation
static void draw_line(int srcx, int srcy, int dstx, int dsty, uint8_t srccolor, uint8_t dstcolor) {
  int width = srcx > dstx ? srcx - dstx : dstx - srcx;
  int height = srcy > dsty ? srcy - dsty : dsty - srcy;
  int count = width > height ? width : height;
  if (count == 0) {
    fb[srcx + srcy*WIDTH] = srccolor;
    return;
  }
  int slopex = ((dstx - srcx) << 16) / count;
  int slopey = ((dsty - srcy) << 16) / count;
  int slopecolor = ((dstcolor - srccolor) << 16) / count;

  while (count-->0) {
    int x = srcx + ((count * slopex) >> 16);
    int y = srcy + ((count * slopey) >> 16);
    fb[x + y*WIDTH] = srccolor + ((count * slopecolor) >> 16);
  }
}

static float scale = 10.0;
static float rot;
#define ROTATION_INC 0.02f
#define CIRCLE_POINTS 150 // max: 256

static void readbank(int bank, int& x, int& y, uint8_t& color) {
  unsigned int first = powf(bank, 1.12f);
  unsigned int last = powf(bank+1.0f, 1.12f)-1;
  float temp = fft1024.read(first, last) * scale / 2.0f;
  temp = log(temp + 1.0f);
  if (temp > 1.0f) temp = 1.0f;
  float mag = HEIGHT/4 + temp * (HEIGHT/4 - 5);
  x = (WIDTH/2) + sinf(((bank + rot) / (float)CIRCLE_POINTS) * 2 * M_PI) * mag;
  y = (HEIGHT/2) - cosf(((bank + rot) / (float)CIRCLE_POINTS) * 2 * M_PI) * mag;
  color = 254 * temp;
}

static void render(void) {
  if (fft1024.available()) {
    // switch to unused buffer
    fb = fb == fb0 ? fb1 : fb0;
    memset(fb, 255, sizeof(fb0));

    int prevx, prevy, firstx, firsty;
    uint8_t prevcolor, firstcolor;
    readbank(0, firstx, firsty, firstcolor);
    prevx = firstx;
    prevy = firsty;
    prevcolor = firstcolor;
    // draw 255 lines around the diameter
    for (int i=1; i < CIRCLE_POINTS; i++) {
      int x, y;
      uint8_t color;
      readbank(i, x, y, color);
      draw_line(prevx, prevy, x, y, prevcolor, color);
      prevx = x;
      prevy = y;
      prevcolor = color;
    }
    // draw final line (back to first point)
    draw_line(prevx, prevy, firstx, firsty, prevcolor, firstcolor);

    fl2000.setFrame(fb, WIDTH*sizeof(*fb));
    rot += (ROTATION_INC * FRAME_INTERVAL);
    if (rot >= (float)CIRCLE_POINTS) rot -= (float)CIRCLE_POINTS;
  }
}

void loop() {
  if (atomSemGet(&frame_done, -1) == ATOM_OK)
    render();
}

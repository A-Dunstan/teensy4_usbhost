#include <SD.h>
#include <Audio.h>
#include <teensy4_usbhost.h>

#ifndef AUDIO_INTERFACE
#error Set USB type to "Audio" under the Tools menu
#endif

AudioInputUSB  audioIn;
AudioOutputSPDIF3 audioOut;
AudioMixer4    mixer;
AudioAnalyzeFFT1024 fft1024;
AudioConnection patchCord1(audioIn, 0, mixer, 0);
AudioConnection patchCord2(audioIn, 1, mixer, 1);
AudioConnection PatchCord3(mixer, fft1024);
AudioConnection out1(audioIn, 0, audioOut, 0);
AudioConnection out2(audioIn, 1, audioOut, 1);

static DMAMEM TeensyUSBHost2 usb;
static FL2000 fl2000;
static EventResponder monitor_responder;
ATOM_SEM frame_ready;

#define WIDTH 320
#define HEIGHT 240
#define FREQ 60

static uint8_t fb0[WIDTH*(HEIGHT+1)] DMAMEM;
static auto *fb = fb0+WIDTH-1;

// only update every X frames
#define FRAME_INTERVAL 1

void monitor_event(EventResponder& ev) {
  int status = ev.getStatus();
  auto monitor = (FL2000*)ev.getData();
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
      Serial.printf("setFormat result: %d\n", monitor->setFormat(WIDTH, HEIGHT, FREQ, COLOR_FORMAT_RGB_8_INDEXED));
      // set palette entries
      for (int i=0; i < 256; i++) {
        float red, green, blue;

        float temp = i / 255.0f;
        if (temp < 0.5) {
          red = 0.0f;
          green = temp * 2.0f;
          blue = 2.0f * (0.5f - temp);
        } else {
          red = temp;
          green = 1.0f - temp;
          blue = 0.0f;
        }
        uint8_t r = 255 * red;
        uint8_t g = 255 * green;
        uint8_t b = 255 * blue;
        uint32_t c = (r << 16) | (g << 8) | b;
        monitor->setPalette(i, 1, &c);
      }
      monitor->setFrame(fb-(WIDTH-1), WIDTH*sizeof(*fb));
      break;
    case MONITOR_NOTIFY_FRAMEDONE:
      if (++framecount >= FRAME_INTERVAL) {
        framecount = 0;
        atomSemPut(&frame_ready);
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

  atomSemCreateLimit(&frame_ready, 0, 1);
  memset(fb0, 0, sizeof(fb0));
  monitor_responder.attach(monitor_event);
  fl2000.set_monitor_event(&monitor_responder);

  usb.begin();
}

static float scale = 12.0;

void loop() {
  if (atomSemGet(&frame_ready, -1) == ATOM_OK) {
    if (fft1024.available()) {
      // hopefully this happens while in vertical refresh...
      if (fb >= fb0+2*WIDTH) {
        memmove(fb0, fb-(WIDTH-1), (HEIGHT*WIDTH-1)*sizeof(*fb));
        fb = fb0+WIDTH-1;
      }

      auto *dst = fb;
      for (int i=0; i < HEIGHT; i++) {
        float temp = fft1024.output[HEIGHT-i-1] * scale / 32768.0f;
        temp = log(temp+1.0f);
        if (temp > 1.0) temp = 1.0;
        dst[WIDTH*i] = 255 * temp;
      }

      fl2000.setFrame(fb-(WIDTH-1), WIDTH*sizeof(*fb));
      ++fb;
    }
  }
}

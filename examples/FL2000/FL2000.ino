#include <teensy4_usbhost.h>

static DMAMEM TeensyUSBHost2 usb;
static FL2000 fl2000;

static EventResponder monitor_responder;

#define WIDTH 320
#define HEIGHT 240
#define FREQ 60

static uint16_t fb[WIDTH*HEIGHT] DMAMEM;
static int framecount;

void monitor_event(EventResponder& ev) {
  int status = ev.getStatus();
  auto monitor = (FL2000*)ev.getData();

  switch (status) {
    case MONITOR_NOTIFY_ERROR:
      Serial.printf("Monitor instance was deleted\n");
      break;
    case MONITOR_NOTIFY_DISCONNECTED:
      Serial.printf("Monitor %p was disconnected\n", monitor);
      break;
    case MONITOR_NOTIFY_CONNECTED:
      Serial.printf("Monitor %p was connected\n", monitor);
      Serial.printf("setFormat result: %d\n", monitor->setFormat(WIDTH, HEIGHT, FREQ, COLOR_FORMAT_RGB_16_565));
      monitor->setFrame(fb, WIDTH*sizeof(fb[0]));
      break;
    case MONITOR_NOTIFY_FRAMEDONE:
      ++framecount;
//      Serial.printf("Monitor %p frame completed (%u)\n", monitor, millis());
      if ((framecount&3)==0) {
        for (size_t i=0; i < sizeof(fb)/sizeof(fb[0]); i++) {
          if (fb[i]==65535)
            fb[i] = 0x0821;
          else if (fb[i])
            fb[i] += 0x0821;
        }
      }
      if (framecount == 300) {
//        Serial.printf("Temperature: %f\n", tempmonGetTemp());
        framecount = 0;
      }
      break;
    default:
      Serial.printf("Unknown monitor %p event: %d\n", monitor, status);
  }
}

static void FillFrameBuffer(void) {
  // draw mandelbrot
  float top = -0.4f;
  float bottom = 0.4f;
  float right = -0.8f;
  float left = -1.5f;

  float xscale = (right - left) / WIDTH;
  float yscale = (bottom - top) / HEIGHT;

  for (int y=0; y < (int)HEIGHT; y++)
  {
    for (int x=0; x < (int)WIDTH; x++)
    {
      float cx = x * xscale + left;
      float cy = y * yscale + top;

      float zx = 0, zy = 0;
      uint16_t c = 0;

      while (((zx * zx + zy * zy) < 5) && (c < 4095))
      {
        float tx = zx * zx - zy * zy + cx;
        zy =2 * zx * zy + cy;
        zx = tx;
        c++;
      }
      fb[y*WIDTH+x] = 4095 - c;
    }
  }
}

void setup() {
  Serial.begin(0);
  while (!Serial);

  if (CrashReport) {
    while(!Serial);
    Serial.print(CrashReport);
    Serial.println("Press enter to continue");
    while (Serial.read() != '\n');
  }

  monitor_responder.attach(monitor_event);
  fl2000.set_monitor_event(&monitor_responder);
  Serial.print("Initializing...");
  FillFrameBuffer();
  Serial.println("Done.");

  usb.begin();
}

void loop() {
}

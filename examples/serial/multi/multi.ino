#include <teensy4_usbhost.h>

DMAMEM static TeensyUSBHost2 usb;

class multi_serial : public ch341::serial, public cdc_acm {
public:
  multi_serial() { begin(115200); };

  void detach(void) override {
    ch341::serial::detach();
    cdc_acm::detach();
  }

  void begin(uint32_t baud, uint16_t format=SERIAL_8N1) override {
    ch341::serial::begin(baud, format, false);
    cdc_acm::begin(baud, format);
  }

  operator bool() override {
    return state & STATE_STARTED;
  }
};

static multi_serial USBSerial;

uint32_t count, prior_count;
uint32_t prior_msec;
uint32_t count_per_second;

void setup() {
  Serial.begin(0);

  if (CrashReport) CrashReport.printTo(Serial);

  pinMode(LED_BUILTIN,OUTPUT);

  usb.begin();
  Serial.println("Waiting to detect USB Serial cable...");
  while (!USBSerial);

  count = 10000000;
  prior_count = count;
  count_per_second = 0;
  prior_msec = millis();
}

void loop() {
  if (!USBSerial) return;

  int r=0;
  USBSerial.printf("count=%u, chars/sec=%u\n%n", count, count_per_second, &r);
  count += r;

  if (USBSerial.available()) {
    while ((r = USBSerial.read()) != -1) {
      if (r == '\r') r = '\n';
      putchar(r);
    }
    fflush(stdout);
  }

  uint32_t msec = millis();
  if (msec - prior_msec > 1000) {
    // when 1 second has elapsed, update the chars/sec count
    prior_msec = prior_msec + 1000;
    count_per_second = count - prior_count;
    prior_count = count;
    digitalToggleFast(LED_BUILTIN);
  }
}

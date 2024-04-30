#include "teensy4_usb.h"

#define USE_CH341
#include "usb_drivers.h"

DMAMEM static TeensyUSBHost2 usb;
DMAMEM static ch341::serial USBSerial;

uint32_t count, prior_count;
uint32_t prior_msec;
uint32_t count_per_second;

void setup() {
	Serial.begin(0);
	
	if (CrashReport) CrashReport.printTo(Serial);
	
	pinMode(LED_BUILTIN,OUTPUT);
	
	usb.begin();
	USBSerial.begin(115200);
	while (!USBSerial);
	
	Serial.println("USBSerial is ready");
	
	count = 10000000;
	prior_count = count;
	count_per_second = 0;
	prior_msec = millis();
}

void loop() {
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

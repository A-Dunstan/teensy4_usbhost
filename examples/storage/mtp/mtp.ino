#define USE_MASS_STORAGE_FAT
#include <teensy4_usbhost.h>

// SET TEENSY USB TYPE TO "Serial + MTP Disk"
// LED ON = USB drive is mounted, should be accessible via MTP
// LED OFF = USB drive not mounted

/* WARNING: Currently MTP does not work correctly when different drives
 * are removed / inserted due to using the FIRST drive as storage for
 * an index file. In some cases this can lead to data corruption/destruction
 * when a new drive is inserted!
 */

static DMAMEM TeensyUSBHost2 usb;
static USB_FAT_Volume USBVol;

FLASHMEM void setup() {
  Serial.begin(0);
  for (elapsedMillis t = 0; t < 5000;) {
    if (Serial) break;
  }

  if (CrashReport) CrashReport.printTo(Serial);

  usb.begin();
  USBVol.begin();
  MTP.begin();
  MTP.loop(); // get rid of thread-unsafe interval timer ASAP
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);
  MTP.addFilesystem(USBVol, "USB FAT Volume");

  Serial.println("MTP START");
}

void loop() {
  if (USBVol.mediaPresent() == false) {
    // try to mount something
    if (USBVol.mount())
      digitalWrite(LED_BUILTIN, HIGH);
    else digitalWrite(LED_BUILTIN, LOW);
  }

  MTP.loop();
}

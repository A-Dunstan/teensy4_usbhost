#include <teensy4_usbhost.h>

// SET TEENSY USB TYPE TO "Serial + MTP Disk"
// LED ON = USB drive is mounted, should be accessible via MTP
// LED OFF = USB drive not mounted

/* WARNING: Currently MTP does not work correctly when different drives
 * are removed / inserted due to using the FIRST drive as storage for
 * an index file. In some cases this can lead to data corruption/destruction
 * when changing drives!
 */

static DMAMEM TeensyUSBHost2 usb;
static USB_FAT_Volume USBVol;

FLASHMEM void setup() {
  MTP.begin();

  Serial.begin(0);
  elapsedMillis t = 0;
  do {
    MTP.loop(); // get rid of thread-unsafe interval timer ASAP
  } while (t < 5000);

  if (CrashReport) CrashReport.printTo(Serial);

  usb.begin();
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

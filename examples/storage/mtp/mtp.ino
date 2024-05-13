#include <teensy4_usb.h>
#include <MTP_Teensy.h>

#define USE_MASS_STORAGE_FAT
#include <usb_drivers.h>

// SET TEENSY USB TYPE TO "Serial + MTP Disk"
// LED ON = USB drive is mounted, should be accessible via MTP
// LED OFF = USB drive not mounted

#define MTP_HANDLE_NONE 0xFFFFFFFF

static DMAMEM TeensyUSBHost2 usb;
static USB_FAT_Volume USBVol;
static uint32_t mtp_handle;

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
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  // store the index in memory, not on a filesystem that could go away at any moment
  MTP.useFileSystemIndexFileStore(MTPStorage::INDEX_STORE_MEM_FILE);
  mtp_handle = MTP_HANDLE_NONE;

  Serial.println("MTP START");
}

void loop() {
  if (USBVol.mediaPresent() == false) {
    digitalWrite(LED_BUILTIN, LOW);
    if (mtp_handle != MTP_HANDLE_NONE) {
      MTP.send_StoreRemovedEvent(mtp_handle);
      MTP.storage()->removeFilesystem(mtp_handle);
      mtp_handle = MTP_HANDLE_NONE;
    }

    // try to mount something
    if (USBVol.mount()) {
      digitalWrite(LED_BUILTIN, HIGH);
      mtp_handle = MTP.addFilesystem(USBVol, "USB Drive");
      if (mtp_handle == MTP_HANDLE_NONE)
        Serial.println("Failed to add new filesystem to MTP");
    }
  }

  MTP.loop();
}

#include <teensy4_usb.h>
#include <MTP_Teensy.h>

#define USE_MASS_STORAGE_FAT
#include <usb_drivers.h>

// SET TEENSY USB TYPE TO "Serial + MTP Disk"

static DMAMEM TeensyUSBHost2 usb;
static USB_FAT_Volume USBVol;
static uint32_t mtp_handle = 0xFFFFFFFF;

void setup() {
  Serial.begin(0);
  elapsedMillis stimer;
  while (!Serial);

  if (CrashReport) CrashReport.printTo(Serial);

  usb.begin();
  USBVol.begin();
  MTP.begin();
  MTP.loop(); // get rid of thread-unsafe interval timer _immediately_
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  // store the index in memory, not on a filesystem that could go away at any moment
  MTP.useFileSystemIndexFileStore(MTPStorage::INDEX_STORE_MEM_FILE);

  printf("MTP START");
}

void loop() {
  if (USBVol.mediaPresent() == false) {
    digitalWrite(LED_BUILTIN, LOW);
    if (mtp_handle != 0xFFFFFFFF) {
      MTP.send_StoreRemovedEvent(mtp_handle);
      MTP.storage()->removeFilesystem(mtp_handle);
      mtp_handle = 0xFFFFFFFF;
    }

    if (USBVol.mount()) {
      digitalWrite(LED_BUILTIN, HIGH);
      mtp_handle = MTP.addFilesystem(USBVol, "USB Drive");
    }
  }

  MTP.loop();
}

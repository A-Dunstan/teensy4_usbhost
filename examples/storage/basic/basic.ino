/* basic.ino: Searches for a USB Mass Storage device, attempts to mount a
 * FAT12, FAT16, FAT32 or EXFAT partition and recursively lists all directories/files.
 * All attached drives, their LUNs and the partitions on them will be searched
 * sequentially until a FAT partition is found.
 * This only does a limited amount of retries; some USB drives require significant time
 * to initialize themselves and become ready.
 */

#include <teensy4_usb.h>
#include <usb_util.h>

#define USE_MASS_STORAGE_FAT
#include "usb_drivers.h"

static DMAMEM TeensyUSBHost2 usb;
static USB_FAT_Volume USBVol;

void setup() {
  Serial.begin(0);
  elapsedMillis stimer;
  do {
    if (Serial) break;
  } while (stimer < 5000);

  if (CrashReport) CrashReport.printTo(Serial);
  
  usb.begin();
  USBVol.begin();
  delay(1000);
  Serial.println("\n\nPress return to search for a USB drive");
}

void loop() {
  if (Serial.read() == '\n') {
    for (int retry=0; USBVol.mount()==false; retry++) {
      if (retry >= 5) {
        Serial.println("Failed to find a USB drive with a FAT partition, press return to try again");
        return;
      }
      delay(100);
    }
    
    Serial.print("Found a ");
    switch (USBVol.type()) {
      case 12: Serial.print("FAT12 "); break;
      case 16: Serial.print("FAT16 "); break;
      case 32: Serial.print("FAT32 "); break;
      case 64: Serial.print("EXFAT "); break;
      default: Serial.print("FAT?? "); break;
    }
    Serial.println("partition.");
    Serial.print("Checking free space... ");
    uint64_t freeSpace = USBVol.totalSize() - USBVol.usedSize(); Serial.println("Done.");
    Serial.print("Bytes free: "); Serial.print(freeSpace);
    Serial.print(" of "); Serial.print(USBVol.totalSize());
    Serial.print(" ("); Serial.print(freeSpace * 100 / USBVol.totalSize()); Serial.println("%)");

    File root = USBVol.open("/");
    if (root && root.isDirectory()) {
      Serial.println("--- FILE LISTING BEGIN ---");
      recurse_dir_print("/", root, Serial);
      Serial.println("--- FILE LISTING COMPLETE ---");
    }
    else Serial.println("Unable to open root directory");

    USBVol.unmount();
    Serial.println("\nUSB Drive may now be removed, press return to scan again");
  }
}

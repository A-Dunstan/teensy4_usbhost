/*
  Copyright (C) 2024 Andrew Dunstan
  This file is part of teensy4_usbhost.

  teensy4_usbhost is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _USB_MASS_STORAGE_FAT_H
#define _USB_MASS_STORAGE_FAT_H

#include "mass_storage.h"

#include <SdFat.h>
#if !defined(SD_FAT_TEENSY_MODIFIED)
#error "This USB Mass Storage library uses a custom modified copy of SdFat.  Standard SdFat was mistakenly used.  Arduino should print multiple libraries found for SdFat.h.  To resolve this error, you will need to move or delete the copy Arduino is using, or otherwise take steps to cause Teensy's special copy of SdFat to be used."
#endif
// Use FILE_READ & FILE_WRITE as defined by FS.h
#if defined(FILE_READ) && !defined(FS_H)
#undef FILE_READ
#endif
#if defined(FILE_WRITE) && !defined(FS_H)
#undef FILE_WRITE
#endif
#include <FS.h>


class USB_FAT_Volume : private FsBlockDeviceInterface, public FS {
private:
  USB_Storage* usbms = NULL;
  uint8_t lun;
  uint64_t sector_count;
  uint32_t sector_size;
  FsVolume FAT;
  uint8_t fat_type = 0;

  // FsBlockDeviceInterface methods
  bool isBusy() override;
  bool syncDevice() override;
  uint32_t sectorCount() override;
  bool readSectors(uint32_t sector, uint8_t *dst, size_t ns) override;
  bool readSector(uint32_t sector, uint8_t *dst) override { return readSectors(sector, dst, 1); }
  bool writeSectors(uint32_t sector, const uint8_t *src, size_t ns) override;
  bool writeSector(uint32_t sector, const uint8_t *src) override { return writeSectors(sector, src, 1); }

  static void TimeCB(uint16_t *date, uint16_t *time, uint8_t *ms10);

  // FS methods
public:
  File open(const char *filename, uint8_t mode = FILE_READ) override;
  bool exists(const char *filepath) override { return FAT.exists(filepath); }
  bool mkdir(const char *filepath) override { return FAT.mkdir(filepath); }
  bool rename(const char *oldfilepath, const char* newfilepath) override { return FAT.rename(oldfilepath, newfilepath); }
  bool remove(const char *filepath) override { return FAT.remove(filepath); }
  bool rmdir(const char *filepath) override { return FAT.rmdir(filepath); }
  uint64_t usedSize() override;
  uint64_t totalSize() override;
  bool mediaPresent() override;
  const char * name() override; // this currently returns the "product" name of the underlying USB mass storage device

  USB_FAT_Volume()  = default;
  ~USB_FAT_Volume() { unmount(); }
  // uncopyable
  USB_FAT_Volume(const USB_FAT_Volume&) = delete;
  USB_FAT_Volume& operator=(const USB_FAT_Volume&) = delete;

  static bool begin() { FsDateTime::setCallback(TimeCB); return true; };

  // mount a FAT partition on the given device and LUN, at the specified location
  bool mount(USB_Storage* usb, uint8_t LUN, uint32_t firstSector, uint32_t numSectors);

  // mount a FAT partition on the given device and LUN from a specific entry in the MBR
  // (0 = no MBR / whole disk is one partition)
  bool mount(USB_Storage* usb, uint8_t LUN, uint8_t part);

  // mount a FAT partition on the given device (and optional LUN) by trying each partition
  bool mount(USB_Storage* usb, uint8_t LUN=0);

  // search all USB devices (and LUNs) for a FAT partition to mount
  bool mount();

  void unmount();
  uint8_t type() const { return fat_type; }
};

#endif // _USB_MASS_STORAGE_FAT_H

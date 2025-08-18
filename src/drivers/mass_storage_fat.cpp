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

#include "mass_storage_fat.h"
#include <sys/time.h>

#define MAX_FILENAME_LEN 256

class USBFATFile : public FileImpl {
friend class USB_FAT_Volume;
  USBFATFile(const FsFile &file) : fsfile(file), filename(NULL) {}
  ~USBFATFile(void) { close(); free(filename); }
  size_t read(void *buf, size_t nbyte) { return fsfile.read(buf, nbyte); }
  size_t write(const void* buf, size_t size) { return fsfile.write(buf, size); }
  int available() { return fsfile.available(); }
  int peek() { return fsfile.peek(); }
  void flush() { fsfile.flush(); }
  bool truncate(uint64_t size=0) { return fsfile.truncate(size); }
  bool seek(uint64_t pos, int mode = SeekSet) {
    if (mode == SeekSet) return fsfile.seekSet(pos);
    if (mode == SeekCur) return fsfile.seekCur(pos);
    if (mode == SeekEnd) return fsfile.seekEnd(pos);
    return false;
  }
  uint64_t position() { return fsfile.curPosition(); }
  uint64_t size() { return fsfile.size(); }
  void close() {
    if (filename) filename[0] = '\0';
    if (fsfile.isOpen()) fsfile.close();
  }
  bool isOpen() { return fsfile.isOpen(); }
  const char* name() {
    if (filename == NULL) {
      filename = (char*)malloc(MAX_FILENAME_LEN);
      if (filename) {
        filename[0] = '\0';
        fsfile.getName(filename, MAX_FILENAME_LEN);
      }
      else
        return "";
    }
    return filename;
  }
  bool isDirectory() { return fsfile.isDirectory(); }
  File openNextFile(uint8_t mode=0) {
    auto file = fsfile.openNextFile();
    if (file) return File(new USBFATFile(file));
    return File();
  }
  void rewindDirectory(void) { fsfile.rewindDirectory(); }

  bool getCreateTime(DateTimeFields &tm) {
    uint16_t fat_date, fat_time;
    if (!fsfile.getCreateDateTime(&fat_date, &fat_time)) return false;
    if ((fat_date == 0) && (fat_time == 0)) return false;
    tm.sec = FS_SECOND(fat_time);
    tm.min = FS_MINUTE(fat_time);
    tm.hour = FS_HOUR(fat_time);
    tm.mday = FS_DAY(fat_date);
    tm.mon = FS_MONTH(fat_date) - 1;
    tm.year = FS_YEAR(fat_date) - 1900;
    return true;
  }

  bool getModifyTime(DateTimeFields &tm) {
    uint16_t fat_date, fat_time;
    if (!fsfile.getModifyDateTime(&fat_date, &fat_time)) return false;
    if ((fat_date == 0) && (fat_time == 0)) return false;
    tm.sec = FS_SECOND(fat_time);
    tm.min = FS_MINUTE(fat_time);
    tm.hour = FS_HOUR(fat_time);
    tm.mday = FS_DAY(fat_date);
    tm.mon = FS_MONTH(fat_date) - 1;
    tm.year = FS_YEAR(fat_date) - 1900;
    return true;
  }

  bool setCreateTime(const DateTimeFields &tm) {
    if (tm.year < 80 || tm.year > 207) return false;
    return fsfile.timestamp(T_CREATE, tm.year + 1900, tm.mon + 1,
      tm.mday, tm.hour, tm.min, tm.sec);
  }

  bool setModifyTime(const DateTimeFields &tm) {
    if (tm.year < 80 || tm.year > 207) return false;
    return fsfile.timestamp(T_WRITE, tm.year + 1900, tm.mon + 1,
      tm.mday, tm.hour, tm.min, tm.sec);
  }

private:
  FsFile fsfile;
  char *filename;
};

bool USB_FAT_Volume::isBusy() {
  return false;
}

bool USB_FAT_Volume::syncDevice() {
  return true;
}

uint32_t USB_FAT_Volume::sectorCount() {
  if (usbms == NULL) return 0;
  // limit max value returned to fit within the return type
  if (sector_count > 0xFFFFFFFFu)
    return 0xFFFFFFFF;
  return (uint32_t)sector_count;
}

bool USB_FAT_Volume::readSectors(uint32_t sector, uint8_t *dst, size_t ns) {
  if (usbms) {
    size_t length = ns * sector_size;
    return usbms->read(lun, sector, ns, dst, length) >= (int)length;
  }
  return false;
}

bool USB_FAT_Volume::writeSectors(uint32_t sector, const uint8_t *src, size_t ns) {
  if (usbms) {
    size_t length = ns * sector_size;
    return usbms->write(lun, sector, ns, src, length) >= (int)length;
  }
  return false;
}

void USB_FAT_Volume::TimeCB(uint16_t *date, uint16_t *time, uint8_t *ms10) {
  struct timeval tv;
  if (gettimeofday(&tv, NULL) == 0) {
    DateTimeFields dt;
    breakTime(tv.tv_sec, dt);
    if (dt.year >= 80) {
      *date = FS_DATE(dt.year + 1900, dt.mon + 1, dt.mday);
      *time = FS_TIME(dt.hour, dt.min, (dt.sec & ~1));
      *ms10 = (tv.tv_usec / 10000) + ((dt.sec & 1) ? 100 : 0);
      return;
    }
  }
  *date = 0;
  *time = 0;
  *ms10 = 0;
}

File USB_FAT_Volume::open(const char* filename, uint8_t mode) {
  oflag_t flags = O_READ;
  if (mode == FILE_WRITE) flags = O_RDWR | O_CREAT | O_AT_END;
  else if (mode == FILE_WRITE_BEGIN) flags = O_RDWR | O_CREAT;
  auto file = FAT.open(filename, flags);
  if (file) return File(new USBFATFile(file));
  return File();
}

uint64_t USB_FAT_Volume::usedSize() {
  if (mediaPresent()) {
    // bytesPerCluster may be broken for FAT16/FAT32 if the cluster size is 128 sectors (64KB)
    return (uint64_t)sector_size * FAT.sectorsPerCluster() * (FAT.clusterCount() - FAT.freeClusterCount());
  }
  return 0;
}

uint64_t USB_FAT_Volume::totalSize() {
  if (mediaPresent()) {
    return (uint64_t)sector_size * FAT.sectorsPerCluster() * FAT.clusterCount();
  }
  return 0;
}

bool USB_FAT_Volume::mediaPresent() {
  if (usbms && usbms->get_lun_count() > lun) {
    if (type() != 0)
      return true;
  }
  return false;
}

// maybe this should return the FAT volume label instead?
const char* USB_FAT_Volume::name() {
  if (mediaPresent()) {
    return usbms->product_name(lun);
  }

  return "USB Drive (Empty)";
}

bool USB_FAT_Volume::mount(USB_Storage* usb, uint8_t LUN, uint32_t firstSector, uint32_t numSectors) {
  unmount();
  if (numSectors == 0) return false;
  if (usb->lun_ready(LUN, sector_count, sector_size) >= 0) {
    if (sector_size == 512 && sector_count >= numSectors && (sector_count - numSectors) >= firstSector) {
      lun = LUN;
      usbms = usb->open();
      if (usbms != NULL) {
        if (FAT.begin(this, true, firstSector, numSectors)) {
          fat_type = FAT.fatType();
          return true;
        }
        usbms->close();
        usbms = NULL;
      }
    }
  }
  return false;
}

bool USB_FAT_Volume::mount(USB_Storage* usb, uint8_t LUN, uint8_t part) {
  unmount();
  if (usb->lun_ready(LUN, sector_count, sector_size) >= 0) {
    if (sector_count > 0 && sector_size == 512) {
      lun = LUN;
      usbms = usb->open();
      if (usbms != NULL) {
        if (FAT.begin(this, true, part)) {
          fat_type = FAT.fatType();
          return true;
        }
        usbms->close();
        usbms = NULL;
      }
    }
  }
  return false;
}

bool USB_FAT_Volume::mount(USB_Storage* usb, uint8_t LUN) {
  /* Check if the LUN is ready first. Otherwise the first
   * few partitions may fail and a later one succeed even
   * though the earlier partition(s) are usable.
   */
  if (usb->lun_ready(LUN) < 0) {
    // some USB drives may never initialize if mount() is called non-stop
    atomTimerDelay(SYSTEM_TICKS_PER_SEC * 50 / 1000);
    return false;
  }

  return mount(usb, LUN, 1) || \
    mount(usb, LUN, 0) || \
    mount(usb, LUN, 2) || \
    mount(usb, LUN, 3) || \
    mount(usb, LUN, 4);
}

bool USB_FAT_Volume::mount() {
  USB_Storage* usb;
  size_t i=0;
  while ((usb = USB_Storage::open_device(i++)) != NULL) {
    for (uint8_t j=0; j < usb->get_lun_count(); j++) {
      if (mount(usb, j)) {
        usb->close();
        return true;
      }
    }
    usb->close();
  }
  return false;
}

void USB_FAT_Volume::unmount() {
  FAT.end();
  fat_type = 0;
  if (usbms) {
    usbms->close();
    usbms = NULL;
  }
}

/* advanced.ino: examines all LUNs on all connected USB Mass Storage devices
 * and reports any identified partitions. Any FAT partitions will be mounted
 * and have their directory contents displayed for verification.
 */

#include <vector>
#include <teensy4_usb.h>
#include <usb_util.h>

#define USE_MASS_STORAGE_FAT
#include <usb_drivers.h>

DMAMEM TeensyUSBHost2 usb;

// partition types that we know about / can identify
typedef enum partition_type {
  PARTITION_FAT16=16,
  PARTITION_FAT32=32,
  PARTITION_EXFAT=64,
  PARTITION_NTFS=100,
  PARTITION_ISO9660,
} partition_type;

typedef struct partition_entry {
  partition_type type;
  uint64_t lba_start;
  uint64_t lba_end;
} partition_entry;

typedef union partition_sector {
  uint8_t data[4096]; // maximum sector size
  struct
  {
    uint8_t jmp[3];                 // 0
    uint8_t OEMName[8];             // 3
    uint16_t bytes_per_sector;      // B
    uint8_t sectors_per_cluster;    // D
    uint16_t reserved_sectors;      // E
    uint8_t fat_count;              // 10
    uint16_t root_entries;          // 11
    uint16_t small_sector_count;    // 13
    uint8_t media_description;      // 15
    uint16_t sectors_per_fat;       // 16
    uint16_t sectors_per_track;     // 18
    uint16_t head_count;            // 1A
    uint32_t hidden_sectors;        // 1C
    uint32_t sector_count;          // 20
    union {
      struct fat16epb {
        uint8_t drive_number;       // 24
        uint8_t reserved1;          // 25
        uint8_t ext_boot_sig;       // 26
        uint32_t volume_id;         // 27
        uint8_t volume_label[11];   // 2B
        uint8_t file_system_type[8];// 36
      } __attribute__((packed)) fat16epb;
      struct {
        uint32_t sectors_per_fat;   // 24
        uint16_t flags;             // 28
        uint16_t version;           // 2A
        uint32_t root_cluster;      // 2C
        uint16_t fsinfo_sector;     // 30
        uint16_t boot_sector;       // 32
        uint8_t reserved[12];       // 34
//        struct fat16ebp fat16;
        uint8_t drive_number;       // 40
        uint8_t reserved1;          // 41
        uint8_t ext_boot_sig;       // 42
        uint32_t volume_id;         // 43
        uint8_t volume_label[11];   // 47
        uint8_t file_system_type[8];// 52
      } __attribute__((packed)) fat32epb;
    } epb;
    uint8_t boot_code[420];         // 5A
    uint8_t boot_signature[2];      // 1FE
  } __attribute__((packed)) bpb;
  struct {
    uint8_t code[446];              // 0
    struct {                        // 1BE
      uint8_t status;
      uint8_t chs_start[3];
      uint8_t partition_type;
      uint8_t chs_end[3];
      uint32_t lba_start;
      uint32_t lba_count;
    } __attribute__((packed)) partition[4];
  } mbr;
  struct {
    uint8_t signature[8];
    uint32_t revision;
    uint32_t header_size;
    uint32_t header_crc;
    uint32_t zero;
    uint64_t current_lba;
    uint64_t backup_lba;
    uint64_t first_lba;
    uint64_t last_lba;
    uint8_t disk_guid[16];
    uint64_t part_lba;
    uint32_t part_count;
    uint32_t part_size;
    uint32_t part_crc;
  } __attribute__((packed)) efi;
  struct {
    uint8_t type_guid[16];
    uint8_t unique_guid[16];
    uint64_t lba_start;
    uint64_t lba_end;
    uint64_t flags;
    uint8_t name[72];
  /* GCC accepts 0 length flexible arrays in non-C99 unions, but for C99 complains about array bounds.
   * As of recently (April 2024) Clang has added a langugage extension but GNU still drags their feet...
   * so length is hardcoded here, it will be handled correctly in code to account for actual sector size
   */
  } __attribute__((packed)) efi_part[32];
} partition_sector;

PGM_P partition_type_to_string(int type) {
  switch (type) {
    case PARTITION_FAT16:
      return PSTR("FAT16");
    case PARTITION_FAT32:
      return PSTR("FAT32");
    case PARTITION_EXFAT:
      return PSTR("EXFAT");
    case PARTITION_NTFS:
      return PSTR("NTFS");
    case PARTITION_ISO9660:
      return PSTR("ISO9660");
  }
  return PSTR("");
}

void find_partitions(class USB_Storage* const bulk, const uint8_t lun, const uint64_t lba_count, const uint32_t lba_size, uint64_t lba, std::vector<struct partition_entry>& part_list) {
  partition_sector sec;
  static const PROGMEM uint8_t GUID_BASIC_DATA[16] = { 0xA2, 0xA0, 0xD0, 0xEB, 0xE5, 0xB9, 0x33, 0x44, 0x87, 0xC0, 0x68, 0xB6, 0xB7, 0x26, 0x99, 0xC7 };

  if (lba_size > sizeof(sec)) return;

  if (lba_size == 2048) {
    if (lba == 0) {
      for (uint32_t i=16; i < 32; i++) {
        if (bulk->read(lun, i, 1, &sec, 2048) != 2048)
          break;
        if (memcmp(sec.data+1, PSTR("CD001\1"), 6)==0) {
          if (sec.data[0] == 1) {
            uint32_t end = *(uint32_t*)(sec.data+0x50);
            if (end == 0)
              end = lba_count;
            part_list.push_back({PARTITION_ISO9660, 0, end-1});
            break;
          }
          if (sec.data[0] == 0xFF)
            break;
        }
      }
    }
    return;
  }

  if (bulk->read(lun, lba, 1, &sec, lba_size) == (int)lba_size) {
    // GPT/EFI: only supports partition entries that are 128 bytes long
    if (memcmp(sec.efi.signature, PSTR("EFI PART"), 8)==0 && sec.efi.part_size==sizeof(sec.efi_part[0])) {
      // GPT uses absolute LBAs, re-read disk capacity since lba_count may not be the entire disk
      uint64_t abs_count;
      uint32_t s;
      if (bulk->read_capacity(lun, abs_count, s) == false)
        return;

      uint32_t part_count = sec.efi.part_count;
      uint64_t part_lba = sec.efi.part_lba;
      const uint32_t parts_per_sector = lba_size / sizeof(sec.efi_part[0]); // 4 for 512 byte sectors, 32 for 4096 byte sectors
      uint32_t i = parts_per_sector;
      Serial.printf("Found an EFI partition table, %u partition entries starting at LBA %llu\n", part_count, part_lba);
      while (part_count-- > 0) {
        // check if the next sector of the GPT needs to be fetched
        if (i==parts_per_sector) {
          if (part_lba >= abs_count || bulk->read(lun, part_lba, 1, &sec, lba_size) != (int)lba_size)
            return;
          part_lba++;
          i = 0;
        }
        // partition must have BASIC_DATA GUID, lba_start before lba_end and lba_end before end of media
        if (memcmp(sec.efi_part[i].type_guid, GUID_BASIC_DATA, 8)==0 && \
            sec.efi_part[i].lba_start < sec.efi_part[i].lba_end && \
            sec.efi_part[i].lba_end < abs_count) {
              find_partitions(bulk, lun, sec.efi_part[i].lba_end-sec.efi_part[i].lba_start+1, lba_size, sec.efi_part[i].lba_start, part_list);
        }
        i++;
      }
      return;
    }
    // else not a GPT header - must have MBR/BPB signature intact
    if (sec.bpb.boot_signature[0]!=0x55 || sec.bpb.boot_signature[1]!=0xAA)
      return;

    // TODO: perform more sanity checks on EXFAT
    if (memcmp(sec.bpb.OEMName, PSTR("EXFAT   "), 8) == 0) {
      part_list.push_back({PARTITION_EXFAT, lba, lba+lba_count-1});
      return;
    }

    // FAT and NTFS bytesPerSector must match hardware sector size
    if (sec.bpb.bytes_per_sector == lba_size) {
      // TODO: perform more sanity checks on NTFS
      if (memcmp(sec.bpb.OEMName, PSTR("NTFS    "), 8) == 0) {
        part_list.push_back({PARTITION_NTFS, lba, lba+lba_count-1});
        return;
      }

      if (!memcmp(sec.bpb.epb.fat32epb.file_system_type, PSTR("FAT32   "), 8) || \
          !memcmp(sec.bpb.epb.fat16epb.file_system_type, PSTR("FAT16   "), 8)) {
        uint64_t count = sec.bpb.small_sector_count;
        if (count == 0)
          count = sec.bpb.sector_count;
        if (count != 0 && count <= lba_count)
          part_list.push_back({sec.bpb.sectors_per_fat ? PARTITION_FAT16 : PARTITION_FAT32, lba, lba+count-1});
        return;
      }
    }


    // if we've got to here: assume this is either the MBR or an extended partition
    for (size_t i=0; i < 4; i++) {
      // most ignore the status byte, partition type and CHS info because it's untrustworthy / irrelevant
      uint64_t start = sec.mbr.partition[i].lba_start;
      uint64_t count = sec.mbr.partition[i].lba_count;
      // GPT entry sometimes uses 0xFFFFFFFF for count instead of the actual disk size
      if (i==0 && sec.mbr.partition[0].partition_type==0xEE && count==0xFFFFFFFF)
        count = lba_count - start;
      Serial.printf("Partition %llu:%u: start %llu, count %llu, type %02X\n", lba, i, start, count, sec.mbr.partition[i].partition_type);
      if (count > 0 && start < lba_count && count <= (lba_count - start)) {
        // this looks valid, check it
        find_partitions(bulk, lun, count, lba_size, start+lba, part_list);
        // if first entry of the partition table was a GPT and partitions were found, we are done
        if (lba==0 && i==0 && sec.mbr.partition[0].partition_type==0xEE && part_list.empty()==false)
          return;
      }
    }
  }
}

// returns true if "complete", false means retry
bool test_usb_lun(USB_Storage* usb, uint8_t lun) {
  uint64_t sector_count;
  uint32_t sector_size;

  int result = usb->lun_ready(lun, sector_count, sector_size);
  if (result < 0) {
    Serial.printf("USB %p:%u was not ready, errno %d\n", usb, lun, errno);
    return (errno != EAGAIN); // can retry for EAGAIN
  }
  else if (sector_count == 0) {
    Serial.printf("USB %p:%u drive is empty\n", usb, lun);
    errno = 0;
    return false;
  }

  std::vector<partition_entry> partitions;
  Serial.printf("USB %p:%u LBA count: %llu, LBA size: %lu\n", usb, lun, sector_count, sector_size);
  find_partitions(usb, lun, sector_count, sector_size, 0, partitions);
  Serial.printf("Found %u partitions\n", partitions.size());
  for (size_t i=0; i < partitions.size(); i++) {
    const partition_entry *p = &partitions[i];
    const char *part_name = partition_type_to_string(p->type);
    Serial.printf("USB %p:%u Partition %u: Type %s, start %llu - end %llu\n", usb, lun, i, part_name, p->lba_start, p->lba_end);
    if (p->type <= PARTITION_EXFAT) {
      USB_FAT_Volume vol;
      if (vol.mount(usb, lun, p->lba_start, p->lba_end - p->lba_start + 1)) {
        Serial.printf("Successfully mounted %s partition\n", part_name);
        Serial.println("Calculating free space on partition...");
        elapsedMillis start;
        Serial.printf("filesystem usage: %llu out of %llu bytes (time: %lums)\n", vol.usedSize(), vol.totalSize(), (long)start);

        File root = vol.open("/");
        if (root && root.isDirectory()) {
          Serial.println("--- FILE LISTING BEGIN ---");
          recurse_dir_print("/", root, Serial);
          Serial.println("--- FILE LISTING COMPLETE ---");
        }
        else Serial.println("Unable to open root directory");
      } else {
        Serial.printf("Mounting %s partition failed\n", part_name);
      }
      Serial.println();
    }
  }
  return true;
}

void setup() {
  Serial.begin(0);
  elapsedMillis stimer;
  do {
    if (Serial) break;
  } while (stimer < 5000);

  if (CrashReport) CrashReport.printTo(Serial);

  usb.begin();
  USB_FAT_Volume::begin();
  delay(1000);
  Serial.println("\n\nPress return to search for a USB drive");
}

void loop() {
  if (Serial.read() == '\n') {
    size_t count = USB_Storage::get_device_count();
    Serial.printf("Found %u USB Storage devices\n", count);
    for (size_t dev_index = 0; dev_index < count; dev_index++) {
      USB_Storage* usbms = USB_Storage::open_device(dev_index);
      if (usbms) {
        Serial.printf("Found a USB Mass Storage device %p, has %u logical units\n", usbms, usbms->get_lun_count());
        for (uint8_t lun=0; lun < usbms->get_lun_count(); lun++) {
          String vendor(usbms->vendor_name(lun));
          String product(usbms->product_name(lun));
          Serial.printf("USB %p:%u Vendor: %s, Product: %s\n", usbms, lun, vendor.c_str(), product.c_str());
          elapsedMillis retry_timer = 0;
          do {
            if (retry_timer >= 100) Serial.println("Retrying...");
            if (test_usb_lun(usbms, lun))
              break;
            delay(errno == EAGAIN ? 100 : 5000);
          } while (retry_timer < 30000);
        }
        usbms->close();
      }
    }

    delay(1000);
    Serial.println("\nPress return to search again");
  }
}

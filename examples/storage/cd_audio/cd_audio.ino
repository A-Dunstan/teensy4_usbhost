#include <SD.h>
#include <Audio.h>

#include <teensy4_usb.h>
#include <cerrno>
#include <cstdlib>
#include <malloc.h>

#define USE_MASS_STORAGE
#include "usb_drivers.h"

#define SCSI_MODE_SENSE_6      0x1A
#define SCSI_START_STOP        0x1B
#define SCSI_READ_SUBCHANNEL   0x42
#define SCSI_READ_TOC          0x43
#define SCSI_STOP_PLAYSCAN     0x4E
#define SCSI_MODE_SENSE_10     0x5A
#define SCSI_PLAY_AUDIO_12     0xA5

static DMAMEM TeensyUSBHost2 usb;
static USB_Storage* usb_cd;
static uint8_t CD_LUN;

/* route SPDIF in to USB audio out (set Teensy USB type to "Serial + MIDI + Audio")
 * CDROM SPDIF IS 5V TTL - DO NOT CONNECT DIRECTLY TO 3.3V TEENSY
 * connect CDROM SPDIF data to diode cathode (e.g. 1N9148), connect anode to Teensy pin 15 (SPDIF in)
 * SPDIF data on the CDROM is the outer pin, GND is the pin closer to the center
 */
AsyncAudioInputSPDIF3 spdifIn;
AudioOutputUSB usbAudioOut;
AudioConnection patchCord1(spdifIn, 0, usbAudioOut, 0);
AudioConnection patchCode2(spdifIn, 1, usbAudioOut, 1);

static void prompt(void) {
    Serial.print("\nEnter command (? for available commands): ");
}

void setup() {
  // configure pin 15 (SPDIF IN) for PULLUP
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
  AudioMemory(12);

  Serial.begin(0);
  elapsedMillis stimer;
  do {
    if (Serial) break;
  } while (stimer < 5000);

  if (CrashReport) CrashReport.printTo(Serial);

  usb.begin();
  delay(1000);

  prompt();
}

static uint8_t check_audio_status(void) {
  struct {
    uint8_t res0;
    uint8_t audio_status;
    uint16_t subchannel_data_length;
    // current position data block
    uint8_t subchannel_data_format; // 1 for current position
    uint8_t control:4;
    uint8_t ADR:4;
    uint8_t track;
    uint8_t index;
    uint8_t absolute_lba[4];
    uint8_t track_lba[4];
  } sub;
  uint8_t cmd[10] = {SCSI_READ_SUBCHANNEL, 0x2,0x40,1,0,0,0,0,sizeof(sub)};

  if (usb_cd->scsi_cmd(CD_LUN, &sub, sizeof(sub), cmd, false) < 16) {
    Serial.printf("Failed to retrieve subchannel header: %d\n", errno);
    return 0;
  }
  Serial.printf("Current audio status: %02X\n", sub.audio_status);
  Serial.printf("ADR: %02X, Control: %02X\n", sub.ADR, sub.control);
  Serial.printf("Track: %d, Index: %d\n", sub.track, sub.index);
  Serial.printf(" Disc Time: %02u:%02u.%02u\n", sub.absolute_lba[1], sub.absolute_lba[2], sub.absolute_lba[3]);
  Serial.printf("Track Time: %02u:%02u.%02u\n", sub.track_lba[1], sub.track_lba[2], sub.track_lba[3]);
  return sub.audio_status;
}

static bool usb_supports_cdda(USB_Storage *usb, uint8_t lun) {
  // get all mode pages, parse for CD Audio Control
  uint8_t mode_sense_length[2];
  uint8_t cmd[10] = {SCSI_MODE_SENSE_10, 0, 0x3F,0, 0,0,0, 0,sizeof(mode_sense_length)};

  int len = usb->scsi_cmd(lun, mode_sense_length, sizeof(mode_sense_length), cmd, false);
  if (len >= 2) {
    len = 2 + (mode_sense_length[0]<<8) + mode_sense_length[1];    
  }
  if (len <= 8) {
    Serial.printf("MODE_SENSE command failed: %d(%d)\n", len, errno);
    return false;
  }

  Serial.printf("MODE SENSE full data size: %u\n", len);
  uint8_t *pg = (uint8_t*)aligned_alloc(32, len+31);
  if (pg) {
    cmd[7] = (uint8_t)(len >> 8);
    cmd[8] = (uint8_t)(len >> 0);
    len = usb->scsi_cmd(lun, pg, len, cmd, false);
    if (len >= 8) {
      Serial.printf("Got full MODE_SENSE data (%d)\n", len);
      // media type may not be accurate - drive may be empty
      Serial.printf("Media Type: %02X\n", pg[2]);
      uint8_t *mode_page = pg+6 + (pg[6]<<8) + pg[7];
      uint8_t *end = pg + len-1;
      while (mode_page < end) {
        if (mode_page[0] == 0xE && mode_page[1] >= 0xE) {
          Serial.println("Found CD-ROM Audio Control Parameters Mode Page");
          free(pg);
          return true;
        }
        if (mode_page[0] & 0x40) { // sub-page
          if (mode_page + 2 >= end) break;
          mode_page += (mode_page[2]<<8) + mode_page[3] + 4;
        } else {
          mode_page += mode_page[1]+2;
        }
      }
      Serial.println("Failed to find CD-ROM Audio Control Parameters Mode Page");
    }
    else Serial.printf("Failed to get mode sense data: %d(%d)\n", len, errno);
    free(pg);
  }
  else Serial.println("Failed to allocate data for mode sense");
  return false;
}

static void find_cd(void) {
  USB_Storage *usb;
  size_t i = 0;
  while ((usb = USB_Storage::open_device(i++)) != NULL) {
    for (uint8_t j=0; j < usb->get_lun_count(); j++) {
      Serial.printf("Attempting to mount USB %p:%u...\n", usb, j);
      uint64_t count;
      uint32_t size;
      // don't care if drive is empty so don't check the sector count
      if (usb->lun_ready(j, count, size) < 0) {
        Serial.printf("USB %p:%u not ready (%d)\n", usb, j, errno);
        continue;
      }

      if (usb_supports_cdda(usb, j)) {
        usb_cd = usb;
        CD_LUN = j;
        Serial.printf("USB drive %p:%u seems to support CD Audio playback\n", usb_cd, CD_LUN);
        return;
      }
      Serial.printf("USB %p:%u does not support CD Audio\n", usb, j);
    }
    usb->close();
  }
  Serial.println("Failed to find a suitable USB CD drive");
}

static void release_cd(void) {
  if (usb_cd) {
    usb_cd->close();
    usb_cd = NULL;
  }
  Serial.println("CD-ROM was released");
}

// returns starting LBA of the specified track
static uint32_t read_toc(uint8_t track) {
  // some drives are buggy when returning portions of the TOC (non-zero starting track)
  // get the whole thing instead and find the requested track to return the starting LBA
  struct {
    uint16_t length;
    uint8_t first_track;
    uint8_t last_track;
    struct {
      uint8_t res0;
      uint8_t control:4; uint8_t adr:4;
      uint8_t track_number;
      uint8_t res1;
      uint32_t address;
    } track[101]; // 100 tracks maximum + leadout
  } toc;
  uint8_t cmd[10] = {SCSI_READ_TOC, 0,0,0,0,0,0, 0,2};

  if (usb_cd->scsi_cmd(CD_LUN, &toc, 2, cmd, false) < 2)
    Serial.printf("Failed to get TOC length: %d\n", errno);
  else {
    uint32_t length = __builtin_bswap16(toc.length)+2;
    Serial.printf("Full TOC length: %u\n", length);
    if (length > sizeof(toc)) length = sizeof(toc);
    Serial.printf("First track: %u, Last track: %u\n", toc.first_track, toc.last_track);
    if (track > toc.last_track) track = 0xAA; // match leadout instead
    cmd[7] = (uint8_t)(length >> 8);
    cmd[8] = (uint8_t)(length >> 0);
    if (usb_cd->scsi_cmd(CD_LUN, &toc, length, cmd, false) < (int)length) {
      Serial.printf("Failed to get TOC data: %d\n", errno);
    } else {
      length -= 4;
      for (int i=0; i<=100 && length>=8; i++,length -= 8) {
        uint32_t address = __builtin_bswap32(toc.track[i].address);
        Serial.printf("Track %d: ADR %X, Control %X, LBA: %u\n", toc.track[i].track_number, \
          toc.track[i].adr, toc.track[i].control, address);
        if (track == toc.track[i].track_number) return address;
      }
    }
  }

  return 0;
}

static void stop_play(void) {
  uint8_t cmd[10] = {SCSI_STOP_PLAYSCAN};
  if (usb_cd->scsi_cmd(CD_LUN, NULL, 0, cmd, true) >= 0) {
    Serial.println("Audio playback was stopped");
  } else {
    Serial.printf("Audio stop command failed (%d)\n", errno);
  }
}

static void play_audio(uint32_t begin, uint32_t end) {
  uint8_t cmd[12] = {SCSI_PLAY_AUDIO_12};

  uint32_t length = end - begin + 1;
  cmd[2] = (uint8_t)((begin >> 24) & 0xFF);
  cmd[3] = (uint8_t)((begin >> 16) & 0xFF);
  cmd[4] = (uint8_t)((begin >>  8) & 0xFF);
  cmd[5] = (uint8_t)((begin >>  0) & 0xFF);
  cmd[6] = (uint8_t)((length >> 24) & 0xFF);
  cmd[7] = (uint8_t)((length >> 16) & 0xFF);
  cmd[8] = (uint8_t)((length >>  8) & 0xFF);
  cmd[9] = (uint8_t)((length >>  0) & 0xFF);

  int ret = usb_cd->scsi_cmd(CD_LUN, NULL, 0, cmd, true);
  if (ret < 0)
    Serial.printf("play command: %u -> %u Failed: %d\n", begin, end, errno);
  else
    Serial.printf("play command: %u -> %u Success\n", begin, end);
}

static void play_disc(uint8_t track) {
  uint32_t start = read_toc(track);
  uint32_t end = read_toc(0xAA); // leadout

  if (end <= start) {
    Serial.printf("Failed to find disc limits: %u %u\n", start, end);
    return;
  }

  play_audio(start, end-1);
}

static void play_track(uint32_t track) {
  uint32_t start = read_toc(track);
  uint32_t end = read_toc(track+1);

  if (end <= start) {
    Serial.printf("Failed to find track %d limits: %u %u\n", track, start, end);
    return;
  }

  play_audio(start, end-1);
}

static void start_stop(bool load) {
  uint8_t cmd[6] = {SCSI_START_STOP, 0,0,0,2}; // loej flag set, no immediate return

  if (load) cmd[4] |= 1; // start bit
  if (usb_cd->scsi_cmd(CD_LUN, NULL, 0, cmd, false) < 0) {
    Serial.printf("start_stop(%d) command failed: %d\n", load, errno);
    return;
  }

  /* if a new disc is loaded wait for drive to become ready
   * before returning, otherwise commands may fail
   */
  if (load) {
    Serial.print("Waiting for drive to become ready...");
    for (int retry=0; retry < 60; retry++) {
      uint64_t count;
      uint32_t size;
      int r = usb_cd->lun_ready(CD_LUN, count, size);
      if (r>=0 && count>0) {
        Serial.println("Unit is ready");
        break;
      }
      // drive was empty or disc isn't usable
      if (r>=0 || errno != EAGAIN) {
        Serial.println("Unit is not ready");
        break;
      }
      // else drive is still initializing
      Serial.print(".");
      delay(1000);
    }
  }
}

static void mem_stats(void) {
  struct mallinfo mi = mallinfo();
  Serial.printf("Memory: %lu free / %lu used of %lu allocated arena\n", mi.fordblks+mi.fsmblks, mi.uordblks, mi.arena);
}

static void show_help(void) {
  Serial.println("\"a\" - show current audio status");
  if (usb_cd == NULL) {
    Serial.println("\"m\" - activate an attached CD drive");
  } else {
    Serial.println("\"c\" - display disc TOC (table of contents)");
    Serial.println("\"d(#)\" - play disc, optionally starting from track #");
    Serial.println("\"i\" - closes the drive");
    Serial.println("\"o\" - ejects the drive");
    Serial.println("\"p#\" - play audio track number #");
    Serial.println("\"s\" - stop audio playback");
    Serial.println("\"u\" - release CD drive");
  }
  Serial.println("\"z\" - show malloc statistics");
  Serial.println("\"?\" - show currently available commands");
}

void loop() {
  int c = Serial.read();
  if (c == -1 || c=='\n') return;

  Serial.println((char)c);

  switch (c) {
    case 'a':
      Serial.printf("SPDIF Samplerate: %f (%s)\n", spdifIn.getInputFrequency(), spdifIn.isLocked() ? "LOCKED" : "UNLOCKED");
      break;
    case 'z':
      mem_stats();
      break;
  }

  if (usb_cd == NULL) {
    switch (c) {
      case 'm':
        find_cd();
        break;
      default:
        show_help();
      case 'a':
      case 'z':
        break;
    }
  } else {
    switch(c) {
      case 'a':
        check_audio_status();
        break;
      case 'c':
        read_toc(0);
        break;
      case 'd':
        play_disc(Serial.parseInt());
        break;
      case 'i':
        start_stop(1);
        break;
      case 'o':
        start_stop(0);
        break;
      case 'p':
        play_track(Serial.parseInt());
        break;
      case 's':
        stop_play();
        break;
      case 'u':
        release_cd();
        break;
      default:
        show_help();
      case 'z':
        break;
    }
  }

  prompt();
}

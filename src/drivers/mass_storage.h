#ifndef _USB_MASS_STORAGE_H
#define _USB_MASS_STORAGE_H

#include <teensy4_usbhost.h>
#include <atomic>
#include <list>

typedef struct __attribute__((packed)) ms_cbw {
  uint32_t dCBWSignature;
  uint32_t dCBWTag;
  uint32_t dCBWDataTransferLength;
  uint8_t bmCBWFlags;
  uint8_t bCBWLUN;
  uint8_t bCBWCBLength;
  uint8_t CBWCB[16];
} ms_cbw;

typedef struct __attribute__((packed)) ms_csw {
  uint32_t dCSWSignature;
  uint32_t dCSWTag;
  uint32_t dCSWDataResidue;
  uint8_t bCSWStatus;
} ms_csw;

class USB_Storage : public USB_Driver_FactoryGlue<USB_Storage> {
private:
  union __attribute__((aligned(32))) {
    uint8_t buf[32];
    ms_cbw cbw;
    ms_csw csw;
  } xfer;
  const uint8_t interface;
  const uint8_t bulk_in;
  const uint8_t bulk_out;

  volatile uint8_t lun_count;
  uint32_t tag;

  std::atomic<unsigned int> ref;
  void addref(void) { ref.fetch_add(1, std::memory_order_relaxed);}
  void deref(void);

  void detach(void);
  USB_Storage(USB_Device*, uint8_t _interface, uint8_t _bulk_in, uint8_t _bulk_out);
  ~USB_Storage();
  /* List of all attached USB mass storage devices. This may be accessed by different
   * threads so it needs to be guarded.
   */
  static std::list<class USB_Storage*> devices;

  /* utility class for a mutex. autolock() returns an object that holds the mutex for
   * as long as it exists.
   */
  class mutex_cxx {
  private:
    ATOM_MUTEX mtx;
  public:
    mutex_cxx() { atomMutexCreate(&mtx); }
    ~mutex_cxx() { atomMutexDelete(&mtx); }

    class auto_lock {
    private:
      const uint8_t locked;
      ATOM_MUTEX *lck;
    public:
      auto_lock(ATOM_MUTEX *g) : locked(atomMutexGet(g, SYSTEM_TICKS_PER_SEC*5)), lck(g) {}
      ~auto_lock() { if (locked == ATOM_OK) atomMutexPut(lck); }
      operator bool() { return locked == ATOM_OK; }
    };
    class auto_lock autolock(void) { return auto_lock(&mtx); }
  };
  // actual mutex object that guards access to the device list
  static mutex_cxx list_lock;
  mutex_cxx cmd_lock;

  // performs USB mass storage reset - this affects *all* LUNs on the device
  void reset();
public:
  // class instance factory methods, accessed by USB Host when a new device is inserted
  static bool offer_interface(const usb_interface_descriptor*,size_t);
  static USB_Driver* attach_interface(const usb_interface_descriptor*,size_t,USB_Device*);

  // return number of attached USB Storage devices
  static size_t get_device_count(void) {
    auto lck = list_lock.autolock();
    return devices.size();
  }
  // return a handle to a USB Storage device
  static USB_Storage* open_device(size_t index);
  // return a new handle for an open device
  USB_Storage* open(void);
  // close a handle to an open device
  void close(void) { deref(); }

  /* process a SCSI command (CDB), wrapping it with CBW/CSW transfers
   * Returns the number of bytes transferred (>= 0) on success or -1 on failure
   * errno should give an indication of the error encountered:
   * ENODEV = USB device was disconnected from host
   * EPIPE,EOVERFLOW,EPROTO = USB protocol errors
   * ENXIO = invalid LUN specified
   * EINVAL = invalid argument / unknown SCSI command group
   * EBUSY = SCSI command failed or phase error
   */
  int scsi_cmd(uint8_t lun, void* data, size_t length, const uint8_t* command, bool host_to_device);

  // basic wrappers for common SCSI commands (used by USB storage). In some cases these decide
  // which specific command to use based on the arguments.

  // return number of LUNs attached to device
  uint8_t get_lun_count(void) const { return lun_count; }
  // test if the specified LUN is ready to receive commands
  int test_unit_ready(uint8_t lun);
  // get the number of sectors and sector size for the specified LUN
  bool read_capacity(uint8_t lun, uint64_t& sectors, uint32_t& sector_size);
  // attempt to fetch $length bytes of inquiry data for the specified LUN
  int inquiry(uint8_t lun, void* dst, uint16_t length);
  // read $count sectors starting at sector $lba for the specified LUN
  int read(uint8_t lun, uint64_t lba, uint32_t count, void* dst, size_t length);
  // write $count sectors starting at sector $lba for the specified LUN
  int write(uint8_t lun, uint64_t lba, uint32_t count, const void *src, size_t length);
  // request $length amount of sense data from the specified LUN. $desc determines if
  // the returned data is fixed format or descriptor format
  int request_sense(uint8_t lun, void* dst, uint8_t length, bool desc=false);

  /* convenience function: check if a LUN is ready, retrieve the sector count and size
   * Returns -1 on failure (check errno) and >= 0 on success
   * NOTE: a device may return FAILURE with errno==EAGAIN, this means it is likely in the process
   * of becoming ready. This is distinct from returning SUCCESS with sector_count==0, which means
   * the device is "ready" but currently empty (e.g. CD drive with no disc).
   */
  int lun_ready(uint8_t lun, uint64_t& sector_count, uint32_t& sector_size);

  // simplified version, no parameters besides LUN
  int lun_ready(uint8_t lun) {
    uint64_t count;
    uint32_t size;
    int r = lun_ready(lun, count, size);
    return (count == 0) ? -1 : r;
  }

  /* these functions return "volatile" strings - they will be overwritten by
   * future calls to the device, so copy them if you want to keep them
   */
  // perform inquiry and return the vendor name
  const char* vendor_name(uint8_t lun);
  // perform inquiry and return the product name
  const char* product_name(uint8_t lun);
};

typedef struct basic_inquiry_response {
  uint8_t peripheral_device_type:5;
  uint8_t peripheral_qualifier:3;

  uint8_t :7; // reserved
  uint8_t removable:1;

  uint8_t version;

  uint8_t response_data_format:4;
  uint8_t hisup:1;
  uint8_t normaca:1;
  uint8_t :2; // obsolete

  uint8_t additional_length;

  uint8_t protect:1;
  uint8_t :2; // reserved
  uint8_t third_pc:1;
  uint8_t tpgs:1;
  uint8_t acc:1;
  uint8_t sccs:1;

  uint8_t :4; // obsolete
  uint8_t multip:1;
  uint8_t :1; // vendor-specific
  uint8_t encserv:1;
  uint8_t :1; // obsolete

  uint8_t :1; // vendor-specific
  uint8_t cmdque:1;
  uint8_t :6; // obsolete

  uint8_t T10_vendor[8];
  uint8_t product_id[16];
  uint8_t product_rev[4];
} basic_inquiry_response;

typedef struct basic_sense_data {
  uint8_t response_code:7;
  uint8_t valid:1;

  uint8_t :8; // obsolete

  uint8_t sense_key:4;
  uint8_t :1; // reserved
  uint8_t ili:1;
  uint8_t eom:1;
  uint8_t filemark:1;

  uint8_t information[4];
  uint8_t additional_sense_length;
  uint8_t command_specific_information[4];
  uint8_t additional_sense_code;
  uint8_t additional_sense_code_qualifier;
  uint8_t field_replaceable_unit_code;
  uint8_t sense_key_specific[3];
} basic_sense_data;

#endif // _USB_MASS_STORAGE_H

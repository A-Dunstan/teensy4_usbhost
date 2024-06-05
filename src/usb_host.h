#ifndef _USB_HOST
#define _USB_HOST

#include <map>
#include <vector>
#include <string>
#include <atomic>
#include <functional>
#include <pgmspace.h>

#include "usb_spec.h"

#if 1
#define dprintf(fmt, ...) ::printf(PSTR(fmt) __VA_OPT__(,) __VA_ARGS__)
#else
#define dprintf(...)
#endif

#define PERIODIC_LIST_SIZE 32
// language used for descriptor strings - nearly all devices only support US language
#define USB_LANG 0x0409

// any global definition of PI (3.1415...) should be M_PI, get rid of it
#undef PI

typedef std::function<void(int)> USBCallback;
typedef int16_t isolength[8];

template <class CTransfer>
class CCallback {
public:
  virtual void callback(const CTransfer*,int) = 0;
  virtual ~CCallback() {}
};

typedef struct {
  const uint8_t CAPLENGTH;
  const uint16_t HCIVERSION;
  struct {
    const uint32_t N_PORTS:4;
    const uint32_t PPC:1;
    const uint32_t :3;
    const uint32_t N_PCC:4;
    const uint32_t N_CC:4;
    const uint32_t PI:1;
    const uint32_t :3;
    const uint32_t N_PTT:4;
    const uint32_t N_TT:4;
    const uint32_t :4;
  } HCSPARAMS;
  struct {
    const uint32_t ADC:1;
    const uint32_t PFL:1;
    const uint32_t ASP:1;
    const uint32_t :1;
    const uint32_t IST:4;
    const uint32_t EECP:8;
    const uint32_t :16;
  } HCCPARAMS;
} usb_ehci_base_t;

typedef struct {
  volatile uint32_t USBCMD;
  volatile uint32_t USBSTS;
  volatile uint32_t USBINTR;
  volatile uint32_t FRINDEX;
  volatile uint32_t CTRLDSSEGMENT;
  void* volatile PERIODICLISTBASE;
  volatile uint32_t ASYNCLISTADDR;
  const uint32_t pad[9];
  volatile uint32_t CONFIGFLAG;
  volatile uint32_t PORTSC[0];
} usb_ehci_cmd_t;

typedef union {
  struct {
    uint16_t CONNECTION:1;
    uint16_t ENABLE:1;
    uint16_t SUSPEND:1;
    uint16_t OVER_CURRENT:1;
    uint16_t RESET:1;
    uint16_t :3;
    uint16_t POWER:1;
    uint16_t LOW_SPEED:1;
    uint16_t HIGH_SPEED:1;
    uint16_t TEST:1;
    uint16_t INDICATOR:1;
    uint16_t :3;
  };
  uint16_t val;
} port_status_t;

// ISOCHRONOUS (HIGH-SPEED) TRANSFER DESCRIPTOR: FIXED LAYOUT (64-BYTE ALIGNED)
typedef struct __attribute__((aligned(64))) usb_iTD_t {
  uint32_t horizontal_link;
  struct {
    uint32_t offset:12;
    uint32_t PG:3;
    uint32_t IOC:1;
    uint32_t total:12;
    uint32_t status:4;
  } transfers[8];

  uint32_t address:7;
  uint32_t :1;
  uint32_t endpt:4;
  uint32_t page0:20;
  uint32_t wMaxPacketSize:11;
  uint32_t dir:1;
  uint32_t page1:20;
  uint32_t Mult:2;
  uint32_t :10;
  uint32_t page2:20;
  uint32_t :12;
  uint32_t page3:20;
  uint32_t :12;
  uint32_t page4:20;
  uint32_t :12;
  uint32_t page5:20;
  uint32_t :12;
  uint32_t page6:20;
} usb_iTD_t;

// SPLIT-TRANSACTION ISOCHRONOUS TRANSACTION DESCRIPTOR: FIXED LAYOUT (32-BYTE ALIGNED)
typedef struct __attribute__((aligned(32))) usb_siTD_t {
  uint32_t horizontal_link;

  typedef struct {
    uint8_t address:7;
    uint8_t :1;
    uint8_t endpt:4;
    uint8_t :4;
    uint8_t hub:7;
    uint8_t :1;
    uint8_t port:7;
    uint8_t dir:1;
  } sitd_state;
  sitd_state state;

  uint8_t s_mask;
  uint8_t c_mask;
  uint16_t :16;

  uint8_t status;
  uint8_t c_prog_mask;
  uint16_t total:10;
  uint16_t :4;
  uint16_t P:1;
  uint16_t IOC:1;

  void* page0;

  uint32_t t_count:3;
  uint32_t TP:2;
  uint32_t :7;
  uint32_t page1:20;

  uint32_t back_pointer;
  // pad up to 32 bytes
  uint32_t :32;
} usb_siTD_t;

typedef struct sitd_transfer : usb_siTD_t {
  struct sitd_transfer *next;
  CCallback<sitd_transfer> *cb;
  uint32_t frame;
  void* buffer;
  int16_t *length;
  uint32_t link_to() const { return ((uint32_t)&horizontal_link)|4; }
} sitd_transfer;

// QUEUE ELEMENT TRANSFER DESCRIPTOR: FIXED LAYOUT (32-BYTE ALIGNED EXCEPT IN QUEUE HEAD)
typedef struct usb_qTD_t {
  struct usb_qTD_t* next;
  struct usb_qTD_t* alt;
  union {
    struct {
      uint32_t status:8;
      uint32_t PID:2;
      uint32_t CERR:2;
      uint32_t c_Page:3;
      uint32_t IOC:1;
      uint32_t total:15;
      uint32_t dt:1;
    };
    uint32_t val;
  } token;
  uint32_t qtd_page[5];

  void fill_qtd(usb_qTD_t* next_qtd, usb_qTD_t* alt_qtd, bool dt, uint16_t total, bool ioc, uint8_t pid, const void* data);
} usb_qTD_t;

// QUEUE HEAD: FIXED LAYOUT, MUST BE 32-BYTE ALIGNED + NOT CROSS 4096-BYTE BOUNDARY (64-BYTE ALIGNMENT)
class __attribute__((aligned(64))) usb_queue_head_t {
protected:
  uint32_t horizontal_link;
  struct {
    uint32_t address:7;
    uint32_t I:1;
    uint32_t Endpt:4;
    uint32_t speed:2;
    uint32_t DTC:1;
    uint32_t H:1;
    uint32_t wMaxPacketSize:11;
    uint32_t C:1;
    uint32_t RL:4;

    uint8_t s_mask;
    uint8_t c_mask;
    uint16_t hub:7;
    uint16_t port:7;
    uint16_t Mult:2;
  } capabilities;
  usb_qTD_t* current;
  usb_qTD_t overlay;

  void sync_after_write(void) const;
  void sync_before_read(void);
  void clean_after_write(void);
  uint32_t link_to() const { return ((uint32_t)&horizontal_link)|2; }
private: // pad size to 64 bytes
  uint32_t pad[4];
};

class __attribute__((aligned(32))) usb_qTD_aligned : public usb_qTD_t {};

class usb_transfer : public usb_qTD_aligned  {
private:
  void sync_chain(usb_qTD_t* end);
public:
  CCallback<class usb_transfer>* cb;
  class usb_transfer *error_handler;

  usb_transfer() : cb(NULL),error_handler(NULL) {}
  virtual ~usb_transfer() = default;

  void sync_chain(void) {sync_chain(this);}
  void sync_after_write(void) const;
  void sync_before_read(void);
};

class static_usb_transfer : public usb_transfer {
  void* operator new(size_t); // undefined, this class doesn't support dynamic allocation
  void operator delete(void*,size_t) {}
};

class usb_control_transfer {
protected:
  usb_control_setup req;
  void* buffer;
public:
  uint8_t getbmRequestType(void) const { return req.bmRequestType; }
  uint8_t getbmRequest(void) const { return req.bmRequest;}
  uint16_t getwValue(void) const { return req.wValue; }
  uint16_t getwIndex(void) const { return req.wIndex; }
  uint16_t getwLength(void) const { return req.wLength; }
  const usb_control_setup* getSetup(void) const { return &req; }
  uint64_t getSetupData(void) const;
  const void* getBuffer(void) const { return buffer; }
};

class usb_bulk_interrupt_transfer : public usb_transfer, private CCallback<usb_transfer> {
private:
  const USBCallback* cbi;

  static_usb_transfer extra[3];
  void callback(const usb_transfer*,int);

  void* buffer;
  uint32_t dlength;
  bool dir_in;
public:
  usb_bulk_interrupt_transfer(bool _dir_in, uint32_t length, void *_buffer, const USBCallback* _cb, uint32_t max_packet);
};

class USB_Endpoint {
protected:
  USB_Endpoint(uint8_t _ep_type) :
  ep_type(_ep_type) {
  }
public:
  class USB_Endpoint* host_next = NULL;
  class USB_Device* device = NULL;
  const uint8_t ep_type;

  virtual void update(void) = 0;
  virtual void flush(void) = 0;

  virtual int ControlTransfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *buffer, CCallback<usb_control_transfer>* cb) {
    return -EOPNOTSUPP;
  }
  virtual int BulkTransfer(uint32_t Length, void *buffer, const USBCallback* cb) {
    return -EOPNOTSUPP;
  }
  virtual int InterruptTransfer(uint32_t Length, void *buffer, const USBCallback* cb) {
    return -EOPNOTSUPP;
  }
  virtual int IsochronousTransfer(isolength* Lengths, void *buffer, const USBCallback* cb) {
    return -EOPNOTSUPP;
  }

  virtual ~USB_Endpoint() = default;
};

class PeriodicScheduler {
private:
  volatile uint32_t& FRINDEX;
protected:
  uint32_t* periodictable;
  PeriodicScheduler(volatile uint32_t&);
  ~PeriodicScheduler();
public:
  bool add_node(uint32_t frame, uint32_t link_to, uint32_t interval);
  bool remove_node(uint32_t frame, uint32_t link_to, uint32_t next);
  uint32_t current_uframe(void);
};

class USB_Periodic_Endpoint : public USB_Endpoint {
private:
  PeriodicScheduler *scheduler = NULL;
protected:
  using USB_Endpoint::USB_Endpoint;
  virtual void new_offset(void) = 0;
  bool add_node(uint32_t frame, uint32_t link_to, uint32_t interval) { return scheduler ? scheduler->add_node(frame, link_to, interval) : false; }
  bool remove_node(uint32_t frame, uint32_t link_to, uint32_t next) { return scheduler ? scheduler->remove_node(frame, link_to, next) : false; }
  uint32_t current_uframe(void) { return scheduler ? scheduler->current_uframe() : 0; }

public:
  uint32_t interval = 0; // calculated interval in uframes, mapped to PERIODIC_LIST_SIZE*8
  uint32_t offset = 0;
  uint8_t stime = 0;
  uint8_t ctime = 0;

  void activate(uint32_t _offset, PeriodicScheduler *s) { scheduler = s; offset = _offset; new_offset(); }
  virtual void get_masks(uint8_t& s_mask, uint8_t& c_mask) const = 0;
  virtual bool set_inactive(void) = 0;
};

class QH_Base : public usb_queue_head_t {
private:
  usb_transfer* pending;
  usb_transfer* dummy;
  bool active;

protected:
  void update(void);
  void flush(void);
  bool enqueue_transfer(usb_transfer* head);
  int BulkIntrTransfer(bool dir_in, uint32_t Length, void *buffer, const USBCallback* cb);
  QH_Base(uint8_t endpoint, uint16_t max_packet_size, uint8_t hub, uint8_t port, uint8_t address, uint8_t speed);
  ~QH_Base();
};

template <class Base>
class USB_QH_Endpoint : public QH_Base, public Base {
protected:
  USB_QH_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t hub, uint8_t port, uint8_t address, uint8_t speed, uint8_t ep_type) :
  QH_Base(endpoint, max_packet_size, hub, port, address, speed),
  Base(ep_type) {
  }

public:
  void update(void) override { QH_Base::update(); }
  void flush(void) override { QH_Base::flush(); }
};

class USB_Async_Endpoint : public USB_QH_Endpoint<USB_Endpoint> {
protected:
  USB_Async_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t hub, uint8_t port, uint8_t address, uint8_t speed, uint8_t ep_type) :
  USB_QH_Endpoint(endpoint, max_packet_size, hub, port, address, speed, ep_type) {
  }
public:
  void set_link(const USB_Async_Endpoint* p);
  USB_Async_Endpoint* get_link(void) {
    return static_cast<USB_Async_Endpoint*>((usb_queue_head_t*)(horizontal_link & ~0x1F));
  }
};

class USB_Control_Endpoint : public USB_Async_Endpoint {
private:
  class transfer : public usb_transfer, private CCallback<usb_transfer>, private usb_control_transfer {
  private:
    static_usb_transfer data, ack;
    CCallback<usb_control_transfer>* const cb;
    bool do_release;

    virtual void callback(const usb_transfer*, int result) override;

  public:
    transfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, \
    void *buffer, CCallback<usb_control_transfer>* _cb, bool release_mem);
  };

public:
  USB_Control_Endpoint(uint8_t max_packet_size, uint8_t address, uint8_t hub, uint8_t port, uint8_t speed);
  // non-virtual
  int Transfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *buffer, CCallback<usb_control_transfer>* cb);
  // virtual calls non-virtual
  int ControlTransfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *buffer, CCallback<usb_control_transfer>* cb) {
    return Transfer(bmRequestType, bmRequest, wValue, wIndex, wLength, buffer, cb);
  }
};

template <class Endpoint_Base>
class USB_Bulk_Interrupt_Endpoint : public Endpoint_Base {
protected:
  const bool dir_in;
  USB_Bulk_Interrupt_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed, uint8_t ep_type) :
  Endpoint_Base(endpoint & 0xF, max_packet_size & 0x7FF, hub_addr, port, address, speed, ep_type),
  dir_in(endpoint & 0x80) {
  }
};

class USB_Bulk_Endpoint : public USB_Bulk_Interrupt_Endpoint<USB_Async_Endpoint> {
public:
  USB_Bulk_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed) :
  USB_Bulk_Interrupt_Endpoint(endpoint, max_packet_size, address, hub_addr, port, speed, USB_ENDPOINT_BULK) {
  }

  int BulkTransfer(uint32_t Length, void *buffer, const USBCallback* cb) {
    return BulkIntrTransfer(dir_in, Length, buffer, cb);
  }
};

class USB_Interrupt_Endpoint : public USB_Bulk_Interrupt_Endpoint<USB_QH_Endpoint<USB_Periodic_Endpoint>> {
private:
  void new_offset(void) override;
public:
  void get_masks(uint8_t& s_mask, uint8_t& c_mask) const override { s_mask = capabilities.s_mask, c_mask = capabilities.c_mask; }
  bool set_inactive(void) override;

  USB_Interrupt_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint8_t speed, uint32_t interval);
  int InterruptTransfer(uint32_t Length, void *buffer, const USBCallback* cb) {
    return BulkIntrTransfer(dir_in, Length, buffer, cb);
  }
};

class USB_ISO_Endpoint : public USB_Periodic_Endpoint {
private:
  const bool dir_in;
  uint16_t wMaxPacketSize;
  usb_siTD_t::sitd_state state;
  uint8_t s_mask, c_mask;

  sitd_transfer* pending = NULL;

  int Transfer(sitd_transfer*, int16_t *length, void *buffer, CCallback<sitd_transfer>*);

public:
  void update(void) override;
  void flush(void) override {}

  void new_offset(void) override;
  bool set_inactive(void) override { return false; }

  USB_ISO_Endpoint(uint8_t endpoint, uint16_t max_packet_size, uint8_t address, uint8_t hub_addr, uint8_t port, uint32_t interval);
  ~USB_ISO_Endpoint();

  void get_masks(uint8_t& smask, uint8_t& cmask) const override {smask = s_mask, cmask = c_mask;}

  int IsochronousTransfer(isolength* Lengths, void *buffer, const USBCallback* cb) override;
};

class USB_Hub {
  friend class USB_Host;
private:
  struct USB_Port {
    uint32_t state;
    class USB_Device *device;
    uint32_t timeout_start;
  } port[7];

  virtual void port_power(uint8_t port, bool set) = 0;
  virtual void port_reset(uint8_t port, bool set) = 0;
  virtual void port_enable(uint8_t port, bool set) = 0;
  virtual uint8_t base_port(uint8_t port) const = 0;
  virtual void phySetHighSpeed(uint8_t port, bool on) {}

  virtual void addref(void) {}
  virtual void deref(void) {}

protected:
  uint8_t const hub_addr;
  USB_Hub(uint8_t);
};

typedef enum {
  USB_MSG_INTERRUPT = 0,
  USB_MSG_ADDRESS_RELEASED,
  USB_MSG_PORT_STATUS = 0x100,
  USB_MSG_PORT_TIMEOUT,
  USB_MSG_ENDPOINT_ACTIVATE = 0x200,
  USB_MSG_ENDPOINT_DEACTIVATE,
  USB_MSG_DEVICE_INIT = 0x300,
  USB_MSG_DEVICE_ENDPOINT_REMOVED,
  USB_MSG_DEVICE_FIND_DRIVER,
  USB_MSG_DEVICE_CONTROL_TRANSFER,
  USB_MSG_DEVICE_BULK_TRANSFER,
  USB_MSG_DEVICE_INTERRUPT_TRANSFER,
  USB_MSG_DEVICE_ISOCHRONOUS_TRANSFER,
} usb_msg_type_t;

typedef struct {
  usb_msg_type_t type;
  union {
    struct {
      USB_Hub* hub;
      uint8_t port;
      port_status_t status;
    } port;
    struct {
      USB_Endpoint *ep;
      USB_Device *device;
    } endpoint;
    struct {
      class USB_Device *dev;
      union {
        const USBCallback* cb;
        CCallback<usb_control_transfer>* control_cb;
      };
      union {
        USB_Endpoint *endpoint;
        struct {
          uint8_t bmRequestType;
          uint8_t bmRequest;
          uint16_t wValue;
          uint16_t wIndex;
          uint16_t wLength;
          void* data;
        } control;
        struct {
          uint8_t bEndpoint;
          uint32_t dLength;
          void* data;
        } bulkintr;
        struct {
          uint8_t bEndpoint;
          isolength* lengths;
          void *data;
        } iso;
      };
    } device;
    uint8_t address;
  };
} usb_msg_t;

class USB_Device : private CCallback<usb_control_transfer> {
  friend class USB_Host;
  friend class USB_Hub_Driver;
private:
  USB_Control_Endpoint control;
  class USB_Host* const host;
  const uint8_t hub_addr;
  const uint8_t port;
  const uint8_t speed;
  const uint8_t address;
  std::atomic_uint refcount;

  struct Endpoint_Elem {
    USB_Endpoint *ep;
    int type;
  };

  class Endpoint_Array {
  private:
    // 1-15=OUT,16-30=IN
    Endpoint_Elem eps[31];
  public:
    Endpoint_Elem& operator[] (size_t ep_addr) {
      size_t index = ep_addr & 0x7F;
      if (index != 0) {
        if (index >= 16) index = 0;
        else if (ep_addr & 0x80) index += 15;
      }
      return eps[index];
  }
  Endpoint_Array() {
      for (size_t i=0; i < sizeof(eps)/sizeof(eps[0]); i++) {
        eps[i].ep = NULL;
        eps[i].type = -1;
      }
    }
  };

  Endpoint_Array Endpoints;

  class dev_interface {
    class setting {
    public:
      const usb_interface_descriptor* const interface_desc;
      size_t length;
      std::vector<const usb_endpoint_descriptor*> endpoints;

      setting(const uint8_t *i, const uint8_t *end);
    };

    std::vector<setting> altSettings;
  public:
    int active;
    dev_interface(const uint8_t *i, const uint8_t *end);
    void add_alternate(const uint8_t *i, const uint8_t *end);

    const usb_interface_descriptor* getInterface(size_t &l, uint8_t index) const;
    const usb_interface_descriptor* getInterface(uint8_t index=0) const;

    const usb_endpoint_descriptor* getEndpoint(uint8_t eindex, uint8_t iindex=0) const;
  };

  class dev_config {
    /* Holds the data returned by USB_REQ_GETDESCRIPTOR for this configuration.
     * Interfaces and endpoints all return pointers into this memory.
     */
    uint8_t* const raw;
    const size_t length;
    // array of interfaces, which is also an array of altSettings
    std::vector<dev_interface> interfaces;

  public:
    dev_config(const usb_configuration_descriptor *desc, size_t len);
    ~dev_config();

    const usb_configuration_descriptor* getConfiguration(void) const { return (const usb_configuration_descriptor*)raw; }
    const usb_configuration_descriptor* getConfiguration(size_t &l) const { l = length; return getConfiguration(); }
    dev_interface* interface(uint8_t index);
  };

  usb_device_descriptor ddesc;
  uint16_t string_lang;
  std::map<uint8_t,std::string> strings;
  std::map<uint8_t,dev_config> configs;
  // array of attached drivers for this device
  std::vector<class USB_Driver*> drivers;
  int active_config, pending_config;

  virtual ~USB_Device() = default;

  virtual void callback(const usb_control_transfer*,int) override;
  void deref(void);

  USB_Device(USB_Host* _host, uint8_t _hub_addr, uint8_t _port, uint8_t _speed, uint8_t _address, uint8_t control_packet_size);
  void disconnect(void);

  void USBMessage(const usb_msg_t&);
  void request_string(uint8_t string_id);
  void search_for_drivers(void);

  void deactivate_endpoint(size_t i);
  void activate_endpoint(const usb_endpoint_descriptor*);
  void activate_interface(uint8_t interface, int altsetting);
  void activate_configuration(int configuration);

  static uint8_t validate_descriptor(const uint8_t* &desc, const uint8_t* const end);

  bool prepare_control_transfer(usb_msg_t& msg);
  void BulkTransfer(uint8_t bEndpoint, uint32_t dLength, void* data, const USBCallback* cb);
  void InterruptTransfer(uint8_t bEndpoint, uint32_t dLength, void* data, const USBCallback* cb);
  void IsochronousTransfer(uint8_t bEndpoint, isolength* Lengths, void* data, const USBCallback* cb);
  void ControlTransfer(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, CCallback<usb_control_transfer>*);

public:
  static uint8_t validate_descriptor(const uint8_t* desc, int length) {
    if (length <= 0) return 0;
    return validate_descriptor(desc, desc+length);
  }
  bool pushMessage(usb_msg_t&);
};

class USB_Driver {
private:
  USB_Device* device;
  virtual void detach(void) = 0;
public:
  void disconnect(void) {device = NULL; detach();}

  class Factory {
  private:
    static class Factory* gList;
    class Factory* next;

  protected:
    Factory();
    ~Factory();
    virtual bool offer(const usb_device_descriptor*,const usb_configuration_descriptor*) {return false;}
    virtual bool offer(const usb_interface_descriptor*,size_t) {return false;}
  public:
    static class USB_Driver::Factory* find_driver(const usb_device_descriptor*,const usb_configuration_descriptor*);
    static class USB_Driver::Factory* find_driver(const usb_interface_descriptor*,size_t);
    virtual class USB_Driver* attach(const usb_device_descriptor*,const usb_configuration_descriptor*,USB_Device*) {return NULL;}
    virtual class USB_Driver* attach(const usb_interface_descriptor*,size_t,USB_Device*) {return NULL;}
  };

  virtual ~USB_Driver() = default;

protected:
  USB_Driver();
  void setDevice(USB_Device *d) {device = d;}
  // return a const pointer so it can only be compared, not accessed
  const USB_Device* getDevice(void) { return device; }
  // asynchronous, temporary callback object
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback&);
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, USBCallback);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, USBCallback);
  int IsochronousMessage(uint8_t bEndpoint, isolength*, void *data, USBCallback);
  // asynchronous, callback pointer
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data, const USBCallback*);
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data, const USBCallback*);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data, const USBCallback*);
  int IsochronousMessage(uint8_t bEndpoint, isolength*, void *data, const USBCallback*);
  // synchronous
  int ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data);
  int BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data);
  int InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data);
  int IsochronousMessage(uint8_t bEndpoint, isolength*, void *data);
};

// base class for a driver that can be instantiated multiple times as needed
template <class Driver>
class USB_Driver_FactoryGlue : public USB_Driver {
  class Register : USB_Driver::Factory {
    bool offer(const usb_device_descriptor *dd, const usb_configuration_descriptor *cd) {
      return Driver::offer_config(dd,cd);
    }
    bool offer(const usb_interface_descriptor *id, size_t l) {
      return Driver::offer_interface(id, l);
    }
    USB_Driver* attach(const usb_device_descriptor *dd, const usb_configuration_descriptor *cd, USB_Device *d) {
      return Driver::attach_config(dd,cd,d);
    }
    USB_Driver* attach(const usb_interface_descriptor *id, size_t l, USB_Device *d) {
      return Driver::attach_interface(id,l,d);
    }
  };
  template<Register&> struct inst {};

  // derived class should implement these as needed
  static bool offer_config(const usb_device_descriptor*,const usb_configuration_descriptor*) {
    return false;
  }
  static bool offer_interface(const usb_interface_descriptor*,size_t) {
    return false;
  }
  static USB_Driver* attach_config(const usb_device_descriptor*,const usb_configuration_descriptor*,USB_Device*) {
    return NULL;
  }
  static USB_Driver* attach_interface(const usb_interface_descriptor*,size_t,USB_Device*) {
    return NULL;
  }

  static Register auto_reg;
  static inst<auto_reg> reg_it;
protected:
  USB_Driver_FactoryGlue(USB_Device* p) {setDevice(p);}
};

template <class Driver> typename USB_Driver_FactoryGlue<Driver>::Register USB_Driver_FactoryGlue<Driver>::auto_reg;

class USB_Host : protected USB_Hub, private CCallback<usb_control_transfer>, private PeriodicScheduler {
private:
  usb_ehci_cmd_t* const EHCI;
  uint32_t addresses[128/32];
  const uint8_t nPorts;
  uint8_t uframe_bandwidth[PERIODIC_LIST_SIZE*8];

  USB_Endpoint *endpoints;
  USB_Endpoint *async_cleanup;
  USB_Endpoint *periodic_cleanup;

  class Enum_Control : public USB_Control_Endpoint {
  public:
    USB_Hub *hub;
    uint8_t port;
    uint8_t speed;
    uint8_t bMaxPacketSize;
    uint8_t address_retry;
    bool in_progress;

    Enum_Control();

    void set(USB_Hub& h, uint8_t p, uint8_t s);
    void clean(void);
    using USB_Control_Endpoint::sync_after_write;
    using USB_Control_Endpoint::sync_before_read;
    using USB_Control_Endpoint::clean_after_write;
  } Enum;

private:
  void callback(const usb_control_transfer* t, int result);

  uint8_t get_address(void);
  void release_address(uint8_t i);

  bool set_port_timeout(USB_Hub& hub, uint8_t port, uint32_t ticks);
  void port_status(USB_Hub& hub, uint8_t port, port_status_t status);
  void port_timeout(usb_msg_t& msg);

  void notify_endpoint_removed(USB_Endpoint *ep);
  void update_transfers(void);

  void add_async_queue(USB_Async_Endpoint *ep);
  bool remove_async_queue(USB_Async_Endpoint *ep);

  void add_periodic_queue(USB_Periodic_Endpoint *ep);
  bool remove_periodic_queue(USB_Periodic_Endpoint *ep);
  void unschedule_periodic(void);
  bool calculate_offset(const USB_Periodic_Endpoint *ep, uint32_t &offset);

//  uint32_t current_uframe(void) override;

  void port_power(uint8_t port, bool set);
  void port_reset(uint8_t port, bool set);
  void port_enable(uint8_t port, bool set);
  uint8_t base_port(uint8_t port) const {return port;}

protected:
  USB_Host(usb_ehci_base_t* const base);
  void usb_process(void);

  // functions to be provided by derived class
public:
  virtual bool putMessage(usb_msg_t&) = 0;
  virtual bool timerMsg(usb_msg_t &msg, uint32_t ms) = 0;
private:
  virtual bool getMessage(usb_msg_t&) = 0;
  virtual void nextIRQ(void) = 0;
  virtual void setHostMode(void) = 0;
  virtual uint32_t getMillis(void) = 0;

public:
  bool activate_endpoint(USB_Endpoint *ep, USB_Device *device);
  bool deactivate_endpoint(USB_Endpoint *ep);
};

#endif

#include <teensyatom.h>
#include <HardwareSerial.h>
#include <EventResponder.h>
#include "usb_host.h"

#define CH341_MIN_BPS      50
#define CH341_MAX_BPS      2000000

#define CH341_DEFAULT_BAUD 9600

namespace ch341 {

template <size_t length>
class circ_buf {
private:
  uint8_t buf[length];
  uint8_t *head;
  uint8_t *tail;
  size_t avail; // space available for writing
public:
  circ_buf();
  size_t available(void) {return length-avail;}
  size_t availableForWrite(void) {return avail;}
  size_t read(uint8_t *dst, size_t len);
  void write(const uint8_t *src, size_t len);
  const uint8_t* peek(void) {return head;}
  void flush(void);
};

class serial : public USB_Driver, public USB_Driver::Factory, public HardwareSerial, public EventResponder {
private:
  uint8_t status_in[8] __attribute__((aligned(32)));
  uint8_t data_in[2][64] __attribute__((aligned(32)));
  // 512 bytes can give max performance for very high baudrates but 128 is fast enough for <= 1228800
  uint8_t data_out[2][128] __attribute__((aligned(32)));
  uint16_t version;
  uint8_t ep_out;
  uint8_t ep_in;
  uint8_t ep_status;
  volatile bool attached;

  volatile bool started;

  bool hw_flow;
  uint8_t lcr;
  uint8_t factor;
  uint8_t divisor;

  void calculate_baud(uint32_t baud);
  void uart_mode(uint32_t mode);
  void start(void);

  void status_callback(int result);
  void get_status(void);

  void init(int result, unsigned int stage);

  uint8_t out_status;
  void dtr_rts_callback(int,uint8_t,uint8_t);
  void set_dtr_rts(uint8_t); // should be called under rx_lock

  uint8_t status;

  void write_callback(int result, uint8_t *buf);
  uint8_t *tx_buf[2];
  uint32_t tx_length;
  uint32_t tx_max;
  ATOM_MUTEX tx_lock;
  ATOM_COND tx_signal;

  static void send_timer_expired(EventResponder&);
  MillisTimer sendTimer;

  void read_callback(int result, uint8_t* buf);
  void queue_read(uint8_t *buf);
  uint8_t *rx_buf[2];
  ATOM_MUTEX rx_lock;

  circ_buf<512> read_buf;

public:
  serial();
  ~serial();

  void set_dtr_rts(bool dtr, bool rts);
  void set_dtr(bool set);
  void set_rts(bool set);

  operator bool ();
  void begin(uint32_t baud, uint16_t format, bool rts_cts);
	void begin(uint32_t baud, uint16_t format=SERIAL_8N1) {begin(baud,format,true);}
	void end();
	int available(void);
	int peek(void);
	int read(void);
	void flush(void);
	int availableForWrite(void);
	size_t write(uint8_t);

  void detach(void);
  bool offer(const usb_device_descriptor* d,const usb_configuration_descriptor*);
  USB_Driver* attach(const usb_device_descriptor*,const usb_configuration_descriptor*, USB_Device* d);
};

} // namespace ch341

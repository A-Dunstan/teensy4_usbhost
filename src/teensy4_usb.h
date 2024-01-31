#include <atomic>
#include <teensyatom.h>
#include "usb_host.h"

#define USB_STACK_SIZE 1024

class USB_Host_Port1 : public USB_Host {
  bool getMessage(usb_msg_t &msg);
  bool putMessage(usb_msg_t &msg);

  void nextIRQ(void);
  void setHostMode(void);

  void phySetHighSpeed(uint8_t port, bool on);
  void port_power(uint8_t port, bool set);

  bool timerMsg(usb_msg_t &msg, uint32_t ms);
  uint32_t getMillis(void);

private:
  // static so it is usable by the ISR
  static ATOM_QUEUE g_usbqueue;
  ATOM_TCB usb_thread;
  uint32_t usb_stack[USB_STACK_SIZE];

  class TimerMsg : public ATOM_TIMER {
    static void timer_callback(POINTER cb_data);

    usb_msg_t msg;
    std::atomic<class TimerMsg*> &release_list;
    class TimerMsg *next;
  public:
    TimerMsg(const usb_msg_t &_msg, uint32_t ms, std::atomic<class TimerMsg*> &r);
    ~TimerMsg();
  };
  std::atomic<TimerMsg*> timerRelease;

  template<uint8_t usb_int>
  static void usb_isr(void);

  void thread(void);

  static void thread_start(uint32_t _p) {
    ((class USB_Host_Port1*)_p)->thread();
  }

public:
  USB_Host_Port1();
  void begin(void);
};

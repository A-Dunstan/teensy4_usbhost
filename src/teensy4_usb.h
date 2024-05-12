#include <atomic>
#include <teensyatom.h>
#include "usb_host.h"

#define USB_STACK_SIZE 1024

typedef struct REG32_QUAD_t {
  volatile uint32_t REG;
  volatile uint32_t SET;
  volatile uint32_t CLR;
  volatile uint32_t TOG;
} REG32_QUAD_t;

typedef struct usb_phy_t {
  REG32_QUAD_t PWD;
  REG32_QUAD_t TX;
  REG32_QUAD_t RX;
  REG32_QUAD_t CTRL;
  volatile uint32_t status;
  REG32_QUAD_t DEBUG;
  volatile uint32_t debug0;
  REG32_QUAD_t DEBUG1;
  volatile uint32_t version;
} usb_phy_t;

class USBHostBase : public USB_Host {
  bool getMessage(usb_msg_t &msg);
  bool putMessage(usb_msg_t &msg);

  bool timerMsg(usb_msg_t &msg, uint32_t ms);

// monotonic clock, counts milliseconds
  uint32_t getMillis(void) {return millis();}

private:
  ATOM_QUEUE* const usbqueue;

  class TimerMsg : public ATOM_TIMER {
    static void timer_callback(POINTER cb_data);

    usb_msg_t msg;
    USBHostBase* const host;
    class TimerMsg *next;
  public:
    TimerMsg(const usb_msg_t &_msg, uint32_t ms, USBHostBase *h);
    ~TimerMsg();
  };
  std::atomic<TimerMsg*> timerRelease;

  ATOM_TCB usb_thread;
  uint32_t usb_stack[USB_STACK_SIZE];
  virtual void thread(void) = 0;
  static void thread_start(uint32_t _p) {
	  ((class USBHostBase*)_p)->thread();
  }

protected:
  USBHostBase(ATOM_QUEUE*, usb_ehci_base_t*);
  static void init_pll(struct REG32_QUAD_t *const PLL);
  static void phy_on(struct usb_phy_t *const PHY);

public:
  void begin(void) {
	  atomThreadCreate(&usb_thread, 64, thread_start, (uint32_t)this, usb_stack, sizeof(usb_stack), 0);
  }
};

template <IRQ_NUMBER_t irq, uint32_t phy, uint32_t ehci, uint32_t pll>
class TeensyUSB : public USBHostBase {
  void nextIRQ(void) {
    /* make sure any EHCI register writes are complete before clearing pending interrupts,
     * otherwise it may immediately re-pend
     */
    asm volatile("dmb");
    NVIC_CLEAR_PENDING(irq);
    NVIC_ENABLE_IRQ(irq);
  }
  void setHostMode(void) {
    volatile uint32_t* HOST_EHCI = (volatile uint32_t*)ehci;
    HOST_EHCI[OFFSET_EHCI_USBMODE/4] = USB_USBMODE_CM(3); // 3 = host mode
    // setting SBUSCFG to anything other than 0 causes random issues...
    HOST_EHCI[OFFSET_EHCI_SBUSCFG/4] = 0;
  }
  void phySetHighSpeed(uint8_t port, bool on) {
    usb_phy_t* HOST_PHY = (usb_phy_t*)phy;
    if (on)
      HOST_PHY->CTRL.SET = USBPHY_CTRL_ENHOSTDISCONDETECT;
    else
      HOST_PHY->CTRL.CLR = USBPHY_CTRL_ENHOSTDISCONDETECT;
  }

  static const size_t OFFSET_EHCI_SBUSCFG = 0x90;
  static const size_t OFFSET_EHCI_CAPLENGTH = 0x100;
  static const size_t OFFSET_EHCI_USBMODE = 0x1A8;

  // static so it is usable by the ISR
  static ATOM_QUEUE g_usbqueue;
  static void usb_isr(void) {
    usb_msg_t intmsg = { USB_MSG_INTERRUPT };
    NVIC_DISABLE_IRQ(irq);
    atomIntEnter();
    if (atomQueuePut(&g_usbqueue, -1, (uint8_t*)&intmsg) != ATOM_OK)
      digitalWriteFast(LED_BUILTIN, HIGH);
    atomIntExit(0);
  }

protected:
  virtual void thread(void) {
    usb_msg_t msgs[50];
    atomQueueCreate(&g_usbqueue, (uint8_t *)msgs, sizeof(msgs[0]), sizeof(msgs) / sizeof(msgs[0]));

    attachInterruptVector(irq, usb_isr);
    init_pll((struct REG32_QUAD_t*)pll);

    // ungate clock
    CCM_CCGR6 |= CCM_CCGR6_USBOH3(CCM_CCGR_ON);
    phy_on((struct usb_phy_t *)phy);

    atomTimerDelay(10 * SYSTEM_TICKS_PER_SEC / 1000);  // 10ms

    NVIC_ENABLE_IRQ(irq);
    usb_process();

    NVIC_DISABLE_IRQ(irq);
    atomQueueDelete(&g_usbqueue);
  }

  TeensyUSB() : USBHostBase(&g_usbqueue, (usb_ehci_base_t*)(ehci+OFFSET_EHCI_CAPLENGTH)) {}
};

template <IRQ_NUMBER_t irq, uint32_t phy, uint32_t ehci, uint32_t pll>
DMAMEM ATOM_QUEUE TeensyUSB<irq,phy,ehci,pll>::g_usbqueue;

class TeensyUSBHost1 : public TeensyUSB<IRQ_USB1, IMXRT_USBPHY1_ADDRESS, IMXRT_USB1_ADDRESS, IMXRT_CCM_ANALOG_ADDRESS+0x10> {
public:
	TeensyUSBHost1() = default;
};

class TeensyUSBHost2 : public TeensyUSB<IRQ_USB2, IMXRT_USBPHY2_ADDRESS, IMXRT_USB2_ADDRESS, IMXRT_CCM_ANALOG_ADDRESS+0x20> {
#ifdef ARDUINO_TEENSY41
private:
	void thread(void);
	void port_power(uint8_t port, bool set);
#endif
public:
	TeensyUSBHost2()  = default;
};

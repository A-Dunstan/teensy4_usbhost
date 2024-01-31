#include <Arduino.h>
#include <unistd.h>  // usleep
#include "teensy4_usb.h"

typedef struct {
  volatile uint32_t REG;
  volatile uint32_t SET;
  volatile uint32_t CLR;
  volatile uint32_t TOG;
} REG32_QUAD_t;

typedef struct {
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

FLASHMEM static void init_pll(REG32_QUAD_t *const PLL) {
  dprintf("Starting USB PLL...\n");
  while (1) {
    // ensure any previous PLL register writes have completed before doing more
    asm volatile("dmb");
    uint32_t n = PLL->REG;
    dprintf("CCM_ANALOG_PLL_USB=%08lX\n", n);
    if (n & CCM_ANALOG_PLL_USB1_DIV_SELECT) {
      PLL->CLR = 0xC000;                           // bypass 24MHz
      PLL->SET = CCM_ANALOG_PLL_USB1_BYPASS;       // bypass
      PLL->CLR = CCM_ANALOG_PLL_USB1_POWER |       // power down
                 CCM_ANALOG_PLL_USB1_DIV_SELECT |  // use 480 MHz
                 CCM_ANALOG_PLL_USB1_ENABLE |      // disable
                 CCM_ANALOG_PLL_USB1_EN_USB_CLKS;  // disable usb clocks
      continue;
    }
    if (!(n & CCM_ANALOG_PLL_USB1_ENABLE)) {
      dprintf("  enable PLL\n");
      PLL->SET = CCM_ANALOG_PLL_USB1_ENABLE;  // enable
      continue;
    }
    if (!(n & CCM_ANALOG_PLL_USB1_POWER)) {
      dprintf("  power up PLL\n");
      PLL->SET = CCM_ANALOG_PLL_USB1_POWER;
      continue;
    }
    if (!(n & CCM_ANALOG_PLL_USB1_LOCK)) {
      dprintf("  wait for lock\n");
      continue;
    }
    if (n & CCM_ANALOG_PLL_USB1_BYPASS) {
      dprintf("  turn off bypass\n");
      PLL->CLR = CCM_ANALOG_PLL_USB1_BYPASS;
      continue;
    }
    if (!(n & CCM_ANALOG_PLL_USB1_EN_USB_CLKS)) {
      dprintf("  enable USB clocks\n");
      PLL->SET = CCM_ANALOG_PLL_USB1_EN_USB_CLKS;
      continue;
    }
    dprintf("  USB PLL is running\n");
    break;
  }
}

FLASHMEM static void phy_on(usb_phy_t *const PHY) {
  PHY->CTRL.CLR = USBPHY_CTRL_SFTRST | USBPHY_CTRL_CLKGATE;
  PHY->CTRL.SET = USBPHY_CTRL_ENUTMILEVEL2 | USBPHY_CTRL_ENUTMILEVEL3;
  PHY->TX.CLR = USBPHY_TX_TXCAL45DP(15) | USBPHY_TX_TXCAL45DN(15) | USBPHY_TX_D_CAL(15);
  PHY->TX.SET = USBPHY_TX_TXCAL45DP(6) | USBPHY_TX_TXCAL45DN(6) | USBPHY_TX_D_CAL(12);
  PHY->PWD.REG = 0;
}

bool USB_Host_Port1::getMessage(usb_msg_t &msg) {
  uint8_t s = atomQueueGet(&g_usbqueue, 0, (uint8_t *)&msg);
  if (s != ATOM_OK)
    digitalWriteFast(LED_BUILTIN, HIGH);
  // release used timers now
  TimerMsg *t = std::atomic_exchange_explicit(&timerRelease, NULL, std::memory_order_relaxed);
  delete t;
  return (s == ATOM_OK);
}

bool USB_Host_Port1::putMessage(usb_msg_t &msg) {
  if (atomQueuePut(&g_usbqueue, -1, (uint8_t *)&msg) != ATOM_OK) {
    digitalWriteFast(LED_BUILTIN, HIGH);
    return false;
  }
  return true;
}

void USB_Host_Port1::nextIRQ(void) {
  /* make sure any EHCI register writes are complete before clearing pending interrupts,
   * otherwise it may immediately re-pend
   */
  asm volatile("dmb");
  NVIC_CLEAR_PENDING(IRQ_USB2);
  NVIC_ENABLE_IRQ(IRQ_USB2);
}

FLASHMEM void USB_Host_Port1::setHostMode(void) {
  USB2_USBMODE = USB_USBMODE_CM(3);  // 3 = host mode
  // setting SBUSCFG to anything other than 0 causes random issues...
  //USB2_SBUSCFG = 3; // bus config: INCR16 -> INCR8 -> INCR4 -> single
  USB2_SBUSCFG = 0;
}

void USB_Host_Port1::phySetHighSpeed(uint8_t port, bool on) {
  if (on)
    USBPHY2_CTRL_SET = USBPHY_CTRL_ENHOSTDISCONDETECT;
  else
    USBPHY2_CTRL_CLR = USBPHY_CTRL_ENHOSTDISCONDETECT;
}

void USB_Host_Port1::port_power(uint8_t port, bool set) {
  if (port == 0) {
    if (set) {
      GPIO8_DR_SET = 1 << 26;
      USB2_PORTSC1 |= USB_PORTSC1_PP;
    } else {
      USB2_PORTSC1 &= ~USB_PORTSC1_PP;
      GPIO8_DR_CLEAR = 1 << 26;
    }
  }
}

bool USB_Host_Port1::timerMsg(usb_msg_t &msg, uint32_t ms) {
  TimerMsg *t = new (std::nothrow) TimerMsg(msg, ms, timerRelease);
  if (t != NULL) {
    if (atomTimerRegister(t) == ATOM_OK)
      return true;
    dprintf("Registering timer<%p> failed\n", t);
    digitalWriteFast(LED_BUILTIN, HIGH);
    delete t;
  }
  return false;
}

void USB_Host_Port1::TimerMsg::timer_callback(POINTER cb_data) {
  TimerMsg *p = (TimerMsg *)cb_data;
  if (atomQueuePut(&g_usbqueue, -1, (uint8_t *)&p->msg) == ATOM_WOULDBLOCK) {
    // queue is full, reschedule to try again
    p->cb_ticks = 1;
    if (atomTimerRegister(p) == ATOM_OK)
      return;
    // else something is wrong, just give up
    digitalWriteFast(LED_BUILTIN, HIGH);
  }
  /* can't delete TimerMsg here because we're in interrupt context
    * so add it to the list to be released later
    */
  p->next = p->release_list.load(std::memory_order_relaxed);
  while (!p->release_list.compare_exchange_weak(p->next, p, std::memory_order_relaxed));
}

USB_Host_Port1::TimerMsg::TimerMsg(const usb_msg_t &_msg, uint32_t ms, std::atomic<class TimerMsg *> &r)
  : msg(_msg), release_list(r) {
  cb_func = timer_callback;
  cb_data = this;
  cb_ticks = (ms * SYSTEM_TICKS_PER_SEC + 999) / 1000;
  next = NULL;
  //dprintf("TIMER %p: %lu ms -> %lu ticks\n", this, ms, cb_ticks);
}

USB_Host_Port1::TimerMsg::~TimerMsg() {
  delete next;
}

template<uint8_t usb_int>
void USB_Host_Port1::usb_isr(void) {
  usb_msg_t intmsg = { USB_MSG_INTERRUPT };
  NVIC_DISABLE_IRQ(usb_int);
  atomIntEnter();
  if (atomQueuePut(&g_usbqueue, -1, (uint8_t *)&intmsg) != ATOM_OK)
    digitalWriteFast(LED_BUILTIN, HIGH);
  atomIntExit(0);
}

FLASHMEM void USB_Host_Port1::thread(void) {
  usb_msg_t msgs[50];
  atomQueueCreate(&g_usbqueue, (uint8_t *)msgs, sizeof(msgs[0]), sizeof(msgs) / sizeof(msgs[0]));

  attachInterruptVector(IRQ_USB2, usb_isr<IRQ_USB2>);
  init_pll((REG32_QUAD_t *)&CCM_ANALOG_PLL_USB2);

  // ungate clock
  CCM_CCGR6 |= CCM_CCGR6_USBOH3(CCM_CCGR_ON);
  phy_on((usb_phy_t *)IMXRT_USBPHY2_ADDRESS);

  // configure USB protection IC GPIO (initially off)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40 = 5;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40 = 0x0008;  // slow speed, weak 150 ohm drive
  GPIO8_GDIR |= 1 << 26;
  GPIO8_DR_CLEAR = 1 << 26;

  atomTimerDelay(10 * SYSTEM_TICKS_PER_SEC / 1000);  // 10ms

  NVIC_ENABLE_IRQ(IRQ_USB2);
  usb_process();

  NVIC_DISABLE_IRQ(IRQ_USB2);
  atomQueueDelete(&g_usbqueue);
}

USB_Host_Port1::USB_Host_Port1()
  : USB_Host((usb_ehci_base_t *)&USB2_HCIVERSION), timerRelease(NULL) {}

FLASHMEM void USB_Host_Port1::begin(void) {
  atomThreadCreate(&usb_thread, 64, thread_start, (uint32_t)this, usb_stack, sizeof(usb_stack), 0);
}

// monotonic clock, counts milliseconds
uint32_t USB_Host_Port1::getMillis(void) {
  return millis();
}

DMAMEM ATOM_QUEUE USB_Host_Port1::g_usbqueue;

__attribute__((weak)) int usleep(useconds_t us) {
  const useconds_t us_per_tick = 1000000 / SYSTEM_TICKS_PER_SEC;
  if (atomTimerDelay((us + (us_per_tick - 1)) / us_per_tick) == ATOM_OK)
    return 0;
  return -1;
}

static void sync_callback(int result, ATOM_SEM *sem, int *err) {
  *err = result;
  atomSemPut(sem);
}

// synchronous functions implemented using atomThreads
int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data) {
  ATOM_SEM sem;
  errno = ENOMEM;
  if (atomSemCreate(&sem, 0) == ATOM_OK) {
    int result;
    std::function<void(int)> int_fn(std::bind(&sync_callback, std::placeholders::_1, &sem, &result));
    result = ControlMessage(bmRequestType, bmRequest, wValue, wIndex, wLength, data, int_fn);
    if (result >= 0) {
      if (atomSemGet(&sem, 0) == ATOM_OK) {
        if (result < 0) {
          errno = -result;
        }
      } else {
        result = -1;
        errno = EINTR;
      }
    }
    atomSemDelete(&sem);
    if (result >= 0) return result;
  }
  return -1;
}

int USB_Driver::BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data) {
  ATOM_SEM sem;
  errno = ENOMEM;
  if (atomSemCreate(&sem, 0) == ATOM_OK) {
    int result;
    std::function<void(int)> int_fn(std::bind(&sync_callback, std::placeholders::_1, &sem, &result));
    result = BulkMessage(bEndpoint, dLength, data, int_fn);
    if (result >= 0) {
      if (atomSemGet(&sem, 0) == ATOM_OK) {
        if (result < 0) {
          errno = -result;
        }
      } else {
        result = -1;
        errno = EINTR;
      }
    }
    atomSemDelete(&sem);
    if (result >= 0) return result;
  }
  return -1;
}

int USB_Driver::InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data) {
  ATOM_SEM sem;
  errno = ENOMEM;
  if (atomSemCreate(&sem, 0) == ATOM_OK) {
    int result;
    std::function<void(int)> int_fn(std::bind(&sync_callback, std::placeholders::_1, &sem, &result));
    result = InterruptMessage(bEndpoint, wLength, data, int_fn);
    if (result >= 0) {
      if (atomSemGet(&sem, 0) == ATOM_OK) {
        if (result < 0) {
          errno = -result;
        }
      } else {
        result = -1;
        errno = EINTR;
      }
    }
    atomSemDelete(&sem);
    if (result >= 0) return result;
  }
  return -1;
}

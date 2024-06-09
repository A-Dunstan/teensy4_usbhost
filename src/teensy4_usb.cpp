#include <Arduino.h>
#include <unistd.h>  // usleep
#include "teensy4_usb.h"

bool USBHostBase::getMessage(usb_msg_t &msg) {
  uint8_t s = atomQueueGet(usbqueue, 0, (uint8_t *)&msg);
  if (s != ATOM_OK)
    digitalWriteFast(LED_BUILTIN, HIGH);
  // release used timers now
  TimerMsg *t = std::atomic_exchange_explicit(&timerRelease, NULL, std::memory_order_relaxed);
  delete t;
  return (s == ATOM_OK);
}

bool USBHostBase::putMessage(usb_msg_t &msg) {
  if (atomQueuePut(usbqueue, -1, (uint8_t *)&msg) != ATOM_OK) {
    digitalWriteFast(LED_BUILTIN, HIGH);
    return false;
  }
  return true;
}

bool USBHostBase::timerMsg(usb_msg_t &msg, uint32_t ms) {
  TimerMsg *t = new (std::nothrow) TimerMsg(msg, ms, this);
  if (t != NULL) {
    if (atomTimerRegister(t) == ATOM_OK)
      return true;
    dprintf("Registering timer<%p> failed\n", t);
    digitalWriteFast(LED_BUILTIN, HIGH);
    delete t;
  }
  return false;
}

void USBHostBase::TimerMsg::timer_callback(POINTER cb_data) {
  TimerMsg *p = (TimerMsg *)cb_data;
  if (atomQueuePut(p->host->usbqueue, -1, (uint8_t *)&p->msg) == ATOM_WOULDBLOCK) {
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
  std::atomic<class TimerMsg*> &release_list = p->host->timerRelease;
  p->next = release_list.load(std::memory_order_relaxed);
  while (!release_list.compare_exchange_weak(p->next, p, std::memory_order_relaxed));
}

USBHostBase::TimerMsg::TimerMsg(const usb_msg_t &_msg, uint32_t ms, USBHostBase *h)
  : msg(_msg), host(h) {
  cb_func = timer_callback;
  cb_data = this;
  cb_ticks = (ms * SYSTEM_TICKS_PER_SEC + 999) / 1000;
  next = NULL;
  //dprintf("TIMER %p: %lu ms -> %lu ticks\n", this, ms, cb_ticks);
}

USBHostBase::TimerMsg::~TimerMsg() {
  delete next;
}

FLASHMEM USBHostBase::USBHostBase(ATOM_QUEUE *q, usb_ehci_base_t *usb) : USB_Host(usb), usbqueue(q) {
  timerRelease = NULL;
}

FLASHMEM void USBHostBase::init_pll(REG32_QUAD_t *const PLL) {
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

FLASHMEM void USBHostBase::phy_on(usb_phy_t *const PHY) {
  PHY->CTRL.CLR = USBPHY_CTRL_SFTRST | USBPHY_CTRL_CLKGATE;
  PHY->CTRL.SET = USBPHY_CTRL_ENUTMILEVEL2 | USBPHY_CTRL_ENUTMILEVEL3;
  PHY->TX.CLR = USBPHY_TX_TXCAL45DP(15) | USBPHY_TX_TXCAL45DN(15) | USBPHY_TX_D_CAL(15);
  PHY->TX.SET = USBPHY_TX_TXCAL45DP(6) | USBPHY_TX_TXCAL45DN(6) | USBPHY_TX_D_CAL(12);
  PHY->PWD.REG = 0;
}

#ifdef ARDUINO_TEENSY41
FLASHMEM void TeensyUSBHost2::thread(void) {
  // enable USB protection IC GPIO (initially off)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40 = 5;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40 = 0x0008;  // slow speed, weak 150 ohm drive
  GPIO8_GDIR |= 1 << 26;
  GPIO8_DR_CLEAR = 1 << 26;

  TeensyUSB::thread();
}

void TeensyUSBHost2::port_power(uint8_t port, bool set) {
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
#endif

__attribute__((weak)) int usleep(useconds_t us) {
  const useconds_t us_per_tick = 1000000 / SYSTEM_TICKS_PER_SEC;
  if (atomTimerDelay((us + (us_per_tick - 1)) / us_per_tick) == ATOM_OK)
    return 0;
  return -1;
}

// synchronous functions implemented using atomThreads (semaphores)
/* using a template here gives a better chance of the compiler inlining this into
 * the calling function, which avoids using dynamic memory.
 */
template <class req_fn>
static int sync_message(const req_fn &req) {
  ATOM_SEM sem;
  int result = -1;
  if (atomSemCreate(&sem, 0) == ATOM_OK) {
    int xfer_r;
    // order of operations is tricky here, follow the numbers
    auto fn = [&](int r) {
      // USB Host thread performs this action when transfer is complete
      xfer_r = r;         // 4: actual result of the transfer is stored
      atomSemPut(&sem);   // 5: unblock main thread
    };
    result = req(fn);     // 1: queue async Control/Bulk/InterruptMessage request
    if (result >= 0) {    // 2: result of attempt to queue the transfer is checked
      if (atomSemGet(&sem, 0) == ATOM_OK) {
                          // 3: main thread blocked on semaphore
        result = xfer_r;
        if (result < 0) { // 6: result of the transfer is checked
          errno = -result;
        }
      } else {
        result = -1;
        errno = EINTR;
      }
    }
    atomSemDelete(&sem);
  }
  else errno = ENOMEM;
  return result;
}

int USB_Driver::ControlMessage(uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength, void *data) {
  return sync_message([=](const USBCallback &cb)->int {
    return ControlMessage(bmRequestType, bmRequest, wValue, wIndex, wLength, data, &cb);
  });
}

int USB_Driver::BulkMessage(uint8_t bEndpoint, uint32_t dLength, void *data) {
  return sync_message([=](const USBCallback &cb)->int {
    return BulkMessage(bEndpoint, dLength, data, &cb);
  });
}

int USB_Driver::InterruptMessage(uint8_t bEndpoint, uint16_t wLength, void *data) {
  return sync_message([=](const USBCallback &cb)->int {
    return InterruptMessage(bEndpoint, wLength, data, &cb);
  });
}

int USB_Driver::IsochronousMessage(uint8_t bEndpoint, isolength& Lengths, void *data) {
  return sync_message([=,&Lengths](const USBCallback &cb)->int {
    return IsochronousMessage(bEndpoint, Lengths, data, &cb);
  });
}

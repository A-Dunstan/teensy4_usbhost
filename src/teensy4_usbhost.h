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

#ifndef _TEENSY4_USBHOST_H
#define _TEENSY4_USBHOST_H

// prefer submodule version if it is present
#if defined __has_include && __has_include("TeensyAtomThreads/TeensyAtomThreads.h")
#include "TeensyAtomThreads/TeensyAtomThreads.h"
#else
#include <TeensyAtomThreads.h>
#endif

#include <atomic>
#include "ehci/host.h"
#include "ehci/driver.h"
#include <imxrt.h>

#define USB_STACK_SIZE 1024

class USBHostBase : public USB_Host {
  bool getMessage(usb_msg_t &msg) override;
  bool putMessage(usb_msg_t &msg) override;

  bool timerMsg(usb_msg_t &msg, uint32_t ms) override;

  // monotonic clock, counts milliseconds
  uint32_t getMillis(void) override;

private:
  ATOM_QUEUE &usbqueue;

  class TimerMsg : public ATOM_TIMER {
    static void timer_callback(POINTER cb_data);

    usb_msg_t msg;
    USBHostBase& host;
    class TimerMsg *next;
  public:
    TimerMsg(const usb_msg_t &_msg, uint32_t ms, USBHostBase &h);
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
  USBHostBase(ATOM_QUEUE&, usb_ehci_base_t*);
  static void init_pll(struct REG32_QUAD_t *const PLL);
  static void phy_on(struct usb_phy_t *const PHY);

public:
  void begin(void);
  static bool isUSBThread(const USB_Device*);
};

template <IRQ_NUMBER_t irq, uint32_t phy, uint32_t ehci, uint32_t pll>
class TeensyUSB : public USBHostBase {
  void nextIRQ(void) override;
  void setHostMode(void) override;
  void phySetHighSpeed(uint8_t port, bool on) override;

  static const size_t OFFSET_EHCI_SBUSCFG = 0x90;
  static const size_t OFFSET_EHCI_CAPLENGTH = 0x100;
  static const size_t OFFSET_EHCI_USBMODE = 0x1A8;

  // static so it is usable by the ISR
  static ATOM_QUEUE g_usbqueue;
  static void usb_isr(void);

protected:
  void thread(void) override;

  TeensyUSB() :
  USBHostBase(g_usbqueue, (usb_ehci_base_t*)(ehci+OFFSET_EHCI_CAPLENGTH)) {}
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
  void port_power(uint8_t port, bool set) override;
public:
  TeensyUSBHost2();
#else
public:
  TeensyUSBHost2() = default;
#endif
};

#include "drivers/drivers.h"

#endif // _TEENSY4_USBHOST_H

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

#include "host.h"
#include "portab.h"

#include <cerrno>
#include <cstring>
#include <unistd.h>

#define MK_BE16(a, b) (((a)<<8)|(b))

USB_Host::Enum_Control::Enum_Control() : USB_Control_Endpoint(64, 0, 0, 0, 0) {
  // this is the first async pipe, link to self
  set_link(this);
  capabilities.H = 1;

  in_progress = false;
}

void USB_Host::Enum_Control::set(USB_Hub& h, uint8_t p, uint8_t s) {
  capabilities.hub = h.hub_addr;
  capabilities.port = h.HS_port(p);
  capabilities.speed = s;
  capabilities.C = (s < 2) ? 1:0;
  cache_flush_invalidate(this);

  hub = &h;
  port = p;
  speed = s;
  address_retry = 0;
  h.addref();
}

void USB_Host::Enum_Control::clean(void) {
  if (in_progress == false) return;

  dprintf("Enumeration failed, disabling port\n");
  in_progress = false;
  if (hub && hub->port[port].state != 3) {
    hub->port[port].state = 3;
    hub->port_enable(port, false);
    hub->deref();
  }
  flush();
}

void USB_Host::callback(const usb_control_transfer* t, int result) {
  if (Enum.in_progress == false) return;

  USB_Hub &hub = *Enum.hub;
  uint8_t port = Enum.port;

  uint32_t state = hub.port[port].state;
  if (state < 4) return;

  if (result >= 0) {
    if (t == NULL) {
      dprintf("!!! Enumeration begin !!!\n");
      Enum.Transfer(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR, (USB_DT_DEVICE << 8), 0, sizeof(usb_device_descriptor), NULL, this);
      return;
    }

    uint16_t rt_rq = MK_BE16(t->getbmRequestType(), t->getbmRequest());

    if (rt_rq == MK_BE16(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS) && t->getwValue()!=0) {
      // address has been set, basic enumeration is complete
      hub.port[port].state = 6;
      hub.deref();
      Enum.in_progress = false;
      dprintf("Port %d:%d successfully set address %d\n", hub.hub_addr, port, t->getwValue());
      /* schedule device to begin initializing after set_address timeout
        * (this should also give time for the control endpoint to be activated)
        */
      usb_msg_t msg = {
        .type = USB_MSG_DEVICE_INIT,
        .device = {
          .dev = hub.port[port].device
        }
      };
      timerMsg(msg,20);
      return;
    }

    if (rt_rq == MK_BE16(USB_REQTYPE_DEVICE_GET, USB_REQ_GET_DESCRIPTOR)) {
      const usb_device_descriptor *desc = (const usb_device_descriptor*)t->getBuffer();
      if (result>=8 && desc->bDescriptorType == USB_DT_DEVICE && desc->bLength == sizeof(usb_device_descriptor)) {
        dprintf("Port %d:%d bDeviceClass %02X bDeviceSubClass %02X bDeviceProtocol %02X bMaxPacketSize %d\n", hub.hub_addr, port, desc->bDeviceClass, desc->bDeviceSubClass, desc->bDeviceProtocol, desc->bMaxPacketSize);
        Enum.bMaxPacketSize = desc->bMaxPacketSize;
        /* this particular device doesn't respond properly to the first SET_ADDRESS request (doesn't return an error but doesn't switch addresses)
          * It has a 16-byte packet control endpoint, so VID/PID/bcdDevice will be filled (enumerator uses wMaxPacketSize==64)
          * Workaround: send a request to set address to 0, doesn't matter if it accepts it or not. Also safe for any other devices that end up here.
          */
        if (result>=16 && desc->idVendor==0x0A12 && desc->idProduct==0x0001 && desc->bcdDevice==0x0134) {
          Enum.Transfer(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS, 0, 0, 0, NULL, this);
          return;
        }
      } else {
        Enum.clean();
        return;
      }
    }

    uint8_t address = get_address();
    if (address > 0) {
      USB_Device *d = new(std::nothrow) USB_Device(*this, hub.hub_addr, hub.HS_port(port), Enum.speed, address, Enum.bMaxPacketSize);
      if (d != NULL) {
        hub.port[port].device = d;
        // set the address (possibly for the second time)
        Enum.Transfer(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS, address, 0, 0, NULL, this);
        return;
      }
      release_address(address);
    }
  } else {
    if (t && result==-EPIPE) {
      if (Enum.address_retry == 0) { // first failure - retry at timeout
        usb_control_setup s = {USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS, t->getwValue()};
        if (t->compare(s)) {
          Enum.address_retry = s.wValue;
          return;
        }
      } // second failure - give up
    } else if (result==-EBUSY && Enum.address_retry!=0) {
      // reset timeout
      set_port_timeout(*Enum.hub, Enum.port, 500);
      Enum.Transfer(USB_REQTYPE_DEVICE_SET, USB_REQ_SET_ADDRESS, Enum.address_retry, 0, 0, NULL, this);
      return;
    }
  }

  Enum.clean();
}

uint8_t USB_Host::get_address(void) {
  for (uint8_t i=1; i < 128; i++)
  {
    if (addresses[i>>5] & (1 << (i&31)))
      continue;
    addresses[i>>5] |= 1 << (i&31);
    return i;
  }
  return 0;
}

void USB_Host::release_address(uint8_t i) {
  if (i != 0)
    addresses[i>>5] &= ~(1<<(i&31));
}

bool USB_Host::set_port_timeout(USB_Hub& hub, uint8_t port, uint32_t ms) {
  usb_msg_t msg = {
    .type = USB_MSG_PORT_TIMEOUT,
    .port = {
      .hub = &hub,
      .port = port
    }
  };
  uint32_t start = getMillis();
  if (start==0) start = 1;
  uint32_t oldstart = hub.port[port].timeout_start;
  hub.port[port].timeout_start = start;
  if (oldstart == 0) {
    if (timerMsg(msg, ms) == false)
      return false;
    hub.addref();
  } else {
    dprintf("Adjusted timer start time\n");
  }
  return true;
}

void USB_Host::port_status(USB_Hub& hub, uint8_t port, port_status_t status) {
  if (status.ENABLE == 0) {
    if (hub.port[port].device) {
      dprintf("Device %p disconnected\n", hub.port[port].device);
      hub.port[port].device->disconnect();
      hub.port[port].device = NULL;
    }
    /* if port was active it might have had an overcurrent event or otherwise
      * unintentionally disabled itself without a disconnection, set the state
      * as if it was disconnected to restart enumeration
      */
    if (hub.port[port].state == 6)
      hub.port[port].state = 1;
    // else don't change state, port was disabled deliberately (e.g. enumeration failure)
  }
  if (status.CONNECTION == 0) {
    if (hub.port[port].state >= 3 && hub.port[port].state < 6) {
      Enum.clean();
    }
    hub.port[port].state = 1;
  }
  if (status.POWER == 0) {
    hub.port[port].state = 0;
  }

  switch (hub.port[port].state) {
    case 0: // unpowered
      if (status.POWER) { // it's powered now, continue to stage 1
        hub.port[port].state = 1;
        dprintf("Port %d:%d Power ON\n", hub.hub_addr, port);
      } else { // else stay unpowered for 1 second before turning back on
        set_port_timeout(hub, port, 1000);
        break;
      }
    case 1: // powered, disconnected
      hub.phySetHighSpeed(port, false);
      if (status.CONNECTION) { // something has connected, start 100ms debounce
        dprintf("Port %d:%d starting debounce\n", hub.hub_addr, port);
        hub.port[port].state = 2;
        set_port_timeout(hub, port, 100);
      }
      break;
    case 3: // port has been reset
      if (status.ENABLE) { // begin 10ms recovery
        dprintf("Port %d:%d starting recovery\n", hub.hub_addr, port);
        hub.port[port].state = 4;
        Enum.set(hub, port, status.HIGH_SPEED ? 2 : status.LOW_SPEED);
        if (status.LOW_SPEED==0 && status.HIGH_SPEED==1)
          hub.phySetHighSpeed(port, true);
        set_port_timeout(hub, port, 10);
      }
      break;
  }
}

void USB_Host::port_timeout(usb_msg_t& msg) {
  USB_Hub& hub = *msg.port.hub;
  uint8_t port = msg.port.port;
  uint32_t now = getMillis();
  uint32_t start = hub.port[port].timeout_start;
  uint32_t ticked = now - start;
  hub.port[port].timeout_start = 0;

  switch (hub.port[port].state) {
    case 0: // port is unpowered: 1 second timer before repowering
      if (ticked >= 1000) {
        dprintf("Port %d:%d finished power timeout\n", hub.hub_addr, port);
        hub.port_power(port, true);
        hub.port[port].state = 1;
        return;
      } else {
        ticked = 1000-ticked;
      }
      break;
    case 2: // device is connected, 100ms debounce
      if (ticked >= 100) {
        if (Enum.in_progress == false) {
          // reset the port
          dprintf("Port %d:%d finished debounce, starting reset\n", hub.hub_addr, port);
          hub.port[port].state = 3;
          hub.port_reset(port, true);
          Enum.in_progress = true;
          return;
        } // else try again in 100ms
        dprintf("Port %d:%d finished debounce but Enumerator is busy, delaying...\n", hub.hub_addr, port);
        ticked = 100;
        start = now;
      } else {
        ticked = 100-ticked;
      }
      break;
    case 4: // 10ms recovery
      if (ticked >= 10) {
        dprintf("Port %d:%d finished recovery\n", hub.hub_addr, port);
        hub.port[port].state = 5;
        set_port_timeout(hub, port, 500);
        callback(NULL, 0);
        return;
      }
      ticked = 10-ticked;
      break;
    case 5: // enumeration stages, must complete in < 500ms
      if (ticked >= 500) {
        dprintf("Port %d:%d enumeration timeout\n", hub.hub_addr, port);
        callback(NULL,-EBUSY);
        return;
      }
      ticked = 500-ticked;
      break;
    default:
      return;
  }

  if (timerMsg(msg, ticked) == true) {
    hub.addref();
    // restore original start time
    hub.port[port].timeout_start = start;
  }
}

void USB_Host::update_transfers(void) {
  USB_Endpoint *ep = endpoints;
  while (ep) {
    ep->update();
    ep = ep->host_next;
  }
}

void USB_Host::add_async_queue(USB_Async_Endpoint *ep) {
  ep->set_link(Enum.get_link());
  Enum.set_link(ep);
  ep->host_next = endpoints;
  endpoints = ep;
}

bool USB_Host::remove_async_queue(USB_Async_Endpoint *ep) {
  dprintf("Removing async USB_Endpoint %p\n", ep);

  USB_Async_Endpoint* prev = &Enum;
  while (prev->get_link() != ep) {
    if (prev->get_link() == &Enum) {
      dprintf("Can't remove USB_Async_Endpoint %p, not found in async list\n", ep);
      return false;
    }
    prev = prev->get_link();
  }
  prev->set_link(ep->get_link());

  // remove from active endpoint list
  USB_Endpoint **p = &endpoints;
  while (*p != NULL) {
    if (*p == ep) {
      *p = ep->host_next;
      break;
    }
    p = &(*p)->host_next;
  }
  // put at end of cleanup list
  ep->host_next = NULL;
  p = &async_cleanup;
  while (*p != NULL)
    p = &(*p)->host_next;
  *p = ep;
  // ring doorbell
  EHCI->USBCMD |= USB_USBCMD_IAA;
  return true;
}

void USB_Host::notify_endpoint_removed(USB_Endpoint *ep) {
  usb_msg_t msg = {
    .type = USB_MSG_DEVICE_ENDPOINT_REMOVED,
    .device = {
      .dev = ep->device,
      .endpoint = ep
    }
  };
  putMessage(msg);
}

void USB_Host::unschedule_periodic(void) {
  if (periodic_cleanup == NULL) return;
  USB_Periodic_Endpoint *ep = static_cast<USB_Periodic_Endpoint*>(periodic_cleanup);

  // low/full speed endpoints must be inactive to ensure split transactions aren't interrupted
  if (ep->set_inactive() == true)
    return;

  uint8_t s_mask, c_mask;
  ep->get_masks(s_mask, c_mask);

  uint8_t stime = ep->stime;
  uint8_t ctime = ep->ctime;
  const uint32_t f_interval = (ep->interval > 8) ? (ep->interval>>3) : 1; // uframes -> frames

  for (uint32_t i=ep->offset >> 3; i < PERIODIC_LIST_SIZE; i += f_interval) {
    for (uint8_t j=0; j < 8; j++) {
      if (s_mask & (1<<j)) {
        uframe_bandwidth[i*8+j] -= stime;
        dprintf("bandwidth[%lu:%u] = %u\n", i, j, uframe_bandwidth[i*8+j]);
      }
      if (c_mask & (1<<j)) {
        uframe_bandwidth[i*8+j] -= ctime;
        dprintf("bandwidth[%lu:%u] = %u\n", i, j, uframe_bandwidth[i*8+j]);
      }
    }
  }

  // endpoint has been removed from schedule and the frame index has rolled over
  // so it should be completely safe to remove now
  dprintf("unschedule: safe to remove %p\n", ep);
  periodic_cleanup = ep->host_next;
  notify_endpoint_removed(ep);
}

bool USB_Host::calculate_offset(const USB_Periodic_Endpoint *ep, uint32_t &offset) {
  const uint32_t interval = ep->interval;
  const uint32_t stime = ep->stime;
  const uint32_t ctime = ep->ctime;
  uint8_t s_mask, c_mask;
  ep->get_masks(s_mask, c_mask);

  uint32_t best_bandwidth = 0xFFFFFFFF;
  // for each possible uframe offset, find the worst uframe_bandwidth
  for (uint32_t off=0; off < interval; off++) {
    /* TODO: host must not schedule a start-split for uframe 6+8X
     * so this function needs to know the endpoint speed to
     * know when start-splits are in use...
     */

    // low/full speed can't split across frames
    if ((s_mask << (off&7)) >= 256) {
//      dprintf("offset %u not allowed due to s_mask\n", off);
      continue;
    }
    if ((c_mask << (off&7)) >= 256) {
//      dprintf("offset %u not allowed due to c_mask\n", off);
      continue;
    }
    uint32_t max_bandwidth = 0;
    for (uint32_t i=off; i < PERIODIC_LIST_SIZE*8; i += (interval > 8 ? interval:8)) {
      uint32_t umax = max_bandwidth;
      for (uint8_t j=0; j < 8; j++) {
        if ((s_mask | c_mask) & (1<<j)) {
          if ((i+j) >= PERIODIC_LIST_SIZE*8) {
            // frame rollover not allowed
//            dprintf("offset %u not allowed due to frame rollover\n", i+j);
            umax = 0xFFFFFFFF;
            break;
          }
          uint32_t bw = uframe_bandwidth[i+j];
          if (s_mask & (1<<j))
            bw += stime;
          if (c_mask & (1<<j))
            bw += ctime;
          if (bw > umax) umax = bw;
        }
      }
      if (umax > max_bandwidth) max_bandwidth = umax;
    }
    // remember which uframe offset is the best
    // uses "<=" over "<" to favor last possible offset
    if (max_bandwidth <= best_bandwidth) {
      best_bandwidth = max_bandwidth;
      offset = off;
    }
  }
  // fail if the best found needs more than 80% (234 * 0.8) in any uframe
  if (best_bandwidth > 187) {
    dprintf("calculate_bandwidth for endpoint %p failed: best_bandwidth %lu, offset %lu, interval %lu\n", ep, best_bandwidth, offset, ep->interval);
    return false;
  }
  return true;
}

void USB_Host::add_periodic_queue(USB_Periodic_Endpoint *ep) {
  uint32_t offset;
  if (calculate_offset(ep, offset) == false) {
    // no bandwidth available - tell the device it is inactive
    notify_endpoint_removed(ep);
    return;
  }

  ep->activate(offset);

  const uint32_t interval = ep->interval;
  const uint32_t f_interval = (interval > 8) ? (interval>>3) : 1;

  uint8_t stime = ep->stime;
  uint8_t ctime = ep->ctime;
  uint8_t s_mask, c_mask;
  ep->get_masks(s_mask, c_mask);
  dprintf("Endpoint<%p> interval %lu offset %lu stime %u ctime %u s_mask %02X c_mask %02X\n", ep, interval, offset, stime, ctime, s_mask, c_mask);

  /* use the interval and offset to insert the endpoint into the periodic schedule,
    * and s_mask+c_mask to update uframe_bandwidth
    * (interval and offset are milliseconds/frames)
    */
  for (uint32_t i=offset>>3; i < PERIODIC_LIST_SIZE; i += f_interval) {
    // update uframe_bandwidth
    for (uint32_t j=0; j < 8; j++) {
      uint32_t mask = 1<<j;
      if (mask & s_mask) {
        uframe_bandwidth[i*8+j] += stime;
        dprintf("Updated frame %lu/%lu bandwidth for stime %u: %u\n", i, j, stime, uframe_bandwidth[i*8+j]);
      }
      if (mask & c_mask) {
        uframe_bandwidth[i*8+j] += ctime;
        dprintf("Updated frame %lu/%lu bandwidth for ctime %u: %u\n", i, j, ctime, uframe_bandwidth[i*8+j]);
      }
    }
  }

  // put in host's active list of endpoints
  ep->host_next = endpoints;
  endpoints = ep;
}

bool USB_Host::remove_periodic_queue(USB_Periodic_Endpoint *ep) {
  dprintf("Removing periodic USB_Endpoint %p\n", ep);
  // first remove it from the host's active endpoint list
  USB_Endpoint **b = &endpoints;
  while (*b != ep) {
    if (*b == NULL) return false;
    b = &(*b)->host_next;
  }
  *b = ep->host_next;

  // add to end of cleanup list
  ep->host_next = NULL;
  b = &periodic_cleanup;
  while (*b != NULL) {
    b = &(*b)->host_next;
  }
  *b = ep;

  // begin cleanup if it's not already scheduled
  if ((EHCI->USBINTR & USB_USBINTR_FRE) == 0) {
    unschedule_periodic();
    EHCI->USBINTR |= USB_USBINTR_FRE;
  }
  return true;
}

void USB_Host::port_power(uint8_t port, bool set) {
  dprintf("Port %d:%d Power %s\n", hub_addr, port, set ? "on":"off");
  if (set)
    EHCI->PORTSC[port] |= USB_PORTSC1_PP;
  else
    EHCI->PORTSC[port] &= ~USB_PORTSC1_PP;
}

void USB_Host::port_reset(uint8_t port, bool set) {
  dprintf("Port %d:%d Reset %s\n", hub_addr, port, set ? "on" : "off");
  if (set)
    EHCI->PORTSC[port] = (EHCI->PORTSC[port] | USB_PORTSC1_PR) & ~USB_PORTSC1_PE;
  else
    EHCI->PORTSC[port] &= ~USB_PORTSC1_PR;
}

void USB_Host::port_enable(uint8_t port, bool set) {
  dprintf("Port %d:%d Enable %s\n", hub_addr, port, set ? "on" : "off");
  if (set)
    EHCI->PORTSC[port] |= USB_PORTSC1_PE;
  else
    EHCI->PORTSC[port] &= ~USB_PORTSC1_PE;
}

USB_Host::USB_Host(usb_ehci_base_t* const base) :
USB_Hub(0),
PeriodicScheduler(((usb_ehci_cmd_t*)((uint8_t*)base + base->CAPLENGTH))->FRINDEX),
EHCI((usb_ehci_cmd_t*)((uint8_t*)base + base->CAPLENGTH)),
nPorts(base->HCSPARAMS.N_PORTS)
{
  memset(addresses, 0, sizeof(addresses));
  // reserve address 0
  addresses[0] = 1;
  endpoints = &Enum;
  async_cleanup = NULL;
  periodic_cleanup = NULL;
}

void USB_Host::usb_process(void) {
  dprintf("begin ehci reset (%d root ports)", nPorts);
  EHCI->USBCMD |= USB_USBCMD_RST;
  while (EHCI->USBCMD & USB_USBCMD_RST) {
    dprintf(".");
  }
  dprintf(" done\n");

  memset(uframe_bandwidth, 0, sizeof(uframe_bandwidth));
  EHCI->USBINTR = 0;
  setHostMode();
  dprintf("PERIODIC_LIST_SIZE: %d (%u bytes @ %p)\n", PERIODIC_LIST_SIZE, PERIODIC_LIST_SIZE*4, &periodictable[0]);
  EHCI->PERIODICLISTBASE = &periodictable[0];
  EHCI->FRINDEX = 0;
  EHCI->ASYNCLISTADDR = (uint32_t)static_cast<usb_queue_head_t*>(&Enum);
  dprintf("Enumeration QH: %08lX\n", EHCI->ASYNCLISTADDR);
  uint32_t usbcmdinit = 0;
#if (PERIODIC_LIST_SIZE >= 8 && PERIODIC_LIST_SIZE < 128)
    usbcmdinit |= USB_USBCMD_FS_2 | USB_USBCMD_FS_1((64 - PERIODIC_LIST_SIZE) / 17);
#elif (PERIODIC_LIST_SIZE >= 128 && PERIODIC_LIST_SIZE <= 1024)
    usbcmdinit |= USB_USBCMD_FS_1((1024 - PERIODIC_LIST_SIZE) / 257);
#else
#error "Unsupported PERIODIC_LIST_SIZE value"
#endif
  dprintf("USBCMD (FS): %08lX\n", usbcmdinit);
  // ensure other registers are written before CMD is updated
  mem_sync();
  EHCI->USBCMD = usbcmdinit | USB_USBCMD_ITC(0) | USB_USBCMD_RS | USB_USBCMD_ASP(3) | USB_USBCMD_ASPE | USB_USBCMD_PSE | USB_USBCMD_ASE;

  EHCI->CONFIGFLAG = 1;
  // turn on ports, set their stage to 1
  for (uint8_t i=0; i < nPorts; i++) {
    port_power(i, true);
    port[i].state = 1;
  }
  // wait 20ms for ports to power up
  usleep(20000);

  EHCI->USBINTR |= USB_USBINTR_AAE | USB_USBINTR_SEE | USB_USBINTR_PCE | USB_USBINTR_UEE | USB_USBINTR_UE;

  dprintf("USB: beginning main loop\n");

  while (1) {
    usb_msg_t msg;
    if (getMessage(msg)==false)
      break;

    switch (msg.type) {
      case USB_MSG_INTERRUPT:
        do {
          uint32_t state = EHCI->USBSTS;
          state &= EHCI->USBINTR;
          EHCI->USBSTS = state;
          if (0) {
            dprintf("ISR: %08lX", state);
            if (state & USB_USBSTS_FRI) dprintf(" Frame Rollover");
            if (state & USB_USBSTS_AAI) dprintf(" Async Advance");
            if (state & USB_USBSTS_SEI) dprintf(" System Error");
            if (state & USB_USBSTS_PCI) dprintf(" Port Change");
            if (state & USB_USBSTS_UEI) dprintf(" USB Error");
            if (state & USB_USBSTS_UI) dprintf(" USB Interrupt");
            dprintf("\n");
          }

          if (state & USB_USBSTS_SEI) {
            dprintf("USB SYSTEM ERROR\n");
            while (1);
          }

          if (state & (USB_USBSTS_UI|USB_USBSTS_UEI))
            update_transfers();

          if (state & USB_USBSTS_PCI) {
            // update all ports on root hub
            for (uint8_t i=0; i < nPorts; i++) {
              uint32_t status = EHCI->PORTSC[i];
              EHCI->PORTSC[i] = status | (USB_PORTSC1_OCC|USB_PORTSC1_PEC|USB_PORTSC1_CSC);
              dprintf("Root Port %d status: %08lX\n", i, status);
              port_status_t pstatus = {0};
              if (status & USB_PORTSC1_CCS)
                pstatus.CONNECTION = 1;
              if (status & USB_PORTSC1_PE)
                pstatus.ENABLE = 1;
              if (status & USB_PORTSC1_SUSP)
                pstatus.SUSPEND = 1;
              if (status & USB_PORTSC1_OCA)
                pstatus.OVER_CURRENT = 1;
              if (status & USB_PORTSC1_PR)
                pstatus.RESET = 1;
              if (status & USB_PORTSC1_PP)
                pstatus.POWER = 1;
              if (status & USB_PORTSC1_PSPD(1))
                pstatus.LOW_SPEED = 1;
              else if (status & USB_PORTSC1_PSPD(2))
                pstatus.HIGH_SPEED = 1;
              if (status & USB_PORTSC1_PTC(0xF))
                pstatus.TEST = 1;
              if (status & USB_PORTSC1_PIC(3))
                pstatus.INDICATOR = 1;
              port_status(*this, i, pstatus);
            }
          }

          if (state & USB_USBSTS_AAI) {
            /* cleanup ONE endpoint from the async cleanup list.
            * Multiple endpoints may have been queued for cleanup at the same time
            * meaning the doorbell may be rung multiple times before it gets answered,
            * we need to be sure the async schedule has advanced after every ring.
            */
            USB_Endpoint *p = async_cleanup;
            if (p->host_next) {
              // ring again for the next cleanup
              EHCI->USBCMD |= USB_USBCMD_IAA;
            }
            async_cleanup = p->host_next;
            notify_endpoint_removed(p);
          }

          if (state & USB_USBSTS_FRI) {
            EHCI->USBINTR &= ~USB_USBINTR_FRE;
            unschedule_periodic();
            // re-enable the interrupt if there's more to cleanup
            if (periodic_cleanup != NULL) {
              EHCI->USBINTR |= USB_USBINTR_FRE;
              mem_sync();
            }
          }
        } while(0);
        nextIRQ();
        continue;
      case USB_MSG_ADDRESS_RELEASED:
        release_address(msg.address);
        continue;
      case USB_MSG_PORT_STATUS:
        port_status(*msg.port.hub, msg.port.port, msg.port.status);
        msg.port.hub->deref();
        continue;
      case USB_MSG_PORT_TIMEOUT:
        port_timeout(msg);
        msg.port.hub->deref();
        continue;
      case USB_MSG_ENDPOINT_ACTIVATE:
        msg.endpoint.ep->device = msg.endpoint.device;
        switch (msg.endpoint.ep->ep_type) {
          case USB_ENDPOINT_INTERRUPT:
          case USB_ENDPOINT_ISOCHRONOUS:
            add_periodic_queue(static_cast<USB_Periodic_Endpoint*>(msg.endpoint.ep));
            break;
          case USB_ENDPOINT_BULK:
          case USB_ENDPOINT_CONTROL:
            add_async_queue(static_cast<USB_Async_Endpoint*>(msg.endpoint.ep));
            break;
          default:
            dprintf("Don't know how to activate endpoint %p, type is %d\n", msg.endpoint.ep, msg.endpoint.ep->ep_type);
            notify_endpoint_removed(msg.endpoint.ep);
            break;
        }
        continue;
      case USB_MSG_ENDPOINT_DEACTIVATE:
        switch (msg.endpoint.ep->ep_type) {
          case USB_ENDPOINT_INTERRUPT:
          case USB_ENDPOINT_ISOCHRONOUS:
            if (remove_periodic_queue(static_cast<USB_Periodic_Endpoint*>(msg.endpoint.ep)) == true)
              continue;
            break;
          case USB_ENDPOINT_CONTROL:
          case USB_ENDPOINT_BULK:
            if (remove_async_queue(static_cast<USB_Async_Endpoint*>(msg.endpoint.ep)) == true)
              continue;
            break;
          default:
            dprintf("Don't know how to deactivate endpoint %p, type is %d\n", msg.endpoint.ep, msg.endpoint.ep->ep_type);
        }
        // removal could not be queued, send notification now
        notify_endpoint_removed(msg.endpoint.ep);
        continue;
      case USB_MSG_DEVICE_INIT:
      case USB_MSG_DEVICE_ENDPOINT_REMOVED:
      case USB_MSG_DEVICE_FIND_DRIVER:
      case USB_MSG_DEVICE_CONTROL_TRANSFER:
      case USB_MSG_DEVICE_BULK_TRANSFER:
      case USB_MSG_DEVICE_INTERRUPT_TRANSFER:
      case USB_MSG_DEVICE_ISOCHRONOUS_TRANSFER:
        // forward to device
        msg.device.dev->USBMessage(msg);
        continue;
    }
    break;
  }
  dprintf("USB: shutting down\n");
}

bool USB_Host::activate_endpoint(USB_Endpoint *ep, USB_Device *dev) {
  usb_msg_t msg = {
    .type = USB_MSG_ENDPOINT_ACTIVATE,
    .endpoint = {
      .ep = ep,
      .device = dev,
    }
  };
  return putMessage(msg);
}

bool USB_Host::deactivate_endpoint(USB_Endpoint *ep) {
  usb_msg_t msg = {
    .type = USB_MSG_ENDPOINT_DEACTIVATE,
    .endpoint = {
      .ep = ep
    }
  };
  return putMessage(msg);
}

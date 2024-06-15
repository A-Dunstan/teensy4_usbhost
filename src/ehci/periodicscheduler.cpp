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

#include "periodicscheduler.h"
#include "log.h"
#include "config.h"
#include "portab.h"
#include <cstdlib>

PeriodicScheduler::PeriodicScheduler(volatile uint32_t& frindex) :
FRINDEX(frindex) {
  periodictable = (uint32_t*)aligned_alloc(4096, sizeof(uint32_t)*PERIODIC_LIST_SIZE);

  dprintf("PERIODIC_LIST_SIZE: %d (%p)\n", PERIODIC_LIST_SIZE, periodictable);
  for (int i=0; i < PERIODIC_LIST_SIZE; i++)
    periodictable[i] = 0x80000001;
  cache_flush(periodictable, sizeof(periodictable[0])*PERIODIC_LIST_SIZE, 4096);
}

PeriodicScheduler::~PeriodicScheduler() {
  free(periodictable);
}

uint32_t PeriodicScheduler::current_uframe(void) {
  return FRINDEX % (PERIODIC_LIST_SIZE*8);
}

bool PeriodicScheduler::add_node(uint32_t frame, uint32_t link_to, uint32_t interval) {
  uint32_t *hlink = (uint32_t*)(link_to & ~0x1F);
  uint32_t *h = periodictable+frame;
  uint32_t hnext = *hlink;
  uint32_t next;
  while (next = *h, (next & 1)==0) {
    if (next == link_to) {
      // found node already in this frame, nothing to insert
      return false;
    }
    if (next == hnext) {
      // h points to the same node as n so point h at n
      break;
    }
    // insert in front of queue heads with smaller intervals, otherwise keep traversing
    if (periodicnode_to_interval(next) < interval)
      break;
    // follow horizontal link
    h = (uint32_t*)(next & ~0x1F);
  }

  // insert node into tree
  if (hnext == 1) {
    // update the endpoint's horizontal link only once - all following nodes repeat with the same interval
    *hlink = next;
    cache_flush_invalidate(hlink);
  } else if (next != hnext) {
    dprintf("ERROR: node %p links to %08X instead of %08X! (frame %u)", h, next, hnext, frame);
  }
  cache_invalidate(h);
  *h = link_to;
  cache_flush_invalidate(h);
  return true;
}

bool PeriodicScheduler::remove_node(uint32_t frame, uint32_t link_to, uint32_t hnext) {
  uint32_t *h = periodictable+frame;
  uint32_t next;
  while (next = *h, (next & 1)==0) {
    if (next == hnext) {
      // node was already removed
      break;
    }
    if (next == link_to) {
      // unlink node from this frame
      cache_invalidate(h);
      *h = hnext;
      cache_flush_invalidate(h);
      return true;
    }
    // follow horizontal link
    h = (uint32_t*)(next & ~0x1F);
  }
  return false;
}

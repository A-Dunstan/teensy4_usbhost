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

#ifndef _USB_PERIODICSCHEDULER_H
#define _USB_PERIODICSCHEDULER_H

#include <cstdint>

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

uint32_t periodicnode_to_interval(uint32_t node);

#endif // _USB_PERIODICSCHEDULER_H

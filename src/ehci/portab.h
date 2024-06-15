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

/* This file needs to provide the following functions (note that some
 * have default arguments and can be split into different functions):
 *
 * void mem_sync(void);
 * - waits for all memory operations to complete
 *
 * void cache_sync(void);
 * - waits for all CPU cache flush operations to complete
 *
 * void cache_flush(const void* p, size_t len=1, size_t align=1);
 * - flushes / writes out all CPU cache lines containing bytes between p and p+len.
 * Location p+len is not included.
 * align is a hint for the alignment of p and must be a power of 2.
 *
 * void cache_invalidate(void* p, size_t len=1, size_t align=1);
 * - as above but cache lines are invalidated / discarded
 *
 * void cache_flush_invalidate(const void* p, size_t len=1, size_t align=1);
 * - as above but cache lines are flushed, then invalidated
 *
 * A definition must also be provided for CACHE_LINE_SIZE: the length of a single cache line.
 */

#include "../usb_portab.h"

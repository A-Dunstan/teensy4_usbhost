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

#ifndef _USB_PORTAB_H
#define _USB_PORTAB_H

#include <imxrt.h>

#define CACHE_LINE_SIZE 32

static inline void __attribute__((unused)) mem_sync(void) {
  asm volatile("dmb");
}

static inline void __attribute__((unused)) cache_sync(void) {
  asm volatile("dsb");
}

// single cache-line methods (minimum 32 bytes)

static inline void __attribute__((unused)) cache_flush(const void* p) {
  SCB_CACHE_DCCMVAC = (uint32_t)p;
}

static inline void __attribute__((unused)) cache_invalidate(void* p) {
  SCB_CACHE_DCIMVAC = (uint32_t)p;
  cache_sync();
}

static inline void __attribute__((unused)) cache_flush_invalidate(const void* p) {
  SCB_CACHE_DCCIMVAC = (uint32_t)p;
  cache_sync();
}

// variable length

static void __attribute__((unused)) cache_flush(const void* p, size_t len) {
  if (len==0) return;
  uint32_t src = (uint32_t)p;
  uint32_t end = src + len;
  src &= ~(CACHE_LINE_SIZE-1);
  do {
    SCB_CACHE_DCCMVAC = src;
    src += CACHE_LINE_SIZE;
  } while (src < end);
}

static void __attribute__((unused)) cache_invalidate(void* p, size_t len) {
  if (len==0) return;
  uint32_t src = (uint32_t)p;
  uint32_t end = src + len;
  src &= ~(CACHE_LINE_SIZE-1);
  do {
    SCB_CACHE_DCIMVAC = src;
    src += CACHE_LINE_SIZE;
  } while (src < end);
  cache_sync();
}

static void __attribute__((unused)) cache_flush_invalidate(const void* p, size_t len) {
  if (len==0) return;
  uint32_t src = (uint32_t)p;
  uint32_t end = src + len;
  src &= ~(CACHE_LINE_SIZE-1);
  do {
    SCB_CACHE_DCCIMVAC = src;
    src += CACHE_LINE_SIZE;
  } while (src < end);
  cache_sync();
}

#endif // _USB_PORTAB_H

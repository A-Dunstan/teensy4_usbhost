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

#ifdef USE_CH341
#include "ch341_serial.h"
#endif

#ifdef USE_MASS_STORAGE
#include "mass_storage.h"
#endif

#ifdef USE_MASS_STORAGE_FAT
#include "mass_storage_fat.h"
#endif

#ifdef USE_MOUSE
#include "mouse.h"
#endif

#ifdef USE_RNDIS
#include "rndis.h"
#endif

#include "FL2000.h"

